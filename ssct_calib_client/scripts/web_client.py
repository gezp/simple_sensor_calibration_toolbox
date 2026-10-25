#!/usr/bin/env python3
# Copyright 2026 Gezp (https://github.com/gezp).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Web calibration client.

Serves a browser UI and bridges it to the existing calibration topics over a
single WebSocket. Only CalibrationStatus and CalibrationCommand are used, plus
the debug image and point cloud topics advertised by the status message.
"""

import asyncio
import contextlib
import json
import os
import struct
import threading
from urllib.parse import urlsplit

import cv2
import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from ssct_interfaces.msg import (
    CalibrationCommand,
    CalibrationParams,
    CalibrationStatus,
    ExtrinsicParam,
)

STATUS_TYPE = 'ssct_interfaces/msg/CalibrationStatus'
IMAGE_TYPE = 'sensor_msgs/msg/Image'
CLOUD_TYPE = 'sensor_msgs/msg/PointCloud2'

COMMANDS = {
    'reset': CalibrationCommand.RESET,
    'start': CalibrationCommand.START,
    'pause': CalibrationCommand.PAUSE,
    'save': CalibrationCommand.SAVE,
}


def web_root():
    try:
        return os.path.join(get_package_share_directory('ssct_calib_client'), 'webui')
    except Exception:
        # running straight from the source tree
        return os.path.join(os.path.dirname(os.path.dirname(__file__)), 'webui')


def status_dict(msg):
    return {
        'type': 'status',
        'state': int(msg.state),
        'progress': int(msg.progress),
        'info': msg.info,
        'frame_id': msg.frame_id,
        'sensor_topic': msg.sensor_topic,
        'child_frame_id': msg.child_frame_id,
        'child_sensor_topic': msg.child_sensor_topic,
        'calibration_type': msg.calibration_type,
        'stamp': {'sec': msg.timestamp.sec, 'nanosec': msg.timestamp.nanosec},
    }


MODES = [
    {'id': 'generic_calib', 'url': '/static/modes/generic_calib/index.js'},
    {'id': 'camera_intrinsic_calib', 'url': '/static/modes/camera_intrinsic_calib/index.js'},
    {'id': 'lidar_cam_manual_calib', 'url': '/static/modes/lidar_cam_manual_calib/index.js'},
    {'id': 'lidar_lidar_manual_calib', 'url': '/static/modes/lidar_lidar_manual_calib/index.js'},
]

PARAMS_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def status_prefix(status_topic):
    suffix = '/status'
    if status_topic and status_topic.endswith(suffix):
        return status_topic[:-len(suffix)]
    return None


def params_dict(msg):
    return {
        'type': 'params',
        'camera': [
            {
                'frame_id': p.frame_id,
                'width': int(p.width),
                'height': int(p.height),
                'model': p.type,
                'intrinsics': [float(v) for v in p.intrinsics],
                'distortion_coeffs': [float(v) for v in p.distortion_coeffs],
            }
            for p in msg.camera_intrinsic_params
        ],
        'extrinsic': [
            {
                'frame_id': e.frame_id,
                'child_frame_id': e.child_frame_id,
                'translation': [
                    e.transform.translation.x,
                    e.transform.translation.y,
                    e.transform.translation.z,
                ],
                'rotation': [
                    e.transform.rotation.x,
                    e.transform.rotation.y,
                    e.transform.rotation.z,
                    e.transform.rotation.w,
                ],
            }
            for e in msg.extrinsic_params
        ],
    }


def encode_cloud(msg, max_points, topic=None):
    """Pack a PointCloud2 into [uint32 header_len][json header][float32 data].

    Data is xyz, or xyzi when the cloud has an intensity field.
    """
    fields = {field.name for field in msg.fields}
    names = ('x', 'y', 'z', 'intensity') if 'intensity' in fields else ('x', 'y', 'z')
    try:
        data = point_cloud2.read_points_numpy(
            msg, field_names=names, skip_nans=True
        )
    except Exception:  # noqa: BLE001
        return None
    if data.size == 0:
        return None
    if data.dtype.names:
        data = np.column_stack([data[name] for name in data.dtype.names])
    data = np.asarray(data, dtype='<f4').reshape(-1, len(names))
    limit = max(1, int(max_points))
    if len(data) > limit:
        idx = np.linspace(0, len(data) - 1, limit).astype(np.int64)
        data = data[idx]
    header = json.dumps(
        {
            'stream': 'cloud',
            'count': int(len(data)),
            'stride': len(names),
            'dtype': 'f32',
            'topic': topic,
        }
    ).encode()
    return struct.pack('<I', len(header)) + header + data.tobytes()


class Bridge:
    """Discovers status topics and caches their latest message."""

    def __init__(self, node, loop):
        self.node = node
        self.loop = loop
        self.latest = {}
        self.version = {}
        self.types = {}
        self.subs = {}
        self.cv_bridge = CvBridge()
        self.timer = node.create_timer(1.0, self.discover)
        self.discover()

    def discover(self):
        self.types = dict(self.node.get_topic_names_and_types())
        for topic, kinds in self.types.items():
            if STATUS_TYPE in kinds and topic not in self.subs:
                self.subs[topic] = self.node.create_subscription(
                    CalibrationStatus,
                    topic,
                    lambda msg, t=topic: self.on_status(t, msg),
                    10,
                )
        for topic in list(self.subs):
            if STATUS_TYPE not in self.types.get(topic, []):
                self.node.destroy_subscription(self.subs.pop(topic))
                self.latest.pop(topic, None)
                self.version.pop(topic, None)

    def on_status(self, topic, msg):
        self.latest[topic] = msg
        self.version[topic] = self.version.get(topic, 0) + 1

    def topics(self):
        items = []
        for topic in list(self.subs):
            item = {'topic': topic}
            msg = self.latest.get(topic)
            if msg is not None:
                item.update(
                    calibration_type=msg.calibration_type,
                    frame_id=msg.frame_id,
                    child_frame_id=msg.child_frame_id,
                )
            items.append(item)
        return sorted(items, key=lambda item: item['topic'])

    def cloud_topics(self, msg):
        result = []
        for topic in (msg.sensor_topic, msg.child_sensor_topic):
            if topic and topic not in result and CLOUD_TYPE in self.types.get(topic, []):
                result.append(topic)
        return result


class Session:
    """Per browser connection state."""

    def __init__(self, bridge, ws, settings):
        self.bridge = bridge
        self.ws = ws
        self.loop = bridge.loop
        self.settings = settings
        self.event = asyncio.Event()
        self.ctrl = asyncio.Queue()
        self.selected = None
        self.status_version = -1
        self.meta_key = None
        self.image_topic = None
        self.image_sub = None
        self.command_topic = None
        self.cmd_pub = None
        self.params_sub = None
        self.result_pub = None
        self.status_msg = None
        self.cloud_subs = {}
        self.cloud_msgs = {}
        self.want_clouds = set()
        self.image = None
        self.generation = 0

    # ---- ros callbacks (spinner thread) ----
    def on_image(self, generation, msg):
        try:
            img = self.bridge.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.bridge.node.get_logger().warning(f'failed to convert debug image: {exc}')
            return
        height, width = img.shape[:2]
        max_width = int(self.settings['max_image_width'])
        if max_width > 0 and width > max_width:
            scale = max_width / width
            img = cv2.resize(
                img, (max_width, max(1, int(height * scale))), interpolation=cv2.INTER_AREA
            )
        quality = int(self.settings['jpeg_quality'])
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            data = buf.tobytes()
            self.loop.call_soon_threadsafe(self.offer_image, generation, data)

    def on_cloud(self, generation, topic, msg):
        if generation == self.generation and topic in self.cloud_subs:
            self.cloud_msgs[topic] = msg

    def on_params(self, msg):
        self.send_ctrl(params_dict(msg))
        self.event.set()

    def offer_image(self, generation, data):
        if generation != self.generation:
            return
        self.image = data
        self.event.set()

    # ---- subscriptions ----
    def close_subs(self):
        self.generation += 1
        if self.image_sub is not None:
            self.bridge.node.destroy_subscription(self.image_sub)
        for sub in self.cloud_subs.values():
            self.bridge.node.destroy_subscription(sub)
        if self.cmd_pub is not None:
            self.bridge.node.destroy_publisher(self.cmd_pub)
        if self.params_sub is not None:
            self.bridge.node.destroy_subscription(self.params_sub)
        if self.result_pub is not None:
            self.bridge.node.destroy_publisher(self.result_pub)
        self.image_sub = None
        self.cloud_subs = {}
        self.cmd_pub = None
        self.params_sub = None
        self.result_pub = None
        self.status_msg = None
        self.image_topic = None
        self.command_topic = None
        self.cloud_msgs = {}
        self.meta_key = None
        self.image = None
        self.want_clouds = set()

    def apply_meta(self, msg):
        types = self.bridge.types
        generation = self.generation
        self.status_msg = msg
        prefix = status_prefix(self.selected)
        debug_topic = prefix + '/debug_image' if prefix else None
        # Nodes that publish a debug image keep it; other nodes advertise the
        # image to display through sensor_topic.
        if debug_topic and IMAGE_TYPE in types.get(debug_topic, []):
            topic = debug_topic
        else:
            topic = msg.sensor_topic or None
        if topic != self.image_topic:
            if self.image_sub is not None:
                self.bridge.node.destroy_subscription(self.image_sub)
                self.image_sub = None
            self.image_topic = topic
        if self.image_sub is None and topic and IMAGE_TYPE in types.get(topic, []):
            self.image_sub = self.bridge.node.create_subscription(
                Image, topic, lambda image, g=generation: self.on_image(g, image), 10
            )
        if msg.command_topic != self.command_topic:
            if self.cmd_pub is not None:
                self.bridge.node.destroy_publisher(self.cmd_pub)
                self.cmd_pub = None
            self.command_topic = msg.command_topic
            if self.command_topic:
                self.cmd_pub = self.bridge.node.create_publisher(
                    CalibrationCommand, self.command_topic, 10
                )
        prefix = status_prefix(self.selected)
        if prefix and self.params_sub is None:
            self.params_sub = self.bridge.node.create_subscription(
                CalibrationParams,
                prefix + '/initial_params',
                self.on_params,
                PARAMS_QOS,
            )
        if prefix and self.result_pub is None:
            self.result_pub = self.bridge.node.create_publisher(
                CalibrationParams, prefix + '/result_params', 10
            )
        wanted = set(self.bridge.cloud_topics(msg))
        for topic in list(self.cloud_subs):
            if topic not in wanted:
                self.bridge.node.destroy_subscription(self.cloud_subs.pop(topic))
        for topic in wanted:
            if topic not in self.cloud_subs:
                self.cloud_subs[topic] = self.bridge.node.create_subscription(
                    PointCloud2,
                    topic,
                    lambda cloud, t=topic, g=generation: self.on_cloud(g, t, cloud),
                    10,
                )
        key = (
            self.image_topic,
            self.image_sub is not None,
            self.command_topic,
            tuple(sorted(self.cloud_subs)),
        )
        if key != self.meta_key:
            self.meta_key = key
            self.send_ctrl({
                'type': 'meta',
                'image': self.image_sub is not None,
                'clouds': list(self.cloud_subs),
            })

    # ---- websocket ----
    def send_ctrl(self, payload):
        self.ctrl.put_nowait(payload)

    async def _send_loop(self):
        while True:
            try:
                await asyncio.wait_for(self.event.wait(), 0.1)
            except asyncio.TimeoutError:
                pass
            self.event.clear()

            while not self.ctrl.empty():
                await self.ws.send_str(json.dumps(self.ctrl.get_nowait()))

            if self.selected is not None:
                msg = self.bridge.latest.get(self.selected)
                if msg is not None:
                    self.apply_meta(msg)
                    version = self.bridge.version.get(self.selected, -1)
                    if version != self.status_version:
                        self.status_version = version
                        payload = status_dict(msg)
                        payload['topic'] = self.selected
                        await self.ws.send_str(json.dumps(payload))

            for topic in list(self.want_clouds):
                msg = self.cloud_msgs.pop(topic, None)
                if msg is None:
                    continue
                self.want_clouds.discard(topic)
                data = await self.loop.run_in_executor(
                    None,
                    encode_cloud,
                    msg,
                    self.settings['cloud_max_points'],
                    topic,
                )
                if data is not None:
                    await self.ws.send_bytes(data)

            if self.image is not None:
                data = self.image
                self.image = None
                header = json.dumps(
                    {'stream': 'image', 'topic': self.selected}
                ).encode()
                await self.ws.send_bytes(struct.pack('<I', len(header)) + header + data)

    async def handle(self, request):
        kind = request.get('type')
        if kind == 'list':
            self.send_ctrl({'type': 'topics', 'items': self.bridge.topics()})
            self.event.set()
        elif kind == 'select':
            self.close_subs()
            self.selected = request.get('topic')
            self.status_version = -1
            self.event.set()
        elif kind == 'command':
            self.publish_command(request)
        elif kind == 'result':
            self.publish_result(request)
            self.event.set()
        elif kind == 'cloud':
            self.want_clouds = set(self.cloud_subs)
            self.event.set()

    def publish_result(self, request):
        if self.result_pub is None or self.status_msg is None:
            self.send_ctrl({'type': 'error', 'message': 'not connected to a calibration node'})
            self.event.set()
            return
        translation = request.get('translation')
        rotation = request.get('rotation')
        if (
            not isinstance(translation, list)
            or not isinstance(rotation, list)
            or len(translation) != 3
            or len(rotation) != 4
        ):
            self.send_ctrl({'type': 'error', 'message': 'invalid calibration result'})
            self.event.set()
            return
        msg = CalibrationParams()
        msg.timestamp = self.bridge.node.get_clock().now().to_msg()
        extrinsic = ExtrinsicParam()
        extrinsic.frame_id = self.status_msg.frame_id
        extrinsic.child_frame_id = self.status_msg.child_frame_id
        extrinsic.transform.translation.x = float(translation[0])
        extrinsic.transform.translation.y = float(translation[1])
        extrinsic.transform.translation.z = float(translation[2])
        extrinsic.transform.rotation.x = float(rotation[0])
        extrinsic.transform.rotation.y = float(rotation[1])
        extrinsic.transform.rotation.z = float(rotation[2])
        extrinsic.transform.rotation.w = float(rotation[3])
        msg.extrinsic_params = [extrinsic]
        self.result_pub.publish(msg)

    def publish_command(self, request):
        if self.cmd_pub is None:
            self.send_ctrl({'type': 'error', 'message': 'not connected to a calibration node'})
            self.event.set()
            return
        msg = CalibrationCommand()
        msg.timestamp = self.ros_node.get_clock().now().to_msg()
        name = request.get('command')
        if name in COMMANDS:
            msg.command = COMMANDS[name]
        else:
            self.send_ctrl({'type': 'error', 'message': f'unknown command: {name}'})
            self.event.set()
            return
        if name == 'save':
            # result_params and command are on different topics; give the node
            # a small window to receive the result first.
            self.loop.call_later(0.03, self.cmd_pub.publish, msg)
        else:
            if name in ('start', 'reset'):
                self.cloud_msgs.clear()
                self.want_clouds.clear()
            self.cmd_pub.publish(msg)


async def index(request):
    return web.FileResponse(os.path.join(request.app['web_root'], 'index.html'))


async def no_cache(request, response):
    # The webui is served from the package share directory and changes often
    # during development; always revalidate instead of trusting the browser cache.
    if request.path == '/' or request.path.startswith('/static/'):
        response.headers['Cache-Control'] = 'no-store'


def origin_allowed(request):
    origin = request.headers.get('Origin')
    if not origin:
        return True
    return urlsplit(origin).netloc.lower() == (request.host or '').lower()


async def websocket_handler(request):
    if not origin_allowed(request):
        raise web.HTTPForbidden()
    ws = web.WebSocketResponse(heartbeat=30)
    await ws.prepare(request)
    session = Session(request.app['bridge'], ws, request.app['settings'])
    task = asyncio.create_task(session._send_loop())
    await ws.send_str(
        json.dumps(
            {
                'type': 'config',
                'mode': request.app['settings']['mode'],
                'modes': MODES,
            }
        )
    )
    try:
        async for message in ws:
            if message.type != WSMsgType.TEXT:
                continue
            try:
                await session.handle(json.loads(message.data))
            except (ValueError, TypeError) as exc:
                await ws.send_str(json.dumps({'type': 'error', 'message': str(exc)}))
    finally:
        task.cancel()
        with contextlib.suppress(BaseException):
            await task
        session.close_subs()
    return ws


def create_app(bridge, settings):
    root = web_root()
    app = web.Application()
    app['bridge'] = bridge
    app['settings'] = settings
    app['web_root'] = root
    app.router.add_get('/', index)
    app.router.add_get('/ws', websocket_handler)
    app.on_response_prepare.append(no_cache)
    # --symlink-install leaves the webui files as symlinks into the source tree
    app.router.add_static('/static', root, follow_symlinks=True)
    return app


def read_settings(node):
    return {
        'host': node.declare_parameter('host', '0.0.0.0').value,
        'port': node.declare_parameter('port', 8080).value,
        'mode': node.declare_parameter('mode', 'generic_calib').value,
        'jpeg_quality': node.declare_parameter('jpeg_quality', 80).value,
        'max_image_width': node.declare_parameter('max_image_width', 1280).value,
        'cloud_max_points': node.declare_parameter('cloud_max_points', 20000).value,
    }


def main():
    rclpy.init()
    node = Node('ssct_calib_client')
    settings = read_settings(node)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    bridge = Bridge(node, loop)
    runner = web.AppRunner(create_app(bridge, settings))
    loop.run_until_complete(runner.setup())
    loop.run_until_complete(
        web.TCPSite(runner, settings['host'], int(settings['port'])).start()
    )
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    node.get_logger().info(
        f"calibration client on http://{settings['host']}:{settings['port']} "
        f"(mode: {settings['mode']})"
    )
    if settings['host'] not in ('127.0.0.1', 'localhost'):
        node.get_logger().warning('web client has no authentication, use it on a trusted LAN only')
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(runner.cleanup())
        rclpy.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
