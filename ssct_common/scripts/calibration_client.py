#!/usr/bin/python3
# Copyright 2024 Gezp (https://github.com/gezp).
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

import rclpy
import sys
import threading

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QMessageBox,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QComboBox,
    QTextBrowser,
)
from PySide6.QtCore import Signal

from ssct_interfaces.msg import CalibrationCommand, CalibrationStatus


class CalibrationClient(QWidget):
    recv_calibration_status = Signal(CalibrationStatus)

    def __init__(self, parent=None):
        super(CalibrationClient, self).__init__(parent)
        # layout
        self.setup_ui()
        self.recv_calibration_status.connect(self.update_calibration_status)
        # ros node
        self.ros_node = rclpy.create_node("calibration_client")
        self.status_sub = None
        self.cmd_pub = None
        # initialize
        self.connected = False
        self.initialized = False
        self.destory_calibration_status()
        # start
        self.ros_spinner = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
        self.ros_spinner.start()

    def setup_ui(self):
        # layout
        layout = QVBoxLayout()
        # topic layout
        layout_topic = QHBoxLayout()
        self.comboBox = QComboBox()
        self.comboBox.setMinimumWidth(500)
        layout_topic.addWidget(self.comboBox)
        self.btn_refresh = QPushButton("refresh")
        self.btn_refresh.clicked.connect(self.btn_refresh_callback)
        layout_topic.addWidget(self.btn_refresh)
        layout_topic.addStretch()
        layout_topic.addSpacing(50)
        self.btn_connect = QPushButton("connect")
        self.btn_connect.clicked.connect(self.btn_connect_callback)
        layout_topic.addWidget(self.btn_connect)
        layout.addLayout(layout_topic)
        # calibration type layout
        layout_calibraion_type = QHBoxLayout()
        layout_calibraion_type.addWidget(QLabel("calibration type:"))
        self.label_calibraion_type = QLineEdit("unknown")
        self.label_calibraion_type.setReadOnly(True)
        layout_calibraion_type.addWidget(self.label_calibraion_type)
        layout_calibraion_type.addStretch()
        layout.addLayout(layout_calibraion_type)
        # frame id layout
        layout_frame_id = QHBoxLayout()
        layout_frame_id.addWidget(QLabel("frame id:"))
        self.label_frame_id = QLineEdit("unknown")
        self.label_frame_id.setReadOnly(True)
        layout_frame_id.addWidget(self.label_frame_id)
        layout_frame_id.addSpacing(50)
        layout_frame_id.addWidget(QLabel("child frame id:"))
        self.label_child_frame_id = QLineEdit("unknown")
        self.label_child_frame_id.setReadOnly(True)
        layout_frame_id.addWidget(self.label_child_frame_id)
        layout_frame_id.addStretch()
        layout.addLayout(layout_frame_id)
        # status layout
        layout_calibration_status = QHBoxLayout()
        layout_calibration_status.addWidget(QLabel("calibration status:"))
        self.label_calibration_status = QLineEdit("unknown")
        self.label_calibration_status.setReadOnly(True)
        layout_calibration_status.addWidget(self.label_calibration_status)
        layout_calibration_status.addStretch()
        layout.addLayout(layout_calibration_status)
        # status info browser
        self.status_info_browser = QTextBrowser()
        layout.addWidget(self.status_info_browser)
        # commnad layout
        layout.addWidget(QLabel("calibration command:"))
        self.btn_commands = []
        btn_command_start = QPushButton("start")
        btn_command_start.clicked.connect(
            lambda: self.send_command(CalibrationCommand.START)
        )
        self.btn_commands.append(btn_command_start)
        btn_command_reset = QPushButton("reset")
        btn_command_reset.clicked.connect(
            lambda: self.send_command(CalibrationCommand.RESET)
        )
        self.btn_commands.append(btn_command_reset)
        btn_command_save_result = QPushButton("save result")
        btn_command_save_result.clicked.connect(
            lambda: self.send_command(CalibrationCommand.SAVE)
        )
        self.btn_commands.append(btn_command_save_result)
        layout_command = QHBoxLayout()
        for btn in self.btn_commands:
            layout_command.addWidget(btn)
        layout_command.addStretch()
        layout.addLayout(layout_command)
        self.setLayout(layout)

    def btn_refresh_callback(self):
        topics = self.ros_node.get_topic_names_and_types()
        valid_topics = []
        for data in topics:
            if data[1][0] == "ssct_interfaces/msg/CalibrationStatus":
                valid_topics.append(data[0])
        self.comboBox.clear()
        self.comboBox.addItems(valid_topics)

    def btn_connect_callback(self):
        if self.connected:
            self.disconnect_status()
        else:
            self.connect_status()

    def connect_status(self):
        if self.comboBox.currentIndex() == -1:
            QMessageBox.critical(
                self, "Error", "no calibration status topics!", QMessageBox.Ok
            )
            return
        self.comboBox.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.btn_connect.setText("disconnect")
        self.status_sub = self.ros_node.create_subscription(
            CalibrationStatus,
            self.comboBox.currentText(),
            self.calibration_status_callback,
            10,
        )
        self.connected = True

    def disconnect_status(self):
        self.comboBox.setEnabled(True)
        self.btn_refresh.setEnabled(True)
        self.btn_connect.setText("connect")
        if self.status_sub:
            self.ros_node.destroy_subscription(self.status_sub)
        self.status_sub = None
        self.destory_calibration_status()
        self.connected = False

    def calibration_status_callback(self, msg: CalibrationStatus):
        self.recv_calibration_status.emit(msg)

    def update_calibration_status(self, msg: CalibrationStatus):
        self.update_calibration_state(msg)
        self.status_info_browser.setText(msg.info)
        if not self.initialized:
            self.initialize_calibration_status(msg)

    def initialize_calibration_status(self, msg: CalibrationStatus):
        self.label_calibraion_type.setText(msg.calibration_type)
        self.label_frame_id.setText(msg.frame_id)
        self.label_child_frame_id.setText(msg.child_frame_id)
        self.cmd_pub = self.ros_node.create_publisher(
            CalibrationCommand, msg.command_topic, 10
        )
        for btn in self.btn_commands:
            btn.setEnabled(True)
        self.initialized = True

    def destory_calibration_status(self):
        if self.cmd_pub:
            self.ros_node.destroy_publisher(self.cmd_pub)
        self.cmd_pub = None
        for btn in self.btn_commands:
            btn.setEnabled(False)
        self.initialized = False

    def send_command(self, cmd):
        if self.cmd_pub is None:
            return
        msg = CalibrationCommand()
        msg.command = cmd
        self.cmd_pub.publish(msg)

    def update_calibration_state(self, msg: CalibrationStatus):
        if msg.state == CalibrationStatus.UNKNOWN:
            self.label_calibration_status.setText("unknown")
        elif msg.state == CalibrationStatus.READY:
            self.label_calibration_status.setText("ready")
        elif msg.state == CalibrationStatus.CALIBRATING:
            self.label_calibration_status.setText("calibrating")
        elif msg.state == CalibrationStatus.SUCCESSED:
            self.label_calibration_status.setText("done[successed]")
        elif msg.state == CalibrationStatus.FAILED:
            self.label_calibration_status.setText("done[failed]")
        else:
            self.label_calibration_status.setText("undefined")


if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    window = CalibrationClient()
    window.show()
    app.exec()
    rclpy.shutdown()
