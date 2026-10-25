// Lidar-lidar manual calibration: the node pairs two point clouds, the browser
// aligns the source cloud to the target by tuning the extrinsic locally.

import {
  eulerFromMatrix,
  matrixFromExtrinsic,
  poseResult,
  rotatePose,
  translatePose,
} from '../../components/extrinsic.js';
import { createLidarLidarView } from '../../components/lidar_lidar_view.js';

const STATE_TEXT = [
  'unknown',
  'ready',
  'calibrating',
  'paused',
  'done (success)',
  'done (failed)',
];
const STATE_CLASS = [
  'state-unknown',
  'state-ready',
  'state-calibrating',
  'state-paused',
  'state-successed',
  'state-failed',
];

export const meta = {
  label: '激光雷达手动标定',
  calibrationType: 'ssct_lidar_lidar_manual_calib',
};

export function mount(root, ctx) {
  root.innerHTML = `
    <section class="panel">
      <section class="meta">
        <div class="field">
          <span class="field-label">calibration type:</span>
          <output data-type>unknown</output>
        </div>
        <div class="field field-pair">
          <span class="field-label">target frame id:</span>
          <output data-frame>unknown</output>
          <span class="field-label">source frame id:</span>
          <output data-child-frame>unknown</output>
        </div>
      </section>

      <section class="projection">
        <figure class="view">
          <div class="viewport lidar-lidar-viewport">
            <canvas class="lidar-lidar-canvas" data-canvas></canvas>
            <aside class="lidar-lidar-sidebar" aria-label="视图设置">
              <span class="sidebar-title">view</span>
              <div class="sidebar-views" data-view-buttons>
                <button type="button" data-view="3d" class="active">3D</button>
                <button type="button" data-view="front">正</button>
                <button type="button" data-view="side">侧</button>
                <button type="button" data-view="top">俯</button>
              </div>
              <section class="sidebar-group">
                <span class="sidebar-title">target</span>
                <label class="sidebar-field">mode:
                  <select data-target-color-mode>
                    <option value="solid">solid</option>
                    <option value="intensity">intensity</option>
                  </select>
                </label>
                <label class="sidebar-field">color:
                  <input type="color" value="#aeb8c2" data-target-color>
                </label>
                <label class="sidebar-field">size:
                  <input type="range" min="1" max="10" step="1" value="2" data-target-size>
                </label>
                <label class="sidebar-field">opacity:
                  <input type="range" min="0" max="1" step="0.05" value="1" data-target-opacity>
                </label>
              </section>
              <section class="sidebar-group">
                <span class="sidebar-title">source</span>
                <label class="sidebar-field">mode:
                  <select data-source-color-mode>
                    <option value="solid">solid</option>
                    <option value="intensity">intensity</option>
                  </select>
                </label>
                <label class="sidebar-field">color:
                  <input type="color" value="#ff5b5b" data-source-color>
                </label>
                <label class="sidebar-field">size:
                  <input type="range" min="1" max="10" step="1" value="2" data-source-size>
                </label>
                <label class="sidebar-field">opacity:
                  <input type="range" min="0" max="1" step="0.05" value="1" data-source-opacity>
                </label>
              </section>
            </aside>
          </div>
        </figure>

        <section class="commands">
          <section class="extrinsic">
            <span class="extrinsic-group">translation (m)</span>
            <output class="extrinsic-value" data-extrinsic-t>—</output>
            <span class="extrinsic-group">rotation rpy (deg)</span>
            <output class="extrinsic-value" data-extrinsic-r>—</output>
          </section>
          <div class="extrinsic-controls">
            <div class="row extrinsic-steps">
              <label class="axis-head field-label">translation step:
                <span class="input-unit">
                  <input type="number" min="0.01" step="0.01" value="0.05" data-translation-step>
                  <span class="unit">m</span>
                </span>
              </label>
              <label class="axis-head field-label">rotation step:
                <span class="input-unit">
                  <input type="number" min="0.1" step="0.1" value="0.5" data-rotation-step>
                  <span class="unit">deg</span>
                </span>
              </label>
              <span class="spacer"></span>
              <button data-reset-extrinsic type="button">reset extrinsic</button>
            </div>
            <div class="axis-buttons" data-translation-buttons></div>
            <div class="axis-buttons" data-rotation-buttons></div>
          </div>
        </section>
      </section>

      <div class="statusline">
        <span class="field-label">calibration status:</span>
        <output class="state-field" data-state>unknown</output>
      </div>

      <figure class="info">
        <figcaption class="field-label">status info:</figcaption>
        <pre data-info></pre>
      </figure>

      <section class="commands">
        <div class="field-label">calibration command:</div>
        <div class="row" data-basic></div>
      </section>
    </section>`;

  const query = (selector) => root.querySelector(selector);
  const canvas = query('[data-canvas]');
  const stateEl = query('[data-state]');
  const typeEl = query('[data-type]');
  const frameEl = query('[data-frame]');
  const childEl = query('[data-child-frame]');
  const translationEl = query('[data-extrinsic-t]');
  const rotationEl = query('[data-extrinsic-r]');
  const infoEl = query('[data-info]');
  const targetColorModeEl = query('[data-target-color-mode]');
  const sourceColorModeEl = query('[data-source-color-mode]');
  const targetColorEl = query('[data-target-color]');
  const sourceColorEl = query('[data-source-color]');
  const targetSizeEl = query('[data-target-size]');
  const sourceSizeEl = query('[data-source-size]');
  const targetOpacityEl = query('[data-target-opacity]');
  const sourceOpacityEl = query('[data-source-opacity]');
  const view = createLidarLidarView(canvas);

  let targetTopic = null;
  let sourceTopic = null;
  let initialPose = null;
  let pose = null;
  const pendingClouds = new Map();
  let lastProgress = -1;
  let lastState = -1;
  let raf = 0;

  function applyOptions() {
    targetColorEl.disabled = targetColorModeEl.value === 'intensity';
    sourceColorEl.disabled = sourceColorModeEl.value === 'intensity';
    view.setOptions({
      targetColorMode: targetColorModeEl.value,
      sourceColorMode: sourceColorModeEl.value,
      targetColor: targetColorEl.value,
      sourceColor: sourceColorEl.value,
      targetSize: Number(targetSizeEl.value),
      sourceSize: Number(sourceSizeEl.value),
      targetOpacity: Number(targetOpacityEl.value),
      sourceOpacity: Number(sourceOpacityEl.value),
    });
  }

  for (const el of [
    targetColorModeEl,
    sourceColorModeEl,
    targetColorEl,
    sourceColorEl,
    targetSizeEl,
    sourceSizeEl,
    targetOpacityEl,
    sourceOpacityEl,
  ]) {
    el.addEventListener('input', applyOptions);
  }
  applyOptions();

  const viewButtons = root.querySelectorAll('[data-view]');
  for (const button of viewButtons) {
    button.addEventListener('click', () => {
      for (const other of viewButtons) other.classList.toggle('active', other === button);
      view.setView(button.dataset.view);
    });
  }

  function routeCloud(data) {
    if (!data?.topic) return;
    if (data.topic === targetTopic) view.setCloud('target', data);
    else if (data.topic === sourceTopic) view.setCloud('source', data);
    else pendingClouds.set(data.topic, data);
  }

  function flushPendingClouds() {
    for (const topic of [targetTopic, sourceTopic]) {
      const data = pendingClouds.get(topic);
      if (!data) continue;
      pendingClouds.delete(topic);
      routeCloud(data);
    }
  }

  function setStatus(status) {
    const state = Number(status.state);
    stateEl.textContent = STATE_TEXT[state] || 'unknown';
    stateEl.className = `state-field ${STATE_CLASS[state] || ''}`;
    typeEl.textContent = status.calibration_type || 'unknown';
    frameEl.textContent = status.frame_id || 'unknown';
    childEl.textContent = status.child_frame_id || 'unknown';
    infoEl.textContent = status.info || '';
    targetTopic = status.sensor_topic || null;
    sourceTopic = status.child_sensor_topic || null;
    flushPendingClouds();
    const progress = Number(status.progress) || 0;
    if (state !== 2) pendingClouds.clear();
    if (state === 2 && progress >= 99 && lastProgress < 99) {
      ctx.requestCloud();
    }
    lastProgress = progress;
    lastState = state;
  }

  function updateExtrinsic() {
    if (!pose) return;
    const m = pose.elements;
    translationEl.textContent =
      [m[12], m[13], m[14]].map((v) => v.toFixed(4)).join(', ');
    rotationEl.textContent =
      eulerFromMatrix(pose).map((v) => v.toFixed(3)).join(', ');
  }

  function draw() {
    updateExtrinsic();
    if (raf) return;
    raf = requestAnimationFrame(() => {
      raf = 0;
      view.draw();
    });
  }

  function translate(dx, dy, dz) {
    if (!pose) return;
    translatePose(pose, dx, dy, dz);
    draw();
  }

  function rotate(axis, degree) {
    if (!pose) return;
    rotatePose(pose, axis, degree);
    draw();
  }

  function addButtons(selector, stepEl, keys, handler) {
    const buttons = query(selector);
    for (const key of keys) {
      for (const sign of [1, -1]) {
        const button = document.createElement('button');
        button.textContent = `${sign > 0 ? '+' : '-'}${key}`;
        button.addEventListener('click', () => {
          const value = Number(stepEl.value);
          if (!Number.isFinite(value) || value <= 0) return;
          handler(key, sign * value);
        });
        buttons.append(button);
      }
    }
  }

  function commandButton(label, className, onClick) {
    const button = document.createElement('button');
    button.textContent = label;
    if (className) button.className = className;
    button.addEventListener('click', onClick);
    query('[data-basic]').append(button);
    return button;
  }

  query('[data-reset-extrinsic]').addEventListener('click', () => {
    if (initialPose) pose = initialPose.clone();
    draw();
  });
  commandButton('start', 'primary', () => ctx.send({ command: 'start' }));
  commandButton('reset', '', () => ctx.send({ command: 'reset' }));
  commandButton('save result', '', () => {
    if (!pose) return;
    if (!confirm('Save calibration result? This overwrites the output file.')) return;
    ctx.result(poseResult(pose));
    ctx.send({ command: 'save' });
  });

  addButtons(
    '[data-translation-buttons]', query('[data-translation-step]'), ['x', 'y', 'z'],
    (key, value) => {
      if (key === 'x') translate(value, 0, 0);
      else if (key === 'y') translate(0, value, 0);
      else translate(0, 0, value);
    },
  );
  addButtons(
    '[data-rotation-buttons]', query('[data-rotation-step]'), ['roll', 'pitch', 'yaw'],
    (key, value) => rotate(key, value),
  );

  return {
    update: setStatus,
    cloud(data) {
      routeCloud(data);
    },
    params(message) {
      const extrinsic = message.extrinsic?.[0];
      if (!extrinsic) return;
      initialPose = matrixFromExtrinsic(extrinsic);
      pose = initialPose.clone();
      view.setPose(pose);
      draw();
    },
    dispose() {
      if (raf) cancelAnimationFrame(raf);
      raf = 0;
    },
  };
}
