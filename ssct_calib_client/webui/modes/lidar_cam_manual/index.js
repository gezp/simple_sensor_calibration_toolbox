// Lidar-camera manual calibration: the node pairs and undistorts, the browser
// projects and tunes the extrinsic locally.

import { Euler, Matrix4, Quaternion } from 'https://unpkg.com/three@0.170.0/build/three.module.min.js';

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

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
  label: '激光相机手动标定',
  calibrationType: 'ssct_lidar_cam_manual_calib',
};

function matrixFromExtrinsic(extrinsic) {
  const quaternion = new Quaternion(...extrinsic.rotation).normalize();
  const matrix = new Matrix4().makeRotationFromQuaternion(quaternion);
  return matrix.setPosition(...extrinsic.translation);
}

// ZYX Euler angles in degrees. The pose itself stays as a matrix/quaternion.
function eulerFromMatrix(matrix) {
  const euler = new Euler().setFromRotationMatrix(matrix, 'ZYX');
  return [euler.x, euler.y, euler.z].map((v) => v * RAD_TO_DEG);
}

function fthetaThetaD(theta, coeffs) {
  const t2 = theta * theta;
  const k1 = coeffs[0] || 0;
  const k2 = coeffs[1] || 0;
  const k3 = coeffs[2] || 0;
  const k4 = coeffs[3] || 0;
  return theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))));
}

function createProjector(camera) {
  const k = camera.intrinsics;
  const d = camera.distortion_coeffs;
  if (camera.model === 'ftheta') {
    return (x, y, z) => {
      const f = k[0];
      const cx = k[1];
      const cy = k[2];
      const r = Math.hypot(x, y);
      if (r < 1e-9) return [cx, cy];
      const rho = f * fthetaThetaD(Math.atan2(r, z), d);
      return [cx + (rho * x) / r, cy + (rho * y) / r];
    };
  }
  const fx = k[0];
  const fy = k[1];
  const cx = k[2];
  const cy = k[3];
  return (x, y, z) => [fx * x / z + cx, fy * y / z + cy];
}

export function mount(root, ctx) {
  root.innerHTML = `
    <section class="panel">
      <section class="meta">
        <div class="field">
          <span class="field-label">calibration type:</span>
          <output data-type>unknown</output>
        </div>
        <div class="field field-pair">
          <span class="field-label">frame id:</span>
          <output data-frame>unknown</output>
          <span class="field-label">child frame id:</span>
          <output data-child-frame>unknown</output>
        </div>
        <div class="field">
          <span class="field-label">camera model:</span>
          <output data-model>waiting for params</output>
        </div>
      </section>

      <section class="projection">
        <section class="display-options">
          <label class="field-label">point color:
            <select data-color-mode>
              <option value="depth">depth</option>
              <option value="intensity">intensity</option>
            </select>
          </label>
          <label class="field-label">point size:
            <input type="range" min="1" max="10" step="1" value="2" data-point-size>
          </label>
          <label class="field-label">point opacity:
            <input type="range" min="0" max="1" step="0.05" value="1" data-opacity>
          </label>
        </section>

        <figure class="view">
          <div class="viewport lidar-cam-viewport">
            <canvas class="lidar-cam-canvas" data-canvas></canvas>
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
  const g = canvas.getContext('2d');
  const stateEl = query('[data-state]');
  const typeEl = query('[data-type]');
  const frameEl = query('[data-frame]');
  const childEl = query('[data-child-frame]');
  const modelEl = query('[data-model]');
  const translationEl = query('[data-extrinsic-t]');
  const rotationEl = query('[data-extrinsic-r]');
  const infoEl = query('[data-info]');
  const sizeEl = query('[data-point-size]');
  const colorEl = query('[data-color-mode]');
  const opacityEl = query('[data-opacity]');

  let camera = null;
  let initialPose = null;
  let pose = null;
  let imageEl = null;
  let imageUrl = null;
  let points = null;
  let pointStride = 3;
  let intensityMin = 0;
  let intensityMax = 0;
  let projector = null;
  let lastProgress = -1;
  let lastState = -1;
  let imageFresh = false;
  let cloudPending = false;
  let raf = 0;

  const palette = Array.from({ length: 16 }, (_, i) => {
    const t = i / 15;
    return `hsl(${210 - 170 * t} 90% ${45 + 20 * t}%)`;
  });
  const intensityPalette = Array.from(
    { length: 16 },
    (_, i) => `hsl(0 0% ${35 + 60 * (i / 15)}%)`,
  );

  for (const el of [sizeEl, colorEl, opacityEl]) {
    el.addEventListener('input', draw);
  }

  function setStatus(status) {
    const state = Number(status.state);
    stateEl.textContent = STATE_TEXT[state] || 'unknown';
    stateEl.className = `state-field ${STATE_CLASS[state] || ''}`;
    typeEl.textContent = status.calibration_type || 'unknown';
    frameEl.textContent = status.frame_id || 'unknown';
    childEl.textContent = status.child_frame_id || 'unknown';
    infoEl.textContent = status.info || '';
    const progress = Number(status.progress) || 0;
    if (state !== 2) cloudPending = false;
    if (state === 2 && lastState !== 2) imageFresh = false;
    if (state === 2 && progress >= 99 && lastProgress < 99) {
      // Wait until the paired image is decoded, so points never appear first.
      cloudPending = true;
      if (imageFresh) {
        cloudPending = false;
        ctx.requestCloud();
      }
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
      render();
    });
  }

  function render() {
    if (!camera) return;
    const width = camera.width;
    const height = camera.height;
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    g.fillStyle = '#05070a';
    g.fillRect(0, 0, width, height);
    if (imageEl) g.drawImage(imageEl, 0, 0, width, height);
    if (!points || !pose || !projector) return;

    const buckets = Array.from({ length: 16 }, () => []);
    const size = Math.max(1, Number(sizeEl.value));
    const alpha = Number(opacityEl.value);
    const useIntensity =
      colorEl.value === 'intensity' &&
      pointStride >= 4 &&
      intensityMax > intensityMin;
    const colors = useIntensity ? intensityPalette : palette;
    const step = Math.max(3, pointStride);
    const m = pose.elements;
    for (let i = 0; i + step - 1 < points.length; i += step) {
      const X = points[i];
      const Y = points[i + 1];
      const Z = points[i + 2];
      const x = m[0] * X + m[4] * Y + m[8] * Z + m[12];
      const y = m[1] * X + m[5] * Y + m[9] * Z + m[13];
      const z = m[2] * X + m[6] * Y + m[10] * Z + m[14];
      if (z <= 0.1) continue;
      const [u, v] = projector(x, y, z);
      if (!Number.isFinite(u) || !Number.isFinite(v)) continue;
      if (u < 0 || u >= width || v < 0 || v >= height) continue;
      let value;
      if (useIntensity) {
        value = (points[i + 3] - intensityMin) / (intensityMax - intensityMin);
      } else {
        value = Math.min(50, Math.hypot(x, y, z)) / 50;
      }
      const bucket = Math.min(15, Math.max(0, Math.floor(value * 15)));
      buckets[bucket].push(u, v);
    }
    g.globalAlpha = alpha;
    for (let b = 0; b < buckets.length; b += 1) {
      const bucket = buckets[b];
      if (bucket.length === 0) continue;
      g.fillStyle = colors[b];
      for (let i = 0; i < bucket.length; i += 2) {
        g.fillRect(bucket[i] - size / 2, bucket[i + 1] - size / 2, size, size);
      }
    }
    g.globalAlpha = 1;
  }

  function translate(dx, dy, dz) {
    pose.elements[12] += dx;
    pose.elements[13] += dy;
    pose.elements[14] += dz;
    draw();
  }

  function rotate(axis, degree) {
    const delta = new Matrix4();
    if (axis === 'roll') delta.makeRotationX(degree * DEG_TO_RAD);
    else if (axis === 'pitch') delta.makeRotationY(degree * DEG_TO_RAD);
    else delta.makeRotationZ(degree * DEG_TO_RAD);
    // Extrinsic is right-multiplied: R = R0 * dR. The pure-rotation delta
    // leaves the translation untouched.
    pose.multiply(delta);
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
    const m = pose.elements;
    ctx.result({
      translation: [m[12], m[13], m[14]],
      rotation: new Quaternion().setFromRotationMatrix(pose).toArray(),
    });
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
    image(blob) {
      if (!blob) return;
      if (imageUrl) URL.revokeObjectURL(imageUrl);
      imageUrl = URL.createObjectURL(blob);
      const img = new Image();
      img.onload = () => {
        imageEl = img;
        imageFresh = true;
        draw();
        if (cloudPending) {
          cloudPending = false;
          ctx.requestCloud();
        }
      };
      img.src = imageUrl;
    },
    cloud(data) {
      points = data.points || data;
      pointStride = data.stride || 3;
      intensityMin = 0;
      intensityMax = 0;
      if (pointStride >= 4) {
        intensityMin = Infinity;
        intensityMax = -Infinity;
        for (let i = 3; i < points.length; i += pointStride) {
          const value = points[i];
          if (!Number.isFinite(value)) continue;
          if (value < intensityMin) intensityMin = value;
          if (value > intensityMax) intensityMax = value;
        }
        if (!Number.isFinite(intensityMin)) {
          intensityMin = 0;
          intensityMax = 0;
        }
      }
      draw();
    },
    params(message) {
      const nextCamera = message.camera?.[0];
      const extrinsic = message.extrinsic?.[0];
      if (nextCamera) {
        camera = nextCamera;
        projector = createProjector(camera);
        modelEl.textContent = `${camera.model} ${camera.width}x${camera.height} intrinsics=[${camera.intrinsics.map((v) => v.toFixed(3)).join(', ')}]`;
        draw();
      }
      if (extrinsic) {
        initialPose = matrixFromExtrinsic(extrinsic);
        pose = initialPose.clone();
        draw();
      }
    },
    dispose() {
      if (imageUrl) URL.revokeObjectURL(imageUrl);
      imageUrl = null;
      if (raf) cancelAnimationFrame(raf);
      raf = 0;
    },
  };
}
