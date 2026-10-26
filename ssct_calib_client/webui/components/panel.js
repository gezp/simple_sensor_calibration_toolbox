// Shared calibration panel laid out like the Python Qt clients:
// metadata, debug image, status/progress, info, then commands.

import { createCloudView } from './pointcloud.js';
import { createImuView } from './imu_plot.js';

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

const BASICS = [
  ['start', 'start', 'primary'],
  ['reset', 'reset', ''],
  ['save', 'save result', ''],
];

export function createPanel(root, ctx, { image = true, cloud = false, imu = false } = {}) {
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
      </section>

      <div class="views">
        <figure class="view" data-imuview hidden>
          <figcaption class="field-label">imu data:</figcaption>
          <div class="viewport imu-viewport">
            <canvas class="imu" data-imucanvas></canvas>
          </div>
        </figure>
        <figure class="view" data-imageview>
          <figcaption class="field-label">debug image:</figcaption>
          <div class="viewport">
            <img class="debug" alt="debug image" hidden>
            <div class="placeholder" data-empty>waiting for debug image</div>
          </div>
        </figure>
        <figure class="view cloud-view" data-cloudview hidden>
          <figcaption class="field-label">
            point cloud:
            <button class="ghost small" data-cloud type="button">refresh</button>
          </figcaption>
          <div class="viewport">
            <canvas class="cloud" data-cloudcanvas></canvas>
          </div>
        </figure>
      </div>

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
  const stateEl = query('[data-state]');
  const typeEl = query('[data-type]');
  const frameEl = query('[data-frame]');
  const childEl = query('[data-child-frame]');
  const infoEl = query('[data-info]');
  const imageEl = query('img.debug');
  const emptyEl = query('[data-empty]');
  let imageUrl = null;

  if (!image) query('[data-imageview]')?.remove();
  if (!cloud) query('[data-cloudview]')?.remove();
  if (!imu) query('[data-imuview]')?.remove();
  if (!query('.view')) query('.views')?.remove();
  if (imu) query('.views')?.classList.add('stacked');

  for (const [command, label, kind] of BASICS) {
    const button = document.createElement('button');
    button.textContent = label;
    if (kind) button.className = kind;
    button.addEventListener('click', () => {
      if (
        command === 'save' &&
        !confirm('Save calibration result? This overwrites the output file.')
      ) {
        return;
      }
      ctx.send({ command });
    });
    query('[data-basic]').append(button);
  }

  let imuView = null;
  if (imu) {
    query('[data-imuview]').hidden = false;
    imuView = createImuView(query('[data-imucanvas]'));
  }

  let cloudView = null;
  if (cloud) {
    query('[data-cloudview]').hidden = false;
    cloudView = createCloudView(query('[data-cloudcanvas]'));
    query('[data-cloud]').addEventListener('click', () => ctx.requestCloud());
    ctx.requestCloud();
  }

  return {
    update(status) {
      const state = Number(status.state);
      stateEl.textContent = STATE_TEXT[state] || 'unknown';
      stateEl.className = `state-field ${STATE_CLASS[state] || ''}`;
      typeEl.textContent = status.calibration_type || 'unknown';
      frameEl.textContent = status.frame_id || 'unknown';
      childEl.textContent = status.child_frame_id || 'unknown';
      infoEl.textContent = status.info || '';
    },
    image(blob) {
      if (!imageEl || !blob) return;
      const next = URL.createObjectURL(blob);
      imageEl.src = next;
      imageEl.hidden = false;
      emptyEl.hidden = true;
      if (imageUrl) URL.revokeObjectURL(imageUrl);
      imageUrl = next;
    },
    imu(data) {
      imuView?.push(data);
    },
    cloud(data) {
      cloudView?.setPoints(data);
    },
    dispose() {
      if (imageUrl) URL.revokeObjectURL(imageUrl);
      imageUrl = null;
    },
  };
}
