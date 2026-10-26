// Live IMU curves from sensor_msgs/Imu. Samples are [gx, gy, gz, ax, ay, az].

const MAX_SAMPLES = 1000;
const COLORS = ['#f85149', '#3fb950', '#58a6ff'];
const AXES = ['x', 'y', 'z'];
const PANELS = [
  { offset: 3, title: 'accel (m/s²)' },
  { offset: 0, title: 'gyro (rad/s)' },
];

export function createImuView(canvas) {
  const g = canvas.getContext('2d');
  const samples = [];

  function drawPanel(dpr, x0, y0, width, height, { offset, title }) {
    let min = Infinity;
    let max = -Infinity;
    for (const sample of samples) {
      for (let axis = 0; axis < 3; axis += 1) {
        const value = sample[offset + axis];
        if (value < min) min = value;
        if (value > max) max = value;
      }
    }
    if (!(max > min)) {
      const center = Number.isFinite(min) ? min : 0;
      min = center - 1;
      max = center + 1;
    }
    const plotX = x0 + 52 * dpr;
    const plotY = y0 + 20 * dpr;
    const plotW = width - 60 * dpr;
    const plotH = height - 36 * dpr;

    // Nice tick step (1/2/5×10^n), then snap the range to exactly 5 ticks
    // spaced by that step.
    const range = max - min;
    const pow = 10 ** Math.floor(Math.log10(range / 4));
    const norm = range / 4 / pow;
    const mult = norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10;
    const step = mult * pow;
    const center = Math.round((min + max) / (2 * step)) * step;
    min = center - 2 * step;
    max = center + 2 * step;
    const digits = Math.max(0, Math.ceil(-Math.log10(step) - 1e-9));

    g.strokeStyle = '#30363d';
    g.lineWidth = dpr;
    g.strokeRect(plotX, plotY, plotW, plotH);
    g.textAlign = 'right';
    g.fillStyle = '#8b949e';
    for (let tick = 0; tick <= 4; tick += 1) {
      const py = plotY + plotH - (tick / 4) * plotH;
      if (tick > 0 && tick < 4) {
        g.beginPath();
        g.moveTo(plotX, py);
        g.lineTo(plotX + plotW, py);
        g.stroke();
      }
      g.fillText(
        (min + tick * step).toFixed(digits),
        plotX - 5 * dpr,
        py + 4 * dpr,
      );
    }
    g.textAlign = 'left';

    g.fillStyle = '#8b949e';
    g.fillText(title, plotX, y0 + 13 * dpr);
    for (let axis = 0; axis < 3; axis += 1) {
      g.fillStyle = COLORS[axis];
      g.fillText(
        AXES[axis],
        plotX + plotW - (48 - axis * 14) * dpr,
        y0 + 13 * dpr,
      );
    }

    const stride = Math.max(1, Math.ceil(samples.length / plotW));
    const span = samples.length > 1 ? samples.length - 1 : 1;
    for (let axis = 0; axis < 3; axis += 1) {
      g.strokeStyle = COLORS[axis];
      g.lineWidth = 1.2 * dpr;
      g.beginPath();
      for (let i = 0; i < samples.length; i += stride) {
        const px = plotX + (i / span) * plotW;
        const py =
          plotY + plotH - ((samples[i][offset + axis] - min) / (max - min)) * plotH;
        if (i === 0) g.moveTo(px, py);
        else g.lineTo(px, py);
      }
      g.stroke();
    }
  }

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const cssWidth = canvas.clientWidth || 480;
    const cssHeight = canvas.clientHeight || 320;
    const width = Math.round(cssWidth * dpr);
    const height = Math.round(cssHeight * dpr);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.fillStyle = '#05070a';
    g.fillRect(0, 0, width, height);
    g.font = `${12 * dpr}px system-ui, sans-serif`;
    g.textAlign = 'left';

    if (samples.length === 0) {
      g.fillStyle = '#8b949e';
      g.fillText('waiting for imu data', 12 * dpr, 22 * dpr);
      return;
    }

    const gap = 24 * dpr;
    const half = (width - gap) / 2;
    for (let i = 0; i < PANELS.length; i += 1) {
      drawPanel(dpr, i * (half + gap), 0, half, height, PANELS[i]);
    }
  }

  function push(data) {
    if (!Array.isArray(data?.samples)) return;
    for (const sample of data.samples) {
      if (Array.isArray(sample) && sample.length >= 6) samples.push(sample);
    }
    if (samples.length > MAX_SAMPLES) samples.splice(0, samples.length - MAX_SAMPLES);
    draw();
  }

  new ResizeObserver(draw).observe(canvas);
  draw();

  return { push };
}
