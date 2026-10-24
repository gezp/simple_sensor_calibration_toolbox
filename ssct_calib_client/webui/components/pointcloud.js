// ponytail: plain canvas projection, no clipping, intensity colors or LOD.
// Swap to Three.js when the viewer needs shaders or > 50k points.

export function createCloudView(canvas) {
  const g = canvas.getContext('2d');
  const palette = Array.from({ length: 32 }, (_, i) => {
    const t = i / 31;
    return `hsl(${210 - 170 * t} 90% ${42 + 22 * t}%)`;
  });
  let points = null;
  let center = [0, 0, 0];
  let radius = 1;
  let zmin = 0;
  let zmax = 1;
  let yaw = 0.6;
  let pitch = -0.35;
  let zoom = 1;
  let drag = null;

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const cssWidth = canvas.clientWidth || 480;
    const cssHeight = canvas.clientHeight || 340;
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

    if (!points) {
      g.fillStyle = '#8b949e';
      g.fillText('no point cloud', 12 * dpr, 22 * dpr);
      return;
    }

    const cx = width / 2;
    const cy = height / 2;
    const focal = Math.min(width, height) * 0.9;
    const distance = (radius * 2.4) / zoom;
    const cosY = Math.cos(yaw);
    const sinY = Math.sin(yaw);
    const cosP = Math.cos(pitch);
    const sinP = Math.sin(pitch);

    const project = (px, py, pz) => {
      const x = px - center[0];
      const y = py - center[1];
      const z = pz - center[2];
      const x1 = x * cosY - y * sinY;
      const y1 = x * sinY + y * cosY;
      const y2 = y1 * cosP - z * sinP;
      const z2 = y1 * sinP + z * cosP;
      const depth = z2 + distance;
      if (depth <= 0.01) return null;
      return [cx + (focal * x1) / depth, cy - (focal * y2) / depth];
    };

    const axes = [
      [[radius * 0.4, 0, 0], '#f85149', 'X'],
      [[0, radius * 0.4, 0], '#3fb950', 'Y'],
      [[0, 0, radius * 0.4], '#58a6ff', 'Z'],
    ];
    const origin = project(center[0], center[1], center[2]);
    g.lineWidth = 1.2 * dpr;
    for (const [axis, color, label] of axes) {
      const tip = project(center[0] + axis[0], center[1] + axis[1], center[2] + axis[2]);
      if (!origin || !tip) continue;
      g.strokeStyle = color;
      g.beginPath();
      g.moveTo(origin[0], origin[1]);
      g.lineTo(tip[0], tip[1]);
      g.stroke();
      g.fillStyle = color;
      g.fillText(label, tip[0] + 3 * dpr, tip[1] - 3 * dpr);
    }

    const buckets = Array.from({ length: 32 }, () => []);
    const range = Math.max(1e-6, zmax - zmin);
    for (let i = 0; i + 2 < points.length; i += 3) {
      const p = project(points[i], points[i + 1], points[i + 2]);
      if (p === null) continue;
      const index = Math.max(
        0,
        Math.min(31, Math.floor(((points[i + 2] - zmin) / range) * 31)),
      );
      buckets[index].push(p[0], p[1]);
    }
    g.globalAlpha = 0.9;
    const size = 1.5 * dpr;
    for (let b = 0; b < buckets.length; b += 1) {
      const bucket = buckets[b];
      if (bucket.length === 0) continue;
      g.fillStyle = palette[b];
      for (let i = 0; i < bucket.length; i += 2) {
        g.fillRect(bucket[i], bucket[i + 1], size, size);
      }
    }
    g.globalAlpha = 1;
    g.fillStyle = '#8b949e';
    g.fillText(`${points.length / 3 | 0} points`, 12 * dpr, 22 * dpr);
  }

  function setPoints(data) {
    if (!data || data.length < 3) {
      points = null;
      draw();
      return;
    }
    points = data;
    let minX = Infinity;
    let minY = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    let maxZ = -Infinity;
    for (let i = 0; i + 2 < data.length; i += 3) {
      minX = Math.min(minX, data[i]);
      maxX = Math.max(maxX, data[i]);
      minY = Math.min(minY, data[i + 1]);
      maxY = Math.max(maxY, data[i + 1]);
      minZ = Math.min(minZ, data[i + 2]);
      maxZ = Math.max(maxZ, data[i + 2]);
    }
    center = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2];
    radius = Math.max(0.5, Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2);
    zmin = minZ;
    zmax = maxZ;
    draw();
  }

  canvas.addEventListener('pointerdown', (event) => {
    drag = { x: event.clientX, y: event.clientY };
    canvas.setPointerCapture(event.pointerId);
    canvas.style.cursor = 'grabbing';
  });
  canvas.addEventListener('pointermove', (event) => {
    if (!drag) return;
    yaw += (event.clientX - drag.x) * 0.008;
    pitch = Math.max(-1.5, Math.min(1.5, pitch + (event.clientY - drag.y) * 0.008));
    drag = { x: event.clientX, y: event.clientY };
    draw();
  });
  const endDrag = () => {
    drag = null;
    canvas.style.cursor = 'grab';
  };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener(
    'wheel',
    (event) => {
      event.preventDefault();
      zoom = Math.max(0.15, Math.min(8, zoom * Math.exp(-event.deltaY * 0.001)));
      draw();
    },
    { passive: false },
  );
  new ResizeObserver(draw).observe(canvas);
  draw();

  return { setPoints };
}
