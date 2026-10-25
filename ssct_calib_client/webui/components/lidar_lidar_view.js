// Two-cloud three.js view. The target cloud is fixed in its own frame; the
// source cloud is transformed by the current T_target_source pose.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const TARGET_COLOR = 0xaeb8c2;
const SOURCE_COLOR = 0xff5b5b;
const VIEW_DIR = new THREE.Vector3(0.8, -1, 0.6).normalize();
const VIEWS = {
  '3d': { dir: VIEW_DIR, up: [0, 0, 1] },
  front: { dir: new THREE.Vector3(1, 0, 0), up: [0, 0, 1] },
  side: { dir: new THREE.Vector3(0, 1, 0), up: [0, 0, 1] },
  top: { dir: new THREE.Vector3(0, 0, 1), up: [0, 1, 0] },
};

const clamp01 = (value) => Math.min(1, Math.max(0, value));

// Classic jet colormap: dark blue -> cyan -> green -> yellow -> red.
function jet(t) {
  return new THREE.Color(
    clamp01(1.5 - Math.abs(4 * t - 3)),
    clamp01(1.5 - Math.abs(4 * t - 2)),
    clamp01(1.5 - Math.abs(4 * t - 1)),
  );
}

function buildLayer(data, kind) {
  const points = data.points || data;
  const stride = data.stride || 3;
  const count = Math.floor(points.length / stride);
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  let minIntensity = Infinity;
  let maxIntensity = -Infinity;
  for (let i = 0; i < count; i += 1) {
    const o = i * stride;
    positions[i * 3] = points[o];
    positions[i * 3 + 1] = points[o + 1];
    positions[i * 3 + 2] = points[o + 2];
    if (stride >= 4 && Number.isFinite(points[o + 3])) {
      minIntensity = Math.min(minIntensity, points[o + 3]);
      maxIntensity = Math.max(maxIntensity, points[o + 3]);
    }
  }
  const hasIntensity = Number.isFinite(minIntensity);
  if (hasIntensity) {
    const range = Math.max(1e-9, maxIntensity - minIntensity);
    for (let i = 0; i < count; i += 1) {
      jet((points[i * stride + 3] - minIntensity) / range).toArray(colors, i * 3);
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  const material = new THREE.PointsMaterial({
    color: kind === 'target' ? TARGET_COLOR : SOURCE_COLOR,
    size: 2,
    sizeAttenuation: false,
    transparent: true,
    depthWrite: false,
  });

  const object = new THREE.Points(geometry, material);
  object.frustumCulled = false;
  return { object, geometry, material, count, hasIntensity, positions };
}

export function createLidarLidarView(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setClearColor(0x05070a);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 1e6);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  controls.addEventListener('change', () => renderer.render(scene, camera));

  const label = document.createElement('div');
  label.className = 'lidar-lidar-label';
  canvas.parentElement?.append(label);

  let target = null;
  let source = null;
  let pose = null;
  let axes = null;
  let center = new THREE.Vector3();
  let radius = 0;
  let targetColorMode = 'solid';
  let sourceColorMode = 'solid';
  let targetColor = '#aeb8c2';
  let sourceColor = '#ff5b5b';
  let targetSize = 2;
  let sourceSize = 2;
  let targetOpacity = 1;
  let sourceOpacity = 1;

  function applyPose() {
    if (!source) return;
    source.object.visible = Boolean(pose);
    if (!pose) return;
    source.object.matrixAutoUpdate = false;
    source.object.matrix.copy(pose);
    source.object.matrixWorldNeedsUpdate = true;
  }

  function replaceLayer(kind, data) {
    const previous = kind === 'target' ? target : source;
    if (previous) {
      scene.remove(previous.object);
      previous.geometry.dispose();
      previous.material.dispose();
    }
    const layer = buildLayer(data, kind);
    scene.add(layer.object);
    if (kind === 'target') {
      target = layer;
      frame();
    } else {
      source = layer;
      applyPose();
      if (!target) frame();
    }
    updateLabel();
    applyMaterials();
  }

  function bounds(layer) {
    let minX = Infinity;
    let minY = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    let maxZ = -Infinity;
    const p = layer.positions;
    for (let i = 0; i + 2 < p.length; i += 3) {
      if (!Number.isFinite(p[i]) || !Number.isFinite(p[i + 1]) || !Number.isFinite(p[i + 2])) {
        continue;
      }
      minX = Math.min(minX, p[i]);
      maxX = Math.max(maxX, p[i]);
      minY = Math.min(minY, p[i + 1]);
      maxY = Math.max(maxY, p[i + 1]);
      minZ = Math.min(minZ, p[i + 2]);
      maxZ = Math.max(maxZ, p[i + 2]);
    }
    if (!Number.isFinite(minX)) return null;
    return {
      center: new THREE.Vector3(
        (minX + maxX) / 2,
        (minY + maxY) / 2,
        (minZ + maxZ) / 2,
      ),
      radius: Math.max(0.5, Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2),
    };
  }

  function placeView(name) {
    const preset = VIEWS[name] || VIEWS['3d'];
    if (!radius) return;
    camera.up.set(...preset.up);
    camera.position.copy(center).addScaledVector(preset.dir, radius * 2.4);
    controls.target.copy(center);
    controls.update();
  }

  // Recenter only when a cloud changes, so pose edits do not move the view.
  function frame() {
    const layer = target || source;
    const box = layer && bounds(layer);
    if (!box) return;
    ({ center, radius } = box);
    axes?.geometry.dispose();
    axes?.material.dispose();
    if (axes) scene.remove(axes);
    axes = new THREE.AxesHelper(radius * 0.5);
    axes.position.copy(center);
    scene.add(axes);

    camera.near = Math.max(radius * 1e-4, 1e-3);
    camera.far = radius * 100;
    controls.minDistance = radius * 0.05;
    controls.maxDistance = radius * 30;
    placeView('3d');
  }

  function applyMaterials() {
    for (const layer of [target, source]) {
      if (!layer) continue;
      const isTarget = layer === target;
      const colorMode = isTarget ? targetColorMode : sourceColorMode;
      const vertexColors = colorMode === 'intensity' && layer.hasIntensity;
      if (layer.material.vertexColors !== vertexColors) {
        layer.material.vertexColors = vertexColors;
        layer.material.needsUpdate = true;
      }
      layer.material.color.set(vertexColors ? 0xffffff : (isTarget ? targetColor : sourceColor));
      layer.material.size =
        Math.max(1, Number(isTarget ? targetSize : sourceSize)) * renderer.getPixelRatio();
      layer.material.opacity = isTarget ? targetOpacity : sourceOpacity;
    }
  }

  function updateLabel() {
    if (!target && !source) {
      label.textContent = 'no point cloud';
      return;
    }
    label.textContent = `target ${target?.count ?? 0} / source ${source?.count ?? 0} points`;
  }

  function draw() {
    applyPose();
    const width = canvas.clientWidth || 480;
    const height = canvas.clientHeight || 340;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    applyMaterials();
    renderer.render(scene, camera);
  }

  new ResizeObserver(draw).observe(canvas);
  draw();
  updateLabel();

  return {
    setCloud(kind, data) {
      if (!data || !data.points) return;
      replaceLayer(kind, data);
      draw();
    },
    setPose(matrix) {
      pose = matrix;
      applyPose();
      draw();
    },
    setOptions(options) {
      if (options.targetColorMode !== undefined) targetColorMode = options.targetColorMode;
      if (options.sourceColorMode !== undefined) sourceColorMode = options.sourceColorMode;
      if (options.targetColor !== undefined) targetColor = options.targetColor;
      if (options.sourceColor !== undefined) sourceColor = options.sourceColor;
      if (options.targetSize !== undefined) targetSize = options.targetSize;
      if (options.sourceSize !== undefined) sourceSize = options.sourceSize;
      if (options.targetOpacity !== undefined) targetOpacity = options.targetOpacity;
      if (options.sourceOpacity !== undefined) sourceOpacity = options.sourceOpacity;
      draw();
    },
    setView(name) {
      placeView(name);
      draw();
    },
    draw,
  };
}
