// Shared extrinsic helpers: pose is a three.js Matrix4 storing T_target_source.

import { Euler, Matrix4, Quaternion } from 'three';

export const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

export function matrixFromExtrinsic(extrinsic) {
  const quaternion = new Quaternion(...extrinsic.rotation).normalize();
  const matrix = new Matrix4().makeRotationFromQuaternion(quaternion);
  return matrix.setPosition(...extrinsic.translation);
}

// ZYX Euler angles in degrees. The pose itself stays as a matrix/quaternion.
export function eulerFromMatrix(matrix) {
  const euler = new Euler().setFromRotationMatrix(matrix, 'ZYX');
  return [euler.x, euler.y, euler.z].map((v) => v * RAD_TO_DEG);
}

export function translatePose(pose, dx, dy, dz) {
  pose.elements[12] += dx;
  pose.elements[13] += dy;
  pose.elements[14] += dz;
}

export function rotatePose(pose, axis, degree) {
  const delta = new Matrix4();
  if (axis === 'roll') delta.makeRotationX(degree * DEG_TO_RAD);
  else if (axis === 'pitch') delta.makeRotationY(degree * DEG_TO_RAD);
  else delta.makeRotationZ(degree * DEG_TO_RAD);
  // Extrinsic is right-multiplied: R = R0 * dR. The pure-rotation delta
  // leaves the translation untouched.
  pose.multiply(delta);
}

export function poseResult(pose) {
  const m = pose.elements;
  return {
    translation: [m[12], m[13], m[14]],
    rotation: new Quaternion().setFromRotationMatrix(pose).toArray(),
  };
}
