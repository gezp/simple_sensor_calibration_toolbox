// IMU intrinsic calibration: filtered to ssct_imu_intrinsic_calib.

import { createPanel } from '../../components/panel.js';

export const meta = {
  label: 'IMU内参标定',
  calibrationType: 'ssct_imu_intrinsic_calib',
};

export function mount(root, ctx) {
  return createPanel(root, ctx, { imu: true });
}
