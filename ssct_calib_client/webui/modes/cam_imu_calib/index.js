// Camera-IMU calibration: filtered to ssct_cam_imu_calib.

import { createPanel } from '../../components/panel.js';

export const meta = {
  label: '相机-IMU标定',
  calibrationType: 'ssct_cam_imu_calib',
};

export function mount(root, ctx) {
  return createPanel(root, ctx);
}
