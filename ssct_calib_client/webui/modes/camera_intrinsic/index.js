// Camera intrinsic calibration: filtered to ssct_camera_intrinsic_calib.

import { createPanel } from '../../components/panel.js';

export const meta = {
  label: '相机内参标定',
  calibrationType: 'ssct_camera_intrinsic_calib',
};

export function mount(root, ctx) {
  return createPanel(root, ctx);
}
