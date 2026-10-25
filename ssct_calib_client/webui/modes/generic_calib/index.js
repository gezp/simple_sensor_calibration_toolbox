// Generic calibration: all calibration nodes, same fields as the common Qt client.

import { createPanel } from '../../components/panel.js';

export const meta = {
  label: '通用标定',
  calibrationType: null,
};

export function mount(root, ctx) {
  return createPanel(root, ctx, { image: false });
}
