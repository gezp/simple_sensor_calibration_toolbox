// Translation / rotation step controls, matching the Python lidar-cam client.

export function createAxes(container, ctx) {
  const groups = [
    {
      title: 'translation step:',
      unit: 'm',
      step: 0.01,
      keys: ['x', 'y', 'z'],
    },
    {
      title: 'rotation step:',
      unit: 'deg',
      step: 1,
      keys: ['yaw', 'pitch', 'roll'],
    },
  ];
  for (const group of groups) {
    const box = document.createElement('div');
    box.className = 'axis-group';
    const head = document.createElement('label');
    head.className = 'axis-head';
    const title = document.createElement('span');
    title.className = 'field-label';
    title.textContent = group.title;
    const input = document.createElement('input');
    input.type = 'number';
    input.min = '0.001';
    input.step = '0.001';
    input.value = String(group.step);
    const unit = document.createElement('span');
    unit.className = 'unit';
    unit.textContent = group.unit;
    head.append(title, input, unit);
    const buttons = document.createElement('div');
    buttons.className = 'axis-buttons';
    for (const key of group.keys) {
      for (const sign of [1, -1]) {
        const button = document.createElement('button');
        button.textContent = `${sign > 0 ? '+' : '-'}${key}`;
        button.addEventListener('click', () => {
          const step = Number(input.value);
          if (!Number.isFinite(step) || step <= 0) return;
          ctx.send({ command: 'custom', key, value: sign * step });
        });
        buttons.append(button);
      }
    }
    box.append(head, buttons);
    container.append(box);
  }
}
