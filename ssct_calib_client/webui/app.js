const modeEl = document.getElementById('mode');
const topicWrapEl = document.getElementById('topic-wrap');
const topicEl = document.getElementById('topic');
const connectEl = document.getElementById('connect');
const dialogEl = document.getElementById('topic-dialog');
const topicListEl = document.getElementById('topic-list');
const dialogRefreshEl = document.getElementById('dialog-refresh');
const dialogCancelEl = document.getElementById('dialog-cancel');
const dialogConnectEl = document.getElementById('dialog-connect');
const panelHost = document.getElementById('panel');

const state = {
  ws: null,
  wsOpen: false,
  modes: [],
  mode: null,
  ready: false,
  list: [],
  selected: null,
  connected: false,
  panel: null,
  pendingImage: null,
  pendingClouds: new Map(),
  pendingParams: null,
};

const ctx = {
  send(payload) {
    send({ type: 'command', ...payload });
  },
  result(payload) {
    send({ type: 'result', ...payload });
  },
  requestCloud() {
    send({ type: 'cloud' });
  },
};

function send(payload) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(payload));
  }
}

function currentMode() {
  return state.modes.find((mode) => mode.id === state.mode) || null;
}

function modeLabel() {
  return currentMode()?.meta.label || state.mode || '标定';
}

function setMode(id) {
  state.mode = id;
  modeEl.value = id || '';
  document.title = `SSCT ${modeLabel()}`;
}

function updateControls() {
  connectEl.disabled = !state.wsOpen || !state.ready;
  connectEl.textContent = state.connected ? 'disconnect' : 'connect';
  const hasTopic = state.connected && Boolean(state.selected);
  topicWrapEl.hidden = !hasTopic;
  topicEl.textContent = hasTopic ? state.selected : '';
}

function resetSelection() {
  state.panel?.dispose?.();
  state.panel = null;
  state.selected = null;
  state.pendingImage = null;
  state.pendingClouds.clear();
  state.pendingParams = null;
  panelHost.replaceChildren();
}

function selectTopic(topic) {
  resetSelection();
  state.selected = topic;
  mountPanel();
  send({ type: 'select', topic });
}

function disconnectTopic() {
  state.connected = false;
  selectTopic(null);
  renderTopics();
  updateControls();
}

function modeForTopic(topic) {
  const item = state.list.find((entry) => entry.topic === topic);
  const type = item?.calibration_type;
  if (!type) return null;
  return state.modes.find((mode) => mode.meta.calibrationType === type) || null;
}

function visibleNodes() {
  const wanted = currentMode()?.meta.calibrationType;
  if (!wanted) return state.list;
  return state.list.filter((item) => item.calibration_type === wanted);
}

function renderTopics() {
  if (!state.ready) return;
  const nodes = visibleNodes();
  dialogConnectEl.disabled = nodes.length === 0;
  if (nodes.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'empty';
    empty.textContent = `没有找到${modeLabel()}节点`;
    topicListEl.replaceChildren(empty);
    return;
  }
  topicListEl.replaceChildren(
    ...nodes.map((item) => {
      const label = document.createElement('label');
      label.className = 'topic-option';
      const radio = document.createElement('input');
      radio.type = 'radio';
      radio.name = 'topic';
      radio.value = item.topic;
      radio.checked = item.topic === state.selected;
      const text = document.createElement('span');
      text.textContent = item.topic;
      label.append(radio, text);
      return label;
    }),
  );
  if (!topicListEl.querySelector('input:checked')) {
    const first = topicListEl.querySelector('input');
    if (first) first.checked = true;
  }
}

function mountPanel() {
  if (state.panel || !state.ready) return;
  const mode = currentMode();
  if (!mode) return;
  state.panel = mode.module.mount(panelHost, ctx);
}

function onStatus(status) {
  if (!state.connected || !state.selected) return;
  if (status.topic && status.topic !== state.selected) return;
  mountPanel();
  state.panel?.update(status);
  if (state.pendingImage) {
    state.panel?.image(state.pendingImage);
    state.pendingImage = null;
  }
  for (const cloud of state.pendingClouds.values()) {
    state.panel?.cloud(cloud);
  }
  state.pendingClouds.clear();
  if (state.pendingParams) {
    state.panel?.params(state.pendingParams);
    state.pendingParams = null;
  }
}

function onBinary(buffer) {
  const headerLength = new DataView(buffer).getUint32(0, true);
  const header = JSON.parse(
    new TextDecoder().decode(new Uint8Array(buffer, 4, headerLength)),
  );
  if (!state.connected || !state.selected) return;
  const payload = new Uint8Array(buffer, 4 + headerLength);
  if (header.stream === 'image') {
    // Debug images belong to the selected status topic; clouds are routed by
    // their own topic so a mode can show target and source together.
    if (header.topic && header.topic !== state.selected) return;
    const blob = new Blob([payload], { type: 'image/jpeg' });
    if (state.panel) state.panel.image(blob);
    else state.pendingImage = blob;
  } else if (header.stream === 'cloud') {
    const data = {
      points: new Float32Array(payload.slice().buffer),
      stride: header.stride || 3,
      topic: header.topic,
    };
    if (state.panel) state.panel.cloud?.(data);
    else state.pendingClouds.set(header.topic, data);
  }
}

function requestList() {
  if (!state.ready) return;
  send({ type: 'list' });
}

async function loadModes(config) {
  const modes = [];
  for (const item of config.modes || []) {
    try {
      const module = await import(item.url);
      modes.push({ id: item.id, module, meta: module.meta || {} });
    } catch (error) {
      console.error(`failed to load mode ${item.id}`, error);
    }
  }
  state.modes = modes;
  modeEl.replaceChildren(
    ...modes.map((mode) => {
      const option = document.createElement('option');
      option.value = mode.id;
      option.textContent = mode.meta.label || mode.id;
      return option;
    }),
  );
  let mode = modes.find((item) => item.id === config.mode);
  if (!mode) mode = modes.find((item) => item.id === 'generic');
  if (!mode) mode = modes[0] || null;
  setMode(mode?.id || null);
  state.ready = true;
  updateControls();
  mountPanel();
  if (state.wsOpen) requestList();
}

function onJson(message) {
  if (message.type === 'config') {
    loadModes(message).catch((error) => console.error(error));
  } else if (message.type === 'topics') {
    state.list = message.items;
    renderTopics();
  } else if (message.type === 'status') {
    onStatus(message);
  } else if (message.type === 'params') {
    if (state.panel) state.panel.params?.(message);
    else state.pendingParams = message;
  } else if (message.type === 'error') {
    console.warn(message.message);
  }
}

function openSocket() {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${scheme}://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  state.ws = ws;
  ws.onopen = () => {
    if (state.ws !== ws) return;
    state.wsOpen = true;
    updateControls();
    requestList();
  };
  ws.onclose = () => {
    if (state.ws !== ws) return;
    state.wsOpen = false;
    state.connected = false;
    dialogEl.close();
    resetSelection();
    updateControls();
    mountPanel();
    setTimeout(openSocket, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (event) => {
    if (typeof event.data === 'string') {
      onJson(JSON.parse(event.data));
    } else {
      onBinary(event.data);
    }
  };
}

function openTopicDialog() {
  renderTopics();
  dialogEl.showModal();
}

modeEl.addEventListener('change', () => {
  state.connected = false;
  setMode(modeEl.value);
  resetSelection();
  renderTopics();
  mountPanel();
  updateControls();
  send({ type: 'select', topic: null });
});

connectEl.addEventListener('click', () => {
  if (state.connected) disconnectTopic();
  else openTopicDialog();
});

dialogCancelEl.addEventListener('click', () => dialogEl.close());
dialogRefreshEl.addEventListener('click', requestList);
dialogConnectEl.addEventListener('click', () => {
  const picked = topicListEl.querySelector('input[name="topic"]:checked');
  if (!picked) return;
  dialogEl.close();
  if (state.mode === 'generic') {
    const match = modeForTopic(picked.value);
    if (match) setMode(match.id);
  }
  state.connected = true;
  selectTopic(picked.value);
  renderTopics();
  updateControls();
});

openSocket();
