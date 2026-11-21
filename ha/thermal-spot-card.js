// Home Assistant dashboard card for an esphome-video thermal camera's spot
// meters: shows the camera live; a click on the image adds a spot of the
// chosen material there, a click on a spot removes it. The camera draws each
// spot in its own colour; the list below the image shows the same colours,
// with each spot's material.
//
//   type: custom:thermal-spot-card
//   camera: camera.esp32_p4_dev_kit_thermal_camera
//   spots: sensor.esp32_p4_dev_kit_thermal_spots   # the isp's `spots` text sensor
//   device: esp32_p4_devkit                         # the ESPHome node name, with _ for -
//
// The device needs the four API actions from the README (thermal_spot,
// thermal_spot_material, thermal_remove_spot, thermal_clear_spots); other
// names can be given as toggle_action, material_action, remove_action and
// clear_action (e.g. esphome.my_device_add_spot).

// Keep in step with MATERIALS in components/isp/isp_core.cpp.
const MATERIALS = [
  ['default', 'Default'],
  ['tortoise_shell', 'Tortoise shell (0.96)'],
  ['skin', 'Skin (0.98)'],
  ['black_plastic', 'Black plastic (0.95)'],
  ['plastic', 'Plastic (0.92)'],
  ['rubber', 'Rubber (0.95)'],
  ['wood', 'Wood (0.90)'],
  ['paper', 'Paper, cardboard (0.93)'],
  ['fabric', 'Fabric (0.95)'],
  ['water', 'Water (0.96)'],
  ['glass', 'Glass (0.92)'],
  ['ceramic', 'Ceramic tile (0.92)'],
  ['concrete', 'Concrete (0.94)'],
  ['brick', 'Brick (0.93)'],
  ['soil', 'Soil (0.94)'],
  ['sand', 'Sand (0.90)'],
  ['plants', 'Plants (0.96)'],
  ['paint', 'Painted surface (0.93)'],
  ['anodized', 'Aluminum, anodized (0.77)'],
  ['aluminum', 'Aluminum, bare (0.10)'],
  ['stainless_steel', 'Stainless steel (0.16)'],
];
// Keep in step with SPOT_COLORS in components/overlay/overlay_core.cpp: a
// spot's colour index picks its reticle's colour.
const SPOT_COLORS = ['rgb(0, 255, 255)', 'rgb(0, 255, 0)', 'rgb(255, 0, 255)', 'rgb(60, 140, 255)',
  'rgb(255, 255, 0)', 'rgb(255, 140, 0)', 'rgb(255, 105, 180)', 'rgb(160, 100, 255)'];
const STORE_KEY = 'thermal-spot-card.material';

const CSS = `
  .frame { position: relative; line-height: 0; background: #000; }
  .frame img { width: 100%; display: block; min-height: 120px; }
  .hit { position: absolute; inset: 0; cursor: crosshair; }
  .ping { position: absolute; width: 24px; height: 24px; margin: -12px 0 0 -12px; border-radius: 50%;
    border: 2px solid #fff; pointer-events: none; animation: ping 1.2s ease-out forwards; }
  @keyframes ping { to { transform: scale(2.2); opacity: 0; } }
  .bar, .row { display: flex; align-items: center; gap: 8px; padding: 8px 16px; }
  .bar { flex-wrap: wrap; }
  .bar label { color: var(--secondary-text-color); font-size: 14px; }
  .grow { flex: 1; }
  select { flex: 1; min-width: 0; font: inherit; padding: 4px; color: var(--primary-text-color);
    background: var(--card-background-color); border: 1px solid var(--divider-color); border-radius: 4px; }
  button { font: inherit; padding: 4px 10px; cursor: pointer; color: var(--primary-text-color);
    background: none; border: 1px solid var(--divider-color); border-radius: 4px; }
  button:disabled { opacity: .5; cursor: default; }
  .swatch { flex: none; width: 22px; height: 22px; box-sizing: border-box; border-radius: 50%;
    border: 2px solid #000; }
  .list { padding-bottom: 8px; }
  .empty, .error { padding: 0 16px 12px; font-size: 14px; color: var(--secondary-text-color); }
  .error { color: var(--error-color); }
  .error:empty, .empty[hidden] { display: none; }
`;

function parseSpots(text) {
  if (!text || text === 'unknown' || text === 'unavailable') return [];
  return text.split(';').filter(Boolean).map((part) => {
    const [x, y, material, color] = part.split(',');
    return { x: parseFloat(x), y: parseFloat(y), material: material || 'default', color: parseInt(color, 10) };
  }).filter((s) => Number.isFinite(s.x) && Number.isFinite(s.y));
}

function materialSelect(selected) {
  const sel = document.createElement('select');
  for (const [key, label] of MATERIALS) sel.add(new Option(label, key, false, key === selected));
  if (!MATERIALS.some(([key]) => key === selected)) sel.add(new Option(selected, selected, false, true));
  return sel;
}

class ThermalSpotCard extends HTMLElement {
  setConfig(config) {
    if (!config.camera || !config.spots) throw new Error('camera and spots are required');
    if (!config.device && !(config.toggle_action && config.material_action && config.remove_action && config.clear_action))
      throw new Error('device (or all four *_action options) is required');
    const action = (key, name) => config[key] || `esphome.${config.device}_${name}`;
    this._config = config;
    this._actions = {
      toggle: action('toggle_action', 'thermal_spot'),
      material: action('material_action', 'thermal_spot_material'),
      remove: action('remove_action', 'thermal_remove_spot'),
      clear: action('clear_action', 'thermal_clear_spots'),
    };
  }

  set hass(hass) {
    this._hass = hass;
    if (!this._root) this._build();
    const cam = hass.states[this._config.camera];
    this._token = cam && cam.attributes.access_token;
    if (!this._img.getAttribute('src')) this._startStream();
    const text = (hass.states[this._config.spots] || {}).state;
    if (text !== this._spotsText) {
      this._spotsText = text;
      this._spots = parseSpots(text);
      this._renderSpots();
    }
  }

  connectedCallback() {
    // The stream runs while the card is on the page, and stops only when the
    // page is hidden later: a page can load hidden (the Companion app) and
    // must show the camera all the same.
    this._hidden = false;
    this._onVisibility = () => {
      this._hidden = document.hidden;
      if (this._hidden) this._stopStream();
      else this._startStream();
    };
    document.addEventListener('visibilitychange', this._onVisibility);
    this._startStream();
  }

  disconnectedCallback() {
    document.removeEventListener('visibilitychange', this._onVisibility);
    clearTimeout(this._retry);
    this._retrying = false;
    this._stopStream();
  }

  getCardSize() { return 8; }

  static getStubConfig() {
    return { camera: '', spots: '', device: '' };
  }

  _build() {
    this._root = this.attachShadow({ mode: 'open' });
    const style = document.createElement('style');
    style.textContent = CSS;
    const card = document.createElement('ha-card');
    if (this._config.title) card.setAttribute('header', this._config.title);

    const frame = document.createElement('div');
    frame.className = 'frame';
    this._img = document.createElement('img');
    this._img.alt = '';
    this._img.addEventListener('error', () => {
      // A dropped stream (or an expired token): try again with the current one.
      if (!this._img.getAttribute('src')) return;
      this._stopStream();
      this._retrying = true;
      clearTimeout(this._retry);
      this._retry = setTimeout(() => {
        this._retrying = false;
        this._startStream();
      }, 3000);
    });
    this._marks = document.createElement('div');
    const hit = document.createElement('div');
    hit.className = 'hit';
    hit.addEventListener('click', (e) => this._click(e));
    frame.append(this._img, this._marks, hit);

    const bar = document.createElement('div');
    bar.className = 'bar';
    const label = document.createElement('label');
    label.textContent = 'New spots:';
    let saved = 'tortoise_shell';
    try { saved = localStorage.getItem(STORE_KEY) || saved; } catch (e) { /* no storage */ }
    this._material = materialSelect(saved);
    this._material.addEventListener('change', () => {
      try { localStorage.setItem(STORE_KEY, this._material.value); } catch (e) { /* no storage */ }
    });
    this._clear = document.createElement('button');
    this._clear.textContent = 'Clear all';
    this._clear.addEventListener('click', () => this._call('clear', {}));
    bar.append(label, this._material, this._clear);

    this._list = document.createElement('div');
    this._list.className = 'list';
    this._empty = document.createElement('div');
    this._empty.className = 'empty';
    this._empty.textContent = 'Click the image to place a spot; click a spot to remove it.';
    this._error = document.createElement('div');
    this._error.className = 'error';

    card.append(frame, bar, this._list, this._empty, this._error);
    this._root.append(style, card);
  }

  _startStream() {
    if (!this._img || !this._token || !this.isConnected || this._hidden || this._retrying) return;
    if (this._img.getAttribute('src')) return;
    this._img.src = `/api/camera_proxy_stream/${this._config.camera}?token=${encodeURIComponent(this._token)}`;
  }

  _stopStream() {
    // An empty src ends the stream's request.
    if (this._img) this._img.removeAttribute('src');
  }

  _click(e) {
    const r = this._img.getBoundingClientRect();
    if (!r.width || !r.height) return;
    const x = ((e.clientX - r.left) / r.width) * 100;
    const y = ((e.clientY - r.top) / r.height) * 100;
    if (x < 0 || x > 100 || y < 0 || y > 100) return;
    const ping = document.createElement('div');
    ping.className = 'ping';
    ping.style.left = `${x}%`;
    ping.style.top = `${y}%`;
    this._marks.append(ping);
    setTimeout(() => ping.remove(), 1300);
    this._call('toggle', { x: Math.round(x * 10) / 10, y: Math.round(y * 10) / 10, material: this._material.value });
  }

  async _call(which, data) {
    const [domain, service] = this._actions[which].split('.');
    this._error.textContent = '';
    try {
      await this._hass.callService(domain, service, data);
    } catch (err) {
      this._error.textContent = `The camera did not take it: ${err.message || err}`;
    }
  }

  _renderSpots() {
    if (!this._root) return;
    this._list.replaceChildren(...this._spots.map((s, i) => {
      const row = document.createElement('div');
      row.className = 'row';
      const swatch = document.createElement('div');
      swatch.className = 'swatch';
      swatch.style.background = SPOT_COLORS[s.color] || '#fff';
      const sel = materialSelect(s.material);
      sel.addEventListener('change', () => this._call('material', { spot: i + 1, material: sel.value }));
      const del = document.createElement('button');
      del.textContent = 'Remove';
      del.addEventListener('click', () => this._call('remove', { spot: i + 1 }));
      row.append(swatch, sel, del);
      return row;
    }));
    this._empty.hidden = this._spots.length > 0;
    this._clear.disabled = this._spots.length === 0;
  }
}

customElements.define('thermal-spot-card', ThermalSpotCard);
window.customCards = window.customCards || [];
window.customCards.push({
  type: 'thermal-spot-card',
  name: 'Thermal spots',
  description: 'Place spot meters on an esphome-video thermal camera by clicking the image.',
});
