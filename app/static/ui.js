'use strict';
// OAK-D VIO web UI: vanilla JS, inline SVG, no external resources (works offline on a boat).
// Polling: /api/status and /api/history once a second while the tab is visible, snapshots only
// while auto-refresh is on, the ArduPilot parameters on load and on demand (the server's own background
// parameter check arrives with /api/status: the banner at the top).

const SVG_NS = 'http://www.w3.org/2000/svg';
const $ = id => document.getElementById(id);
const ui = {
  schema: null, fields: {}, initial: {}, status: null, history: null, historyEtag: null, historyKey: '',
  autopilot: null, busy: {}, pendingWrite: null,
};
const COLORS = { vx: '#6fa8d6', vy: '#e67e22', dvlx: '#c4e3fb', dvly: '#ffd7a8', grid: '#2c3647', axis: '#34425a',
                 text: '#8fa3bd', bad: '#e74c3c', ok: '#2ecc71', inl: '#e67e22', ring: '#2c3647', vehicle: '#8a97a8' };
const TRAIL_S = 2.5;

// --- helpers ------------------------------------------------------------------------------------

const visible = () => document.visibilityState === 'visible';
const isNum = v => typeof v === 'number' && isFinite(v);
const fmt = (v, digits = 1, suffix = '') => isNum(v) ? v.toFixed(digits) + suffix : '–';
const signed = (v, digits = 2, suffix = '') => {
  if (!isNum(v)) return '–';
  const text = v.toFixed(digits);
  return (Number(text) === 0 ? '+' + (0).toFixed(digits) : (v > 0 ? '+' : '') + text) + suffix;
};

function setText(id, text) {
  const node = $(id);
  if (node) node.textContent = text;
}

function showToast(message, type) {
  const toast = $('toast');
  toast.textContent = message;
  toast.className = 'toast show ' + type;
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => toast.className = 'toast', 3500);
}

function svg(tag, attrs, parent) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [key, value] of Object.entries(attrs || {})) node.setAttribute(key, value);
  if (parent) parent.appendChild(node);
  return node;
}

function svgText(parent, x, y, text, attrs) {
  const node = svg('text', { x, y, fill: COLORS.text, 'font-size': 11, ...attrs }, parent);
  node.textContent = text;
  return node;
}

function errorText(data, status) {
  if (!data) return 'Error ' + status;
  if (typeof data.detail === 'string') return data.detail;
  if (data.message) return data.message;
  if (Array.isArray(data.detail)) return data.detail.map(d => d.msg).join('; ');
  return 'Error ' + status;
}

async function fetchJSON(url, options) {
  const res = await fetch(url, options);
  const data = await res.json().catch(() => null);
  return { ok: res.ok, status: res.status, data, res };
}

async function once(name, fn) {
  if (ui.busy[name]) return;
  ui.busy[name] = true;
  try { await fn(); } finally { ui.busy[name] = false; }
}

function niceStep(span) {
  const raw = Math.max(span, 1e-9);
  const power = Math.pow(10, Math.floor(Math.log10(raw)));
  const unit = raw / power;
  return (unit >= 5 ? 5 : unit >= 2 ? 2 : 1) * power;
}

function niceCeil(value) {
  const step = niceStep(value / 2);
  return Number((Math.ceil(value / step) * step).toPrecision(6));
}

function duration(seconds) {
  if (!isNum(seconds)) return '–';
  const s = Math.floor(seconds);
  return [Math.floor(s / 3600), Math.floor(s / 60) % 60, s % 60].map(v => String(v).padStart(2, '0')).join(':');
}

// Segments of finite points: a rejected frame (null) breaks the line
function pathData(xs, ys, sx, sy) {
  let d = '', pen = false;
  for (let i = 0; i < xs.length; i++) {
    if (!isNum(xs[i]) || !isNum(ys[i])) { pen = false; continue; }
    d += (pen ? 'L' : 'M') + sx(xs[i]).toFixed(1) + ',' + sy(ys[i]).toFixed(1);
    pen = true;
  }
  return d;
}

function emptyPlot(node, width, height, message) {
  node.replaceChildren();
  svgText(node, width / 2, height / 2, message, { 'text-anchor': 'middle', 'font-size': 13 });
}

// --- status -------------------------------------------------------------------------------------

function pill(id, text, cls) {
  const node = $(id);
  node.textContent = text;
  node.className = 'pill ' + cls;
}

function cameraSummary(camera) {
  if (!camera || !camera.intrinsics) return '';
  const k = values => values.map(v => v.toFixed(1)).join(', ');
  let text = 'PnP camera matrix (' + camera.source + ', in air) fx, fy, cx, cy = ' + k(camera.intrinsics);
  if (camera.effective_intrinsics && camera.refractive_index !== undefined) {
    text += ' · n ' + camera.refractive_index + ' × depth scale ' + camera.depth_scale
      + ' → effective ' + k(camera.effective_intrinsics) + ', depth × ' + camera.total_depth_scale.toFixed(4);
  }
  return text;
}

function statusMessage(s, t) {
  if (!s.running) {
    if (s.restart_in !== null && s.restart_in !== undefined) {
      return ['VIO exited (code ' + s.last_exit_code + '), restarting in ' + Math.ceil(s.restart_in) + ' s', ''];
    }
    return ['VIO stopped: values below are from its last run (if any). Start it with Start or Apply.', ''];
  }
  if (!t) return ['Waiting for the VIO process to publish…', 'info'];
  if (t.stale) return ['VIO not publishing (last update ' + fmt(t.age_s, 0) + ' s ago)', ''];
  if (t.phase === 'opening_device') return ['Opening the camera…', 'info'];
  if (t.phase !== 'running') return ['VIO ' + t.phase + '…', 'info'];
  if (t.last_frame_age_s === null || t.last_frame_age_s > 3) return ['No frames from the camera for ' + fmt(t.last_frame_age_s, 0) + ' s', ''];
  return null;
}

function renderStatus(s) {
  const t = s.telemetry;
  const live = !!(t && !t.stale);
  const cam = s.camera || {};
  $('pill-mock').hidden = !s.mock;
  if (s.running) pill('pill-vio', 'VIO running', 'g');
  else if (s.restart_in !== null && s.restart_in !== undefined) pill('pill-vio', 'VIO restarting', 'y');
  else pill('pill-vio', 'VIO stopped', 'r');
  if (s.camera_connected === true) pill('pill-camera', 'camera connected', 'g');
  else if (s.camera_connected === false) pill('pill-camera', 'camera not connected', 'r');
  else pill('pill-camera', 'camera ?', 'dim');
  if (s.log_only) pill('pill-output', 'LOG-ONLY (no MAVLink output)', 'y');
  else if (s.mock) pill('pill-output', 'mock output (nothing sent)', 'dim');
  else pill('pill-output', 'sending MAVLink', 'g');

  const vehicle = s.vehicle || {};
  const armed = vehicle.armed === true ? 'ARMED' : vehicle.armed === false ? 'DISARMED' : '?';
  const checked = vehicle.checked_at ? ' (' + fmt(s.server_time - vehicle.checked_at, 0) + ' s ago)' : '';
  setText('header-right', 'vehicle: ' + armed + checked + ' · uptime ' + duration(s.uptime_s) + ' · CPU '
    + fmt(s.cpu_percent, 0, ' %'));

  renderParamCheck(s.autopilot_params);

  const banner = $('log-only-banner');
  banner.hidden = !s.log_only;
  banner.textContent = s.log_only_forced_by_env
    ? 'LOG-ONLY (forced by OAKD_LOG_ONLY on the container): nothing is sent to the autopilot'
    : 'LOG-ONLY: nothing is sent to the autopilot';
  setText('camera-status', cameraSummary(s.camera));

  const message = statusMessage(s, t);
  const box = $('status-message');
  box.hidden = !message;
  if (message) { box.textContent = message[0]; box.className = 'panel-message ' + message[1]; }
  for (const id of ['status-kv', 'failures-kv', 'link-kv', 'link-streams']) $(id).classList.toggle('stale', !live);
  document.querySelector('#panel-status .big').classList.toggle('stale', !live);

  const last = (t && t.last_frame) || {};
  const recent = (t && t.recent) || {};
  setText('big-fps', fmt(recent.fps, 1));
  setText('big-inliers', isNum(last.inliers) ? String(last.inliers) : '–');
  setText('big-ok', fmt(recent.ok_percent, 0, '%'));
  setText('big-ok-label', 'frames ok (' + (recent.window_s || 10) + ' s)');
  setText('big-proc', fmt(recent.proc_ms_mean, 0));
  setText('kv-features', isNum(last.features) ? last.features + ' / ' + last.valid_depth : '–');
  setText('kv-matches', isNum(last.matches) ? String(last.matches) : '–');
  setText('kv-n', isNum(cam.refractive_index) ? String(cam.refractive_index) : '–');
  setText('kv-depth-scale', isNum(cam.depth_scale)
    ? cam.depth_scale.toFixed(3) + ' (effective ×' + fmt(cam.total_depth_scale, 3) + ')' : '–');
  const k = cam.effective_intrinsics;
  setText('kv-k', k ? 'fx ' + k[0].toFixed(1) + ' fy ' + k[1].toFixed(1) + ' cx ' + k[2].toFixed(1) + ' cy ' + k[3].toFixed(1) : '–');
  setText('kv-resets', t ? t.reference_resets + ' / ' + t.gap_frames_total : '–');
  setText('kv-recoveries', t ? String(t.recoveries) : '–');
  setText('kv-cpu', fmt(s.cpu_percent, 0, ' %'));
  setText('kv-temp', isNum(s.chip_temperature_c) ? s.chip_temperature_c.toFixed(1) + ' °C' : 'n/a');

  const failures = $('failures-kv');
  failures.replaceChildren();
  const f = t && t.failures;
  if (f && f.frames) {
    setText('failures-window', '(last ' + Math.round(f.window_s / 60) + ' min: ' + f.failed + ' of ' + f.frames
      + ' frames, ' + fmt(f.failed_percent, 0, '%') + ')');
    if (!f.by_reason.length) failures.append(Object.assign(document.createElement('span'), { textContent: 'no failures' }), document.createElement('span'));
    for (const r of f.by_reason) {
      const name = document.createElement('span');
      name.textContent = r.reason;
      const value = document.createElement('span');
      value.textContent = r.percent_of_frames.toFixed(r.percent_of_frames < 10 ? 1 : 0) + '% · ' + r.count;
      value.title = r.count + ' frames: ' + r.percent_of_frames.toFixed(1) + '% of all frames, '
        + r.percent_of_failures.toFixed(1) + '% of the failures';
      failures.append(name, value);
    }
  } else {
    setText('failures-window', '(last 5 min)');
    failures.append(Object.assign(document.createElement('span'), { textContent: 'no data' }), document.createElement('span'));
  }

  renderLink(t && t.link);
  renderKv('transport-kv', transportRows(s.depth_transport, s.viewers));
  renderKv('images-kv', imageStreamRows(s.image_streams));

  const out = (t && t.output) || {};
  setText('out-mode', out.mode || (s.log_only ? 'log-only' : 'sending'));
  setText('out-sent', isNum(out.messages_sent) ? String(out.messages_sent) : '–');
  setText('out-rate', isNum(out.rate_hz) ? out.rate_hz.toFixed(1) + ' Hz' : '–');
  setText('out-error', out.last_error || '–');
  const button = $('btn-log-only');
  button.textContent = 'Log-only: ' + (s.log_only ? 'ON' : 'OFF');
  button.disabled = !!s.log_only_forced_by_env;
  button.title = s.log_only_forced_by_env ? 'Forced on by OAKD_LOG_ONLY on the container'
    : 'Saved in the configuration; restarts VIO';

  const restarts = s.restart_count ? ' · ' + s.restart_count + ' automatic restart(s), last exit code ' + s.last_exit_code : '';
  setText('process-status', s.running ? 'VIO Running (PID: ' + s.pid + ')' + restarts : 'VIO Stopped' + restarts);
  renderDepthStream(s);
}

function renderKv(id, rows) {
  const kv = $(id);
  kv.replaceChildren();
  for (const [label, value] of rows) {
    kv.append(Object.assign(document.createElement('span'), { textContent: label }),
              Object.assign(document.createElement('span'), { textContent: value }));
  }
}

// Camera link box: link_panel.js formats telemetry.link
function renderLink(link) {
  renderKv('link-kv', linkSummaryRows(link));
  const rows = linkStreamRows(link);
  $('link-streams').hidden = !rows.length;
  $('link-streams-body').replaceChildren(...rows.map(r => {
    const tr = document.createElement('tr');
    tr.title = r.description;
    const name = Object.assign(document.createElement('td'), { textContent: r.name });
    const bar = Object.assign(document.createElement('div'), { className: 'link-bar' });
    bar.style.width = (100 * r.fraction).toFixed(1) + '%';
    name.append(bar);
    tr.append(name, ...[r.mbps, r.rate, r.size].map(text => Object.assign(document.createElement('td'), { textContent: text })));
    return tr;
  }));
}

async function updateStatus() {
  await once('status', async () => {
    try {
      const r = await fetchJSON('api/status');
      if (!r.ok) throw new Error(errorText(r.data, r.status));
      ui.status = r.data;
      renderStatus(r.data);
    } catch (e) {
      pill('pill-vio', 'web server not reachable', 'r');
    }
  });
}

// --- history plots ------------------------------------------------------------------------------

async function updateHistory() {
  await once('history', async () => {
    const seconds = Number($('hist-window').value);
    const key = 'seconds=' + seconds + '&tail=' + TRAIL_S;
    const headers = ui.historyEtag && ui.historyKey === key ? { 'If-None-Match': ui.historyEtag } : {};
    let res;
    try { res = await fetch('api/history?' + key, { headers }); } catch (e) { return; }
    if (res.status === 304) return;
    if (!res.ok) return;
    ui.historyEtag = res.headers.get('ETag');
    ui.historyKey = key;
    ui.history = await res.json();
    renderHistory();
  });
}

function renderHistory() {
  const h = ui.history;
  if (!h) return;
  const empty = !h.points ? (h.vio_running ? 'waiting for data…' : 'no data (VIO stopped)') : null;
  setText('traj-window', h.seconds + ' s');
  setText('min-inliers', String(h.min_inliers));
  renderTrajectory(h, empty);
  renderVelocity(h);
  renderTimeSeries(h, empty);
  renderInliers(h, empty);
}

function renderTrajectory(h, empty) {
  const node = $('plot-trajectory');
  const c = h.columns;
  const idx = c.x.map((v, i) => i).filter(i => isNum(c.x[i]) && isNum(c.y[i]));
  if (empty || idx.length < 2) return emptyPlot(node, 300, 300, empty || 'no valid motion in this window');
  node.replaceChildren();
  const x0 = c.x[idx[0]], y0 = c.y[idx[0]];
  const xs = idx.map(i => c.x[i] - x0), ys = idx.map(i => c.y[i] - y0);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const span = Math.max(maxX - minX, maxY - minY, 0.2) * 1.2;
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, scale = 260 / span;
  const px = y => 150 + (y - cy) * scale, py = x => 150 - (x - cx) * scale;
  const step = niceStep(span / 4);
  for (let g = Math.ceil((cy - span / 2) / step) * step; g <= cy + span / 2; g += step) {
    svg('line', { x1: px(g), x2: px(g), y1: 10, y2: 290, stroke: COLORS.grid, 'stroke-width': Math.abs(g) < 1e-9 ? 1.2 : 0.6 }, node);
  }
  for (let g = Math.ceil((cx - span / 2) / step) * step; g <= cx + span / 2; g += step) {
    svg('line', { x1: 10, x2: 290, y1: py(g), y2: py(g), stroke: COLORS.grid, 'stroke-width': Math.abs(g) < 1e-9 ? 1.2 : 0.6 }, node);
  }
  svg('path', { d: pathData(ys, xs, px, py), fill: 'none', stroke: '#3498db', 'stroke-width': 2, 'stroke-linejoin': 'round' }, node);
  svg('circle', { cx: px(0), cy: py(0), r: 4, fill: 'none', stroke: COLORS.text, 'stroke-width': 1.5 }, node);
  const last = idx[idx.length - 1], yaw = c.yaw[last] || 0;
  const tipX = px(ys[ys.length - 1]), tipY = py(xs[xs.length - 1]);
  const fx = Math.sin(yaw), fy = -Math.cos(yaw);
  const points = [[tipX + fx * 10, tipY + fy * 10], [tipX - fx * 5 - fy * 6, tipY - fy * 5 + fx * 6],
                  [tipX - fx * 5 + fy * 6, tipY - fy * 5 - fx * 6]];
  svg('polygon', { points: points.map(p => p.map(v => v.toFixed(1)).join(',')).join(' '), fill: '#dde3ea' }, node);
  svg('line', { x1: 14, x2: 14 + step * scale, y1: 286, y2: 286, stroke: '#dde3ea', 'stroke-width': 2 }, node);
  svgText(node, 14, 280, step < 1 ? (step * 100).toFixed(0) + ' cm' : step + ' m');
  svgText(node, 150, 22, '+x fwd', { 'text-anchor': 'middle' });
  svgText(node, 286, 154, '+y', { 'text-anchor': 'end' });
}

function currentVelocity(h) {
  const tail = h.tail && h.tail.t.length ? h.tail : h.columns;
  const n = tail.t.length;
  if (!n) return null;
  const lastOk = tail.ok[n - 1] === 1;
  for (let i = n - 1; i >= 0; i--) {
    if (isNum(tail.vx[i]) && isNum(tail.vy[i])) {
      return { vx: tail.vx[i], vy: tail.vy[i], vz: tail.vz[i], t: tail.t[i], inliers: tail.inliers[i],
               fresh: lastOk && tail.t[n - 1] - tail.t[i] < 1.0 && tail.t[n - 1] > -2.0, tail };
    }
  }
  return { fresh: false, tail };
}

function renderVelocity(h) {
  const dial = $('plot-dial'), bar = $('plot-vbar');
  const scale = Number($('vel-scale').value);
  dial.replaceChildren();
  bar.replaceChildren();
  const C = { x: 120, y: 112 }, R = 88;
  svg('circle', { cx: C.x, cy: C.y, r: R, fill: 'none', stroke: COLORS.ring, 'stroke-width': 14 }, dial);
  svg('circle', { cx: C.x, cy: C.y, r: R / 2, fill: 'none', stroke: COLORS.ring, 'stroke-dasharray': '3 4' }, dial);
  svg('line', { x1: C.x - R + 8, x2: C.x + R - 8, y1: C.y, y2: C.y, stroke: COLORS.grid, 'stroke-width': 0.6 }, dial);
  svg('line', { x1: C.x, x2: C.x, y1: C.y - R + 8, y2: C.y + R - 8, stroke: COLORS.grid, 'stroke-width': 0.6 }, dial);
  svg('circle', { cx: C.x, cy: C.y - R, r: 4, fill: '#7fd1a8' }, dial);
  svg('polygon', { points: `${C.x - 9},${C.y + R + 22} ${C.x + 9},${C.y + R + 22} ${C.x + 6},${C.y + R + 8} ${C.x - 6},${C.y + R + 8}`, fill: COLORS.vehicle }, dial);
  svgText(dial, C.x + 6, C.y - R + 22, '+x fwd', { 'font-size': 10 });
  svgText(dial, C.x + R - 26, C.y - 4, '+y', { 'font-size': 10 });
  svgText(dial, 4, 246, 'ring = ' + scale + ' m/s', { 'font-size': 10 });
  svgText(dial, C.x + 4, C.y - R / 2 + 12, (scale / 2) + '', { 'font-size': 9 });

  const top = 22, bottom = 222, mid = (top + bottom) / 2, half = (bottom - top) / 2;
  svg('rect', { x: 18, y: top, width: 34, height: bottom - top, fill: 'none', stroke: COLORS.ring, 'stroke-width': 8 }, bar);
  svg('line', { x1: 22, x2: 48, y1: mid, y2: mid, stroke: COLORS.grid }, bar);
  svgText(bar, 35, 14, 'vz ↓+', { 'font-size': 10, 'text-anchor': 'middle' });
  svgText(bar, 35, 246, '±' + scale, { 'font-size': 10, 'text-anchor': 'middle' });

  const v = h.points ? currentVelocity(h) : null;
  const k = R / scale;
  if (v && v.tail) {
    const tail = v.tail, n = tail.t.length, tEnd = n ? tail.t[n - 1] : 0;
    let prev = null;
    for (let i = 0; i < n; i++) {
      if (tail.t[i] < tEnd - TRAIL_S || !isNum(tail.vx[i]) || !isNum(tail.vy[i])) { prev = null; continue; }
      const p = clampToRing(tail.vy[i] * k, -tail.vx[i] * k, R);
      if (prev) {
        const age = (tEnd - tail.t[i]) / TRAIL_S;
        svg('line', { x1: C.x + prev[0], y1: C.y + prev[1], x2: C.x + p[0], y2: C.y + p[1], stroke: COLORS.vx,
                      'stroke-opacity': (0.75 * (1 - age)).toFixed(2), 'stroke-width': 2, 'stroke-linecap': 'round' }, dial);
      }
      prev = p;
    }
  }
  if (v && isNum(v.vx)) {
    const raw = [v.vy * k, -v.vx * k];
    const tip = clampToRing(raw[0], raw[1], R);
    const clipped = Math.hypot(raw[0], raw[1]) > R;
    const color = !v.fresh ? '#5d6b80' : clipped ? '#f1c40f' : COLORS.vx;
    arrow(dial, C.x, C.y, C.x + tip[0], C.y + tip[1], color, 4);
    if (isNum(v.vz)) {
      const len = Math.max(-half, Math.min(half, v.vz / scale * half));
      if (Math.abs(len) > 1) arrow(bar, 35, mid, 35, mid + len, color, 4);
    }
  } else {
    svgText(dial, C.x, C.y + 30, 'no valid velocity', { 'text-anchor': 'middle', 'font-size': 11 });
  }
  svg('circle', { cx: C.x, cy: C.y, r: 3, fill: '#dde3ea' }, dial);

  const fresh = v && isNum(v.vx) && v.fresh;
  document.querySelector('.velocity-kv').classList.toggle('stale', !fresh);
  setText('vel-vx', v && isNum(v.vx) ? signed(v.vx, 2, ' m/s') : '–');
  setText('vel-vy', v && isNum(v.vy) ? signed(v.vy, 2, ' m/s') : '–');
  setText('vel-vz', v && isNum(v.vz) ? signed(v.vz, 2, ' m/s') : '–');
  const speed = v && isNum(v.vx) ? Math.hypot(v.vx, v.vy) : null;
  setText('vel-speed', isNum(speed) ? speed.toFixed(2) + ' m/s (xy)' : '–');
  if (isNum(speed) && speed >= 0.01) {
    const deg = Math.atan2(v.vy, v.vx) * 180 / Math.PI;
    setText('vel-dir', Math.abs(deg).toFixed(0) + '° ' + (Math.abs(deg) < 0.5 ? 'ahead' : deg > 0 ? 'stbd' : 'port'));
  } else {
    setText('vel-dir', '–');
  }
  const valid = $('vel-valid');
  const last = ui.status && ui.status.telemetry && ui.status.telemetry.last_frame;
  if (fresh) {
    valid.textContent = '✔ ' + (isNum(v.inliers) ? v.inliers + ' inl.' : '');
    valid.className = 'ok';
  } else {
    valid.textContent = '✖ ' + (last && last.reason ? last.reason : 'no recent valid frame');
    valid.className = 'bad';
  }
}

function clampToRing(x, y, R) {
  const m = Math.hypot(x, y);
  return m > R ? [x * R / m, y * R / m] : [x, y];
}

function arrow(parent, x1, y1, x2, y2, color, width) {
  const dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy);
  if (len < 1) return;
  const ux = dx / len, uy = dy / len, head = Math.min(12, len * 0.6);
  const bx = x2 - ux * head, by = y2 - uy * head;
  svg('line', { x1, y1, x2: bx, y2: by, stroke: color, 'stroke-width': width, 'stroke-linecap': 'round' }, parent);
  svg('polygon', { points: `${x2},${y2} ${bx - uy * head * 0.5},${by + ux * head * 0.5} ${bx + uy * head * 0.5},${by - ux * head * 0.5}`, fill: color }, parent);
}

function timeAxis(node, W, left, right, bottom, top) {
  const step = niceStep(W / 6);
  for (let s = 0; s <= W + 1e-9; s += step) {
    const x = right - s / W * (right - left);
    svg('line', { x1: x, x2: x, y1: top, y2: bottom, stroke: COLORS.grid, 'stroke-width': 0.5 }, node);
    svgText(node, x, bottom + 14, s ? '-' + (s >= 60 && s % 60 === 0 ? s / 60 + ' min' : s + ' s') : 'now',
            { 'text-anchor': 'middle', 'font-size': 10 });
  }
}

function renderTimeSeries(h, empty) {
  const node = $('plot-vxvy');
  const dvl = h.dvl || {};
  const hint = $('dvl-hint');
  if (dvl.available && dvl.source) {
    hint.textContent = 'DVL overlay (dashed): ' + dvl.message + ' from system ' + dvl.source.system_id + ', component '
      + dvl.source.component_id + (dvl.hint ? ' · ' + dvl.hint : '');
  } else {
    hint.textContent = dvl.hint || 'DVL not found';
  }
  if (empty) return emptyPlot(node, 600, 220, empty);
  node.replaceChildren();
  const left = 46, right = 592, top = 22, bottom = 196, W = h.seconds;
  const c = h.columns;
  const values = [...c.vx, ...c.vy, ...(dvl.available ? [...dvl.vx, ...dvl.vy] : [])].filter(isNum).map(Math.abs);
  const ymax = niceCeil(Math.max(0.1, ...values));
  const sx = t => right + t / W * (right - left), sy = v => (top + bottom) / 2 - v / ymax * (bottom - top) / 2;
  timeAxis(node, W, left, right, bottom, top);
  for (const v of [ymax, ymax / 2, 0, -ymax / 2, -ymax]) {
    svg('line', { x1: left, x2: right, y1: sy(v), y2: sy(v), stroke: v === 0 ? COLORS.axis : COLORS.grid,
                  'stroke-width': v === 0 ? 1.2 : 0.5, 'stroke-dasharray': v === 0 ? '' : '3 4' }, node);
    svgText(node, left - 4, sy(v) + 4, signed(v, ymax < 0.5 ? 2 : 1), { 'text-anchor': 'end', 'font-size': 10 });
  }
  svg('path', { d: pathData(c.t, c.vx, sx, sy), fill: 'none', stroke: COLORS.vx, 'stroke-width': 1.8 }, node);
  svg('path', { d: pathData(c.t, c.vy, sx, sy), fill: 'none', stroke: COLORS.vy, 'stroke-width': 1.8 }, node);
  if (dvl.available) {
    for (const [key, color] of [['vx', COLORS.dvlx], ['vy', COLORS.dvly]]) {
      svg('path', { d: pathData(dvl.t, dvl[key], sx, sy), fill: 'none', stroke: color, 'stroke-width': 1.6,
                    'stroke-dasharray': '5 3' }, node);
    }
  }
  svgText(node, left + 4, 14, 'vx (fwd)', { fill: COLORS.vx });
  svgText(node, left + 64, 14, 'vy (right)', { fill: COLORS.vy });
  if (dvl.available) {
    svgText(node, left + 134, 14, '- - DVL vx', { fill: COLORS.dvlx });
    svgText(node, left + 204, 14, '- - DVL vy', { fill: COLORS.dvly });
  }
  svgText(node, right, 14, '±' + ymax + ' m/s, ' + W + ' s', { 'text-anchor': 'end', 'font-size': 10 });
}

function renderInliers(h, empty) {
  const node = $('plot-inliers');
  if (empty) return emptyPlot(node, 600, 220, empty);
  node.replaceChildren();
  const left = 46, right = 592, top = 22, bottom = 196, W = h.seconds, c = h.columns;
  const ymax = niceCeil(Math.max(h.min_inliers * 2, ...c.inliers.filter(isNum)) * 1.05);
  const sx = t => right + t / W * (right - left), sy = v => bottom - v / ymax * (bottom - top);
  timeAxis(node, W, left, right, bottom, top);
  for (const v of [0, ymax / 2, ymax]) {
    svg('line', { x1: left, x2: right, y1: sy(v), y2: sy(v), stroke: COLORS.grid, 'stroke-width': 0.5 }, node);
    svgText(node, left - 4, sy(v) + 4, String(Math.round(v)), { 'text-anchor': 'end', 'font-size': 10 });
  }
  let ticks = '';
  c.ok.forEach((ok, i) => { if (ok === 0) ticks += 'M' + sx(c.t[i]).toFixed(1) + ',' + bottom + 'v-7'; });
  if (ticks) svg('path', { d: ticks, stroke: COLORS.bad, 'stroke-width': 1, 'stroke-opacity': 0.7 }, node);
  svg('path', { d: pathData(c.t, c.inliers, sx, sy), fill: 'none', stroke: COLORS.inl, 'stroke-width': 1.6 }, node);
  svg('line', { x1: left, x2: right, y1: sy(h.min_inliers), y2: sy(h.min_inliers), stroke: COLORS.bad, 'stroke-dasharray': '5 4' }, node);
  svgText(node, right, sy(h.min_inliers) - 4, 'min_inliers ' + h.min_inliers, { 'text-anchor': 'end', fill: COLORS.bad, 'font-size': 10 });
  svgText(node, left + 4, 14, 'inliers', { fill: COLORS.inl });
}

// --- live views and depth probe -----------------------------------------------------------------

function imageSize() {
  const camera = ui.status && ui.status.camera;
  if (camera && camera.image_size) return camera.image_size;
  const form = $('config-form');
  return [Number(form.elements.resolution_width.value) || 320, Number(form.elements.resolution_height.value) || 200];
}

function probeParams(names) {
  const params = new URLSearchParams();
  for (const name of names) {
    const value = $('probe-' + name).value;
    if (value !== '') params.set(name, value);
  }
  return params;
}

async function loadView(which) {
  const figure = $(which === 'left' ? 'view-left' : 'view-right');
  const caption = figure.querySelector('figcaption');
  const img = figure.querySelector('img');
  const stream = which === 'left' ? $('view-left-stream').value : 'depth';
  let query = '';
  if (stream === 'rect_left' && $('view-overlay').checked) query += '&overlay=1';
  if ($('view-crosshair').checked && stream !== 'rgb') query += '&crosshair=1&' + probeParams(['x', 'y', 'w', 'h']);
  if (!img.getAttribute('src')) caption.textContent = 'requesting ' + stream + '…';
  let res;
  try {
    res = await fetch('api/snapshot?stream=' + stream + query);
  } catch (e) {
    caption.textContent = stream + ': web server not reachable';
    return;
  }
  if (!res.ok) {
    const error = await res.json().catch(() => ({}));
    caption.textContent = stream + ': ' + (error.detail || ('Error ' + res.status));
    return;
  }
  if (img.src) URL.revokeObjectURL(img.src);
  img.src = URL.createObjectURL(await res.blob());
  figure.classList.remove('empty');
  let text = stream + ' at ' + new Date().toLocaleTimeString() + ' · seq ' + res.headers.get('X-Snapshot-Sequence');
  if (res.headers.get('X-Overlay') === 'features') {
    text += ' · ' + res.headers.get('X-Overlay-Inliers') + ' inliers, ' + res.headers.get('X-Overlay-Outliers')
      + ' outliers, ' + res.headers.get('X-Overlay-Unmatched') + ' unmatched (frame ' + res.headers.get('X-Overlay-Frame-Sequence') + ')';
  } else if (res.headers.get('X-Overlay') === 'unavailable') {
    text += ' · no feature overlay yet';
  }
  caption.textContent = text;
}

async function refreshViews(manual) {
  await once('views', async () => {
    await loadView('left');
    if ($('view-depth').checked) await loadView('right');
  });
  if (manual && !$('view-depth').checked) $('view-right').querySelector('figcaption').textContent = 'depth view off';
}

function aimAt(event) {
  const img = event.currentTarget;
  if (!img.clientWidth) return;
  const [width, height] = imageSize();
  $('probe-x').value = Math.round(event.offsetX / img.clientWidth * width);
  $('probe-y').value = Math.round(event.offsetY / img.clientHeight * height);
  $('view-crosshair').checked = true;
  refreshViews(true);
}

async function showProbeResult(url, busy, summarize) {
  const summary = $('probe-summary');
  const result = $('probe-result');
  summary.textContent = busy;
  let r;
  try { r = await fetchJSON(url, { method: 'POST' }); } catch (e) { summary.textContent = 'web server not reachable'; return; }
  if (!r.ok) {
    summary.textContent = errorText(r.data, r.status);
    result.textContent = '';
    return;
  }
  summary.textContent = summarize(r.data);
  result.textContent = JSON.stringify(r.data, null, 2);
}

function depthProbe() {
  return showProbeResult('api/depth_probe?' + probeParams(['x', 'y', 'w', 'h', 'frames']),
    'Probing depth...', data => '→ ' + data.summary + (data.clamped && data.clamped.length ? ' (clamped: ' + data.clamped.join(', ') + ')' : ''));
}

function focalCheck() {
  return showProbeResult('api/focal_check', 'Taking rect_left, rect_right and depth snapshots (keep the camera still)...',
    data => (data.ok ? 'OK: ' : 'CHECK: ') + data.message);
}

// --- live stream (camera image + features, or depth) --------------------------------------------
// The stream's gray (depth_stream.py, d.encoding): "gray8-mono-rectified-left" (mono_features, the default)
// is the rectified left camera image the features are tracked on, shown as it is, with the tracked features
// (api/stream/features, drawn by features_overlay.js) on top. "gray8-depth-linear" is linear in depth,
// 0 = no depth, gray_far (max_mm) .. gray_near (min_mm), redrawn by depth on a canvas with a 256-entry lookup
// table: jet (min_mm red .. max_mm blue), or with colorize off gray linear in depth (near bright), so the
// contrast is stretched; the JPEG itself is unchanged and the legend shows the scale drawn.

const MONO_ENCODING = 'gray8-mono-rectified-left';
const FEATURES_URL = 'api/stream/features';
const FEATURES_STALE_MS = 2000;
const depthStream = { on: false, retry: null, timer: null, lut: null, lutKey: '', scale: null, events: null,
                      features: null, featuresAt: 0 };

function jet(x) {
  const c = v => Math.round(255 * Math.max(0, Math.min(1, v)));
  return [c(1.5 - Math.abs(4 * x - 3)), c(1.5 - Math.abs(4 * x - 2)), c(1.5 - Math.abs(4 * x - 1))];
}

// Until the VIO publishes (stale), the configured encoding says what will be shown
const monoConfigured = d => d.stale && ['auto', 'mono_features'].includes(d.configured_encoding);

function streamScale(d) {
  const range = (d && d.depth_range_mm) || [300, 4000];
  if (d && (d.stale ? monoConfigured(d) : d.encoding === MONO_ENCODING)) return { kind: 'mono', min: range[0], max: range[1] };
  return { kind: 'depth', min: range[0], max: range[1], far: (d && d.gray_far) || 16, near: (d && d.gray_near) || 255,
           invalid: (d && d.gray_invalid_below) || 8 };
}

const isMono = scale => scale.kind === 'mono';
const colorOn = () => $('depth-stream-color').checked;

// Depth (mm) of a gray value on the scale, NaN for no depth
function grayDepth(scale, g) {
  if (g < scale.invalid) return NaN;
  return scale.max - (scale.max - scale.min) * (g - scale.far) / (scale.near - scale.far);
}

// Colour drawn for a depth (mm): jet, or gray linear in depth (near bright) with colorize off; black = none
function depthColor(scale, mm, color) {
  if (Number.isNaN(mm)) return [0, 0, 0];
  const x = (scale.max - mm) / (scale.max - scale.min);
  if (color) return jet(x);
  const g = Math.round(16 + 239 * Math.max(0, Math.min(1, x)));
  return [g, g, g];
}

function depthLut(scale, color) {
  const key = [scale.kind, scale.far, scale.near, scale.invalid, scale.min, scale.max, color].join(',');
  if (depthStream.lutKey === key) return depthStream.lut;
  const lut = new Uint8ClampedArray(256 * 3);
  for (let g = 0; g < 256; g++) lut.set(depthColor(scale, grayDepth(scale, g), color), g * 3);
  depthStream.lut = lut;
  depthStream.lutKey = key;
  return lut;
}

function drawDepthLegend(scale, color) {
  const canvas = $('depth-legend');
  const ctx = canvas.getContext('2d');
  const image = ctx.createImageData(canvas.width, 1);
  for (let i = 0; i < canvas.width; i++) {
    image.data.set(depthColor(scale, scale.min + (scale.max - scale.min) * i / (canvas.width - 1), color), i * 4);
    image.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(image, 0, 0);
  const m = mm => (mm / 1000).toFixed(2) + ' m';
  setText('legend-near', m(scale.min) + ' near');
  setText('legend-mid', m((scale.min + scale.max) / 2));
  setText('legend-far', m(scale.max) + ' far');
}

// "16 mm steps, 240 gray levels"
function quantizationText(d, scale) {
  if (isMono(scale)) return 'camera image (8-bit gray), not depth';
  return isNum(d.quantization_mm) ? d.quantization_mm + ' mm steps, ' + (d.gray_near - d.gray_far + 1) + ' gray levels' : '–';
}

const STREAM_URL = 'api/stream/depth';
const BLANK_GIF = 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
const depthStreaming = () => ($('depth-stream-img').getAttribute('src') || '').startsWith(STREAM_URL);

// The depth kinds are always redrawn on the canvas (by depth, see above); the camera image is shown as it is
function colorizeDepth() {
  const img = $('depth-stream-img'), canvas = $('depth-stream-canvas');
  const scale = depthStream.scale || streamScale(null);
  const draw = !isMono(scale) && depthStreaming();
  canvas.hidden = !draw;
  img.classList.toggle('under', draw);
  drawStreamFeatures();
  if (!draw || !img.naturalWidth) return;
  if (canvas.width !== img.naturalWidth || canvas.height !== img.naturalHeight) {
    canvas.width = img.naturalWidth;
    canvas.height = img.naturalHeight;
  }
  const ctx = canvas.getContext('2d', { willReadFrequently: true });
  try { ctx.drawImage(img, 0, 0); } catch (e) { return; }
  const frame = ctx.getImageData(0, 0, canvas.width, canvas.height), px = frame.data;
  const lut = depthLut(scale, colorOn());
  for (let i = 0; i < px.length; i += 4) {
    const k = px[i] * 3;
    px[i] = lut[k]; px[i + 1] = lut[k + 1]; px[i + 2] = lut[k + 2];
  }
  ctx.putImageData(frame, 0, 0);
}

// --- tracked features on the live image (only while streaming with "features" ticked) ---

const featuresFresh = () => depthStream.features && performance.now() - depthStream.featuresAt < FEATURES_STALE_MS;

function drawStreamFeatures() {
  const canvas = $('depth-stream-overlay'), frame = canvas.parentElement;
  const dpr = window.devicePixelRatio || 1;
  const width = frame.clientWidth, height = frame.clientHeight;
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const img = $('depth-stream-img'), f = depthStream.features;
  if (!depthStreaming() || !img.naturalWidth || img.naturalWidth < 2 || !featuresFresh()) return;
  // The shown JPEG's own size: the overlay maps the points onto it (features_overlay.js)
  drawFeatures(ctx, f, featureContainRect([img.naturalWidth, img.naturalHeight], width, height));
}

function renderFeatureCounts() {
  const f = featuresFresh() ? depthStream.features : null;
  for (const state of FEATURE_STATES) {
    setText('ds-n-' + state, f && f.counts && isNum(f.counts[state]) ? f.counts[state] : '–');
  }
  setText('ds-features', !$('depth-stream-features').checked ? 'off' : !depthStream.events ? '–'
    : !f ? 'waiting…' : f.points.length + ' tracked · frame ' + f.sequence
      + (f.image_sequence == null ? ' (latest)' : f.synced ? ' (same frame as the image)' : ' (image frame ' + f.image_sequence + ')'));
}

function startFeatureEvents() {
  if (depthStream.events || !$('depth-stream-features').checked || !depthStream.on || !visible()) return;
  const es = new EventSource(FEATURES_URL);
  depthStream.events = es;
  es.addEventListener('features', e => {
    try { depthStream.features = JSON.parse(e.data); } catch (err) { return; }
    depthStream.featuresAt = performance.now();
    renderFeatureCounts();
    drawStreamFeatures();
  });
  es.onerror = () => { if (es.readyState === EventSource.CLOSED) stopFeatureEvents(); };  // the next status retries
}

function stopFeatureEvents() {
  if (depthStream.events) depthStream.events.close();
  depthStream.events = null;
  depthStream.features = null;
  renderFeatureCounts();
  drawStreamFeatures();
}

function startDepthStream() {
  clearTimeout(depthStream.retry);
  depthStream.retry = null;
  if (!depthStream.on || !visible() || depthStreaming()) return;
  $('depth-stream-img').src = STREAM_URL + '?t=' + Date.now();
  setText('depth-stream-state', 'connecting…');
  const fps = (ui.status && ui.status.depth_stream && ui.status.depth_stream.fps_target) || 5;
  clearInterval(depthStream.timer);
  depthStream.timer = setInterval(colorizeDepth, Math.max(60, Math.min(500, 500 / fps)));
  startFeatureEvents();
}

function stopDepthStream(state) {
  clearTimeout(depthStream.retry);
  depthStream.retry = null;
  clearInterval(depthStream.timer);
  const img = $('depth-stream-img');
  if (img.getAttribute('src')) {
    img.src = BLANK_GIF;  // aborts the multipart request, so the camera stops streaming
    img.removeAttribute('src');
  }
  stopFeatureEvents();
  colorizeDepth();
  $('depth-stream-crosshair').hidden = true;
  $('depth-stream-view').classList.add('empty');
  setText('depth-stream-state', state || 'off');
}

function depthStreamError() {
  if (!depthStream.on || !depthStreaming()) return;
  stopDepthStream('not available, retrying…');
  depthStream.retry = setTimeout(startDepthStream, 3000);
}

function toggleDepthStream() {
  depthStream.on = $('depth-stream-on').checked;
  if (depthStream.on) startDepthStream();
  else stopDepthStream();
  if (ui.status) renderDepthStream(ui.status);
}

function toggleStreamFeatures() {
  if ($('depth-stream-features').checked && depthStreaming()) startFeatureEvents();
  else stopFeatureEvents();
}

function toggleStreamColor() {
  renderDepthStream.legend = null;
  if (ui.status) renderDepthStream(ui.status);
  colorizeDepth();
}

function depthStreamProblem(s) {
  const d = s.depth_stream || {};
  if (!d.enabled) return 'the live stream is off in the settings (Live depth stream)';
  if (!s.running) return 'VIO stopped';
  if (d.stale) return 'waiting for the VIO process…';
  return null;
}

function depthEncodingText(d) {
  if (d.stale || !d.variant) return d.configured_encoding ? d.configured_encoding + ' (configured)' : '–';
  const parts = [d.variant + (d.encoded_on === 'pi' ? ' · fallback: raw depth over the link, encoded on the Pi' : ' · encoded on the camera')];
  if (d.requested_variant && d.requested_variant !== d.variant) parts.push('requested ' + d.requested_variant);
  if (d.variant_reason && d.requested_variant !== 'auto') parts.push(d.variant_reason);
  if (d.fallback) parts.push('fallback: ' + d.fallback.reason + (d.fallback.at_start ? '' : ' (from ' + d.fallback.from + ')'));
  if (d.encoding === MONO_ENCODING) parts.push('gray = the camera image, not depth');
  const host = d.host_encoder;
  if (d.encoded_on === 'pi' && host) {
    parts.push('Pi CPU ' + fmt(host.cpu_percent, 1) + ' % of a core (cap ' + fmt(host.cpu_budget_percent, 0) + ' %)'
      + (host.skipped_cpu ? ', ' + host.skipped_cpu + ' frames skipped for the cap' : ''));
  }
  if (d.size_problem) parts.push(d.size_problem);
  if (d.debug) parts.push('camera debug log ' + (d.debug_state && d.debug_state.active ? 'on' : 'on while streaming'));
  return parts.join(' · ');
}

function depthErrorsText(d, now) {
  // The stall flag clears when the stream stops; the last error stays, so it is shown either way
  if (d.stale) return '–';
  const error = d.last_error;
  const age = error && isNum(error.unix_time) && isNum(now) ? ' (' + Math.max(0, now - error.unix_time).toFixed(0) + ' s ago)' : '';
  if (d.encoder_stalled) return 'encoder outputs nothing: ' + (error ? error.text + age : 'frames go in, no JPEG comes back');
  if (d.blank) return d.blank.text;
  if (d.errors) return d.errors + (error ? ' · ' + (error.fatal ? 'stopped: ' : '') + error.text + age : '');
  return error ? 'none now · last: ' + error.text + age : 'none';
}

function renderDeviceLog(d, now) {
  const log = !d.stale && d.device_log;
  const lines = log ? log.stream.concat(log.other).sort((a, b) => a.unix_time - b.unix_time) : [];
  $('ds-log-box').hidden = !lines.length && !d.debug;
  setText('ds-log-summary', 'camera log (' + (log ? log.stream.length + ' stream, ' + log.other.length + ' other lines'
    : 'none') + (d.debug ? ', debug level while streaming' : '') + ')');
  setText('ds-log', lines.length ? lines.map(l => (isNum(now) ? '-' + Math.max(0, now - l.unix_time).toFixed(0) + ' s ' : '')
    + l.level + ' ' + l.node + ': ' + l.text).join('\n') : 'no camera log lines yet');
}

function renderDepthStream(s) {
  const d = s.depth_stream || {};
  depthStream.scale = streamScale(d);
  const mono = isMono(depthStream.scale);
  // The title, legend and hint follow what is shown: the camera image (mono_features) or depth
  setText('depth-stream-title', mono ? 'Live camera' : 'Live depth');
  setText('depth-stream-subtitle', mono
    ? '(the rectified left camera image the features are tracked on, with the tracked features, MJPEG-encoded on the camera; only sent while streaming)'
    : '(stereo depth as MJPEG, encoded on the ' + (d.encoded_on === 'pi' ? 'Pi' : 'camera') + ', with the tracked features; only sent while streaming)');
  $('depth-legend-box').hidden = mono;
  $('depth-stream-color-label').hidden = mono;
  setText('feature-legend-title', mono ? 'camera image (rectified left), not depth; the tracked features:'
    : 'the tracked features:');
  setText('depth-stream-hint', mono
    ? 'The camera encodes the rectified left image the feature tracker works on; the Pi only forwards the JPEGs, and '
      + 'the browser draws the tracked features on it. Raw = device depth in air, before the refractive index and '
      + 'depth scale. The crosshair marks the centre pixel.'
    : 'The VIO\'s depth (after the stereo filters and the min/max depth thresholds) as 8-bit gray, linear in depth; '
      + 'the browser redraws it by depth. Raw = device depth in air, before the refractive index and depth scale. The crosshair marks the centre pixel.');
  const legendKey = JSON.stringify(depthStream.scale) + colorOn();
  if (!mono && renderDepthStream.legend !== legendKey) {
    drawDepthLegend(depthStream.scale, colorOn());
    renderDepthStream.legend = legendKey;
  }
  const camera = s.camera || {};
  const scaleText = isNum(camera.total_depth_scale) && camera.total_depth_scale !== 1
    ? mm => ' (×' + camera.total_depth_scale.toFixed(3) + ' = ' + (mm * camera.total_depth_scale / 1000).toFixed(3) + ' m in PnP)'
    : () => '';
  const c = d.streaming ? d.center : null;
  setText('ds-center', c && isNum(c.median_mm)
    ? (c.median_mm / 1000).toFixed(3) + ' m' + scaleText(c.median_mm) + ' · ' + c.valid + '/' + c.pixels + ' valid'
    : c ? 'no depth' : '–');
  setText('ds-center-pixel', c ? (c.pixel_mm ? (c.pixel_mm / 1000).toFixed(3) + ' m' : 'no depth') + ' at (' + c.x + ', ' + c.y + ')' : '–');
  setText('ds-range', (depthStream.scale.min / 1000).toFixed(2) + ' – ' + (depthStream.scale.max / 1000).toFixed(2) + ' m');
  setText('ds-quant', quantizationText(d, depthStream.scale));
  setText('ds-rate', d.streaming && isNum(d.fps) ? d.fps.toFixed(1) + ' fps (target ' + fmt(d.fps_target, 1) + ')'
    : isNum(d.fps_target) ? 'target ' + d.fps_target.toFixed(1) + ' fps' : '–');
  setText('ds-kbps', d.streaming && isNum(d.kbps) ? d.kbps.toFixed(0) + ' kbit/s' : '–');
  const last = d.last_frame;
  setText('ds-jpeg', (isNum(d.quality) ? 'quality ' + d.quality : '–') + (d.streaming && last ? ' · ' + (last.bytes / 1024).toFixed(1) + ' KiB/frame' : ''));
  setText('ds-encoding', depthEncodingText(d));
  setText('ds-source', d.stale || !d.source ? '–' : d.source + (d.image_size ? ' · ' + d.image_size.join('×') : ''));
  const cost = d.stale ? null : d.device_stats;
  setText('ds-cost', cost ? (d.encoded_on === 'pi' ? 'Pi encode ' : '') + fmt(cost.convert_ms_mean, 1) + ' ms mean, '
    + fmt(cost.convert_ms_max, 1) + ' ms max' : '–');
  setText('ds-errors', depthErrorsText(d, s.server_time));
  const v = s.viewers;
  setText('ds-viewers', v ? v.text + (d.depth_frames === 'on_demand' && d.encoded_on === 'pi'
    ? ' · full depth frames sent only while watched' : '') : '–');
  renderDeviceLog(d, s.server_time);
  renderFeatureCounts();

  const problem = depthStreamProblem(s);
  const img = $('depth-stream-img');
  const what = mono ? 'camera image' : 'depth';
  if (!depthStream.on) {
    setText('depth-stream-caption', problem ? 'Live stream unavailable: ' + problem
      : 'Tick "stream" to watch what the VIO sees, live (' + what + ' and the tracked features).');
    return;
  }
  const showing = depthStreaming() && img.naturalWidth > 1;
  $('depth-stream-view').classList.toggle('empty', !showing);
  $('depth-stream-crosshair').hidden = !showing;
  if (problem) setText('depth-stream-caption', problem);
  else if (!d.streaming) setText('depth-stream-caption', 'asking the camera to stream…');
  else setText('depth-stream-caption', what + ' ' + (d.image_size ? d.image_size.join('×') : '') + ' · '
    + (d.frames_total || 0) + ' frames this run · ' + new Date().toLocaleTimeString());
  if (showing) setText('depth-stream-state', 'streaming');
  if (!problem && !depthStreaming() && !depthStream.retry) startDepthStream();
  else if (depthStreaming()) startFeatureEvents();
}

// --- process control ----------------------------------------------------------------------------

async function post(url, body, okMessage) {
  const r = await fetchJSON(url, { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                   body: body === undefined ? undefined : JSON.stringify(body) });
  if (r.ok) showToast(okMessage, 'success');
  else showToast(errorText(r.data, r.status), 'error');
  setTimeout(updateStatus, 800);
  return r;
}

function startVIO() { return post('api/start', undefined, 'VIO started'); }
function stopVIO() { return post('api/stop', undefined, 'VIO stopped'); }

function resetVIO() {
  if (!confirm('Restart the VIO process? The PnP reference, trajectory and plots start over.')) return;
  return post('api/start', undefined, 'VIO restarting');
}

async function toggleLogOnly() {
  const on = !(ui.status && ui.status.log_only);
  const what = on ? 'Turn LOG-ONLY on (stop sending MAVLink)' : 'Turn LOG-ONLY off (start sending VISION_POSITION_DELTA to the autopilot)';
  if (!confirm(what + '? This is saved and restarts VIO.')) return;
  const r = await post('api/log_only', { log_only: on }, 'Log-only ' + (on ? 'on' : 'off') + ', VIO restarting');
  if (r.ok) {
    const box = $('config-form').elements.log_only;
    box.checked = on;
    ui.initial.log_only = on;
    markChanged();
  }
}

// --- settings editor ----------------------------------------------------------------------------

const VEHICLE_IP = /^[A-Za-z0-9.\-:\[\]]+$/;

function fieldInput(name) {
  return $('config-form').elements[name];
}

function rawValue(f) {
  const input = fieldInput(f.name);
  if (!input) return undefined;
  return f.type === 'bool' ? input.checked : input.value;
}

function numbers(text) {
  return String(text).trim().split(/[\s,]+/).filter(Boolean).map(Number);
}

// [value to post, error message or '']
function parseField(f) {
  const raw = rawValue(f);
  if (raw === undefined) return [undefined, ''];
  if (f.type === 'bool') return [raw, ''];
  const text = String(raw).trim();
  const range = () => (f.min !== null && f.max !== null) ? ' between ' + f.min + ' and ' + f.max : '';
  if (f.type === 'int' || f.type === 'float') {
    if (text === '') return f.nullable ? [null, ''] : [null, 'required'];
    const value = Number(text);
    if (!isFinite(value)) return [null, 'must be a number'];
    if (f.type === 'int' && !Number.isInteger(value)) return [null, 'must be a whole number'];
    if ((f.min !== null && value < f.min) || (f.max !== null && value > f.max)) return [value, 'must be' + range()];
    return [value, ''];
  }
  if (f.type === 'choice') {
    const numeric = f.choices.every(c => typeof c === 'number');
    const value = numeric ? Number(text) : text;
    return f.choices.some(c => String(c) === text) ? [value, ''] : [value, 'must be one of ' + f.choices.join(', ')];
  }
  if (f.type === 'lever_arm') {
    const values = numbers(text);
    if (values.length !== 3 || !values.every(isFinite)) return [text, 'needs three numbers x,y,z'];
    if (!values.every(v => Math.abs(v) <= f.max)) return [text, 'values must be within ±' + f.max + ' m'];
    return [text, ''];
  }
  if (f.type === 'intrinsics') {
    if (text === '') return [null, ''];
    const values = numbers(text);
    if (values.length !== 4 || !values.every(v => isFinite(v) && v > 0)) return [text, 'needs four positive numbers fx,fy,cx,cy'];
    return [text, ''];
  }
  if (f.name === 'vehicle_ip' && !VEHICLE_IP.test(text)) return [text, 'must be a host name or IP address, optionally with :port'];
  return [text, ''];
}

function showFieldError(name, message) {
  const container = document.querySelector('.field[data-field="' + name + '"]');
  if (!container) return;
  container.querySelector('.error').textContent = message;
  const input = fieldInput(name);
  if (input) input.classList.toggle('invalid', !!message);
}

function crossFieldErrors(config) {
  const errors = {};
  if (isNum(config.stereo_depth_min_mm) && isNum(config.stereo_depth_max_mm)
      && config.stereo_depth_min_mm >= config.stereo_depth_max_mm) {
    errors.stereo_depth_max_mm = 'must be above Min depth (' + config.stereo_depth_min_mm + ')';
  }
  return errors;
}

function collectConfig() {
  const config = {}, errors = {};
  for (const f of ui.schema.fields) {
    const [value, error] = parseField(f);
    if (value === undefined) continue;
    config[f.name] = value;
    if (error) errors[f.name] = error;
  }
  Object.assign(errors, crossFieldErrors(config));
  return { config, errors };
}

function showErrors(errors) {
  for (const f of ui.schema.fields) showFieldError(f.name, errors[f.name] || '');
  const box = $('settings-errors');
  const names = Object.keys(errors);
  box.hidden = !names.length;
  box.textContent = names.map(name => (ui.fields[name] ? ui.fields[name].label : name) + ' (' + name + '): ' + errors[name]).join(' · ');
  for (const name of names) {
    const container = document.querySelector('.field[data-field="' + name + '"]');
    if (container) container.closest('details').open = true;
  }
}

function formValue(f, value) {
  if (f.type === 'bool') return !!value;
  if (value === null || value === undefined) return '';
  if (f.type === 'lever_arm') return Array.isArray(value) ? value.join(',') : String(value);
  return String(value);
}

function setField(f, value) {
  const input = fieldInput(f.name);
  if (!input) return;
  if (f.type === 'bool') input.checked = !!value;
  else input.value = formValue(f, value);
}

// Camera Angle only applies to the custom mount. A disabled input keeps its value, which collectConfig still posts
function syncCameraAngle() {
  const mount = fieldInput('camera_mount'), angle = fieldInput('camera_angle');
  if (mount && angle) angle.disabled = mount.value !== 'custom';
}

function currentKey(f) {
  const raw = rawValue(f);
  return f.type === 'bool' ? !!raw : String(raw).trim();
}

function markChanged() {
  let changed = 0;
  for (const f of ui.schema.fields) {
    const input = fieldInput(f.name);
    if (!input) continue;
    const differs = currentKey(f) !== ui.initial[f.name];
    input.classList.toggle('changed', differs);
    changed += differs;
  }
  setText('settings-state', changed ? changed + ' unsaved change(s) — Apply to save and restart VIO' : 'validated client + server');
}

async function initSettings() {
  const r = await fetchJSON('api/config/schema');
  if (!r.ok) { setText('settings-state', 'schema unavailable: ' + errorText(r.data, r.status)); return; }
  ui.schema = r.data;
  for (const f of ui.schema.fields) {
    ui.fields[f.name] = f;
    const container = document.querySelector('.field[data-field="' + f.name + '"]');
    const input = fieldInput(f.name);
    if (!container || !input) continue;
    const reset = document.createElement('button');
    reset.type = 'button';
    reset.className = 'reset-one';
    reset.textContent = '↺';
    reset.title = 'Reset to default (' + (formValue(f, f.default) === '' ? 'empty' : formValue(f, f.default)) + ')';
    reset.addEventListener('click', () => {
      setField(f, f.default);
      showFieldError(f.name, parseField(f)[1]);
      markChanged();
      if (f.name === 'camera_mount') syncCameraAngle();
    });
    container.appendChild(reset);
    const error = document.createElement('div');
    error.className = 'error';
    container.appendChild(error);
    if (input.type === 'number') {
      if (f.min !== null) input.min = f.min;
      if (f.max !== null) input.max = f.max;
      if (f.nullable) input.placeholder = 'auto';
    }
    input.addEventListener('input', () => { showFieldError(f.name, parseField(f)[1]); markChanged(); });
    input.addEventListener('change', () => { showFieldError(f.name, parseField(f)[1]); markChanged(); });
    if (f.name === 'camera_mount') {
      input.addEventListener('input', syncCameraAngle);
      input.addEventListener('change', syncCameraAngle);
    }
    ui.initial[f.name] = currentKey(f);
  }
  syncCameraAngle();
  markChanged();
}

async function applySettings(event) {
  event.preventDefault();
  if (!ui.schema) return showToast('Settings schema not loaded', 'error');
  const { config, errors } = collectConfig();
  showErrors(errors);
  if (Object.keys(errors).length) return showToast('Fix the highlighted settings first', 'error');
  const r = await fetchJSON('api/config', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                             body: JSON.stringify(config) });
  if (r.ok) {
    showToast('Configuration saved! Restarting VIO...', 'success');
    for (const f of ui.schema.fields) if (fieldInput(f.name)) ui.initial[f.name] = currentKey(f);
    markChanged();
    setTimeout(updateStatus, 1500);
    return;
  }
  if (r.status === 422 && r.data && Array.isArray(r.data.detail)) {
    const serverErrors = {};
    for (const d of r.data.detail) serverErrors[d.field || (d.loc || []).slice(-1)[0]] = d.msg;
    showErrors(serverErrors);
  }
  showToast('Not saved: ' + errorText(r.data, r.status), 'error');
}

async function resetDefaults() {
  if (!confirm('Reset every setting to its default and restart VIO?')) return;
  const r = await fetchJSON('api/reset', { method: 'POST' });
  if (r.ok) {
    showToast('Reset to defaults. Reloading...', 'success');
    setTimeout(() => location.reload(), 1000);
  } else {
    showToast(errorText(r.data, r.status), 'error');
  }
}

// --- ArduPilot parameters -----------------------------------------------------------------------

function paramText(value) {
  if (!isNum(value)) return '–';
  return Number.isInteger(value) ? String(value) : String(Number(value.toPrecision(6)));
}

function armedText(heartbeat) {
  if (!heartbeat) return 'vehicle: ?';
  if (heartbeat.armed === true) return 'vehicle ARMED ✖ (writing refused)';
  if (heartbeat.armed === false) return 'vehicle disarmed ✔';
  return 'vehicle armed state unknown (' + (heartbeat.detail || 'no HEARTBEAT') + ')';
}

async function readAutopilot() {
  const body = $('ap-table').tBodies[0];
  body.innerHTML = '<tr><td colspan="6" class="muted">Reading parameters (PARAM_REQUEST_READ only)…</td></tr>';
  let r;
  try { r = await fetchJSON('api/autopilot/params/read', { method: 'POST' }); } catch (e) { r = { ok: false, status: 0, data: null }; }
  if (!r.ok) {
    body.innerHTML = '';
    const message = $('ap-message');
    message.hidden = false;
    message.textContent = 'Could not read: ' + errorText(r.data, r.status);
    return;
  }
  ui.autopilot = r.data;
  renderAutopilot();
}

function renderAutopilot() {
  const data = ui.autopilot;
  const heartbeat = data.heartbeat || {};
  const message = $('ap-message');
  const unreadable = data.params.every(p => p.status === 'unreadable');
  message.hidden = !unreadable;
  message.textContent = unreadable ? 'No parameters read: ' + (heartbeat.detail || 'no reply') + ' (' + data.mavlink2rest + ')' : '';
  const armed = $('ap-armed');
  armed.textContent = armedText(heartbeat);
  armed.className = heartbeat.armed === false ? 'ok' : 'bad';
  const body = $('ap-table').tBodies[0];
  body.replaceChildren();
  for (const p of data.params) {
    const row = body.insertRow();
    row.className = p.status;
    row.dataset.name = p.name;
    const select = document.createElement('input');
    select.type = 'checkbox';
    select.className = 'ap-select';
    select.checked = p.status === 'differs' && !p.keep_current;
    select.disabled = p.status === 'unreadable';
    select.title = 'Include in "Write selected"';
    row.insertCell().appendChild(select);
    row.insertCell().innerHTML = '<b></b>';
    row.cells[1].firstChild.textContent = p.name;
    row.insertCell().textContent = paramText(p.current);
    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'ap-value';
    input.value = isNum(p.suggested) ? paramText(p.suggested) : '';
    input.title = 'Allowed: ' + (p.allowed ? p.allowed.join(', ') : p.min + ' to ' + p.max) + (p.integer ? ' (whole number)' : '');
    input.addEventListener('input', () => {
      input.classList.toggle('invalid', !!paramError(p, input.value));
      select.checked = !select.disabled && !paramError(p, input.value) && !sameParam(p.current, Number(input.value));
    });
    row.insertCell().appendChild(input);
    const status = row.insertCell();
    status.className = 'status';
    status.textContent = p.status + (p.keep_current ? ' (keep)' : '');
    const why = row.insertCell();
    why.className = 'why';
    why.appendChild(Object.assign(document.createElement('div'), { className: 'why-text', textContent: p.rationale,
                                                                   title: p.rationale }));
    if (p.needs_reboot) why.appendChild(Object.assign(document.createElement('span'), { className: 'tag reboot', textContent: 'reboot' }));
    if (p.error) why.appendChild(Object.assign(document.createElement('div'), { className: 'bad', textContent: p.error }));
  }
}

function sameParam(a, b) {
  return isNum(a) && isNum(b) && Math.abs(a - b) <= Math.max(1e-6, 1e-6 * Math.abs(b));
}

function paramError(p, text) {
  if (String(text).trim() === '') return 'empty';
  const value = Number(text);
  if (!isFinite(value)) return 'not a number';
  if (value < p.min || value > p.max) return 'outside ' + p.min + '..' + p.max;
  if (p.integer && !Number.isInteger(value)) return 'must be a whole number';
  if (p.allowed && !p.allowed.includes(value)) return 'must be one of ' + p.allowed.join(', ');
  return '';
}

// --- parameter check banner (param_check.js) -----------------------------------------------------

function renderParamCheck(check) {
  const view = paramCheckView(check);
  const box = $('params-check');
  box.className = 'params-check ' + view.state;
  setText('params-title', view.title);
  setText('params-detail', view.detail);
  const body = $('params-table-body');
  body.replaceChildren();
  for (const [name, current, recommended, reboot] of view.rows) {
    const row = body.insertRow();
    row.insertCell().appendChild(Object.assign(document.createElement('b'), { textContent: name }));
    row.insertCell().textContent = current;
    row.insertCell().textContent = '→';
    row.insertCell().textContent = recommended;
    const tag = row.insertCell();
    if (reboot) tag.appendChild(Object.assign(document.createElement('span'), { className: 'tag reboot', textContent: 'needs reboot' }));
  }
  $('params-table').hidden = !view.rows.length;
  const button = $('params-configure');
  button.hidden = !view.configure;
  button.disabled = !view.configureEnabled;
  button.title = view.configureTitle;
  const notes = $('params-notes');
  notes.replaceChildren(...view.notes.map(text => Object.assign(document.createElement('div'), { textContent: text })));
}

function openConfigureDialog() {
  const check = ui.status && ui.status.autopilot_params;
  const changes = paramConfigureChanges(check);
  if (!changes.length) return showToast('Nothing differs from the recommended setup', 'error');
  showWriteDialog(changes, { armed: check.armed }, 'Configure: write ' + changes.length
    + ' recommended parameter(s) to the autopilot?');
}

function showWriteDialog(changes, heartbeat, title) {
  ui.pendingWrite = changes;
  setText('ap-dialog-title', title);
  const armed = $('ap-dialog-armed');
  armed.textContent = (heartbeat.armed === false ? 'Vehicle DISARMED at the last read. '
    : heartbeat.armed === true ? 'Vehicle ARMED at the last read: the server will refuse. '
    : 'Armed state unknown at the last read: the server will refuse unless a fresh HEARTBEAT says disarmed. ')
    + 'The server checks a new HEARTBEAT again right before writing and refuses (HTTP 409) if armed or unknown.';
  const list = $('ap-dialog-list');
  list.replaceChildren();
  for (const c of changes) {
    const item = document.createElement('li');
    item.innerHTML = '<b></b> <span></span>';
    item.firstChild.textContent = c.name;
    item.lastChild.textContent = paramText(c.old) + ' → ' + paramText(c.new) + (c.reboot ? '  (needs an autopilot reboot)' : '');
    list.appendChild(item);
  }
  $('ap-dialog-result').replaceChildren();
  const confirmButton = $('ap-dialog-confirm');
  confirmButton.hidden = false;
  confirmButton.disabled = false;
  confirmButton.textContent = 'Write ' + changes.length + ' parameter(s)';
  setText('ap-dialog-cancel', 'Cancel');
  $('ap-dialog').showModal();
}

function openWriteDialog() {
  if (!ui.autopilot) return showToast('Read the parameters first', 'error');
  const changes = [], problems = [];
  for (const row of $('ap-table').tBodies[0].rows) {
    if (!row.querySelector('.ap-select').checked) continue;
    const p = ui.autopilot.params.find(q => q.name === row.dataset.name);
    const text = row.querySelector('.ap-value').value;
    const error = paramError(p, text);
    if (error) { problems.push(p.name + ': ' + error); continue; }
    const value = Number(text);
    if (sameParam(p.current, value)) continue;
    changes.push({ name: p.name, old: p.current, new: value, reboot: p.needs_reboot });
  }
  if (problems.length) return showToast('Invalid: ' + problems.join('; '), 'error');
  if (!changes.length) return showToast('Nothing selected differs from the vehicle', 'error');
  showWriteDialog(changes, ui.autopilot.heartbeat || {}, 'Write ' + changes.length + ' parameter(s) to the autopilot?');
}

function closeWriteDialog() {
  $('ap-dialog').close();
}

function resultList(title, items, format) {
  const block = document.createElement('div');
  const heading = document.createElement('h2');
  heading.textContent = title;
  heading.className = 'sub';
  const list = document.createElement('ul');
  list.className = 'changes';
  for (const item of items) list.appendChild(Object.assign(document.createElement('li'), { textContent: format(item) }));
  block.append(heading, list);
  return block;
}

async function confirmWrite() {
  const changes = ui.pendingWrite;
  if (!changes) return;
  const confirmButton = $('ap-dialog-confirm');
  confirmButton.disabled = true;
  confirmButton.textContent = 'Writing…';
  const params = {};
  for (const c of changes) params[c.name] = c.new;
  let r;
  try {
    r = await fetchJSON('api/autopilot/params', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                                    body: JSON.stringify({ confirm: true, params }) });
  } catch (e) {
    r = { ok: false, status: 0, data: { detail: 'web server not reachable' } };
  }
  ui.pendingWrite = null;
  confirmButton.hidden = true;
  setText('ap-dialog-cancel', 'Close');
  const result = $('ap-dialog-result');
  result.replaceChildren();
  if (!r.ok) {
    const box = document.createElement('div');
    box.className = 'panel-message';
    box.textContent = (r.status === 409 ? 'Refused, nothing written (HTTP 409): ' : 'Not written (HTTP ' + r.status + '): ')
      + errorText(r.data, r.status);
    result.appendChild(box);
    if (r.data && r.data.heartbeat && ui.autopilot) { ui.autopilot.heartbeat = r.data.heartbeat; renderAutopilotArmed(); }
    updateStatus();
    return;
  }
  const data = r.data;
  if (data.changed.length) result.appendChild(resultList('Changed (read back)', data.changed,
    c => c.name + ': ' + paramText(c.old) + ' → ' + paramText(c.new) + ', read back ' + paramText(c.readback)));
  if (data.unchanged.length) result.appendChild(resultList('Already set', data.unchanged, c => c.name + ' = ' + paramText(c.value)));
  if (data.failed.length) result.appendChild(resultList('Failed', data.failed, c => c.name + ': ' + c.error));
  if (data.reboot_required.length) {
    result.appendChild(Object.assign(document.createElement('div'), { className: 'warn',
      textContent: 'Reboot the autopilot for ' + data.reboot_required.join(', ') + ' to take effect (not done automatically).' }));
  }
  if (ui.autopilot) {
    for (const c of data.changed) {
      const p = ui.autopilot.params.find(q => q.name === c.name);
      if (p) { p.current = c.readback; p.status = sameParam(p.current, p.suggested) ? 'ok' : 'differs'; }
    }
    ui.autopilot.heartbeat = data.heartbeat;
    renderAutopilot();
  }
  updateStatus();
  showToast(data.changed.length + ' parameter(s) changed' + (data.failed.length ? ', ' + data.failed.length + ' failed' : ''),
            data.failed.length ? 'error' : 'success');
}

function renderAutopilotArmed() {
  const armed = $('ap-armed');
  armed.textContent = armedText(ui.autopilot.heartbeat);
  armed.className = ui.autopilot.heartbeat.armed === false ? 'ok' : 'bad';
}

// --- data logs ----------------------------------------------------------------------------------

async function updateLogs() {
  let logs;
  try { logs = await (await fetch('api/logs')).json(); } catch (e) { return; }
  const list = $('logs');
  list.replaceChildren();
  if (!logs.length) {
    const item = document.createElement('li');
    item.textContent = 'No logs yet';
    list.appendChild(item);
  }
  for (const log of logs) {
    const item = document.createElement('li');
    const link = document.createElement('a');
    link.href = 'api/logs/' + encodeURIComponent(log.name);
    link.textContent = log.name;
    link.download = log.name;
    item.appendChild(link);
    item.append(' · ' + (log.size / 1024).toFixed(1) + ' KiB · ' + new Date(log.mtime * 1000).toLocaleString());
    list.appendChild(item);
  }
}

// --- start --------------------------------------------------------------------------------------

function tick() {
  if (!visible()) return;
  updateStatus();
  updateHistory();
}

$('config-form').addEventListener('submit', applySettings);
$('view-left').querySelector('img').addEventListener('click', aimAt);
$('vel-scale').addEventListener('change', () => ui.history && renderVelocity(ui.history));
$('hist-window').addEventListener('change', () => { ui.historyEtag = null; updateHistory(); });
$('view-left-stream').addEventListener('change', () => refreshViews(true));
document.addEventListener('visibilitychange', () => {
  if (visible()) { tick(); startDepthStream(); } else if (depthStream.on) stopDepthStream('paused (tab hidden)');
});
$('depth-stream-on').addEventListener('change', toggleDepthStream);
$('depth-stream-color').addEventListener('change', toggleStreamColor);
$('depth-stream-features').addEventListener('change', toggleStreamFeatures);
$('depth-stream-img').addEventListener('error', depthStreamError);
$('depth-stream-img').addEventListener('load', () => { if (depthStreaming()) colorizeDepth(); });
if (window.ResizeObserver) new ResizeObserver(drawStreamFeatures).observe($('depth-stream-img').parentElement);
setInterval(() => { renderFeatureCounts(); drawStreamFeatures(); }, 1000);  // let stale features fade out
for (const figure of document.querySelectorAll('.view')) figure.classList.add('empty');

initSettings();
tick();
setInterval(tick, 1000);
setInterval(() => { if ($('view-auto').checked && visible()) refreshViews(false); }, 1000);
updateLogs();
setInterval(() => visible() && updateLogs(), 15000);
readAutopilot();
