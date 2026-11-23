'use strict';
// OAK-D VIO depth widget (Cockpit external widget, also usable on its own).
// Sources, both only while this page is visible (the camera and the VIO stop publishing a few seconds
// after the last client leaves):
//   api/stream/depth     MJPEG, gray linear in depth (depth_stream.py), colorized here on a canvas
//   api/stream/features  server-sent events: "features" (points [u, v, state] on the depth image's
//                        pixel grid, feature_stream.py) and "status" (cockpit_widget.widget_status)
// URLs are relative to the extension's base, so the page works on port 8095 and behind BlueOS's
// /extensionv2/<name>/ proxy. Query options: colorize=0 (gray depth), features=0, status=0.

const BASE = location.pathname.replace(/widget\/depth\/?$/, '');
const DEPTH_URL = BASE + 'api/stream/depth';
const EVENTS_URL = BASE + 'api/stream/features';
const OPTIONS = new URLSearchParams(location.search);
const option = (name, fallback) => (OPTIONS.has(name) ? OPTIONS.get(name) !== '0' : fallback);
const COLORIZE = option('colorize', true);
const SHOW_FEATURES = option('features', true);
const SHOW_STATUS = option('status', true);
const FEATURES_STALE_MS = 2000;
const RETRY_MS = 3000;
const STATES = ['no_depth', 'depth', 'matched', 'outlier', 'inlier'];
const BLANK_GIF = 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';

const $ = id => document.getElementById(id);
const isNum = v => typeof v === 'number' && isFinite(v);
const fmt = (v, digits) => (isNum(v) ? v.toFixed(digits) : '–');
const visible = () => document.visibilityState !== 'hidden';

const w = { events: null, eventsRetry: null, status: null, features: null, featuresAt: 0, depthOn: false,
            depthRetry: null, colorTimer: null, lut: null, lutKey: '', scale: null, frame: null };

// --- depth colours (same scale and lookup table as the full page) ---------------------------------

function jet(x) {
  const c = v => Math.round(255 * Math.max(0, Math.min(1, v)));
  return [c(1.5 - Math.abs(4 * x - 3)), c(1.5 - Math.abs(4 * x - 2)), c(1.5 - Math.abs(4 * x - 1))];
}

function streamScale(d) {
  const range = (d && d.depth_range_mm) || [300, 4000];
  return { min: range[0], max: range[1], far: (d && d.gray_far) || 16, near: (d && d.gray_near) || 255,
           invalid: (d && d.gray_invalid_below) || 8 };
}

function depthLut(scale) {
  const key = [scale.far, scale.near, scale.invalid, COLORIZE].join(',');
  if (w.lutKey === key) return w.lut;
  const lut = new Uint8ClampedArray(256 * 3);
  for (let g = 0; g < 256; g++) {
    const rgb = g < scale.invalid ? [0, 0, 0] : COLORIZE ? jet((g - scale.far) / (scale.near - scale.far)) : [g, g, g];
    lut.set(rgb, g * 3);
  }
  w.lut = lut;
  w.lutKey = key;
  return lut;
}

function drawLegend(scale) {
  const canvas = $('depth-legend'), ctx = canvas.getContext('2d');
  const lut = depthLut(scale), image = ctx.createImageData(canvas.width, 1);
  for (let i = 0; i < canvas.width; i++) {
    const g = Math.round(scale.near - (scale.near - scale.far) * i / (canvas.width - 1));
    image.data.set(lut.subarray(g * 3, g * 3 + 3), i * 4);
    image.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(image, 0, 0);
  $('legend-near').textContent = (scale.min / 1000).toFixed(2) + ' m';
  $('legend-far').textContent = (scale.max / 1000).toFixed(2) + ' m';
}

function colorize() {
  const img = $('depth-img'), canvas = $('depth-canvas');
  if (!w.depthOn || !img.naturalWidth || img.naturalWidth < 2) return;
  if (canvas.width !== img.naturalWidth || canvas.height !== img.naturalHeight) {
    canvas.width = img.naturalWidth;
    canvas.height = img.naturalHeight;
  }
  const ctx = canvas.getContext('2d', { willReadFrequently: true });
  try { ctx.drawImage(img, 0, 0); } catch (e) { return; }
  const frame = ctx.getImageData(0, 0, canvas.width, canvas.height), px = frame.data;
  const lut = depthLut(w.scale || streamScale(null));
  for (let i = 0; i < px.length; i += 4) {
    const k = px[i] * 3;
    px[i] = lut[k]; px[i + 1] = lut[k + 1]; px[i + 2] = lut[k + 2];
  }
  ctx.putImageData(frame, 0, 0);
  if (!w.frame || w.frame[0] !== canvas.width || w.frame[1] !== canvas.height) {
    w.frame = [canvas.width, canvas.height];
    drawOverlay();
  }
  $('depth-canvas').classList.remove('off');
}

// --- depth stream (the <img> keeps the multipart request open; dropping its src ends it) ---------

function startDepth() {
  clearTimeout(w.depthRetry);
  w.depthRetry = null;
  if (w.depthOn || !visible()) return;
  w.depthOn = true;
  $('depth-img').src = DEPTH_URL + '?t=' + Date.now();
  const fps = (w.status && w.status.depth && w.status.depth.fps_target) || 5;
  clearInterval(w.colorTimer);
  w.colorTimer = setInterval(colorize, Math.max(60, Math.min(500, 500 / fps)));
}

function stopDepth() {
  clearTimeout(w.depthRetry);
  w.depthRetry = null;
  clearInterval(w.colorTimer);
  w.depthOn = false;
  const img = $('depth-img');
  if (img.getAttribute('src')) {
    img.src = BLANK_GIF;
    img.removeAttribute('src');
  }
  $('depth-canvas').classList.add('off');
  w.frame = null;
  drawOverlay();
}

function depthError() {
  if (!w.depthOn) return;
  stopDepth();
  w.depthRetry = setTimeout(() => { w.depthRetry = null; syncDepth(); }, RETRY_MS);
}

function syncDepth() {
  const s = w.status, d = (s && s.depth) || {};
  const wanted = visible() && s && !s.problem && d.enabled;
  if (wanted && !w.depthOn && !w.depthRetry) startDepth();
  else if (!wanted && w.depthOn) stopDepth();
}

// --- features overlay -----------------------------------------------------------------------------

// Where the depth frame (frame_size, letterboxed like object-fit: contain) sits in the view, in CSS px
function frameRect(frameSize, width, height) {
  const s = Math.min(width / frameSize[0], height / frameSize[1]);
  const dw = frameSize[0] * s, dh = frameSize[1] * s;
  return { x: (width - dw) / 2, y: (height - dh) / 2, w: dw, h: dh };
}

function marker(ctx, state, x, y) {
  ctx.beginPath();
  if (state === 4) {  // inlier
    ctx.arc(x, y, 3.5, 0, 2 * Math.PI);
    ctx.fillStyle = '#00ff66';
    ctx.fill();
    ctx.stroke();
  } else if (state === 3) {  // outlier: rejected by RANSAC
    ctx.rect(x - 3, y - 3, 6, 6);
    ctx.fillStyle = '#ff2bd6';
    ctx.fill();
    ctx.stroke();
  } else if (state === 2) {  // matched, no RANSAC result
    ctx.arc(x, y, 3.5, 0, 2 * Math.PI);
    ctx.lineWidth = 3; ctx.stroke();
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.strokeStyle = '#000'; ctx.lineWidth = 1;
  } else if (state === 1) {  // valid depth, not in the reference
    ctx.arc(x, y, 2, 0, 2 * Math.PI);
    ctx.fillStyle = '#8fd3ff';
    ctx.fill();
    ctx.stroke();
  } else {  // no depth
    ctx.arc(x, y, 2, 0, 2 * Math.PI);
    ctx.strokeStyle = '#9aa4b0';
    ctx.stroke();
    ctx.strokeStyle = '#000';
  }
}

function drawOverlay() {
  const canvas = $('overlay'), view = $('view');
  const dpr = window.devicePixelRatio || 1;
  const width = view.clientWidth, height = view.clientHeight;
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const f = w.features;
  const fresh = f && performance.now() - w.featuresAt < FEATURES_STALE_MS;
  const frameSize = w.frame || (f && f.frame_size) || [640, 400];
  const rect = frameRect(frameSize, width, height);
  if (!w.frame) {
    ctx.strokeStyle = '#2a3442';
    ctx.strokeRect(rect.x + 0.5, rect.y + 0.5, rect.w - 1, rect.h - 1);
  }
  if (!SHOW_FEATURES || !fresh || !f.frame_size) return;
  // Points are on the depth image's grid (frame_size); the shown JPEG covers the same field of view
  const sx = rect.w / f.frame_size[0], sy = rect.h / f.frame_size[1];
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 1;
  for (const state of [0, 1, 2, 3, 4]) {  // best on top
    for (const p of f.points) {
      if (p[2] === state) marker(ctx, state, rect.x + (p[0] + 0.5) * sx, rect.y + (p[1] + 0.5) * sy);
    }
  }
}

function renderFeatureKeys() {
  const f = w.features;
  const fresh = f && performance.now() - w.featuresAt < FEATURES_STALE_MS;
  for (const state of STATES) {
    $('n-' + state).textContent = fresh && f.counts && isNum(f.counts[state]) ? f.counts[state] : '–';
  }
}

// --- status line ----------------------------------------------------------------------------------

function span(text, cls) {
  const el = document.createElement('span');
  el.textContent = text;
  if (cls) el.className = cls;
  return el;
}

function renderStatus(s) {
  w.status = s;
  const d = s.depth || {};
  w.scale = streamScale(d);
  const key = JSON.stringify(w.scale);
  if (renderStatus.legend !== key) {
    drawLegend(w.scale);
    renderStatus.legend = key;
  }
  $('depth-rate').textContent = d.streaming && isNum(d.fps) ? d.fps.toFixed(1) + ' fps'
    : d.enabled === false ? 'depth stream off' : '';

  const line = $('status-line');
  line.hidden = !SHOW_STATUS;
  const parts = [];
  if (s.problem) {
    parts.push(span(s.problem, 'warn'));
  } else {
    const last = s.last_frame || {};
    parts.push(span(fmt(s.fps, 1) + ' fps'));
    parts.push(span('ok ' + fmt(s.ok_percent, 0) + '%', isNum(s.ok_percent) && s.ok_percent < 50 ? 'bad' : ''));
    parts.push(span('inliers ' + (isNum(last.inliers) ? last.inliers : '–')
      + (isNum(s.inliers_mean) ? ' (mean ' + s.inliers_mean.toFixed(0) + ')' : ''),
      isNum(last.inliers) && isNum(s.min_inliers) && last.inliers < s.min_inliers ? 'bad' : ''));
    parts.push(span('depth ' + (isNum(last.valid_depth) ? last.valid_depth : '–') + '/'
      + (isNum(last.features) ? last.features : '–')));
    if (s.log_only) parts.push(span('LOG-ONLY, nothing sent', 'warn'));
    else parts.push(span('sent ' + (isNum(s.sent_total) ? s.sent_total : '–') + ' (' + fmt(s.send_rate_hz, 1) + '/s)'));
  }
  if (s.problem && s.log_only) parts.push(span('LOG-ONLY', 'warn'));
  const failure = s.last_failure;
  if (failure && failure.reason) {
    const age = isNum(failure.age_s) ? ' ' + (failure.age_s < 60 ? failure.age_s.toFixed(0) + ' s' : (failure.age_s / 60).toFixed(0) + ' min') + ' ago' : '';
    parts.push(span('last fail: ' + String(failure.reason).split(':')[0] + age, failure.age_s < 2 ? 'bad' : 'muted'));
  }
  line.replaceChildren(...parts);

  let message = s.problem || '';
  if (!message && !d.enabled) message = 'depth stream off in the settings: features only';
  else if (!message && !w.frame && !(w.features && performance.now() - w.featuresAt < FEATURES_STALE_MS)) message = 'waiting for frames…';
  $('message').textContent = message;
  syncDepth();
  renderFeatureKeys();
  drawOverlay();
}

// --- server-sent events ---------------------------------------------------------------------------

function startEvents() {
  clearTimeout(w.eventsRetry);
  w.eventsRetry = null;
  if (w.events || !visible()) return;
  const es = new EventSource(EVENTS_URL);
  w.events = es;
  es.addEventListener('status', e => { try { renderStatus(JSON.parse(e.data)); } catch (err) { /* partial */ } });
  es.addEventListener('features', e => {
    try { w.features = JSON.parse(e.data); } catch (err) { return; }
    w.featuresAt = performance.now();
    renderFeatureKeys();
    drawOverlay();
  });
  es.onerror = () => {
    if (es.readyState === EventSource.CLOSED) {  // HTTP error: EventSource gives up, so retry here
      stopEvents();
      $('message').textContent = 'web server not reachable, retrying…';
      w.eventsRetry = setTimeout(startEvents, RETRY_MS);
    }
  };
}

function stopEvents() {
  clearTimeout(w.eventsRetry);
  w.eventsRetry = null;
  if (w.events) w.events.close();
  w.events = null;
}

document.addEventListener('visibilitychange', () => {
  if (visible()) { startEvents(); syncDepth(); } else { stopEvents(); stopDepth(); }
});
$('depth-img').addEventListener('error', depthError);
if (window.ResizeObserver) new ResizeObserver(drawOverlay).observe($('view'));
else window.addEventListener('resize', drawOverlay);
setInterval(() => { renderFeatureKeys(); drawOverlay(); }, 1000);  // let stale features fade out
$('feature-keys').hidden = !SHOW_FEATURES;
drawLegend(streamScale(null));
drawOverlay();
startEvents();
