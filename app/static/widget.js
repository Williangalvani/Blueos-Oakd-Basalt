'use strict';
// OAK-D VIO depth widget (Cockpit external widget, also usable on its own).
// Sources, both only while this page is visible (the camera and the VIO stop publishing a few seconds
// after the last client leaves):
//   api/stream/depth     MJPEG (depth_stream.py): with mono_features (the default) the rectified-left camera
//                        image the features are tracked on, shown as it is; else gray linear in depth (or
//                        disparity), colorized here on a canvas. status.depth.encoding says which
//   api/stream/features  server-sent events: "features" (points [u, v, state] on the depth image's
//                        pixel grid, feature_stream.py; drawn by features_overlay.js, inlined before this)
//                        and "status" (cockpit_widget.widget_status)
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
const MONO_ENCODING = 'gray8-mono-rectified-left';
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

// d.encoding "gray8-depth-linear": gray linear in depth (gray_far at max_mm .. gray_near at min_mm);
// "gray8-disparity": gray = disparity in px, depth_mm = d.disparity.depth_times_disparity / gray. 0 = no depth.
// "gray8-mono-rectified-left": the camera image, no depth scale (kind 'mono', shown without a lookup table).
function streamScale(d) {
  const range = (d && d.depth_range_mm) || [300, 4000];
  if (d && d.encoding === MONO_ENCODING) return { kind: 'mono', min: range[0], max: range[1] };
  const disparity = d && d.encoding === 'gray8-disparity' && d.disparity;
  if (disparity && disparity.depth_times_disparity > 0) {
    return { kind: 'disparity', min: range[0], max: range[1], far: disparity.gray_far, near: disparity.gray_near,
             invalid: disparity.gray_invalid_below || 3, k: disparity.depth_times_disparity };
  }
  return { kind: 'depth', min: range[0], max: range[1], far: (d && d.gray_far) || 16, near: (d && d.gray_near) || 255,
           invalid: (d && d.gray_invalid_below) || 8 };
}

function grayDepth(scale, g) {
  if (g < scale.invalid) return NaN;
  if (scale.kind === 'disparity') return scale.k / g;
  return scale.max - (scale.max - scale.min) * (g - scale.far) / (scale.near - scale.far);
}

// Colour of a depth (mm): jet near red .. far blue, or (no COLORIZE) gray near bright
function depthColor(scale, mm) {
  if (Number.isNaN(mm)) return [0, 0, 0];
  const x = (scale.max - mm) / (scale.max - scale.min);
  if (COLORIZE) return jet(x);
  const g = Math.round(16 + 239 * Math.max(0, Math.min(1, x)));
  return [g, g, g];
}

function depthLut(scale) {
  const key = [scale.kind, scale.far, scale.near, scale.invalid, scale.k, scale.min, scale.max, COLORIZE].join(',');
  if (w.lutKey === key) return w.lut;
  const lut = new Uint8ClampedArray(256 * 3);
  for (let g = 0; g < 256; g++) lut.set(depthColor(scale, grayDepth(scale, g)), g * 3);
  w.lut = lut;
  w.lutKey = key;
  return lut;
}

function drawLegend(scale) {
  const canvas = $('depth-legend'), ctx = canvas.getContext('2d');
  const image = ctx.createImageData(canvas.width, 1);
  for (let i = 0; i < canvas.width; i++) {
    image.data.set(depthColor(scale, scale.min + (scale.max - scale.min) * i / (canvas.width - 1)), i * 4);
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
  const scale = w.scale || streamScale(null);
  if (scale.kind !== 'mono') {
    const frame = ctx.getImageData(0, 0, canvas.width, canvas.height), px = frame.data;
    const lut = depthLut(scale);
    for (let i = 0; i < px.length; i += 4) {
      const k = px[i] * 3;
      px[i] = lut[k]; px[i + 1] = lut[k + 1]; px[i + 2] = lut[k + 2];
    }
    ctx.putImageData(frame, 0, 0);
  }
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

// featureContainRect, drawFeatures: features_overlay.js

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
  // The shown JPEG's own size (depth: frame_size, mono_features: image_size; the same field of view)
  const frameSize = w.frame || (f && (f.image_size || f.frame_size)) || [640, 400];
  const rect = featureContainRect(frameSize, width, height);
  if (!w.frame) {
    ctx.strokeStyle = '#2a3442';
    ctx.strokeRect(rect.x + 0.5, rect.y + 0.5, rect.w - 1, rect.h - 1);
  }
  if (SHOW_FEATURES && fresh) drawFeatures(ctx, f, rect);
}

function renderFeatureKeys() {
  const f = w.features;
  const fresh = f && performance.now() - w.featuresAt < FEATURES_STALE_MS;
  for (const state of FEATURE_STATES) {
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
  const mono = w.scale.kind === 'mono';
  const key = JSON.stringify(w.scale);
  if (renderStatus.legend !== key) {
    if (!mono) drawLegend(w.scale);
    renderStatus.legend = key;
  }
  // The legend is a depth scale: only for the depth and disparity images
  $('depth-legend-box').hidden = mono;
  $('mono-label').hidden = !mono;
  document.title = mono ? 'OAK-D VIO camera' : 'OAK-D VIO depth';
  const rate = d.streaming && isNum(d.fps) ? d.fps.toFixed(1) + ' fps' : d.enabled === false ? 'stream off' : '';
  $('depth-rate').textContent = rate;
  $('mono-rate').textContent = rate;
  const what = mono ? 'camera' : 'depth';

  const line = $('status-line');
  line.hidden = !SHOW_STATUS;
  const parts = [];
  if (s.mock) parts.push(span('MOCK', 'mock'));
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
  if (!s.problem && d.errors) {
    const error = span(what + ': ' + d.errors + ' camera errors', 'bad');
    if (d.last_error) error.title = d.last_error.text;
    parts.push(error);
  }
  if (!s.problem && d.encoder_stalled) {
    const stalled = span(what + ': ' + (d.encoded_on === 'pi' ? 'Pi' : 'camera') + ' encoder outputs nothing', 'bad');
    if (d.last_error) stalled.title = d.last_error.text;
    parts.push(stalled);
  } else if (!s.problem && d.blank && d.encoded_on !== 'pi') {
    const blank = span(what + ': blank frames from ' + d.blank.variant, 'bad');
    blank.title = d.blank.text;
    parts.push(blank);
  } else if (!s.problem && d.streaming && d.encoded_on === 'pi') {
    const pi = span('depth: encoded on the Pi' + (d.fallback ? ' (fallback: ' + d.fallback.reason + ')' : ''), 'muted');    if (d.fallback && d.fallback.detail) pi.title = d.fallback.detail;
    parts.push(pi);
  }
  const link = s.problem ? '' : linkWidgetText(s.link);
  if (link) parts.push(span(link, 'muted'));
  const pose = s.pose;
  if (!s.problem && pose && pose.attitude_source === 'gyro') {
    const gyro = span('3D pose attitude: gyro-integrated, drifting', 'warn');
    gyro.title = pose.frame_text || '';
    parts.push(gyro);
  }
  line.replaceChildren(...parts);

  let message = s.problem || '';
  if (!message && !d.enabled) message = 'live stream off in the settings: features only';
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
