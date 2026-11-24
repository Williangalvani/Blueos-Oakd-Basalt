'use strict';
// Camera link (XLink) numbers for the main page's status panel and the Cockpit widget's status line:
// formatting only, no DOM. Input: /api/status telemetry.link (link_stats.LinkMonitor.status), and for the
// depth transport part /api/status depth_transport and viewers.

const USB_SPEEDS = { LOW: '1.5 Mbps', FULL: '12 Mbps', HIGH: '480 Mbps', SUPER: '5 Gbps', SUPER_PLUS: '10 Gbps' };

function linkNumber(v) {
  return typeof v === 'number' && isFinite(v);
}

function linkMbps(v, digits) {
  return linkNumber(v) ? v.toFixed(digits === undefined ? 1 : digits) + ' Mbps' : '–';
}

function linkPercent(fraction) {
  return linkNumber(fraction) ? (100 * fraction).toFixed(0) + '%' : '–';
}

function linkBytes(n) {
  if (!linkNumber(n)) return '–';
  if (n >= 1e6) return (n / 1e6).toFixed(1) + ' MB';
  if (n >= 1e3) return (n / 1e3).toFixed(1) + ' kB';
  return n.toFixed(0) + ' B';
}

function linkSource(link) {
  return link.source + (link.mock && !String(link.source).includes('mock') ? ' (mock)' : '');
}

function linkWhere(link) {
  const usb = link.usb_speed && link.usb_speed !== 'UNKNOWN'
    ? link.usb_speed + (USB_SPEEDS[link.usb_speed] ? ' (' + USB_SPEEDS[link.usb_speed] + ')' : '') : null;
  const where = [link.protocol, usb].filter(Boolean).join(' ');
  return [where, link.platform].filter(Boolean).join(' · ');
}

// [[label, value], ...] for the status panel's key-value grid
function linkSummaryRows(link) {
  if (!link) return [['camera link', 'n/a (no link data from the VIO)']];
  if (!link.available) {
    const rows = [['camera link', 'n/a: ' + (link.reason || 'XLink profiling unavailable')]];
    const where = linkWhere(link);
    if (where) rows.push(['link', where]);
    return rows;
  }
  const span = linkNumber(link.window_span_s) ? link.window_span_s.toFixed(0) : '–';
  const rows = [
    ['down (camera → Pi)', linkMbps(link.down_mbps) + ' now · ' + linkMbps(link.down_mbps_10s)
      + ' mean (' + span + ' s)'],
    ['up (Pi → camera)', linkMbps(link.up_mbps, 2) + ' now · ' + linkMbps(link.up_mbps_10s, 2) + ' mean'],
    ['peak down (' + (link.peak_window_s || 60) + ' s)', linkMbps(link.peak_down_mbps_60s)],
    ['share of ' + (link.reference_mbps || 100) + ' Mbps', linkPercent(link.share_of_100mbps) + ' mean · '
      + linkPercent(link.peak_share_of_100mbps) + ' peak'],
    ['link', linkWhere(link) || '–'],
  ];
  if (link.totals_bytes) {
    rows.push(['total since connect', linkBytes(link.totals_bytes.down) + ' down · '
      + linkBytes(link.totals_bytes.up) + ' up']);
  }
  if (linkNumber(link.streams_sum_mbps)) {
    rows.push(['streams (estimate)', linkMbps(link.streams_sum_mbps) + ' · unaccounted '
      + linkMbps(link.unaccounted_mbps)]);
  }
  if (linkNumber(link.streams_up_sum_mbps) && link.streams_up_sum_mbps > 0) {
    rows.push(['streams up (estimate)', linkMbps(link.streams_up_sum_mbps, 2)]);
  }
  rows.push(['source', linkSource(link)]);
  return rows;
}

// One row per host queue; fraction: its share of the measured total of its direction (else of the streams'
// sum). Streams the Pi sends to the camera (direction "up") are named with a leading ↑.
function linkStreamRows(link) {
  if (!link || !Array.isArray(link.streams)) return [];
  const measured = (v, sum) => (linkNumber(v) && v > 0 ? v : sum);
  const totals = {
    down: measured(link.down_mbps_10s, link.streams_sum_mbps),
    up: measured(link.up_mbps_10s, link.streams_up_sum_mbps),
  };
  return link.streams.map(s => {
    const up = s.direction === 'up';
    const total = totals[up ? 'up' : 'down'];
    return {
      name: (up ? '↑ ' : '') + s.name,
      direction: up ? 'up' : 'down',
      description: s.description || '',
      mbps: linkNumber(s.mbps) ? s.mbps.toFixed(2) : '–',
      rate: linkNumber(s.msgs_per_s) ? s.msgs_per_s.toFixed(s.msgs_per_s >= 100 ? 0 : 1) + '/s' : '–',
      size: linkBytes(s.bytes_per_msg),
      fraction: linkNumber(s.mbps) && linkNumber(total) && total > 0 ? Math.min(1, s.mbps / total) : 0,
    };
  });
}

// The Cockpit widget's status line part ('' while unavailable)
function linkWidgetText(link) {
  if (!link || !link.available || !linkNumber(link.down_mbps_10s)) return '';
  return 'link ' + linkMbps(link.down_mbps_10s) + ' (peak ' + (linkNumber(link.peak_down_mbps_60s)
    ? link.peak_down_mbps_60s.toFixed(1) : '–') + ') · ' + linkPercent(link.share_of_100mbps) + ' of '
    + (link.reference_mbps || 100) + ' Mbps' + (link.mock ? ' (mock)' : '');
}

// --- depth transport, camera load and viewers (/api/status depth_transport and viewers) ---

function pct(v, digits) {
  return linkNumber(v) ? v.toFixed(digits === undefined ? 1 : digits) + '%' : '–';
}

function byReason(counts) {
  const entries = Object.entries(counts || {}).filter(([, n]) => n);
  return entries.length ? entries.map(([reason, n]) => reason + ' ' + n).join(', ') : '';
}

function viewersText(viewers) {
  if (!viewers) return '–';
  const clients = Object.entries(viewers.by_client || {}).filter(([, n]) => n).map(([c, n]) => c + ' ' + n);
  return viewers.text + (clients.length ? ' (' + clients.join(', ') + ')' : '');
}

function cameraLoadRows(load) {
  if (!load) return [['camera CPU', '–']];
  const memory = name => (load[name] ? pct(load[name].used_percent, 0) : '–');
  return [
    ['camera CPU (Leon)', 'CSS ' + pct(load.leon_css_cpu_percent, 0) + ' · MSS ' + pct(load.leon_mss_cpu_percent, 0)],
    ['camera memory', 'DDR ' + memory('ddr') + ' · CMX ' + memory('cmx') + ' · heap CSS ' + memory('leon_css_heap')
      + ' / MSS ' + memory('leon_mss_heap')],
  ];
}

// [[label, value], ...] for the status panel's "Depth for PnP" grid
function transportRows(transport, viewers) {
  const t = transport || {};
  const rows = [['setting', !t.configured ? '–' : t.configured === 'points'
    ? 'points (ROIs listed on the ' + (t.configured_rois === 'camera' ? 'camera' : 'Pi') + ')' : 'full_frame']];
  const live = t.live;
  if (!live || live.error) {
    rows.push(['running', live && live.error ? 'status error: ' + live.error : 'n/a (VIO not running)']);
  } else if (live.transport === 'points') {
    rows.push(['running', 'points: depth at the features only' + (live.mock ? ' (mock)' : '')]);
    rows.push(['frames with depth', pct(live.paired_percent, 2) + ' (' + live.paired + ' of ' + live.frames + ')']);
    const reasons = byReason(live.missing_by_reason);
    rows.push(['missing depth', live.missing + (reasons ? ' (' + reasons + ')' : '')]);
    if (live.last_problem) rows.push(['last problem', live.last_problem.reason + ': ' + live.last_problem.text]);
    rows.push(['feature depth valid', pct(live.valid_percent)]);
    rows.push(['frame drops', 'features ' + live.feature_gap_frames + ' (' + live.feature_gaps + ' gaps) · points '
      + live.points_gap_frames + ' (' + live.points_gaps + ' gaps) · unpaired points ' + live.orphan_points]);
    rows.push(['wait for points', (linkNumber(live.wait_ms_mean) ? live.wait_ms_mean.toFixed(1) + ' ms mean, '
      + live.wait_ms_max.toFixed(1) + ' max' : '–') + ' (limit ' + live.wait_limit_ms + ' ms'
      + (live.waiting ? '' : ', not waiting: no points') + ')']);
    const device = live.device || {};
    const stats = device.stats;
    rows.push(['camera script', (device.errors || 0) + ' errors · ' + (device.misses || 0) + ' misses'
      + (stats ? ' · SLC ' + stats.slc_ms_mean.toFixed(1) + ' ms mean' : '') + (device.fatal ? ' · STOPPED' : '')]);
    const full = live.full_depth;
    rows.push(['full depth frames', !full ? '–' : full.on
      ? 'on, every ' + full.every + ' frame(s), for ' + full.reasons.join(', ') + ' · ' + full.frames + ' sent'
      : 'off (only for the depth probe or a watched host_jpeg stream) · ' + full.frames + ' sent']);
  } else {
    rows.push(['running', 'full_frame: every depth frame to the Pi' + (live.mock ? ' (mock)' : '')]);
    rows.push(['frame drops', 'features ' + (live.feature_gap_frames || 0) + ' (' + (live.feature_gaps || 0) + ' gaps)']);
  }
  rows.push(...cameraLoadRows(t.camera_load));
  rows.push(['viewers', viewersText(viewers)]);
  return rows;
}

// The widget's status line parts: [text, css class] ('' text: nothing to say)
function transportWidgetParts(transport, viewers) {
  const parts = [];
  if (transport && transport.transport === 'points') {
    const p = transport.paired_percent;
    parts.push(['depth points ' + pct(p, 0), linkNumber(p) && p < 90 ? 'bad' : 'muted']);
  } else if (transport && transport.transport === 'full_frame') {
    parts.push(['depth: full frames', 'muted']);
  }
  if (viewers && viewers.text) parts.push([viewers.text, 'muted']);
  return parts;
}

// [[label, value], ...] for the "Image streams" grid (/api/status image_streams): codec, quality, where encoded,
// Mbps while viewed; raw frames encoded on the Pi are flagged as fallbacks
function imageStreamRows(images) {
  const streams = (images && images.streams) || [];
  if (!streams.length) return [['image streams', '–']];
  return streams.map((s) => {
    const codec = s.codec ? s.codec.toUpperCase() + (linkNumber(s.quality) ? ' q' + s.quality : '') : '–';
    const where = !s.encoded_on ? 'n/a (VIO not running)' : s.encoded_on === 'camera' ? 'encoded on the camera'
      : 'encoded on the Pi';
    const parts = [codec, where];
    if (s.per_request) {
      parts.push('one frame per request' + (linkNumber(s.last_bytes) ? ', last ' + linkBytes(s.last_bytes) : ''));
    } else {
      parts.push(s.viewed ? 'viewed · ' + linkMbps(s.mbps, 2) : 'not viewed: nothing sent');
    }
    if (s.measurement) parts.push('16-bit measurement data, raw');
    if (s.note) parts.push(s.note);
    if (s.fallback) parts.push(s.fallback);
    else if (s.reason && s.encoded_on === 'pi' && !s.measurement) parts.push(s.reason);
    return [s.label || s.name, parts.join(' · ')];
  });
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { linkSummaryRows, linkStreamRows, linkWidgetText, transportRows, transportWidgetParts,
    imageStreamRows };
}
