'use strict';
// Camera link (XLink) numbers for the main page's status panel and the Cockpit widget's status line:
// formatting only, no DOM. Input: /api/status telemetry.link (link_stats.LinkMonitor.status).

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
  rows.push(['source', linkSource(link)]);
  return rows;
}

// One row per host output queue; fraction: its share of the measured total (else of the streams' sum)
function linkStreamRows(link) {
  if (!link || !Array.isArray(link.streams)) return [];
  const total = linkNumber(link.down_mbps_10s) && link.down_mbps_10s > 0 ? link.down_mbps_10s
    : link.streams_sum_mbps;
  return link.streams.map(s => ({
    name: s.name,
    description: s.description || '',
    mbps: linkNumber(s.mbps) ? s.mbps.toFixed(2) : '–',
    rate: linkNumber(s.msgs_per_s) ? s.msgs_per_s.toFixed(s.msgs_per_s >= 100 ? 0 : 1) + '/s' : '–',
    size: linkBytes(s.bytes_per_msg),
    fraction: linkNumber(s.mbps) && linkNumber(total) && total > 0 ? Math.min(1, s.mbps / total) : 0,
  }));
}

// The Cockpit widget's status line part ('' while unavailable)
function linkWidgetText(link) {
  if (!link || !link.available || !linkNumber(link.down_mbps_10s)) return '';
  return 'link ' + linkMbps(link.down_mbps_10s) + ' (peak ' + (linkNumber(link.peak_down_mbps_60s)
    ? link.peak_down_mbps_60s.toFixed(1) : '–') + ') · ' + linkPercent(link.share_of_100mbps) + ' of '
    + (link.reference_mbps || 100) + ' Mbps' + (link.mock ? ' (mock)' : '');
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { linkSummaryRows, linkStreamRows, linkWidgetText };
}
