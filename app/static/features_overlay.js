'use strict';
// Tracked-features overlay, shared by the main page (ui.js) and the Cockpit widget (widget.js); main.py
// inlines it before either. Also loads under node (module.exports) for tests/test_features_overlay.py.
//
// A "features" event (feature_stream.py) has points [u, v, state] on the depth image's grid (frame_size),
// scaled from the rectified-left grid the FeatureTracker works on (image_size), in pixel-centre coordinates
// (pixel u spans u - 0.5 .. u + 0.5). The depth JPEG (frame_size) and the mono_features JPEG (the
// rectified-left image itself, image_size) cover the same field of view, so one mapping serves both: back
// to the image grid, + 0.5 from pixel-centre to edge coordinates (0 = the image's left/top edge), then
// scaled onto where the shown image sits in the view (object-fit: contain, so letterboxed and centred).

const FEATURE_STATES = ['no_depth', 'depth', 'matched', 'outlier', 'inlier'];

// Where an image of size [w, h] sits in a width x height box (CSS px) with object-fit: contain
function featureContainRect(size, width, height) {
  const s = Math.min(width / size[0], height / size[1]);
  const dw = size[0] * s, dh = size[1] * s;
  return { x: (width - dw) / 2, y: (height - dh) / 2, w: dw, h: dh };
}

// [x, y] in the view of a point [u, v, ...] of a features event f, the shown image at rect
function featureToView(point, f, rect) {
  const grid = f.frame_size;
  const image = f.image_size || grid;
  const u = point[0] * image[0] / grid[0], v = point[1] * image[1] / grid[1];
  return [rect.x + (u + 0.5) * rect.w / image[0], rect.y + (v + 0.5) * rect.h / image[1]];
}

// One feature's marker; the styles match the keys (.dot.inlier etc. in the CSS)
function drawFeatureMarker(ctx, state, x, y) {
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

// All of f's points onto ctx (CSS px), the best states drawn last (on top)
function drawFeatures(ctx, f, rect) {
  if (!f || !f.frame_size || !f.points) return;
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 1;
  for (let state = 0; state < FEATURE_STATES.length; state++) {
    for (const p of f.points) {
      if (p[2] === state) {
        const xy = featureToView(p, f, rect);
        drawFeatureMarker(ctx, state, xy[0], xy[1]);
      }
    }
  }
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { FEATURE_STATES, featureContainRect, featureToView, drawFeatureMarker, drawFeatures };
}
