// 3D pose viewer: the PnP VIO's integrated pose (api/stream/pose, pose_stream.py) over a grid, with the
// trail, the vehicle's orientation axes and the camera frustum. The main page's "3D pose" panel and the
// /widget/pose Cockpit widget both mount it. three.js r156 (MIT) is vendored in vendor/three; every URL is
// relative to this module or to `base` (the extension's base path), so it works on port 8095 and behind
// BlueOS's /extensionv2/<name>/ proxy. The stream is only open while the viewer is on screen.
//
// Frames (p.frame): "NED" from the VIO start position (x north, y east, z down) once the autopilot's
// ATTITUDE has been read, else "start-FRD", the body frame at VIO start (x forward, y right, z down; NED
// with north at the initial heading if the vehicle started level). The attitude comes from the autopilot
// or, while it is missing or stale, is gyro-integrated and drifts (p.attitude_source, shown in the status
// line). Angles are shown in degrees (p.roll_deg ...; p.roll ... are radians). three.js is y-up: a frame
// vector (x, y, z) is drawn at (y, -z, -x), so x points into the screen from the default view, y is right
// and down is down. The same proper rotation maps the attitude quaternion [w, x, y, z] to three's
// (x: y, y: -z, z: -x, w). The grid is the start depth (z = 0), 1 m cells.

import * as THREE from './vendor/three/three.module.min.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

const MAX_TRAIL = 20000;
// Same spacing as the server's trail (pose_stream.TRAIL_STEP_M / TRAIL_STEP_S)
const TRAIL_STEP_M = 0.05;
const TRAIL_STEP_S = 2.0;
const STALE_MS = 3000;
const DEG = 180 / Math.PI;
const HFOV = 70 / DEG;
const VFOV = 45 / DEG;
const FRUSTUM_M = 0.8;
const GRID_SEGMENT = 2;
const COLORS = { x: 0xff4d4d, y: 0x3ddc84, z: 0x4d8dff, trail: 0xffb020, frustum: 0xdbe7f3 };

const STYLE = `
.pv { position: relative; overflow: hidden; background: #0b1622; color: #dbe7f3;
      font: 12px/1.35 system-ui, -apple-system, "Segoe UI", sans-serif; min-height: 220px; }
.pv.pv-panel { height: 380px; border-radius: 6px; }
.pv.pv-fill { position: absolute; inset: 0; }
.pv canvas { display: block; width: 100%; height: 100%; touch-action: none; }
.pv-view { position: absolute; inset: 0; }
.pv-box { position: absolute; background: rgba(8, 16, 26, 0.78); border-radius: 5px; padding: 4px 7px; }
.pv-status { top: 6px; left: 6px; right: 6px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.pv-readout { left: 6px; bottom: 6px; font-variant-numeric: tabular-nums; }
.pv-readout table { border-collapse: collapse; }
.pv-readout td { padding: 0 4px; text-align: right; }
.pv-readout td.k { color: #8aa0b5; text-align: left; }
.pv-tools { right: 6px; bottom: 6px; display: flex; gap: 5px; align-items: center; }
.pv-tools button { background: #1c2b3a; color: #dbe7f3; border: 1px solid #35506b; border-radius: 4px;
                   padding: 2px 7px; font: inherit; cursor: pointer; }
.pv-tools button:hover { background: #26394c; }
.pv-tools label { display: flex; gap: 3px; align-items: center; cursor: pointer; }
.pv-message { top: 50%; left: 50%; transform: translate(-50%, -50%); max-width: 80%; text-align: center; }
.pv-message:empty { display: none; }
.pv-legend { top: 30px; right: 6px; color: #8aa0b5; }
.pv-legend b { font-weight: 600; }
.pv-x { color: #ff4d4d; } .pv-y { color: #3ddc84; } .pv-z { color: #4d8dff; } .pv-t { color: #ffb020; }
.pv-muted { color: #8aa0b5; }
.pv-warn { color: #ffb020; }
`;

// Axis names of each frame for the legend
const FRAME_AXES = { NED: ['x north', 'y east', 'z down'], 'start-FRD': ['x fwd', 'y right', 'z down'] };

/** The legend for a frame: its name and the colours of its axes. */
export function frameLegend(frame) {
  const name = FRAME_AXES[frame] ? frame : 'start-FRD';
  const [x, y, z] = FRAME_AXES[name];
  return `${name}: <b class="pv-x">${x}</b> <b class="pv-y">${y}</b> <b class="pv-z">${z}</b>` +
    ' · <b class="pv-t">trail</b> · grid 1 m';
}

/** "attitude: ..." for the status line; a warning while gyro-integrated. */
export function attitudeText(p) {
  if (!p || !p.attitude_source) return '';
  if (p.attitude_source === 'autopilot') {
    const age = Number.isFinite(p.attitude_age_s) ? ` (${p.attitude_age_s.toFixed(1)} s old)` : '';
    return `<span title="roll, pitch, yaw from the autopilot (mavlink2rest)">attitude: ArduPilot ATTITUDE${age}</span>`;
  }
  const why = p.attitude_mode === 'autopilot'
    ? (Number.isFinite(p.attitude_age_s) ? `, autopilot ATTITUDE ${p.attitude_age_s.toFixed(0)} s old` : ', no autopilot ATTITUDE')
    : '';
  return `<b class="pv-warn" title="gyro rotation chained from the start or from the last autopilot attitude; ` +
    `no bias correction">attitude: gyro-integrated, drifting${why}</b>`;
}

/** A frame vector (x, y, z) in three.js coordinates. */
export function toThree(x, y, z) {
  return new THREE.Vector3(y, -z, -x);
}

/** An attitude quaternion [w, x, y, z] (body -> frame) in three.js coordinates. */
export function quaternionToThree(q) {
  return new THREE.Quaternion(q[2], -q[3], -q[1], q[0]);
}

function injectStyle() {
  if (document.getElementById('pv-style')) return;
  const style = document.createElement('style');
  style.id = 'pv-style';
  style.textContent = STYLE;
  document.head.appendChild(style);
}

// Like THREE.GridHelper, but every line is cut into GRID_SEGMENT pieces: some WebGL renderers (software ones
// among them) drop a whole line that passes behind the camera, and the camera is usually inside the grid.
function gridLines(size, divisions, centerColor, color) {
  const half = size / 2, step = size / divisions, positions = [], colors = [];
  const center = new THREE.Color(centerColor), other = new THREE.Color(color);
  for (let i = 0; i <= divisions; i++) {
    const k = -half + i * step, c = 2 * i === divisions ? center : other;
    for (let a = -half; a < half; a += GRID_SEGMENT) {
      const b = Math.min(a + GRID_SEGMENT, half);
      positions.push(a, 0, k, b, 0, k, k, 0, a, k, 0, b);
      for (let n = 0; n < 4; n++) colors.push(c.r, c.g, c.b);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  return new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ vertexColors: true }));
}

function axisLines(length) {
  // x, y, z of the frame (or of the body, inside the vehicle group) as red, green, blue segments
  const positions = [], colors = [];
  const add = (v, hex) => {
    const c = new THREE.Color(hex);
    positions.push(0, 0, 0, v.x, v.y, v.z);
    colors.push(c.r, c.g, c.b, c.r, c.g, c.b);
  };
  add(toThree(length, 0, 0), COLORS.x);
  add(toThree(0, length, 0), COLORS.y);
  add(toThree(0, 0, length), COLORS.z);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  return new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ vertexColors: true }));
}

function frustumPositions(cameraAngleDeg) {
  // The camera looks along body x pitched down by camera_angle (0 forward, 90 down)
  const a = (cameraAngleDeg || 0) / DEG;
  const dir = [Math.cos(a), 0, Math.sin(a)], up = [Math.sin(a), 0, -Math.cos(a)], right = [0, 1, 0];
  const hw = FRUSTUM_M * Math.tan(HFOV / 2), hh = FRUSTUM_M * Math.tan(VFOV / 2);
  const corner = (sx, sy) => toThree(...[0, 1, 2].map(i => FRUSTUM_M * dir[i] + sx * hw * right[i] + sy * hh * up[i]));
  const c = [corner(-1, 1), corner(1, 1), corner(1, -1), corner(-1, -1)];
  const o = new THREE.Vector3();
  const segments = [[o, c[0]], [o, c[1]], [o, c[2]], [o, c[3]], [c[0], c[1]], [c[1], c[2]], [c[2], c[3]], [c[3], c[0]]];
  // A tick on the image's top edge shows which way is up
  const top = c[0].clone().lerp(c[1], 0.5);
  segments.push([top, top.clone().add(toThree(...up.map(v => 0.12 * v)))]);
  return segments.flatMap(([p, q]) => [p.x, p.y, p.z, q.x, q.y, q.z]);
}

function formatAge(ms) {
  return ms < 1500 ? '' : ` · ${(ms / 1000).toFixed(0)} s old`;
}

/**
 * Builds the viewer inside root. options: base (the extension's base path, ending in "/"), fill (fill the
 * root, for the widget page) and storage (a Storage for the follow and clear-trail choices).
 */
export function mountPoseViewer(root, options = {}) {
  injectStyle();
  const base = options.base || '';
  let storage = options.storage;
  if (storage === undefined) {
    try {
      storage = window.localStorage;  // throws in iframes whose storage the browser blocks
    } catch (e) {
      storage = null;
    }
  }
  root.innerHTML = `
    <div class="pv ${options.fill ? 'pv-fill' : 'pv-panel'}">
      <div class="pv-view"></div>
      <div class="pv-box pv-status">connecting…</div>
      <div class="pv-box pv-legend">${frameLegend('start-FRD')}</div>
      <div class="pv-box pv-readout" title="position in m, roll / pitch / yaw in degrees (ZYX)"><table>
        <tr><td class="k">x m</td><td data-v="x">–</td><td class="k">roll °</td><td data-v="roll">–</td></tr>
        <tr><td class="k">y m</td><td data-v="y">–</td><td class="k">pitch °</td><td data-v="pitch">–</td></tr>
        <tr><td class="k">z m</td><td data-v="z">–</td><td class="k">yaw °</td><td data-v="yaw">–</td></tr>
      </table></div>
      <div class="pv-box pv-tools">
        <button type="button" data-act="reset" title="Look at the vehicle and its trail again">Reset view</button>
        <button type="button" data-act="clear" title="Clear the drawn trail (this browser)">Clear trail</button>
        <label title="Keep the vehicle in the middle of the view"><input type="checkbox" data-act="follow"> Follow</label>
      </div>
      <div class="pv-box pv-message"></div>
    </div>`;
  const el = root.querySelector('.pv');
  const view = el.querySelector('.pv-view');
  const statusBox = el.querySelector('.pv-status');
  const legend = el.querySelector('.pv-legend');
  const message = el.querySelector('.pv-message');
  const values = Object.fromEntries([...el.querySelectorAll('[data-v]')].map(n => [n.dataset.v, n]));
  const follow = el.querySelector('[data-act="follow"]');
  follow.checked = storage?.getItem('oakd-pose-follow') === '1';

  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (e) {
    message.textContent = `3D view unavailable: no WebGL in this browser (${e.message || e})`;
    return { destroy() {} };
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x0b1622);
  view.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 1000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;

  scene.add(gridLines(40, 40, 0x5a7a99, 0x2e4560));
  scene.add(axisLines(1.0));

  const vehicle = new THREE.Group();
  vehicle.add(axisLines(0.5));
  const hull = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.12, 0.4),  // right, up, back in three's axes
                              new THREE.MeshBasicMaterial({ color: 0x9fb4c8, transparent: true, opacity: 0.35 }));
  vehicle.add(hull);
  const frustumGeometry = new THREE.BufferGeometry();
  const frustum = new THREE.LineSegments(frustumGeometry, new THREE.LineBasicMaterial({ color: COLORS.frustum }));
  vehicle.add(frustum);
  scene.add(vehicle);

  // From the vehicle straight up or down to the grid: shows the depth change
  const dropGeometry = new THREE.BufferGeometry();
  dropGeometry.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
  const dropLine = new THREE.Line(dropGeometry, new THREE.LineDashedMaterial({ color: 0x4d8dff, dashSize: 0.1, gapSize: 0.08 }));
  scene.add(dropLine);

  const trailPositions = new Float32Array(MAX_TRAIL * 3);
  const trailGeometry = new THREE.BufferGeometry();
  trailGeometry.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  trailGeometry.setDrawRange(0, 0);
  scene.add(new THREE.Line(trailGeometry, new THREE.LineBasicMaterial({ color: COLORS.trail })));

  const state = { es: null, active: false, inView: true, frame: null, epoch: null, clearedAt: 0, trail: [],
                  pose: null, poseAt: 0, problem: null, cameraAngle: null, mock: false, shownFrame: 'start-FRD' };

  function setFrustum(angle) {
    if (angle === state.cameraAngle) return;
    state.cameraAngle = angle;
    frustumGeometry.setAttribute('position', new THREE.Float32BufferAttribute(frustumPositions(angle), 3));
  }
  setFrustum(0);

  // --- trail ---

  function writeTrail() {
    const points = state.trail;
    for (let i = 0; i < points.length; i++) {
      const v = toThree(points[i][1], points[i][2], points[i][3]);
      trailPositions[3 * i] = v.x;
      trailPositions[3 * i + 1] = v.y;
      trailPositions[3 * i + 2] = v.z;
    }
    trailGeometry.setDrawRange(0, points.length);
    trailGeometry.attributes.position.needsUpdate = true;
    trailGeometry.computeBoundingSphere();
  }

  function addTrailPoint(t, x, y, z) {
    if (t <= state.clearedAt) return;
    const last = state.trail[state.trail.length - 1];
    if (last) {
      const moved = Math.hypot(x - last[1], y - last[2], z - last[3]);
      if (moved < TRAIL_STEP_M && !(moved > 0 && t - last[0] >= TRAIL_STEP_S)) return;
    }
    state.trail.push([t, x, y, z]);
    if (state.trail.length > MAX_TRAIL) {
      state.trail = state.trail.filter((_, i) => i % 2 === 0 || i === state.trail.length - 1);
      writeTrail();
      return;
    }
    const i = state.trail.length - 1, v = toThree(x, y, z);
    trailPositions.set([v.x, v.y, v.z], 3 * i);
    trailGeometry.setDrawRange(0, state.trail.length);
    trailGeometry.attributes.position.needsUpdate = true;
    trailGeometry.computeBoundingSphere();
  }

  function newEpoch(epoch) {
    state.epoch = epoch;
    state.trail = [];
    state.clearedAt = Number(storage?.getItem('oakd-pose-cleared-' + epoch)) || 0;
    writeTrail();
  }

  function onTrail(data) {
    if (data.epoch !== state.epoch) newEpoch(data.epoch);
    const points = (data.points || []).filter(p => p[0] > state.clearedAt);
    const lastT = points.length ? points[points.length - 1][0] : -Infinity;
    state.trail = points.concat(state.trail.filter(p => p[0] > lastT)).slice(-MAX_TRAIL);
    writeTrail();
  }

  function clearTrail() {
    state.clearedAt = state.pose ? state.pose.unix_time : Date.now() / 1000;
    if (state.epoch) storage?.setItem('oakd-pose-cleared-' + state.epoch, String(state.clearedAt));
    state.trail = [];
    writeTrail();
  }

  // --- pose ---

  function onPose(p) {
    if (p.epoch !== state.epoch) newEpoch(p.epoch);
    const position = toThree(p.x, p.y, p.z);
    if (follow.checked) {
      const shift = position.clone().sub(controls.target);
      controls.target.add(shift);
      camera.position.add(shift);
    }
    vehicle.position.copy(position);
    vehicle.quaternion.copy(quaternionToThree(p.q));
    setFrustum(p.camera_angle_deg || 0);
    const drop = dropGeometry.attributes.position;
    drop.setXYZ(0, position.x, position.y, position.z);
    drop.setXYZ(1, position.x, 0, position.z);
    drop.needsUpdate = true;
    dropLine.computeLineDistances();
    addTrailPoint(p.unix_time, p.x, p.y, p.z);
    const firstPose = !state.pose;
    state.pose = p;
    state.poseAt = Date.now();
    values.x.textContent = p.x.toFixed(2) + ' m';
    values.y.textContent = p.y.toFixed(2) + ' m';
    values.z.textContent = p.z.toFixed(2) + ' m';
    for (const k of ['roll', 'pitch', 'yaw']) {
      values[k].textContent = (Number.isFinite(p[k + '_deg']) ? p[k + '_deg'] : p[k] * DEG).toFixed(1) + '°';
    }
    if (p.frame !== state.shownFrame) {
      state.shownFrame = p.frame;
      legend.innerHTML = frameLegend(p.frame);
      legend.title = p.frame_text || '';
    }
    if (firstPose) resetView();
    showStatus();
  }

  function onStatus(s) {
    state.problem = s.problem || null;
    state.mock = !!s.mock;
    showStatus();
  }

  function showStatus() {
    const p = state.pose, age = p ? Date.now() - state.poseAt : Infinity;
    const stale = age > STALE_MS;
    const tracking = p ? (p.ok ? 'tracking' : 'not tracking (pose held)') : '';
    statusBox.innerHTML = p
      ? `${state.mock ? '<b>mock</b> · ' : ''}${tracking} · ${attitudeText(p)} · ` +
        `${p.ok_frames}/${p.frames} frames with motion<span class="pv-muted">${formatAge(age)}</span>`
      : (state.mock ? '<b>mock</b> · ' : '') + 'waiting for the pose…';
    message.textContent = state.problem ? state.problem : (p && stale ? `no pose for ${(age / 1000).toFixed(0)} s` : '');
  }

  // --- view ---

  function resetView() {
    const box = new THREE.Box3();
    box.expandByPoint(vehicle.position);
    for (const p of state.trail) box.expandByPoint(toThree(p[1], p[2], p[3]));
    const size = box.getSize(new THREE.Vector3()).length();
    const target = follow.checked ? vehicle.position.clone() : box.getCenter(new THREE.Vector3());
    const distance = Math.max(4, 1.1 * size);
    controls.target.copy(target);
    camera.position.copy(target).add(new THREE.Vector3(-0.35, 0.6, 1).normalize().multiplyScalar(distance));
    camera.lookAt(target);
    controls.update();
  }
  resetView();

  el.querySelector('[data-act="reset"]').addEventListener('click', resetView);
  el.querySelector('[data-act="clear"]').addEventListener('click', clearTrail);
  follow.addEventListener('change', () => {
    storage?.setItem('oakd-pose-follow', follow.checked ? '1' : '0');
    if (follow.checked) resetView();
  });

  function resize() {
    const w = view.clientWidth || 1, h = view.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(view);
  resize();

  // --- stream and render loop, only while on screen ---

  let lastStatusCheck = 0;
  function loop(now) {
    state.frame = requestAnimationFrame(loop);
    controls.update();
    renderer.render(scene, camera);
    if (now - lastStatusCheck > 500) {
      lastStatusCheck = now;
      showStatus();
    }
  }

  function connect() {
    if (state.es) return;
    const es = new EventSource(base + 'api/stream/pose?client=' + (options.fill ? 'widget' : 'page'));
    es.addEventListener('pose', e => onPose(JSON.parse(e.data)));
    es.addEventListener('trail', e => onTrail(JSON.parse(e.data)));
    es.addEventListener('status', e => onStatus(JSON.parse(e.data)));
    es.onerror = () => { if (!state.pose) statusBox.textContent = 'reconnecting…'; };
    state.es = es;
  }

  function setActive() {
    const active = state.inView && document.visibilityState !== 'hidden';
    if (active === state.active) return;
    state.active = active;
    if (active) {
      connect();
      state.frame = requestAnimationFrame(loop);
    } else {
      if (state.es) state.es.close();
      state.es = null;
      cancelAnimationFrame(state.frame);
    }
  }
  const intersection = new IntersectionObserver(entries => {
    state.inView = entries.some(entry => entry.isIntersecting);
    setActive();
  });
  intersection.observe(el);
  document.addEventListener('visibilitychange', setActive);
  setActive();

  return {
    destroy() {
      state.inView = false;
      setActive();
      intersection.disconnect();
      resizeObserver.disconnect();
      document.removeEventListener('visibilitychange', setActive);
      renderer.dispose();
    },
  };
}
