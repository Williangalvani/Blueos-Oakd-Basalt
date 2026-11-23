# BlueOS OAK-D Visual Odometry

A [BlueOS](https://github.com/bluerobotics/BlueOS) extension that runs stereo visual-inertial odometry (VIO) on a Luxonis OAK-D camera and feeds the result to an ArduPilot autopilot over MAVLink.

It uses the [depthai-core](https://github.com/luxonis/depthai-core) `BasaltVIO` node with the OAK-D's two mono cameras and IMU, and sends the estimated motion as `VISION_POSITION_DELTA` messages through BlueOS's mavlink2rest.

## What it does on startup

1. Starts a configuration web page on port **8095** (8080 is used by mavlink-server on BlueOS 1.4.x).
2. Configures the autopilot's EKF to use visual odometry (see below).
3. Starts the VIO pipeline with the saved settings and begins sending MAVLink.

## Autopilot parameters

Every time VIO starts, the extension reads these parameters from the autopilot (system 1, component 1) and writes any that differ:

| Parameter | Value | Meaning | Needs reboot |
|---|---|---|---|
| `AHRS_EKF_TYPE` | 3 | Use EKF3 | yes |
| `EK3_ENABLE` | 1 | Enable EKF3 | yes |
| `VISO_TYPE` | 1 | Visual odometry over MAVLink | yes |
| `EK3_SRC1_POSXY` | 6 | Horizontal position from ExternalNav | no |
| `EK3_SRC1_VELXY` | 6 | Horizontal velocity from ExternalNav | no |
| `EK3_SRC1_POSZ` | 1 | Depth/altitude from the barometer | no |

- Parameters are only changed while the vehicle is **disarmed**. If it is armed, the step is skipped and a warning is logged.
- Each write is confirmed against the autopilot's `PARAM_VALUE` reply.
- If a parameter that needs a reboot was changed, the log says so. The extension never reboots the autopilot itself. Reboot it from BlueOS or your ground station once after the first install.
- Any custom EKF source setup on the vehicle will be overwritten on each start. To change the list, edit `VISUAL_ODOMETRY_PARAMS` in [`app/autopilot_params.py`](app/autopilot_params.py).

The log lines look like this (BlueOS → Extensions → Installed → the extension's logs):

```
VISO_TYPE: 0 -> 1
EK3_SRC1_POSXY: 0 -> 6
EK3_SRC1_VELXY: 3 -> 6
EK3_SRC1_POSZ = 1 (already set)
Reboot the autopilot for VISO_TYPE to take effect
```

### Suggested parameters for optical position hold (written only from the page)

`SUGGESTED_OPTICAL_HOLD_PARAMS` in [`app/autopilot_params.py`](app/autopilot_params.py) lists what position hold on `VISION_POSITION_DELTA` should use: name, suggested value, whether it needs a reboot, and why. `diff_params(current)` compares it with a `{name: value}` dict and returns the entries that differ or are missing, with status `change` or `missing`. Float32 rounding is ignored. The page's [ArduPilot panel](#ardupilot-panel) shows them next to the vehicle's values and writes them only when you confirm, while the vehicle is disarmed. Nothing writes them at startup, and the automatic setup above is unchanged.

| Parameter | Suggested | Reboot | Why |
|---|---|---|---|
| `VISO_TYPE` | 1 | yes | MAVLink visual odometry. |
| `VISO_POS_X/Y/Z` | 0.1655, −0.0069, 0.180 | no | Camera relative to the IMU, m FRD, from the CAD. EKF3 applies this lever arm itself (see [Lever arm](#lever-arm-viso_pos-pnp)). |
| `VISO_ORIENT` | 0 | no | The extension already rotates into body FRD. |
| `VISO_DELAY_MS` | keep | no | Unverified; tune from logs. |
| `EK3_SRC1_VELXY` | 6 | no | Fuse the visual body velocity. |
| `EK3_SRC1_POSXY` | 0 | no | None: the EKF dead-reckons a relative position from the visual velocity, which ArduSub accepts for position hold with visual odometry enabled. 3 (GPS) only helps with a fix at the surface. |
| `EK3_SRC1_POSZ` | 1 | no | Baro (depth). |
| `EK3_SRC1_YAW` | 1 | no | Compass: the deltas carry no heading. |

The automatic setup sets `EK3_SRC1_POSXY` to 6 (ExternalNav). That asks for an absolute position this extension never sends; with only `VISION_POSITION_DELTA` the EKF ends up in relative aiding, as with 0.

## MAVLink output

Messages are sent with component ID 195 (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`) to `ws://<vehicle_ip>/mavlink2rest/ws/mavlink`.

- `VISION_POSITION_DELTA`: position change since the previous frame, in the body frame (front-right-down), one per camera frame. `angle_delta` is not computed and is always zero.
- `NAMED_VALUE_FLOAT`, rate-limited by the *Quality Rate* setting (MAVLink limits names to 10 characters; `time_boot_ms` counts from the VIO process start):
  - `VIO_QUAL`: average tracking quality, 0 to 1
  - `VIO_FEAT`: tracked features / 1000, capped at 1
  - `VIO_LMKS`: landmarks / 10000, capped at 1
  - `VIO_PROC`: processing time of the last frame, in seconds
  - `VIO_TRACK`: 1 while tracking, 0 when lost

With the experimental **PnP** algorithm (see [NOTES_OPTION_D.md](NOTES_OPTION_D.md)), `VISION_POSITION_DELTA` also carries `angle_delta` and an inlier-based `confidence`, and the metrics are:
  - `VIO_TRACK`: 1 while tracking, 0 when lost (also sent immediately on change)
  - `VIO_FEAT`: number of features tracked on the camera (raw count)
  - `VIO_INLIER`: PnP RANSAC inliers of the last frame
  - `VIO_CONF`: confidence of the last frame, 0 to 100
  - `VIO_PROCMS`: host processing time of the last frame, in milliseconds

PnP's `VISION_POSITION_DELTA` fields: `position_delta` in metres and `angle_delta` in radians, both body FRD over the interval since the reference frame; `time_delta_usec`, that interval from device timestamps, rounded to whole µs; `confidence` 0–100; `time_usec`, host wall time when sent. How ArduPilot (EKF3) uses them, from the source:

- Velocity = `position_delta` / `time_delta_usec`, fused as body-frame velocity, with the `VISO_POS_*` lever arm applied by the EKF (see [Lever arm](#lever-arm-viso_pos-pnp)). `VISO_ORIENT` rotates both vectors, so it must stay 0 (Forward): the extension already applies *Camera Angle*.
- `time_usec` is ignored. The message is stamped on arrival minus `VISO_DELAY_MS` (default 10 ms). A delta is the average over its interval, so its true delay is about half a frame interval (167 ms at 3 fps) plus the pipeline latency. `VISO_DELAY_MS` (0–250) is not tuned yet.
- `confidence` sets the velocity noise linearly: `EK3_VIS_VERR_MIN + (EK3_VIS_VERR_MAX − EK3_VIS_VERR_MIN)·(1 − confidence/100)`. `VISO_QUAL_MIN` does **not** apply to `VISION_POSITION_DELTA`.
- `angle_delta` is stored but not fused. The EKF uses its own gyro, also for the lever-arm term.
- Messages less than 50 ms apart are dropped. To start using body odometry, the EKF also wants one within the last 200 ms, which 3 fps only meets part of the time. Use 5 fps or more for position hold.

## Configuration page

Open `http://<vehicle>:8095` (or the extension's entry in the BlueOS menu). Saving restarts VIO with the new values. Settings are stored in `/usr/blueos/extensions/oakd-vio/config.json` on the vehicle, so they survive restarts and updates.

The page is one self-contained file (`app/static/ui.html`, `ui.css` and `ui.js`, inlined by `main.py`): no CDN, no build step, so it works offline on a boat. It has these panels (see [Web UI](#web-ui-pnp) for how the live data is gathered):

- **Live status**: camera connected, VIO running, fps, inliers, frames ok (last 10 s), processing time, features / valid depth, matches, refractive index, depth scale, the effective camera matrix, reference resets / gap frames, recoveries, Pi CPU and the camera chip temperature; failures by reason over the last 5 minutes; the output mode, messages sent, rate and last error, a *Log-only* toggle (saved, restarts VIO) and *Reset VIO*.
- **Live views**: `rect_left` with the tracked features of the last processed frame (inliers green, outliers red, unmatched grey, with a line from the previous position) and the colorized depth, refreshed on demand or about once a second; the depth probe (click the image to aim it) and the focal check.
- **Trajectory** (integrated from the deltas and the gyro yaw; it drifts), a **velocity** panel in the layout of the Water Linked DVL A50 widgets, **vx / vy over time** with an optional DVL overlay, and **inliers over time** with `min_inliers`.
- **Settings**, generated from `GET /api/config/schema`, checked in the browser and again on the server; *Apply* restarts VIO.
- **ArduPilot parameters**: the suggested optical-hold parameters next to the vehicle's values, with a guarded write.
- **Data logs**.

| Setting | Default | Notes |
|---|---|---|
| Algorithm | Basalt VIO | *Basalt VIO*, or the experimental *PnP* (on-device stereo depth and feature tracking, PnP on the host). |
| FPS | 3 | Camera frame rate. The Raspberry Pi CPU is the limit; raise it slowly. |
| Camera Angle (°) | 0 | Downward pitch of the camera. 0 = facing forward, 90 = facing down. |
| Width × Height | 320 × 200 | Mono camera resolution fed to VIO. |
| Optical Flow Grid Size | 80 | Larger means fewer features and less CPU. Basalt default is 50. |
| Flow Iterations | 3 | Optical flow iterations per level. |
| Pyramid Levels | 2 | Optical flow image pyramid levels. |
| Skip Frames | 1 | Run optical flow on every Nth frame. |
| Max States | 2 | Sliding window size. |
| Max Keyframes | 4 | Keyframes kept in the optimization. |
| VIO Iterations | 3 | Optimizer iterations per frame. |
| PnP Features | 300 | PnP only: target number of features tracked on the camera. The Basalt settings above are ignored in PnP mode. |
| PnP Min Inliers | 15 | PnP only: RANSAC inliers a frame needs to be accepted and sent (`--min-inliers`, 6–500). The inliers plot draws it. |
| PnP Recovery After (frames) | 20 | PnP only: consecutive failed frames before [tracking recovery](#tracking-recovery-pnp); 0 disables it. |
| PnP Max Reference Age (s) | 1.0 | PnP only: how long [failed frames](#failed-frames-pnp) keep the last good frame as the reference (`--max-reference-age`); 0 removes the limit. |
| Snapshots | on | PnP only: allow [camera snapshots](#snapshots-pnp). Nothing is streamed until one is requested. |
| Live depth stream | on | PnP only: allow the page's [live depth stream](#live-depth-stream), MJPEG encoded on the camera. Nothing is sent until a client watches (`--depth-stream`). |
| Depth stream fps | 5 | PnP only: target rate of the depth stream (0.5–15; every n-th camera frame, `--depth-stream-fps`). |
| Depth stream JPEG quality | 70 | PnP only: the camera's MJPEG quality (10–100, `--depth-stream-quality`). |
| PnP Intrinsics Source | rectified | PnP only: where the rectified-left camera matrix comes from (`--intrinsics-from`). See [Camera matrix](#camera-matrix-pnp). |
| PnP Intrinsics Override | empty | PnP only: `fx,fy,cx,cy` in pixels on the width × height grid (`--intrinsics-override`). Wins over the source. Empty = off. |
| PnP Depth Scale | 1.0 | PnP only: extra trim on the camera depth before back-projection, on top of the refractive index (`--depth-scale`, 0.2–5). Translation scales with it. Use it for a residual measured with the [depth probe](#depth-probe-pnp). |
| PnP Refractive Index (water) | 1.333 | PnP only: the water in front of the flat port (`--refractive-index`, 1–2). 1.333 is fresh water and about 1.34 sea water. **Set 1.0 for tests in air.** See [Water (refractive index)](#water-refractive-index-pnp). |
| Lever arm diagnostics (log only) | off | PnP only, **diagnostics only**: also log each delta corrected to the IMU (`--lever-arm`). It never changes what is sent; the autopilot applies `VISO_POS_X/Y/Z` itself. See [Lever arm](#lever-arm-viso_pos-pnp). |
| Lever Arm x,y,z (m) | 0.1655,-0.0069,0.18 | Camera relative to the IMU in body FRD, from the CAD (`--lever-arm-m`). Only used by the diagnostics. |
| Vehicle IP | 127.0.0.1 | Where mavlink2rest is reachable. The extension uses host networking, so the default works on the vehicle itself. |
| Quality Rate (Hz) | 1 | Rate of the `NAMED_VALUE_FLOAT` quality messages. |
| Log only | off | Run VIO as a passive logger: no autopilot parameter changes and no MAVLink output at all. See [Log-only mode](#log-only-mode). |
| DVL overlay source | auto | *auto*: plot a DVL's velocity dashed over the VIO velocity, if mavlink2rest has one (see [DVL overlay](#dvl-overlay)); *off*: never poll. |

The defaults are tuned for low CPU usage on a Raspberry Pi.

### Stereo depth (PnP)

The *Stereo Depth (PnP)* section configures the on-camera `StereoDepth` node. The preset is applied first, then every setting below overrides it. The defaults aim for sparse but trustworthy depth at the tracked features, not a dense depth map: no hole filling, and depth stays on the same full-resolution rectified-left pixel grid the feature tracker runs on. Empty optional fields mean "keep the preset or device value". Each setting is passed to `pnp_vio.py` as a command-line flag (`--stereo-...`; `pnp_vio.py --help` lists them).

| Setting | Default | Notes |
|---|---|---|
| Stereo preset | ROBOTICS | `setDefaultProfilePreset`; NONE skips it. On RVC2, ROBOTICS also turns on decimation 2 and the spatial filter, which the defaults below turn off again. |
| Left-right check / threshold | on / 5 | Max disparity difference (px) between the left and right matches. |
| Confidence threshold | 50 | 0–255; pixels with a worse confidence score are invalidated, so lower is stricter. |
| Min / max brightness | auto / 255 | Pixels at or below min, or at or above max, are invalidated. Auto min is 1 when the edge fill color is 0, which drops the black rectification border, otherwise 0. |
| Speckle filter / range | on / 50 | |
| Min / max depth (mm) | 300 / 4000 | Threshold filter. Depth outside the range is set to 0. |
| Spatial filter, temporal filter | off, off | Spatial fills holes by smoothing across edges; temporal lags behind motion. |
| Decimation | 1 | 2–4 make depth coarser than the feature grid. Feature depth lookups are scaled to it, at a cost in accuracy. |
| Subpixel / bits | on / 3 | The median filter needs 3 bits; with 4 or 5 it is turned off (with a warning). |
| Median filter | 7x7 | off, 3x3, 5x5 or 7x7. |
| Distortion correction | on | |
| Alpha scaling | 0 | 0 crops the rectified image to valid pixels only. Empty keeps the device default. |
| Edge fill color | 0 | Fill for the area outside the rectified image; -1 replicates edge pixels. |
| Horizontal P2 penalty | empty | Cost aggregation P2; empty keeps the preset value. |

*Distortion correction* and *Alpha scaling* decide the camera matrix of the rectified image, which PnP needs; see [Camera matrix](#camera-matrix-pnp). The requested settings, the values read back from the node, and any warnings are logged at startup and written to the data log as a `stereo_config` event. The JSONL `start` event also includes them.

### Camera matrix (PnP)

PnP back-projects each feature with the camera matrix (K) of the rectified-left image the features and depth are on. With distortion correction on and an alpha set (the defaults: on, 0), the camera rectifies onto OpenCV's `getOptimalNewCameraMatrix(K, D[:8], size, alpha)` of the CAM_B calibration. That matrix has fx ≠ fy. For the OAK-D-W-PoE at 640×400 and alpha 0 it is fx 163.1, fy 237.1, cx 329.4, cy 199.0; the raw calibration is fx = fy = 285.1, cx 323.2, cy 199.1.

Before this fix, PnP used the raw calibration, because the depth frame's own camera matrix turned out to be the raw one. With that wrong K, lateral translation came out at 0.57× and vertical at 0.83× the true value; forward was unaffected. The device depth confirms the rectified matrix: f = Z·d/B is about 167 px. See [NOTES_OPTION_D.md](NOTES_OPTION_D.md#rectified-intrinsics) for the depthai-core sources and the measurements.

*PnP Intrinsics Source* (`--intrinsics-from`):

| Source | Camera matrix |
|---|---|
| `rectified` (default) | Computed on the host from the CAM_B calibration and distortion coefficients and the configured alpha. The depth frame's matrix is only logged, with whether it equals the raw or the rectified K. If alpha is empty (device default) or distortion correction is off, the device's matrix is unknown. PnP then uses the depth frame's matrix and logs a warning that it is unverified. |
| `frame` | The depth frame's matrix. If it equals the raw CAM_B calibration while distortion correction and alpha are set, it is the unrectified K: PnP ignores it with a warning and uses the computed rectified K. |
| `cam_b`, `cam_c` | The raw calibration of that camera. Wrong whenever the image is rectified; kept for comparison. |

*PnP Intrinsics Override* (`--intrinsics-override fx,fy,cx,cy`) wins over all of them. Every fallback is logged as a warning, in the container log and as a JSONL `warning` event.

The chosen matrix and its source go to the container log and the JSONL `intrinsics` event. The raw and rectified matrices, alpha, distortion correction and the stereo baseline go there too. The same information is in `/api/status` under `camera`. The baseline comes from the calibration (`getBaselineDistance(CAM_C, CAM_B, useSpecTranslation=False)`) and is logged at startup next to the design value.

### Water (refractive index) (PnP)

The calibration is done in air, but underwater the camera looks through a flat port into water. To a first (paraxial) approximation, a ray at angle *a* in the water leaves the port at n·tan *a*. The in-air camera therefore images the water scene as if its focal lengths were n·fx and n·fy, and the device's stereo depth, Z = f·B/d with the in-air f, comes out at Z_true / n. PnP corrects both, after the camera matrix above (source or override) is chosen:

- K: fx and fy are multiplied by n; cx and cy are unchanged.
- Depth: multiplied by n × *PnP Depth Scale*. The device's min/max depth still apply to the uncorrected depth.
- The gyro prior is a rotation, so it is unaffected.

What this changes: with K and depth both scaled by n, the lateral back-projection X = (u − cx)·Z/fx does not change. So for the same pixel motion, n scales the translation **along the optical axis** by n, and leaves the sideways components alone (tested on a synthetic flat-port scene). For a camera looking down, that is the vertical component. In the old model, a depth scale of 1.33 scaled every axis, which was wrong for the lateral ones. Beyond the paraxial approximation a flat port also bends rays near the image edges; that is not modelled.

The in-air matrix, `effective_intrinsics` (with n), `refractive_index`, `depth_scale` and `total_depth_scale` (n × depth scale) are in the JSONL `intrinsics` event and in `/api/status` → `camera`. The page's status area shows them too. The focal check keeps using the in-air matrix, because the device depth is computed with the in-air focal length.

### Lever arm (VISO_POS) (PnP)

The camera is about 0.17 m forward and 0.18 m below the IMU. When the vehicle rotates, the camera swings on that lever arm, so its measured delta includes a displacement r − Rᵀr (to first order ω × r·dt) that the IMU did not make. **ArduPilot corrects this itself for `VISION_POSITION_DELTA`**, so the extension sends the camera's own delta (rotated into body FRD) and must not correct it a second time. In the ArduPilot source (`~/git/ardupilot`):

- `AP_VisualOdom_Backend::handle_vision_position_delta_msg` (`libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp`, ~l.62–69) passes `_frontend.get_pos_offset()`, i.e. `VISO_POS_X/Y/Z`, to `AP::ahrs().writeBodyFrameOdom(...)`.
- EKF3 (`libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`, ~l.1493–1498) predicts the sensor velocity as the body velocity plus `(delAng / delAngDT) × posOffsetBody`, with `posOffsetBody = VISO_POS − INS_POS`. So `VISO_POS_*` is relative to the vehicle origin; with `INS_POS_*` at 0 that is the IMU.

Set `VISO_POS_X/Y/Z` on the autopilot to the lever arm (0.1655, −0.0069, 0.180 m from the CAD; see [Suggested parameters](#suggested-parameters-for-optical-position-hold-written-only-from-the-page)). *Lever arm diagnostics* only adds JSONL step fields, for comparing the two in logs:

- `lever_delta_position`: the IMU delta, `position_delta − (r − Rᵀr)`, body FRD, m;
- `lever_delta_correction`: the `r − Rᵀr` term;
- `lever_delta_rotation`: `gyro` (the gyro rotation over the delta's interval) or `pnp` (the PnP rotation, when there is no gyro prior).

They are `null` on frames without a delta, absent when the setting is off, and never sent.

Depth is not gated with the stereo confidence map. On the camera, the `confidenceMap` output gets no left-right check, alignment or post-processing, so it doesn't match the depth pixels. Streaming it would also add about 20 Mbit/s, and the confidence threshold above already applies the same score on the camera.

The page is backed by a small HTTP API:

| Method | Path | Action |
|---|---|---|
| GET | `/api/config` | Current settings |
| GET | `/api/config/schema` | Every setting: `name`, `type`, `min`, `max`, `default`, `help`, `label`, `group`, `choices`, `nullable` |
| POST | `/api/config` | Validate and save settings (JSON body, same keys; missing keys get defaults) and restart VIO. 422 lists every invalid field: `detail: [{field, msg, loc, type}]`, e.g. `FPS (fps): must be between 1 and 60 (got 0)`; nothing is saved |
| POST | `/api/log_only` | `{"log_only": true\|false}`: save the setting and restart VIO; 400 for anything else |
| GET | `/api/status` | Whether the VIO process is running, restarts, whether log-only mode is active (`log_only`, `log_only_forced_by_env`), and for PnP the `camera` matrix in use (`null` until the first frame): the in-air `intrinsics`, `effective_intrinsics`, `refractive_index`, `depth_scale` and `total_depth_scale`. Also `algorithm`, `telemetry` (PnP live status, below; `null` until published, `stale: true` when VIO stopped), `camera_connected`, `chip_temperature_c`, `cpu_percent`, `uptime_s` and the last known `vehicle` armed state |
| GET | `/api/history?seconds=60&max_points=600&tail=0` | PnP plot history, oldest first, `t` in seconds before `server_time`: `vx`, `vy`, `vz` (m/s, body FRD, `null` for rejected frames), integrated `x`, `y`, `yaw`, `inliers`, `ok`. At most `max_points` (≤ 600) over `seconds` (≤ 600); `tail` (≤ 10 s) adds the newest frames at full rate. `dvl` holds the [DVL overlay](#dvl-overlay). Weak `ETag`; `If-None-Match` gives 304 |
| POST | `/api/autopilot/params/read` | The suggested optical-hold parameters as the vehicle has them, and the HEARTBEAT armed state. Read only (POST because it sends MAVLink requests). See [ArduPilot panel](#ardupilot-panel) |
| POST | `/api/autopilot/params` | `{"confirm": true, "params": {"NAME": value}}`: guarded write with read-back. See [ArduPilot panel](#ardupilot-panel) |
| GET | `/api/logs` | Data logs: name, size (bytes), mtime (unix time), newest first |
| GET | `/api/logs/{name}` | Download one data log |
| POST | `/api/start`, `/api/stop` | Start or stop VIO |
| POST | `/api/reset` | Restore defaults and restart VIO |
| GET | `/api/snapshot?stream=<stream>` | PnP only: PNG of one current camera frame; `stream` is `rect_left`, `rect_right`, `depth` or `rgb`. `&crosshair=1&x=&y=&w=&h=` draws the depth probe's ROI; `&overlay=1` (rect_left) draws the tracked features. See [Snapshots](#snapshots-pnp). |
| POST | `/api/depth_probe?x=&y=&w=&h=&frames=` | PnP only: depth statistics in an ROI over the next frames. See [Depth probe](#depth-probe-pnp). |
| POST | `/api/focal_check?tolerance=` | PnP only: checks fx against the focal length the device depth implies. See [Focal check](#focal-check-pnp). |
| GET | `/api/stream/depth` | PnP only: live depth as `multipart/x-mixed-replace` MJPEG (grayscale, encoded on the camera); 503 when PnP is stopped or the stream is off. `?frames=N` ends after N frames. See [Live depth stream](#live-depth-stream). |
| GET | `/api/stream/features` | Server-sent events for the [Cockpit widget](#cockpit-widget): `features` (the PnP tracker's features, about 5 per second) and `status` (once a second; `problem` says why there is nothing to show). Always 200. `?events=N` ends after N events. |
| GET | `/api/features` | PnP only: the latest `features` object once; 404 until the VIO process has published one (it starts on request) |
| GET | `/widget/depth` | The [Cockpit widget](#cockpit-widget) page |
| GET | `/register_service`, `/cockpit_extras.json` | BlueOS service metadata and the Cockpit extras that declare the widget |

`/openapi.json` (and `/docs`) lists only the GET routes that answer at once with a bounded body and start nothing: `/`, `/widget/depth`, `/register_service`, `/cockpit_extras.json`, `/api/config`, `/api/config/schema`, `/api/status`, `/api/logs` and `/api/logs/{name}`. BlueOS's helper GETs every path listed there every few seconds and reads each body to its end, so the streams (`/api/stream/*`) and the routes that wait for the camera or start work (`/api/snapshot`, `/api/history`, `/api/features`) are left out of the schema, and everything that runs something on the camera or the autopilot is a POST. `app/tests/test_openapi_scan.py` runs that scan against the app and fails on any other GET in the schema.

### Snapshots (PnP)

`GET /api/snapshot?stream=rect_left|rect_right|depth|rgb` returns a fresh PNG from the running PnP VIO, without stopping it. The page's *Live views* use it.

- `rect_left`, `rect_right`: the rectified mono images the stereo matcher sees, 8-bit grayscale. The feature tracker runs on `rect_left`.
- `depth`: depth aligned to `rect_left`, colorized from red (near) to blue (far) over the min–max depth range. Pixels with no depth are black. `&raw=1` returns the 16-bit depth in millimetres instead (`raw` is only valid for depth).
- `rgb`: the color camera (CAM_A) at 640×400 and 5 fps. It runs only on devices that have it; on others the request returns 404.

Responses carry `Cache-Control: no-store` and `X-Snapshot-Unix-Time`, `X-Snapshot-Device-Timestamp` and `X-Snapshot-Sequence` headers. A request returns 504 if no frame arrives within 3 s (PnP not running, or the camera is not answering), 404 if the stream is unavailable or snapshots are disabled, and 422 for an unknown stream.

How it works:

- On the camera, a `Script` node holds only the latest frame of each stream, in size-1 non-blocking queues. It forwards one frame only when the host asks for it, so snapshots use no USB/Ethernet bandwidth until requested. Each stereo output's frame pool grows from 3 to 4 frames to cover the frame the gate holds (to 6 with the [live depth stream](#live-depth-stream), whose Script holds one frame and converts another).
- `main.py` and the VIO process share `/app/data/snapshots/` (`/usr/blueos/extensions/oakd-vio/snapshots/` on the vehicle). The endpoint atomically writes a request file, `requests/<stream>`, holding the request time.
- The VIO loop checks the directory at most every 0.2 s and claims requests by renaming them. It sends at most one request to the camera per second, with only one in flight at a time; repeated requests for the same stream are merged. Each frame it does a non-blocking check for a reply.
- PNG encoding runs on a separate writer thread with a bounded queue, so the VIO loop never waits on it. The writer saves `<stream>.png`, plus `depth_raw.png` for depth, then writes `<stream>.json` last, holding the request time it answers. The endpoint waits until that file shows a request time at least as recent as its own.
- If the camera does not reply within 2 s, an error is written for the endpoint, and the next request can go out.

Host cost, measured on a desktop with a 300-feature PnP frame: about 0.9 ms per frame without snapshots, and about 1.0 ms with a request on every frame (a request rate 15 times higher than the throttle allows). The camera-side cost of the RGB camera and the extra frame per pool has not been measured on hardware.

`&overlay=1` (rect_left only, else 400) draws the features of the last frame the VIO loop processed: inliers green, outliers (matched, rejected by RANSAC) red, unmatched or without depth grey, each with a line from its position in the previous frame. The VIO loop only keeps references to that frame's ids and pixels; the overlay is built on the snapshot writer thread when a rect_left snapshot is served, and drawn by the web server, so it costs nothing until asked. `X-Overlay` is `features` (with `X-Overlay-Inliers`, `-Outliers`, `-Unmatched` and `-Frame-Sequence`) or `unavailable`. The processed frame is usually one or two frames older than the image.

To aim the [depth probe](#depth-probe-pnp), add `&crosshair=1` (and the probe's `x`, `y`, `w`, `h`) to a `rect_left`, `rect_right` or `depth` snapshot. The PNG then has a magenta crosshair and ROI box, for example `/api/snapshot?stream=rect_left&crosshair=1&x=320&y=200&w=41&h=41`. It doesn't work with `raw=1` or `rgb` (400). Without `crosshair`, snapshots are unchanged.

### Depth probe (PnP)

`POST /api/depth_probe` measures the device depth in a small region while PnP keeps running. Use it to compare the camera's depth with a tape measure. In water, compare the tape with `scaled_m` (which includes the refractive index) and set *PnP Depth Scale* for any residual. The *Depth probe* part of the page's *Live views* has the inputs and a *Probe* button; click the left image to aim, and tick *probe ROI* to draw the crosshair on the views. The parameters go in the query string; a GET returns 405.

```
curl -X POST 'http://<vehicle>:8095/api/depth_probe'                                # image centre, 41×41 px, 10 frames
curl -X POST 'http://<vehicle>:8095/api/depth_probe?x=320&y=200&w=41&h=41&frames=10'
```

- `x`, `y`: ROI centre in pixels on the rectified-left grid (width × height, 640×400 by default). The default is the image centre. Coordinates outside the image return 400.
- `w`, `h`: ROI size in pixels, default 41, at most 200. `frames`: how many consecutive depth frames to average, default 10, at most 50. Larger values are clamped and listed in `clamped`.
- The response:
  - `valid_fraction` and `n_valid_px`: depth outside the camera's min/max depth (300–4000 mm by default) is invalid and only counts against `valid_fraction`.
  - Over the valid pixels of all frames: `median_mm`, `p10_mm`, `p90_mm` and `mean_mm`. Also `median_of_frame_medians_mm`, with `frame_medians_mm` and `frame_valid_fractions` per frame.
  - `raw_m`: the median of the frame medians, in metres. This is Z, the device depth along the optical axis. `scaled_m` = `raw_m` × `total_scale`, where `total_scale` = `refractive_index` × `depth_scale`; that is what PnP uses. The `*_mm` statistics are always the device depth.
  - `range_m` (raw and scaled): the straight-line distance to the ROI centre, Z·√(1 + ((u−cx)/fx)² + ((v−cy)/fy)²). `point_m` is the 3D point (X, Y, Z), raw and scaled. Raw uses the in-air camera matrix in use (`intrinsics`, `intrinsics_source`). Scaled uses that matrix with fx and fy × n, as PnP does.
  - `device_timestamps`, `sequences` and `summary`, e.g. `median 1.234 m (scaled 1.645 m, n 1.333), valid 87%` (the `n` part is left out in air).
  - `note` explains Z versus range.
- Where to measure from: the camera's depth is measured from the left camera's optical centre. That is a few mm behind the front of the lens, plus the port. Aim square at a flat target and tape-measure Z from the lens. Off-centre, compare the tape with `range_m`.
- Errors: 400 for bad parameters, 503 if the algorithm isn't PnP or VIO is stopped, and 504 if no result arrives within frames/fps + 3 s.

How it works: like snapshots, it costs nothing until asked.
1. The endpoint writes `/app/data/snapshots/depth_probe/requests/<id>.json`.
2. The VIO loop checks that directory at most every 0.2 s.
3. While a probe is active, it copies the ROI out of each depth frame it already has. That takes about 0.02 ms for 200×200, measured on a desktop.
4. After N frames, a separate thread computes the statistics and atomically writes `results/<id>.json`. The endpoint returns that file and deletes it.

Requests are served one at a time, oldest first. The probe works with *Snapshots* turned off; it needs no camera-side gate.

### Focal check (PnP)

`POST /api/focal_check` (the *Focal check* button) checks the camera matrix's fx against the focal length the device used to turn disparity into depth. It finds feature matches between `rect_left` and `rect_right` on the same image row (SIFT on contrast-enhanced images, |dy| < 2 px, disparity > 0.5 px). At each match it reads the device depth (median of a 3×3 window) and computes f = Z·d/B.

```
curl -X POST 'http://<vehicle>:8095/api/focal_check'               # tolerance 15%
curl -X POST 'http://<vehicle>:8095/api/focal_check?tolerance=0.1'
```

The response has `fx_K`, `f_from_depth` (median), `f_iqr`, `ratio`, `n_matches`, `ok` (|ratio − 1| ≤ tolerance, with at least 10 matches) and a `message`. It also gives the `baseline_m` used and the `device_timestamps` of the three snapshots. With the rectified K the expected ratio is about 1.0. With the raw K it is about 0.59, and the message says so.

Caveats:
- The three frames are separate snapshots, taken about 1 s apart because of the snapshot throttle. They are not one stereo pair, so keep the camera and the scene still.
- It runs in the web server, never in the VIO loop.
- It checks fx only, not fy, cx or cy.
- Point the camera at a textured scene 0.5–3 m away.

The same check runs offline on snapshot PNGs:

```bash
python app/tools/check_rectified_focal.py rect_left.png rect_right.png depth_raw.png 7.496 --fx 163.1
```

The arguments are the baseline in cm, then optionally the fx to check. `depth_raw.png` is the `raw=1` depth snapshot.

## Web UI (PnP)

The live panels need the PnP algorithm; with Basalt they say so, and the settings, logs and ArduPilot panels still work. The page polls `/api/status` and `/api/history` once a second only while its tab is visible (`document.visibilityState`), snapshots only while *auto-refresh* is on, the logs every 15 s, and the ArduPilot parameters on load and on *Re-read*. No websockets. A stopped VIO, a missing camera or a missing mavlink2rest show as such ("VIO stopped", greyed values, "DVL not found", "unreadable"), and the page picks up again after a VIO restart.

### Live telemetry

The VIO process publishes its live status through files, like snapshots ([`app/vio_telemetry.py`](app/vio_telemetry.py)):

- Per frame, the loop calls `Telemetry.record_frame()`: one row into a preallocated numpy ring buffer, a rolling window of failure reasons, a few counters. Measured with a micro-benchmark in the tests: about 0.02 ms per frame including the publisher step, also while the page is open (the test fails above 0.2 ms).
- A publisher thread writes `status.json` (a few KB) at most twice a second and `history.npy` (the ring buffer) once a second, but only while `main.py` asked within the last 10 s: each `/api/status` and `/api/history` touches a lease file. With nobody watching nothing is serialized or written. The camera chip temperature (`device.getChipTemperature()`, an RPC) is sampled on that thread at most every 5 s, never in the VIO loop.
- The files live in `/dev/shm/oakd-vio-telemetry/` (RAM, no SD card wear; `main.py` passes `--telemetry-dir`). A new VIO run deletes the previous run's files.
- Memory bound: 10 minutes of frames at min(fps, 30), +10 %, 12 float64 columns: at most 19800 rows, 1.9 MB (0.63 MB at 10 fps). The failure window holds at most 5 minutes × 30 fps of (time, reason) pairs.
- `/api/history` downsamples to at most 600 points (every k-th frame, the newest always kept) and caches the parsed file by mtime. Pi CPU % comes from `/proc/stat` deltas in `main.py` (no psutil).
- The trajectory rotates each body-frame delta by the yaw integrated so far (from the gyro prior, else the PnP rotation) and starts at the origin facing +x. It drifts, and deltas dropped at reference resets are missing from it.

### Live depth stream

The *Live depth* panel shows the camera's stereo depth at about 5 fps while its *stream* box is ticked. The *colorize* box colours it in the browser (near is red, far is blue, invalid is black). The panel also shows a legend for *Stereo Depth Min/Max*, the depth at the image centre (the median of the centre 5×5 pixels and the centre pixel), the frame rate and the bandwidth. It is the `<img>` of `GET /api/stream/depth` (MJPEG), so it also opens in a browser tab or in VLC.

The depth is made 8-bit and JPEG-encoded on the camera; the Pi only forwards the bytes ([`app/depth_stream.py`](app/depth_stream.py)):

- On the camera, a `Script` node takes `StereoDepth.depth` (RAW16 mm, aligned to rectified left, 640×400) in a size-1 non-blocking queue. It forwards every n-th frame (n = fps / stream fps; 3 at 15 fps) as a GRAY8 frame, converted with two 256-byte lookup tables on the high and low bytes (`bytes.translate`, 50 rows at a time). A `VideoEncoder` in MJPEG mode (quality 70) encodes it; its input is non-blocking with size 1, so a busy encoder drops stream frames, never VIO frames.
- The gray scale is linear in depth: 255 at *Stereo Depth Min*, 16 at *Stereo Depth Max*, 0 for invalid or out-of-range pixels, in 2^k mm steps (16 mm for a 4 m max). The browser treats gray < 8 as invalid, which absorbs JPEG ringing. `/api/status` → `depth_stream` gives the scale (`depth_range_mm`, `gray_near`, `gray_far`, `quantization_mm`), `fps`, `kbps` and `center`.
- With nobody watching, the Script blocks on its control input, so it converts, encodes and sends nothing. Each `/api/stream/depth` client touches `depth_stream.lease` in the telemetry directory every second. The VIO loop checks the lease every 0.5 s and sends the Script on/off. The stream stops about 4 s after the last client leaves.
- The VIO loop takes the newest encoded frame (`tryGetAll`, non-blocking) and writes it atomically to `/dev/shm/oakd-vio-telemetry/depth_stream.jpg`. `main.py` polls the file every 50 ms and sends each new one as a multipart part. The centre depth goes to `depth_stream.json` once a second while streaming.
- The Script uses only the camera's own Script API, which is smaller than the host's `dai.ImgFrame`: `ImgFrame(width * height)` (the size in bytes; `ImgFrame()` does not exist there), `setData`, `setType(GRAY8)`, `setWidth`, `setHeight`, `setTimestamp` and `setSequenceNum`. There is no `setStride` (GRAY8's stride is the width), `setMetadata` or `setTimestampDevice`. The list was checked against the RVC2 firmware that depthai 3.3.0 ships, and a test runs the Script against fakes that accept only that API.
- Errors: the Script counts each frame that fails, and sends the count and the latest error on its `status` output: the first one at once, then at most one a second. It also logs the 1st and every 100th with `node.warn`. A metadata failure still sends the frame. `/api/status` → `depth_stream` has `errors`, `last_error` (`text`, `unix_time`, `fatal` if the Script stopped) and `device_stats` (the camera's convert + send time, every 150 frames). The panel shows them as *camera conversion* and *camera errors*, and the widget's status line shows the error count.
- Cost: on the Pi, one file write per streamed frame and a 5×5 median once a second. A sparse depth frame from the stage-1 snapshots (`snap4`, 13 % valid) is 26 KB as a quality-70 JPEG (OpenCV's encoder), so about 130 KB/s (1 Mbit/s) on the link at 5 fps. The Script logs its own conversion time on the camera every 150 frames ("depth stream: … ms") and reports it in `device_stats`; it has not been measured on hardware yet. If it competes with the VIO, lower *Depth stream fps* or turn *Live depth stream* off, which removes the Script and encoder from the pipeline.

### Cockpit widget

The extension offers a Cockpit widget, *OAK-D VIO depth*. It shows the live depth with the PnP tracker's features drawn on top and a status line. In Cockpit, open Edit mode; the widget is in the widget list, offered by BlueOS. It is also the page `widget/depth`, linked from the *Live depth* panel.

- **Discovery:** `GET /register_service` declares `works_in_relative_paths` (BlueOS proxies the extension at `/extensionv2/oakdvio/`) and `extras.cockpit: /cockpit_extras.json`. Cockpit fetches the extras JSON through that proxy. The widget entry uses `useExtensionPathAsBaseUrl`, so the iframe is `http://<vehicle>/extensionv2/oakdvio/widget/depth` and works from a remote Cockpit too. Every URL in both pages is relative, so they work on port 8095 and behind the proxy.
- **Depth:** the same MJPEG as the *Live depth* panel, colorized on a canvas, with the depth-range legend.
- **Features**, coloured by state:
  - *inlier* (green): matched to the reference and kept by RANSAC;
  - *rejected* (magenta square): matched and rejected by RANSAC;
  - *matched* (white ring): matched, no RANSAC result;
  - *depth* (light blue): not in the reference, with valid depth;
  - *no depth* (grey ring).
  Positions are on the depth image's pixel grid (scaled when the depth is decimated).
- **Status line:** VIO fps, ok %, inliers, features with depth, VISION_POSITION_DELTA messages sent, the log-only flag and the last failure's reason. When there is nothing to show, it says why (VIO stopped, not PnP, depth stream off).
- **Cost on the Pi:** nothing is decoded or drawn on the Pi.
  - While a widget is connected, `/api/stream/features` touches `features.lease`. The VIO loop then writes `features.json` (about 4 KB for 300 features, one line of JSON) at most every 0.2 s. That took 0.22 ms per write on the x86 desktop.
  - The web server forwards the bytes unchanged and adds one small status event per second.
  - With nobody watching, the VIO loop does one clock comparison per frame and one stat every 0.5 s.
  - The page closes both streams while it is hidden.
- **Options** (in the iframe URL): `?colorize=0` shows gray depth, `?features=0` hides the features, `?status=0` hides the status line.

### DVL overlay

With *DVL overlay source* `auto`, `main.py` ([`app/dvl_source.py`](app/dvl_source.py)) looks for a DVL in mavlink2rest and the vx/vy plot draws its velocity dashed:

- Message: `VISION_POSITION_DELTA`, which the Water Linked DVL extension sends to ArduSub; velocity = `position_delta / time_delta_usec`, in the same body frame as ours, so the overlay compares exactly the two inputs the EKF would fuse. Our own message (system 1, component 195) is excluded.
- Component: found automatically. The whole message tree (`GET /mavlink2rest/mavlink/vehicles`) is read at most every 10 s until another component's `VISION_POSITION_DELTA` turns up, then only that message is polled, at most twice a second with a 0.4 s timeout, and only while the page asked for history in the last 10 s. A sample is taken when mavlink2rest's message counter advances. After about 3 s of failed polls the source is dropped and searched for again.
- The overlay only has samples from when the page started asking (up to 10 minutes kept). Nothing here raises; without a DVL the page shows "DVL not found".
- Not verified on a vehicle: it needs the DVL's messages to reach mavlink2rest's MAVLink endpoint.

### ArduPilot panel

The panel reads the [suggested parameters](#suggested-parameters-for-optical-position-hold-written-only-from-the-page) with `PARAM_REQUEST_READ` and shows current, suggested (editable), status (`ok`, `differs`, `unreadable`), why and *reboot*, plus the armed state from the autopilot's HEARTBEAT (`base_mode & 128`). Reading (`POST /api/autopilot/params/read`, on load and on *Re-read*) never writes. Both reading and writing are POST, so no GET (BlueOS's API scan, a browser prefetch) reaches the autopilot.

Writing (`POST /api/autopilot/params`):

- The page's *Write selected…* opens a dialog listing each change (`NAME old → new`, reboot flag) and the armed state; a second, explicit click sends `{"confirm": true, "params": {...}}`. Nothing is written automatically or at startup.
- 400 without `"confirm": true`, for any name outside the suggested table, and for values outside the parameter's range or allowed values (e.g. `VISO_TYPE` 0–3, `VISO_POS_*` ±5 m, `VISO_ORIENT` 0/2/4/6/24/25); one bad entry rejects the whole request.
- 409, nothing written, unless a HEARTBEAT that arrives during the request (its counter must advance within 2.5 s) says disarmed: armed, no HEARTBEAT, a stale one, or no mavlink2rest all refuse. The armed bit is checked again before each `PARAM_SET`; if the vehicle arms midway, the rest are reported as failed and not written.
- Each written parameter is read back with a fresh `PARAM_REQUEST_READ`. The response lists `changed` (`name`, `old`, `new`, `readback`), `failed` (with the error, e.g. a read-back mismatch), `unchanged` and `reboot_required`. The autopilot is never rebooted by the extension.

### Demo server and screenshots

[`app/tools/ui_demo_server.py`](app/tools/ui_demo_server.py) runs the real web app against fakes, for working on the page without a vehicle: it replays a PnP data log at real time into the real telemetry code, serves generated camera frames through the real snapshot and depth-probe services, and runs [`app/tools/fake_mavlink2rest.py`](app/tools/fake_mavlink2rest.py) (an autopilot with parameters and HEARTBEAT, and a DVL) on 127.0.0.1. It never contacts another host. [`app/tools/ui_screenshots.py`](app/tools/ui_screenshots.py) takes full-page screenshots with headless Chrome.

```bash
python app/tools/ui_demo_server.py --log vio-20261118-190503-10.jsonl --port 8099            # add --armed, --no-dvl
python app/tools/ui_screenshots.py --url http://127.0.0.1:8099/ --out /tmp/ui
```

## Log-only mode and data logs

On a vehicle whose EKF already uses ExternalNav (`VISO_TYPE` 1, `EK3_SRC1_VELXY` 6), every `VISION_POSITION_DELTA` the extension sends is fused. To record VIO without affecting the vehicle, turn on log-only mode in any of these ways:

- tick *Log only: do not send anything to the autopilot* on the configuration page,
- set `OAKD_LOG_ONLY=1` in the container environment; this forces log-only whatever the saved setting, and the page shows that it is forced,
- run `pnp_vio.py` or `basalt_configurable.py` with `--log-only`.

In log-only mode the autopilot parameters are neither read nor written and no MAVLink connection is created, so nothing reaches mavlink2rest. The VIO process logs `LOG-ONLY: no MAVLink output, autopilot params untouched` at startup.

Each VIO process start writes a JSONL file to `/app/data/logs/` (`/usr/blueos/extensions/oakd-vio/logs/` on the vehicle), named `vio-YYYYmmdd-HHMMSS-<pid>.jsonl`, in both modes. Every line has `type`, `wall_time` (ISO, UTC), `unix_time` and `monotonic`. Lines are flushed as they are written.

- `"type": "event"` lines: `start` (arguments, `log_only`, and for PnP the resolved `stereo` settings, `depth_scale`, `refractive_index`, `intrinsics_from`, `intrinsics_override`, `lever_arm` and `lever_arm_m`), `warning`, `device_open` attempts, `device`, `calibration`, `tracking` changes, `error`, `exit`. PnP also writes:
  - `calibration`: raw CAM_B `intrinsics` (`fx, fy, cx, cy`), `distortion` (14 coefficients), the computed `rectified` matrix (or `null` and `rectified_problem`), `alpha`, `distortion_correction`, `baseline_m` (calibration) and `baseline_spec_m` (design), and `R_cam_imu`;
  - `stereo_config`: `requested` settings, `effective` values read back from the node, and `warnings`;
  - `intrinsics`: the camera matrix used (`intrinsics`) and its `source`, which is one of:
    - `rectified`, `frame`, `cam_b`, `cam_c` or `override`;
    - a fallback: `rectified (frame matrix is raw)`, `rectified (fallback)`, `frame (fallback)` or `cam_b (fallback)`.
    It also has `effective_intrinsics` (fx, fy × the refractive index: what PnP runs with), `requested_source`, `raw`, `rectified`, the depth `frame` matrix and `frame_matches` (`raw`, `rectified` or `neither`), `override`, `alpha`, `distortion_correction`, `baseline_m`, `depth_scale`, `refractive_index`, `total_depth_scale` and `warnings`;
  - `snapshots`: the available `streams`, and `unavailable` ones with the reason;
  - `recovery`: see [Tracking recovery](#tracking-recovery-pnp).
- `"type": "step"` lines (PnP only): one per frame, with `device_timestamp`, `sequence`, `time_delta_us`, `position_delta` and `angle_delta` (body FRD, exactly what is or would be sent), `confidence`, `features`, `new_features` and `tracked_features` (ids not seen / seen in the previous frame), `valid_depth`, `matches`, `inliers`, `reprojection_rms_px`, `gyro_disagreement_deg`, `proc_ms`, `status` (`ok` or `rejected`), `reason`, `sent`, and `reference_age_s`, `gap_frames` and `reference_reset` (see [Failed frames](#failed-frames-pnp)). The deltas are `null` for rejected frames. With *Lever arm diagnostics* on, also `lever_delta_position`, `lever_delta_correction` and `lever_delta_rotation` (see [Lever arm](#lever-arm-viso_pos-pnp)); these are never sent.

Basalt runs write the events only, not per-frame poses. The supervisor appends its launches, exits (code or signal), restarts and stops to `supervisor.jsonl` in the same directory. Nothing is rotated or deleted. The configuration page links to the files.

## Failed frames (PnP)

A frame fails when PnP finds no trustworthy pose: too few matches or inliers, a rotation that disagrees with the gyro (`--gyro-gate`), or an implausible translation. Nothing is sent for it, but the last good frame stays the reference, so the next good frame is solved against it. Its `VISION_POSITION_DELTA` then covers the whole gap since that frame: `time_delta_us`, `position_delta` and `angle_delta` all span the same interval, and the gyro prior is integrated over it too. Before, the failed frame became the reference and the motion up to it was lost; failures cluster during fast motion, so this lost a share of the distance travelled.

Only the reference frame's points are used. The failed frame's own points are in its own camera frame, which has no pose, so they can't be added to the reference. As features stop being tracked, the reference only loses matches. A failed frame replaces the reference (the gap is dropped, as before) when:

- the reference is older than *PnP Max Reference Age* (`--max-reference-age`, default 1 s), or
- fewer than `--min-inliers` (15) reference points are still matched, so it can never solve again.

The translation gate (0.5 m) applies per frame interval: a solve that bridges *n* failed frames may move up to (n + 1) × 0.5 m. The gyro gate stays at 5°, because it bounds an error, not the motion. Frames with a repeated or earlier timestamp are rejected and leave the reference alone. [Tracking recovery](#tracking-recovery-pnp) still drops the reference.

Each JSONL step records `reference_age_s` (age of the reference the frame was solved against, the same as the delta's interval), `gap_frames` (failed frames between the reference and this frame) and `reference_reset` (this failed frame replaced the reference). The 10 s summary counts the gaps bridged and the reference resets.

## Tracking recovery (PnP)

In a pool test (`vio-20261117-180438-381.jsonl`), PnP tracked every frame for 10 minutes with about 110 features. After the vehicle was handled, it never recovered: the camera kept reporting 10–40 features with depth on only a few, even after the vehicle was static again at its starting heading. A restart in the same scene gave 106 features and 92% good frames. At the time, the host kept no state beyond the previous frame, so the stuck state is in the camera's feature tracker. See [NOTES_OPTION_D.md](NOTES_OPTION_D.md#tracking-degradation-in-the-first-pool-test).

After *PnP Recovery After* (`--recovery-frames`, default 20) consecutive failed frames, the VIO process:

1. drops the host PnP reference, including one held across failed frames, so the next frame starts a new baseline;
2. re-initialises the camera's `FeatureTracker` through its `inputConfig` queue, without restarting the pipeline: it sends a config with the feature maintainer off, then the normal config one frame later, to make the tracker detect features from scratch.

If tracking still fails, the wait before the next recovery doubles each time (20, 40, 80, ... frames, up to 16×). A successful frame resets the counter and the wait. Each recovery logs a warning and writes a `recovery` event with `count`, `failed_frames`, `reason`, `features`, `valid_depth`, `matches` and `next_after`. The 10 s summary counts recoveries, and the per-frame `new_features` and `tracked_features` show whether the tracker is re-detecting or stuck on old ids.

## Installing

The image is built for **64-bit ARM (arm64) only**. It won't run on 32-bit BlueOS installs; BlueOS flags the image as incompatible, and System Information shows a kernel ending in `-v7l+`. Flash a 64-bit BlueOS image first.

In BlueOS, go to **Extensions → Installed → +** and fill in:

- **Extension Identifier**: `williangalvani.oakd-basalt`
- **Extension Name**: `Oak-D Basalt VIO`
- **Docker image**: `williangalvani/blueos-oakd-basalt`
- **Docker tag**: `arm64-rtabmap`
- **Custom settings**:

```json
{
  "ExposedPorts": {"8095/tcp": {}},
  "HostConfig": {
    "NetworkMode": "host",
    "Binds": ["/dev/:/dev/", "/usr/blueos/extensions/oakd-vio:/app/data"],
    "DeviceCgroupRules": ["c 189:* rmw"]
  }
}
```

`/dev` and the `c 189:*` cgroup rule give the container access to the OAK-D over USB.

### Vehicles without internet

If the vehicle can't reach Docker Hub, save the image on a computer that can and upload it:

```bash
docker pull --platform linux/arm64 williangalvani/blueos-oakd-basalt:arm64-rtabmap
docker save williangalvani/blueos-oakd-basalt:arm64-rtabmap -o oakd-vio-arm64.tar
```

Then use the upload option in BlueOS's extension manager (BlueOS versions with Kraken v2), and enter the same identifier, name and custom settings as above.

## Building

The image is built in two stages so that the slow part only runs once:

1. **Builder image** ([`dockerfile.builder`](dockerfile.builder)): compiles depthai-core with its Python bindings and Basalt, RTAB-Map and PCL support. Built under QEMU for arm64 this takes several hours, mostly compiling vcpkg dependencies. A prebuilt copy is at `williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap`.

   ```bash
   docker buildx build --platform linux/arm64 -f dockerfile.builder \
     -t williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap --push .
   ```

2. **Runtime image** ([`dockerfile`](dockerfile)): a slim Debian image that copies the Python bindings and shared libraries out of the builder and adds the app. This takes a few minutes.

   ```bash
   docker buildx build --platform linux/arm64 \
     --build-arg BUILDER_IMAGE=williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap \
     -t williangalvani/blueos-oakd-basalt:arm64-rtabmap --push .
   ```

`depthai-core` is a git submodule and must be checked out before building the builder image.

## Code layout

| File | Purpose |
|---|---|
| `app/main.py` | Web UI and API; starts and stops the VIO process |
| `app/static/ui.html`, `ui.css`, `ui.js` | The web page (inlined into one page by `main.py`) |
| `app/static/widget.html`, `widget.css`, `widget.js`, `widget_icon.svg` | The Cockpit widget page (inlined the same way) and its icon |
| `app/cockpit_widget.py` | BlueOS service metadata, the Cockpit extras JSON and the widget's status line |
| `app/depth_stream.py` | Live depth stream: on-camera 8-bit conversion Script, VIO-side service, `main.py` reader |
| `app/feature_stream.py` | Tracked features for the widget: VIO-side lease-gated publisher, `main.py` reader and SSE framing |
| `app/config_schema.py` | Every setting: schema, defaults, server-side validation, form fields |
| `app/vio_telemetry.py` | PnP live status and plot history: ring buffer and publisher (VIO process), readers (`main.py`) |
| `app/dvl_source.py` | Optional DVL velocity overlay read from mavlink2rest |
| `app/system_stats.py` | Pi CPU load from `/proc/stat` |
| `app/tools/ui_demo_server.py`, `fake_mavlink2rest.py`, `ui_screenshots.py` | Web UI development: replay server, fake mavlink2rest, screenshots |
| `app/vio_supervisor.py` | Restarts the VIO process with backoff when it exits unexpectedly |
| `app/device_open.py` | Waits for the OAK-D and opens it, retrying with backoff |
| `app/vio_log.py` | Log-only mode selection and the JSONL data logs |
| `app/basalt_configurable.py` | The VIO pipeline: cameras, IMU, `BasaltVIO`, MAVLink output |
| `app/autopilot_params.py` | Reads and sets the autopilot's EKF parameters through mavlink2rest; the page's guarded suggested-parameter read and write |
| `app/mavlink_node.py` | depthai host node that turns poses into MAVLink messages |
| `app/basalt.py` | Standalone command-line version of the pipeline, useful for testing |
| `app/pnp_vio.py`, `app/pnp_odometry.py` | Experimental PnP VIO: camera pipeline and host loop; pose math |
| `app/rectified_intrinsics.py` | PnP rectified-left camera matrix: computing it from the calibration, choosing the source, and the focal check |
| `app/depth_probe.py` | Depth probe requests (`main.py`), serving them in the VIO loop, and the crosshair overlay |
| `app/tools/check_rectified_focal.py` | Offline focal check on snapshot PNGs |
| `app/stereo_config.py` | PnP stereo settings: defaults, web form, command line and `StereoDepth` configuration |
| `app/snapshots.py` | Snapshot requests (`main.py`) and serving them from the VIO process |
| `app/tracking_recovery.py` | Tracking recovery state machine and feature tracker re-initialisation |
| `entrypoint.sh` | Container entrypoint; starts the web server |

## Known limitations

- VIO runs on the Raspberry Pi CPU, which limits it to a few frames per second. The builder includes depthai's `RTABMapVIO` node, which can use feature tracking on the camera itself, but the pipeline does not use it yet.
- `VISION_POSITION_DELTA` reports a fixed confidence of 100% and no rotation delta.
- At startup the VIO process waits up to 30 s (`--device-timeout`) for the OAK-D to be discoverable and to open, retrying with backoff, and then exits with code 2. A PoE OAK-D can take many seconds to come back after the previous pipeline closed; opening it too early has been seen to segfault inside depthai.
- If the VIO process exits without being stopped from the page (error, crash or segfault), `main.py` restarts it after 2 s, doubling up to 30 s; the delay resets after a run of 60 s. `/api/status` reports `restart_count`, `last_exit_code` (negative for a signal, e.g. -11 for SIGSEGV) and `restart_in`.
- Web UI: the plots lag by up to about a second (history is published once a second); the plots and the trajectory start over when VIO restarts; the DVL overlay has no history from before the page was opened. Live telemetry, the feature overlay and the chip temperature are PnP only. The web UI was developed against the demo server only; on a vehicle the DVL discovery, `getChipTemperature()` and the Pi CPU cost of the publisher are unverified.
