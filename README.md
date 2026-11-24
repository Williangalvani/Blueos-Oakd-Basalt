# BlueOS OAK-D Visual Odometry

A [BlueOS](https://github.com/bluerobotics/BlueOS) extension that runs stereo visual-inertial odometry (VIO) on a Luxonis OAK-D camera and feeds the result to an ArduPilot autopilot over MAVLink.

It uses the [depthai-core](https://github.com/luxonis/depthai-core) `BasaltVIO` node with the OAK-D's two mono cameras and IMU, and sends the estimated motion as `VISION_POSITION_DELTA` messages through BlueOS's mavlink2rest.

## What it does on startup

1. Starts a configuration web page on port **8095** (8080 is used by mavlink-server on BlueOS 1.4.x).
2. Starts the VIO pipeline with the saved settings and begins sending MAVLink.
3. Checks the autopilot's parameters against the recommended set (below), read-only, and shows the result on the page.

It never changes autopilot parameters by itself.

## Autopilot parameters

`SUGGESTED_OPTICAL_HOLD_PARAMS` in [`app/autopilot_params.py`](app/autopilot_params.py) is the one recommended set for position hold on `VISION_POSITION_DELTA` (ArduSub, EKF3): name, value, whether it needs a reboot, and why, with references to the ArduPilot source.

| Parameter | Recommended | Reboot | Why |
|---|---|---|---|
| `AHRS_EKF_TYPE` | 3 | yes | EKF3 (ArduPilot's default). Only EKF3 takes `VISION_POSITION_DELTA`. |
| `EK3_ENABLE` | 1 | yes | EKF3 enabled (default). |
| `VISO_TYPE` | 1 | yes | MAVLink visual odometry. The other `VISO_*` parameters only appear after this is set and the autopilot rebooted. |
| `VISO_POS_X/Y/Z` | 0.1655, −0.0069, 0.180 | no | Camera relative to the IMU, m FRD, from the CAD. EKF3 applies this lever arm itself (see [Lever arm](#lever-arm-viso_pos-pnp)). |
| `VISO_ORIENT` | 0 | no | The extension already rotates into body FRD. |
| `VISO_DELAY_MS` | keep | no | Unverified; tune from logs. Never counted as a difference. |
| `EK3_SRC1_VELXY` | 6 | no | ExternalNav: the EKF fuses the visual body velocity only with this. |
| `EK3_SRC1_POSXY` | 0 | no | None: see below. |
| `EK3_SRC1_POSZ` | 1 | no | Baro (depth). |
| `EK3_SRC1_VELZ` | 0 | no | None (ArduSub's default): the body-frame deltas already carry the vertical velocity. |
| `EK3_SRC1_YAW` | 1 | no | Compass: the deltas carry no heading. |

Why `EK3_SRC1_POSXY` = 0 and not 6 (ArduPilot master 6f5d8560db):

- `VISION_POSITION_DELTA` becomes body-frame odometry (`AP_VisualOdom_Backend.cpp:62`), fused only when `EK3_SRC1_VELXY` is ExternalNav (`AP_NavEKF3_PosVelFusion.cpp:2059-2063`, `AP_NavEKF3_Control.cpp:572-576`).
- With POSXY 6 the EKF waits for an absolute ExternalNav position (`VISION_POSITION_ESTIMATE`, `AP_NavEKF3_Control.cpp:619-626`) that this extension never sends. It therefore ends up in relative aiding anyway (`AP_NavEKF3_Control.cpp:297-305`), exactly as with 0, so 6 only misleads.
- In relative aiding the EKF dead-reckons a relative position from the body velocity (`AP_NavEKF3_Control.cpp:791, 812`). ArduSub's position hold accepts that relative position while visual odometry is enabled (`ArduSub/system.cpp:196-204, 234-266`).
- 3 (GPS) needs a GPS fix (`AP_NavEKF3_Control.cpp:595-601`), which is lost when submerged.

### The check and the Configure button

The page checks the autopilot every 30 s, on demand (the [ArduPilot panel](#ardupilot-panel)'s *Read*) and right after a write. Checks only send `PARAM_REQUEST_READ`. While mavlink2rest or the autopilot does not answer, the check backs off up to 5 minutes. `/api/status` carries the cached result as `autopilot_params`: `checked_at`, `reachable`, `armed`, `ok`, `mismatches` (`name`, `current`, `recommended`, `needs_reboot`), `unreadable`, `pending_reboot` and `error`.

The top of the page shows one of these:

- **A warning** ("Autopilot parameters differ from the recommended setup"): a table of current → recommended, with the parameters that need a reboot marked.
- **Not checked**: mavlink2rest or the autopilot HEARTBEAT is missing.
- **A reboot reminder**: everything is written but the autopilot has not rebooted since. This clears once `SYSTEM_TIME` shows the autopilot booted after the write.
- **A small green line**: everything matches.

*Configure…* opens a dialog listing exactly the changes. Nothing is written until you confirm. The write then goes through the same guarded `POST /api/autopilot/params` as the ArduPilot panel:

- It is refused (HTTP 409, nothing written) unless a fresh HEARTBEAT says the vehicle is disarmed.
- Each value is read back and shown in the dialog.
- Parameters that need a reboot are listed. The extension never reboots the autopilot: do it from BlueOS or your ground station.

The Cockpit widget shows a one-line hint when the parameters differ or a reboot is pending.

`diff_params(current)` compares the set with a `{name: value}` dict and returns the entries that differ or are missing. Float32 rounding is ignored.

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

- **Live status**: camera connected, VIO running, fps, inliers, frames ok (last 10 s), processing time, features / valid depth, matches, refractive index, depth scale, the effective camera matrix, reference resets / gap frames, recoveries, Pi CPU and the camera chip temperature; failures by reason over the last 5 minutes; the [camera link](#camera-link-bandwidth) (measured Mbps down and up, peak, share of 100 Mbps, and an estimated per-stream breakdown); the output mode, messages sent, rate and last error, a *Log-only* toggle (saved, restarts VIO) and *Reset VIO*.
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
| PnP Max Features | 0 | PnP only: hard cap on the features the camera detects per frame (FeatureTracker `numMaxFeatures`, `max_features`, `--max-features`); 0 = depthai's automatic maximum. Each feature costs about 64 B per frame on the link (the `TrackedFeatures` message has no smaller form: its 32-byte descriptor is always serialized) and one ROI in `points` mode. Not tried on hardware: a cap can change what the VIO tracks. |
| PnP Min Inliers | 15 | PnP only: RANSAC inliers a frame needs to be accepted and sent (`--min-inliers`, 6–500). The inliers plot draws it. |
| PnP Recovery After (frames) | 20 | PnP only: consecutive failed frames before [tracking recovery](#tracking-recovery-pnp); 0 disables it. |
| PnP Max Reference Age (s) | 1.0 | PnP only: how long [failed frames](#failed-frames-pnp) keep the last good frame as the reference (`--max-reference-age`); 0 removes the limit. |
| PnP Depth Transport | points | PnP only: how the depth reaches the PnP (`depth_transport`, `--depth-transport`). `points`: the camera computes the depth at each tracked feature and sends only those numbers (a few kB per frame instead of the 512 kB depth frame); `full_frame`: every depth frame goes to the Pi, as before (for A/B). See [Depth at the features](#depth-at-the-features-pnp-depth_transport). |
| PnP Depth Points ROIs | host | PnP, `points` only: who lists the per-feature windows for the camera (`depth_points_rois`, `--depth-points-rois`). `host`: the Pi, from the features it receives anyway (about 12 kB per frame up the link, almost no camera CPU); `camera`: **slow**, a Script on the camera (no upstream traffic, but 111 ms of the camera's Leon CPU per frame of ~255 features on test16b: under 10 fps). |
| Snapshots | on | PnP only: allow [camera snapshots](#snapshots-pnp). Nothing is streamed until one is requested. |
| Snapshot Encoding | camera_jpeg | PnP only (`snapshot_encoding`, `--snapshot-encoding`): `camera_jpeg` = `rect_left`, `rect_right` and `rgb` snapshots are JPEG-encoded on the camera (quality 95); `raw` = sent raw and PNG-encoded on the Pi (the fallback). The depth snapshot is always raw (16-bit millimetres). |
| Live depth stream | on | PnP only: allow the page's [live stream](#live-depth-stream) (the camera image with the tracked features by default, or the depth), MJPEG encoded on the camera. Nothing is sent until a client watches (`--depth-stream`). |
| Depth stream fps | 5 | PnP only: target rate of the depth stream (0.5–15; every n-th camera frame, `--depth-stream-fps`). |
| Depth stream JPEG quality | 70 | PnP only: the camera's MJPEG quality (10–100, `--depth-stream-quality`). |
| Depth Stream Encoding | auto | PnP only: what the live stream shows and how it reaches a JPEG encoder (`depth_stream_encoding`, `--depth-stream-encoding`): `auto`, `mono_features`, `disparity`, `manip`, `nv12`, `yuv400p` or `host_jpeg`. `auto` is `mono_features` (the rectified left camera image the features are tracked on). `disparity` needs an 8-bit disparity (*Stereo Subpixel* off, and *Speckle filter* off or *Median filter last* on); otherwise it runs as `host_jpeg` and says why. See [Encoding variants](#encoding-variants). |
| 3D Pose Attitude | autopilot | PnP only, viewer only: `autopilot` = the [3D pose](#3d-pose-viewer)'s orientation is ArduPilot's `ATTITUDE` (read from mavlink2rest), gyro-integrated (drifting) while it is missing or more than 3 s old; `gyro` = gyro-integrated only (`pose_attitude_source`, `--pose-attitude-source`). The VIO output does not use it. |
| Depth Stream Debug | off | PnP only: while the depth stream runs, raise the camera's log level to DEBUG and show the encoder's log lines in the *Live depth* panel (`depth_stream_debug`, `--depth-stream-debug`). See [Debug mode](#debug-mode-depth_stream_debug). |
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
| Median filter last | off | **Experimental, not verified on the camera** (stereo probe case G tests it). Runs the median after the other post-processing filters (`filteringOrder` DECIMATION, SPECKLE, SPATIAL, TEMPORAL, MEDIAN instead of the preset's DECIMATION, MEDIAN, SPECKLE, …), so the disparity stays 8-bit with the speckle filter on. It changes the VIO's depth filtering, so it is off by default. |
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

Set `VISO_POS_X/Y/Z` on the autopilot to the lever arm (0.1655, −0.0069, 0.180 m from the CAD; see [Autopilot parameters](#autopilot-parameters)). *Lever arm diagnostics* only adds JSONL step fields, for comparing the two in logs:

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
| GET | `/api/status` | Whether the VIO process is running, restarts, whether log-only mode is active (`log_only`, `log_only_forced_by_env`), and for PnP the `camera` matrix in use (`null` until the first frame): the in-air `intrinsics`, `effective_intrinsics`, `refractive_index`, `depth_scale` and `total_depth_scale`. Also `algorithm`, `telemetry` (PnP live status, below; `null` until published, `stale: true` when VIO stopped; `telemetry.link` is the [camera link](#camera-link-bandwidth)), `camera_connected`, `chip_temperature_c`, `cpu_percent`, `uptime_s`, the last known `vehicle` armed state, the cached [parameter check](#the-check-and-the-configure-button) `autopilot_params` (this GET never contacts the autopilot) and `mock` (true in [mock mode](#mock-mode-no-oak-d-no-autopilot)). `depth_transport` ([depth at the features](#depth-at-the-features-pnp-depth_transport): the settings, the live pairing counters and the camera's CPU / memory load), `viewers` and `image_streams` ([images on demand](#images-on-demand-and-viewers)) |
| GET | `/api/history?seconds=60&max_points=600&tail=0` | PnP plot history, oldest first, `t` in seconds before `server_time`: `vx`, `vy`, `vz` (m/s, body FRD, `null` for rejected frames), integrated `x`, `y`, `yaw`, `inliers`, `ok`. At most `max_points` (≤ 600) over `seconds` (≤ 600); `tail` (≤ 10 s) adds the newest frames at full rate. `dvl` holds the [DVL overlay](#dvl-overlay). Weak `ETag`; `If-None-Match` gives 304 |
| POST | `/api/autopilot/params/read` | The suggested optical-hold parameters as the vehicle has them, and the HEARTBEAT armed state. Read only (POST because it sends MAVLink requests). See [ArduPilot panel](#ardupilot-panel) |
| POST | `/api/autopilot/params` | `{"confirm": true, "params": {"NAME": value}}`: guarded write with read-back, refused (409) unless a fresh HEARTBEAT says disarmed. See [ArduPilot panel](#ardupilot-panel) |
| GET | `/api/logs` | Data logs: name, size (bytes), mtime (unix time), newest first |
| GET | `/api/logs/{name}` | Download one data log |
| POST | `/api/start`, `/api/stop` | Start or stop VIO |
| POST | `/api/reset` | Restore defaults and restart VIO |
| GET | `/api/snapshot?stream=<stream>` | PnP only: one current camera frame (the camera's JPEG for `rect_left`, `rect_right` and `rgb`, PNG for `depth`, for a raw fallback and with an overlay); `stream` is `rect_left`, `rect_right`, `depth` or `rgb`. `&crosshair=1&x=&y=&w=&h=` draws the depth probe's ROI; `&overlay=1` (rect_left) draws the tracked features. See [Snapshots](#snapshots-pnp). |
| POST | `/api/depth_probe?x=&y=&w=&h=&frames=` | PnP only: depth statistics in an ROI over the next frames. See [Depth probe](#depth-probe-pnp). |
| POST | `/api/focal_check?tolerance=` | PnP only: checks fx against the focal length the device depth implies. See [Focal check](#focal-check-pnp). |
| POST | `/api/stereo_probe?cases=` | Diagnostics (`app/stereo_probe.py`): stops the VIO for about 90 s, runs StereoDepth/encoder test cases A–G (or a comma-separated subset; each case reports `disparity_format`) on the camera, restarts the VIO. Returns the JSON and saves it as `probe/stereo-<unix time>.json` in the data directory. 409 while one runs, 403 in mock mode. Not in the OpenAPI schema. |
| POST | `/api/stereo_probe?mode=mjpeg_loss[&seconds=][&qualities=]` | [MJPEG loss probe](#mjpeg-loss-probe-pnp): how much depth the MJPEG encoding of the RAW8 disparity loses at the tracked features, and what subpixel off costs. Stops the VIO for about 40–60 s (2–3 min if the encoders do not all run at once), restarts it. Saved as `probe/mjpeg_loss-<unix time>.json`. Same 409 / 403 rules. |
| GET | `/api/stream/depth` | PnP only: the live stream as `multipart/x-mixed-replace` MJPEG (grayscale: the rectified left camera image with `mono_features`, else depth or disparity; encoded on the camera, or on the Pi with `host_jpeg`); 503 when PnP is stopped or the stream is off. `?frames=N` ends after N frames. See [Live depth stream](#live-depth-stream). |
| GET | `/api/stream/pose` | Server-sent events for the [3D pose viewer](#3d-pose-viewer): `pose` (at most 10 per second), `trail` (once per connection, and again after a VIO restart) and `status` (once a second). Always 200. `?events=N` ends after N events. |
| GET | `/api/pose` | PnP only: the latest `pose` object once; 404 until the VIO process has published one (it starts on request) |
| GET | `/api/stream/features` | Server-sent events for the [Cockpit widget](#cockpit-widget): `features` (the PnP tracker's features, about 5 per second) and `status` (once a second; `problem` says why there is nothing to show). Always 200. `?events=N` ends after N events. |
| GET | `/api/features` | PnP only: the latest `features` object once; 404 until the VIO process has published one (it starts on request) |
| GET | `/widget/depth` | The [Cockpit widget](#cockpit-widget) page |
| GET | `/widget/pose` | The [3D pose viewer](#3d-pose-viewer) as a Cockpit widget page |
| GET | `/register_service`, `/cockpit_extras.json` | BlueOS service metadata and the Cockpit extras that declare the two widgets |

`/openapi.json` (and `/docs`) lists only the GET routes that answer at once with a bounded body and start nothing: `/`, `/widget/depth`, `/widget/pose`, `/register_service`, `/cockpit_extras.json`, `/api/config`, `/api/config/schema`, `/api/status`, `/api/logs` and `/api/logs/{name}`. BlueOS's helper GETs every path listed there every few seconds and reads each body to its end, so the streams (`/api/stream/*`) and the routes that wait for the camera or start work (`/api/snapshot`, `/api/history`, `/api/features`) are left out of the schema, and everything that runs something on the camera or the autopilot is a POST. `app/tests/test_openapi_scan.py` runs that scan against the app and fails on any other GET in the schema.

### Snapshots (PnP)

`GET /api/snapshot?stream=rect_left|rect_right|depth|rgb` returns a fresh frame from the running PnP VIO, without stopping it. The page's *Live views* use it.

- `rect_left`, `rect_right`: the rectified mono images the stereo matcher sees, 8-bit grayscale. The feature tracker runs on `rect_left`. JPEG-encoded on the camera (`image/jpeg`, quality 95), served as the camera sent them.
- `depth`: depth aligned to `rect_left`, colorized from red (near) to blue (far) over the min–max depth range. Pixels with no depth are black. `&raw=1` returns the 16-bit depth in millimetres instead (`raw` is only valid for depth). The depth crosses the link raw (one 512 kB frame per request) and is PNG-encoded on the Pi: 16-bit millimetres are measurement data (the focal check, `depth_raw.png`) that a JPEG cannot carry.
- `rgb`: the color camera (CAM_A) at 640×400 and 5 fps, JPEG-encoded on the camera. It runs only on devices that have it; on others the request returns 404.

With *Snapshot Encoding* `raw`, or after the camera's encoder returned nothing for a stream (no JPEG within 2 s: that request answers 404 "… the next rect_left snapshot is sent raw"), the image streams come raw and are PNG-encoded on the Pi for the rest of the run. That is a fallback, flagged as "fallback: raw frames over the link, encoded on the Pi" in `X-Snapshot-Fallback`, `/api/status` → `image_streams`, the page and the widget.

Responses carry `Cache-Control: no-store` and `X-Snapshot-Unix-Time`, `X-Snapshot-Device-Timestamp`, `X-Snapshot-Sequence` and `X-Snapshot-Encoded-On` (`camera` or `pi`) headers. The overlays below are drawn on the Pi and return a PNG. A request returns 504 if no frame arrives within 3 s (PnP not running, or the camera is not answering), 404 if the stream is unavailable or snapshots are disabled, and 422 for an unknown stream.

How it works:

- On the camera, a `Script` node holds only the latest frame of each stream, in size-1 non-blocking queues. It forwards one frame only when the host asks for it, so snapshots use no USB/Ethernet bandwidth until requested. The request's sequence number is the stream's index, plus 4 for the raw frame; the Script sends `rect_left` / `rect_right` to one MJPEG `VideoEncoder` (`mono`, StereoDepth's pooled RAW8 frames, as the `mono_features` stream sends them), `rgb` (NV12) to another (`color`), and `depth` or a raw request to its `snapshot` output. Each stereo output's frame pool grows from 3 to 5 frames to cover the frame the gate holds and the one in the mono encoder (4 with *Snapshot Encoding* `raw`; more with the [live depth stream](#live-depth-stream), whose Script holds one frame and converts another). With a width the encoder rejects (not a multiple of 32) the mono snapshots are raw, and the status says why.
- `main.py` and the VIO process share `/app/data/snapshots/` (`/usr/blueos/extensions/oakd-vio/snapshots/` on the vehicle). The endpoint atomically writes a request file, `requests/<stream>`, holding the request time.
- The VIO loop checks the directory at most every 0.2 s and claims requests by renaming them. It sends at most one request to the camera per second, with only one in flight at a time; repeated requests for the same stream are merged. Each frame it does a non-blocking check for a reply.
- Writing runs on a separate writer thread with a bounded queue, so the VIO loop never waits on it. The writer saves the camera's JPEG as `<stream>.jpg` (its size read from the JPEG header, not decoded; anything that is not a JPEG is an error), or PNG-encodes a raw frame as `<stream>.png`, plus `depth_raw.png` for depth, then writes `<stream>.json` last, holding the request time it answers and the files. The endpoint waits until that file shows a request time at least as recent as its own.
- If the camera does not reply within 2 s, an error is written for the endpoint, and the next request can go out (raw, if it was a JPEG request).
- `encoding.json` in the snapshot directory has each stream's `codec`, `quality`, `encoded_on`, `over_link`, `fallback`, `reason` and `last_bytes` (`SnapshotService.status`); `/api/status` → `image_streams` shows it.

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

### MJPEG loss probe (PnP)

A diagnostic for one question: how much depth do we lose if the camera sends its disparity as MJPEG instead of the depth the VIO uses? It is not part of the VIO. It runs `stereo_probe.py --mode mjpeg_loss`; the analysis is in `app/mjpeg_loss.py`.

```sh
curl -sS -X POST 'http://<vehicle>:8095/api/stereo_probe?mode=mjpeg_loss' -o mjpeg-loss.json   # 10 s per run, q 70,85,95,100
curl -sS -X POST 'http://<vehicle>:8095/api/stereo_probe?mode=mjpeg_loss&seconds=15&qualities=80,95' | jq -r .probe.summary_table
```

Keep the camera **still**, looking at a static, textured scene with depth between 0.3 and 4 m. The VIO and its MAVLink output are off while it runs, and it restarts afterwards whatever happened.

- **Run A** uses the saved stereo settings with subpixel off and the median last (speckle on; stereo probe case G), so `stereo.disparity` is RAW8 (0–95 px, 0 = invalid). The same disparity frames go to the Pi raw and into one MJPEG encoder per quality. The VIO's own FeatureTracker runs on `rectifiedLeft` (`pnp_vio.create_feature_tracker`, with the saved feature settings), so the points are the VIO's. Every n-th `stereo.depth` frame is kept as well.
- **Run B**, right after: the saved settings as the VIO runs them (subpixel on, 3 bits). Every n-th RAW16 depth frame and the features.
- **Encoders at once:** the firmware's only limit is the total rate (about 450 MPix/s for JPEG; four 640×400 streams at 15 fps are 15 MPix/s). If a quality's JPEGs still come with fewer than half of run A's frames, that quality is measured again in its own run with one encoder, on the same static scene. `encoders.note` says which qualities ran at once and which ran in sequence.
- **On the Pi:**
  - Depth is `fx·B / disparity`, using the rectified fx and the baseline the VIO uses for its disparity stream (`calibration`).
  - It is sampled at every tracked feature with the VIO's rules: `points` (the default transport: the camera SLC's average of the in-range pixels in the 5×5 window, at least 30 % valid), `full_frame` (the 5×5 median, `lookup_depth`) and `centre` (the feature's pixel).
  - Each JPEG is compared with the raw frame it came from, paired by sequence number and checked by device timestamp (by timestamp alone if the encoder numbers its own frames).
  - JPEG gray is decoded two ways: `zero` (0 is invalid, the raw rule) and `viewer` (below 3 is invalid, the depth stream page's rule against JPEG ringing).
- **Output (`probe` in the response):**
  - `mjpeg.<quality>`: JPEG bytes per frame and Mbps at the camera fps (`rate`); the gray-level error at the features and over the frame; and per decode and sampling rule the per-point error in mm and % (median, p95, max), the bias, the valid→invalid and invalid→valid flips, all of it again `by_distance` (<0.5, 0.5–1, 1–2, 2–4, ≥4 m).
  - `raw_rates_mbps`: the uncompressed RAW8 disparity and RAW16 depth.
  - `conversion_check`: the camera's own depth × disparity against the host's fx·B, which should be about 1.
  - `subpixel`: run A's integer-disparity depth against run B's subpixel depth.
  - `quantisation`: theory and the measured integer step.
  - `summary_table`: the same as a compact text table, also logged to stderr.
- **Subpixel method and its limits:** runs A and B are different frames, and the FeatureTracker ids restart with each run, so points cannot be matched by id. Each run is reduced to per-pixel temporal medians (about 20 frames). The medians are compared at the features of run B's frames (`median_maps`), and so is each B frame against A's median (`per_frame`, which includes B's noise). Anything that moved between the runs, or an exposure change, counts as error. The medians hide each run's noise, which is reported in `noise`.
- **Quantisation:** one disparity pixel moves the depth by about z²/(fx·B), 8 times less with 3 subpixel bits. With fx 163 px and B 75 mm (fx·B ≈ 12.2 m·px) that is about 82 mm at 1 m and 330 mm at 2 m (integer), or 10 mm and 41 mm with subpixel.
- **Cost:** run A sends about 31 Mbps of raw disparity plus 61 Mbps of RAW16 depth at 640×400 and 15 fps (for the conversion check; only every n-th depth frame is kept). The Pi holds up to 300 raw disparity frames (about 77 MB at 640×400) until the run ends.

## Web UI (PnP)

The live panels need the PnP algorithm; with Basalt they say so, and the settings, logs and ArduPilot panels still work. The page polls `/api/status` and `/api/history` once a second only while its tab is visible (`document.visibilityState`), snapshots only while *auto-refresh* is on, the logs every 15 s, and the ArduPilot parameters on load and on *Re-read*. No websockets. A stopped VIO, a missing camera or a missing mavlink2rest show as such ("VIO stopped", greyed values, "DVL not found", "unreadable"), and the page picks up again after a VIO restart.

### Live telemetry

The VIO process publishes its live status through files, like snapshots ([`app/vio_telemetry.py`](app/vio_telemetry.py)):

- Per frame, the loop calls `Telemetry.record_frame()`: one row into a preallocated numpy ring buffer, a rolling window of failure reasons, a few counters. Measured with a micro-benchmark in the tests: about 0.02 ms per frame including the publisher step, also while the page is open (the test fails above 0.2 ms).
- A publisher thread writes `status.json` (a few KB) at most twice a second and `history.npy` (the ring buffer) once a second, but only while `main.py` asked within the last 10 s: each `/api/status` and `/api/history` touches a lease file. With nobody watching nothing is serialized or written. The camera chip temperature (`device.getChipTemperature()`, an RPC) is sampled on that thread at most every 5 s, never in the VIO loop, and so is the camera's load (`telemetry.camera_load`: `getLeonCssCpuUsage`, `getLeonMssCpuUsage` in %, `getDdrMemoryUsage`, `getCmxMemoryUsage`, `getLeonCssHeapUsage`, `getLeonMssHeapUsage` as `used` / `total` / `used_percent`; a failing getter gives `null` with the error in `errors`). The [camera link](#camera-link-bandwidth) counters are sampled on the same thread about once a second, whether or not anyone is watching, so the 60 s peak is there when the page opens.
- The files live in `/dev/shm/oakd-vio-telemetry/` (RAM, no SD card wear; `main.py` passes `--telemetry-dir`). A new VIO run deletes the previous run's files.
- Memory bound: 10 minutes of frames at min(fps, 30), +10 %, 12 float64 columns: at most 19800 rows, 1.9 MB (0.63 MB at 10 fps). The failure window holds at most 5 minutes × 30 fps of (time, reason) pairs.
- `/api/history` downsamples to at most 600 points (every k-th frame, the newest always kept) and caches the parsed file by mtime. Pi CPU % comes from `/proc/stat` deltas in `main.py` (no psutil).
- The trajectory rotates each body-frame delta by the yaw integrated so far (from the gyro prior, else the PnP rotation) and starts at the origin facing +x. It drifts, and deltas dropped at reference resets are missing from it.

### Camera link bandwidth

How much the PnP pipeline sends between the camera and the Pi over XLink, to judge whether it would fit a PoE camera's 100 Mbps Ethernet ([`app/link_stats.py`](app/link_stats.py)). It is shown in the *Camera link* box of the live status, one line of the depth widget's status line, `telemetry.link` in `/api/status` and the VIO's periodic log line (`link down 61.8 up 0.0 Mbps (10 s mean), peak 63.0, 62% of 100 Mbps, streams est. 61.4, USB SUPER`).

- **Measured total:** `device.getProfilingData()`, sampled about once a second on the telemetry publisher thread. It is a host-side read of XLink's per-link byte counters, not an RPC, and it needs no profiling switch (depthai-core v3.3.0 and its XLink count them on every read and write). `numBytesRead` is camera → Pi (down), `numBytesWritten` Pi → camera (up). They count the payload of every XLink stream (each message's data, its serialized metadata and a 24-byte trailer (datatype, metadata size and a 16-byte end marker), including streams the code does not read, such as RPC and logs), but not XLink's own packet headers or the USB / TCP framing. If the counters are missing, fail or stay at 0, the box says "n/a" with the reason and no numbers.
- **Per-stream estimate:** for each host queue the VIO counts the messages and bytes it reads: image data as `len(getData())` plus a header estimate (ImgFrame about 516 B, EncodedFrame 466 B), tracked features 64 B each (from libnop's encoding: the position, scores, age and the 32-byte descriptor, which RVC2 leaves zero but always serializes, plus the id in 1–5 B by its value), IMU packets 156 B each, MessageGroup 80 B, buffers 56 B, device log messages 64 B plus the text; each includes the 24-byte trailer. The sizes are estimates of depthai's serialized metadata, not measured. The `features` stream is about 16 kB per message at ~255 features (test16b); test15's ~3.5 kB came from far fewer features per message, not a different size per feature. The remainder (measured minus streams) is RPC, logs, XLink trailers and frames the camera sent but the Pi dropped in a full queue.
- **Streams (down):** `full_frame`: `depth` (the VIO's depth frame; RAW16 at 640 x 400 is 512 kB, about 61.5 Mbps at 15 fps, by far the largest) and `sync_group`; `points`: `depth_points` (the camera's depth at the features: 64 B + 90 B per feature and window, about 27 kB per frame of 300 features) and `depth_points_status`, plus `depth_on_demand` (full depth frames, only while the depth probe runs or a `host_jpeg` stream is watched). Both: `features`, `imu`, `snapshot` (the camera's JPEGs; the depth snapshot raw), `depth_stream` and `depth_stream_status` (only while the live stream is watched), `device_log`.
- **Streams (up, ↑):** `depth_points_rois` (`points` with ROIs from the Pi: the 56 B tag, then the config, 56 B + 45 B per window, about 14 kB per frame of 300 features) and `control` (gate on/off and snapshot requests, 56 B each). They are summed apart in `streams_up_sum_mbps` and compared with the measured up rate; `streams_sum_mbps` is the down streams only. Each entry has `direction` (`down` or `up`).
- **Fields** (`telemetry.link`): `available`, `reason`, `source` (`"xlink profiling"`), `usb_speed` (e.g. `SUPER`, `null` on PoE), `protocol` (`USB` or `TCP/IP (PoE)`), `down_mbps` / `up_mbps` (the last second, 1 Mbps = 10⁶ bit/s), `down_mbps_10s`, `up_mbps_10s`, `peak_down_mbps_60s` (the highest 1 s rate in the last 60 s), `totals_bytes` (`{down, up}` since the camera opened), `streams` (`[{name, description, mbps, msgs_per_s, bytes_per_msg}]` over the last 10 s), `streams_sum_mbps`, `unaccounted_mbps`, and `share_of_100mbps` (the 10 s mean as a fraction of 100 Mbps; `peak_share_of_100mbps` for the peak). The window resets when the camera reconnects.
- **100 Mbps:** the share is against the raw Ethernet rate. The usable rate on a PoE camera is lower (TCP/IP and Ethernet overhead), so a share near 1 does not fit.

### Live depth stream

The live panel shows, at about 5 fps while its *stream* box is ticked, either the camera image the VIO tracks its features on (*Live camera*, `mono_features`, the default) or the stereo depth (*Live depth*, the other variants). The *features* box draws the PnP tracker's features on it, in the [widget's styles](#cockpit-widget). For depth, the *colorize* box colours it in the browser (near is red, far is blue, invalid is black; unticked, gray linear in depth, near bright) and the legend shows the scale drawn; for the camera image there is no depth legend, only the feature keys. The panel also shows the depth at the image centre (the median of the centre 5×5 pixels and the centre pixel, from the VIO's own depth frame), the frame rate, the bandwidth and the source (`stereo.rectifiedLeft`, `stereo.disparity` or `stereo.depth`). It is the `<img>` of `GET /api/stream/depth` (MJPEG), so it also opens in a browser tab or in VLC.

The frame is made 8-bit (if it is not already) and JPEG-encoded on the camera, and the Pi only forwards the bytes; `host_jpeg` (below) encodes the depth on the Pi instead ([`app/depth_stream.py`](app/depth_stream.py)):

- On the camera, a `Script` node takes `StereoDepth.rectifiedLeft` (`mono_features`: RAW8, the image the `FeatureTracker` gets), `StereoDepth.disparity` (`disparity`: RAW8) or `StereoDepth.depth` (the other camera variants: RAW16 mm) in a size-1 non-blocking queue, all on the rectified left grid, 640×400. It forwards every n-th frame (n = fps / stream fps; 3 at 15 fps). `mono_features` and `disparity` send StereoDepth's own frame on unchanged, straight into the encoder; the depth variants convert it to an 8-bit gray frame first, with two 256-byte lookup tables on the high and low bytes (`bytes.translate`, 50 rows at a time). A `VideoEncoder` in MJPEG mode (quality 70) encodes it, directly or after an `ImageManip` depending on the [variant](#encoding-variants); its input is non-blocking with size 1, so a busy encoder drops stream frames, never VIO frames.
- What the gray means depends on the variant; `/api/status` → `depth_stream.encoding` says which, and the page and the widget draw their legend and colours from it:
  - `gray8-mono-rectified-left` (`mono_features`): the rectified left camera image, not depth. The browser shows it as it is and draws the tracked features on top.
  - `gray8-depth-linear` (`manip`, `nv12`, `yuv400p`, `host_jpeg`): linear in depth, 255 at *Stereo Depth Min*, 16 at *Stereo Depth Max*, 0 for invalid or out-of-range pixels, in 2^k mm steps (16 mm for a 4 m max). The browser treats gray < 8 as invalid, which absorbs JPEG ringing. The scale is in `depth_range_mm`, `gray_near`, `gray_far` and `quantization_mm`.
  - `gray8-disparity` (`disparity`): the stereo disparity in pixels, 0 to 95, 0 for invalid. Depth = focal length (px) × baseline / gray, so near is bright and far is dark and coarse: one gray level is one pixel of disparity, about 0.5 m at 4 m with the OAK-D-W (75 mm baseline, fx ≈ 440). Depths closer than focal × baseline / 95 (about 35 cm) are not representable. `depth_stream.disparity` has `focal_px`, `baseline_m`, `depth_times_disparity`, `max_disparity`, `gray_far`, `gray_near`, `nearest_mm` and `gray_invalid_below` (3); the legend is marked in metres at those gray levels.
- `/api/status` → `depth_stream` also gives `fps`, `kbps` and `center`. The centre depth is always measured from the VIO's own depth frame, not the JPEG.
- With nobody watching, the Script blocks on its control input, so it converts, encodes and sends nothing. Each `/api/stream/depth` client is a viewer in `main.py`'s [viewer registry](#images-on-demand-and-viewers), which keeps `depth_stream.lease` in the telemetry directory fresh (every second) while there is one, for 4 s after the last one leaves, and then removes it. The VIO loop checks the lease every 0.5 s and sends the Script on/off. The stream stops within about 0.5 s of the lease going (about 4.5 s after the last client).
- In `points` mode the centre depth comes from the camera too: one more 5×5 window at the image centre in every SpatialLocationCalculator config (no depth thresholds) and its centre pixel (`center.source` = `depth_points`). `host_jpeg` needs the full depth frame: in `points` mode the VIO asks the camera's full depth gate for every n-th frame only while the stream is watched (`depth_frames` = `on_demand`), and the panel says "fallback: raw depth over the link, encoded on the Pi".
- The VIO loop takes the newest encoded frame (`tryGetAll`, non-blocking) and writes it atomically to `/dev/shm/oakd-vio-telemetry/depth_stream.jpg`. `main.py` polls the file every 50 ms and sends each new one as a multipart part. The centre depth goes to `depth_stream.json` once a second while streaming.
- The Script uses only the camera's own Script API, which is smaller than the host's `dai.ImgFrame`: `ImgFrame(size)` (the size in bytes; `ImgFrame()` does not exist there), `setData`, `setType`, `setWidth`, `setHeight`, `setTimestamp` and `setSequenceNum`. There is no `setStride`, `setMetadata` or `setTimestampDevice`. The list was checked against the RVC2 firmware that depthai 3.3.0 ships, and the tests run the Script of every variant against fakes that accept only that API.
- The camera's `VideoEncoder` takes `NV12`, `YUV400p` and, from StereoDepth's pool, `RAW8` (test14: the RAW8 disparity and the rectified images encode directly). It drops anything else with a device log warning per frame ("Arrived frame type (30) is not either NV12 or YUV400p (8-bit Gray)"; 30 is `GRAY8`, 14 is `RAW16`) and outputs nothing. The test fakes drop it the same way.
- Errors: the Script counts each frame that fails, and sends the count and the latest error on its `status` output: the first one at once, then at most one a second. It also logs the 1st and every 100th with `node.warn`. A metadata failure still sends the frame. `/api/status` → `depth_stream` has `errors`, `last_error` (`text`, `unix_time`, `fatal` if the Script stopped) and `device_stats` (the camera's convert + send time, every 150 frames). The panel shows them as *camera conversion* and *camera errors*, and the widget's status line shows the error count.
- Encoder check: while streaming, the Script also reports how many frames it has sent, about once a second. If that count keeps rising for more than 3 s after its first report since the last JPEG, and no JPEG comes back, `depth_stream.encoder_stalled` is `true`. `last_error` then reads "camera converts frames but the encoder outputs nothing: …" with the variant, and the VIO log has it once. A gap in the reports longer than 2.5 s or 2.5 frame intervals restarts the 3 s, so a pause in depth frames is not mistaken for it. The next JPEG, or the end of the stream, clears the flag. `host_jpeg` sends the same reports (frames offered to the Pi's encoder), so the check works for every variant. The panel shows it under *camera errors* as "encoder outputs nothing: …"; after it clears, the row keeps "none now · last: …" with the age, so a stall that ended with the stream is still visible. The widget shows "depth: camera encoder outputs nothing".
- Blank check: JPEGs that come back but show nothing. A camera variant's frame counts as blank when it is byte-identical to the previous one or its decoded gray has a standard deviation below 0.75 (checked at most every 0.5 s). The VIO process samples its own depth frame once a second; when that has at least 5 % valid pixels with varying depth (and isn't more than 3 s old), and the JPEGs stayed blank for 2 s and at least 3 samples, `depth_stream.blank` is set (`variant`, `frames`, `bytes`, `gray`, `text`: "blank frames from manip: 10 constant JPEGs (4668 bytes, gray 1) in 2.0 s while the camera depth varies (40 % valid)"), `last_error` gets it with `blank: true`, and the log has it once. A covered lens, open water or a flat wall filling the view gives a constant image too, but then the depth itself is constant, so it is not flagged. A varied frame clears it. The widget shows "depth: blank frames from …". Frames that are not JPEGs at all (empty) are counted in `bad_frames` and not shown.
- Cost: on the Pi, one file write per streamed frame and a 5×5 median once a second. A sparse depth frame from the stage-1 snapshots (`snap4`, 13 % valid) is 26 KB as a quality-70 JPEG (OpenCV's encoder), so about 130 KB/s (1 Mbit/s) on the link at 5 fps. The Script logs its own conversion time on the camera every 150 frames ("depth stream: … ms") and reports it in `device_stats`; it has not been measured on hardware yet. If it competes with the VIO, lower *Depth stream fps* or turn *Live depth stream* off, which removes the Script and encoder from the pipeline.

#### Encoding variants

On an OAK-D-W (test11) the Script sent `YUV400p` frames, 640×400 at 5 fps, with no errors, and the encoder returned nothing: `/api/stream/depth` answered 200 with no bytes and `encoder_stalled` came up after 3.6 s, with no VideoEncoder lines in the log at the default level. On test12, `nv12` and `yuv400p` gave 0 bytes with no log either, and `manip` gave constant frames (every JPEG 4668 bytes, every pixel 1) while `ImageManip` logged "Pool size of the input message is too small... Currently it is 0". *Depth Stream Encoding* chooses how the frame reaches an encoder, so each candidate can be tried on the camera without a rebuild.

| Variant | Pipeline | Encoded on |
|---|---|---|
| `auto` (default) | `mono_features`. Switches to `host_jpeg` (the depth) for the rest of the run the first time the encoder stalls or its frames are blank (logged once; `fallback` in `/api/status`). Never picks `disparity` | camera, then Pi |
| `mono_features` | `StereoDepth.rectifiedLeft` → Script (on/off gate only: forwards StereoDepth's own RAW8 frame) → `VideoEncoder`. The browser draws the tracked features on it | camera |
| `disparity` | `StereoDepth.disparity` → Script (gate only) → `VideoEncoder`. Only with an 8-bit (RAW8) disparity, predicted from the applied stereo config; otherwise it runs as `host_jpeg` with the reason (e.g. "disparity is RAW16 (max disparity 8192) with the speckle filter on (it runs after the median); turn the speckle filter off or try Median filter last (stereo_median_last, experimental); or use mono_features") | camera |
| `manip` | Script → `GRAY8` frame → `ImageManip` (converts to `NV12`) → `VideoEncoder`. Blank on test12 | camera |
| `nv12` | Script → `NV12` frame (the gray plane and a 128 chroma plane, w×h×3/2 bytes) → `VideoEncoder`. No output on test12 | camera |
| `yuv400p` | Script → `YUV400p` frame (w×h bytes) → `VideoEncoder`: the test11 pipeline. No output on test12 | camera |
| `host_jpeg` | nothing extra on the camera: the VIO process converts the depth frame it already has with the same tables and JPEG-encodes it with OpenCV on a worker thread | Pi |

`manip`, `nv12` and `yuv400p` are kept for diagnosis. Why they fail and why `mono_features` and `disparity` work, from the RVC2 firmware that depthai 3.3.0 ships (a406121b), the depthai-core v3.3.0 source, test12, test13 and the stereo probe (test14):

- A frame a Script creates (`ImgFrame(size)`) comes from a pool of size 0. `ImageManip` requires an input pool of at least 2 ("Pool size of the input message is too small... Currently it is {}") and then outputs a constant image; the `VideoEncoder` drops such frames without a log. That explains all three test12 results.
- A frame the Script received and sends on unchanged keeps the pool it came from (test13). StereoDepth's outputs have their own pool (3 by default; the pipeline adds one per stream queue, 3 for the forwarding variants), and the encoder takes their RAW8 frames directly (test14), so neither variant needs an `ImageManip`.
- When the disparity is RAW8: depthai-core's `getMaxDisparity` is 95 (63 with a 64-pixel search, 175 with companding) plus the disparity shift, ×2 with extended disparity, ×2^bits with subpixel, and 8192 when the last post-processing filter that runs is not the median. RAW8 needs that to be at most 255. The ROBOTICS preset's `filteringOrder` is DECIMATION, MEDIAN, SPECKLE, SPATIAL, TEMPORAL, so with the saved settings (speckle on) the speckle filter runs after the median and the disparity is RAW16 even with subpixel off (probe case A: max disparity 8192). Cases B–D and F (speckle off, or median off with nothing after it) are RAW8 with max 95. `stereo_config.disparity_format` makes that prediction from the config read back from the node, falling back to the settings field by field; `/api/status` → `depth_stream.disparity_format` (`type`, `max_disparity`, `executed_filters`, `filtering_order`, `reason`) and `disparity_problem` show it. *Median filter last* moves the median to the end of `filteringOrder` (probe case G checks it on the camera).
- RAW16 neither encodes (the encoder warns "frame type (14)") nor converts through `ImageManip` (test14: "Processing failed, potentially unsupported config" and a constant image). `StereoDepth.depth` is always RAW16.
- Without subpixel, depth steps are one pixel of disparity (about 0.12 m at 2 m, 0.5 m at 4 m, with fx ≈ 440), which changes the VIO's depth too, so `disparity` is a trial, not the default. The disparity output has the same confidence threshold, depth range and post-processing filters as `StereoDepth.depth` (depthai filters in the disparity domain, then converts). The VIO still uses `StereoDepth.depth`.
- The extra on the camera for `mono_features` and `disparity` is the encoder; the Script does no per-pixel work. Both run only while a client streams, and their queues are non-blocking, so a slow encoder drops stream frames, not VIO frames.

If the width is not a multiple of 32, `auto` goes to `host_jpeg` from the start (and `disparity` too when decimation makes the disparity's width unsuitable).

The `VideoEncoder` (MJPEG) requirements, from the strings and asserts of the encoder plugin in the RVC2 firmware that depthai 3.3.0 ships (a406121b) and from the depthai-core source:

- type `NV12` or `YUV400p` ("Arrived frame type ({}) is not either NV12 or YUV400p (8-bit Gray)"), or a pooled `RAW8` frame from StereoDepth (test14);
- width a multiple of 32 and height a multiple of 2 ("Width must be multiple of 32, height multiple of 2 for MJPEG encoder profile"), at most 16384×8192;
- every frame the size of the first ("Arrived frame ({}, {}) does not match the encoder size");
- `YUV400p` needs an extra gray chroma buffer the encoder reserves ("Encoder couldn't reserve extra space for gray chroma buffer"); `NV12` does not;
- source, timestamp and sequence metadata present (asserts on all three), so the Script copies the depth frame's timestamp and sequence number;
- a 64-byte aligned buffer base and size (asserts). Pool buffers are aligned; a Script's `ImgFrame(size)` buffer is not guaranteed to be;
- only one of its `bitstream` and `out` outputs linked (we link `out`);
- at the DEBUG log level it logs "VideoEncoder took '{}' ms." for each frame.

`host_jpeg` is the last resort: it uses Pi CPU, and the panel says "encoded on the Pi". The encoder thread is capped at 25 % of one core on average: after an encode that took c seconds of thread CPU, the next one waits c × 3, and frames arriving earlier, or while one is being encoded, are skipped. `/api/status` → `depth_stream.host_encoder` has `cpu_percent` (last 5 s), `cpu_budget_percent`, `frames`, `offered`, `taken`, `skipped_cpu`, `skipped_busy` and `errors`. A 640×400 frame took a few ms on the x86 desktop; the Pi cost is not measured yet.

`/api/status` → `depth_stream` reports `variant` (running), `requested_variant`, `variant_reason`, `configured_encoding` (the saved setting), `encoding`, `source` (the StereoDepth output shown; it follows a fallback), `encoded_on` (`camera` or `pi`), `fallback` (`from`, `to`, `reason`, `detail`, `unix_time`, `at_start`), `blank`, `bad_frames`, `size_problem`, `disparity_format` and `disparity_problem`. The fallback reasons are "no JPEG from <variant> (the camera encoder outputs nothing)", "blank frames from <variant>", the RAW16 disparity reason, or the size problem. The live panel shows them on its *encoding* row ("fallback: …").

#### Debug mode (`depth_stream_debug`)

The VIO process always registers a device log callback and keeps the camera's log lines: the last 30 from the stream's nodes (its Script, `VideoEncoder` and `ImageManip`) and the last 10 others, each cut at 300 characters. They are in `/api/status` → `depth_stream.device_log` (`stream`, `other`, `total`) and in the panel's *camera log* box. With *Depth Stream Debug* on, the device log level is raised to DEBUG while a camera variant streams and restored to the previous level when the stream stops or falls back to the Pi (`debug_state`: `active`, `level`, `restore_level`, `error`). At DEBUG the encoder logs "VideoEncoder took '…' ms." per frame, which shows whether frames reach it at all. Other nodes log more at DEBUG too, so leave it off in normal use.

### Images on demand and viewers

No image crosses the camera link unless someone is looking at it. The gates, on the camera:

| What | Gate | Sent while |
|---|---|---|
| Live stream (`mono_features`, `disparity`, depth variants) | the stream Script blocks on its control input | a `video` viewer (or its 4 s linger) |
| Live stream `host_jpeg` (`points`) | the full depth gate Script (every n-th depth frame) | a `video` viewer, and the stream runs on the Pi |
| Full depth frames (`points`) | the full depth gate Script | the depth probe runs, or the above |
| Snapshots (`rect_left`, `rect_right`, `rgb`, `depth`) | the snapshot Script, one frame per request | a snapshot is requested |
| Features (overlay) | not gated (the PnP needs them anyway) | always |

In `full_frame` the depth frame crosses on every frame for the PnP, so `host_jpeg` adds nothing on the link there.

`main.py` counts the clients of `/api/stream/depth` (`video`), `/api/stream/features` (`features`) and `/api/stream/pose` (`pose`) in a viewer registry ([`app/viewers.py`](app/viewers.py)). Each client opens on connect and closes when it goes; `?client=page|widget` (the page and the widgets add it) says who it is, anything else counts as `other`. While a kind has a viewer, the registry renews its lease file every second (features and pose also renew the status lease); 4 s after the last viewer of a kind leaves (the linger, so a page reload or a widget re-layout does not stop and restart the camera stream) it removes the lease, unless a one-shot request (`/api/features`, `/api/pose`) renewed it since. A daemon thread ticks the registry every 0.5 s, so the linger also ends with nobody polling.

`/api/status` → `viewers`: `total`, `by_kind`, `by_client`, `video`, `video_idle` (no video viewer, nothing lingering), `lingering_s` (seconds left per kind), `linger_s`, `opened_total` and `text`: "1 watching video, 1 other", "video stopping (no viewer)" during the linger, "video idle" or "video idle (2 viewers without video)". The page shows it in *Depth for PnP* and on the live panel; the widget's status line shows the text.

`/api/status` → `image_streams.streams` lists every image stream that can cross the link (the live stream, then each snapshot stream): `codec` (`mjpeg`, or `png` for a raw frame encoded on the Pi), `quality`, `encoded_on` (`camera` or `pi`), `over_link` (`jpeg` or `raw`), `viewed`, `mbps` (the live stream's per-stream link estimate, 10 s mean, while viewed), `last_bytes` (snapshots, one frame per request), `measurement` (the depth snapshot: raw 16-bit data, not a fallback), `reason` and `fallback`. `image_streams.fallbacks` collects the fallback texts, which the widget shows. The page's *Image streams* box shows one row per stream.

### Cockpit widget

The extension offers a Cockpit widget, *OAK-D VIO depth*. It shows the live stream (the rectified left camera image by default, else the depth) with the PnP tracker's features drawn on top and a status line. In Cockpit, open Edit mode; the widget is in the widget list, offered by BlueOS. It is also the page `widget/depth`, linked from the live panel.

- **Discovery:** `GET /register_service` declares `works_in_relative_paths` (BlueOS proxies the extension at `/extensionv2/oakdvio/`) and `extras.cockpit: /cockpit_extras.json`. Cockpit fetches the extras JSON through that proxy. The widget entry uses `useExtensionPathAsBaseUrl`, so the iframe is `http://<vehicle>/extensionv2/oakdvio/widget/depth` and works from a remote Cockpit too. Every URL in both pages is relative, so they work on port 8095 and behind the proxy.
- **Image:** the same MJPEG as the live panel. With `mono_features` it is the camera image as it is, labelled "camera (rectified left) + features", with no depth legend; with a depth variant it is colorized on a canvas, with the depth-range legend.
- **Features**, coloured by state:
  - *inlier* (green): matched to the reference and kept by RANSAC;
  - *rejected* (magenta square): matched and rejected by RANSAC;
  - *matched* (white ring): matched, no RANSAC result;
  - *depth* (light blue): not in the reference, with valid depth;
  - *no depth* (grey ring).
  Positions are on the depth image's pixel grid (scaled when the depth is decimated), with `image_size` (the rectified left grid) alongside. [`app/static/features_overlay.js`](app/static/features_overlay.js), shared by the widget and the main page, maps them back to that grid, onto the pixel centre, then to where the image sits in the view (`object-fit: contain` letterboxing), so they land on the same pixels in the camera image and in the depth image (`app/tests/test_features_overlay.py` runs it under node).
  - **Sync:** with `mono_features` the VIO publishes, with each camera image it writes, the features of the frame that image came from (matched by sequence number among the last 30 frames; `synced: true`, `image_sequence`), instead of every 0.2 s; the server sends both within its 50 ms polling. If that frame was dropped, the nearest one goes out with `synced: false`. Otherwise the latest features are drawn.
- **Status line:** VIO fps, ok %, inliers, features with depth, VISION_POSITION_DELTA messages sent, the log-only flag, the last failure's reason, the [camera link](#camera-link-bandwidth) (10 s mean, 60 s peak, share of 100 Mbps), "depth points 99%" (the share of frames that got their depth, red below 90 %) or "depth: full frames", and the viewers ("1 watching video", "video idle"). Raw image fallbacks are warnings: "depth: fallback: raw depth over the link, encoded on the Pi" and "snapshots: fallback: raw frames over the link, encoded on the Pi" (the reason in the tooltip). When there is nothing to show, it says why (VIO stopped, not PnP, depth stream off).
- **Cost on the Pi:** nothing is decoded or drawn on the Pi.
  - While a widget is connected (a `features` viewer, kept 4 s after it leaves), `features.lease` is kept fresh. The VIO loop then writes `features.json` (about 4 KB for 300 features, one line of JSON) at most every 0.2 s. That took 0.22 ms per write on the x86 desktop.
  - The web server forwards the bytes unchanged and adds one small status event per second.
  - With nobody watching, the VIO loop does one clock comparison per frame and one stat every 0.5 s.
  - The page closes both streams while it is hidden.
- **Options** (in the iframe URL): `?colorize=0` shows gray depth (a depth variant), `?features=0` hides the features, `?status=0` hides the status line.

### 3D pose viewer

The *3D pose* panel on the main page, and the second Cockpit widget *OAK-D VIO 3D pose* (`widget/pose`, declared in `/cockpit_extras.json` with relative URLs and `useExtensionPathAsBaseUrl`, like the depth widget), show the PnP VIO's integrated pose in 3D:

- the vehicle's position and orientation as body axes, a small hull and the camera frustum (70°×45°, pitched down by *Camera Angle*);
- the trail, a 1 m grid at the start depth, the start axes, and a dashed line from the vehicle up or down to the grid that shows the depth change;
- x, y, z (m) and roll, pitch, yaw (°) as labelled numbers, the frame, and the attitude source with its age ("gyro-integrated, drifting" is shown as a warning);
- orbit controls (drag to rotate, right-drag or two fingers to pan, wheel to zoom), *Reset view* (frames the vehicle and its trail), *Clear trail* (this browser only, kept per VIO run) and *Follow* (keeps the vehicle in the middle).

**Attitude** (*3D Pose Attitude*, `pose_attitude_source`; viewer only, the MAVLink output is unchanged):

- `autopilot` (default): ArduPilot's `ATTITUDE` (roll, pitch, yaw of the body relative to NED). The VIO process reads it from mavlink2rest, read only: one GET of `…/vehicles/1/components/1/messages/ATTITUDE` every 0.5 s with a 0.4 s timeout, every 5 s while it is missing. Between reads, the attitude is carried forward with the gyro rotation since the sample's time. A sample older than 3 s (mavlink2rest keeps the last message forever) is stale; the attitude then continues from the last one with the gyro deltas, labelled **gyro-integrated, drifting**, until a fresh sample arrives.
- `gyro`: the gyro rotation deltas chained from the start, always labelled **gyro-integrated, drifting**. On test12 it drifted about 0.65°/s at rest (the BMI270's bias; nothing estimates it).

Neither is a filter: no bias estimate, no accelerometer correction. The gyro prior and gate of the VIO are unchanged.

**Frames:**

- `NED`: x north, y east, z down, in metres from the VIO start position, while the autopilot attitude is or has been in use.
- `start-FRD`: the vehicle body frame at VIO start (x forward, y right, z down); NED with north at the initial heading if the vehicle started level. Used with `gyro`, and with `autopilot` until the first sample.

roll, pitch and yaw are ZYX (aerospace) Euler angles; `q` is the same rotation as a quaternion `[w, x, y, z]`. In the browser the frame is drawn y-up: FRD (x, y, z) is three.js (y, −z, −x); the legend names the axes of the current frame.

**Integration** ([`app/pose_stream.py`](app/pose_stream.py)): each accepted frame adds the displacement sent in `VISION_POSITION_DELTA` (body FRD), rotated by the attitude in use: p ← p + R·position delta. The 2D trajectory panel is a yaw-only gyro version. Like any dead reckoning the position drifts, and the motion during a [reference reset or recovery](#failed-frames-pnp) is not in it.

**Switching sources** never makes the trail jump:

- The first autopilot sample switches `start-FRD` to `NED`: the position and the trail so far are rotated about the start by R_autopilot·R_gyroᵀ and redrawn (new epoch). Normally mavlink2rest answers before the camera has opened, so the trail is still at the origin and nothing moves.
- Autopilot → gyro (stale): continues from the last autopilot attitude; nothing moves, and the frame stays `NED` while the heading drifts.
- Gyro → autopilot (fresh again): the attitude snaps back by whatever the gyro drifted meanwhile (a few degrees after a short loss); the position stays, and later displacements go in the right direction again.

`/api/status` → `pose` has `configured_source`, `frame`, `frame_text`, `attitude_mode`, `attitude_source` (`autopilot` or `gyro`), `attitude_text`, `attitude_age_s`, `switches`, `last_switch`, `realigned` and `autopilot` (`fresh`, `age_s`, `stale_after_s`, `reads`, `samples`, `errors`, `last_error`, `description`). Each `pose` event carries `frame`, `attitude_source`, `attitude_text`, `attitude_age_s`, `roll`/`pitch`/`yaw` (rad), `roll_deg`/`pitch_deg`/`yaw_deg` and `units`.

**Feed:** like the feature stream. While a viewer is connected to `GET /api/stream/pose` (and 4 s after), `pose.lease` is kept fresh; the VIO loop then writes `pose.json` (about 300 bytes) at most 10 times a second and `pose_trail.json` (`[t, x, y, z]` points 5 cm or 2 s apart, at most 1500, thinned to every other point when full) at most once a second, and every 2 s anyway. The server sends the trail once per connection and then each pose; the viewer extends the trail from the poses. With nobody watching, the VIO loop does the integration (a few 3×3 products per frame) and one stat every 0.5 s. Both streaming routes are left out of the OpenAPI schema. The page opens the stream only while the panel is on screen and the tab is visible.

**three.js:** r156 (npm `three` 0.156.1), MIT license, vendored in [`app/static/vendor/three/`](app/static/vendor/three/) with its `LICENSE` and a `README` that gives the sha256 of the unchanged `three.module.min.js` and the one change to `OrbitControls.js` (its import of `'three'` points to the local file, so no import map is needed). Nothing is loaded from a CDN. The main page loads the viewer module only when the panel scrolls into view.

### DVL overlay

With *DVL overlay source* `auto`, `main.py` ([`app/dvl_source.py`](app/dvl_source.py)) looks for a DVL in mavlink2rest and the vx/vy plot draws its velocity dashed:

- Message: `VISION_POSITION_DELTA`, which the Water Linked DVL extension sends to ArduSub; velocity = `position_delta / time_delta_usec`, in the same body frame as ours, so the overlay compares exactly the two inputs the EKF would fuse. Our own message (system 1, component 195) is excluded.
- Component: found automatically. The whole message tree (`GET /mavlink2rest/mavlink/vehicles`) is read at most every 10 s until another component's `VISION_POSITION_DELTA` turns up, then only that message is polled, at most twice a second with a 0.4 s timeout, and only while the page asked for history in the last 10 s. A sample is taken when mavlink2rest's message counter advances. After about 3 s of failed polls the source is dropped and searched for again.
- The overlay only has samples from when the page started asking (up to 10 minutes kept). Nothing here raises; without a DVL the page shows "DVL not found".
- Not verified on a vehicle: it needs the DVL's messages to reach mavlink2rest's MAVLink endpoint.

### ArduPilot panel

The panel reads the [recommended parameters](#autopilot-parameters) with `PARAM_REQUEST_READ` and shows current, suggested (editable), status (`ok`, `differs`, `unreadable`), why and *reboot*, plus the armed state from the autopilot's HEARTBEAT (`base_mode & 128`). Reading (`POST /api/autopilot/params/read`, on load and on *Re-read*) never writes; it also refreshes the [parameter check](#the-check-and-the-configure-button) at the top of the page. Both reading and writing are POST, so no GET (BlueOS's API scan, a browser prefetch) reaches the autopilot.

Writing (`POST /api/autopilot/params`):

- The page's *Write selected…* (and the check's *Configure…*, with exactly the differences) opens a dialog listing each change (`NAME old → new`, reboot flag) and the armed state; a second, explicit click sends `{"confirm": true, "params": {...}}`. Nothing is written automatically or at startup.
- 400 without `"confirm": true`, for any name outside the suggested table, and for values outside the parameter's range or allowed values (e.g. `VISO_TYPE` 0–3, `VISO_POS_*` ±5 m, `VISO_ORIENT` 0/2/4/6/24/25); one bad entry rejects the whole request.
- 409, nothing written, unless a HEARTBEAT that arrives during the request (its counter must advance within 2.5 s) says disarmed: armed, no HEARTBEAT, a stale one, or no mavlink2rest all refuse. The armed bit is checked again before each `PARAM_SET`; if the vehicle arms midway, the rest are reported as failed and not written.
- Each written parameter is read back with a fresh `PARAM_REQUEST_READ`. The response lists `changed` (`name`, `old`, `new`, `readback`), `failed` (with the error, e.g. a read-back mismatch), `unchanged` and `reboot_required`. The autopilot is never rebooted by the extension.

### Demo server and screenshots

[`app/tools/ui_demo_server.py`](app/tools/ui_demo_server.py) runs the real web app against fakes, for working on the page without a vehicle: it replays a PnP data log at real time into the real telemetry code, serves generated camera frames through the real snapshot and depth-probe services, and runs [`app/tools/fake_mavlink2rest.py`](app/tools/fake_mavlink2rest.py) (an autopilot with parameters and HEARTBEAT, and a DVL) on 127.0.0.1. It never contacts another host. [`app/tools/ui_screenshots.py`](app/tools/ui_screenshots.py) takes full-page screenshots with headless Chrome.

```bash
python app/tools/ui_demo_server.py --log vio-20261118-190503-10.jsonl --port 8099            # add --armed, --no-dvl
python app/tools/ui_screenshots.py --url http://127.0.0.1:8099/ --out /tmp/ui
```

### Mock mode (no OAK-D, no autopilot)

`OAKD_MOCK=1` runs the extension's own server on a PC (x86 is fine), with the configuration page and the Cockpit widgets (`/widget/depth`, `/widget/pose`, `/cockpit_extras.json`, `/register_service`) fully working. Unlike the demo server, it is the normal startup: `main.py` launches [`app/mock_vio.py`](app/mock_vio.py) through the same supervisor, instead of `pnp_vio.py`. The fake VIO publishes through the real telemetry, depth-stream, feature-stream, snapshot and depth-probe services, so every server path runs as with a camera. depthai is never imported.

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt          # tests also need: .venv/bin/pip install pytest httpx
cd app
OAKD_MOCK=1 ../.venv/bin/python -m uvicorn main:app --host 127.0.0.1 --port 8095
# or: ../.venv/bin/python main.py --mock --host 127.0.0.1 --port 8095
# open http://127.0.0.1:8095/, http://127.0.0.1:8095/widget/depth and http://127.0.0.1:8095/widget/pose
```

- **Data:** config, snapshots, telemetry and logs live in `OAKD_MOCK_DIR` (default `/tmp/oakd-vio-mock-<uid>`). On first start the configuration is PnP, 640 x 400 at 10 fps; settings, Start/Stop/Reset and log-only work as usual.
- **Camera:** a synthetic camera, in `mock_vio.py`:
  - a panning seabed texture;
  - depth on a tilted plane whose distance drifts, with three nearer blobs moving across it and no depth in dark spots, the left edge band and outside the stereo range;
  - a right image warped by the matching disparity.
- **Features:** tracked features in every state. Most of those on a moving blob are rejected.
- **Failures:** a turbid spell of 2.5 s every 50 s (counted from the start), where depth drops out and frames fail.
- **Motion:** a lawnmower survey that closes every 164 s: four 24 s lanes at 0.35 m/s (about 8.4 m), joined by 90° turns around a short step between lanes. Each step also changes the depth by 0.4 m: two steps down, then two back up. A slow roll and pitch wobble (±2°, ±1.5°) runs on top. The deltas go through the same 3D pose path as on the camera, so the [3D pose viewer](#3d-pose-viewer) draws the survey; the turbid spells (reference resets) make it drift a little, as on the camera.
- **Live data:** telemetry at the configured fps, chip temperature, the depth MJPEG at `depth_stream_fps` and the pose, each only while a client holds its lease. The host CPU is the real one.
- **Depth stream encoding:** the camera variants are emulated (`variant`, `encoding`, `encoded_on`, `source`, the frame size and the disparity format checks, a device log with "VideoEncoder took … ms. (mock)" lines while *Depth Stream Debug* is on). `auto` and `mono_features` send the synthetic rectified left image the mock features are on (with the synced features), `disparity` the disparity image, the others the linear depth image. The mock's camera variants all work: the test12 failures (blank `manip`, empty `nv12` / `yuv400p`) are only in the unit tests' strict fakes. `disparity` with the saved settings (RAW16) and `host_jpeg` use the real `HostJpegEncoder`, CPU cap included. `python app/tools/ui_screenshots.py --url http://127.0.0.1:8095/ --scenario live` screenshots the live panel and the widget.
- **Autopilot attitude:** an emulated `ATTITUDE` (10 Hz, the survey's attitude with the heading 30° from the start heading) feeds the 3D pose's `autopilot` source. It pauses for 8 s every 120 s, so the viewer shows the switch to "gyro-integrated, drifting" and back; the mock gyro has a 0.3°/s yaw bias (pose viewer only) so the drift is visible. `OAKD_MOCK_ATTITUDE` = `gaps` (default), `on` (no gaps) or `off` (no autopilot).
- **Camera link:** synthetic XLink counters, labelled `mock (synthetic XLink counters)` and `"mock": true`: down is the bytes the fake VIO counts for its streams (the same accounting as on the camera) + 1 % + 1.5 kB/s, up the counted up streams + 0.8 kB/s; USB SUPER. With `points` about 4.1 Mbps down (`depth_points` 2.2, `features` 1.5, `imu` 0.3) and 1.1 Mbps up (`depth_points_rois`) at the mock's 10 fps; with `full_frame` about 43 Mbps down. They only show that the box works, not what a camera sends. `--scenario link` of `ui_screenshots.py` screenshots the box and the widget line.
- **Depth transport:** `points` (the default) and `full_frame` as described in [Depth at the features](#depth-at-the-features-pnp-depth_transport), with both ROI sources, the full depth gate, the camera load (its Leon CPU follows the emulated work) and the viewers.
- **Snapshots:** each snapshot shows the scene at its request time. `rect_left` / `rect_right` come as JPEGs (OpenCV standing in for the camera's encoder, quality 95, counted at their size), `depth` raw. The depth probe and the focal check therefore give real results on the synthetic frames. RGB is "not available".
- **Autopilot:** simulated in memory (`mock_mode.MockAutopilot`), with the dev vehicle's parameters, so the [parameter check](#the-check-and-the-configure-button) warns about `VISO_POS_X/Y/Z` and `EK3_SRC1_POSXY`. *Configure…* writes the in-memory values (until the server restarts). The HEARTBEAT is disarmed; `OAKD_MOCK_ARMED=1` makes it armed, so writes are refused (409).
- **DVL:** the overlay is fed by an in-process fake mavlink2rest.
- **Network:** urllib refuses every URL, so nothing leaves the machine. The fake VIO opens no MAVLink connection; "messages sent" counts what would have been sent.
- **Visible:** a purple MOCK badge on the page and in the widget's status line, and `"mock": true` in `/api/status` (and in the widget's status events).

## Log-only mode and data logs

On a vehicle whose EKF already uses ExternalNav (`VISO_TYPE` 1, `EK3_SRC1_VELXY` 6), every `VISION_POSITION_DELTA` the extension sends is fused. To record VIO without affecting the vehicle, turn on log-only mode in any of these ways:

- tick *Log only: do not send anything to the autopilot* on the configuration page,
- set `OAKD_LOG_ONLY=1` in the container environment; this forces log-only whatever the saved setting, and the page shows that it is forced,
- run `pnp_vio.py` or `basalt_configurable.py` with `--log-only`.

In log-only mode the VIO process neither reads nor writes autopilot parameters and creates no MAVLink connection. It logs `LOG-ONLY: no MAVLink output, autopilot params untouched` at startup. The web page's [parameter check](#the-check-and-the-configure-button) keeps running: it only reads (`PARAM_REQUEST_READ`) and writes nothing unless you confirm *Configure…*.

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

## Depth at the features (PnP, `depth_transport`)

The PnP needs the depth at its tracked features only, about 300 numbers per frame. With `full_frame` the whole RAW16 depth frame (512 kB at 640×400, 41.9 Mbps at 10 fps on test15, ~99 % of the link) crosses to the Pi, which looks the depth up itself. With `points` (the default) the depth frame stays on the camera ([`app/depth_points.py`](app/depth_points.py)):

- **On the camera:** a `SpatialLocationCalculator` (SLC, on the SHAVEs) gets one ROI per feature: the 5×5 window around it on the depth grid (clamped to the frame), `AVERAGE`, depth thresholds = *Min / Max depth* (exclusive bounds, so the range itself is taken). AVERAGE, not MEDIAN: the firmware computes MEDIAN only on a depth frame straight from StereoDepth ("Depth frame must be originating from StereoDepth node! Defaulting to 'AVERAGE/MEAN'"), and ours comes through the pairing Script (test16b: every ROI came back as an average). AVERAGE's AUTO step is 1, so all 25 pixels are sampled, like the Pi's own lookup (a median there). Three more ROIs: the centre 5×5 patch and the centre pixel (the live panel's centre depth) and a 1×1 marker at the bottom-right pixel (the depth size, so every message checks itself).
- **Who lists the ROIs** (*PnP Depth Points ROIs*): `host` (default): the Pi builds the config from the features it receives anyway (0.34 ms for 300 on the desktop) and sends it up right after a *tag*, about 12 kB per frame; `camera` (**slow**): a Script builds it from the `TrackedFeatures` (no upstream traffic, but 111 ms of Leon CPU per frame of ~255 features on test16b, Leon MSS 98 %, 84.5 % paired at 10 fps: it does not keep up with 10 fps). It cannot be made much cheaper: reading the feature positions and building the ROIs are most of it (a leaner variant was 9 % faster on the desktop).
- **The tag:** the camera reads 0 as the sequence number of every `SpatialLocationCalculatorConfig` the Pi sends (test16b: 0 % paired, every request "request 0"), while a plain `Buffer`'s sequence number survives the link (the gates' controls). So per frame the Pi sends a `Buffer` with the features' sequence number (the tag) and then the config, on the same queue; the Script pairs a config with the tag right before it, and reports a config without a tag or a tag without a config ("pairing" errors). It also reports once what the config's own number read on the camera (`device.config_sequence`: `survives` false as on test16b).
- **Sync (no stale depth):** the camera Script takes a request (the tagged config, or the features), takes depth frames from its input (non-blocking, the newest 4 frames: a request up to 2.5 frame periods late still finds its frame) until one has the request's sequence number (older ones are dropped; a newer one means the request's frame is gone: a miss, reported), sends the config and that frame to the SLC (its inputs are blocking, size 1, `inputConfig.setWaitForMessage(True)`) and waits for the SLC's output and passthrough depth before the next request: one config and one frame in the SLC. It then sets the output's sequence number and timestamp from the passthrough frame. The Pi pairs points and features by sequence number and checks the echoed ROIs against the windows it expects for that features message. That check, not the tag, is what makes the depth right: a mispaired request can cost a frame, never give it another frame's depth.
- **Pipelining:** the Pi requests each features message as soon as it fetches it, also the ones queued behind the frame it solves (at most 4 in flight, what the camera's `rois` queue holds: 8 messages; older queued ones are dropped unrequested, `host_behind_dropped`). So when the Pi is slower than a frame the camera already computes the next frames' depth. Why the depth goes through the Script, not `stereo.depth → SLC.inputDepth` (which would give MEDIAN): the SLC would then take whatever depth frame is at its input when a config comes (the only config-sync pattern in the depthai source takes the config first, then the head of the image input), which is pairing by timing, not by sequence number; making that input blocking would stall StereoDepth.
- **Missing depth:** a frame whose points are missing, late (2.5 frame periods, at least 50 ms; `camera` ROIs +150 ms for the Script's build), for a later frame, unreadable, have the wrong ROI count, size or windows, or a timestamp more than half a frame away from the features, gets no depth at all (all NaN), counted by reason (`not_requested`, `dropped`, `timeout`, `unreadable`, `count`, `size`, `roi`, `timestamp`). It still solves against the reference (the PnP needs the reference's 3D points and this frame's pixels only), but it cannot become the next reference: if it solved, the reference's RANSAC inliers are moved into this frame's camera with the solved pose and kept as the reference (`reference_carried` in the data log), so one missing frame costs no delta. After 5 timeouts in a row the Pi stops waiting until points come again.
- **On the Pi:** the same back-projection, depth range, `min_valid_depth_fraction` (of the 25 sampled pixels), *PnP Depth Scale* and refractive index as the frame lookup (`pnp_odometry.depth_from_points`). If a firmware filled only the other algorithm's depth (MEDIAN), it is used with its 9 samples and counted (`other_algorithm_rois`).
- **Rate:** nothing here is tied to 10 fps: the wait, the depth buffer and the timestamp check are frame periods. Measured limits (test16b, 640×400): the SLC takes 7–8 ms per frame of ~258 ROIs, so it is not the limit up to 30 fps; `camera` ROIs are (under 10 fps). Estimated link at 30 fps with ~255 features: `features` + `depth_points` ≈ 40 kB per frame, about 9.5 Mbps down, and 2.8 Mbps up. Not measured yet: StereoDepth (with subpixel and the speckle / median filters) and the FeatureTracker at 640×400 above 10 fps on the camera, the Script's own time per request, and the Pi's solve time per frame; the 20C report has the check list.
- **Full depth frames on demand:** the depth probe and a watched `host_jpeg` stream need whole frames; a gate Script forwards every n-th depth frame (`depth_on_demand`) only while one of them asks. Snapshots and the focal check use the snapshot gate as in `full_frame`.
- **Status** (`/api/status` → `depth_transport.live`, the page's *Depth for PnP* box): `frames`, `paired`, `paired_percent`, `missing`, `missing_by_reason`, `last_problem`, `valid_percent`, frame drops (`feature_gaps` / `feature_gap_frames` from the features' sequence numbers, `points_gaps` / `points_gap_frames`, `orphan_points`), `wait_ms_mean` / `wait_ms_max` / `wait_limit_ms`, `timestamp_diff_ms_max`, `algorithm` / `other_algorithm_rois`, `depth_buffer_frames`, `requested_ahead_max` / `host_behind_dropped`, `device` (the Script's errors incl. pairing, misses, `config_sequence` and SLC time every 100 frames), `full_depth` (`on`, `every`, `reasons`, `frames`), plus `camera_load`. `full_frame` reports `frames` and the feature gaps. The VIO's 10 s log line adds the paired count, the missing reasons, the gaps and the Leon CSS / MSS CPU.
- **Mock mode** emulates both transports with the real host code: an SLC stand-in computes the same windows, step and thresholds on the synthetic depth and answers through the same messages (tag and config), status lines and byte counts (a request whose depth frame is gone is a reported miss, as on the camera). The unit tests' strict fakes cover the failure paths (dropped, late, out of order, corrupt, wrong ROIs, a lost tag or config), with the link zeroing the config's sequence number as on the hardware and the depth input dropping its oldest frames as the device queue does.

`full_frame` keeps the previous path exactly (Sync node, depth lookup on the Pi), for A/B tests.

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

## Camera link drops (PnP)

On test11 the XLink connection dropped and depthai reconnected by itself ("Reconnection successful"). VIO frames came back, but snapshots answered 404 until VIO was stopped and started. Host → camera queues can stay stuck after depthai's reconnect: in depthai-core v3.3.0, after a failed write `XLinkOutHost` waits for the new connection without checking whether it already arrived, so a reconnect that happened first leaves it waiting for good. That breaks snapshot requests, the depth stream's on/off control and the feature tracker settings.

The VIO process now restarts its own pipeline when the camera restarts under it (`DeviceRestartWatch` in [`app/device_open.py`](app/device_open.py)):

- Signals: depthai's reconnection callback (`setMaxReconnectionAttempts(1, callback)`, the default of one attempt) reporting `RECONNECTED`, or a frame whose sequence number drops by more than 30, or whose device clock goes back more than 2 s.
- The first frame of the restarted camera is not used. The pipeline is left (which closes the device and wakes depthai's stuck threads), the camera is reopened with the usual wait and backoff, and a new pipeline starts with a fresh PnP reference. The data log, the MAVLink output, the telemetry and the [3D pose](#3d-pose-viewer) carry on in the same process.
- The data log gets a `reconnect` event (`count`, `reason`), the VIO log a warning, and `/api/status` → `telemetry` has `reconnects` and `last_reconnect` (`unix_time`, `reason`). A snapshot requested during the drop answers with "the device did not return a frame"; the next one is served by the new pipeline. A streaming depth client is switched on again on the new pipeline by its lease.
- If depthai's reconnect fails, its queues raise and the process exits; the supervisor restarts it as before.

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
| `app/static/features_overlay.js` | The tracked-features overlay (marker styles, grid and letterbox mapping), inlined into both pages |
| `app/static/link_panel.js` | Formatting of the camera link box and the widget's link line, inlined into both pages |
| `app/link_stats.py` | Camera link bandwidth: XLink counter sampling, rates and peak, per-stream estimates |
| `app/cockpit_widget.py` | BlueOS service metadata, the Cockpit extras JSON and the widget's status line |
| `app/depth_stream.py` | Live stream (camera image or depth): on-camera gate / 8-bit conversion Script, variants, blank and stall checks, VIO-side service, `main.py` reader |
| `app/feature_stream.py` | Tracked features for the widget: VIO-side lease-gated publisher, `main.py` reader and SSE framing |
| `app/pose_stream.py` | 3D pose for the pose viewer: integrator, VIO-side lease-gated publisher, `main.py` reader |
| `app/autopilot_attitude.py` | The 3D pose's attitude: ArduPilot `ATTITUDE` read from mavlink2rest (read only), staleness |
| `app/static/pose3d.js`, `pose_widget.html`, `pose_icon.svg`, `vendor/three/` | The 3D pose viewer (main-page panel and Cockpit widget page) and the vendored three.js r156 |
| `app/config_schema.py` | Every setting: schema, defaults, server-side validation, form fields |
| `app/vio_telemetry.py` | PnP live status and plot history: ring buffer and publisher (VIO process), readers (`main.py`) |
| `app/dvl_source.py` | Optional DVL velocity overlay read from mavlink2rest |
| `app/system_stats.py` | Pi CPU load from `/proc/stat` |
| `app/tools/ui_demo_server.py`, `fake_mavlink2rest.py`, `ui_screenshots.py` | Web UI development: replay server, fake mavlink2rest, screenshots |
| `app/mock_mode.py`, `app/mock_vio.py` | Mock mode (`OAKD_MOCK=1`): read-only mock autopilot, fake DVL, network block; the fake PnP VIO with a synthetic camera |
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
| `app/stereo_probe.py` | `POST /api/stereo_probe`: StereoDepth/encoder cases on the camera, and the `mjpeg_loss` mode's camera runs |
| `app/mjpeg_loss.py` | The `mjpeg_loss` analysis: JPEG vs raw disparity at the features with the VIO's depth rules, subpixel loss, quantisation |
| `app/stereo_config.py` | PnP stereo settings: defaults, web form, command line and `StereoDepth` configuration |
| `app/snapshots.py` | Snapshot requests (`main.py`) and serving them from the VIO process (camera JPEGs, raw fallback, encoding status) |
| `app/depth_points.py` | `depth_transport: points`: SLC windows and configs, the camera Scripts, decoding, pairing by sequence number, the receiver and the full depth gate |
| `app/viewers.py` | The viewer registry: counts the stream clients, keeps their lease files with a linger |
| `app/tracking_recovery.py` | Tracking recovery state machine and feature tracker re-initialisation |
| `entrypoint.sh` | Container entrypoint; starts the web server |

## Known limitations

- VIO runs on the Raspberry Pi CPU, which limits it to a few frames per second. The builder includes depthai's `RTABMapVIO` node, which can use feature tracking on the camera itself, but the pipeline does not use it yet.
- `VISION_POSITION_DELTA` reports a fixed confidence of 100% and no rotation delta.
- At startup the VIO process waits up to 30 s (`--device-timeout`) for the OAK-D to be discoverable and to open, retrying with backoff, and then exits with code 2. A PoE OAK-D can take many seconds to come back after the previous pipeline closed; opening it too early has been seen to segfault inside depthai.
- If the VIO process exits without being stopped from the page (error, crash or segfault), `main.py` restarts it after 2 s, doubling up to 30 s; the delay resets after a run of 60 s. `/api/status` reports `restart_count`, `last_exit_code` (negative for a signal, e.g. -11 for SIGSEGV) and `restart_in`.
- Live stream: `mono_features` (the default) is not yet run in the VIO on hardware; the stereo probe showed the rectified images encode directly (test14). `disparity` needs a RAW8 disparity, which the saved settings (speckle after the median) do not give; *Median filter last* is untested on the camera (probe case G). On test12 `manip` gave constant frames and `nv12` / `yuv400p` gave nothing (Script frames have no pool). The fallback, `host_jpeg`, is capped at 25 % of a core; its Pi cost is not measured.
- Camera link: the per-stream sizes are estimates (data length plus a guessed metadata size), and the measured counters leave out XLink's packet headers and the USB / Ethernet framing. Not yet run on the camera; that `getProfilingData()` gives non-zero counters on the Pi's USB link is read from the source, not seen.
- `depth_transport: points`: on test16b (10 fps) `camera` ROIs worked (84.5 % paired, too slow) and `host` ROIs paired nothing, because the configs' sequence numbers read 0 on the camera; the tag that fixes it is not yet run on the camera. Unverified on hardware: the pairing at 15–30 fps, the Script's time per request, that a host config arrives in the Script with `getConfigData` (how it tells a config from a tag; if not, every frame reports a "pairing" error), and the depth AVERAGE gives against the Pi's 25-pixel median (the same on smooth surfaces, closer to the mean of the two sides at an edge). Feature pixels are rounded with floor(x + 0.5) (what the camera Script's `int(x + 0.5)` does) where the `full_frame` lookup uses `rint`, which differ only at exact halves. The frame time is the features' (end of exposure) instead of the depth frame's mid-exposure, the same offset for every frame. *PnP Intrinsics Source* `frame` has no depth frame to read its matrix from, so it falls back as the log says.
- Snapshots: the camera JPEG path (two more `VideoEncoder`s, three with the live stream) is not yet run on hardware; how many encoders the RVC2 runs at once with this pipeline is unverified. If one returns nothing, that stream falls back to raw frames, flagged.
- 3D pose: the position is dead reckoning from the deltas and drifts, and motion during reference resets and recoveries is missing. The attitude comes from ArduPilot's `ATTITUDE`; without it, it is gyro-integrated and drifts (about 0.65°/s on test12).
- Web UI: the plots lag by up to about a second (history is published once a second); the plots and the trajectory start over when VIO restarts; the DVL overlay has no history from before the page was opened. Live telemetry, the feature overlay and the chip temperature are PnP only. The web UI was developed against the demo server only; on a vehicle the DVL discovery, `getChipTemperature()` and the Pi CPU cost of the publisher are unverified.
