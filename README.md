# BlueOS OAK-D Visual Odometry

A [BlueOS](https://github.com/bluerobotics/BlueOS) extension that turns a Luxonis OAK-D (tested on an OAK-D-W) into a visual odometry sensor for an ArduPilot vehicle (ArduSub, EKF3).

The camera does the image work: stereo depth (`StereoDepth`), feature tracking (`FeatureTracker`) and the depth at each tracked feature (`SpatialLocationCalculator`). The Raspberry Pi gets only the features, their depth and the IMU, solves the camera motion between frames with PnP RANSAC, seeded and checked by the gyro, and sends it to the autopilot as `VISION_POSITION_DELTA` through BlueOS's mavlink2rest. Images cross the camera link only while someone looks at them, JPEG-encoded on the camera.

Production setup (the defaults): 640×400 at 30 fps, subpixel and speckle filter on, depth at the features computed on the camera (`depth_transport` `points`) with the windows listed by the Pi. Measured on the vehicle (test17), the camera link carries about 9.3 Mbps down and 2.2 Mbps up at 30 fps. Sending every depth frame to the Pi instead (`full_frame`, the fallback) is 127.5 Mbps, more than a PoE camera's 100 Mbps Ethernet.

## What it does on startup

1. Starts a configuration web page on port **8095** (8080 is used by mavlink-server on BlueOS 1.4.x).
2. Starts the VIO process with the saved settings and begins sending MAVLink (unless [log-only](#log-only-mode-and-data-logs) is on).
3. Checks the autopilot's parameters against the recommended set (below), read-only, and shows the result on the page.

It never changes autopilot parameters by itself.

## Autopilot parameters

`SUGGESTED_OPTICAL_HOLD_PARAMS` in [`app/autopilot_params.py`](app/autopilot_params.py) is the one recommended set for position hold on `VISION_POSITION_DELTA` (ArduSub, EKF3): name, value, whether it needs a reboot, and why, with references to the ArduPilot source.

| Parameter | Recommended | Reboot | Why |
|---|---|---|---|
| `AHRS_EKF_TYPE` | 3 | yes | EKF3 (ArduPilot's default). Only EKF3 takes `VISION_POSITION_DELTA`. |
| `EK3_ENABLE` | 1 | yes | EKF3 enabled (default). |
| `VISO_TYPE` | 1 | yes | MAVLink visual odometry. The other `VISO_*` parameters only appear after this is set and the autopilot rebooted. |
| `VISO_POS_X/Y/Z` | 0.1655, −0.0069, 0.180 | no | Camera relative to the IMU, m FRD, from the CAD. EKF3 applies this lever arm itself (see [Lever arm](#lever-arm-viso_pos)). |
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

The page checks the autopilot every 30 s, on demand (the [ArduPilot panel](#ardupilot-panel)'s *Read*) and right after a write. Checks only send `PARAM_REQUEST_READ`. While mavlink2rest or the autopilot does not answer, the check backs off up to 5 minutes. `/api/status` carries the cached result as `autopilot_params`: `checked_at`, `reachable`, `armed`, `ok`, `mismatches` (`name`, `current`, `recommended`, `needs_reboot`), `unreadable`, `pending_reboot` and `error`. Float32 rounding is not a difference.

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

## MAVLink output

Messages are sent with component ID 195 (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`) over mavlink2rest's websocket, `ws://<vehicle_ip>/mavlink2rest/ws/mavlink`.

- `VISION_POSITION_DELTA`, one per accepted frame: `position_delta` (m) and `angle_delta` (rad), both body FRD over the interval since the reference frame; `time_delta_usec`, that interval from the camera's timestamps, rounded to whole µs; `confidence` 0–100 (`100 · inlier_ratio · min(1, inliers/60)`); `time_usec`, host wall time when sent.
- `NAMED_VALUE_FLOAT` (names are at most 10 characters, sent as NUL-padded char lists; `time_boot_ms` counts from the VIO process start), at the *Quality Rate*:
  - `VIO_TRACK`: 1 while tracking, 0 when lost (also sent at once on a change)
  - `VIO_FEAT`: features tracked on the camera
  - `VIO_INLIER`: PnP RANSAC inliers of the last frame
  - `VIO_CONF`: confidence of the last frame, 0 to 100
  - `VIO_PROCMS`: host processing time of the last frame, in milliseconds

How ArduPilot (EKF3) uses `VISION_POSITION_DELTA`, from the source:

- Velocity = `position_delta` / `time_delta_usec`, fused as body-frame velocity, with the `VISO_POS_*` lever arm applied by the EKF (see [Lever arm](#lever-arm-viso_pos)). `VISO_ORIENT` rotates both vectors, so it must stay 0 (Forward): the extension already applies *Camera Angle*.
- `time_usec` is ignored. The message is stamped on arrival minus `VISO_DELAY_MS` (default 10 ms). A delta is the average over its interval, so its true delay is about half a frame interval (17 ms at 30 fps) plus the pipeline latency. `VISO_DELAY_MS` (0–250) is not tuned yet.
- `confidence` sets the velocity noise linearly: `EK3_VIS_VERR_MIN + (EK3_VIS_VERR_MAX − EK3_VIS_VERR_MIN)·(1 − confidence/100)`. `VISO_QUAL_MIN` does **not** apply to `VISION_POSITION_DELTA`.
- `angle_delta` is stored but not fused. The EKF uses its own gyro, also for the lever-arm term.
- Messages less than 50 ms apart are dropped. To start using body odometry, the EKF also wants one within the last 200 ms.

The output path (log-only off) is tested end to end against a fake mavlink2rest websocket: every accepted step is sent once with exactly the logged body-FRD deltas, `time_delta_usec` 100000 for 0.1 s, and header system 1, component 195; a spinning down-looking camera gives +z `angle_delta` (nose right).

## How it works

**On the camera:**

- `CAM_B` / `CAM_C` feed `StereoDepth`, configured by `stereo_config.apply_stereo_config`: the preset (ROBOTICS) first, then every [stereo setting](#stereo-depth). On RVC2, ROBOTICS is DENSITY plus decimation 2, the spatial filter, speckle range 200, confidence 15 and LR threshold 10 (`StereoDepth.cpp`), which is why decimation and the spatial filter are set explicitly after it. Extended disparity is forced off (incompatible with subpixel); the median filter is turned off, with a warning, if subpixel bits ≠ 3.
- Depth is aligned to `RECTIFIED_LEFT`, so it shares a pixel grid with `stereo.rectifiedLeft`. With decimation the feature pixels are scaled to the depth grid for the lookup only.
- `rectifiedLeft` feeds the `FeatureTracker`: Harris corners (`pnp_vio.py --corner-detector shi-tomasi` for the other one), *PnP Features* target (300), Lucas-Kanade optical flow with persistent ids, feature maintainer on.
- `points` (default): a pairing Script and a `SpatialLocationCalculator` compute the depth in a 5×5 window at each feature and send only those numbers ([Depth at the features](#depth-at-the-features-depth_transport)). `full_frame`: a `Sync` node pairs each features message with its whole depth frame.
- The IMU sends raw gyro and accelerometer at 200 Hz on its own queue (the accelerometer is not used; metric scale comes from the stereo depth).
- Gate Scripts hold the latest snapshot frames and the live stream, and forward them only on request ([Images on demand](#images-on-demand-and-viewers)).

**On the Pi, for each frame:**

1. Drain the IMU queue into a gyro buffer.
2. Take the depth at each feature (`points`: the camera's 25-pixel average; `full_frame`: the median of the valid pixels in a 5×5 window). 0 and out-of-range values are rejected, and at least 30 % of the window must be valid. The depth is multiplied by *Refractive Index* × *Depth Scale*, and the features are back-projected with the rectified-left camera matrix, fx and fy × the refractive index.
3. Match the features to the reference frame by id: the last good frame, usually the previous one. The reference supplies the 3D points, the current frame the 2D points.
4. Integrate the gyro between the two frames' timestamps for the rotation prior. The buffer keeps 2 s before the older frame (at most 10000 samples); a hole over 0.1 s gives no prior.
5. `cv2.solvePnPRansac`, seeded with the gyro rotation (EPnP minimal sets, then LM refine on the inliers).
6. Reject the frame if it has fewer than *Min Inliers* (15) inliers or an inlier ratio below 0.3, if its rotation differs from the gyro by more than 5° (`--gyro-gate`), or if it moved more than 0.5 m per frame interval.
7. On success, convert to body FRD and send `VISION_POSITION_DELTA`; the frame becomes the reference.
8. On failure, send nothing and keep the reference ([Failed frames](#failed-frames)). After 20 failures in a row, [re-initialise the tracker](#tracking-recovery).

**Conventions:**

- Camera frame: OpenCV optical frame of the rectified left camera (x right, y down, z forward). Units: metres, radians, seconds; device depth is `uint16` mm.
- PnP maps `X1 = R_10·X0 + t_10` (0 = reference, 1 = current). `angle_delta = rotvec(R_01)` with `R_01 = R_10ᵀ`; `position_delta = −t_10`, the camera displacement expressed in the current frame, as MAVLink defines `VISION_POSITION_DELTA`.
- Body frame: optical → FLU (`x=z, y=−x, z=−y`), then the *Camera Angle* pitch, then FLU → FRD; the rotation vector goes through the same transform. A forward camera maps optical forward/right/down to FRD x/y/z; at 90° optical forward is body down. Both are unit tested.
- Timestamps are the camera's (device clock). `full_frame` uses the depth frame's mid-exposure time; `points` the features' time, the same offset on every frame. Gyro samples use their device timestamps on the same clock.
- Gyro → camera: the rotation part of `calib.getImuToCameraExtrinsics(CAM_B, True)`, zero-order hold between samples. The gyro bias is not estimated; without the extrinsics the prior and the gate are off.
- Dropped frames: host queues are non-blocking. Ids persist on the camera, so the next delta covers a longer interval.

## Configuration page

Open `http://<vehicle>:8095` (or the extension's entry in the BlueOS menu). Saving restarts VIO with the new values. Settings are stored in `/usr/blueos/extensions/oakd-vio/config.json` on the vehicle, so they survive restarts and updates. A config saved by an older version loads: settings that no longer exist are ignored and a choice that no longer exists gets its default.

The page is one self-contained file (`app/static/ui.html`, `ui.css` and `ui.js`, inlined by `main.py`): no CDN, no build step, so it works offline on a boat. Panels:

- **Live status**: camera connected, VIO running, fps, inliers, frames ok (last 10 s), processing time, features / valid depth, matches, refractive index, depth scale, the effective camera matrix, reference resets / gap frames, recoveries, Pi CPU and the camera chip temperature; failures by reason over the last 5 minutes; *Depth for PnP*; the [camera link](#camera-link-bandwidth); the output mode, messages sent, rate and last error, a *Log-only* toggle and *Reset VIO*.
- **Live views**: [snapshots](#snapshots) of `rect_left` with the tracked features and the colorized depth, on demand or about once a second; the [depth probe](#depth-probe) and the [focal check](#focal-check).
- **[Live stream](#live-stream)**: the camera image with the tracked features (or the depth), MJPEG.
- **Trajectory** (integrated from the deltas and the gyro yaw; it drifts), a **velocity** panel in the layout of the Water Linked DVL A50 widgets, **vx / vy over time** with an optional [DVL overlay](#dvl-overlay), **inliers over time**, and the **[3D pose](#3d-pose-viewer)**.
- **Settings**, generated from `GET /api/config/schema`, checked in the browser and again on the server; *Apply* restarts VIO. The expert settings are in a collapsed *Advanced* group (it opens by itself when one of them has an error).
- **ArduPilot parameters** and **Data logs**.

| Setting | Default | Notes |
|---|---|---|
| *PnP (option D)* | | |
| PnP Refractive Index (water) | 1.333 | The water in front of the flat port (1–2). 1.333 fresh water, about 1.34 sea water. **Set 1.0 for tests in air.** See [Water](#water-refractive-index). |
| PnP Depth Scale | 1.0 | Extra trim on the camera depth on top of the refractive index (0.2–5). Translation scales with it. Use it for a residual measured with the [depth probe](#depth-probe). |
| PnP Min Inliers | 15 | RANSAC inliers a frame needs to be accepted and sent (6–500). The inliers plot draws it. |
| PnP Features | 300 | Target number of features tracked on the camera. |
| Snapshots | on | Allow [snapshots](#snapshots). Nothing is sent until one is requested. |
| Live depth stream | on | Allow the [live stream](#live-stream). Nothing is sent until a client watches. |
| Depth Stream FPS | 5 | Target stream rate (0.5–15; every n-th camera frame). |
| Depth Stream JPEG Quality | 70 | The camera's MJPEG quality (10–100). |
| Depth Stream Encoding | auto | What the live stream shows: `auto` = `mono_features`, falling back to `host_jpeg` if the camera sends no frames or blank ones; `mono_features` = the rectified left camera image the features are tracked on, encoded on the camera; `host_jpeg` = the depth, encoded on the Pi (Pi CPU, capped at 25 % of a core). |
| *Camera* | | |
| FPS | 30 | Camera frame rate (1–60). 30 is the measured production rate; above about 31 fps StereoDepth with subpixel is the limit. |
| Camera Angle (°) | 0 | Downward pitch of the camera. 0 = facing forward, 90 = facing down. |
| Width × Height | 640 × 400 | Rectified-left image size. The camera encoders need a width that is a multiple of 32. |
| *Stereo depth (PnP)* | | See [Stereo depth](#stereo-depth). |
| *MAVLink output* | | |
| Vehicle IP | 127.0.0.1 | Where mavlink2rest is reachable. The extension uses host networking, so the default works on the vehicle itself. |
| Log only | off | No MAVLink output and no autopilot parameter changes. See [Log-only mode](#log-only-mode-and-data-logs). |
| DVL overlay source | auto | *auto*: plot a DVL's velocity dashed over the VIO velocity, if mavlink2rest has one (see [DVL overlay](#dvl-overlay)); *off*: never poll. |
| *Advanced* | | |
| PnP Max Reference Age (s) | 1.0 | How long [failed frames](#failed-frames) keep the last good frame as the reference; 0 removes the limit. |
| PnP Max Features | 0 | Hard cap on the features the camera detects per frame (FeatureTracker `numMaxFeatures`); 0 = depthai's automatic maximum. Each feature costs about 64 B per frame on the link and one window in `points` mode. |
| PnP Recovery After (frames) | 20 | Consecutive failed frames before [tracking recovery](#tracking-recovery); 0 disables it. |
| Lever arm diagnostics (log only) | off | Also log each delta corrected to the IMU. It never changes what is sent; the autopilot applies `VISO_POS_X/Y/Z` itself. See [Lever arm](#lever-arm-viso_pos). |
| Lever Arm x,y,z (m) | 0.1655,-0.0069,0.18 | Camera relative to the IMU in body FRD, from the CAD. Only used by the diagnostics. |
| PnP Intrinsics Source | rectified | Where the rectified-left camera matrix comes from. See [Camera matrix](#camera-matrix). |
| PnP Intrinsics Override | empty | `fx,fy,cx,cy` in pixels on the width × height grid. Wins over the source. |
| PnP Depth Transport | points | `points`: the camera computes the depth at the features and sends only those; `full_frame`: every depth frame goes to the Pi (the fallback; too much for PoE at 30 fps). See [Depth at the features](#depth-at-the-features-depth_transport). |
| Depth stream camera debug log | off | While the stream runs, raise the camera's log level to DEBUG and show the stream nodes' log lines (see [Debug mode](#debug-mode)). |
| Quality Rate (Hz) | 1 | Rate of the `NAMED_VALUE_FLOAT` messages. |
| Fine stereo settings | | LR check threshold, min / max brightness, P2, edge fill color, alpha, distortion correction, decimation, subpixel bits: see [Stereo depth](#stereo-depth). |

Each setting is also a `pnp_vio.py` flag (`pnp_vio.py --help`).

### Stereo depth

The *Stereo depth (PnP)* group configures the on-camera `StereoDepth` node. The preset is applied first, then every setting below overrides it. The defaults aim for sparse but trustworthy depth at the tracked features, not a dense depth map: no hole filling, and depth stays on the full-resolution rectified-left pixel grid the feature tracker runs on. Empty optional fields mean "keep the preset or device value" (shown as *auto*). The settings marked *A* are under *Advanced*.

| Setting | Default | Notes |
|---|---|---|
| Stereo preset | ROBOTICS | `setDefaultProfilePreset`; NONE skips it. On RVC2, ROBOTICS also turns on decimation 2 and the spatial filter, which the defaults turn off again. |
| Left-right check / threshold (*A*) | on / 5 | Max disparity difference (px) between the left and right matches. |
| Confidence threshold | 50 | 0–255; pixels with a worse confidence score are invalidated, so lower is stricter. |
| Min (*A*) / max (*A*) brightness | auto / 255 | Pixels at or below min, or at or above max, are invalidated. Auto min is 1 when the edge fill color is 0, which drops the black rectification border, otherwise 0. |
| Speckle filter / range | on / 50 | |
| Min / max depth (mm) | 300 / 4000 | Threshold filter. Depth outside the range is set to 0. |
| Spatial filter, temporal filter | off, off | Spatial fills holes by smoothing across edges; temporal lags behind motion. |
| Decimation (*A*) | 1 | 2–4 make depth coarser than the feature grid. |
| Subpixel / bits (*A*) | on / 3 | The median filter needs 3 bits; with 4 or 5 it is turned off (with a warning). |
| Median filter | 7x7 | off, 3x3, 5x5 or 7x7. |
| Distortion correction (*A*) | on | |
| Alpha scaling (*A*) | 0 | 0 crops the rectified image to valid pixels only. Empty keeps the device default. |
| Edge fill color (*A*) | 0 | Fill for the area outside the rectified image; -1 replicates edge pixels. |
| Horizontal P2 penalty (*A*) | empty | Cost aggregation P2; empty keeps the preset value. |

*Distortion correction* and *Alpha scaling* decide the camera matrix of the rectified image; see [Camera matrix](#camera-matrix). The requested settings, the values read back from the node (`effective`) and any warnings are logged at startup and written to the data log as a `stereo_config` event.

Depth is not gated with the stereo confidence map: on the camera the `confidenceMap` output gets no left-right check, alignment or post-processing, so it doesn't match the depth pixels, and the confidence threshold above already applies the same score on the camera.

### Camera matrix

PnP back-projects each feature with the camera matrix (K) of the rectified-left image the features and depth are on. With distortion correction on and an alpha set (the defaults: on, 0), the camera rectifies onto OpenCV's `getOptimalNewCameraMatrix(K, D[:8], size, alpha)` of the CAM_B calibration, which has fx ≠ fy. For the OAK-D-W-PoE at 640×400 and alpha 0:

| | fx | fy | cx | cy |
|---|---|---|---|---|
| Raw CAM_B | 285.11 | 285.05 | 323.22 | 199.08 |
| Rectified, alpha 0, D[:8] (used) | 163.10 | 237.09 | 329.37 | 198.98 |
| Rectified, alpha 0, all 14 coefficients | 163.12 | 237.09 | 328.65 | 197.98 |
| CAM_C rectified, alpha 0 | 161.9 | 236.0 | 326.1 | 196.4 |

The depth frame's own matrix turned out to be the raw one; with it, lateral translation came out at 0.57× and vertical at 0.83× the true value. The device depth confirms the rectified matrix: on a stage-1 snapshot f = Z·d/B is 167.7 px (39 SIFT matches; a dense estimate gives 166.9), 3 % from 163.1 and 41 % from 285.1. The baseline is 7.496 cm calibrated, 7.5 cm by design.

Where this comes from in depthai-core (`4d4a8c159`; the RVC2 stereo firmware is closed, so these are its documented contracts and depthai-core's own model of the output):

| Where | What |
|---|---|
| `StereoDepthProperties.hpp:132-142`, `StereoDepth.hpp:316-325` | With distortion correction, rectification is a mesh generated on the device, "same as opencv's initUndistortRectifyMap"; "Only the first 8 distortion coefficients are used". |
| `StereoDepth.cpp:161-163` | `enableDistortionCorrection(x)` is `useHomographyRectification(!x)`. |
| `StereoDepthProperties.hpp:181-189`, `StereoDepth.hpp:371-377` | `alphaScaling`: "See getOptimalNewCameraMatrix from opencv". |
| `rtabmap/CalibrationHandler.cpp:14-30` | `getRTABMapCameraModel`, depthai-core's model of the rectified stereo output: the camera's K, D[:8], and `getOptimalNewCameraMatrix(K, D, size, alpha)` when alpha > −1. It does not apply R1. |
| `StereoDepthProperties.hpp:126-130` | `focalLengthFromCalibration = true`: the focal length for disparity → depth comes from calibration on the device (the rectified fx, as measured). |
| `CalibrationHandler.hpp:307-309` | `getBaselineDistance` defaults to the design value; the calibrated one needs `useSpecTranslation = False`. |

Not verifiable from the sources: whether the firmware folds R1 into the new matrix, its OpenCV version, or a different left/right reference. Any of these could move K by a few percent; the [focal check](#focal-check) checks fx on the camera.

*PnP Intrinsics Source*:

| Source | Camera matrix |
|---|---|
| `rectified` (default) | Computed on the Pi from the CAM_B calibration, distortion and the configured alpha. The depth frame's matrix is only logged, with whether it equals the raw or the rectified K. If alpha is empty or distortion correction is off, the device's matrix is unknown and PnP uses the depth frame's matrix with a warning. |
| `frame` | The depth frame's matrix (`full_frame` only; `points` has no depth frame and falls back). If it equals the raw CAM_B calibration while distortion correction and alpha are set, PnP ignores it with a warning and uses the rectified K. |
| `cam_b`, `cam_c` | The raw calibration of that camera, for comparison. |

*PnP Intrinsics Override* (`fx,fy,cx,cy`) wins over all of them. Every fallback is logged as a warning, in the container log and as a data log `warning` event. The chosen matrix, its source, the raw and rectified matrices, alpha, distortion correction and the baseline go to the container log, the `intrinsics` event and `/api/status` → `camera`. `back_project` uses fx and fy separately; nothing assumes a single focal length.

### Water (refractive index)

The calibration is done in air, but underwater the camera looks through a flat port into water. To a first (paraxial) approximation the in-air camera images the water scene as if its focal lengths were n·fx and n·fy, and the device's stereo depth, Z = f·B/d with the in-air f, comes out at Z_true / n. PnP corrects both:

- K: fx and fy are multiplied by n; cx and cy are unchanged.
- Depth: multiplied by n × *PnP Depth Scale*. The device's min/max depth still apply to the uncorrected depth.

With K and depth both scaled by n, the lateral back-projection X = (u − cx)·Z/fx does not change: n scales the translation **along the optical axis** only (for a camera looking down, the vertical component). That is tested on a synthetic flat-port scene. Ray bending near the image edges beyond the paraxial model is not modelled. The in-air matrix, `effective_intrinsics` (with n), `refractive_index`, `depth_scale` and `total_depth_scale` are in the `intrinsics` event and `/api/status` → `camera`. The focal check keeps the in-air matrix, because the device depth is computed with the in-air focal length. `main.py` clamps a hand-edited out-of-range value into 1–2.

### Lever arm (VISO_POS)

The camera is about 0.17 m forward and 0.18 m below the IMU, so when the vehicle rotates its delta includes a displacement r − Rᵀr the IMU did not make. **ArduPilot corrects this itself for `VISION_POSITION_DELTA`**, so the extension sends the camera's own delta (rotated into body FRD) and must not correct it a second time:

- `AP_VisualOdom_Backend::handle_vision_position_delta_msg` (`AP_VisualOdom_Backend.cpp`, ~l.62–69) passes `VISO_POS_X/Y/Z` to `AP::ahrs().writeBodyFrameOdom(...)`.
- EKF3 (`AP_NavEKF3_PosVelFusion.cpp`, ~l.1493–1498) predicts the sensor velocity as the body velocity plus `(delAng / delAngDT) × (VISO_POS − INS_POS)`.

Set `VISO_POS_X/Y/Z` on the autopilot (see [Autopilot parameters](#autopilot-parameters)). *Lever arm diagnostics* only adds data log step fields, never sent: `lever_delta_position` (the IMU delta, body FRD, m), `lever_delta_correction` (`r − Rᵀr`) and `lever_delta_rotation` (`gyro` or `pnp`, the rotation used).

## HTTP API

| Method | Path | Action |
|---|---|---|
| GET | `/api/config` | Current settings |
| GET | `/api/config/schema` | Every setting: `name`, `type`, `min`, `max`, `default`, `help`, `label`, `group`, `choices`, `nullable` |
| POST | `/api/config` | Validate and save settings (JSON body, same keys; missing keys get defaults) and restart VIO. 422 lists every invalid field: `detail: [{field, msg, loc, type}]`, e.g. `FPS (fps): must be between 1 and 60 (got 0)`; nothing is saved |
| POST | `/api/log_only` | `{"log_only": true\|false}`: save the setting and restart VIO; 400 for anything else |
| GET | `/api/status` | The VIO process (running, restarts), `log_only` / `log_only_forced_by_env`, `camera` (the matrix in use, `null` until the first frame), `telemetry` (live status, `stale: true` when VIO stopped; `telemetry.link` is the [camera link](#camera-link-bandwidth)), `camera_connected`, `chip_temperature_c`, `cpu_percent`, `uptime_s`, the last known `vehicle` armed state, the cached [parameter check](#the-check-and-the-configure-button) `autopilot_params` (this GET never contacts the autopilot), `depth_transport`, `depth_stream`, `pose`, `viewers`, `image_streams` and `mock` |
| GET | `/api/history?seconds=60&max_points=600&tail=0` | Plot history, oldest first, `t` in seconds before `server_time`: `vx`, `vy`, `vz` (m/s, body FRD, `null` for rejected frames), integrated `x`, `y`, `yaw`, `inliers`, `ok`. At most `max_points` (≤ 600) over `seconds` (≤ 600); `tail` (≤ 10 s) adds the newest frames at full rate. `dvl` holds the [DVL overlay](#dvl-overlay). Weak `ETag`; `If-None-Match` gives 304 |
| POST | `/api/autopilot/params/read` | The suggested parameters as the vehicle has them, and the HEARTBEAT armed state. Read only (POST because it sends MAVLink requests) |
| POST | `/api/autopilot/params` | `{"confirm": true, "params": {"NAME": value}}`: guarded write with read-back, refused (409) unless a fresh HEARTBEAT says disarmed. See [ArduPilot panel](#ardupilot-panel) |
| GET | `/api/logs`, `/api/logs/{name}` | Data logs (name, size, mtime, newest first); download one |
| POST | `/api/start`, `/api/stop`, `/api/reset` | Start or stop VIO; restore defaults and restart |
| GET | `/api/snapshot?stream=<stream>` | One current camera frame. See [Snapshots](#snapshots) |
| POST | `/api/depth_probe?x=&y=&w=&h=&frames=` | Depth statistics in an ROI over the next frames. See [Depth probe](#depth-probe) |
| POST | `/api/focal_check?tolerance=` | Checks fx against the focal length the device depth implies. See [Focal check](#focal-check) |
| GET | `/api/stream/depth` | The [live stream](#live-stream) as `multipart/x-mixed-replace` MJPEG; 503 when VIO is stopped or the stream is off. `?frames=N` ends after N frames |
| GET | `/api/stream/features`, `/api/features` | Server-sent events for the [Cockpit widget](#cockpit-widget): `features` (about 5 per second) and `status` (once a second); the latest `features` once (404 until published) |
| GET | `/api/stream/pose`, `/api/pose` | Server-sent events for the [3D pose viewer](#3d-pose-viewer): `pose` (at most 10 per second), `trail`, `status`; the latest `pose` once (404 until published) |
| GET | `/widget/depth`, `/widget/pose` | The two Cockpit widget pages |
| GET | `/register_service`, `/cockpit_extras.json` | BlueOS service metadata and the Cockpit extras that declare the two widgets |

`/openapi.json` (and `/docs`) lists only the GET routes that answer at once with a bounded body and start nothing: `/`, `/widget/depth`, `/widget/pose`, `/register_service`, `/cockpit_extras.json`, `/api/config`, `/api/config/schema`, `/api/status`, `/api/logs` and `/api/logs/{name}`. BlueOS's helper GETs every path listed there every few seconds and reads each body to its end, so the streams and the routes that wait for the camera or start work are left out of the schema, and everything that runs something on the camera or the autopilot is a POST. `app/tests/test_openapi_scan.py` runs that scan against the app.

## Images on demand and viewers

No image crosses the camera link unless someone is looking at it, and the camera encodes what it sends. The gates, on the camera:

| What | Gate | Sent while |
|---|---|---|
| Live stream `mono_features` | the stream Script blocks on its control input | a `video` viewer (or its 4 s linger) |
| Live stream `host_jpeg` (`points`) | the full depth gate Script (every n-th depth frame) | a `video` viewer, and the stream runs on the Pi |
| Full depth frames (`points`) | the full depth gate Script | the depth probe runs, or the above |
| Snapshots (`rect_left`, `rect_right`, `rgb`, `depth`) | the snapshot Script, one frame per request | a snapshot is requested |
| Features | not gated (the PnP needs them anyway) | always |

In `full_frame` the depth frame crosses on every frame for the PnP, so `host_jpeg` adds nothing on the link there.

`main.py` counts the clients of `/api/stream/depth` (`video`), `/api/stream/features` (`features`) and `/api/stream/pose` (`pose`) in a viewer registry ([`app/viewers.py`](app/viewers.py)); `?client=page|widget` says who it is, anything else counts as `other`. While a kind has a viewer the registry renews its lease file every second ([`app/leases.py`](app/leases.py); features and pose also renew the status lease); 4 s after the last viewer leaves (so a page reload does not stop and restart the camera stream) it removes the lease, unless a one-shot request (`/api/features`, `/api/pose`) renewed it since. The VIO process polls each lease every 0.5 s.

`/api/status` → `viewers`: `total`, `by_kind`, `by_client`, `video`, `video_idle`, `lingering_s`, `linger_s`, `opened_total` and `text` ("1 watching video, 1 other", "video stopping (no viewer)", "video idle"). `/api/status` → `image_streams.streams` lists every image stream that can cross the link (the live stream, then each snapshot stream): `codec`, `quality`, `encoded_on` (`camera` or `pi`), `over_link` (`jpeg` or `raw`), `viewed`, `mbps`, `last_bytes`, `measurement` (the depth snapshot: raw 16-bit data, not a fallback), `reason` and `fallback`; `image_streams.fallbacks` collects the fallback texts, which the widget shows.

### Snapshots

`GET /api/snapshot?stream=rect_left|rect_right|depth|rgb` returns a fresh frame from the running VIO, without stopping it. The page's *Live views* use it.

- `rect_left`, `rect_right`: the rectified mono images the stereo matcher sees (the feature tracker runs on `rect_left`), JPEG-encoded on the camera (quality 95), served as the camera sent them.
- `depth`: depth aligned to `rect_left`, colorized from red (near) to blue (far) over the min–max depth range, black where there is none. `&raw=1` returns the 16-bit depth in millimetres. The depth crosses the link raw (one 512 kB frame per request) and is PNG-encoded on the Pi: 16-bit millimetres are measurement data a JPEG cannot carry.
- `rgb`: the color camera (CAM_A) at 640×400 and 5 fps, JPEG-encoded on the camera; 404 on devices without it.

If the camera's encoder returns nothing for a stream (no JPEG within 2 s: that request answers 404 "… the next rect_left snapshot is sent raw"), or the width is not a multiple of 32, that stream comes raw and is PNG-encoded on the Pi for the rest of the run, flagged as "fallback: raw frames over the link, encoded on the Pi" in `X-Snapshot-Fallback`, `/api/status` → `image_streams`, the page and the widget.

Responses carry `Cache-Control: no-store` and `X-Snapshot-Unix-Time`, `X-Snapshot-Device-Timestamp`, `X-Snapshot-Sequence` and `X-Snapshot-Encoded-On` headers. 504 if no frame arrives within 3 s, 404 if the stream is unavailable or snapshots are off, 422 for an unknown stream.

- `&overlay=1` (rect_left only) draws the features of the last frame the VIO processed: inliers green, outliers red, unmatched or without depth grey, each with a line from its previous position (`X-Overlay`, `X-Overlay-Inliers`, …). It is built on the snapshot writer thread and drawn by the web server, so it costs nothing until asked.
- `&crosshair=1&x=&y=&w=&h=` draws the [depth probe](#depth-probe)'s ROI (not with `raw=1` or `rgb`).

How it works: on the camera a `Script` holds only the latest frame of each stream in size-1 non-blocking queues and forwards one when the Pi asks (the request's sequence number is the stream's index, plus 4 for a raw frame): `rect_left` / `rect_right` to an MJPEG `VideoEncoder` (StereoDepth's pooled RAW8 frames), `rgb` (NV12) to another, `depth` and raw requests to its `snapshot` output. Each stereo output's frame pool grows from 3 to 5 frames to cover the frame the gate holds and the one in the encoder (4 when the mono snapshots are raw; more with the live stream). `main.py` and the VIO process share `/app/data/snapshots/`: the endpoint writes a request file, the VIO loop claims it (at most every 0.2 s, one request per second to the camera, one in flight), a writer thread saves `<stream>.jpg` (or PNG-encodes a raw frame) and `<stream>.json` last; `encoding.json` has each stream's codec and fallback. Host cost on a desktop with a 300-feature frame: 0.9 ms per frame without snapshots, 1.0 ms with a request on every frame.

### Depth probe

`POST /api/depth_probe` measures the device depth in a small region while VIO keeps running. Use it to compare the camera's depth with a tape measure; in water compare the tape with `scaled_m` (which includes the refractive index) and set *PnP Depth Scale* for any residual. The page's *Depth probe* has the inputs; click the left image to aim, and tick *probe ROI* to draw the crosshair.

```
curl -X POST 'http://<vehicle>:8095/api/depth_probe'                                # image centre, 41×41 px, 10 frames
curl -X POST 'http://<vehicle>:8095/api/depth_probe?x=320&y=200&w=41&h=41&frames=10'
```

- `x`, `y`: ROI centre in pixels on the rectified-left grid (default the image centre). `w`, `h`: ROI size, default 41, at most 200. `frames`: default 10, at most 50. Larger values are clamped and listed in `clamped`.
- The response: `valid_fraction`, `n_valid_px`; `median_mm`, `p10_mm`, `p90_mm`, `mean_mm` over the valid pixels of all frames; `median_of_frame_medians_mm` with the per-frame medians and valid fractions; `raw_m` (Z, the device depth along the optical axis) and `scaled_m` (× refractive index × depth scale, what PnP uses); `range_m` (straight-line distance to the ROI centre) and `point_m` (X, Y, Z), raw and scaled; `device_timestamps`, `sequences`, `summary` (e.g. `median 1.234 m (scaled 1.645 m, n 1.333), valid 87%`) and `note`.
- The depth is measured from the left camera's optical centre, a few mm behind the front of the lens, plus the port.
- Errors: 400 for bad parameters, 503 if VIO is stopped, 504 if no result arrives within frames/fps + 3 s.

In `points` mode the VIO asks the camera's full depth gate for frames while a probe runs. The ROI copy costs about 0.02 ms per frame (200×200, desktop); the statistics run on a separate thread. One probe at a time.

### Focal check

`POST /api/focal_check` (the *Focal check* button) checks the camera matrix's fx against the focal length the device used to turn disparity into depth: it finds feature matches between `rect_left` and `rect_right` on the same row (SIFT on contrast-enhanced images, |dy| < 2 px, disparity > 0.5 px), reads the device depth at each (median of 3×3) and computes f = Z·d/B.

```
curl -X POST 'http://<vehicle>:8095/api/focal_check'               # tolerance 15%
curl -X POST 'http://<vehicle>:8095/api/focal_check?tolerance=0.1'
```

The response has `fx_K`, `f_from_depth` (median), `f_iqr`, `ratio`, `n_matches`, `ok` (|ratio − 1| ≤ tolerance with at least 10 matches), `message`, `baseline_m` and the three snapshots' `device_timestamps`. With the rectified K the ratio is about 1.0; with the raw K about 0.59, and the message says so. The three frames are snapshots about 1 s apart, so keep the camera and the scene still; aim at a textured scene 0.5–3 m away. It checks fx only. The same check runs offline: `python app/tools/check_rectified_focal.py rect_left.png rect_right.png depth_raw.png 7.496 --fx 163.1` (baseline in cm; `depth_raw.png` is the `raw=1` depth snapshot).

### Live stream

The live panel shows, at about 5 fps while its *stream* box is ticked, the camera image the VIO tracks its features on (*Live camera*, `mono_features`, the default) or the stereo depth (*Live depth*, `host_jpeg`). The *features* box draws the tracker's features on it, in the [widget's styles](#cockpit-widget). For depth, the *colorize* box colours it in the browser (near red, far blue, invalid black) with a legend. The panel also shows the depth at the image centre (from the VIO's own depth, not the JPEG), the frame rate, the bandwidth and the source. It is the `<img>` of `GET /api/stream/depth`, so it also opens in a browser tab or VLC.

- `mono_features`: on the camera a `Script` takes `StereoDepth.rectifiedLeft` (RAW8, the image the `FeatureTracker` gets) from a size-1 non-blocking queue and, only while the Pi switched it on, forwards every n-th frame (n = fps / stream fps; 6 at 30 fps) unchanged into a `VideoEncoder` (MJPEG, quality 70, non-blocking input, so a busy encoder drops stream frames, never VIO frames). The VIO publishes, with each image, the features of the frame it came from (matched by sequence number; `synced: true`).
- `host_jpeg`: nothing extra on the camera; the VIO process maps its depth frame to 8-bit gray (linear in depth, 255 at *Min depth*, 16 at *Max depth*, 0 invalid, 16 mm steps for 4 m) and JPEG-encodes it with OpenCV on a worker thread, capped at 25 % of a core (`depth_stream.host_encoder` has the counters). In `points` mode it needs full depth frames, so it opens the full depth gate while watched ("fallback: raw depth over the link, encoded on the Pi").
- `auto` (default) is `mono_features`, and switches to `host_jpeg` for the rest of the run the first time the encoder stalls or its frames are blank (logged once; `fallback` in `/api/status`), or from the start if the width is not a multiple of 32.
- The VIO loop writes the newest JPEG to `/dev/shm/oakd-vio-telemetry/depth_stream.jpg`; `main.py` polls it every 50 ms and sends each new one as a multipart part.
- Encoder check: while streaming, the Script reports its count of frames sent about once a second. If that keeps rising for 3 s and no JPEG comes back, `depth_stream.encoder_stalled` is set ("camera converts frames but the encoder outputs nothing").
- Blank check: JPEGs that are byte-identical or have a gray standard deviation below 0.75 for 2 s while the VIO's own depth varies set `depth_stream.blank` ("blank frames from mono_features: …"). A covered lens or a flat wall gives a constant image and constant depth, so it is not flagged.
- Errors: the Script counts failed frames and sends the count and the latest error on its `status` output (`errors`, `last_error`, `device_stats`).

Why only StereoDepth's own frames go to the camera's encoder (RVC2 firmware a406121b strings, depthai-core v3.3.0, tests 12–14 on an OAK-D-W): a frame a Script creates has no pool, so `ImageManip` outputs a constant image and the `VideoEncoder` drops it silently; a frame the Script received and forwards keeps StereoDepth's pool, and the encoder takes RAW8 directly. RAW16 (`stereo.depth`, and the subpixel disparity) is refused by the encoder. The `VideoEncoder` also needs a width that is a multiple of 32 and a height that is a multiple of 2, every frame the size of the first, and source / timestamp / sequence metadata. The earlier Script-built variants (`manip`, `nv12`, `yuv400p`, broken on the camera) and a RAW8 disparity variant (needs subpixel off; the MJPEG loss probe on test18 measured p95 depth errors over 1 m at quality ≤ 95 at 30 fps) were removed; older configs that saved them load as `auto`.

`/api/status` → `depth_stream`: `variant`, `requested_variant`, `variant_reason`, `configured_encoding`, `encoding` (`gray8-mono-rectified-left` or `gray8-depth-linear`), `source`, `encoded_on`, `fallback`, `blank`, `encoder_stalled`, `bad_frames`, `size_problem`, `fps`, `kbps`, `center`, `device_log`, `debug_state`.

#### Debug mode

The VIO process keeps the camera's log lines: the last 30 from the stream's nodes (its Script and `VideoEncoder`) and the last 10 others, in `/api/status` → `depth_stream.device_log` and the panel's *camera log* box. With *Depth stream camera debug log* on, the device log level is raised to DEBUG while the camera streams and restored afterwards; the encoder then logs "VideoEncoder took '…' ms." per frame. Leave it off in normal use.

## Web UI

The page polls `/api/status` and `/api/history` once a second only while its tab is visible, snapshots only while *auto-refresh* is on, the logs every 15 s, and the ArduPilot parameters on load and on *Re-read*. A stopped VIO, a missing camera or a missing mavlink2rest show as such, and the page picks up again after a VIO restart.

### Live telemetry

The VIO process publishes its live status through files ([`app/vio_telemetry.py`](app/vio_telemetry.py)):

- Per frame, `Telemetry.record_frame()` adds one row to a preallocated numpy ring buffer, a rolling window of failure reasons and a few counters: about 0.02 ms per frame including the publisher step (a test fails above 0.2 ms).
- A publisher thread writes `status.json` at most twice a second and `history.npy` once a second, only while `main.py` asked within the last 10 s (each `/api/status` and `/api/history` touches a lease file). The camera chip temperature and the camera's load (`getLeonCssCpuUsage`, `getLeonMssCpuUsage`, memory and heap use) are sampled on that thread at most every 5 s, never in the VIO loop; the camera link counters about once a second whether or not anyone watches.
- The files live in `/dev/shm/oakd-vio-telemetry/` (RAM). Memory bound: 10 minutes of frames at min(fps, 60), +10 %, 12 float64 columns: at most 3.8 MB (1.9 MB at 30 fps).
- `/api/history` downsamples to at most 600 points and caches the parsed file by mtime. Pi CPU % comes from `/proc/stat`.
- The trajectory rotates each body-frame delta by the yaw integrated so far and starts at the origin facing +x. It drifts.

### Camera link bandwidth

How much the pipeline sends between the camera and the Pi over XLink, to judge whether it fits a PoE camera's 100 Mbps Ethernet ([`app/link_stats.py`](app/link_stats.py)). It is in the *Camera link* box, the depth widget's status line, `telemetry.link` in `/api/status` and the VIO's periodic log line (`link down 9.3 up 2.2 Mbps (10 s mean), peak 9.5, 9% of 100 Mbps, …`).

Measured on the vehicle (test17, OAK-D-W over USB, 640×400, 10 s means):

| Transport | fps | Down (Mbps) | Up (Mbps) | Largest streams |
|---|---|---|---|---|
| `points` | 10 | 3.3 | 0.7 | `depth_points` 1.9, `features` 1.3, `imu` 0.3; up `depth_points_rois` 1.0 |
| `points` | 30 | 9.3 | 2.2 | `depth_points` 5.7, `features` 4.0, `imu` 0.3; up `depth_points_rois` 2.9 |
| `full_frame` | 15 | 63.9 | 0.0 | `depth` 61.5, `features` 2.0 |
| `full_frame` | 30 | 127.5 | 0.0 | `depth` 123.1, `features` 4.0 |

At 30 fps `points` paired 99.7 % of the frames with 93.6 % valid depth at the features; the camera's Leon CSS / MSS CPU was 63 % / 42 %. Both transports ran every rate up to 30 fps with all frames ok; above about 31 fps StereoDepth (subpixel on) is the limit.

- **Measured total:** `device.getProfilingData()`, a host-side read of XLink's per-link byte counters (no RPC, no profiling switch). `numBytesRead` is down, `numBytesWritten` up. They count every XLink stream's payload (data, serialized metadata and a 24-byte trailer, including RPC and logs), not XLink's packet headers or the USB / TCP framing. If the counters are missing or stay at 0, the box says "n/a" with the reason.
- **Per-stream estimate:** for each host queue the VIO counts messages and bytes: image data plus a header estimate (ImgFrame about 516 B, EncodedFrame 466 B), tracked features 64 B each (the 32-byte descriptor is always serialized), IMU packets 156 B, MessageGroup 80 B, buffers 56 B, device log messages 64 B plus the text. The remainder (measured minus streams) is RPC, logs, trailers and frames dropped in a full queue.
- **Streams (down):** `full_frame`: `depth` (512 kB per RAW16 frame) and `sync_group`; `points`: `depth_points` (64 B + 90 B per window), `depth_points_status`, and `depth_on_demand` (full depth frames, only while the depth probe runs or a `host_jpeg` stream is watched). Both: `features`, `imu`, `snapshot`, `depth_stream` and `depth_stream_status` (only while watched), `device_log`.
- **Streams (up, ↑):** `depth_points_rois` (the 56 B tag, then the config, 56 B + 45 B per window) and `control` (gate on/off and snapshot requests). `streams_up_sum_mbps` sums them apart from the down streams' `streams_sum_mbps`.
- **Fields** (`telemetry.link`): `available`, `reason`, `source`, `usb_speed`, `protocol`, `down_mbps` / `up_mbps` (last second), `down_mbps_10s`, `up_mbps_10s`, `peak_down_mbps_60s`, `totals_bytes`, `streams` (`name`, `description`, `direction`, `mbps`, `msgs_per_s`, `bytes_per_msg`), `streams_sum_mbps`, `unaccounted_mbps`, `share_of_100mbps`, `peak_share_of_100mbps`. The window resets when the camera reconnects.
- **100 Mbps:** the share is against the raw Ethernet rate; the usable rate on a PoE camera is lower, so a share near 1 does not fit.

### Cockpit widget

The extension offers a Cockpit widget, *OAK-D VIO depth*: the live stream with the tracker's features on top and a status line. In Cockpit, open Edit mode; the widget is in the widget list, offered by BlueOS. It is also the page `widget/depth`.

- **Discovery:** `GET /register_service` declares `works_in_relative_paths` (BlueOS proxies the extension at `/extensionv2/oakdvio/`) and `extras.cockpit: /cockpit_extras.json`. The widget entry uses `useExtensionPathAsBaseUrl`, so the iframe works from a remote Cockpit too. Every URL in both pages is relative.
- **Features**, coloured by state: *inlier* (green), *rejected* by RANSAC (magenta square), *matched* without a RANSAC result (white ring), *depth* (light blue: not in the reference, valid depth), *no depth* (grey ring). [`app/static/features_overlay.js`](app/static/features_overlay.js), shared with the main page, maps the positions onto the displayed image (`object-fit: contain` letterboxing); `app/tests/test_features_overlay.py` runs it under node.
- **Status line:** VIO fps, ok %, inliers, features with depth, messages sent, the log-only flag, the last failure, the camera link, "depth points 99%" (red below 90 %) or "depth: full frames", the viewers, raw image fallbacks as warnings, and why there is nothing to show (VIO stopped, depth stream off).
- **Cost on the Pi:** nothing is decoded or drawn on the Pi. While a widget is connected the VIO loop writes `features.json` (about 4 KB for 300 features) at most every 0.2 s (0.22 ms per write on a desktop); with nobody watching, one clock comparison per frame and one stat every 0.5 s.
- **Options** (in the iframe URL): `?colorize=0`, `?features=0`, `?status=0`.

### 3D pose viewer

The *3D pose* panel on the main page, and the second Cockpit widget *OAK-D VIO 3D pose* (`widget/pose`), show the integrated pose in 3D: the vehicle as body axes, a small hull and the camera frustum (pitched by *Camera Angle*); the trail, a 1 m grid at the start depth and a dashed depth line; x, y, z and roll, pitch, yaw, the frame and the attitude source with its age; orbit controls, *Reset view*, *Clear trail* and *Follow*.

**Attitude** (viewer only; the MAVLink output does not use it): ArduPilot's `ATTITUDE`, read from mavlink2rest (one GET every 0.5 s with a 0.4 s timeout, every 5 s while it is missing). Between reads the attitude is carried forward with the camera's gyro rotation since the sample's time. A sample older than 3 s (mavlink2rest keeps the last message forever) is stale; the attitude then continues from the last one with the gyro deltas, labelled **gyro-integrated, drifting** (about 0.65°/s at rest on test12), until a fresh sample arrives. No bias estimate, no accelerometer correction.

**Frames:** `NED` (x north, y east, z down, m from the VIO start) while the autopilot attitude is or has been in use; `start-FRD` (the body frame at VIO start) until the first sample. roll, pitch and yaw are ZYX Euler angles; `q` is `[w, x, y, z]`. Switching never makes the trail jump: the first autopilot sample rotates the trail so far into NED (new epoch); a stale autopilot continues from its last attitude; a fresh one snaps the attitude back by whatever the gyro drifted, keeping the position.

**Integration** ([`app/pose_stream.py`](app/pose_stream.py)): each accepted frame adds the sent `position_delta` (body FRD) rotated by the attitude in use. It is dead reckoning, so it drifts.

`/api/status` → `pose`: `frame`, `frame_text`, `attitude_mode`, `attitude_source` (`autopilot` or `gyro`), `attitude_text`, `attitude_age_s`, `switches`, `last_switch`, `realigned` and `autopilot` (`fresh`, `age_s`, `reads`, `samples`, `errors`, `last_error`, `description`); empty while VIO is stopped.

**Feed:** while a viewer is connected (and 4 s after), the VIO loop writes `pose.json` at most 10 times a second and `pose_trail.json` (points 5 cm or 2 s apart, at most 1500) at most once a second. **three.js** r156 (MIT) is vendored in [`app/static/vendor/three/`](app/static/vendor/three/) with its license and the one change to `OrbitControls.js`; the main page loads it only when the panel scrolls into view.

### DVL overlay

With *DVL overlay source* `auto`, `main.py` ([`app/dvl_source.py`](app/dvl_source.py)) looks for a DVL in mavlink2rest and the vx/vy plot draws its velocity dashed. It reads another component's `VISION_POSITION_DELTA` (what the Water Linked DVL extension sends; our own system 1 / component 195 is excluded), so the overlay compares the two inputs the EKF would fuse. The message tree is read at most every 10 s until such a component turns up, then only that message is polled, at most twice a second, and only while the page asked for history in the last 10 s. Without a DVL the page shows "DVL not found".

### ArduPilot panel

The panel reads the [recommended parameters](#autopilot-parameters) with `PARAM_REQUEST_READ` and shows current, suggested (editable), status (`ok`, `differs`, `unreadable`), why and *reboot*, plus the armed state from the HEARTBEAT (`base_mode & 128`). Reading never writes; it also refreshes the [parameter check](#the-check-and-the-configure-button).

Writing (`POST /api/autopilot/params`, from *Write selected…* or the check's *Configure…*):

- A dialog lists each change (`NAME old → new`, reboot flag) and the armed state; a second, explicit click sends `{"confirm": true, "params": {...}}`. Nothing is written automatically or at startup.
- 400 without `"confirm": true`, for any name outside the suggested table, and for values outside the parameter's range or allowed values; one bad entry rejects the whole request.
- 409, nothing written, unless a HEARTBEAT that arrives during the request (within 2.5 s) says disarmed. The armed bit is checked again before each `PARAM_SET`; if the vehicle arms midway the rest are reported as failed and not written.
- Each written parameter is read back. The response lists `changed`, `failed`, `unchanged` and `reboot_required`.

### Mock mode (no OAK-D, no autopilot)

`OAKD_MOCK=1` (or `main.py --mock`) runs the extension's own server on a PC (x86 is fine), with the page and the Cockpit widgets fully working. `main.py` launches [`app/mock_vio.py`](app/mock_vio.py) through the same supervisor instead of `pnp_vio.py`; the fake VIO publishes through the real telemetry, stream, feature, pose, snapshot and depth-probe services, so every server path runs as with a camera. depthai is never imported.

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt          # tests also need: .venv/bin/pip install pytest httpx
cd app
../.venv/bin/python main.py --mock --host 127.0.0.1 --port 8095
# open http://127.0.0.1:8095/, http://127.0.0.1:8095/widget/depth and http://127.0.0.1:8095/widget/pose
```

- **Data** in `OAKD_MOCK_DIR` (default `/tmp/oakd-vio-mock-<uid>`). The first configuration is 640×400 at 10 fps, to spare the PC's CPU.
- **Camera:** a panning seabed texture, depth on a tilted plane with three nearer moving blobs and holes, a right image warped by the matching disparity; features in every state; a turbid spell of 2.5 s every 50 s where frames fail. **Motion:** a lawnmower survey that closes every 164 s, with depth steps and a slow roll / pitch wobble.
- **Depth transport:** `points` and `full_frame` with the real host code: an SLC stand-in computes the same windows on the synthetic depth and answers through the same messages; the camera link counters are synthetic (`mock (synthetic XLink counters)`), about 4.1 Mbps down and 1.1 up with `points` at 10 fps.
- **Live stream:** `auto` / `mono_features` send the synthetic rectified left image (with the synced features); `host_jpeg` uses the real `HostJpegEncoder`. **Snapshots** come as JPEGs (OpenCV standing in for the camera's encoder), depth raw, so the depth probe and the focal check give real results; RGB is "not available".
- **Autopilot:** `mock_mode.MockAutopilot` with the dev vehicle's parameters, so the parameter check warns about `VISO_POS_X/Y/Z` and `EK3_SRC1_POSXY`; *Configure…* writes in memory. `OAKD_MOCK_ARMED=1` makes it armed (writes refused). An emulated `ATTITUDE` feeds the 3D pose, with an 8 s gap every 120 s (`OAKD_MOCK_ATTITUDE` = `gaps`, `on` or `off`).
- **Network:** urllib refuses every URL, so nothing leaves the machine. A purple MOCK badge and `"mock": true` in `/api/status` mark it.

[`app/tools/ui_screenshots.py`](app/tools/ui_screenshots.py) takes headless Chrome screenshots of a mock server (scenarios `main`, `armed`, `live`, `link`; its docstring has the commands). [`app/tools/fake_mavlink2rest.py`](app/tools/fake_mavlink2rest.py) is the tests' fake autopilot.

## Log-only mode and data logs

On a vehicle whose EKF already uses ExternalNav (`VISO_TYPE` 1, `EK3_SRC1_VELXY` 6), every `VISION_POSITION_DELTA` the extension sends is fused. To record VIO without affecting the vehicle, turn on log-only mode: tick *Log only* on the page, set `OAKD_LOG_ONLY=1` in the container environment (forces it whatever the saved setting; the page shows that it is forced), or run `pnp_vio.py --log-only`. The VIO process then creates no MAVLink connection and logs `LOG-ONLY: no MAVLink output, autopilot params untouched`. The page's parameter check keeps running; it only reads.

Each VIO process start writes a JSONL file to `/app/data/logs/` (`/usr/blueos/extensions/oakd-vio/logs/` on the vehicle), `vio-YYYYmmdd-HHMMSS-<pid>.jsonl`, in both modes. Every line has `type`, `wall_time` (ISO, UTC), `unix_time` and `monotonic`, and is flushed as written.

- `"type": "event"` lines: `start` (arguments, `log_only`, the resolved `stereo` settings, `depth_scale`, `refractive_index`, intrinsics and lever arm settings), `warning`, `device_open`, `device`, `calibration` (raw CAM_B intrinsics and distortion, the computed `rectified` matrix, alpha, baselines, `R_cam_imu`), `stereo_config` (`requested`, `effective`, `warnings`), `intrinsics` (the matrix used and its `source`, `effective_intrinsics`, the raw / rectified / frame matrices, `frame_matches`, scales), `depth_transport`, `snapshots`, `recovery`, `reconnect`, `tracking` changes, `error`, `exit`.
- `"type": "step"` lines: one per frame, with `device_timestamp`, `sequence`, `time_delta_us`, `position_delta` and `angle_delta` (body FRD, exactly what is or would be sent; `null` for rejected frames), `confidence`, `features`, `new_features`, `tracked_features`, `valid_depth`, `matches`, `inliers`, `reprojection_rms_px`, `gyro_disagreement_deg`, `proc_ms`, `status` (`ok` or `rejected`), `reason`, `sent`, `reference_age_s`, `gap_frames`, `reference_reset`, and the lever-arm fields when the diagnostics are on.

The supervisor appends its launches, exits, restarts and stops to `supervisor.jsonl` in the same directory. Nothing is rotated or deleted. The page links to the files.

## Depth at the features (`depth_transport`)

The PnP needs the depth at its tracked features only, about 300 numbers per frame. With `full_frame` the whole RAW16 depth frame (512 kB at 640×400, 123 Mbps at 30 fps) crosses to the Pi, which looks the depth up itself. With `points` (the default) the depth frame stays on the camera ([`app/depth_points.py`](app/depth_points.py)):

- **On the camera:** a `SpatialLocationCalculator` (SLC, on the SHAVEs) gets one ROI per feature: the 5×5 window around it on the depth grid (clamped to the frame), `AVERAGE`, depth thresholds = *Min / Max depth*. AVERAGE, not MEDIAN: the firmware computes MEDIAN only on a depth frame straight from StereoDepth, and ours comes through the pairing Script (test16b: every ROI came back as an average). All 25 pixels are sampled. Three more ROIs: the centre 5×5 patch and centre pixel (the live panel's centre depth) and a 1×1 marker at the bottom-right pixel (the depth size, so every message checks itself).
- **ROIs from the Pi:** the Pi builds the SLC config from the features it receives anyway (0.34 ms for 300 on a desktop) and sends it up right after a *tag*: the camera reads 0 as the sequence number of every `SpatialLocationCalculatorConfig` the Pi sends, while a plain `Buffer`'s sequence number survives the link. So per frame the Pi sends a `Buffer` with the features' sequence number and then the config, on the same queue; the Script pairs a config with the tag right before it and reports a config without a tag or a tag without a config ("pairing" errors). (A camera-side ROI builder was tried on test16b: 111 ms of Leon CPU per frame, under 10 fps; it was removed.)
- **Sync (no stale depth):** the Script takes a request, takes depth frames from its input (non-blocking, the newest 4) until one has the request's sequence number (older ones are dropped; a newer one means the frame is gone: a reported miss), sends the config and that frame to the SLC (blocking inputs, `inputConfig.setWaitForMessage(True)`) and waits for the SLC's output before the next request, then stamps the output with the frame's sequence number and timestamp. The Pi pairs points and features by sequence number and checks the echoed ROIs against the windows it expects: a mispaired request can cost a frame, never give it another frame's depth.
- **Pipelining:** the Pi requests each features message as soon as it fetches it (at most 4 in flight; older queued ones are dropped unrequested, `host_behind_dropped`), so when the Pi is slower than a frame the camera already computes the next frames' depth.
- **Missing depth:** a frame whose points are missing, late (2.5 frame periods, at least 50 ms), for a later frame, unreadable, have the wrong ROI count, size or windows, or a timestamp more than half a frame away from the features, gets no depth, counted by reason (`not_requested`, `dropped`, `timeout`, `unreadable`, `count`, `size`, `roi`, `timestamp`). It still solves against the reference but cannot become the next reference: if it solved, the reference's inliers are moved into this frame's camera with the solved pose and kept (`reference_carried`), so one missing frame costs no delta. After 5 timeouts in a row the Pi stops waiting until points come again.
- **On the Pi:** the same back-projection, depth range, valid fraction (of the 25 sampled pixels), depth scale and refractive index as the frame lookup (`pnp_odometry.depth_from_points`).
- **Rate:** nothing is tied to a frame rate. Measured: the SLC takes 7–8 ms per frame of ~258 ROIs (test16b); at 30 fps 99.7 % of the frames paired (test17, see [Camera link](#camera-link-bandwidth)).
- **Full depth frames on demand:** the depth probe and a watched `host_jpeg` stream need whole frames; a gate Script forwards every n-th depth frame (`depth_on_demand`) only while one of them asks.
- **Status** (`/api/status` → `depth_transport.live`, the page's *Depth for PnP* box): `frames`, `paired`, `paired_percent`, `missing`, `missing_by_reason`, `last_problem`, `valid_percent`, frame drops (`feature_gaps`, `points_gaps`, `orphan_points`), `wait_ms_mean` / `wait_ms_max` / `wait_limit_ms`, `timestamp_diff_ms_max`, `algorithm` / `other_algorithm_rois`, `depth_buffer_frames`, `requested_ahead_max` / `host_behind_dropped`, `device` (the Script's errors, misses, `config_sequence` and SLC time every 100 frames), `full_depth`, plus `camera_load`. The VIO's 10 s log line adds the paired count, the missing reasons, the gaps and the Leon CSS / MSS CPU.

`full_frame` keeps the original path (Sync node, depth lookup on the Pi) as a fallback and A/B baseline.

## Failed frames

A frame fails when PnP finds no trustworthy pose: too few matches or inliers, a rotation that disagrees with the gyro, or an implausible translation. Nothing is sent for it, but the last good frame stays the reference, so the next good frame is solved against it and its `VISION_POSITION_DELTA` covers the whole gap: `time_delta_us`, `position_delta` and `angle_delta` span the same interval, and the gyro prior is integrated over it too.

The failed frame's own points have no pose, so they can't be added to the reference; as features stop being tracked, the reference only loses matches. A failed frame replaces the reference (the gap is dropped) when the reference is older than *PnP Max Reference Age* (1 s) or fewer than *Min Inliers* reference points are still matched. The translation gate (0.5 m) applies per frame interval: a solve that bridges *n* failed frames may move up to (n + 1) × 0.5 m; the gyro gate stays at 5°. Repeated or earlier timestamps are rejected and leave the reference alone.

Each step records `reference_age_s`, `gap_frames` and `reference_reset`; the 10 s summary counts the gaps bridged and the reference resets.

## Tracking recovery

In the first pool test (`vio-20261117-180438-381.jsonl`, 10 fps), PnP tracked every frame for 10 minutes with about 110 features. After the vehicle was handled (up to 1.9 rad/s, pitch −27° to +60°), it never recovered: every failure was `too_few_matches`, the camera kept reporting 10–40 features with depth on few of them, also during a slow full yaw turn (~0.1°/frame) and when static again at the starting heading, while a restart in the same scene gave 106 features and 92 % ok. The host kept no state beyond the previous frame and the gyro gate never fired, so the stuck state is in the camera's `FeatureTracker`, most likely its automatic per-cell corner thresholds adapting during blur and low texture (the firmware is closed).

After *PnP Recovery After* (20) consecutive failed frames, the VIO process:

1. drops the PnP reference, so the next frame starts a new baseline;
2. re-initialises the camera's `FeatureTracker` through its `inputConfig` queue, without restarting the pipeline: a config with the feature maintainer off, then the normal config one frame later, so the tracker detects features from scratch.

If tracking still fails, the wait before the next recovery doubles (20, 40, 80, ... frames, up to 16×); a good frame resets it. Each recovery logs a warning and a `recovery` event; the per-frame `new_features` and `tracked_features` show whether the tracker is re-detecting or stuck on old ids.

## Device open, restarts and link drops

- **Opening:** `device_open.py` polls `dai.Device.getAllAvailableDevices()` with a backoff of 0.5 s doubling to 5 s, waits 1 s after discovery, then opens the device, retrying in the same budget (`--device-timeout`, 30 s); then the process exits with code 2. On an OAK-D-W-PoE, reopening about 1 s after the previous pipeline closed segfaulted inside depthai.
- **Restarts:** a segfault can't be caught in Python, so `main.py`'s supervisor ([`app/vio_supervisor.py`](app/vio_supervisor.py)) restarts any VIO exit it didn't ask for after 2 s, doubling up to 30 s; the delay resets after 60 s of stable running. `/api/status` reports `restart_count`, `last_exit_code` (negative for a signal) and `restart_in`.
- **Link drops:** on test11 XLink dropped and depthai reconnected by itself, but host → camera queues stayed stuck (depthai-core v3.3.0's `XLinkOutHost` can wait for a reconnect that already happened), breaking snapshot requests, the stream control and the tracker config. The VIO process therefore restarts its own pipeline when the camera restarts under it (`DeviceRestartWatch`: depthai's `RECONNECTED` callback, a sequence number drop over 30, or the device clock going back over 2 s): it leaves the pipeline, reopens the camera with the usual backoff and starts a fresh pipeline and PnP reference, keeping the data log, MAVLink output, telemetry and 3D pose. A `reconnect` event and `telemetry.reconnects` / `last_reconnect` record it.

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

`/dev` and the `c 189:*` cgroup rule give the container access to the OAK-D over USB. The image and identifier names are historical (the extension started with depthai's Basalt VIO); they are kept so installed vehicles update in place.

### Vehicles without internet

If the vehicle can't reach Docker Hub, save the image on a computer that can and upload it:

```bash
docker pull --platform linux/arm64 williangalvani/blueos-oakd-basalt:arm64-rtabmap
docker save williangalvani/blueos-oakd-basalt:arm64-rtabmap -o oakd-vio-arm64.tar
```

Then use the upload option in BlueOS's extension manager (BlueOS versions with Kraken v2), with the same identifier, name and custom settings as above.

## Building

The image is built in two stages so that the slow part only runs once:

1. **Builder image** ([`dockerfile.builder`](dockerfile.builder)): compiles depthai-core (the `depthai-core` submodule, declared in `.gitmodules`) with its Python bindings. The gitlink pins `4d4a8c1`, four local commits (metrics, Python bindings, a fix for building with OpenCV off) on top of luxonis `fa1ae86`. That commit is not on luxonis upstream, so `git submodule update` only works once it has been pushed somewhere the submodule URL can reach. Built under QEMU for arm64 this takes hours, mostly compiling vcpkg dependencies. The prebuilt `williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap` is what the runtime image uses; it was built with the Basalt, RTAB-Map and PCL options that the dockerfile no longer enables (the runtime copies none of their libraries).

   ```bash
   git submodule update --init depthai-core
   docker buildx build --platform linux/arm64 -f dockerfile.builder \
     -t williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap --push .
   ```

2. **Runtime image** ([`dockerfile`](dockerfile)): a slim Debian image. From the builder it copies the depthai Python module (a single `depthai.cpython-313-aarch64-linux-gnu.so`) and the two libraries it links that Debian doesn't ship (`libusb-1.0.so` from vcpkg, `libdynamic_calibration.so`). It then adds the Python requirements and `app/` without `tests/` and `tools/` (`.dockerignore`). No system OpenCV: `cv2` comes from the `opencv-python-headless` wheel. This takes a few minutes. The builder has its own ignore file, `dockerfile.builder.dockerignore`, which BuildKit (`docker buildx`) uses so that only `depthai-core` goes into its context.

   ```bash
   docker buildx build --platform linux/arm64 \
     --build-arg BUILDER_IMAGE=williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap \
     -t williangalvani/blueos-oakd-basalt:arm64-rtabmap --push .
   ```

Tests (on any machine, no camera): `python -m pytest app/tests` with `requirements.txt`, `pytest` and `httpx` installed; the JavaScript tests need `node`.

## Code layout

| File | Purpose |
|---|---|
| `app/main.py` | Web UI and API; starts and stops the VIO process |
| `app/config_schema.py` | Every setting: schema, defaults, groups, server-side validation, migration of old configs, form fields |
| `app/static/ui.html`, `ui.css`, `ui.js` | The web page (inlined into one page by `main.py`) |
| `app/static/widget.html`, `widget.css`, `widget.js`, `widget_icon.svg` | The Cockpit depth widget page and its icon |
| `app/static/pose3d.js`, `pose_widget.html`, `pose_icon.svg`, `vendor/three/` | The 3D pose viewer and the vendored three.js r156 |
| `app/static/features_overlay.js`, `link_panel.js`, `param_check.js` | The features overlay, the camera link box and the parameter check banner, shared by the pages |
| `app/pnp_vio.py` | The VIO process: camera pipeline, host loop, MAVLink output |
| `app/pnp_odometry.py` | Pose math (numpy + OpenCV, no depthai): depth lookup, back-projection, gyro integration, PnP RANSAC, frame conversion |
| `app/depth_points.py` | `depth_transport: points`: SLC windows and configs, the pairing and full depth gate Scripts, decoding, the receiver |
| `app/stereo_config.py` | Stereo settings: defaults, command line, `StereoDepth` configuration and read-back; refractive index, depth scale and lever arm parsing |
| `app/rectified_intrinsics.py` | The rectified-left camera matrix, choosing the source, the focal check |
| `app/tracking_recovery.py` | Tracking recovery state machine and feature tracker re-initialisation |
| `app/mavlink_sender.py` | Websocket sender for mavlink2rest (`VISION_POSITION_DELTA`, `NAMED_VALUE_FLOAT`) |
| `app/device_open.py` | Waits for the OAK-D and opens it with backoff; `DeviceRestartWatch` |
| `app/vio_supervisor.py` | Restarts the VIO process with backoff when it exits unexpectedly |
| `app/vio_log.py` | Log-only mode selection and the JSONL data logs |
| `app/vio_telemetry.py` | Live status and plot history: ring buffer and publisher (VIO process), readers (`main.py`) |
| `app/link_stats.py` | Camera link bandwidth: XLink counters, rates, per-stream estimates |
| `app/leases.py`, `app/viewers.py` | Lease files between `main.py` and the VIO process; the viewer registry |
| `app/depth_stream.py` | Live stream: on-camera gate, `host_jpeg` encoder, blank and stall checks, VIO-side service, `main.py` reader |
| `app/feature_stream.py`, `app/pose_stream.py` | Features for the widget; 3D pose integrator and publisher |
| `app/autopilot_attitude.py` | ArduPilot `ATTITUDE` from mavlink2rest (read only) for the 3D pose |
| `app/snapshots.py`, `app/depth_probe.py` | Snapshots (requests, serving, camera JPEGs, raw fallback); the depth probe |
| `app/autopilot_params.py`, `app/param_checker.py` | The recommended parameters, guarded read and write through mavlink2rest; the periodic check |
| `app/cockpit_widget.py` | BlueOS service metadata, the Cockpit extras JSON and the widget's status line |
| `app/dvl_source.py`, `app/system_stats.py` | DVL overlay from mavlink2rest; Pi CPU from `/proc/stat` |
| `app/mock_mode.py`, `app/mock_vio.py` | Mock mode: mock autopilot, fake DVL and attitude, network block; the fake VIO with a synthetic camera |
| `app/tools/` | `check_rectified_focal.py` (offline focal check), `ui_screenshots.py`, `fake_mavlink2rest.py` |
| `entrypoint.sh` | Container entrypoint; `exec`s the web server |

## Known limitations

- 30 fps is the production rate; StereoDepth with subpixel tops out at about 31 fps on the camera.
- Live stream: `host_jpeg`'s Pi cost is not measured (it is capped at 25 % of a core).
- Camera link: the per-stream sizes are estimates, and the measured counters leave out XLink's packet headers and the USB / Ethernet framing. A PoE camera has not been run yet; the numbers above are over USB.
- `points`: the depth is the camera's 25-pixel AVERAGE where `full_frame` uses the Pi's median (the same on smooth surfaces, closer to the mean of both sides at an edge). The frame time is the features' rather than the depth frame's mid-exposure (the same offset on every frame). *PnP Intrinsics Source* `frame` has no depth frame and falls back.
- The gyro prior is not used to pre-filter matches before RANSAC, the gyro bias is not estimated, and the accelerometer is unused.
- Underwater: the flat-port model is paraxial; check it against a tape with the depth probe (`scaled_m`) at the centre and near the edges, and trim with *PnP Depth Scale*. `VISO_DELAY_MS` and ArduSub's response to `confidence` (`EK3_VIS_VERR_MIN/MAX`) are not tuned yet.
- 3D pose and trajectory: dead reckoning from the deltas; they drift, and motion during reference resets and recoveries is missing.
- Web UI: the plots lag by up to about a second and start over when VIO restarts; the DVL overlay has no history from before the page was opened.
