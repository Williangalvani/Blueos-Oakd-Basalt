# Option D: device front-end + Python PnP on the host

Prototype on branch `option-d-pnp`. **Not run on hardware yet.**

## Files

| File | What |
|------|------|
| `app/pnp_vio.py` | Entry point. Builds the depthai pipeline, runs the host loop, sends MAVLink. |
| `app/pnp_odometry.py` | Pure pose math (numpy + OpenCV, no depthai): depth lookup, back-projection, gyro integration, PnP RANSAC, frame conversion, `PnPOdometry` state. |
| `app/mavlink_sender.py` | depthai-free websocket sender for mavlink2rest (`VISION_POSITION_DELTA`, `NAMED_VALUE_FLOAT`). |
| `app/device_open.py` | Waits for the device to be discoverable, then opens it, with backoff (shared with `basalt_configurable.py`). |
| `app/vio_supervisor.py` | Used by `main.py`: restarts the VIO process with backoff after an unexpected exit. |
| `app/stereo_config.py` | depthai-free table of the `StereoDepth` settings: defaults, argparse flags, the `main.py` command line and form, `apply_stereo_config`, and the effective-config readback. |
| `app/snapshots.py` | depthai-free snapshot IPC: request files written by `main.py`, `SnapshotService` (polling, throttling, one in flight) and `SnapshotWriter` (PNG thread) in the VIO process. |
| `app/tracking_recovery.py` | `TrackingRecovery` (N failures, then reset with backoff), `FeatureTrackerReinit` (two-step config resend), `feature_id_counts`. |
| `app/tests/test_stereo_config.py`, `test_snapshots.py`, `test_tracking_recovery.py` | Plumbing, IPC/endpoint, and recovery tests. These include a `pnp_vio.run()` with a fake device that replays the collapse seen in the pool test. |
| `app/tests/test_pnp_odometry.py` | pytest unit tests on synthetic scenes. |
| `app/tests/test_device_open.py`, `app/tests/test_vio_supervisor.py` | Backoff and restart-policy tests (fake clock, and real subprocesses that exit or SIGSEGV). |

`main.py` is not wired up. On this branch it has no algorithm selector, so adding one wasn't trivial. Run it by hand:
```bash
python /app/pnp_vio.py --fps 15 --width 640 --height 400 --num-features 300 --vehicle-ip 127.0.0.1
```

## Design

**Device (OAK-D).**
- `CAM_B`/`CAM_C` feed `StereoDepth`, configured by `stereo_config.apply_stereo_config`.
  - The preset (ROBOTICS) is applied first, then every override: LR check 5, confidence 50, brightness 1–255, speckle 50, threshold 300–4000 mm, spatial/temporal off, decimation 1, subpixel 3 bits, median 7x7, distortion correction, alpha 0, edge fill 0, optional P2. See the README table.
  - On RVC2, ROBOTICS is DENSITY plus decimation 2, the spatial filter, speckle range 200, confidence 15 and LR threshold 10 (`StereoDepth.cpp`). That is why decimation and the spatial filter have to be set explicitly after it.
  - Extended disparity is forced off, since it is incompatible with subpixel. The median filter is turned off, with a warning, if subpixel bits ≠ 3.
- Depth is aligned to `RECTIFIED_LEFT`, so it shares a pixel grid with `stereo.rectifiedLeft`. If decimation is set anyway, `PnPOdometry(image_size=...)` scales the feature pixels to the depth grid for the lookup only; back-projection stays on the full grid.
- **Intrinsics** come from the first depth frame's `getTransformation().getIntrinsicMatrix()` (`--intrinsics-from frame`). That is the matrix of the grid the depth is actually on, including any alpha scaling (a calibration matrix would be wrong with alpha ≠ device default). `intrinsics_from_frame` scales a uniformly downscaled frame back up, and rejects a non-finite matrix, an aspect-ratio mismatch, fx/fy ≤ 0 or a principal point outside the image. In those cases it falls back to the CAM_B calibration with a warning. `cam_b`/`cam_c` force the calibration.
- **Confidence gating** of depth is not implemented. The `confidenceMap` output is RAW8 on the stereo input grid; it gets no LR check, alignment or post-processing, so it doesn't line up with the RECTIFIED_LEFT depth. Streaming it through `Sync` would add ~20 Mbit/s to the ~45 used on the 100 Mbit PoE link. The device's `confidenceThreshold` already invalidates the same pixels before depth leaves the camera, so a stricter host gate is just a lower threshold.
- `rectifiedLeft` feeds `FeatureTracker`: Harris (or `--corner-detector shi-tomasi`), 300 target features, motion estimator on with Lucas-Kanade optical flow (persistent ids), and the feature maintainer on.
- `outputFeatures` and `depth` are paired by an on-device `Sync` node (threshold of half a frame period).
- The IMU sends raw accel and gyro at 200 Hz on its own queue. Accel is enabled but not used yet.

**Host (Pi), for each synced group:**
1. Drain the IMU queue into a gyro buffer.
2. Look up depth at each feature: the median of the valid pixels in a 5×5 window. 0 and out-of-range values are rejected, and at least 30% of the window must be valid. Valid features are back-projected with the rectified-left intrinsics.
3. Match the features to the reference frame by id: the last good frame, usually the previous one. The reference supplies the 3D points, the current frame the 2D points.
4. Integrate the gyro between the reference and current timestamps to get the rotation prior `R_01`. The gyro buffer keeps 2 s before the older of the two (capped at 10000 samples), and `integrate_gyro` returns no prior if there is a hole over 0.1 s in the interval.
5. Run `cv2.solvePnPRansac`, seeded with the gyro rotation (EPnP minimal sets, then LM refine on the inliers).
6. Reject the frame if:
   - there are fewer than 15 inliers, or the inlier ratio is below 0.3;
   - the PnP rotation differs from the gyro by more than 5° (`--gyro-gate`);
   - the translation is over 0.5 m per frame interval, that is (1 + `gap_frames`) × 0.5 m for a solve that bridges failed frames.
7. On success, convert to body FRD and send `VISION_POSITION_DELTA` with the real `angle_delta` and a confidence of `100 · inlier_ratio · min(1, inliers/60)`. The current frame becomes the reference.
8. On failure, send nothing and keep the reference, so the next good delta (and its `dt`) covers the gap since the last good frame. Points from the failed frame aren't merged in: without its pose they can't be expressed in the reference camera. The failed frame replaces the reference instead (old "baseline reset", the gap is dropped) if the reference is older than `--max-reference-age` (1 s; 0 = no cap) or fewer than `--min-inliers` reference points are still matched. Repeated or earlier timestamps leave the reference alone. `VIO_TRACK` goes to 0. Until task 6 every failed frame became the reference, which dropped the motion between the last good frame and the failure.
9. After `--recovery-frames` (20) consecutive failures (`no_reference` frames don't count), `PnPOdometry.reset()` drops the reference. A `FeatureTrackerConfig` with the maintainer off goes to `feature_tracker.inputConfig`, then the normal config the next frame. The next recovery waits twice as long, up to 16×; a success resets the wait.

**Snapshots.** A `Script` node gets `rectifiedLeft`, `rectifiedRight`, `depth` and, if the device has CAM_A, a 640×400 NV12 output at 5 fps. Each goes into a non-blocking size-1 input, so it holds just the latest frame. A `dai.Buffer` on its `request` input, with the stream index as sequence number, makes it forward one frame. `stereo.setNumFramesPool(4)` (default 3) covers the frame each gate input keeps. On the host, `SnapshotService.tick()` and one `tryGet()` run each frame. They are counted in `proc_ms`. The PNG work runs on the `SnapshotWriter` thread. The request/response files are described in the README.

**Logging.**
- Every frame gets a `logger.debug` line with timing, counts and the failure reason. Loguru's default sink shows DEBUG.
- Every 10 s an INFO summary reports fps, ok/total, mean/max processing ms, mean inliers and failure reasons.

## Conventions and assumptions

- **Camera frame:** OpenCV optical frame of the rectified left camera (x right, y down, z forward).
- **Units:** metres, radians, seconds. Depth is `uint16` mm.
- **PnP output:** maps `X1 = R_10·X0 + t_10` (C0 = previous, C1 = current).
  - `angle_delta = rotvec(R_01)`, with `R_01 = R_10ᵀ`: the current frame's rotation relative to the previous one.
  - `position_delta = −t_10`: the camera displacement from C0 to C1, **expressed in the current frame**, per the MAVLink definition of `VISION_POSITION_DELTA`. Over one frame the difference from "expressed in the previous frame" is second order.
- **Body frame:** the same chain as `MavlinkNode`. Optical → FLU (`x=z, y=−x, z=−y`), then the same `camera_angle` pitch matrix as `MavlinkNode.create_camera_rotation_matrix`, then FLU → FRD. The same transform is applied to the rotation vector. A forward camera maps optical forward/right/down to FRD x/y/z. With `--camera-angle 90`, optical forward becomes body down. Both cases are unit tested.
- **Timestamps:**
  - Frame time is the depth frame's device timestamp at mid-exposure (`CameraExposureOffset.MIDDLE`), which is how `RTABMapVIO` stamps frames.
  - Gyro samples use the device timestamp of each `gyroscope` report, on the same clock.
  - `time_delta_usec` comes from device timestamps. `time_usec` is host wall time, as in `MavlinkNode`.
- **Gyro → camera:** the rotation part of `calib.getImuToCameraExtrinsics(CAM_B, True)`. The integration holds each sample until the next one (zero-order hold). Gyro bias isn't estimated; a raw bias of a few deg/s is well inside the 5° per-frame gate at 15 fps, but over a reference held for the full 1 s it would use most of the gate. If the extrinsics are missing, the gyro prior and gate are disabled.
- **Dropped frames:** the sync output queue is non-blocking with size 4. If the host falls behind, frames are dropped. Ids persist on the device, so the next delta simply covers a longer `dt`.
- **NAMED_VALUE_FLOAT names (max 10 chars):** `VIO_TRACK` (0/1, sent on every change plus at `--quality-rate`), `VIO_INLIER`, `VIO_FEAT`, `VIO_CONF`, `VIO_PROCMS`.
  - `name` goes out as a NUL-padded char list, the way `AutopilotParams` encodes `param_id`.
  - `time_boot_ms` is milliseconds since the sender started.

## Verified locally (x86, no device)

- `python -m pytest app/tests` passes: 13 tests. They cover:
  - recovering a known motion, with and without a gyro prior;
  - 30% gross outliers;
  - too few points;
  - pure noise;
  - the gyro gate;
  - depth median, hole and range rejection;
  - gyro integration, including IMU→camera rotation;
  - body-frame mapping;
  - confidence;
  - an end-to-end `PnPOdometry` run on rendered depth images of a slanted plane, including a tracking loss and `dt` continuity.
- `pnp_vio.py`, `pnp_odometry.py` and `mavlink_sender.py` pass `ast.parse` and pyflakes.
- `pnp_vio.main()` was run against a mocked depthai, with a fake pipeline feeding 40 synthetic frames (0.3 m/s forward, 10°/s yaw, one dropout) and fake 200 Hz gyro. It sent 37 `VISION_POSITION_DELTA`s of +0.02 m x and +0.667° yaw each, as expected. The dropout cost 2 deltas without a jump, `VIO_TRACK` toggled 1→0→1, and host time was about 1 ms per frame on the desktop.
- depthai isn't installed on this machine. All depthai calls were checked by name against the `depthai-core` v3.3.0 bindings source but **never executed**.
- Stereo, snapshot and recovery work (task 5): `python -m pytest app/tests` passes 119 tests, including:
  - stereo settings round trips through the API, the `main.py` command line and argparse;
  - `apply_stereo_config` call order against a mock node;
  - the snapshot request → device → PNG → endpoint path, with a fake device and a thread standing in for VIO;
  - the recovery state machine;
  - `pnp_vio.run()` replaying the collapse with a fake pipeline. That run checks two recoveries, maintainer off/on configs, the `intrinsics` event `source: frame`, and a snapshot served mid-run.
- Reference held across failed frames (task 6): 129 tests, adding a slanted-wall run where frame 1 fails and frame 2's delta and `dt` cover 0 → 2, the per-interval translation gate, the age cap (and 0 = no cap), a dying reference, timestamps that go backwards, gyro integration over long intervals with holes, recovery clearing a held reference, and `pnp_vio.run()` runs checking `time_delta_us` and the gyro prior over a 2.6 s gap.
- Snapshot host overhead on a desktop with a 300-feature PnP frame: 0.9 ms/frame without snapshots, 1.0 ms with a request every frame (`/tmp/bench_snapshots.py`, not in the repo).

## To verify on hardware

1. **The pipeline starts:** `Sync` runs on the device with `FeatureTracker` + `StereoDepth` + subpixel at 640×400 / 15 fps. `setHardwareResources(1, 2)` must allow 300 features next to stereo. Watch for "Sequence mismatch" warnings, which mean the sync pairs the wrong frames.
2. **Rectified-left intrinsics:** the script reads the matrix from the depth frame (`intrinsics` event, `source: frame`). Compare it with the `calibration` event. With alpha 0 they should differ; if the frame matrix equals the raw calibration, the depth frame's `ImgTransformation` doesn't carry the rectification, and the alpha-scaled matrix must come from elsewhere. Check by holding the camera still in front of a flat wall at a measured distance: back-projected features should lie on a plane at the right depth.
3. **Depth alignment:** `RECTIFIED_LEFT` alignment should put depth on the feature pixel grid. The other checkout uses `setDepthAlign(CAM_B)`, which may warp depth to the *unrectified* left camera and offset it from the feature pixels.
4. **Rotation sign/frame:** the IMU→CAM_B rotation ignores the small rectification rotation (R1). Rotate the camera by hand: the per-frame `gyro_diff` in the debug log should stay well under 1–2°. If it's large and grows with rotation speed, the extrinsics or the axis convention are wrong.
5. **Sign check, as in HANDOFF:** forward/right/down motion should give +x/+y/+z `position_delta`, and yawing right should give +z `angle_delta`.
6. **Pi CPU and processing time:** compare the 10 s summaries with RTABMap and Basalt. The Python per-feature loops (building `ids`/`uv` from `trackedFeatures`) are the likely hot spot.
7. **mavlink2rest message format:** check that `NAMED_VALUE_FLOAT` with a char-list name and `VISION_POSITION_DELTA` are accepted. Check `/mavlink2rest/mavlink/vehicles/1/components/195/messages/...`.
8. **ArduSub's response to `confidence`:** check how ArduPilot uses it (the quality threshold in `AP_VisualOdom`), and tune the formula or `--min-inliers`.
9. **Underwater behaviour:** low texture, backscatter and refraction (the in-air calibration makes depth scale wrong behind a flat port).
10. **Two OpenCVs in one process:** the image installs `opencv-python-headless` from PyPI (4.14 abi3 aarch64 wheel with numpy 2.x on trixie's Python 3.13), while the depthai bindings link the system/vcpkg OpenCV. The wheel bundles its own renamed libraries, so they shouldn't clash. Still, check that `import depthai, cv2` works in the container and that the first PnP frames solve.
11. **Stereo config:** compare `effective` in the `stereo_config` event with the requested settings. Check that the preset's `setPostProcessingHardwareResources(3, 3)` and `FeatureTracker.setHardwareResources(1, 2)` fit together (the pipeline starts), and look at the fraction of valid depth on features (`valid_depth / features`) against the old 20–30%.
12. **Snapshots:** each stream from the page, while watching the 10 s fps summary and `proc_ms`; RGB on a device with CAM_A, and a 404 on one without. Check that `NumFramesPool(4)` doesn't cause frame drops or "pool exhausted" stalls on the stereo outputs, and how much CAM_A at 5 fps costs the camera.
13. **Recovery:** after a recovery, `new_features` should jump (re-detection) and `features` return towards 300. If the count stays low, the maintainer toggle doesn't reset the tracker's corner thresholds. The next things to try then are sending a config with explicit `CornerDetector.Thresholds` (e.g. a low `initialValue`), or re-sending `setNumTargetFeatures`.
14. **Min inliers vs sparse depth:** at the end of the pool test, frames had 7–12 consistent matches and were all rejected by `--min-inliers 15`. Look at whether 8–10 with a tighter reprojection threshold is safe underwater.

## Tracking degradation in the first pool test

Data: `~/oakd-stage1/flight1/vio-20261117-180438-381.jsonl` (14530 frames at 10 fps, ~24 min). Vehicle attitude comes from the matching `rovrec-window.jsonl` (`ATTITUDE`, DVL). Times are seconds from the first frame.

| Window | ok | features (mean / min) | valid depth | matches | Vehicle |
|---|---|---|---|---|---|
| 60–600 s | 100% | 89–114 / ≥78 | 23–33 | 22–33 | static, rotation rate ≤ 0.01 rad/s |
| 600–660 s | 51% | 68 / 3 | 19 | 17 | handled from 615 s: up to 1.9 rad/s, pitch −22° to +22° |
| 660–720 s | 4% | 40 / 3 | 6 | 3 | handled: pitch −27° to +60°, yaw −55° → −97° |
| 720–1200 s | 0% | 35–47 / ≥11 | 2–3 | 2–3 | one slow full yaw turn (≤ 0.018 rad/s, ~0.1°/frame), level |
| 1200–1380 s | 0–1% | 2–19 / 0 | 0.5–5 | 0.2–4 | handled again, up to 0.85 rad/s yaw rate |
| 1385–1450 s | 0% | 9–16 | 5–12 | 5–12 | static, yaw back to −64° (it started at −55 to −66°) |

Every failure after 690 s is `too_few_matches`. In the whole run there are no `gyro_mismatch` rejections and one sequence gap. A restart in the same scene gave 106 features and 92% ok (from the task description).

Ruled out:
- **Host reference or gyro prior stuck:** at the time, `PnPOdometry` replaced its reference with the current frame on every frame, ok or not (`prev_points = current`). The only cross-frame state is that reference and the gyro buffer, and the gyro gate never fired. A stuck host would show many matches and a rejection reason other than `too_few_matches`.
- **No keyframe to reset:** PnP is frame-to-frame.
- **Scene alone:** during the 720–1200 s turn the camera swept every heading, including the starting one, at ~0.1°/frame, slow enough for LK tracking. Features stayed at 35–47 and never got back to ~110. At the end the vehicle was static, with depth on 60–70% of the features (it was ~30% at the start), at about the starting heading. Yet the tracker reported 9–16 features instead of ~110. The DVL suggests about 1.5 m of translation during the second handling, but it drifts ~0.5 cm/s while static, so that is weak evidence.

Conclusion: the collapse is in the camera's `FeatureTracker`, which never repopulated after the blur and low-texture periods. The likely mechanism is its automatic per-cell corner thresholds (`CornerDetector.Thresholds.initialValue = 0`), which adapt while the image is blurred and/or after the maintained set has been decimated. The firmware is closed, so this can't be fixed at the source. The mitigation is the recovery in this branch (reset plus a re-sent config with the maintainer toggled), and the new per-frame `new_features`/`tracked_features` will show on the next run whether re-detection happens. A contributing factor is that the stereo config at the time (no preset, LR threshold 10, no brightness or threshold filter) gave depth on only 20–30% of features, so any feature loss quickly fell below 15 matches. The new stereo defaults aim to raise that fraction; this is unverified.

## Running it in the extension

Select *PnP* as the algorithm on the configuration page (or POST `"algorithm": "pnp"` to `/api/config`). `main.py` then starts `python -u /app/pnp_vio.py` with the FPS, camera angle, resolution, vehicle IP, quality rate, *PnP Features*, *PnP Recovery After*, *PnP Max Reference Age*, *Snapshots* and *Stereo Depth (PnP)* settings. Output is unbuffered and goes to the container log. The web UI is on port 8095 (8080 is taken by mavlink-server on BlueOS 1.4.x). At startup the log shows the device name, IMU and USB speed. Every 10 s it prints fps, ok/total, mean/max processing time, mean features, mean inliers, failure reasons and the chip temperature.

## Device open and restarts

Seen on an OAK-D-W-POE (2026-11-17): restarting `pnp_vio.py` about 1 s after the previous pipeline closed segfaulted inside depthai ("Address not mapped") before device info was printed. Restarting ~75 s later worked, and the open took ~9 s. Two layers handle this:

- `device_open.py`: poll `dai.Device.getAllAvailableDevices()` with a backoff of 0.5 s doubling to 5 s, wait a 1 s settle delay after discovery, then `dai.Device(info)`. Open errors are retried in the same budget (`--device-timeout`, 30 s). When the budget runs out the script logs the reason and exits with code 2. The pipeline is then built with `dai.Pipeline(device)`.
- `vio_supervisor.py`: a segfault can't be caught in Python, so `main.py` restarts any VIO exit it didn't ask for after 2 s, 4 s, ... up to 30 s. The delay resets after 60 s of stable running. Stop, Start and config changes don't trigger restarts. `/api/status` adds `restart_count`, `last_exit_code` and `restart_in`.

Not checked on hardware: whether the 1 s settle delay is enough to avoid the segfault, or whether the supervisor's backoff ends up absorbing it.

## Open issues and notes for the existing code

- Fixed since: `MavlinkNode.send_named_value_float` used to send names longer than 10 chars as plain strings and epoch milliseconds as `time_boot_ms`, which overflowed `uint32`. It now uses the short names `VIO_QUAL`/`VIO_FEAT`/`VIO_LMKS`/`VIO_PROC`/`VIO_TRACK`, NUL-padded char lists, and milliseconds since process start.
- `rtabmap_vio.py` in the other checkout sets `initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49`. That assigns an attribute on the nested *class*, not on the config, so it has no effect. The config's fields aren't exposed in the Python bindings; use `setFeatureMaintainer(...)` instead.
- The gyro prior isn't used to pre-filter matches before RANSAC. With the EPnP kernel, the initial guess mostly matters for the final LM refine; the gyro's main job is the gate.
- Accelerometer data is collected but unused. Metric scale comes from stereo depth.
