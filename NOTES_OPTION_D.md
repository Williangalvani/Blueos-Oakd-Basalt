# Option D: device front-end + Python PnP on the host

Prototype on branch `option-d-pnp`. **Not run on hardware yet.**

## Files

| File | What |
|------|------|
| `app/pnp_vio.py` | Entry point. Builds the depthai pipeline, runs the host loop, sends MAVLink. |
| `app/pnp_odometry.py` | Pure pose math (numpy + OpenCV, no depthai): depth lookup, back-projection, gyro integration, PnP RANSAC, frame conversion, `PnPOdometry` state. |
| `app/mavlink_sender.py` | depthai-free websocket sender for mavlink2rest (`VISION_POSITION_DELTA`, `NAMED_VALUE_FLOAT`). |
| `app/tests/test_pnp_odometry.py` | pytest unit tests on synthetic scenes. |

`main.py` is not wired up. On this branch it has no algorithm selector, so adding one wasn't trivial. Run it by hand:
```bash
python /app/pnp_vio.py --fps 15 --width 640 --height 400 --num-features 300 --vehicle-ip 127.0.0.1
```

## Design

**Device (OAK-D).**
- `CAM_B`/`CAM_C` feed `StereoDepth`, with LR check, subpixel, distortion correction and LR threshold 10.
- Depth is aligned to `RECTIFIED_LEFT`, so it shares a pixel grid with `stereo.rectifiedLeft`.
- `rectifiedLeft` feeds `FeatureTracker`: Harris (or `--corner-detector shi-tomasi`), 300 target features, motion estimator on with Lucas-Kanade optical flow (persistent ids), and the feature maintainer on.
- `outputFeatures` and `depth` are paired by an on-device `Sync` node (threshold of half a frame period).
- The IMU sends raw accel and gyro at 200 Hz on its own queue. Accel is enabled but not used yet.

**Host (Pi), for each synced group:**
1. Drain the IMU queue into a gyro buffer.
2. Look up depth at each feature: the median of the valid pixels in a 5×5 window. 0 and out-of-range values are rejected, and at least 30% of the window must be valid. Valid features are back-projected with the rectified-left intrinsics.
3. Match the features to the previous frame by id. The previous frame supplies the 3D points, the current frame the 2D points.
4. Integrate the gyro between the two frame timestamps to get the rotation prior `R_01`.
5. Run `cv2.solvePnPRansac`, seeded with the gyro rotation (EPnP minimal sets, then LM refine on the inliers).
6. Reject the frame if:
   - there are fewer than 15 inliers, or the inlier ratio is below 0.3;
   - the PnP rotation differs from the gyro by more than 5° (`--gyro-gate`);
   - the translation is over 0.5 m per frame.
7. On success, convert to body FRD and send `VISION_POSITION_DELTA` with the real `angle_delta` and a confidence of `100 · inlier_ratio · min(1, inliers/60)`.
8. On failure, send nothing for that interval. The current frame becomes the new reference ("baseline reset"), so the next delta starts here and no jump is integrated. `VIO_TRACK` goes to 0.

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
- **Gyro → camera:** the rotation part of `calib.getImuToCameraExtrinsics(CAM_B, True)`. The integration holds each sample until the next one (zero-order hold). Gyro bias isn't estimated; a raw bias of a few deg/s is well inside the 5° per-frame gate at 15 fps. If the extrinsics are missing, the gyro prior and gate are disabled.
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

## To verify on hardware

1. **The pipeline starts:** `Sync` runs on the device with `FeatureTracker` + `StereoDepth` + subpixel at 640×400 / 15 fps. `setHardwareResources(1, 2)` must allow 300 features next to stereo. Watch for "Sequence mismatch" warnings, which mean the sync pairs the wrong frames.
2. **Rectified-left intrinsics:** the script uses `getCameraIntrinsics(CAM_B, w, h)`, the same as `RTABMapVIO`. depthai may rectify both images with the right camera's matrix; if so, try `--intrinsics-from cam_c`. Check by holding the camera still in front of a flat wall at a measured distance: back-projected features should lie on a plane at the right depth.
3. **Depth alignment:** `RECTIFIED_LEFT` alignment should put depth on the feature pixel grid. The other checkout uses `setDepthAlign(CAM_B)`, which may warp depth to the *unrectified* left camera and offset it from the feature pixels.
4. **Rotation sign/frame:** the IMU→CAM_B rotation ignores the small rectification rotation (R1). Rotate the camera by hand: the per-frame `gyro_diff` in the debug log should stay well under 1–2°. If it's large and grows with rotation speed, the extrinsics or the axis convention are wrong.
5. **Sign check, as in HANDOFF:** forward/right/down motion should give +x/+y/+z `position_delta`, and yawing right should give +z `angle_delta`.
6. **Pi CPU and processing time:** compare the 10 s summaries with RTABMap and Basalt. The Python per-feature loops (building `ids`/`uv` from `trackedFeatures`) are the likely hot spot.
7. **mavlink2rest message format:** check that `NAMED_VALUE_FLOAT` with a char-list name and `VISION_POSITION_DELTA` are accepted. Check `/mavlink2rest/mavlink/vehicles/1/components/195/messages/...`.
8. **ArduSub's response to `confidence`:** check how ArduPilot uses it (the quality threshold in `AP_VisualOdom`), and tune the formula or `--min-inliers`.
9. **Underwater behaviour:** low texture, backscatter and refraction (the in-air calibration makes depth scale wrong behind a flat port).

## Open issues and notes for the existing code (not changed here)

- `MavlinkNode.send_named_value_float` sends `name` as a plain string longer than 10 chars (`VIO_TRACKING_STATUS`). It sets `time_boot_ms` to epoch milliseconds, which overflows the `uint32` field. mavlink2rest may reject both, so check whether those metrics arrive at all.
- `rtabmap_vio.py` in the other checkout sets `initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49`. That assigns an attribute on the nested *class*, not on the config, so it has no effect. The config's fields aren't exposed in the Python bindings; use `setFeatureMaintainer(...)` instead.
- The gyro prior isn't used to pre-filter matches before RANSAC. With the EPnP kernel, the initial guess mostly matters for the final LM refine; the gyro's main job is the gate.
- Accelerometer data is collected but unused. Metric scale comes from stereo depth.
