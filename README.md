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

## Configuration page

Open `http://<vehicle>:8095` (or the extension's entry in the BlueOS menu). Saving restarts VIO with the new values. Settings are stored in `/usr/blueos/extensions/oakd-vio/config.json` on the vehicle, so they survive restarts and updates.

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
| PnP Recovery After (frames) | 20 | PnP only: consecutive failed frames before [tracking recovery](#tracking-recovery-pnp); 0 disables it. |
| PnP Max Reference Age (s) | 1.0 | PnP only: how long [failed frames](#failed-frames-pnp) keep the last good frame as the reference (`--max-reference-age`); 0 removes the limit. |
| Snapshots | on | PnP only: allow [camera snapshots](#snapshots-pnp). Nothing is streamed until one is requested. |
| Vehicle IP | 127.0.0.1 | Where mavlink2rest is reachable. The extension uses host networking, so the default works on the vehicle itself. |
| Quality Rate (Hz) | 1 | Rate of the `NAMED_VALUE_FLOAT` quality messages. |
| Log only | off | Run VIO as a passive logger: no autopilot parameter changes and no MAVLink output at all. See [Log-only mode](#log-only-mode). |

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

PnP takes the camera matrix from the depth frame's own transformation (`--intrinsics-from frame`), so it matches the rectified-left grid even when alpha scaling changes it. If the frame has no plausible matrix, it falls back to the CAM_B calibration and logs a warning. The requested settings, the values read back from the node, and any warnings are logged at startup and written to the data log as a `stereo_config` event. The JSONL `start` event also includes them.

Depth is not gated with the stereo confidence map. On the camera, the `confidenceMap` output gets no left-right check, alignment or post-processing, so it doesn't match the depth pixels. Streaming it would also add about 20 Mbit/s, and the confidence threshold above already applies the same score on the camera.

The page is backed by a small HTTP API:

| Method | Path | Action |
|---|---|---|
| GET | `/api/config` | Current settings |
| POST | `/api/config` | Save settings (JSON body, same keys) and restart VIO |
| GET | `/api/status` | Whether the VIO process is running, restarts, and whether log-only mode is active (`log_only`, `log_only_forced_by_env`) |
| GET | `/api/logs` | Data logs: name, size (bytes), mtime (unix time), newest first |
| GET | `/api/logs/{name}` | Download one data log |
| POST | `/api/start`, `/api/stop` | Start or stop VIO |
| POST | `/api/reset` | Restore defaults and restart VIO |
| GET | `/api/snapshot?stream=<stream>` | PnP only: PNG of one current camera frame; `stream` is `rect_left`, `rect_right`, `depth` or `rgb`. See [Snapshots](#snapshots-pnp). |

### Snapshots (PnP)

`GET /api/snapshot?stream=rect_left|rect_right|depth|rgb` returns a fresh PNG from the running PnP VIO, without stopping it. The *Snapshots (PnP)* buttons on the page use it.

- `rect_left`, `rect_right`: the rectified mono images the stereo matcher sees, 8-bit grayscale. The feature tracker runs on `rect_left`.
- `depth`: depth aligned to `rect_left`, colorized from red (near) to blue (far) over the min–max depth range. Pixels with no depth are black. `&raw=1` returns the 16-bit depth in millimetres instead (`raw` is only valid for depth).
- `rgb`: the color camera (CAM_A) at 640×400 and 5 fps. It runs only on devices that have it; on others the request returns 404.

Responses carry `Cache-Control: no-store` and `X-Snapshot-Unix-Time`, `X-Snapshot-Device-Timestamp` and `X-Snapshot-Sequence` headers. A request returns 504 if no frame arrives within 3 s (PnP not running, or the camera is not answering), 404 if the stream is unavailable or snapshots are disabled, and 422 for an unknown stream.

How it works:

- On the camera, a `Script` node holds only the latest frame of each stream, in size-1 non-blocking queues. It forwards one frame only when the host asks for it, so snapshots use no USB/Ethernet bandwidth until requested. Each stereo output's frame pool grows from 3 to 4 frames to cover the frame the gate holds.
- `main.py` and the VIO process share `/app/data/snapshots/` (`/usr/blueos/extensions/oakd-vio/snapshots/` on the vehicle). The endpoint atomically writes a request file, `requests/<stream>`, holding the request time.
- The VIO loop checks the directory at most every 0.2 s and claims requests by renaming them. It sends at most one request to the camera per second, with only one in flight at a time; repeated requests for the same stream are merged. Each frame it does a non-blocking check for a reply.
- PNG encoding runs on a separate writer thread with a bounded queue, so the VIO loop never waits on it. The writer saves `<stream>.png`, plus `depth_raw.png` for depth, then writes `<stream>.json` last, holding the request time it answers. The endpoint waits until that file shows a request time at least as recent as its own.
- If the camera does not reply within 2 s, an error is written for the endpoint, and the next request can go out.

Host cost, measured on a desktop with a 300-feature PnP frame: about 0.9 ms per frame without snapshots, and about 1.0 ms with a request on every frame (a request rate 15 times higher than the throttle allows). The camera-side cost of the RGB camera and the extra frame per pool has not been measured on hardware.

## Log-only mode and data logs

On a vehicle whose EKF already uses ExternalNav (`VISO_TYPE` 1, `EK3_SRC1_VELXY` 6), every `VISION_POSITION_DELTA` the extension sends is fused. To record VIO without affecting the vehicle, turn on log-only mode in any of these ways:

- tick *Log only: do not send anything to the autopilot* on the configuration page,
- set `OAKD_LOG_ONLY=1` in the container environment; this forces log-only whatever the saved setting, and the page shows that it is forced,
- run `pnp_vio.py` or `basalt_configurable.py` with `--log-only`.

In log-only mode the autopilot parameters are neither read nor written and no MAVLink connection is created, so nothing reaches mavlink2rest. The VIO process logs `LOG-ONLY: no MAVLink output, autopilot params untouched` at startup.

Each VIO process start writes a JSONL file to `/app/data/logs/` (`/usr/blueos/extensions/oakd-vio/logs/` on the vehicle), named `vio-YYYYmmdd-HHMMSS-<pid>.jsonl`, in both modes. Every line has `type`, `wall_time` (ISO, UTC), `unix_time` and `monotonic`. Lines are flushed as they are written.

- `"type": "event"` lines: `start` (arguments, `log_only`, and the resolved `stereo` settings for PnP), `warning`, `device_open` attempts, `device`, `calibration`, `tracking` changes, `error`, `exit`. PnP also writes:
  - `stereo_config`: `requested` settings, `effective` values read back from the node, and `warnings`;
  - `intrinsics`: the camera matrix used (`fx, fy, cx, cy`) and its `source` (`frame`, `cam_b`, `cam_c` or `cam_b (fallback)`);
  - `snapshots`: the available `streams`, and `unavailable` ones with the reason;
  - `recovery`: see [Tracking recovery](#tracking-recovery-pnp).
- `"type": "step"` lines (PnP only): one per frame, with `device_timestamp`, `sequence`, `time_delta_us`, `position_delta` and `angle_delta` (body FRD, exactly what is or would be sent), `confidence`, `features`, `new_features` and `tracked_features` (ids not seen / seen in the previous frame), `valid_depth`, `matches`, `inliers`, `reprojection_rms_px`, `gyro_disagreement_deg`, `proc_ms`, `status` (`ok` or `rejected`), `reason`, `sent`, and `reference_age_s`, `gap_frames` and `reference_reset` (see [Failed frames](#failed-frames-pnp)). The deltas are `null` for rejected frames.

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
| `app/main.py` | Configuration web page and API; starts and stops the VIO process |
| `app/vio_supervisor.py` | Restarts the VIO process with backoff when it exits unexpectedly |
| `app/device_open.py` | Waits for the OAK-D and opens it, retrying with backoff |
| `app/vio_log.py` | Log-only mode selection and the JSONL data logs |
| `app/basalt_configurable.py` | The VIO pipeline: cameras, IMU, `BasaltVIO`, MAVLink output |
| `app/autopilot_params.py` | Reads and sets the autopilot's EKF parameters through mavlink2rest |
| `app/mavlink_node.py` | depthai host node that turns poses into MAVLink messages |
| `app/basalt.py` | Standalone command-line version of the pipeline, useful for testing |
| `app/pnp_vio.py`, `app/pnp_odometry.py` | Experimental PnP VIO: camera pipeline and host loop; pose math |
| `app/stereo_config.py` | PnP stereo settings: defaults, web form, command line and `StereoDepth` configuration |
| `app/snapshots.py` | Snapshot requests (`main.py`) and serving them from the VIO process |
| `app/tracking_recovery.py` | Tracking recovery state machine and feature tracker re-initialisation |
| `entrypoint.sh` | Container entrypoint; starts the web server |

## Known limitations

- VIO runs on the Raspberry Pi CPU, which limits it to a few frames per second. The builder includes depthai's `RTABMapVIO` node, which can use feature tracking on the camera itself, but the pipeline does not use it yet.
- `VISION_POSITION_DELTA` reports a fixed confidence of 100% and no rotation delta.
- At startup the VIO process waits up to 30 s (`--device-timeout`) for the OAK-D to be discoverable and to open, retrying with backoff, and then exits with code 2. A PoE OAK-D can take many seconds to come back after the previous pipeline closed; opening it too early has been seen to segfault inside depthai.
- If the VIO process exits without being stopped from the page (error, crash or segfault), `main.py` restarts it after 2 s, doubling up to 30 s; the delay resets after a run of 60 s. `/api/status` reports `restart_count`, `last_exit_code` (negative for a signal, e.g. -11 for SIGSEGV) and `restart_in`.
