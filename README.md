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
| Vehicle IP | 127.0.0.1 | Where mavlink2rest is reachable. The extension uses host networking, so the default works on the vehicle itself. |
| Quality Rate (Hz) | 1 | Rate of the `NAMED_VALUE_FLOAT` quality messages. |
| Log only | off | Run VIO as a passive logger: no autopilot parameter changes and no MAVLink output at all. See [Log-only mode](#log-only-mode). |

The defaults are tuned for low CPU usage on a Raspberry Pi.

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

## Log-only mode and data logs

On a vehicle whose EKF already uses ExternalNav (`VISO_TYPE` 1, `EK3_SRC1_VELXY` 6), every `VISION_POSITION_DELTA` the extension sends is fused. To record VIO without affecting the vehicle, turn on log-only mode in any of these ways:

- tick *Log only: do not send anything to the autopilot* on the configuration page,
- set `OAKD_LOG_ONLY=1` in the container environment; this forces log-only whatever the saved setting, and the page shows that it is forced,
- run `pnp_vio.py` or `basalt_configurable.py` with `--log-only`.

In log-only mode the autopilot parameters are neither read nor written and no MAVLink connection is created, so nothing reaches mavlink2rest. The VIO process logs `LOG-ONLY: no MAVLink output, autopilot params untouched` at startup.

Each VIO process start writes a JSONL file to `/app/data/logs/` (`/usr/blueos/extensions/oakd-vio/logs/` on the vehicle), named `vio-YYYYmmdd-HHMMSS-<pid>.jsonl`, in both modes. Every line has `type`, `wall_time` (ISO, UTC), `unix_time` and `monotonic`. Lines are flushed as they are written.

- `"type": "event"` lines: `start` (arguments, `log_only`), `warning`, `device_open` attempts, `device`, `calibration`, `tracking` changes, `error`, `exit`.
- `"type": "step"` lines (PnP only): one per frame, with `device_timestamp`, `sequence`, `time_delta_us`, `position_delta` and `angle_delta` (body FRD, exactly what is or would be sent), `confidence`, `features`, `valid_depth`, `matches`, `inliers`, `proc_ms`, `status` (`ok` or `rejected`), `reason` and `sent`. The deltas are `null` for rejected frames.

Basalt runs write the events only, not per-frame poses. The supervisor appends its launches, exits (code or signal), restarts and stops to `supervisor.jsonl` in the same directory. Nothing is rotated or deleted. The configuration page links to the files.

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
| `entrypoint.sh` | Container entrypoint; starts the web server |

## Known limitations

- VIO runs on the Raspberry Pi CPU, which limits it to a few frames per second. The builder includes depthai's `RTABMapVIO` node, which can use feature tracking on the camera itself, but the pipeline does not use it yet.
- `VISION_POSITION_DELTA` reports a fixed confidence of 100% and no rotation delta.
- At startup the VIO process waits up to 30 s (`--device-timeout`) for the OAK-D to be discoverable and to open, retrying with backoff, and then exits with code 2. A PoE OAK-D can take many seconds to come back after the previous pipeline closed; opening it too early has been seen to segfault inside depthai.
- If the VIO process exits without being stopped from the page (error, crash or segfault), `main.py` restarts it after 2 s, doubling up to 30 s; the delay resets after a run of 60 s. `/api/status` reports `restart_count`, `last_exit_code` (negative for a signal, e.g. -11 for SIGSEGV) and `restart_in`.
