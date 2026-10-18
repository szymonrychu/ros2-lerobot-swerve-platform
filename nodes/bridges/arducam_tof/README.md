# Arducam ToF bridge

**Purpose:** Publishes real depth and amplitude frames from an Arducam ToF camera (ArducamDepthCamera SDK, CSI) on ROS2, for the web UI head tab. Never publishes a zero, default or synthetic frame.

**Audience:** Mid-level Python developer with ROS2 (rclpy, sensor_msgs).

## Topics

All three messages of one frame share one header: the same stamp and `frame_id`.

| Topic | Type | Encoding | Content |
|-------|------|----------|---------|
| `/tof/depth/image_raw` | `sensor_msgs/Image` | `16UC1` | Depth in millimetres. `0` means invalid: SDK confidence below `confidence_threshold`, not finite, `<= 0` or beyond 65535 mm. |
| `/tof/amplitude/image_raw` | `sensor_msgs/Image` | `mono16` | The SDK `amplitude_data`, or `confidence_data` when the frame carries no amplitude, rounded and clipped to 0..65535. mono16 keeps the full dynamic range; viewers scale it for display. |
| `/tof/camera_info` | `sensor_msgs/CameraInfo` | - | Intrinsics from the SDK (`INTRINSIC_FX/FY/CX/CY`, reported x100), `plumb_bob` with zero distortion. Not published when the SDK reports no focal length. |

A timer ticks at `publish_rate_hz` and requests one frame per tick, so the publish rate never exceeds it.

## Failure behaviour (no placeholder data)

- A frame request that errors, times out (1000 ms), returns a non-depth frame or malformed arrays publishes nothing for that tick. It is logged (throttled to one line per 5 s) and counted in `tof_frames_dropped_total{reason}`.
- A camera that fails to open or start is logged, counted, released and retried every 5 s; nothing is published meanwhile.
- A camera that yields no valid frame for 5 s is closed and reopened.

## Config

YAML file named by `ARDUCAM_TOF_CONFIG` (default `/etc/ros2/arducam_tof/config.yaml`), validated by pydantic (`arducam_tof/config.py`, `TofConfig`). Unknown keys and out-of-range values are rejected and the process exits with status 1. An empty file gives the defaults.

| Key | Default | Allowed | Meaning |
|-----|---------|---------|---------|
| `camera_index` | `0` | 0..63 | Video device index the SDK opens over CSI (`/dev/video<N>`). Used only when `csi_device` is unset. |
| `csi_device` | unset | CSI controller name | When set, the node opens the `csi2_ch0` video node of that controller, found under `/sys/class/video4linux` at every open. The robot uses `1f00128000.csi` (Pi 5 cam1): the `/dev/video` number changes between kernels (video8 on 6.8.0-1051, video10 on 6.8.0-1065). |
| `range_mode` | `far` | `near`, `far` | `far` is 4 m (head camera looking across a room); `near` is 2 m with finer depth. The SDK gets millimetres; if it rejects them, metres are tried. |
| `confidence_threshold` | `30.0` | 0..65535 | Pixels with SDK confidence below this are published as depth 0. |
| `frame_id` | `tof_optical_frame` | non-empty | TF frame of every header. |
| `publish_rate_hz` | `10.0` | > 0, <= 30 | Upper bound of the publish rate. |
| `stamp_latency_ms` | `0.0` | 0..200 | Measured capture-to-return latency of the SDK frame in ms; subtracted from the frame-return time (see Timestamps). |
| `metrics_port` | unset | 1..65535 | Prometheus `/metrics` on 127.0.0.1; unset falls back to `METRICS_PORT`, then off. |

## Timestamps

Every frame's header stamp (shared by depth, amplitude and camera_info) is the node clock read immediately after the SDK `requestFrame` returns the frame, minus `stamp_latency_ms`. The reading is taken before the depth/amplitude conversions, so conversion and publish time never end up in the stamp. A result below zero is clamped to the reading itself and logged once. The SDK frame timestamp is not used: its clock domain is undocumented.

Why: the RGB camera stamps at the start of exposure, so the ToF stamp has to be the capture time, not the time Python got round to publishing, or depth pairs with the wrong RGB frame.

How the value is measured: pan-sweep the head and find the ToF-RGB time offset, as described in `docs/head-rgbd-fusion-methodology.md`. How it is set: `stamp_latency_ms` in the `arducam_tof` config in `ansible/group_vars/client.yml`; 0 (default) stamps with the frame-return time.

## Metrics

Node name `arducam_tof`, via `ros2-metrics`.

| Metric | Type | Meaning |
|--------|------|---------|
| `tof_frames_published_total` | counter | Frames published (depth, amplitude, camera_info) |
| `tof_frames_dropped_total{reason}` | counter | `error`, `no_frame`, `wrong_type` or `invalid`: requests that published nothing |
| `tof_camera_open_failures_total` | counter | Failed attempts to open and start the camera |
| `tof_camera_reopens_total` | counter | Close and reopen after 5 s without a valid frame |

## Code layout

- `arducam_tof/config.py` - pydantic `TofConfig`, `load_config()`, `config_path_from_env()`.
- `arducam_tof/bridge.py` - `TofBridge` (open, retry, publish), pure helpers `depth_to_mm()`, `amplitude_to_mono16()`, `image_msg()`, `camera_info_msg()`, and `run()`.
- `arducam_tof/__main__.py` - entry point, `python3 -m arducam_tof`.

## Dependencies

`numpy>=1.26,<2` (sensor_msgs needs it at runtime; the SDK needs numpy < 2), pydantic, pyyaml, prometheus-client, `ros2-metrics` (path dependency on `shared/ros2_metrics`). `ArducamDepthCamera` is declared for `sys_platform == 'linux' and platform_machine == 'aarch64'` only, so `uv sync` and the tests work on macOS.

## Tests

Run in this directory: `uv run pytest tests -q` and `uv run poe lint` (ruff and vulture).

- `tests/test_config.py` - defaults, every key, rejection of unknown keys and out-of-range values, file and YAML errors, `csi_device` resolution against a fake sysfs tree.
- `tests/test_bridge.py` - the SDK (`ArducamDepthCamera`) and rclpy are mocked: encodings, the confidence mask, one shared header, the rate bound, no publish on a failed, missing or malformed frame, open failure and retry, open with the index resolved from `csi_device` (an unresolvable one retries), release of a camera that fails after start, reopen after a stall, and `main()`.

## Pi 5: camera firmware

On the Pi 5 the RP1 CSI receiver drops every packet unless the sensor sends MIPI Frame Start/End packets (Arducam issue #121): `CSI2_DISCARDS_UNMATCHED` in `/sys/kernel/debug/rp1-cfe:1f00128000.csi/csi2_regs` climbs and the SDK never gets a frame. Reflashing the camera with Arducam's `B0410_Tof_Firmware.tar.gz` tool fixed it on the robot (2026-10-18; it still reports 0x10002): power off, bridge the two recovery pads, power on, remove the bridge, then `sudo ./firmware_update -i 11 -m b0410_tof_firmware` (i2c-11 = cam1; needs `libtinfo.so.5`), reboot.

## Deploy

The node is deployed on the client by Ansible as node type `arducam_tof`, **enabled** (`ansible/group_vars/client.yml`), scraped by Alloy at `127.0.0.1:19112`. The camera sits on Pi 5 CSI cam1; `ansible/playbooks/tasks/pi_camera_boot_config.yml` writes `dtoverlay=arducam-pivariety,cam1` (with `camera_auto_detect=0`) and reboots on change. Deploy:

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client arducam_tof 2>&1 | tee .logs/deploy-client.log
```

Never run `ansible-playbook` directly.
