---
name: d435-diagnostics
description: Use when checking the RealSense D435 head camera on the client - USB link, firmware, PSU/USB current, driver errors, /head/d435 topics and stream rates - or after deploying the realsense_d435 node
---

# RealSense D435 Diagnostics

Run `scripts/d435_diag.sh` for every D435 live check. Never SSH manually and never start rclpy or `ros2` CLI probes on
the client (an ad-hoc DDS participant there broke another node's discovery). The script is read-only: SSH runs only
`systemctl`, `journalctl`, `lsusb`, sysfs and device-tree reads, `vcgencmd`, `curl` to the local topic_scraper_api, and
with `--device` `rs-enumerate-devices`.

## Quick Reference

```bash
./scripts/d435_diag.sh                          # Service, USB, firmware, PSU, error counts (last 30 min), journal tail
./scripts/d435_diag.sh --since "2 hours ago"    # Error counts over another window (journalctl --since syntax)
./scripts/d435_diag.sh --lines 40               # More journal lines (default: 20)
./scripts/d435_diag.sh --topics                 # /head/d435/* topics and types (topic_scraper_api)
./scripts/d435_diag.sh --rates --interval 5     # camera_info rates from header stamps over 5 s (default 10)
./scripts/d435_diag.sh --device                 # rs-enumerate-devices -s / -c; only while the unit is stopped
./scripts/d435_diag.sh --watch --interval 30    # Repeat the chosen mode
```

Env: `D435_CLIENT_HOST` (default `client.ros2.lan`), `D435_SSH_USER`, `SCRAPER_PORT` (default 18100).

`--topics` and `--rates` need topic_scraper_api, which is disabled by default: set `enabled: true` for it in
`ansible/group_vars/client.yml` and run `scripts/deploy-nodes.sh client topic_scraper_api`. Without it they exit 2 with
that hint. Set it back to `enabled: false` afterwards (it costs CPU).

## Output Sections (default mode)

| Section | What to look for |
|---------|-----------------|
| Service | `ros2-realsense_d435: active` = driver running. `inactive` / `failed` = read the journal tail. Empty = client unreachable. |
| USB | `lsusb` shows `8086:0b07 ... RealSense D435`. `sysfs <port> speed: 5000 Mb/s (USB 3, ok)`. `480` = USB 2 link: wrong port (use a blue USB 3 port), cable, or plugged in slowly; re-plug the cable into the camera first. |
| Firmware | `Device FW version: X` and `Device USB type: 3.2` from the driver's latest start. The design asks for firmware >= 5.17.3.10. No lines = the unit has not started since the journal was rotated. |
| PSU / USB current | `usb_max_current_enable=1` lifts the Pi 5 USB budget to 1.6 A. `PSU max current (firmware)` is what the supply negotiated; below 5000 mA the Pi caps USB at 600 mA unless `usb_max_current_enable=1` is set (a D435 at full load can exceed 600 mA). `usb_over_current_detected: 1` = the port tripped. A `5A` journal line is printed when the firmware logged one. |
| Errors | Counts over `--since`. Any `VIDIOC_QBUF` or `No such device` = USB resets / the camera dropped off the bus. `Frame didn't arrive` = frame timeouts (bandwidth, power or a stalled stream). All three should stay 0 over 30 min of streaming. |
| Journal | The last `--lines` lines of the unit. |

## --topics

One line per `/head/d435/` topic: `topic  type  sample=true|false`. No lines = the driver is not running or the scraper
has not discovered it yet (discovery runs every 2 s). Only allowed types are listed (no raw `sensor_msgs/msg/Image`).

## --rates

One line per `/head/d435/*/camera_info`: `<rate> Hz (<frames> frames over <span> s, <polls> polls)`. The driver
publishes camera_info once per frame, so this is the stream rate. The scraper keeps only the latest sample, so the
script polls it on the client and counts distinct header stamps. Expect the configured fps (e.g. 30.0 Hz) on every
stream; colour and depth must match for hardware sync.

- `(lower bound: every poll saw a new frame ...)` = polling was not faster than the stream; the real rate may be higher.
- `no sample` = the scraper has no message for that topic (stream disabled, driver down, or CameraInfo missing from
  `allowed_types`).

## --device

Runs `rs-enumerate-devices -s` (name, serial, firmware) and `-c` (per-stream intrinsics and the extrinsics, including
`Extrinsic from "Depth" To "Color"`: rotation matrix and translation in metres; the D435 baseline to colour is about
0.015 m on x). It refuses (exit 1) while `ros2-realsense_d435` is active, because the driver holds the camera; stop the
unit only when nobody is using the camera, and start it again afterwards.

## Common Patterns

| Symptom | Cause | Fix |
|---------|-------|-----|
| speed `480` | USB 2 link | Blue USB 3 port, short USB 3 cable, plug into the camera first |
| `VIDIOC_QBUF` / `No such device` counts growing | Camera re-enumerating (power or cable) | Check PSU section, cable; consider lower fps |
| Rates below the configured fps | CPU or USB bandwidth | Check the unit's CPU in Grafana, the error counts, the USB speed |
| `--device` refuses | Unit active | Expected; stop the unit first if the check is really needed |
| `--topics` exit 2 | topic_scraper_api disabled | Enable and deploy it as above |
