# GPS RTK bridge (gps_rtk)

ROS2 node for LC29H-BS (base station) and LC29H-DA (rover) over **native RPi UART** (hat). Publishes `sensor_msgs/NavSatFix` and streams RTCM3 corrections over TCP. Handles mixed NMEA + RTCM3 + proprietary binary on the serial line.

## Modes

- **base** — Server with LC29H(BS): configures module, reads NMEA+RTCM3 from serial, publishes `/server/gps/fix`, serves RTCM3 on TCP (default port 5016).
- **rover** — Client with LC29H(DA): connects to base TCP, forwards RTCM3 to serial, reads NMEA, publishes `/client/gps/fix` (RTK fix when corrections flow).

## Config

YAML config path: `GPS_RTK_CONFIG` or `/etc/ros2/gps_rtk/config.yaml`.

- `mode`: `base` | `rover`
- `serial_port`: e.g. `/dev/ttyAMA0` (RPi 4 with disable-bt) or `/dev/ttyS0` (legacy)
- `baud_rate`: default `115200`
- `topic`: e.g. `/server/gps/fix`, `/client/gps/fix`
- `frame_id`: default `gps_link`
- `publish_hz`: default `10.0`
- `configure_on_start`: send LC29H-BS configure commands on startup (base)
- `rtcm_tcp_port`, `rtcm_tcp_bind`: base RTCM server
- `rtcm_server_host`, `rtcm_server_port`, `rtcm_reconnect_interval_s`: rover RTCM client
- `status_topic`: optional `std_msgs/String` topic for the compact JSON status below (default `null` = disabled)
- `status_hz`: status publish rate, must be > 0 (default `1.0`)

## Status topic

With `status_topic` set, both modes publish compact JSON (`std_msgs/String`) at `status_hz`. Nothing is published until a real GGA sentence has been parsed. Builders live in `gps_rtk/status.py` (pure functions, unit-tested in `tests/test_status.py`).

Fix labels: quality 0 `No fix`, 1 `GPS`, 2 `DGPS`, 4 `RTK Fixed`, 5 `RTK Float`, 6 `Dead reckoning`, anything else `Unknown`.

Rover:

```json
{"role":"rover","quality":4,"fix":"RTK Fixed","num_satellites":18,"hdop":0.7,"diff_age_s":1.0,"ntrip_connected":true,"ntrip_rx_bytes":123456}
```

Base:

```json
{"role":"base","latitude":52.1,"longitude":21.0,"altitude":110.5,"quality":4,"fix":"RTK Fixed","num_satellites":18,"hdop":0.7,"ntrip_clients":1,"rtcm_tx_frames":900,"rtcm_tx_bytes":123456,"rtcm_types":[1005,1074,1084]}
```

The base status carries the base antenna position (`latitude`/`longitude` in degrees, `altitude` in m) from its GGA; a GGA without a position does not parse, so no status is published for it. The web UI draws the base marker from it. The rover status carries no position.

`num_satellites`, `hdop` and `diff_age_s` are `null` when the GGA sentence did not carry them.

## Serial and binary stream

On RPi native UART the LC29H can emit proprietary binary alongside NMEA and RTCM3. The node uses a byte-level parser: NMEA lines (`$...*XX\r\n`) and RTCM3 frames (0xD3 + length + payload + CRC24Q) are extracted; other bytes are discarded.

## Calibration (base)

One-time survey-in is done with `scripts/calibrate_rtk_base.py` on the server (see repo root). After calibration, the base position is stored in the module; the node only sends the per-boot configure commands.

**From your computer:** use `scripts/rtk_calibrate.sh` to run the full workflow over SSH (stops base service, runs survey-in, then instructs power-cycle and restart). See [scripts/README.md](../../../scripts/README.md#gps-rtk) for calibration, verification, and status scripts.

## Deploy

Ansible deploys this node on server (base) and client (rover). Serial device: on RPi 4 (server) with `dtoverlay=disable-bt` the LC29H-BS hat uses `/dev/ttyAMA0` (PL011 UART on GPIO 14/15); on RPi 5 (client) the LC29H-DA hat uses `/dev/ttyAMA0` (requires `dtoverlay=uart0-pi5` in boot config, added by the deploy playbook). Ensure UART is enabled on the host (e.g. `enable_uart=1` in boot config). Using `/dev/ttyS0` on RPi 4 with disable-bt causes "Input/output error" because the primary UART is ttyAMA0. The topic scraper's `allowed_types` must include `sensor_msgs/msg/NavSatFix` to observe GPS topics.
