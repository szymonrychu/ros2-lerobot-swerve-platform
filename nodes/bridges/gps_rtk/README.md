# GPS RTK bridge (gps_rtk)

ROS2 node for LC29H-BS (base station) and LC29H-DA (rover) over **native RPi UART** (hat). Publishes `sensor_msgs/NavSatFix` and streams RTCM3 corrections over TCP. Handles mixed NMEA + RTCM3 + proprietary binary on the serial line.

## Modes

- **base** — Server with LC29H(BS): configures module, reads NMEA+RTCM3 from serial, publishes `/server/gps/fix`, serves RTCM3 on TCP (default port 5016).
- **rover** — Client with LC29H(DA): connects to base TCP, forwards RTCM3 to serial, reads NMEA, publishes `/client/gps/fix` (RTK fix when corrections flow).

## Config

YAML config path: `GPS_RTK_CONFIG` or `/etc/ros2/gps_rtk/config.yaml`.

- `mode`: `base` | `rover`
- `serial_port`: e.g. `/dev/ttyS0`
- `baud_rate`: default `115200`
- `topic`: e.g. `/server/gps/fix`, `/client/gps/fix`
- `frame_id`: default `gps_link`
- `publish_hz`: default `10.0`
- `configure_on_start`: send LC29H-BS configure commands on startup (base)
- `rtcm_tcp_port`, `rtcm_tcp_bind`: base RTCM server
- `rtcm_server_host`, `rtcm_server_port`, `rtcm_reconnect_interval_s`: rover RTCM client

## Serial and binary stream

On RPi native UART the LC29H can emit proprietary binary alongside NMEA and RTCM3. The node uses a byte-level parser: NMEA lines (`$...*XX\r\n`) and RTCM3 frames (0xD3 + length + payload + CRC24Q) are extracted; other bytes are discarded.

## Calibration (base)

One-time survey-in is done with `scripts/calibrate_rtk_base.py` on the server (see repo root). After calibration, the base position is stored in the module; the node only sends the per-boot configure commands.

## Deploy

Ansible deploys this node on server (base) and client (rover) with `--device=/dev/ttyS0:/dev/ttyS0`. Ensure UART is enabled on the host (e.g. `enable_uart=1` in boot config).
