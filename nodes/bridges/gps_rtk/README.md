# GPS RTK bridge (gps_rtk)

ROS2 node for LC29H-BS (base station) and LC29H-DA (rover) over **native RPi UART** (hat). Publishes `sensor_msgs/NavSatFix` and streams RTCM3 corrections over TCP. Handles mixed NMEA + RTCM3 + proprietary binary on the serial line.

## Modes

- **base** - Server with LC29H(BS): configures module, reads NMEA+RTCM3 from serial, publishes `/server/gps/fix`, serves RTCM3 on TCP (default port 5016) and its status JSON on `GET /status` of the same port.
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
- `nmea_timeout_s`: rover only, seconds without any NMEA sentence before the status says the receiver is silent, must be > 0 (default `3.0`; keep it plus one status period below web_ui `stale_after_s`)
- `metrics_port`: rover only, Prometheus `/metrics` port on 127.0.0.1 (robot: `19104`); unset falls back to env `METRICS_PORT`, then disabled. The base never serves metrics.

## Metrics

Rover only, node name `gps_rtk_rover`, served via `ros2-metrics` on `127.0.0.1:<metrics_port>`.

| Metric | Type | Meaning |
|--------|------|---------|
| `gps_fix_quality` | gauge | GGA fix quality code (4 RTK fixed, 5 RTK float) |
| `gps_satellites` | gauge | Satellites used (NaN when not reported) |
| `gps_hdop` | gauge | HDOP (NaN when not reported) |
| `gps_diff_age_seconds` | gauge | Age of differential corrections (NaN when not reported) |
| `gps_ntrip_connected` | gauge | 1 while the NTRIP client is connected |
| `gps_ntrip_rx_bytes_total` | counter | RTCM bytes received from the caster |
| `gps_ntrip_reconnects_total` | counter | NTRIP reconnect attempts |
| `gps_serial_errors_total{op}` | counter | Serial errors, `op` is `open` or `read` |
| `gps_fixes_published_total` | counter | NavSatFix messages published |

## Status topic

With `status_topic` set, both modes publish compact JSON (`std_msgs/String`) at `status_hz`. Builders live in `gps_rtk/status.py` (pure functions, unit-tested in `tests/test_status.py`).

The status is built from **every** GGA sentence, also one without a fix: indoors the LC29H sends `$GNGGA,...,,,,,0,07,3.5,...` (quality 0, empty latitude/longitude), and the status then reports `No fix` with the satellites and HDOP the receiver reports. That GGA is read by `parse_gga_status` (`nmea_parser.py`, receiver-state fields only); `parse_gga` still needs a position, so a `NavSatFix` is never published without one.

Fix labels: quality 0 `No fix`, 1 `GPS`, 2 `DGPS`, 4 `RTK Fixed`, 5 `RTK Float`, 6 `Dead reckoning`, anything else `Unknown`.

Rover (published while the last GGA is at most `nmea_timeout_s` old):

```json
{"role":"rover","quality":0,"fix":"No fix","num_satellites":7,"hdop":3.5,"diff_age_s":null,"ntrip_connected":true,"ntrip_rx_bytes":123456,"nmea_silent":false,"nmea_received":true,"nmea_age_s":0.2}
```

When no NMEA sentence of any kind arrived for `nmea_timeout_s` (receiver unpowered, UART wrong, module hung), the rover publishes the silent status instead: the fix fields are `null` (unknown, not zero), `nmea_age_s` is the seconds since the last sentence, or since node start with `nmea_received: false` when none ever arrived. The NTRIP link state and rx bytes are real either way.

```json
{"role":"rover","quality":null,"fix":"Receiver silent","num_satellites":null,"hdop":null,"diff_age_s":null,"ntrip_connected":true,"ntrip_rx_bytes":123456,"nmea_silent":true,"nmea_received":true,"nmea_age_s":12.3}
```

Nothing is published during the first `nmea_timeout_s` after start, or while NMEA arrives but no GGA did within `nmea_timeout_s`. The rover metrics (`gps_fix_quality`, `gps_satellites`, `gps_hdop`) are also set from every GGA, so a no-fix receiver shows its real satellite count.

Base:

```json
{"role":"base","latitude":52.1,"longitude":21.0,"altitude":110.5,"quality":4,"fix":"RTK Fixed","num_satellites":18,"hdop":0.7,"ntrip_clients":1,"rtcm_tx_frames":900,"rtcm_tx_bytes":123456,"rtcm_types":[1005,1074,1084],"ref_station_position":true,"survey_state":"fixed_position","gga_seq":4711}
```

The base status carries the base antenna position (`latitude`/`longitude` in degrees, `altitude` in m) from its GGA; after a GGA without a position the keys are absent (never zero). The web UI draws the base marker from it. The rover status carries no position.

- `survey_state`: base calibration (survey-in) state found at configure time (`serial_handler.py`): `fixed_position` (stored survey-in ECEF found, fixed-position mode re-activated), `uncalibrated` (no stored ECEF: run `scripts/rtk_calibrate.sh`), `unreadable` (PQTMCFGSVIN answer not parseable), `configure_failed` (configure commands could not be written), `not_checked` (`configure_on_start: false`).
- `ref_station_position`: `true` once RTCM 1005/1006 (reference station position, needed by the rover for RTK) was seen.
- `gga_seq`: number of GGA sentences received since start; a poller that sees it unchanged knows the base stopped receiving NMEA.

`num_satellites`, `hdop` and `diff_age_s` are `null` when the GGA sentence did not carry them.

## Base status over HTTP (`GET /status`)

Server topics are not visible on the client over DDS, so the base serves the same status object on its NTRIP caster port: `GET /status HTTP/1.1` on `rtcm_tcp_port` (5016) answers `200` with `Content-Type: application/json` and the JSON above, then closes; before the first GGA it answers `503` with `{"error":"no status yet"}`. The route needs no NTRIP auth (read-only status on the LAN) and never joins the RTCM broadcast. NTRIP behaviour is unchanged: `GET /rtk` gets `ICY 200 OK` (or `401` with wrong credentials), any other path, including `GET /` (source-table request), gets the bare `404`. The web UI base chip polls it (`base_url: http://server.ros2.lan:5016/status`).

```bash
curl -s http://server.ros2.lan:5016/status
```

## Serial and binary stream

On RPi native UART the LC29H can emit proprietary binary alongside NMEA and RTCM3. The node uses a byte-level parser: NMEA lines (`$...*XX\r\n`) and RTCM3 frames (0xD3 + length + payload + CRC24Q) are extracted; other bytes are discarded.

## Calibration (base)

One-time survey-in is done with `scripts/calibrate_rtk_base.py` on the server (see repo root). After calibration, the base position is stored in the module; the node only sends the per-boot configure commands.

**From your computer:** use `scripts/rtk_calibrate.sh` to run the full workflow over SSH (stops base service, runs survey-in, then instructs power-cycle and restart). See [scripts/README.md](../../../scripts/README.md#gps-rtk) for calibration, verification, and status scripts.

## Deploy

Ansible deploys this node on server (base) and client (rover). Serial device: on RPi 4 (server) with `dtoverlay=disable-bt` the LC29H-BS hat uses `/dev/ttyAMA0` (PL011 UART on GPIO 14/15); on RPi 5 (client) the LC29H-DA hat uses `/dev/ttyAMA0` (requires `dtoverlay=uart0-pi5` in boot config, added by the deploy playbook). Ensure UART is enabled on the host (e.g. `enable_uart=1` in boot config). Using `/dev/ttyS0` on RPi 4 with disable-bt causes "Input/output error" because the primary UART is ttyAMA0. The topic scraper's `allowed_types` must include `sensor_msgs/msg/NavSatFix` to observe GPS topics.
