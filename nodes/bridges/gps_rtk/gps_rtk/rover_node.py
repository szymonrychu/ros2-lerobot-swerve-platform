"""Rover node: LC29H-DA, NavSatFix publisher, NTRIP v1 client to base."""

import collections
import logging
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from .config import GpsRtkConfig
from .nmea_parser import build_nav_sat_fix, drift_from_mean, parse_gga, quality_label
from .ntrip_client import NtripClient
from .serial_handler import SerialHandler

LOG = logging.getLogger(__name__)

POSITION_HISTORY_LEN = 10
DIAG_INTERVAL_TICKS = 100  # at 10 Hz → every 10 s


class GpsRtkRoverNode(Node):
    """ROS2 node for GPS RTK rover: serial in/out, NavSatFix out, NTRIP v1 client."""

    def __init__(self, config: GpsRtkConfig) -> None:
        super().__init__("gps_rtk_rover")
        self.config = config
        self._latest_fix: NavSatFix | None = None
        self._latest_gga: dict[str, Any] | None = None
        self._latest_gga_sentence: str | None = None
        self._fix_lock = __import__("threading").Lock()
        self._position_history: collections.deque[tuple[float, float, float]] = collections.deque(
            maxlen=POSITION_HISTORY_LEN
        )

        self.pub = self.create_publisher(NavSatFix, config.topic, 10)
        self.serial: SerialHandler | None = None
        self._ntrip: NtripClient | None = None
        self._diag_counter = 0

    def _on_nmea(self, sentence: str) -> None:
        if "GGA" not in sentence:
            return
        parsed = parse_gga(sentence)
        if parsed is None:
            return
        try:
            msg = build_nav_sat_fix(
                lat=parsed["latitude"],
                lon=parsed["longitude"],
                alt=parsed["altitude"],
                status=parsed["status"],
                frame_id=self.config.frame_id,
                hdop=parsed.get("hdop"),
            )
            with self._fix_lock:
                self._latest_fix = msg
                self._latest_gga = parsed
                self._latest_gga_sentence = sentence
                self._position_history.append((parsed["latitude"], parsed["longitude"], parsed["altitude"]))
        except Exception as e:
            self.get_logger().debug("NavSatFix build error: %s", e)

    def _on_rtcm3(self, _frame: bytes) -> None:
        pass

    def _get_gga_for_ntrip(self) -> str | None:
        """Return the latest GGA sentence for the NTRIP caster position report."""
        with self._fix_lock:
            return self._latest_gga_sentence

    def _on_ntrip_data(self, data: bytes) -> None:
        """Write RTCM3 data received from NTRIP caster to the rover's serial port."""
        if self.serial is not None:
            try:
                self.serial.write(data)
            except (RuntimeError, OSError) as e:
                self.get_logger().debug("Serial write error (NTRIP data): %s", e)

    def run(self) -> None:
        """Open serial, start NTRIP client thread, run spin."""
        self.serial = SerialHandler(
            port=self.config.serial_port,
            baud_rate=self.config.baud_rate,
            on_nmea=self._on_nmea,
            on_rtcm3=self._on_rtcm3,
            log_unknown_bytes=self.get_logger().get_effective_level() <= logging.DEBUG,
        )
        self.serial.open()

        self._ntrip = NtripClient(
            host=self.config.rtcm_server_host,
            port=self.config.rtcm_server_port,
            mountpoint=self.config.ntrip_mountpoint,
            user=self.config.ntrip_user,
            password=self.config.ntrip_password,
            on_data=self._on_ntrip_data,
            get_gga=self._get_gga_for_ntrip,
            gga_interval_s=self.config.ntrip_gga_interval_s,
            reconnect_interval_s=self.config.rtcm_reconnect_interval_s,
            logger=self.get_logger(),
        )
        self._ntrip.start()

        period_ns = int(1e9 / max(0.1, self.config.publish_hz))
        self._timer = self.create_timer(period_ns / 1e9, self._publish_cb)
        rclpy.spin(self)

    def _publish_cb(self) -> None:
        with self._fix_lock:
            msg = self._latest_fix
            gga = self._latest_gga
            history = list(self._position_history)
        if msg is not None:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
        self._diag_counter += 1
        if self._diag_counter % DIAG_INTERVAL_TICKS == 0:
            self._log_diag(gga, history)

    def _log_diag(self, gga: dict[str, Any] | None, history: list[tuple[float, float, float]]) -> None:
        ntrip_status = "UP" if (self._ntrip is not None and self._ntrip.is_connected) else "DOWN"
        rx_bytes = self._ntrip.rx_bytes if self._ntrip is not None else 0

        if gga is None:
            self.get_logger().info(f"[diag] no_fix | ntrip_rx={rx_bytes}B ntrip={ntrip_status}")
            return

        q = gga["quality"]
        label = quality_label(q)
        sats = gga.get("num_satellites")
        hdop = gga.get("hdop")
        diff_age = gga.get("diff_age_s")
        drift = drift_from_mean(history)

        parts = [f"fix={label}(q{q})"]
        parts.append(f"sats={sats}" if sats is not None else "sats=?")
        parts.append(f"hdop={hdop:.2f}" if hdop is not None else "hdop=?")
        if diff_age is not None:
            parts.append(f"age={diff_age:.1f}s")
        if drift is not None:
            parts.append(f"drift2d={drift[0]:.2f}m")
            parts.append(f"drift3d={drift[1]:.2f}m")
        parts.append(f"ntrip_rx={rx_bytes}B")
        parts.append(f"ntrip={ntrip_status}")

        self.get_logger().info(f"[diag] {' | '.join(parts)}")

    def shutdown(self) -> None:
        """Close NTRIP client and serial."""
        if self._ntrip is not None:
            self._ntrip.stop()
            self._ntrip = None
        if self.serial is not None:
            self.serial.close()
