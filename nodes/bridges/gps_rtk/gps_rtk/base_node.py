"""Base station node: LC29H-BS, NavSatFix publisher, NTRIP v1 caster."""

import collections
import logging
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from .config import GpsRtkConfig
from .nmea_parser import build_nav_sat_fix, drift_from_mean, parse_gga, parse_gga_status, quality_label
from .ntrip_caster import NtripCaster
from .rtcm3 import parse_rtcm3_message_type
from .serial_handler import SURVEY_NOT_CHECKED, SerialHandler
from .status import base_status, status_json

LOG = logging.getLogger(__name__)

POSITION_HISTORY_LEN = 10
DIAG_INTERVAL_TICKS = 100  # at 10 Hz → every 10 s

# Thresholds for no_fix escalation (in publish ticks, not diag ticks)
NO_FIX_WARN_TICKS = 3000  # ~5 min at 10 Hz
NO_FIX_ERROR_TICKS = 12000  # ~20 min at 10 Hz

# How many RTCM frames to observe before warning about missing 1005/1006
RTCM_TYPE_CHECK_FRAMES = 80  # ~10 s worth


class GpsRtkBaseNode(Node):
    """ROS2 node for GPS RTK base: serial in, NavSatFix out, NTRIP v1 caster."""

    def __init__(self, config: GpsRtkConfig) -> None:
        super().__init__("gps_rtk_base")
        self.config = config
        self._latest_fix: NavSatFix | None = None
        self._latest_gga: dict[str, Any] | None = None
        # Receiver state of the latest GGA (with position when it had one) and the GGA count, for the status.
        self._latest_gga_state: dict[str, Any] | None = None
        self._gga_seq = 0
        self._fix_lock = threading.Lock()
        self._position_history: collections.deque[tuple[float, float, float]] = collections.deque(
            maxlen=POSITION_HISTORY_LEN
        )

        self._no_fix_ticks: int = 0
        self._rtcm_frames_seen: int = 0
        self._rtcm_types_seen: set[int] = set()
        self._ref_station_warned: bool = False
        self._diag_counter = 0

        self._caster: NtripCaster | None = None
        self.pub = self.create_publisher(NavSatFix, config.topic, 10)
        self.serial: SerialHandler | None = None
        self._status_pub = self.create_publisher(String, config.status_topic, 10) if config.status_topic else None

    def _on_nmea(self, sentence: str) -> None:
        if "GGA" not in sentence:
            return
        state = parse_gga_status(sentence)
        if state is None:
            return
        parsed = parse_gga(sentence)
        with self._fix_lock:
            self._latest_gga_state = parsed if parsed is not None else state
            self._gga_seq += 1
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
                self._position_history.append((parsed["latitude"], parsed["longitude"], parsed["altitude"]))
        except Exception as e:
            self.get_logger().debug(f"NavSatFix build error: {e}")

    def _on_rtcm3(self, frame: bytes) -> None:
        self._rtcm_frames_seen += 1
        msg_type = parse_rtcm3_message_type(frame)
        if msg_type is not None:
            with self._fix_lock:
                self._rtcm_types_seen.add(msg_type)
        if self._caster is not None:
            self._caster.broadcast_rtcm(frame)

    def run(self) -> None:
        """Open serial, optionally configure BS, start NTRIP caster, run spin."""
        self._caster = NtripCaster(
            bind=self.config.rtcm_tcp_bind,
            port=self.config.rtcm_tcp_port,
            mountpoint=self.config.ntrip_mountpoint,
            user=self.config.ntrip_user,
            password=self.config.ntrip_password,
            logger=self.get_logger(),
            status_provider=self.current_status,
        )
        self._caster.start()

        self.serial = SerialHandler(
            port=self.config.serial_port,
            baud_rate=self.config.baud_rate,
            on_nmea=self._on_nmea,
            on_rtcm3=self._on_rtcm3,
            log_unknown_bytes=self.get_logger().get_effective_level() <= logging.DEBUG,
        )
        self.serial.open()
        if self.config.configure_on_start:
            self.get_logger().info("Sending LC29H-BS configure commands (NMEA + RTCM + save)")
            self.serial.send_base_configure()

        period_ns = int(1e9 / max(0.1, self.config.publish_hz))
        self._timer = self.create_timer(period_ns / 1e9, self._publish_cb)
        if self._status_pub is not None:
            self._status_timer = self.create_timer(1.0 / self.config.status_hz, self._publish_status)
        rclpy.spin(self)

    def current_status(self) -> dict[str, Any] | None:
        """Build the base status from the latest GGA (also a no-fix one) and the caster state.

        Called from the status timer and from the caster's GET /status thread.

        Returns:
            dict[str, Any] | None: The status published on status_topic and served on GET /status, or None
                until a GGA has been received.
        """
        with self._fix_lock:
            gga = self._latest_gga_state
            seq = self._gga_seq
            rtcm_types = set(self._rtcm_types_seen)
        if gga is None:
            return None
        caster = self._caster
        return base_status(
            gga,
            ntrip_clients=caster.client_count if caster else 0,
            rtcm_tx_frames=caster.tx_frames if caster else 0,
            rtcm_tx_bytes=caster.tx_bytes if caster else 0,
            rtcm_types=rtcm_types,
            survey_state=self.serial.survey_state if self.serial is not None else SURVEY_NOT_CHECKED,
            gga_seq=seq,
        )

    def _publish_status(self) -> None:
        """Publish compact JSON status; skipped until a GGA has been received."""
        status = self.current_status()
        if status is None or self._status_pub is None:
            return
        self._status_pub.publish(String(data=status_json(status)))

    def _publish_cb(self) -> None:
        with self._fix_lock:
            msg = self._latest_fix
            gga = self._latest_gga
            history = list(self._position_history)

        if msg is not None:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            self._no_fix_ticks = 0
        else:
            self._no_fix_ticks += 1

        self._diag_counter += 1
        if self._diag_counter % DIAG_INTERVAL_TICKS == 0:
            self._log_diag(gga, history)

    def _log_diag(self, gga: dict[str, Any] | None, history: list[tuple[float, float, float]]) -> None:
        n_clients = self._caster.client_count if self._caster else 0
        tx_frames = self._caster.tx_frames if self._caster else 0
        tx_bytes = self._caster.tx_bytes if self._caster else 0

        # Check for missing reference station position message
        if (
            not self._ref_station_warned
            and self._rtcm_frames_seen >= RTCM_TYPE_CHECK_FRAMES
            and not (self._rtcm_types_seen & {1005, 1006})
        ):
            self.get_logger().warning(
                "No RTCM 1005/1006 (reference station position) seen after %d frames — "
                "base may need re-calibration (run rtk_calibrate.sh)",
                self._rtcm_frames_seen,
            )
            self._ref_station_warned = True

        if gga is None:
            no_fix_min = self._no_fix_ticks / (self.config.publish_hz * 60)
            msg = (
                f"[diag] no_fix | rtcm_tx={tx_frames}f/{tx_bytes}B "
                f"types={sorted(self._rtcm_types_seen)} clients={n_clients}"
            )
            if self._no_fix_ticks >= NO_FIX_ERROR_TICKS:
                self.get_logger().error(
                    f"{msg} | no fix for {no_fix_min:.0f} min — consider re-running calibration (run rtk_calibrate.sh)"
                )
            elif self._no_fix_ticks >= NO_FIX_WARN_TICKS:
                self.get_logger().warning(f"{msg} | no fix for {no_fix_min:.0f} min")
            else:
                self.get_logger().info(msg)
            return

        q = gga["quality"]
        label = quality_label(q)
        sats = gga.get("num_satellites")
        hdop = gga.get("hdop")
        drift = drift_from_mean(history)

        parts = [f"fix={label}(q{q})"]
        parts.append(f"sats={sats}" if sats is not None else "sats=?")
        parts.append(f"hdop={hdop:.2f}" if hdop is not None else "hdop=?")
        if drift is not None:
            parts.append(f"drift2d={drift[0]:.2f}m")
            parts.append(f"drift3d={drift[1]:.2f}m")
        parts.append(f"rtcm_tx={tx_frames}f/{tx_bytes}B")
        parts.append(f"types={sorted(self._rtcm_types_seen)}")
        parts.append(f"clients={n_clients}")

        self.get_logger().info(f"[diag] {' | '.join(parts)}")

    def shutdown(self) -> None:
        """Close serial and NTRIP caster."""
        if self._caster is not None:
            self._caster.stop()
            self._caster = None
        if self.serial is not None:
            self.serial.close()
