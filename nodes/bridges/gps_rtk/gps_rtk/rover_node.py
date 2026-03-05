"""Rover node: LC29H-DA, NavSatFix publisher, RTCM3 TCP client to base."""

import logging
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from .config import GpsRtkConfig
from .nmea_parser import build_nav_sat_fix, parse_gga
from .serial_handler import SerialHandler

LOG = logging.getLogger(__name__)


class GpsRtkRoverNode(Node):
    """ROS2 node for GPS RTK rover: serial in/out, NavSatFix out, RTCM3 TCP client."""

    def __init__(self, config: GpsRtkConfig) -> None:
        super().__init__("gps_rtk_rover")
        self.config = config
        self._latest_fix: NavSatFix | None = None
        self._fix_lock = threading.Lock()
        self.pub = self.create_publisher(NavSatFix, config.topic, 10)
        self.serial: SerialHandler | None = None
        self._rtcm_socket: socket.socket | None = None
        self._rtcm_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._rtcm_connected = threading.Event()
        self._rtcm_rx_bytes = 0
        self._rtcm_wx_bytes = 0
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
            )
            with self._fix_lock:
                self._latest_fix = msg
                self._latest_quality = parsed["quality"]
        except Exception as e:
            self.get_logger().debug("NavSatFix build error: %s", e)

    def _on_rtcm3(self, _frame: bytes) -> None:
        pass

    def _rtcm_client_loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10.0)
                sock.connect((self.config.rtcm_server_host, self.config.rtcm_server_port))
                self._rtcm_socket = sock
                self._rtcm_connected.set()
                self.get_logger().info(
                    f"RTCM connected to {self.config.rtcm_server_host}:{self.config.rtcm_server_port}"
                )
                while not self._stop.is_set() and self.serial is not None:
                    try:
                        data = sock.recv(4096)
                        if not data:
                            break
                        self._rtcm_rx_bytes += len(data)
                        self.serial.write(data)
                        self._rtcm_wx_bytes += len(data)
                    except (ConnectionResetError, BrokenPipeError, OSError) as e:
                        self.get_logger().warning(f"RTCM connection lost: {e}")
                        break
            except (socket.timeout, socket.error, OSError) as e:
                self.get_logger().debug(f"RTCM connect failed: {e}")
            finally:
                self._rtcm_connected.clear()
                if self._rtcm_socket is not None:
                    try:
                        self._rtcm_socket.close()
                    except OSError:
                        pass
                    self._rtcm_socket = None
            if not self._stop.is_set():
                time.sleep(self.config.rtcm_reconnect_interval_s)

    def run(self) -> None:
        """Open serial, start RTCM client thread, run spin."""
        self.serial = SerialHandler(
            port=self.config.serial_port,
            baud_rate=self.config.baud_rate,
            on_nmea=self._on_nmea,
            on_rtcm3=self._on_rtcm3,
            log_unknown_bytes=self.get_logger().get_effective_level() <= logging.DEBUG,
        )
        self.serial.open()

        self._rtcm_thread = threading.Thread(target=self._rtcm_client_loop, daemon=True)
        self._rtcm_thread.start()

        period_ns = int(1e9 / max(0.1, self.config.publish_hz))
        self._timer = self.create_timer(period_ns / 1e9, self._publish_cb)
        rclpy.spin(self)

    def _publish_cb(self) -> None:
        with self._fix_lock:
            msg = self._latest_fix
            quality = getattr(self, "_latest_quality", None)
        if msg is not None:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
        self._diag_counter += 1
        if self._diag_counter % 100 == 0:
            connected = self._rtcm_connected.is_set()
            self.get_logger().info(
                f"[diag] gga_q={quality} rtcm_rx={self._rtcm_rx_bytes}B wx={self._rtcm_wx_bytes}B tcp={'UP' if connected else 'DOWN'}"
            )

    def shutdown(self) -> None:
        """Close serial and RTCM socket."""
        self._stop.set()
        if self._rtcm_socket is not None:
            try:
                self._rtcm_socket.close()
            except OSError:
                pass
            self._rtcm_socket = None
        if self.serial is not None:
            self.serial.close()
