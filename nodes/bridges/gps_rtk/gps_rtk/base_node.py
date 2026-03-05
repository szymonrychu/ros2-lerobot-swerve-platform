"""Base station node: LC29H-BS, NavSatFix publisher, RTCM3 TCP server."""

import logging
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from .config import GpsRtkConfig
from .nmea_parser import build_nav_sat_fix, parse_gga
from .serial_handler import SerialHandler

LOG = logging.getLogger(__name__)


class GpsRtkBaseNode(Node):
    """ROS2 node for GPS RTK base: serial in, NavSatFix out, RTCM3 TCP server."""

    def __init__(self, config: GpsRtkConfig) -> None:
        super().__init__("gps_rtk_base")
        self.config = config
        self._latest_fix: NavSatFix | None = None
        self._fix_lock = threading.Lock()
        self._rtcm_clients: list[socket.socket] = []
        self._rtcm_lock = threading.Lock()
        self._rtcm_queue: list[bytes] = []
        self.pub = self.create_publisher(NavSatFix, config.topic, 10)
        self.serial: SerialHandler | None = None
        self._server_socket: socket.socket | None = None
        self._server_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._rtcm_tx_frames = 0
        self._rtcm_tx_bytes = 0
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
            self.get_logger().debug(f"NavSatFix build error: {e}")

    def _on_rtcm3(self, frame: bytes) -> None:
        with self._rtcm_lock:
            self._rtcm_queue.append(frame)
            n_clients = len(self._rtcm_clients)
            for sock in self._rtcm_clients[:]:
                try:
                    sock.sendall(frame)
                    self._rtcm_tx_frames += 1
                    self._rtcm_tx_bytes += len(frame)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    self._rtcm_clients.remove(sock)

    def _accept_loop(self) -> None:
        while not self._stop.is_set() and self._server_socket is not None:
            try:
                self._server_socket.settimeout(1.0)
                client, _ = self._server_socket.accept()
                with self._rtcm_lock:
                    self._rtcm_clients.append(client)
                self.get_logger().info("RTCM client connected")
            except socket.timeout:
                continue
            except OSError:
                break

    def run(self) -> None:
        """Open serial, optionally configure BS, start TCP server, run spin."""
        self.serial = SerialHandler(
            port=self.config.serial_port,
            baud_rate=self.config.baud_rate,
            on_nmea=self._on_nmea,
            on_rtcm3=self._on_rtcm3,
            log_unknown_bytes=self.get_logger().get_effective_level() <= logging.DEBUG,
        )
        self.serial.open()
        if self.config.configure_on_start:
            self.get_logger().info("Sending LC29H-BS configure commands")
            self.serial.send_base_configure()

        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.config.rtcm_tcp_bind, self.config.rtcm_tcp_port))
        self._server_socket.listen(5)
        self.get_logger().info(f"RTCM3 TCP server on {self.config.rtcm_tcp_bind}:{self.config.rtcm_tcp_port}")
        self._server_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._server_thread.start()

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
            with self._rtcm_lock:
                n_clients = len(self._rtcm_clients)
            self.get_logger().info(
                f"[diag] gga_q={quality} rtcm_tx={self._rtcm_tx_frames}f/{self._rtcm_tx_bytes}B clients={n_clients}"
            )

    def shutdown(self) -> None:
        """Close serial and TCP server."""
        self._stop.set()
        if self.serial is not None:
            self.serial.close()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None
        with self._rtcm_lock:
            for sock in self._rtcm_clients:
                try:
                    sock.close()
                except OSError:
                    pass
            self._rtcm_clients.clear()
