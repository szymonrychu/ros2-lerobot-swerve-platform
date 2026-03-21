"""Minimal NTRIP v1 client: receives RTCM3 stream from a caster, sends periodic GGA."""

import base64
import logging
import socket
import threading
import time
from typing import Any, Callable

LOG = logging.getLogger(__name__)

NTRIP_USER_AGENT = "gps_rtk/1.0"
_RECV_SIZE = 4096
_RESPONSE_READ_SIZE = 4096
_CONNECT_TIMEOUT_S = 10.0


class NtripClient:
    """NTRIP v1 client: connects to a caster, streams RTCM3 to serial, sends periodic GGA.

    Usage::

        client = NtripClient(
            host="server.ros2.lan", port=5016, mountpoint="/rtk",
            user="", password="",
            on_data=lambda data: serial.write(data),
            get_gga=lambda: latest_gga_sentence,
            logger=self.get_logger(),
        )
        client.start()
        # later:
        client.stop()
    """

    def __init__(
        self,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
        on_data: Callable[[bytes], None],
        get_gga: Callable[[], str | None],
        gga_interval_s: float = 10.0,
        reconnect_interval_s: float = 5.0,
        logger: Any | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._mountpoint = "/" + mountpoint.lstrip("/")
        self._auth_header = (
            "Authorization: Basic " + base64.b64encode(f"{user}:{password}".encode()).decode() + "\r\n" if user else ""
        )
        self._on_data = on_data
        self._get_gga = get_gga
        self._gga_interval_s = gga_interval_s
        self._reconnect_interval_s = reconnect_interval_s
        self._logger = logger

        self._stop = threading.Event()
        self._connected = threading.Event()
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._rx_bytes: int = 0

    def start(self) -> None:
        """Start the NTRIP connection loop in a background daemon thread."""
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal stop and close the socket."""
        self._stop.set()
        self._connected.clear()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def is_connected(self) -> bool:
        """True while the NTRIP connection is active."""
        return self._connected.is_set()

    @property
    def rx_bytes(self) -> int:
        """Total RTCM bytes received from the caster."""
        return self._rx_bytes

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._connect_and_stream()
            except Exception as e:
                self._log_debug("NTRIP connection error: %s", e)
            finally:
                self._connected.clear()
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
            if not self._stop.is_set():
                self._log_debug("NTRIP disconnected — reconnecting in %.1fs", self._reconnect_interval_s)
                self._stop.wait(self._reconnect_interval_s)

    def _connect_and_stream(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(_CONNECT_TIMEOUT_S)
        sock.connect((self._host, self._port))
        self._sock = sock

        # Send NTRIP GET request
        request = (
            f"GET {self._mountpoint} HTTP/1.1\r\n"
            f"Host: {self._host}:{self._port}\r\n"
            f"User-Agent: {NTRIP_USER_AGENT}\r\n"
            f"{self._auth_header}"
            "Connection: close\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))

        # Read response headers (look for blank line separator)
        response_buf = b""
        while b"\r\n\r\n" not in response_buf and b"\n\n" not in response_buf:
            chunk = sock.recv(_RESPONSE_READ_SIZE)
            if not chunk:
                raise ConnectionError("NTRIP caster closed connection during handshake")
            response_buf += chunk
            if len(response_buf) > 8192:
                raise ConnectionError("NTRIP response header too large")

        header_part, _, leftover = response_buf.partition(b"\r\n\r\n")
        if not leftover:
            _, _, leftover = response_buf.partition(b"\n\n")

        header_text = header_part.decode("ascii", errors="replace")
        first_line = header_text.split("\n")[0].strip()

        if "401" in first_line:
            raise PermissionError(f"NTRIP auth failed: {first_line}")
        if "404" in first_line:
            raise ConnectionError(f"NTRIP mountpoint not found: {first_line}")
        if "ICY 200 OK" not in first_line and "200 OK" not in first_line:
            raise ConnectionError(f"Unexpected NTRIP response: {first_line}")

        self._log_info("NTRIP connected to %s:%d%s", self._host, self._port, self._mountpoint)
        self._connected.set()
        sock.settimeout(None)  # switch to blocking for streaming

        # Feed any leftover bytes already received after headers
        if leftover:
            self._rx_bytes += len(leftover)
            self._on_data(leftover)

        # Stream RTCM data, periodically sending GGA
        last_gga_time = time.monotonic()
        while not self._stop.is_set():
            sock.settimeout(1.0)
            try:
                data = sock.recv(_RECV_SIZE)
            except socket.timeout:
                data = None  # timeout — not an EOF
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                self._log_debug("NTRIP stream error: %s", e)
                break
            if data == b"":  # EOF — caster closed connection
                self._log_debug("NTRIP caster closed connection")
                break
            if data:
                self._rx_bytes += len(data)
                self._on_data(data)
            # Send GGA position report periodically
            now = time.monotonic()
            if now - last_gga_time >= self._gga_interval_s:
                self._send_gga(sock)
                last_gga_time = now

    def _send_gga(self, sock: socket.socket) -> None:
        gga = self._get_gga()
        if not gga:
            return
        try:
            line = gga.strip()
            if not line.endswith("\r\n"):
                line += "\r\n"
            sock.sendall(line.encode("ascii"))
        except OSError:
            pass

    def _log_info(self, msg: str, *args: Any) -> None:
        if self._logger is not None:
            self._logger.info(msg % args)
        else:
            LOG.info(msg, *args)

    def _log_debug(self, msg: str, *args: Any) -> None:
        if self._logger is not None:
            self._logger.debug(msg % args)
        else:
            LOG.debug(msg, *args)
