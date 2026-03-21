"""Minimal NTRIP v1 caster: accepts rover connections and streams RTCM3 frames."""

import base64
import logging
import socket
import threading
from typing import Any

LOG = logging.getLogger(__name__)

# Bytes to read from newly connected clients for the HTTP request
_REQUEST_READ_SIZE = 4096
_REQUEST_TIMEOUT_S = 5.0

NTRIP_USER_AGENT = "gps_rtk-caster/1.0"


class NtripCaster:
    """Self-hosted NTRIP v1 caster embedded in the base node.

    Accepts NTRIP client connections, validates the HTTP GET request and
    optional Basic auth, then streams validated RTCM3 frames to all
    connected clients. The mixed NMEA+RTCM3+proprietary serial stream is
    handled upstream by SerialStreamParser; only clean RTCM3 frames are
    broadcast here.

    Usage::

        caster = NtripCaster(
            bind="0.0.0.0", port=5016, mountpoint="/rtk",
            user="", password="", logger=self.get_logger()
        )
        caster.start()
        # later:
        caster.broadcast_rtcm(frame_bytes)
        caster.stop()
    """

    def __init__(
        self,
        bind: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
        logger: Any | None = None,
    ) -> None:
        self._bind = bind
        self._port = port
        self._mountpoint = mountpoint.lstrip("/")
        self._auth_required = bool(user)
        self._expected_token = base64.b64encode(f"{user}:{password}".encode()).decode() if user else ""
        self._logger = logger

        self._server_socket: socket.socket | None = None
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._tx_frames = 0
        self._tx_bytes = 0

    def start(self) -> None:
        """Bind TCP socket and start the accept loop in a background thread."""
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self._bind, self._port))
        self._server_socket.listen(5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        self._log_info("NTRIP caster listening on %s:%d/%s", self._bind, self._port, self._mountpoint)

    def stop(self) -> None:
        """Signal stop, close all client sockets and server socket."""
        self._stop.set()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None
        with self._lock:
            for sock in self._clients:
                try:
                    sock.close()
                except OSError:
                    pass
            self._clients.clear()

    def broadcast_rtcm(self, frame: bytes) -> None:
        """Send one RTCM3 frame to all connected NTRIP clients.

        Args:
            frame: Validated RTCM3 frame bytes.
        """
        with self._lock:
            dead: list[socket.socket] = []
            for sock in self._clients:
                try:
                    sock.sendall(frame)
                    self._tx_frames += 1
                    self._tx_bytes += len(frame)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(sock)
            for sock in dead:
                self._clients.remove(sock)
                try:
                    sock.close()
                except OSError:
                    pass
            if dead:
                self._log_debug("Removed %d disconnected NTRIP client(s)", len(dead))

    @property
    def client_count(self) -> int:
        """Number of currently connected NTRIP clients."""
        with self._lock:
            return len(self._clients)

    @property
    def tx_frames(self) -> int:
        """Total RTCM frames sent across all clients."""
        return self._tx_frames

    @property
    def tx_bytes(self) -> int:
        """Total bytes sent across all clients."""
        return self._tx_bytes

    def _accept_loop(self) -> None:
        while not self._stop.is_set() and self._server_socket is not None:
            try:
                self._server_socket.settimeout(1.0)
                client, addr = self._server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._log_debug("NTRIP connection from %s:%d", addr[0], addr[1])
            t = threading.Thread(target=self._handle_client, args=(client, addr), daemon=True)
            t.start()

    def _handle_client(self, sock: socket.socket, addr: tuple[str, int]) -> None:
        """Read HTTP request, validate, respond, add to client list."""
        try:
            sock.settimeout(_REQUEST_TIMEOUT_S)
            raw = sock.recv(_REQUEST_READ_SIZE)
        except (socket.timeout, OSError) as e:
            self._log_debug("Failed to read NTRIP request from %s: %s", addr[0], e)
            try:
                sock.close()
            except OSError:
                pass
            return

        try:
            request = raw.decode("ascii", errors="replace")
        except Exception:
            request = ""

        # Parse first line: e.g. "GET /rtk HTTP/1.1"
        lines = request.replace("\r\n", "\n").split("\n")
        first_line = lines[0].strip() if lines else ""
        parts = first_line.split()
        if len(parts) < 2 or parts[0] != "GET":
            self._send_response(sock, "HTTP/1.1 400 Bad Request\r\n\r\n")
            sock.close()
            return

        # Check mountpoint (strip leading /)
        requested = parts[1].lstrip("/")
        if requested != self._mountpoint:
            self._log_debug(
                "NTRIP client %s requested unknown mountpoint /%s (want /%s)",
                addr[0],
                requested,
                self._mountpoint,
            )
            self._send_response(sock, "HTTP/1.1 404 Not Found\r\n\r\n")
            sock.close()
            return

        # Check Basic auth if configured
        if self._auth_required:
            auth_token = self._extract_auth(lines)
            if auth_token != self._expected_token:
                self._log_debug("NTRIP client %s auth failed", addr[0])
                self._send_response(sock, "HTTP/1.1 401 Unauthorized\r\n\r\n")
                sock.close()
                return

        # All good — send ICY 200 OK and add to broadcast list
        self._send_response(sock, "ICY 200 OK\r\n\r\n")
        sock.settimeout(None)  # blocking mode for sendall in broadcast_rtcm
        with self._lock:
            self._clients.append(sock)
        self._log_info("NTRIP client %s connected to /%s", addr[0], self._mountpoint)

    @staticmethod
    def _extract_auth(lines: list[str]) -> str:
        """Extract Base64 token from Authorization: Basic <token> header."""
        for line in lines:
            if line.lower().startswith("authorization:"):
                parts = line.split(None, 2)
                if len(parts) == 3 and parts[1].lower() == "basic":
                    return parts[2].strip()
        return ""

    @staticmethod
    def _send_response(sock: socket.socket, response: str) -> None:
        try:
            sock.sendall(response.encode("ascii"))
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
