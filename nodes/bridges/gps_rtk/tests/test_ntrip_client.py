"""Tests for NtripClient: handshake, streaming, GGA sending, reconnect."""

import socket
import threading
import time

from gps_rtk.ntrip_client import NtripClient


def _start_fake_caster(
    port: int,
    response: bytes,
    data_to_send: bytes = b"",
    read_after_connect: bool = True,
) -> threading.Thread:
    """Start a fake NTRIP caster in a daemon thread."""

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(1)
        server.settimeout(3.0)
        try:
            conn, _ = server.accept()
            if read_after_connect:
                try:
                    conn.recv(4096)  # consume the GET request
                except OSError:
                    pass
            conn.sendall(response)
            if data_to_send:
                conn.sendall(data_to_send)
            time.sleep(0.5)
            conn.close()
        except (socket.timeout, OSError):
            pass
        finally:
            server.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    return t


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_ntrip_build_request_contains_mountpoint() -> None:
    """The HTTP request sent to the caster includes the mountpoint and Host header."""
    received: list[bytes] = []

    port = _free_port()

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(1)
        server.settimeout(3.0)
        try:
            conn, _ = server.accept()
            data = conn.recv(4096)
            received.append(data)
            conn.sendall(b"ICY 200 OK\r\n\r\n")
            time.sleep(0.2)
            conn.close()
        except (socket.timeout, OSError):
            pass
        finally:
            server.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    time.sleep(0.05)

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="",
        password="",
        on_data=lambda _: None,
        get_gga=lambda: None,
        reconnect_interval_s=99,
    )
    client.start()
    t.join(timeout=2.0)
    client.stop()

    assert received, "Caster received no request"
    req = received[0].decode("ascii", errors="replace")
    assert "GET /rtk" in req
    assert "Host:" in req


def test_ntrip_auth_header_included_when_credentials_set() -> None:
    """Basic auth header is included when user/password are non-empty."""
    import base64

    received: list[bytes] = []
    port = _free_port()

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(1)
        server.settimeout(3.0)
        try:
            conn, _ = server.accept()
            data = conn.recv(4096)
            received.append(data)
            conn.sendall(b"ICY 200 OK\r\n\r\n")
            time.sleep(0.1)
            conn.close()
        except (socket.timeout, OSError):
            pass
        finally:
            server.close()

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.05)

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="admin",
        password="secret",
        on_data=lambda _: None,
        get_gga=lambda: None,
        reconnect_interval_s=99,
    )
    client.start()
    time.sleep(0.5)
    client.stop()

    assert received
    req = received[0].decode("ascii", errors="replace")
    expected_token = base64.b64encode(b"admin:secret").decode()
    assert expected_token in req


def test_ntrip_parse_icy_200_receives_data() -> None:
    """Client forwards RTCM data from caster to on_data callback."""
    received: list[bytes] = []
    port = _free_port()
    rtcm_data = b"\xD3\x00\x04\x3E\xD0\x00\x00\x00\x00\x00"

    _start_fake_caster(port, b"ICY 200 OK\r\n\r\n", data_to_send=rtcm_data)
    time.sleep(0.05)

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="",
        password="",
        on_data=received.append,
        get_gga=lambda: None,
        reconnect_interval_s=99,
    )
    client.start()
    time.sleep(0.8)
    client.stop()

    combined = b"".join(received)
    assert rtcm_data in combined


def test_ntrip_handles_401() -> None:
    """Client logs/handles 401 and reconnects (does not raise unhandled)."""
    port = _free_port()
    _start_fake_caster(port, b"HTTP/1.1 401 Unauthorized\r\n\r\n")
    time.sleep(0.05)

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="",
        password="",
        on_data=lambda _: None,
        get_gga=lambda: None,
        reconnect_interval_s=0.1,
    )
    client.start()
    time.sleep(0.4)
    client.stop()  # must not raise


def test_ntrip_is_connected_reflects_state() -> None:
    """is_connected is True while streaming and False after stop."""
    port = _free_port()

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(1)
        server.settimeout(3.0)
        try:
            conn, _ = server.accept()
            conn.recv(4096)
            conn.sendall(b"ICY 200 OK\r\n\r\n")
            time.sleep(1.0)
            conn.close()
        except (socket.timeout, OSError):
            pass
        finally:
            server.close()

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.05)

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="",
        password="",
        on_data=lambda _: None,
        get_gga=lambda: None,
        reconnect_interval_s=99,
    )
    assert not client.is_connected
    client.start()
    time.sleep(0.3)
    assert client.is_connected
    client.stop()
    assert not client.is_connected


def test_ntrip_sends_gga_to_caster() -> None:
    """Client sends GGA sentence to caster at configured interval."""
    sent_to_caster: list[bytes] = []
    port = _free_port()

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(1)
        server.settimeout(3.0)
        try:
            conn, _ = server.accept()
            conn.recv(4096)  # initial GET
            conn.sendall(b"ICY 200 OK\r\n\r\n")
            conn.settimeout(2.0)
            # collect anything the client sends back
            while True:
                try:
                    data = conn.recv(4096)
                    if not data:
                        break
                    sent_to_caster.append(data)
                except socket.timeout:
                    break
            conn.close()
        except (socket.timeout, OSError):
            pass
        finally:
            server.close()

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.05)

    gga = "$GNGGA,120000.00,5402.5000,N,01818.5000,E,1,08,0.9,50.0,M,0.0,M,,*6E"

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="",
        password="",
        on_data=lambda _: None,
        get_gga=lambda: gga,
        gga_interval_s=0.1,  # very short for test
        reconnect_interval_s=99,
    )
    client.start()
    time.sleep(1.5)
    client.stop()

    combined = b"".join(sent_to_caster)
    assert b"GNGGA" in combined


def test_ntrip_reconnects_on_disconnect() -> None:
    """Client reconnects after caster drops the connection."""
    connect_count = [0]
    port = _free_port()

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(5)
        server.settimeout(5.0)
        try:
            for _ in range(3):
                try:
                    conn, _ = server.accept()
                    connect_count[0] += 1
                    conn.recv(4096)
                    conn.sendall(b"ICY 200 OK\r\n\r\n")
                    time.sleep(0.05)
                    conn.close()  # immediately drop
                except (socket.timeout, OSError):
                    break
        finally:
            server.close()

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.05)

    client = NtripClient(
        host="127.0.0.1",
        port=port,
        mountpoint="/rtk",
        user="",
        password="",
        on_data=lambda _: None,
        get_gga=lambda: None,
        reconnect_interval_s=0.05,
    )
    client.start()
    time.sleep(1.0)
    client.stop()

    assert connect_count[0] >= 2, f"Expected ≥2 reconnects, got {connect_count[0]}"
