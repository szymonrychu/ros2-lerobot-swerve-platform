"""Tests for NtripCaster: request parsing, auth, broadcasting."""

import socket
import time

from gps_rtk.ntrip_caster import NtripCaster


def _make_request(
    mountpoint: str = "/rtk",
    user: str = "",
    password: str = "",
    version: str = "HTTP/1.1",
) -> bytes:
    """Build a minimal NTRIP GET request."""
    import base64

    auth = ""
    if user:
        token = base64.b64encode(f"{user}:{password}".encode()).decode()
        auth = f"Authorization: Basic {token}\r\n"
    return f"GET {mountpoint} {version}\r\nUser-Agent: test\r\n{auth}\r\n".encode("ascii")


def _caster_with_request(
    request: bytes,
    mountpoint: str = "/rtk",
    user: str = "",
    password: str = "",
) -> bytes:
    """Connect to a started NtripCaster, send request, return response."""
    caster = NtripCaster(
        bind="127.0.0.1",
        port=0,
        mountpoint=mountpoint,
        user=user,
        password=password,
    )
    # patch socket to use a free port
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", 0))
    port = server_sock.getsockname()[1]
    server_sock.close()

    caster._bind = "127.0.0.1"
    caster._port = port
    caster.start()
    time.sleep(0.05)

    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2.0)
        client.connect(("127.0.0.1", port))
        client.sendall(request)
        try:
            response = client.recv(1024)
        except socket.timeout:
            response = b""
        client.close()
    finally:
        caster.stop()

    return response


def test_valid_ntrip_request_gets_icy_200() -> None:
    response = _caster_with_request(_make_request("/rtk"))
    assert b"ICY 200 OK" in response


def test_invalid_mountpoint_gets_404() -> None:
    response = _caster_with_request(_make_request("/wrong"), mountpoint="rtk")
    assert b"404" in response


def test_unauthorized_gets_401() -> None:
    response = _caster_with_request(
        _make_request("/rtk", user="bad", password="bad"),
        mountpoint="rtk",
        user="admin",
        password="secret",
    )
    assert b"401" in response


def test_correct_auth_gets_icy_200() -> None:
    response = _caster_with_request(
        _make_request("/rtk", user="admin", password="secret"),
        mountpoint="rtk",
        user="admin",
        password="secret",
    )
    assert b"ICY 200 OK" in response


def test_no_auth_when_not_configured() -> None:
    """No auth header needed when user/password are empty."""
    response = _caster_with_request(_make_request("/rtk", user="", password=""), mountpoint="rtk")
    assert b"ICY 200 OK" in response


def test_bad_request_gets_400() -> None:
    """Non-GET request gets 400."""
    bad_request = b"POST /rtk HTTP/1.1\r\n\r\n"
    response = _caster_with_request(bad_request)
    assert b"400" in response


def test_broadcast_rtcm_to_connected_clients() -> None:
    """broadcast_rtcm() sends data to all connected clients."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", 0))
    port = server_sock.getsockname()[1]
    server_sock.close()

    caster = NtripCaster(bind="127.0.0.1", port=port, mountpoint="rtk", user="", password="")
    caster.start()
    time.sleep(0.05)

    clients = []
    try:
        for _ in range(2):
            c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            c.settimeout(2.0)
            c.connect(("127.0.0.1", port))
            c.sendall(_make_request("/rtk"))
            resp = c.recv(256)
            assert b"ICY 200 OK" in resp
            clients.append(c)

        time.sleep(0.05)
        assert caster.client_count == 2

        frame = b"\xD3\x00\x02\x01\x02\x00\x00\x00"  # fake frame (not CRC-validated here)
        caster.broadcast_rtcm(frame)
        for c in clients:
            c.settimeout(1.0)
            data = c.recv(256)
            assert data == frame
    finally:
        for c in clients:
            c.close()
        caster.stop()


def test_disconnected_client_removed() -> None:
    """broadcast_rtcm() removes clients that have disconnected."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", 0))
    port = server_sock.getsockname()[1]
    server_sock.close()

    caster = NtripCaster(bind="127.0.0.1", port=port, mountpoint="rtk", user="", password="")
    caster.start()
    time.sleep(0.05)

    try:
        c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        c.settimeout(2.0)
        c.connect(("127.0.0.1", port))
        c.sendall(_make_request("/rtk"))
        resp = c.recv(256)
        assert b"ICY 200 OK" in resp
        time.sleep(0.05)
        assert caster.client_count == 1

        c.close()  # simulate disconnect
        time.sleep(0.05)

        # First broadcast may succeed on macOS (CLOSE_WAIT) — second detects the dead socket
        caster.broadcast_rtcm(b"\xD3\x00\x01\xFF\x00\x00\x00")
        time.sleep(0.05)
        caster.broadcast_rtcm(b"\xD3\x00\x01\xFF\x00\x00\x00")
        time.sleep(0.05)
        assert caster.client_count == 0
    finally:
        caster.stop()
