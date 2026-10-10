"""Tests for the rover Prometheus metrics: GGA gauges, NTRIP client counters and serial error counters."""

import math
import socket
import threading
import time
from unittest.mock import MagicMock, patch

import serial
from prometheus_client import REGISTRY

from gps_rtk import metrics
from gps_rtk.ntrip_client import NtripClient
from gps_rtk.serial_handler import SerialHandler


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    """Return the current sample value, 0.0 when the series does not exist yet."""
    value = REGISTRY.get_sample_value(name, labels or {})
    return 0.0 if value is None else value


def free_port() -> int:
    """Return a free localhost TCP port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_record_gga_sets_all_gauges() -> None:
    metrics.record_gga({"quality": 4, "num_satellites": 17, "hdop": 0.7, "diff_age_s": 1.5})
    assert sample("gps_fix_quality") == 4
    assert sample("gps_satellites") == 17
    assert sample("gps_hdop") == 0.7
    assert sample("gps_diff_age_seconds") == 1.5


def test_record_gga_missing_fields_become_nan_not_zero() -> None:
    metrics.record_gga({"quality": 1, "num_satellites": None, "hdop": None, "diff_age_s": None})
    assert sample("gps_fix_quality") == 1
    assert math.isnan(sample("gps_satellites"))
    assert math.isnan(sample("gps_hdop"))
    assert math.isnan(sample("gps_diff_age_seconds"))


def test_record_fix_published_counts() -> None:
    before = sample("gps_fixes_published_total")
    metrics.record_fix_published()
    assert sample("gps_fixes_published_total") == before + 1


def test_ntrip_client_exports_connection_bytes_and_reconnects() -> None:
    port = free_port()
    payload = b"\xd3\x00\x04\x3e\xd0\x00\x00\x00\x00\x00"
    connected_during: list[float] = []

    def serve() -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(5)
        server.settimeout(3.0)
        try:
            for _ in range(2):
                conn, _ = server.accept()
                conn.recv(4096)
                conn.sendall(b"ICY 200 OK\r\n\r\n" + payload)
                time.sleep(0.3)
                connected_during.append(sample("gps_ntrip_connected"))
                conn.close()
        except (TimeoutError, OSError):
            pass
        finally:
            server.close()

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.05)
    rx_before = sample("gps_ntrip_rx_bytes_total")
    reconnects_before = sample("gps_ntrip_reconnects_total")
    client = NtripClient("127.0.0.1", port, "/rtk", "", "", lambda _d: None, lambda: None, reconnect_interval_s=0.05)
    client.start()
    time.sleep(1.2)
    client.stop()

    assert connected_during and connected_during[0] == 1
    assert sample("gps_ntrip_rx_bytes_total") - rx_before >= 2 * len(payload)
    assert sample("gps_ntrip_reconnects_total") - reconnects_before >= 1
    assert sample("gps_ntrip_connected") == 0


def test_serial_open_failures_counted_per_attempt() -> None:
    labels = {"op": "open"}
    before = sample("gps_serial_errors_total", labels)
    handler = SerialHandler("/dev/null", 115200, lambda _s: None, lambda _f: None)
    ok = MagicMock()
    with patch("gps_rtk.serial_handler.serial.Serial", side_effect=[serial.SerialException("x"), OSError("y"), ok]):
        with patch("gps_rtk.serial_handler.threading.Thread"):
            with patch.object(handler._stop, "wait"):
                handler.open()
    assert sample("gps_serial_errors_total", labels) == before + 2


def test_serial_read_error_counted() -> None:
    labels = {"op": "read"}
    before = sample("gps_serial_errors_total", labels)
    handler = SerialHandler("/dev/null", 115200, lambda _s: None, lambda _f: None)
    ser = MagicMock()
    ser.is_open = True
    type(ser).in_waiting = property(lambda _self: (_ for _ in ()).throw(OSError("boom")))
    handler._ser = ser

    def stop_after_wait(_t: float) -> bool:
        handler._stop.set()
        return True

    with patch.object(handler._stop, "wait", side_effect=stop_after_wait):
        handler._read_loop()
    assert sample("gps_serial_errors_total", labels) == before + 1
