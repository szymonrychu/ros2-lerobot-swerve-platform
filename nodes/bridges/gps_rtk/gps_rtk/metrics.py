"""Prometheus metrics of the GPS RTK rover, registered once in the default registry.

The base node shares the serial handler and so increments the serial error counter, but it never starts a metrics
server (metrics_port unset), so nothing is exported there.
"""

from typing import Any

from prometheus_client import Counter, Gauge

FIX_QUALITY = Gauge("gps_fix_quality", "GGA fix quality code (0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float)")
SATELLITES = Gauge("gps_satellites", "Satellites used in the GGA fix (NaN when not reported)")
HDOP = Gauge("gps_hdop", "GGA horizontal dilution of precision (NaN when not reported)")
DIFF_AGE = Gauge("gps_diff_age_seconds", "Age of the differential corrections in the GGA (NaN when not reported)")
NTRIP_CONNECTED = Gauge("gps_ntrip_connected", "1 while the NTRIP client is connected to the caster")
NTRIP_RX_BYTES = Counter("gps_ntrip_rx_bytes_total", "RTCM bytes received from the NTRIP caster")
NTRIP_RECONNECTS = Counter("gps_ntrip_reconnects_total", "NTRIP reconnect attempts after a failed or lost connection")
SERIAL_ERRORS = Counter("gps_serial_errors_total", "Serial port errors by operation", ["op"])
FIXES_PUBLISHED = Counter("gps_fixes_published_total", "NavSatFix messages published")


def record_gga(gga: dict[str, Any]) -> None:
    """Export the fields of a parsed GGA sentence; fields the receiver did not report become NaN.

    Args:
        gga (dict[str, Any]): Output of parse_gga (quality, num_satellites, hdop, diff_age_s).
    """
    FIX_QUALITY.set(gga["quality"])
    for gauge, key in ((SATELLITES, "num_satellites"), (HDOP, "hdop"), (DIFF_AGE, "diff_age_s")):
        value = gga.get(key)
        gauge.set(float("nan") if value is None else value)


def record_fix_published() -> None:
    """Count a published NavSatFix."""
    FIXES_PUBLISHED.inc()
