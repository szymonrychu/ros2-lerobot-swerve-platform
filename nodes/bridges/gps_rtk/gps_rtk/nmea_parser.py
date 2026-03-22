"""Parse NMEA GGA/RMC and build sensor_msgs/NavSatFix."""

import math
from typing import Any

# GGA fix quality: 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK fixed, 5=RTK float, 6=est, 7=manual, 8=sim
# NavSatStatus: STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
GGA_QUALITY_TO_STATUS = {
    0: -1,
    1: 0,
    2: 1,
    3: 2,
    4: 2,
    5: 2,
    6: 0,
    7: 0,
    8: 0,
}

GGA_QUALITY_LABELS = {
    0: "NoFix",
    1: "GPS",
    2: "DGPS",
    3: "PPS",
    4: "RTK_Fixed",
    5: "RTK_Float",
    6: "Estimated",
    7: "Manual",
    8: "Simulation",
}

METRES_PER_DEG_LAT = 111_111.0


def parse_gga_lat_lon(parts: list[str]) -> tuple[float, float] | None:
    """Parse latitude and longitude from GGA fields (parts[2], parts[3], parts[4], parts[5]).

    Args:
        parts: Comma-split GGA sentence fields.

    Returns:
        (latitude_deg, longitude_deg) or None if invalid.
    """
    if len(parts) < 6:
        return None
    try:
        lat_str = parts[2]
        lat_ns = parts[3]
        lon_str = parts[4]
        lon_ew = parts[5]
        if not lat_str or not lon_str:
            return None
        lat_deg = float(lat_str[:2]) + float(lat_str[2:]) / 60.0
        if lat_ns == "S":
            lat_deg = -lat_deg
        lon_deg = float(lon_str[:3]) + float(lon_str[3:]) / 60.0
        if lon_ew == "W":
            lon_deg = -lon_deg
        return (lat_deg, lon_deg)
    except (ValueError, IndexError):
        return None


def parse_gga_altitude(parts: list[str]) -> float:
    """Parse altitude from GGA (parts[9], meters)."""
    if len(parts) < 10 or not parts[9]:
        return 0.0
    try:
        return float(parts[9])
    except ValueError:
        return 0.0


def parse_gga_quality(parts: list[str]) -> int:
    """Parse fix quality from GGA (parts[6], 0-8)."""
    if len(parts) < 7 or not parts[6].isdigit():
        return 0
    return int(parts[6])


def _safe_float(s: str) -> float | None:
    """Parse a string to float, returning None on failure or empty."""
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def parse_gga(sentence: str) -> dict[str, Any] | None:
    """Parse GGA sentence into a rich dict.

    Extracted fields:
        latitude, longitude, altitude, quality, status (NavSatStatus),
        num_satellites, hdop, diff_age_s, diff_ref_id.

    Args:
        sentence: Full NMEA sentence including $ and *XX.

    Returns:
        Dict or None if the sentence is invalid / not GGA.
    """
    if "GGA" not in sentence or not sentence.startswith("$"):
        return None
    # Strip checksum suffix so field 14 isn't polluted
    star = sentence.rfind("*")
    body = sentence[:star] if star > 0 else sentence
    parts = body.split(",")
    if len(parts) < 10:
        return None
    lat_lon = parse_gga_lat_lon(parts)
    if lat_lon is None:
        return None
    lat, lon = lat_lon
    alt = parse_gga_altitude(parts)
    quality = parse_gga_quality(parts)
    status = GGA_QUALITY_TO_STATUS.get(quality, -1)

    num_sats = int(parts[7]) if len(parts) > 7 and parts[7].isdigit() else None
    hdop = _safe_float(parts[8]) if len(parts) > 8 else None
    diff_age = _safe_float(parts[13]) if len(parts) > 13 else None
    diff_ref = parts[14].strip() if len(parts) > 14 and parts[14].strip() else None

    return {
        "latitude": lat,
        "longitude": lon,
        "altitude": alt,
        "quality": quality,
        "status": status,
        "num_satellites": num_sats,
        "hdop": hdop,
        "diff_age_s": diff_age,
        "diff_ref_id": diff_ref,
    }


def quality_label(quality: int) -> str:
    """Human-readable label for GGA fix quality code.

    Args:
        quality: GGA quality integer (0-8).

    Returns:
        Short label string.
    """
    return GGA_QUALITY_LABELS.get(quality, f"Unknown({quality})")


def drift_from_mean(
    positions: list[tuple[float, float, float]],
) -> tuple[float, float] | None:
    """Compute 2-D and 3-D drift of the last position from the mean of all positions.

    Args:
        positions: List of (lat_deg, lon_deg, alt_m) tuples (at least 2).

    Returns:
        (drift_2d_m, drift_3d_m) or None if fewer than 2 positions.
    """
    if len(positions) < 2:
        return None
    mean_lat = sum(p[0] for p in positions) / len(positions)
    mean_lon = sum(p[1] for p in positions) / len(positions)
    mean_alt = sum(p[2] for p in positions) / len(positions)
    last = positions[-1]
    dlat_m = (last[0] - mean_lat) * METRES_PER_DEG_LAT
    dlon_m = (last[1] - mean_lon) * METRES_PER_DEG_LAT * math.cos(math.radians(mean_lat))
    dalt_m = last[2] - mean_alt
    d2d = math.sqrt(dlat_m**2 + dlon_m**2)
    d3d = math.sqrt(dlat_m**2 + dlon_m**2 + dalt_m**2)
    return (d2d, d3d)


def build_nav_sat_fix(
    lat: float,
    lon: float,
    alt: float,
    status: int,
    frame_id: str = "gps_link",
    hdop: float | None = None,
) -> Any:
    """Build sensor_msgs/NavSatFix (requires ROS env).

    Args:
        lat: Latitude degrees.
        lon: Longitude degrees.
        alt: Altitude meters.
        status: NavSatStatus.status value (-1, 0, 1, 2).
        frame_id: Header frame_id.
        hdop: Horizontal dilution of precision (used for covariance estimate).

    Returns:
        sensor_msgs.msg.NavSatFix instance.
    """
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    msg = NavSatFix()
    msg.header.frame_id = frame_id
    msg.status.status = status
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    if status >= 0:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        # Approximate horizontal σ ≈ HDOP * 2.5 m (typical UERE)
        h_var = ((hdop or 1.0) * 2.5) ** 2
        msg.position_covariance[0] = h_var
        msg.position_covariance[4] = h_var
        msg.position_covariance[8] = h_var * 4.0  # vertical ~2× worse
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return msg
