"""Parse NMEA GGA/RMC and build sensor_msgs/NavSatFix."""

from typing import Any

# GGA fix quality: 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK fixed, 5=RTK float, 6=est, 7=manual, 8=sim
# NavSatStatus: STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
GGA_QUALITY_TO_STATUS = {
    0: -1,
    1: 0,
    2: 1,
    3: 2,
    4: 2,
    5: 1,
    6: 0,
    7: 0,
    8: 0,
}


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
        # DDMM.MMMM or DDDMM.MMMM
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


def parse_gga(sentence: str) -> dict[str, Any] | None:
    """Parse GGA sentence into dict: lat, lon, alt, quality, status.

    Args:
        sentence: Full NMEA sentence including $ and *XX.

    Returns:
        Dict with lat, lon, alt, quality, status (NavSatStatus value), or None.
    """
    if "GGA" not in sentence or not sentence.startswith("$"):
        return None
    parts = sentence.split(",")
    if len(parts) < 10:
        return None
    lat_lon = parse_gga_lat_lon(parts)
    if lat_lon is None:
        return None
    lat, lon = lat_lon
    alt = parse_gga_altitude(parts)
    quality = parse_gga_quality(parts)
    status = GGA_QUALITY_TO_STATUS.get(quality, -1)
    return {
        "latitude": lat,
        "longitude": lon,
        "altitude": alt,
        "quality": quality,
        "status": status,
    }


def build_nav_sat_fix(
    lat: float,
    lon: float,
    alt: float,
    status: int,
    frame_id: str = "gps_link",
) -> Any:
    """Build sensor_msgs/NavSatFix (requires ROS env).

    Args:
        lat: Latitude degrees.
        lon: Longitude degrees.
        alt: Altitude meters.
        status: NavSatStatus.status value (-1, 0, 1, 2).
        frame_id: Header frame_id.

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
        msg.position_covariance[0] = 1.0
        msg.position_covariance[4] = 1.0
        msg.position_covariance[8] = 1.0
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return msg
