"""Pure builders for the compact GPS status JSON published on status_topic."""

import json
from typing import Any

DISPLAY_LABELS: dict[int, str] = {
    0: "No fix",
    1: "GPS",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "Dead reckoning",
}
UNKNOWN_LABEL = "Unknown"


def display_label(quality: int) -> str:
    """Human-readable fix label for the web UI.

    Args:
        quality (int): GGA fix quality code.

    Returns:
        str: Display label, "Unknown" for unmapped codes.
    """
    return DISPLAY_LABELS.get(quality, UNKNOWN_LABEL)


def rover_status(gga: dict[str, Any], ntrip_connected: bool, ntrip_rx_bytes: int) -> dict[str, Any]:
    """Build the rover status dict from a parsed GGA and NTRIP client state.

    Args:
        gga (dict[str, Any]): Output of parse_gga.
        ntrip_connected (bool): Whether the NTRIP client is connected.
        ntrip_rx_bytes (int): RTCM bytes received from the caster.

    Returns:
        dict[str, Any]: Rover status payload.
    """
    return {
        "role": "rover",
        "quality": gga["quality"],
        "fix": display_label(gga["quality"]),
        "num_satellites": gga.get("num_satellites"),
        "hdop": gga.get("hdop"),
        "diff_age_s": gga.get("diff_age_s"),
        "ntrip_connected": ntrip_connected,
        "ntrip_rx_bytes": ntrip_rx_bytes,
    }


def base_status(
    gga: dict[str, Any], ntrip_clients: int, rtcm_tx_frames: int, rtcm_tx_bytes: int, rtcm_types: set[int]
) -> dict[str, Any]:
    """Build the base status dict from a parsed GGA and caster state.

    Args:
        gga (dict[str, Any]): Output of parse_gga (a GGA without a position never parses, so latitude and
            longitude are always real).
        ntrip_clients (int): Connected NTRIP clients.
        rtcm_tx_frames (int): RTCM frames sent.
        rtcm_tx_bytes (int): RTCM bytes sent.
        rtcm_types (set[int]): RTCM message types seen.

    Returns:
        dict[str, Any]: Base status payload.
    """
    return {
        "role": "base",
        "latitude": gga["latitude"],
        "longitude": gga["longitude"],
        "altitude": gga.get("altitude"),
        "quality": gga["quality"],
        "fix": display_label(gga["quality"]),
        "num_satellites": gga.get("num_satellites"),
        "hdop": gga.get("hdop"),
        "ntrip_clients": ntrip_clients,
        "rtcm_tx_frames": rtcm_tx_frames,
        "rtcm_tx_bytes": rtcm_tx_bytes,
        "rtcm_types": sorted(rtcm_types),
    }


def status_json(status: dict[str, Any]) -> str:
    """Serialize a status dict as compact JSON.

    Args:
        status (dict[str, Any]): Status payload.

    Returns:
        str: Compact JSON string.
    """
    return json.dumps(status, separators=(",", ":"))
