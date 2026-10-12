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
SILENT_LABEL = "Receiver silent"
# RTCM reference station position messages (stationary antenna coordinates the rover needs for RTK).
REF_STATION_TYPES = frozenset({1005, 1006})
AGE_DECIMALS = 1


def display_label(quality: int) -> str:
    """Human-readable fix label for the web UI.

    Args:
        quality (int): GGA fix quality code.

    Returns:
        str: Display label, "Unknown" for unmapped codes.
    """
    return DISPLAY_LABELS.get(quality, UNKNOWN_LABEL)


def rover_status(
    gga: dict[str, Any], ntrip_connected: bool, ntrip_rx_bytes: int, nmea_age_s: float | None = None
) -> dict[str, Any]:
    """Build the rover status dict from the receiver state of the latest GGA and the NTRIP client state.

    Args:
        gga (dict[str, Any]): Output of parse_gga_status or parse_gga (a no-fix GGA has quality 0).
        ntrip_connected (bool): Whether the NTRIP client is connected.
        ntrip_rx_bytes (int): RTCM bytes received from the caster.
        nmea_age_s (float | None): Seconds since the last NMEA sentence, or None when not tracked.

    Returns:
        dict[str, Any]: Rover status payload (never a position).
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
        "nmea_silent": False,
        "nmea_received": True,
        "nmea_age_s": None if nmea_age_s is None else round(nmea_age_s, AGE_DECIMALS),
    }


def rover_silent_status(
    nmea_age_s: float, nmea_received: bool, ntrip_connected: bool, ntrip_rx_bytes: int
) -> dict[str, Any]:
    """Build the rover status for a receiver that sent no NMEA for longer than nmea_timeout_s.

    The fix fields are None: the receiver state is unknown, only the silence and the NTRIP link are observed.

    Args:
        nmea_age_s (float): Seconds since the last NMEA sentence (since node start when none arrived).
        nmea_received (bool): Whether any NMEA sentence arrived since node start.
        ntrip_connected (bool): Whether the NTRIP client is connected.
        ntrip_rx_bytes (int): RTCM bytes received from the caster.

    Returns:
        dict[str, Any]: Rover status payload with nmea_silent True and the same keys as rover_status.
    """
    return {
        "role": "rover",
        "quality": None,
        "fix": SILENT_LABEL,
        "num_satellites": None,
        "hdop": None,
        "diff_age_s": None,
        "ntrip_connected": ntrip_connected,
        "ntrip_rx_bytes": ntrip_rx_bytes,
        "nmea_silent": True,
        "nmea_received": nmea_received,
        "nmea_age_s": round(nmea_age_s, AGE_DECIMALS),
    }


def build_rover_status(
    now_s: float,
    started_s: float,
    last_nmea_s: float | None,
    last_gga_s: float | None,
    gga: dict[str, Any] | None,
    nmea_timeout_s: float,
    ntrip_connected: bool,
    ntrip_rx_bytes: int,
) -> dict[str, Any] | None:
    """Pick the rover status for this cycle from the receiver timing and the latest GGA.

    Args:
        now_s (float): Current monotonic time in seconds.
        started_s (float): Monotonic time the node started.
        last_nmea_s (float | None): Monotonic time of the last NMEA sentence of any kind, or None.
        last_gga_s (float | None): Monotonic time of the last GGA, or None.
        gga (dict[str, Any] | None): Receiver state of the last GGA (parse_gga_status), or None.
        nmea_timeout_s (float): Silence in seconds after which the receiver counts as silent.
        ntrip_connected (bool): Whether the NTRIP client is connected.
        ntrip_rx_bytes (int): RTCM bytes received from the caster.

    Returns:
        dict[str, Any] | None: The silent status when no NMEA arrived for nmea_timeout_s; the GGA status when
            the last GGA is at most nmea_timeout_s old; otherwise None (nothing real to report this cycle).
    """
    nmea_age_s = now_s - (last_nmea_s if last_nmea_s is not None else started_s)
    if nmea_age_s > nmea_timeout_s:
        return rover_silent_status(nmea_age_s, last_nmea_s is not None, ntrip_connected, ntrip_rx_bytes)
    if gga is None or last_gga_s is None or now_s - last_gga_s > nmea_timeout_s:
        return None
    return rover_status(gga, ntrip_connected, ntrip_rx_bytes, nmea_age_s=nmea_age_s)


def base_status(
    gga: dict[str, Any],
    ntrip_clients: int,
    rtcm_tx_frames: int,
    rtcm_tx_bytes: int,
    rtcm_types: set[int],
    survey_state: str | None = None,
    gga_seq: int | None = None,
) -> dict[str, Any]:
    """Build the base status dict from the latest GGA and the caster state.

    Args:
        gga (dict[str, Any]): Output of parse_gga (with a position) or parse_gga_status (no-fix GGA, no position);
            latitude, longitude and altitude are included only when the GGA had a position.
        ntrip_clients (int): Connected NTRIP clients.
        rtcm_tx_frames (int): RTCM frames sent.
        rtcm_tx_bytes (int): RTCM bytes sent.
        rtcm_types (set[int]): RTCM message types seen.
        survey_state (str | None): Base calibration (survey-in) state from SerialHandler.survey_state.
        gga_seq (int | None): Number of GGA sentences received; unchanged between polls means no new GGA.

    Returns:
        dict[str, Any]: Base status payload.
    """
    position = {k: gga.get(k) for k in ("latitude", "longitude", "altitude")} if "latitude" in gga else {}
    return {
        "role": "base",
        **position,
        "quality": gga["quality"],
        "fix": display_label(gga["quality"]),
        "num_satellites": gga.get("num_satellites"),
        "hdop": gga.get("hdop"),
        "ntrip_clients": ntrip_clients,
        "rtcm_tx_frames": rtcm_tx_frames,
        "rtcm_tx_bytes": rtcm_tx_bytes,
        "rtcm_types": sorted(rtcm_types),
        "ref_station_position": bool(rtcm_types & REF_STATION_TYPES),
        "survey_state": survey_state,
        "gga_seq": gga_seq,
    }


def status_json(status: dict[str, Any]) -> str:
    """Serialize a status dict as compact JSON.

    Args:
        status (dict[str, Any]): Status payload.

    Returns:
        str: Compact JSON string.
    """
    return json.dumps(status, separators=(",", ":"))
