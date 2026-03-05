"""RTCM3 frame parsing: preamble 0xD3, 10-bit length, payload, CRC24Q."""

# CRC24Q polynomial for RTCM3 (0x1864CFB)
CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    """Compute CRC24Q over data (preamble + header + payload, no CRC bytes).

    Args:
        data: Bytes to checksum (typically full frame minus last 3 bytes).

    Returns:
        int: 24-bit CRC value (0x000000 .. 0xFFFFFF).
    """
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            if crc & 0x800000:
                crc = (crc << 1) ^ CRC24Q_POLY
            else:
                crc = crc << 1
            crc &= 0xFFFFFF
    return crc


def parse_rtcm3_length(header: bytes) -> int | None:
    """Extract payload length from RTCM3 header (2 bytes after 0xD3).

    RTCM3: 0xD3, then 6 reserved bits + 10-bit length (big-endian in 2 bytes).
    Length is payload only (not including header or CRC).

    Args:
        header: Exactly 2 bytes after 0xD3.

    Returns:
        Payload length in bytes, or None if invalid.
    """
    if len(header) < 2:
        return None
    length = ((header[0] & 0x03) << 8) | header[1]
    if length > 1023:
        return None
    return length


def is_valid_rtcm3_frame(frame: bytes) -> bool:
    """Check that frame has 0xD3, valid length, and correct CRC24Q.

    Args:
        frame: Full frame: 0xD3 (1) + header (2) + payload (length) + CRC (3).

    Returns:
        True if valid.
    """
    if len(frame) < 6 or frame[0] != 0xD3:
        return False
    length = parse_rtcm3_length(frame[1:3])
    if length is None:
        return False
    if len(frame) != 1 + 2 + length + 3:
        return False
    computed = crc24q(frame[:-3])
    received = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
    return computed == received
