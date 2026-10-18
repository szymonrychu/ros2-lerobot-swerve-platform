"""Tests for shared/ros2_common/marker_payload.py: payload grammar, codec round trip, MarkerInfo."""

import pytest
from pydantic import ValidationError

from shared.ros2_common.marker_payload import (
    ALPHANUMERIC,
    KNOWN_KINDS,
    MarkerInfo,
    MarkerPayload,
    PayloadError,
    decode_payload,
    encode_payload,
    marker_info,
)


def make(kind: str = "CHARGER", name: str = "DOCK-1", size: float = 150.0, **kw: object) -> MarkerPayload:
    """Build a payload with defaults."""
    return MarkerPayload(version_major=1, version_minor=None, kind_code=kind, name=name, size_mm=size, params={}, **kw)


@pytest.mark.parametrize("code,poi", sorted(KNOWN_KINDS.items()))
def test_round_trip_known_kinds(code: str, poi: str) -> None:
    p = make(kind=code, name="A-1", size=42.5)
    text = encode_payload(p)
    assert set(text) <= set(ALPHANUMERIC)
    q = decode_payload(text)
    assert q == p
    assert q.poi_kind == poi


def test_round_trip_params_and_minor() -> None:
    p = MarkerPayload(
        version_major=1, version_minor=3, kind_code="BUTTON", name="B1", size_mm=20.0, params={"FLOOR": "2 $%*+.-", "A": ""}
    )
    assert decode_payload(encode_payload(p)) == p


def test_decode_example() -> None:
    p = decode_payload("P1/K:CHARGER/N:DOCK-1/S:150")
    assert p.poi_kind == "charger"
    assert p.key == "charger/DOCK-1"
    assert p.size_mm == 150.0


def test_canonical_order_and_size_format() -> None:
    p = make(size=150.0)
    p.params = {"ZED": "1", "ALPHA": "2"}
    assert encode_payload(p) == "P1/K:CHARGER/N:DOCK-1/S:150/ALPHA:2/ZED:1"
    assert encode_payload(make(size=42.5)).endswith("/S:42.5")


def test_minor_accepted_major_rejected() -> None:
    p = decode_payload("P1.3/K:BASE/N:HOME/S:100")
    assert p.version_minor == 3
    with pytest.raises(PayloadError, match="unsupported payload version 2"):
        decode_payload("P2/K:BASE/N:HOME/S:100")


def test_unknown_kind_lowercased() -> None:
    p = decode_payload("P1/K:LAMP/N:L1/S:50")
    assert p.poi_kind == "lamp"
    assert p.key == "lamp/L1"


@pytest.mark.parametrize(
    "text,reason",
    [
        ("K:BASE/N:HOME/S:100", "P segment"),
        ("PX/K:BASE/N:HOME/S:100", "P segment"),
        ("P1/N:HOME/S:100", "missing K"),
        ("P1/K:BASE/S:100", "missing N"),
        ("P1/K:BASE/N:HOME", "missing S"),
        ("P1/K:BASE/K:BASE/N:HOME/S:100", "duplicate key K"),
        ("P1/K:BASE/N:HOME/S:100/X:1/X:2", "duplicate key X"),
        ("P1/K:BASE//N:HOME/S:100", "empty segment"),
        ("P1/K:BASE/N:HOME/S:100/", "empty segment"),
        ("P1/K:base/N:HOME/S:100", "character"),
        ("P1/K:BASE/N:HOME/S:19", "size"),
        ("P1/K:BASE/N:HOME/S:1001", "size"),
        ("P1/K:BASE/N:HOME/S:abc", "size"),
        ("P1/K:BASE/N:HOME/S:10.55", "size"),
        ("P1/K:BASE/N:HOME/S:", "size"),
        ("P1/K:BASE/N:HOME/S:100/NOCOLON", "colon"),
        ("", "P segment"),
    ],
)
def test_malformed(text: str, reason: str) -> None:
    with pytest.raises(PayloadError, match=reason):
        decode_payload(text)


def test_payload_error_is_value_error() -> None:
    assert issubclass(PayloadError, ValueError)


def test_model_validation() -> None:
    with pytest.raises(ValidationError):
        make(name="has space")
    with pytest.raises(ValidationError):
        MarkerPayload(version_major=1, version_minor=None, kind_code="A", name="B", size_mm=50, params={"P": "1"})


def test_marker_info_carries_canonical_text() -> None:
    p = make()
    p.params = {"B": "2", "A": "1"}
    info = marker_info(p)
    assert isinstance(info, MarkerInfo)
    assert info.payload == "P1/K:CHARGER/N:DOCK-1/S:150/A:1/B:2"
    assert (info.kind, info.id, info.key, info.size_mm, info.version, info.pose_samples) == (
        "charger",
        "DOCK-1",
        "charger/DOCK-1",
        150.0,
        1,
        1,
    )
    assert info.params == {"B": "2", "A": "1"}
    with pytest.raises(ValidationError):
        MarkerInfo(**{**info.model_dump(), "extra": 1})
    with pytest.raises(ValidationError):
        marker_info(p, pose_samples=-1)
