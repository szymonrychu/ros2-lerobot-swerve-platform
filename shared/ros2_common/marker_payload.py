"""Marker payload codec shared by the marker generator, the detector and poi_store (pydantic, no ROS).

Grammar: ``P<major>[.<minor>]/K:<KIND>/N:<NAME>/S:<MM>[/<KEY>:<VAL>...]`` in the QR alphanumeric set. See
shared/README.md (marker_payload) and the marker-and-calibration-board design D2/D3.
"""

import re
from typing import Self

from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator

PAYLOAD_PREFIX = "P"
SUPPORTED_MAJOR = 1
ALPHANUMERIC = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:"
KNOWN_KINDS = {
    "BASE": "base",
    "CHARGER": "charger",
    "DROPOFF": "drop_off",
    "BUTTON": "button",
    "SWITCH": "switch",
    "TOOLSTATION": "tool_station",
}
MIN_SIZE_MM = 20.0
MAX_SIZE_MM = 1000.0
RESERVED_KEYS = frozenset("PKNS")
KIND_RE = re.compile(r"^[A-Z0-9-]{1,16}$")
NAME_RE = re.compile(r"^[A-Z0-9-]{1,20}$")
PARAM_KEY_RE = re.compile(r"^[A-Z0-9]{1,8}$")
PARAM_VALUE_RE = re.compile(r"^[A-Z0-9 $%*+.-]{0,24}$")
VERSION_RE = re.compile(r"^P(\d+)(?:\.(\d+))?$")
SIZE_RE = re.compile(r"^\d+(\.\d)?$")


class PayloadError(ValueError):
    """A marker payload text or field is invalid; the message names the problem."""


class MarkerPayload(BaseModel):
    """One decoded marker payload.

    version_major / version_minor: payload version (minor None when absent). kind_code: raw K. name: N. size_mm: S,
    the symbol edge excluding the quiet zone. params: extra KEY:VAL pairs.
    """

    version_major: int
    version_minor: int | None = None
    kind_code: str
    name: str
    size_mm: float
    params: dict[str, str] = Field(default_factory=dict)

    @field_validator("kind_code")
    @classmethod
    def check_kind(cls, value: str) -> str:
        """Validate K against ``[A-Z0-9-]{1,16}``."""
        if not KIND_RE.match(value):
            raise ValueError(f"invalid kind {value!r}: expected [A-Z0-9-]{{1,16}}")
        return value

    @field_validator("name")
    @classmethod
    def check_name(cls, value: str) -> str:
        """Validate N against ``[A-Z0-9-]{1,20}``."""
        if not NAME_RE.match(value):
            raise ValueError(f"invalid name {value!r}: expected [A-Z0-9-]{{1,20}}")
        return value

    @field_validator("size_mm")
    @classmethod
    def check_size(cls, value: float) -> float:
        """Validate the size range (20..1000) and at most one decimal."""
        if not MIN_SIZE_MM <= value <= MAX_SIZE_MM:
            raise ValueError(f"size {value} out of range {MIN_SIZE_MM:g}..{MAX_SIZE_MM:g} mm")
        if abs(value * 10 - round(value * 10)) > 1e-6:
            raise ValueError(f"size {value} has more than one decimal")
        return value

    @field_validator("params")
    @classmethod
    def check_params(cls, value: dict[str, str]) -> dict[str, str]:
        """Validate param keys (not P/K/N/S) and values."""
        for key, val in value.items():
            if key in RESERVED_KEYS or not PARAM_KEY_RE.match(key):
                raise ValueError(f"invalid param key {key!r}")
            if not PARAM_VALUE_RE.match(val):
                raise ValueError(f"invalid value {val!r} for param {key}")
        return value

    @model_validator(mode="after")
    def check_major(self) -> Self:
        """Only the supported major version is valid."""
        if self.version_major != SUPPORTED_MAJOR:
            raise ValueError(f"unsupported payload version {self.version_major}")
        return self

    @property
    def poi_kind(self) -> str:
        """POI marker kind: the KNOWN_KINDS value, else the lower-case K."""
        return KNOWN_KINDS.get(self.kind_code, self.kind_code.lower())

    @property
    def key(self) -> str:
        """Marker key ``<poi_kind>/<name>``."""
        return f"{self.poi_kind}/{self.name}"


class MarkerInfo(BaseModel):
    """POI sub-model for a detected marker (payload is the canonical text)."""

    model_config = {"extra": "forbid"}

    kind: str
    id: str
    key: str
    size_mm: float
    params: dict[str, str]
    payload: str
    version: int
    pose_samples: int = Field(ge=0)


def format_size(size_mm: float) -> str:
    """Format S without a trailing ``.0``.

    Args:
        size_mm: Symbol edge in mm.

    Returns:
        ``150`` or ``42.5``.
    """
    return f"{size_mm:.1f}".removesuffix(".0")


def encode_payload(payload: MarkerPayload) -> str:
    """Encode a payload to its one canonical text (P, K, N, S, then params sorted by key).

    Args:
        payload: A valid MarkerPayload.

    Returns:
        Text using only ALPHANUMERIC characters.
    """
    version = f"{PAYLOAD_PREFIX}{payload.version_major}"
    if payload.version_minor is not None:
        version += f".{payload.version_minor}"
    parts = [version, f"K:{payload.kind_code}", f"N:{payload.name}", f"S:{format_size(payload.size_mm)}"]
    parts += [f"{key}:{payload.params[key]}" for key in sorted(payload.params)]
    return "/".join(parts)


def decode_payload(text: str) -> MarkerPayload:
    """Decode payload text.

    Args:
        text: Text read from a marker.

    Returns:
        The MarkerPayload (unknown keys become params).

    Raises:
        PayloadError: naming the problem (bad P segment, unsupported version, missing/duplicate key, empty segment,
            bad character, bad size, invalid field).
    """
    bad = sorted({c for c in text if c not in ALPHANUMERIC})
    if bad:
        raise PayloadError(f"character {bad[0]!r} outside the alphanumeric set")
    segments = text.split("/")
    match = VERSION_RE.match(segments[0])
    if not match:
        raise PayloadError(f"missing or malformed P segment {segments[0]!r}")
    major = int(match.group(1))
    if major != SUPPORTED_MAJOR:
        raise PayloadError(f"unsupported payload version {major}")
    minor = int(match.group(2)) if match.group(2) is not None else None
    pairs: dict[str, str] = {}
    for segment in segments[1:]:
        if not segment:
            raise PayloadError("empty segment")
        key, sep, value = segment.partition(":")
        if not sep:
            raise PayloadError(f"segment {segment!r} has no colon")
        if not key:
            raise PayloadError(f"segment {segment!r} has an empty key")
        if key in pairs:
            raise PayloadError(f"duplicate key {key}")
        pairs[key] = value
    for required in ("K", "N", "S"):
        if required not in pairs:
            raise PayloadError(f"missing {required}")
    size_text = pairs.pop("S")
    if not SIZE_RE.match(size_text):
        raise PayloadError(f"malformed size {size_text!r}")
    try:
        return MarkerPayload(
            version_major=major,
            version_minor=minor,
            kind_code=pairs.pop("K"),
            name=pairs.pop("N"),
            size_mm=float(size_text),
            params=pairs,
        )
    except ValidationError as exc:
        raise PayloadError("; ".join(str(err["msg"]) for err in exc.errors())) from exc


def marker_info(payload: MarkerPayload, pose_samples: int = 1) -> MarkerInfo:
    """Build the POI MarkerInfo for a payload.

    Args:
        payload: The decoded payload.
        pose_samples: Number of pose samples averaged (>= 0).

    Returns:
        MarkerInfo with ``payload`` set to the canonical text.
    """
    return MarkerInfo(
        kind=payload.poi_kind,
        id=payload.name,
        key=payload.key,
        size_mm=payload.size_mm,
        params=dict(payload.params),
        payload=encode_payload(payload),
        version=payload.version_major,
        pose_samples=pose_samples,
    )
