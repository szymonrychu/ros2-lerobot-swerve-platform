---
title: Marker payload codec
status: escalated
repo: ros2-lerobot-sverve-platform
value: "Every marker carries one canonical, versioned text that the generator, the robot and the POI store read the same way."
files: [shared/ros2_common/marker_payload.py, tests/test_shared_marker_payload.py, shared/README.md]
verify: "uv run pytest tests/test_shared_marker_payload.py -q && uv run poe lint"
depends_on: []
owns: [marker-payload]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/marker-and-calibration-board--marker-payload-codec
branch: marker-and-calibration-board--marker-payload-codec
verified_head: c72ec5cb52aa8c31a7273dc834a95e3202aa9aa2
run: marker-and-calibration-board
provides: []
acs_for_audit: [a1, a3]
process_gaps: []
---

# Marker payload codec

## Requirement

- `shared/ros2_common/marker_payload.py` (pydantic v2, no ROS, constants at the top), the format of design D2/D3:
  - Constants: `PAYLOAD_PREFIX = "P"`, `SUPPORTED_MAJOR = 1`, `ALPHANUMERIC = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:"`, `KNOWN_KINDS = {"BASE": "base", "CHARGER": "charger", "DROPOFF": "drop_off", "BUTTON": "button", "SWITCH": "switch", "TOOLSTATION": "tool_station"}`, `MIN_SIZE_MM = 20.0`, `MAX_SIZE_MM = 1000.0`.
  - `class MarkerPayload(BaseModel)`: `version_major: int`, `version_minor: int | None`, `kind_code: str` (`[A-Z0-9-]{1,16}`), `name: str` (`[A-Z0-9-]{1,20}`), `size_mm: float` (20..1000, at most one decimal), `params: dict[str, str]` (keys `[A-Z0-9]{1,8}` not in P/K/N/S, values `[A-Z0-9 $%*+.-]{0,24}`); properties `poi_kind` (KNOWN_KINDS value, else `kind_code.lower()`), `key` (`f"{poi_kind}/{name}"`).
  - `encode_payload(payload: MarkerPayload) -> str`: `P<major>[.<minor>]/K:<K>/N:<N>/S:<S>` then params sorted by key as `/<KEY>:<VAL>`; `S` printed without a trailing `.0` (`150`, `42.5`); the result only uses ALPHANUMERIC characters.
  - `decode_payload(text: str) -> MarkerPayload`: raises `PayloadError(ValueError)` with a message naming the problem for: a missing or malformed `P` segment, major != 1 (`unsupported payload version 2`), a missing K, N or S, a duplicate key, an empty segment, a character outside ALPHANUMERIC, an out-of-range or malformed S. Unknown keys become params. `decode_payload(encode_payload(p)) == p` for every valid p.
  - `class MarkerInfo(BaseModel)` (extra forbidden): `kind: str`, `id: str`, `key: str`, `size_mm: float`, `params: dict[str, str]`, `payload: str`, `version: int`, `pose_samples: int >= 0`; `marker_info(payload: MarkerPayload, pose_samples: int = 1) -> MarkerInfo` builds it (payload = canonical text).
- Tests first in `tests/test_shared_marker_payload.py` (import as `shared.ros2_common.marker_payload`), seen failing: round trip for every known kind and for params; `P1/K:CHARGER/N:DOCK-1/S:150` decodes to kind charger, key `charger/DOCK-1`, size 150.0; canonical order (params given out of order encode sorted); `P1.3/...` accepted, `P2/...` rejected naming version 2; each malformed case above raises PayloadError with its reason; unknown kind `K:LAMP` maps to `lamp`; lower-case input rejected (outside the alphanumeric set); MarkerInfo built from a payload carries the canonical text.
- `shared/README.md`: a `marker_payload` section with the grammar, the kind table, the size definition (symbol edge excluding the quiet zone, design D3) and the marker frame axes.

## Acceptance conditions

- a1: encode and decode round-trip every known kind and params in canonical order [evidence: test: tests/test_shared_marker_payload.py]
- a2: unknown major versions and every malformed payload are rejected with a named reason [evidence: test: tests/test_shared_marker_payload.py]
- a3: unknown kinds map to their lower-case POI kind and the key is kind/name [evidence: test: tests/test_shared_marker_payload.py]
- a4: MarkerInfo is built from a decoded payload [evidence: symbol: shared/ros2_common/marker_payload.py:marker_info]
- a5: shared README documents the grammar, kinds, size and frame [evidence: command: grep -q 'marker_payload' shared/README.md && grep -q 'quiet zone' shared/README.md]

## Files

- `shared/ros2_common/marker_payload.py`
- `tests/test_shared_marker_payload.py`
- `shared/README.md`

## Interfaces

- `shared/ros2_common/marker_payload.py`: `MarkerPayload`, `PayloadError`, `encode_payload`, `decode_payload`, `MarkerInfo`, `marker_info`, `KNOWN_KINDS` (part `marker-payload`).

## Expected outcome

`uv run pytest tests/test_shared_marker_payload.py -q` passes; any component can turn a QR text into a validated payload and back.

## Steps

- Write `tests/test_shared_marker_payload.py`; run `uv run pytest tests/test_shared_marker_payload.py -q`; see it fail on the missing module.
- Implement `shared/ros2_common/marker_payload.py`; rerun until green; `uv run poe lint`.
- Update `shared/README.md`; commit `feat(shared): marker payload codec`.

## Notes

- Precedent for a shared pydantic model used by poi_store: `shared/ros2_common/photos.py:PhotoSelector`.
- No regex-free hand parser games: a split on `/` then `:` (first colon) is enough; validation lives in the pydantic fields.
- Do not add a POI `category`; the agent-skills epic adds it later (design D11).
