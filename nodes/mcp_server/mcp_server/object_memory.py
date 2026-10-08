"""Object memory: remembered objects are POIs (kind "object", created_by "agent") in poi_store (no ROS here)."""

import json
import logging
import math
import os
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any, Protocol

from pydantic import ValidationError

from .geometry import normalize_angle
from .perception_models import ObjectRecord, ObjectView

LOGGER = logging.getLogger("mcp_server.object_memory")
DEFAULT_NEAR_RADIUS_M = 1.0
OBJECT_KIND = "object"
OBJECT_CREATED_BY = "agent"
POI_FRAME = "map"
MAX_LABEL_LEN = 60  # poi_store caps POI names
MIGRATED_SUFFIX = ".migrated"


class PoiBackend(Protocol):
    """The poi_store access the object memory needs (RobotApi provides it)."""

    def poi_list(self) -> tuple[list[dict[str, Any]], int]:
        """Latest /poi/list (POIs, revision)."""
        ...

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        """Send a /poi/command and wait for its result."""
        ...


class ObjectStoreError(ValueError):
    """Invalid input or an unknown object (a ValueError so the tools report it to the model)."""


def label_key(label: str) -> str:
    """Normalised label used to decide whether two sightings are the same kind of object.

    Args:
        label (str): Label as given.

    Returns:
        str: Stripped, case-folded label.
    """
    return label.strip().casefold()


def record_from_poi(poi: dict[str, Any]) -> ObjectRecord:
    """Object view of an object POI; sighting fields a hand-made POI lacks fall back to its timestamps.

    Args:
        poi (dict[str, Any]): POI JSON of kind "object".

    Returns:
        ObjectRecord: The object.
    """
    created = float(poi.get("created_at") or 0.0)
    updated = float(poi.get("updated_at") or created)
    first_seen = float(poi.get("first_seen") or created)
    return ObjectRecord(
        id=str(poi["id"]),
        label=str(poi.get("name", "")),
        x=float(poi["x"]),
        y=float(poi["y"]),
        note=str(poi.get("note", "")),
        confidence=float(poi.get("confidence", 1.0)),
        first_seen=first_seen,
        last_seen=float(poi.get("last_seen") or updated or first_seen),
        times_seen=max(1, int(poi.get("times_seen") or 0)),
    )


class ObjectMemory:
    """Remembered objects, stored as object POIs in poi_store and read from its latched /poi/list."""

    def __init__(
        self,
        robot: PoiBackend,
        merge_radius_m: float = 0.25,
        clock: Callable[[], float] = time.time,
        legacy_path: Path | None = None,
    ) -> None:
        """Create the memory.

        Args:
            robot (PoiBackend): poi_list / poi_request provider.
            merge_radius_m (float): Same-label sightings within this distance are one object.
            clock (Callable[[], float]): Unix seconds.
            legacy_path (Path | None): Old objects.json; imported once as object POIs, then renamed to *.migrated.
        """
        self.robot = robot
        self.merge_radius_m = merge_radius_m
        self.clock = clock
        self.legacy_path = None if legacy_path is None else Path(legacy_path)
        self.lock = threading.RLock()
        self.legacy_done = legacy_path is None

    def import_legacy(self) -> None:
        """Import the old objects.json once (when poi_store is reachable) and rename it to <name>.migrated.

        An unreadable file is moved to <name>.corrupt-<ts>. When poi_store is down the RobotError propagates and the
        file stays for the next call.
        """
        with self.lock:
            path = self.legacy_path
            if self.legacy_done or path is None:
                return
            if not path.exists():
                self.legacy_done = True
                return
            try:
                data = json.loads(path.read_text())
                records = [ObjectRecord.model_validate(item) for item in data["objects"]]
            except (OSError, ValueError, KeyError, TypeError, ValidationError) as exc:
                aside = path.with_name(f"{path.name}.corrupt-{int(self.clock())}")
                LOGGER.warning("legacy object file %s unreadable (%s); moved to %s", path, exc, aside)
                os.replace(path, aside)
                self.legacy_done = True
                return
            for record in records:
                self.robot.poi_request("add", poi_fields(record))
            os.replace(path, path.with_name(path.name + MIGRATED_SUFFIX))
            LOGGER.info("imported %d legacy objects from %s as object POIs", len(records), path)
            self.legacy_done = True

    def object_pois(self) -> list[dict[str, Any]]:
        """Object POIs from the latest /poi/list (after the one-time legacy import).

        Returns:
            list[dict[str, Any]]: POI JSON dicts of kind "object", in store order.
        """
        self.import_legacy()
        return [p for p in self.robot.poi_list()[0] if p.get("kind") == OBJECT_KIND]

    def list(self) -> list[ObjectRecord]:
        """All remembered objects.

        Returns:
            list[ObjectRecord]: Objects in store order.
        """
        return [record_from_poi(p) for p in self.object_pois()]

    def remember(self, label: str, x: float, y: float, note: str = "", confidence: float = 0.7) -> ObjectRecord:
        """Remember a sighting: merge into the nearest same-label object within merge_radius_m, else add a new one.

        A merge moves the object to the weighted average of the old position (weight times_seen x its confidence) and
        the new one (weight = this confidence), bumps times_seen and last_seen, keeps the higher confidence and
        replaces the note only when a new one is given.

        Args:
            label (str): What it is.
            x (float): Map x (m).
            y (float): Map y (m).
            note (str): Free text.
            confidence (float): 0..1.

        Returns:
            ObjectRecord: The stored (new or merged) object.
        """
        label = label.strip()
        if not label:
            raise ObjectStoreError("label must not be empty")
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ObjectStoreError("x and y must be finite")
        if not (math.isfinite(confidence) and 0.0 <= confidence <= 1.0):
            raise ObjectStoreError("confidence must be within 0..1")
        now = self.clock()
        with self.lock:
            candidates = [
                (math.hypot(r.x - x, r.y - y), r)
                for r in self.list()
                if label_key(r.label) == label_key(label) and math.hypot(r.x - x, r.y - y) <= self.merge_radius_m
            ]
            if candidates:
                _, old = min(candidates, key=lambda item: item[0])
                w_old, w_new = old.times_seen * max(old.confidence, 1e-6), max(confidence, 1e-6)
                record = old.model_copy(
                    update={
                        "x": (old.x * w_old + x * w_new) / (w_old + w_new),
                        "y": (old.y * w_old + y * w_new) / (w_old + w_new),
                        "note": note or old.note,
                        "confidence": max(old.confidence, confidence),
                        "last_seen": now,
                        "times_seen": old.times_seen + 1,
                    }
                )
                changes = {
                    "id": record.id,
                    "x": record.x,
                    "y": record.y,
                    "note": record.note,
                    "confidence": record.confidence,
                    "first_seen": record.first_seen,
                    "last_seen": record.last_seen,
                    "times_seen": record.times_seen,
                }
                result = self.robot.poi_request("update", changes)
            else:
                record = ObjectRecord(
                    id="",
                    label=label,
                    x=x,
                    y=y,
                    note=note,
                    confidence=confidence,
                    first_seen=now,
                    last_seen=now,
                    times_seen=1,
                )
                result = self.robot.poi_request("add", poi_fields(record))
            stored = result.get("poi") or {}
            return record.model_copy(update={"id": str(stored.get("id", record.id))})

    def forget(self, object_id: str) -> ObjectRecord:
        """Remove an object (only object POIs; other POIs are removed with delete_poi).

        Args:
            object_id (str): Its id.

        Returns:
            ObjectRecord: The removed object.
        """
        with self.lock:
            match = next((p for p in self.object_pois() if p.get("id") == object_id), None)
            if match is None:
                raise ObjectStoreError(f"unknown object id {object_id!r}")
            self.robot.poi_request("delete", {"id": object_id})
            return record_from_poi(match)


def poi_fields(record: ObjectRecord) -> dict[str, Any]:
    """The poi_store "add" payload of an object.

    Args:
        record (ObjectRecord): The object (its id is left to the store when empty).

    Returns:
        dict[str, Any]: POI fields of kind "object", created_by "agent".
    """
    fields: dict[str, Any] = {
        "kind": OBJECT_KIND,
        "frame": POI_FRAME,
        "name": record.label[:MAX_LABEL_LEN],
        "x": record.x,
        "y": record.y,
        "note": record.note,
        "confidence": record.confidence,
        "first_seen": record.first_seen,
        "last_seen": record.last_seen,
        "times_seen": record.times_seen,
        "created_by": OBJECT_CREATED_BY,
    }
    return fields


def describe_objects(
    records: Sequence[ObjectRecord],
    pose: tuple[float, float, float] | None,
    label_contains: str | None = None,
    near: tuple[float, float] | None = None,
    radius_m: float | None = None,
) -> list[ObjectView]:
    """Filter objects and add distance/bearing from the robot, nearest first.

    Args:
        records (Sequence[ObjectRecord]): Remembered objects.
        pose (tuple[float, float, float] | None): Robot (x, y, yaw) in the map frame, None when unknown.
        label_contains (str | None): Case-insensitive substring of the label.
        near (tuple[float, float] | None): Keep only objects within radius_m of this map point.
        radius_m (float | None): Radius for `near` (default 1.0 m).

    Returns:
        list[ObjectView]: Sorted by distance to the robot (most recently seen first when the pose is unknown).
    """
    needle = label_contains.strip().casefold() if label_contains else ""
    radius = DEFAULT_NEAR_RADIUS_M if radius_m is None else radius_m
    views: list[ObjectView] = []
    for rec in records:
        if needle and needle not in rec.label.casefold():
            continue
        if near is not None and math.hypot(rec.x - near[0], rec.y - near[1]) > radius:
            continue
        view = ObjectView(**rec.model_dump())
        if pose is not None:
            view.distance_m = round(math.hypot(rec.x - pose[0], rec.y - pose[1]), 3)
            bearing = normalize_angle(math.atan2(rec.y - pose[1], rec.x - pose[0]) - pose[2])
            view.bearing_deg = round(math.degrees(bearing), 1)
        views.append(view)
    if pose is None:
        return sorted(views, key=lambda v: -v.last_seen)
    return sorted(views, key=lambda v: v.distance_m or 0.0)
