"""Object memory: remembered objects in the map frame, persisted atomically as JSON (no ROS)."""

import json
import logging
import math
import os
import threading
import time
import uuid
from collections.abc import Callable, Sequence
from pathlib import Path

from pydantic import ValidationError

from .geometry import normalize_angle
from .perception_models import ObjectRecord, ObjectView

LOGGER = logging.getLogger("mcp_server.object_memory")
DEFAULT_NEAR_RADIUS_M = 1.0


class ObjectStoreError(ValueError):
    """Invalid input or a store that cannot be read/written (a ValueError so the tools report it to the model)."""


def label_key(label: str) -> str:
    """Normalised label used to decide whether two sightings are the same kind of object.

    Args:
        label (str): Label as given.

    Returns:
        str: Stripped, case-folded label.
    """
    return label.strip().casefold()


class ObjectStore:
    """In-memory object list backed by a JSON file ({"objects": [...]}), written atomically on every change."""

    def __init__(
        self,
        path: Path,
        merge_radius_m: float = 0.25,
        clock: Callable[[], float] = time.time,
        new_id: Callable[[], str] = lambda: uuid.uuid4().hex,
    ) -> None:
        """Create the store; the file is read on first use.

        Args:
            path (Path): objects.json location.
            merge_radius_m (float): Same-label sightings within this distance are one object.
            clock (Callable[[], float]): Unix seconds.
            new_id (Callable[[], str]): Id factory.
        """
        self.path = Path(path)
        self.merge_radius_m = merge_radius_m
        self.clock = clock
        self.new_id = new_id
        self.lock = threading.Lock()
        self.objects: dict[str, ObjectRecord] | None = None

    def load(self) -> dict[str, ObjectRecord]:
        """Read the file once; an unreadable file is moved aside (objects.json.corrupt-<ts>) and the store starts empty.

        Returns:
            dict[str, ObjectRecord]: Objects by id.
        """
        if self.objects is not None:
            return self.objects
        self.objects = {}
        if not self.path.exists():
            return self.objects
        try:
            data = json.loads(self.path.read_text())
            records = [ObjectRecord.model_validate(item) for item in data["objects"]]
        except (OSError, ValueError, KeyError, TypeError, ValidationError) as exc:
            aside = self.path.with_name(f"{self.path.name}.corrupt-{int(self.clock())}")
            LOGGER.warning("object store %s unreadable (%s); moved to %s, starting empty", self.path, exc, aside)
            try:
                os.replace(self.path, aside)
            except OSError as move_exc:
                LOGGER.warning("cannot move %s aside: %s", self.path, move_exc)
            return self.objects
        self.objects = {r.id: r for r in records}
        return self.objects

    def save(self) -> None:
        """Write the whole list to a temp file next to the store and move it into place (os.replace)."""
        assert self.objects is not None
        payload = json.dumps({"objects": [r.model_dump() for r in self.objects.values()]}, indent=2)
        tmp = self.path.with_name(self.path.name + ".tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(tmp, "w") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp, self.path)
        except OSError as exc:
            tmp.unlink(missing_ok=True)
            raise ObjectStoreError(f"cannot write object memory {self.path}: {exc}") from exc

    def list(self) -> list[ObjectRecord]:
        """All remembered objects.

        Returns:
            list[ObjectRecord]: Copies in insertion order.
        """
        with self.lock:
            return [r.model_copy() for r in self.load().values()]

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
            objects = self.load()
            candidates = [
                (math.hypot(r.x - x, r.y - y), r)
                for r in objects.values()
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
            else:
                record = ObjectRecord(
                    id=self.new_id(),
                    label=label,
                    x=x,
                    y=y,
                    note=note,
                    confidence=confidence,
                    first_seen=now,
                    last_seen=now,
                    times_seen=1,
                )
            previous = dict(objects)
            objects[record.id] = record
            try:
                self.save()
            except ObjectStoreError:
                self.objects = previous
                raise
            return record.model_copy()

    def forget(self, object_id: str) -> ObjectRecord:
        """Remove an object.

        Args:
            object_id (str): Its id.

        Returns:
            ObjectRecord: The removed object.
        """
        with self.lock:
            objects = self.load()
            if object_id not in objects:
                raise ObjectStoreError(f"unknown object id {object_id!r}")
            previous = dict(objects)
            record = objects.pop(object_id)
            try:
                self.save()
            except ObjectStoreError:
                self.objects = previous
                raise
            return record


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
