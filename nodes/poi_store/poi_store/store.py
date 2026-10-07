"""JSON-file POI store: add/update/delete semantics, atomic persistence, revision counter. No ROS imports."""

import json
import logging
import os
import time
import uuid
from pathlib import Path
from typing import Any, Callable

from pydantic import ValidationError

from .models import Command, Poi

LOGGER = logging.getLogger("poi_store")
IMMUTABLE_FIELDS = ("id", "created_at", "created_by")
MISSING_ID_MESSAGE = "poi.id is required"


class PoiStore:
    """The POIs, their revision and the file they persist to.

    Attributes:
        path: JSON file.
        revision: Incremented on every successful change.
    """

    def __init__(self, path: Path | str, clock: Callable[[], float] = time.time) -> None:
        """Load the store file (a corrupt one is moved aside) and start from it.

        Args:
            path: JSON file {"pois": [...], "revision": int}; created on the first change.
            clock: Unix-seconds time source.
        """
        self.path = Path(path)
        self.clock = clock
        self.revision = 0
        self.pois: dict[str, Poi] = {}
        self.load()

    def load(self) -> None:
        """Read the file; on any parse/validation failure log, move it to .corrupt-<ts> and start empty."""
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text())
            pois = [Poi(**item) for item in data["pois"]]
            if not all(p.id for p in pois):
                raise ValueError("stored POI without id")
            self.revision = int(data.get("revision", 0))
            self.pois = {p.id: p for p in pois}
        except (OSError, ValueError, KeyError, TypeError, ValidationError) as exc:
            aside = self.path.with_name("%s.corrupt-%d" % (self.path.name, int(self.clock())))
            LOGGER.error("POI file %s is corrupt (%s); moving it to %s and starting empty", self.path, exc, aside)
            self.pois = {}
            self.revision = 0
            os.replace(self.path, aside)

    def list_pois(self) -> list[dict[str, Any]]:
        """All POIs in insertion order.

        Returns:
            list[dict[str, Any]]: Contract JSON dicts.
        """
        return [p.to_json_dict() for p in self.pois.values()]

    def list_json(self) -> str:
        """The /poi/list payload.

        Returns:
            str: JSON {"pois": [...], "revision": int}.
        """
        return json.dumps({"pois": self.list_pois(), "revision": self.revision})

    def save(self) -> None:
        """Write the store atomically (tmp file + os.replace)."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(json.dumps({"pois": self.list_pois(), "revision": self.revision}))
        os.replace(tmp, self.path)

    def commit(self) -> None:
        """Bump the revision and persist."""
        self.revision += 1
        self.save()

    def apply(self, command: Command) -> tuple[bool, str, dict[str, Any] | None]:
        """Execute one command; when persisting fails (OSError) the in-memory change and revision are rolled back.

        Args:
            command: Parsed command.

        Returns:
            tuple[bool, str, dict[str, Any] | None]: ok, message, the resulting (add/update) or deleted POI.
        """
        pois_before, revision_before = dict(self.pois), self.revision
        try:
            return getattr(self, "op_" + command.op)(command.poi)
        except ValidationError as exc:
            return False, "; ".join("%s: %s" % (".".join(map(str, e["loc"])), e["msg"]) for e in exc.errors()), None
        except OSError as exc:
            self.pois, self.revision = pois_before, revision_before
            LOGGER.error("could not save %s (%s); %s rolled back", self.path, exc, command.op)
            return False, "could not save POI store %s: %s" % (self.path, exc), None

    def op_add(self, fields: dict[str, Any]) -> tuple[bool, str, dict[str, Any] | None]:
        """Add a POI, assigning id and timestamps when absent.

        Args:
            fields: POI fields.

        Returns:
            tuple[bool, str, dict[str, Any] | None]: Result triple.
        """
        now = self.clock()
        poi = Poi(**fields)
        poi.id = poi.id or uuid.uuid4().hex
        if poi.id in self.pois:
            return False, "poi %s already exists" % poi.id, None
        poi.created_at = poi.created_at or now
        poi.updated_at = poi.updated_at or now
        self.pois[poi.id] = poi
        self.commit()
        return True, "added", poi.to_json_dict()

    def op_update(self, fields: dict[str, Any]) -> tuple[bool, str, dict[str, Any] | None]:
        """Merge changed fields into an existing POI and bump updated_at.

        Args:
            fields: id plus changed fields (id, created_at, created_by cannot change).

        Returns:
            tuple[bool, str, dict[str, Any] | None]: Result triple.
        """
        poi_id = fields.get("id")
        if not poi_id:
            return False, MISSING_ID_MESSAGE, None
        current = self.pois.get(poi_id)
        if current is None:
            return False, "unknown poi id %s" % poi_id, None
        merged = current.to_json_dict()
        merged.update({k: v for k, v in fields.items() if k not in IMMUTABLE_FIELDS})
        updated = Poi(**merged)
        updated.updated_at = self.clock()
        self.pois[poi_id] = updated
        self.commit()
        return True, "updated", updated.to_json_dict()

    def op_delete(self, fields: dict[str, Any]) -> tuple[bool, str, dict[str, Any] | None]:
        """Delete a POI.

        Args:
            fields: Must contain id.

        Returns:
            tuple[bool, str, dict[str, Any] | None]: Result triple with the deleted POI.
        """
        poi_id = fields.get("id")
        if not poi_id:
            return False, MISSING_ID_MESSAGE, None
        removed = self.pois.pop(poi_id, None)
        if removed is None:
            return False, "unknown poi id %s" % poi_id, None
        self.commit()
        return True, "deleted", removed.to_json_dict()

    def handle_message(self, raw: str) -> str:
        """Process one /poi/command payload.

        Args:
            raw: JSON command text.

        Returns:
            str: The /poi/result JSON {request_id, ok, message, poi}.
        """
        request_id = ""
        try:
            data = json.loads(raw)
            if isinstance(data, dict):
                request_id = str(data.get("request_id", ""))
            command = Command(**data)
        except (ValueError, TypeError, ValidationError) as exc:
            return json.dumps({"request_id": request_id, "ok": False, "message": "bad command: %s" % exc, "poi": None})
        ok, message, poi = self.apply(command)
        return json.dumps({"request_id": command.request_id, "ok": ok, "message": message, "poi": poi})
