"""JSON-file POI store: add/update/delete semantics, atomic persistence, revision counter. No ROS imports.

Photo POIs (kind "photo") are not kept here: commands on them are dispatched to the PhotoStore, so they never reach
poi.json, the /poi/list payload or its revision.
"""

import json
import logging
import os
import time
import uuid
from collections.abc import Callable
from pathlib import Path
from typing import Any

from pydantic import ValidationError
from ros2_common.photos import PHOTO_KIND, PhotoAdd, PhotoSelector, photo_as_poi

from .metrics import POI_COMMANDS, POI_COUNT, POI_SAVE_FAILURES
from .models import Command, Poi
from .photo_store import PhotoNotFoundError, PhotoStore, PhotoStoreError

LOGGER = logging.getLogger("poi_store")
IMMUTABLE_FIELDS = ("id", "created_at", "created_by")
MISSING_ID_MESSAGE = "poi.id is required"
NO_PHOTO_STORE_MESSAGE = "no photo store configured"

Result = tuple[bool, str, dict[str, Any] | None]


class PoiStore:
    """The POIs, their revision and the file they persist to.

    Attributes:
        path: JSON file.
        revision: Incremented on every successful change of the (non-photo) POIs.
        photos: Photo store that photo commands go to; None rejects them.
    """

    def __init__(
        self, path: Path | str, clock: Callable[[], float] = time.time, photos: PhotoStore | None = None
    ) -> None:
        """Load the store file (a corrupt one is moved aside) and start from it.

        Args:
            path: JSON file {"pois": [...], "revision": int}; created on the first change.
            clock: Unix-seconds time source.
            photos: Photo store for kind "photo" commands; None rejects them.
        """
        self.path = Path(path)
        self.clock = clock
        self.photos = photos
        self.revision = 0
        self.pois: dict[str, Poi] = {}
        self.load()
        POI_COUNT.set(len(self.pois))

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
            aside = self.path.with_name(f"{self.path.name}.corrupt-{int(self.clock())}")
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

    def apply(self, command: Command) -> Result:
        """Execute one command; when persisting fails (OSError) the in-memory change and revision are rolled back.

        Args:
            command: Parsed command.

        Returns:
            Result: ok, message, the resulting (add/update) or deleted POI.
        """
        pois_before, revision_before = dict(self.pois), self.revision
        try:
            if command.op == "clear":
                return self.op_clear(command.created_by, command.statuses)
            if command.op == "delete_photos":
                return self.op_delete_photos(command.selector)
            return getattr(self, "op_" + command.op)(command.poi)
        except PhotoStoreError as exc:
            return False, str(exc), None
        except ValidationError as exc:
            return False, "; ".join("{}: {}".format(".".join(map(str, e["loc"])), e["msg"]) for e in exc.errors()), None
        except OSError as exc:
            self.pois, self.revision = pois_before, revision_before
            POI_SAVE_FAILURES.inc()
            LOGGER.error("could not save %s (%s); %s rolled back", self.path, exc, command.op)
            return False, f"could not save POI store {self.path}: {exc}", None
        finally:
            POI_COUNT.set(len(self.pois))

    def op_add(self, fields: dict[str, Any]) -> Result:
        """Add a POI, assigning id and timestamps when absent.

        Args:
            fields: POI fields.

        Returns:
            Result: Result triple.
        """
        if fields.get("kind") == PHOTO_KIND:
            return self.add_photo(fields)
        now = self.clock()
        poi = Poi(**fields)
        poi.id = poi.id or uuid.uuid4().hex
        if poi.id in self.pois:
            return False, f"poi {poi.id} already exists", None
        poi.created_at = poi.created_at or now
        poi.updated_at = poi.updated_at or now
        self.pois[poi.id] = poi
        self.commit()
        return True, "added", poi.to_json_dict()

    def op_update(self, fields: dict[str, Any]) -> Result:
        """Merge changed fields into an existing POI and bump updated_at.

        Args:
            fields: id plus changed fields (id, created_at, created_by cannot change).

        Returns:
            Result: Result triple.
        """
        poi_id = fields.get("id")
        if not poi_id:
            return False, MISSING_ID_MESSAGE, None
        if self.is_photo(poi_id):
            return self.update_photo(poi_id, fields)
        current = self.pois.get(poi_id)
        if current is None:
            return False, f"unknown poi id {poi_id}", None
        merged = current.to_json_dict()
        merged.update({k: v for k, v in fields.items() if k not in IMMUTABLE_FIELDS})
        updated = Poi(**merged)
        updated.updated_at = self.clock()
        self.pois[poi_id] = updated
        self.commit()
        return True, "updated", updated.to_json_dict()

    def op_delete(self, fields: dict[str, Any]) -> Result:
        """Delete a POI.

        Args:
            fields: Must contain id.

        Returns:
            Result: Result triple with the deleted POI.
        """
        poi_id = fields.get("id")
        if not poi_id:
            return False, MISSING_ID_MESSAGE, None
        if self.is_photo(poi_id):
            return self.delete_photo(poi_id)
        removed = self.pois.pop(poi_id, None)
        if removed is None:
            return False, f"unknown poi id {poi_id}", None
        self.commit()
        self.strip_photo_links(poi_id)
        return True, "deleted", removed.to_json_dict()

    def is_photo(self, poi_id: str) -> bool:
        """Whether the photo store holds this id.

        Args:
            poi_id: Candidate id.

        Returns:
            bool: True for a photo id.
        """
        return self.photos is not None and poi_id in self.photos

    def check_poi_links(self, poi_ids: Any) -> str | None:
        """Check that photo links name existing non-photo POIs.

        Args:
            poi_ids: The poi_ids value of a photo add or update (non-lists are left to model validation).

        Returns:
            str | None: The error message, None when every id is fine.
        """
        if not isinstance(poi_ids, list):
            return None
        for poi_id in poi_ids:
            if not isinstance(poi_id, str):
                return f"poi_ids: entries must be POI id strings, got {poi_id!r}"
            if self.is_photo(poi_id):
                return f"poi_ids: {poi_id} is a photo; photos link only to non-photo POIs"
            if poi_id not in self.pois:
                return f"poi_ids: unknown poi id {poi_id}"
        return None

    def photo_failure(self, exc: OSError, op: str) -> Result:
        """Result of a photo store write that failed on disk.

        Args:
            exc: The error.
            op: The command op.

        Returns:
            Result: An ok=False result triple.
        """
        root = self.photos.root if self.photos is not None else None
        LOGGER.error("could not write photo store %s (%s); %s failed", root, exc, op)
        return False, f"could not write photo store {root}: {exc}", None

    def add_photo(self, fields: dict[str, Any]) -> Result:
        """Index a photo (its JPEG is already in the photo store directory).

        Args:
            fields: PhotoAdd fields plus kind "photo".

        Returns:
            Result: Result triple with the photo in POI form.
        """
        if self.photos is None:
            return False, NO_PHOTO_STORE_MESSAGE, None
        photo = PhotoAdd(**{k: v for k, v in fields.items() if k != "kind"})
        error = self.check_poi_links(photo.poi_ids)
        if error:
            return False, error, None
        try:
            rec = self.photos.add(photo)
        except OSError as exc:
            return self.photo_failure(exc, "add")
        return True, "added", photo_as_poi(rec)

    def update_photo(self, photo_id: str, fields: dict[str, Any]) -> Result:
        """Change a photo's name, note, status or POI links.

        Args:
            photo_id: Photo id (known to the photo store).
            fields: id plus changed fields.

        Returns:
            Result: Result triple with the photo in POI form.
        """
        assert self.photos is not None  # is_photo held
        error = self.check_poi_links(fields.get("poi_ids"))
        if error:
            return False, error, None
        try:
            rec = self.photos.update(photo_id, fields)
        except PhotoNotFoundError:
            return False, f"unknown poi id {photo_id}", None
        except OSError as exc:
            return self.photo_failure(exc, "update")
        return True, "updated", photo_as_poi(rec)

    def delete_photo(self, photo_id: str) -> Result:
        """Delete a photo's index line and its JPEG.

        Args:
            photo_id: Photo id (known to the photo store).

        Returns:
            Result: Result triple with the deleted photo in POI form.
        """
        assert self.photos is not None  # is_photo held
        try:
            rec = self.photos.delete(photo_id)
        except PhotoNotFoundError:
            return False, f"unknown poi id {photo_id}", None
        except OSError as exc:
            return self.photo_failure(exc, "delete")
        return True, "deleted", photo_as_poi(rec)

    def strip_photo_links(self, poi_id: str) -> None:
        """Remove a deleted POI from every photo's poi_ids; a failure is logged (the POI delete already persisted).

        Args:
            poi_id: The deleted POI id.
        """
        if self.photos is None:
            return
        try:
            self.photos.strip_links(poi_id)
        except OSError as exc:
            LOGGER.error("could not strip deleted poi %s from the photo links (%s)", poi_id, exc)

    def op_delete_photos(self, selector: PhotoSelector | None) -> Result:
        """Bulk delete photos (or report what a dry run would delete).

        Args:
            selector: Which photos (Command validation guarantees it is set).

        Returns:
            Result: Result triple; the third item is {removed, bytes_freed, missing} or {would_remove, bytes}.
        """
        if self.photos is None:
            return False, NO_PHOTO_STORE_MESSAGE, None
        assert selector is not None  # Command.check_op_arguments
        try:
            result = self.photos.delete_many(selector)
        except OSError as exc:
            return self.photo_failure(exc, "delete_photos")
        if selector.dry_run:
            return True, f"would delete {result['would_remove']} photo(s)", result
        return True, f"deleted {result['removed']} photo(s)", result

    def op_clear(self, created_by: str | None, statuses: list[str] | None = None) -> Result:
        """Delete the POIs created by one creator and strip them from photo links (photos themselves stay).

        Args:
            created_by: "agent" or "user".
            statuses: Only delete POIs whose status is in this list; None deletes every status, [] deletes nothing.

        Returns:
            Result: Result triple; the third item is {"removed": count}.
        """
        doomed = [
            poi_id
            for poi_id, poi in self.pois.items()
            if poi.created_by == created_by and (statuses is None or poi.status in statuses)
        ]
        for poi_id in doomed:
            del self.pois[poi_id]
        if doomed:
            self.commit()
        for poi_id in doomed:
            self.strip_photo_links(poi_id)
        return True, f"cleared {len(doomed)}", {"removed": len(doomed)}

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
            POI_COMMANDS.labels("invalid", "false").inc()
            return json.dumps({"request_id": request_id, "ok": False, "message": f"bad command: {exc}", "poi": None})
        ok, message, poi = self.apply(command)
        POI_COMMANDS.labels(command.op, str(ok).lower()).inc()
        return json.dumps({"request_id": command.request_id, "ok": ok, "message": message, "poi": poi})
