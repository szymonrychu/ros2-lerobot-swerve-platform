"""Named map operations of map_manager: save, load (with relocalisation) and delete, behind ports (no ROS imports).

The rclpy layer implements the ports (slam_toolbox services, poi_store topics, TF, ``/map``, ``/scan_filtered``,
``/pose``) and calls :meth:`MapManager.handle` from a worker thread. One operation runs at a time.

Load sequence (design D5, the order is part of the contract): validate the bundle; check the laser frame; autosave
the current map; status ``loading`` (localised false); refine the pose hint with the candidate search; pause new
measurements (toggle-safe); ``deserialize_map`` with ``START_AT_GIVEN_POSE`` at the refined pose; wait for the first
``/pose`` after the call; replace the POI set with the bundle's; collect agreement samples and judge; unpause only
when localised. Facts behind it: slam_toolbox never judges the near-region match, an unreadable file still answers
success (so the files are checked first and the load is confirmed by ``/pose``), and the first scan after the
deserialize is processed even while new measurements are paused.
"""

from __future__ import annotations

import logging
import shutil
import threading
import time
import uuid
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from ros2_common.maps import Localisation, LocalisationState, MapCommand, MapResult, MapStatus, MapSummary, Pose2D
from ros2_common.scan_agreement import GridMap, prepare_map

from map_manager.bundle import (
    POSEGRAPH_BASE,
    BundleExists,
    BundleInvalid,
    BundleMeta,
    BundleNotFound,
    BundleStore,
    LoadedBundle,
    MapInfo,
)
from map_manager.config import MapManagerConfig
from map_manager.relocalise import LocalisationJudge, SearchResult, search_candidates

START_AT_GIVEN_POSE = 2
UNCONFIRMED_REASON = "load not confirmed by slam_toolbox"
DESERIALIZE_FAILED_REASON = "slam_toolbox deserialize_map failed"
NO_SCAN_REASON = "no scan received; deserializing at the hint"
PAUSE_FAILED_REASON = "could not pause slam_toolbox new measurements"
NO_POSE_REASON = "no last known pose"
SCRATCH_DIR = ".relocalise"
UNPAUSE_FAILED_REASON = "unpausing new measurements failed, slam_toolbox stays paused"

LOGGER = logging.getLogger(__name__)


class SlamPort(Protocol):
    """slam_toolbox services and its ``/pose`` output."""

    def serialize(self, base: str) -> bool:
        """Serialize the live pose graph to ``<base>.posegraph`` and ``<base>.data``; True on success."""
        ...

    def deserialize(self, base: str, pose: Pose2D) -> bool:
        """Call deserialize_map on ``<base>`` with match type ``START_AT_GIVEN_POSE`` (2) and pose; True on success."""
        ...

    def query_paused(self) -> bool | None:
        """Read ``paused_new_measurements``; None when it cannot be read."""
        ...

    def toggle_pause(self) -> bool:
        """Call the Pause toggle; True when the service answered with status true."""
        ...

    def wait_pose_after(self, stamp: float, timeout_s: float) -> Pose2D | None:
        """First ``/pose`` stamped after stamp within timeout_s; None when none arrived."""
        ...

    def pose_covariance(self) -> list[float] | None:
        """Covariance diagonal ``[xx, yy, yawyaw]`` of the last ``/pose``; None before any."""
        ...


class PoiPort(Protocol):
    """poi_store's list and its replace op."""

    def latest_list(self) -> tuple[list[dict[str, Any]], int] | None:
        """Latest ``/poi/list`` POIs and revision; None before the first list."""
        ...

    def replace(self, pois: list[dict[str, Any]]) -> tuple[bool, str]:
        """Swap the live non-photo POI set; (ok, message) from poi_store's answer."""
        ...


class SensorPort(Protocol):
    """TF, ``/map``, ``/scan_filtered`` and agreement samples; heavy subscriptions live only during an operation."""

    def robot_pose(self) -> Pose2D | None:
        """TF map <- base_link younger than ``max_pose_age_s``; None otherwise."""
        ...

    def latest_grid(self) -> GridMap | None:
        """Latest ``/map``; None before any."""
        ...

    def scan_points_base(self) -> np.ndarray | None:
        """(N, 2) endpoints of the latest scan in the base frame; None without a scan or TF."""
        ...

    def agreement_samples(self, window_s: float) -> list[float]:
        """Collect scan-to-map agreement ratios for window_s and return them."""
        ...

    def laser_frame(self) -> str | None:
        """Frame id of the live scan; None before any."""
        ...

    def start_operation(self) -> None:
        """Create the TF listener and the ``/map``, scan and ``/pose`` subscriptions."""
        ...

    def end_operation(self) -> None:
        """Destroy what start_operation created."""
        ...


class OperationFailed(Exception):
    """An operation step refused; the message is the result message."""


@dataclass(frozen=True)
class LoadPlan:
    """What a load hands slam_toolbox, fixed before the deserialize call.

    Attributes:
        command (MapCommand): The load command.
        bundle (LoadedBundle): The validated bundle.
        hint (Pose2D): Pose hint (request pose or saved last pose).
        target (Pose2D): Pose handed to slam_toolbox (refined, or the hint when the search is not ok).
        search (SearchResult | None): Candidate search result (None without a scan).
        search_reason (str): Its reason.
        autosaved (bool): The current map was autosaved first.
    """

    command: MapCommand
    bundle: LoadedBundle
    hint: Pose2D
    target: Pose2D
    search: SearchResult | None
    search_reason: str
    autosaved: bool

    def data(self, state: LocalisationState, active_map: str | None, load_seq: int) -> dict[str, Any]:
        """Result ``data`` of this load before the slam confirmation, verdict and POI replace are known.

        Args:
            state (LocalisationState): Localisation state after the load.
            active_map (str | None): Active map after the load.
            load_seq (int): Load sequence after the load.

        Returns:
            dict[str, Any]: The result data; ``ratio``, ``slam_pose``, ``covariance`` None, ``pois_replaced`` False.
        """
        return {
            "state": state,
            "ratio": None,
            "pose": self.target.model_dump(),
            "hint": self.hint.model_dump(),
            "slam_pose": None,
            "search_ratio": self.search.ratio if self.search is not None else None,
            "search_ok": self.search is not None and self.search.ok,
            "search_reason": self.search_reason,
            "ambiguous": self.search is not None and self.search.ambiguous,
            "covariance": None,
            "autosaved": self.autosaved,
            "pois_replaced": False,
            "poi_count": len(self.bundle.meta.pois),
            "active_map": active_map,
            "load_seq": load_seq,
        }


class MapManager:
    """Runs save, load and delete of named maps one at a time and keeps the status.

    Args:
        config: Node settings (MapManagerConfig).
        store: Bundle store under ``config.maps_root`` (BundleStore).
        slam: slam_toolbox port (SlamPort).
        poi: poi_store port (PoiPort).
        sensors: TF / map / scan port (SensorPort).
        clock: Wall clock in seconds (Callable[[], float]).
        on_change: Called after every status change, e.g. to publish status and localised (Callable[[], None]).
    """

    def __init__(
        self,
        config: MapManagerConfig,
        store: BundleStore,
        slam: SlamPort,
        poi: PoiPort,
        sensors: SensorPort,
        clock: Callable[[], float] = time.time,
        on_change: Callable[[], None] | None = None,
    ) -> None:
        self.config = config
        self.store = store
        self.slam = slam
        self.poi = poi
        self.sensors = sensors
        self.clock = clock
        self.on_change = on_change
        self.operation_lock = threading.Lock()
        self.state_lock = threading.RLock()
        self.running = ""
        self.maps: list[MapSummary] = store.list()
        self.active_map: str | None = None
        self.load_seq = 0
        self.localisation = Localisation(state="unknown")
        self.localised_value: bool | None = None
        self.before_load: tuple[Localisation, bool | None] | None = None
        self.paused: bool | None = None
        self.pose_ring: deque[tuple[float, Pose2D]] = deque()

    # ------------------------------------------------------------------ state

    def status(self) -> MapStatus:
        """Snapshot of the status.

        Returns:
            MapStatus: Maps, active map, load sequence and localisation.
        """
        with self.state_lock:
            return MapStatus(
                maps=list(self.maps),
                active_map=self.active_map,
                load_seq=self.load_seq,
                localisation=self.localisation,
            )

    def localised(self) -> bool | None:
        """Value for ``/map_manager/localised``.

        Returns:
            bool | None: None until a load reached ``loading``; then False while loading or not localised, True when
            localised.
        """
        with self.state_lock:
            return self.localised_value

    def notify(self) -> None:
        """Tell the owner that the status changed."""
        if self.on_change is not None:
            self.on_change()

    def refresh_maps(self) -> None:
        """Re-read the bundle list and notify."""
        maps = self.store.list()
        with self.state_lock:
            self.maps = maps
        self.notify()

    def set_localisation(self, localisation: Localisation, localised: bool | None) -> None:
        """Replace the localisation and the localised value, then notify.

        Args:
            localisation (Localisation): New localisation.
            localised (bool | None): New ``/map_manager/localised`` value.
        """
        with self.state_lock:
            self.localisation = localisation
            self.localised_value = localised
        self.notify()

    def complete_load(self, name: str, localisation: Localisation) -> None:
        """Record a completed load: active map, load sequence, verdict.

        Args:
            name (str): Loaded map.
            localisation (Localisation): Verdict.
        """
        with self.state_lock:
            self.active_map = name
            self.load_seq += 1
            self.before_load = None
        self.set_localisation(localisation, localisation.state == "localised")

    # ------------------------------------------------------------------ dispatch

    def handle(self, command: MapCommand) -> MapResult:
        """Run one command; a second command while one runs is refused.

        Args:
            command (MapCommand): Validated command.

        Returns:
            MapResult: The answer (a load answers after the verdict).
        """
        if not self.operation_lock.acquire(blocking=False):
            return MapResult(request_id=command.request_id, ok=False, message=f"busy: {self.running or 'operation'}")
        try:
            self.running = f"{command.op} {command.name}".strip()
            handlers: dict[str, Callable[[MapCommand], MapResult]] = {
                "save": self.save,
                "load": self.load,
                "delete": self.delete,
                "relocalise": self.relocalise,
            }
            handler = handlers.get(command.op)
            if handler is None:
                return MapResult(
                    request_id=command.request_id, ok=False, message=f"op {command.op} is not handled by map_manager"
                )
            try:
                return handler(command)
            except Exception as exc:
                LOGGER.exception("map_manager %s failed", self.running)
                self.restore_before_load()
                return MapResult(request_id=command.request_id, ok=False, message=f"{command.op} failed: {exc}")
        finally:
            self.running = ""
            self.operation_lock.release()

    def restore_before_load(self) -> None:
        """Put back the localisation from before a load that stopped before slam_toolbox was touched."""
        with self.state_lock:
            snapshot = self.before_load
            self.before_load = None
        if snapshot is not None:
            self.set_localisation(*snapshot)

    # ------------------------------------------------------------------ save

    def save(self, command: MapCommand) -> MapResult:
        """Save the live map, POIs and pose as a named bundle.

        Args:
            command (MapCommand): ``op: save`` with name, overwrite and note.

        Returns:
            MapResult: ok with the map summary, else the reason; nothing is written on failure.
        """
        self.sensors.start_operation()
        try:
            summary = self.write_bundle(command.name, command.note, command.overwrite)
        except OperationFailed as exc:
            LOGGER.warning("map_manager save %s refused: %s", command.name, exc)
            return MapResult(request_id=command.request_id, ok=False, message=str(exc))
        finally:
            self.sensors.end_operation()
        return MapResult(
            request_id=command.request_id,
            ok=True,
            message=f"saved map {command.name} with {summary.poi_count} POIs",
            data=summary.model_dump(),
        )

    def write_bundle(self, name: str, note: str, overwrite: bool) -> MapSummary:
        """Serialize slam_toolbox into a temp directory and commit it with the grid, POIs and pose.

        Args:
            name (str): Map name.
            note (str): Free text note.
            overwrite (bool): Replace an existing bundle.

        Returns:
            MapSummary: Summary of the stored map.

        Raises:
            OperationFailed: Name taken, an input missing (all named), serialize or commit failed.
        """
        if not overwrite and self.store.path_of(name).exists():
            raise OperationFailed(f"map {name!r} already exists; save with overwrite to replace it")
        pose = self.sensors.robot_pose()
        grid = self.sensors.latest_grid()
        listing = self.poi.latest_list()
        laser = self.sensors.laser_frame()
        inputs = (("robot pose", pose), ("map grid", grid), ("POI list", listing), ("laser frame", laser))
        missing = [label for label, value in inputs if value is None]
        if missing or pose is None or grid is None or listing is None or laser is None:
            raise OperationFailed(f"cannot save map {name!r}: missing {', '.join(missing)}")
        pois, revision = listing
        tmp = self.store.begin(name)
        try:
            serialized = self.slam.serialize(str(tmp / POSEGRAPH_BASE))
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        if not serialized:
            shutil.rmtree(tmp, ignore_errors=True)
            raise OperationFailed(f"cannot save map {name!r}: slam_toolbox serialize_map failed")
        height, width = grid.cells.shape
        meta = BundleMeta(
            version=1,
            name=name,
            saved_at=self.clock(),
            note=note,
            last_pose=pose,
            pois=pois,
            poi_revision=revision,
            map_info=MapInfo(
                resolution=grid.resolution,
                width=width,
                height=height,
                origin_x=grid.origin_x,
                origin_y=grid.origin_y,
                origin_yaw=grid.origin_yaw,
            ),
            laser_frame=laser,
        )
        try:
            summary = self.store.commit(tmp, name, meta, grid, overwrite)
        except (BundleExists, BundleInvalid, OSError) as exc:
            raise OperationFailed(f"cannot save map {name!r}: {exc}") from exc
        self.refresh_maps()
        return summary

    # ------------------------------------------------------------------ load

    def load(self, command: MapCommand) -> MapResult:
        """Load a named map, replace the POIs and relocalise (design D5, D6).

        Args:
            command (MapCommand): ``op: load`` with name and optional pose hint.

        Returns:
            MapResult: ok only when localised and the POIs were replaced; ``data`` holds the verdict details.
        """
        try:
            bundle = self.store.load(command.name)
        except (BundleNotFound, BundleInvalid) as exc:
            return MapResult(request_id=command.request_id, ok=False, message=f"cannot load: {exc}")
        self.sensors.start_operation()
        try:
            return self.run_load(command, bundle)
        except OperationFailed as exc:
            LOGGER.warning("map_manager load %s refused: %s", command.name, exc)
            return MapResult(request_id=command.request_id, ok=False, message=str(exc))
        finally:
            self.sensors.end_operation()

    def run_load(self, command: MapCommand, bundle: LoadedBundle) -> MapResult:
        """Checks, autosave, hint refinement and pause; then :meth:`finish_load`.

        Args:
            command (MapCommand): The load command.
            bundle (LoadedBundle): The validated bundle.

        Returns:
            MapResult: The load answer.

        Raises:
            OperationFailed: A check, the autosave or the pause failed; slam_toolbox's map is untouched.
        """
        name = command.name
        meta = bundle.meta
        laser = self.sensors.laser_frame()
        if laser is None:
            raise OperationFailed(f"cannot load {name!r}: live laser frame unknown, the map needs {meta.laser_frame!r}")
        if laser != meta.laser_frame:
            raise OperationFailed(
                f"cannot load {name!r}: laser frame {laser!r} differs from the map's {meta.laser_frame!r}"
            )
        autosaved = False
        if self.config.autosave_before_load and name != self.config.autosave_name:
            try:
                self.write_bundle(self.config.autosave_name, f"before loading {name}", overwrite=True)
            except OperationFailed as exc:
                raise OperationFailed(f"autosave failed: {exc}") from exc
            autosaved = True
        with self.state_lock:
            self.before_load = (self.localisation, self.localised_value)
        self.set_localisation(
            Localisation(state="loading", message=f"loading map {name}", updated_at=self.clock()), False
        )
        hint = command.pose or meta.last_pose
        search, search_reason = self.refine(bundle.grid, hint)
        target = search.pose if search is not None and search.ok else hint
        if not self.set_paused(True):
            self.restore_before_load()
            raise OperationFailed(f"load of {name!r} aborted: {PAUSE_FAILED_REASON}")
        plan = LoadPlan(command, bundle, hint, target, search, search_reason, autosaved)
        started = self.clock()
        try:
            deserialized = self.slam.deserialize(str(bundle.posegraph_base), target)
        except Exception as exc:
            LOGGER.exception("map_manager load %s: deserialize_map raised", name)
            return self.refuse_load(plan, f"{DESERIALIZE_FAILED_REASON}: {exc}")
        if not deserialized:
            return self.refuse_load(plan, DESERIALIZE_FAILED_REASON)
        try:
            return self.finish_load(plan, started)
        except Exception as exc:
            LOGGER.exception("map_manager load %s failed after the pause", name)
            reason = f"load failed: {exc}"
            self.complete_load(name, Localisation(state="not_localised", message=reason, updated_at=self.clock()))
            return MapResult(request_id=command.request_id, ok=False, message=f"map {name}: {reason}")

    def refine(self, grid: GridMap, hint: Pose2D) -> tuple[SearchResult | None, str]:
        """Candidate search of the live scan against the bundle grid around the hint.

        Args:
            grid (GridMap): Bundle occupancy grid.
            hint (Pose2D): Pose hint.

        Returns:
            tuple[SearchResult | None, str]: The search result (None without a scan) and its reason.
        """
        points = self.sensors.scan_points_base()
        if points is None or len(points) == 0:
            return None, NO_SCAN_REASON
        settings = self.config.search
        prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)
        result = search_candidates(prepared, points, hint, settings)
        return result, result.reason

    def refuse_load(self, plan: LoadPlan, reason: str) -> MapResult:
        """Answer a load slam_toolbox did not take: it keeps its previous map, so the load is not completed.

        ``active_map``, ``load_seq`` and the live POIs stay as they were; new measurements stay paused and the
        robot reports ``not_localised`` (no localisation is claimed without a measured match).

        Args:
            plan (LoadPlan): The refused load.
            reason (str): Why slam_toolbox did not take the map.

        Returns:
            MapResult: ``ok: false`` naming the reason.
        """
        name = plan.command.name
        with self.state_lock:
            self.before_load = None
            active_map, load_seq = self.active_map, self.load_seq
        self.set_localisation(
            Localisation(state="not_localised", message=f"map {name} not loaded: {reason}", updated_at=self.clock()),
            False,
        )
        return MapResult(
            request_id=plan.command.request_id,
            ok=False,
            message=f"map {name} not loaded: {reason}; active map stays {active_map or 'none'}, POIs unchanged",
            data=plan.data("not_localised", active_map, load_seq),
        )

    def finish_load(self, plan: LoadPlan, started: float) -> MapResult:
        """After slam_toolbox took the map: confirm, replace the POIs, verify, unpause when localised, record.

        Args:
            plan (LoadPlan): The load.
            started (float): Clock reading just before the deserialize call.

        Returns:
            MapResult: The load answer.
        """
        name = plan.command.name
        confirmed = self.slam.wait_pose_after(started, self.config.load_timeout_s)
        covariance = self.slam.pose_covariance() if confirmed is not None else None
        pois_ok, poi_message = self.poi.replace(plan.bundle.meta.pois)
        state: LocalisationState
        ratio: float | None = None
        if confirmed is None:
            state, reason = "not_localised", UNCONFIRMED_REASON
        else:
            judge = LocalisationJudge(localised_ratio=self.config.localised_ratio, min_samples=self.config.min_samples)
            for sample in self.sensors.agreement_samples(self.config.verify_window_s):
                judge.add(sample)
            ambiguous = plan.search is not None and plan.search.ambiguous
            state, ratio, reason = judge.verdict(ambiguous=ambiguous)
        notes = [reason]
        if state == "localised" and not self.set_paused(False):
            notes.append(UNPAUSE_FAILED_REASON)
        if not pois_ok:
            notes.append(f"POIs not replaced: {poi_message}")
        self.complete_load(
            name,
            Localisation(state=state, ratio=ratio, pose=confirmed, message=reason, updated_at=self.clock()),
        )
        data = plan.data(state, name, self.load_seq)
        data.update(
            ratio=ratio,
            slam_pose=confirmed.model_dump() if confirmed is not None else None,
            covariance=covariance,
            pois_replaced=pois_ok,
        )
        return MapResult(
            request_id=plan.command.request_id,
            ok=state == "localised" and pois_ok,
            message=f"map {name} loaded: {state} ({'; '.join(notes)})",
            data=data,
        )

    def set_paused(self, desired: bool) -> bool:
        """Bring slam_toolbox's new-measurement pause to desired through the toggle service (as map_guard).

        Reads the state (falling back to the tracked state when unreadable), toggles only when it differs, re-reads.

        Args:
            desired (bool): True to pause, False to resume.

        Returns:
            bool: The state is desired (or the toggle answered and the re-read is unreadable).
        """
        actual = self.slam.query_paused()
        current = actual if actual is not None else self.paused
        if current == desired:
            self.paused = desired
            return True
        if not self.slam.toggle_pause():
            LOGGER.warning("map_manager: pause toggle failed; new measurements stay %s", current)
            self.paused = current
            return False
        verified = self.slam.query_paused()
        if verified is not None and verified != desired:
            LOGGER.warning("map_manager: pause toggle answered but paused is still %s", verified)
            self.paused = verified
            return False
        self.paused = desired
        return True

    # ------------------------------------------------------------------ delete

    def delete(self, command: MapCommand) -> MapResult:
        """Delete a bundle (the map and its POIs); deleting the active map also empties the live POI set (U7).

        Args:
            command (MapCommand): ``op: delete`` with name.

        Returns:
            MapResult: ok when deleted (and, for the active map, the live POIs emptied).
        """
        name = command.name
        try:
            self.store.delete(name)
        except BundleNotFound as exc:
            return MapResult(request_id=command.request_id, ok=False, message=f"cannot delete: {exc}")
        was_active = self.active_map == name
        data: dict[str, Any] = {"name": name, "was_active": was_active, "pois_cleared": None}
        ok, message = True, f"deleted map {name} and its POIs"
        if was_active:
            with self.state_lock:
                self.active_map = None
            cleared, poi_message = self.poi.replace([])
            data["pois_cleared"] = cleared
            if cleared:
                message += "; it was the active map, live POIs emptied"
            else:
                ok = False
                message += f"; it was the active map but emptying the live POIs failed: {poi_message}"
        self.refresh_maps()
        return MapResult(request_id=command.request_id, ok=ok, message=message, data=data)

    # ------------------------------------------------------------------ relocalise

    def record_pose(self) -> None:
        """Append the current robot pose to the ring and drop entries older than ``pose_ring_s``.

        Nothing is recorded when the pose is unavailable (no placeholder).
        """
        pose = self.sensors.robot_pose()
        if pose is None:
            return
        now = self.clock()
        with self.state_lock:
            self.pose_ring.append((now, pose))
            while self.pose_ring and self.pose_ring[0][0] < now - self.config.pose_ring_s:
                self.pose_ring.popleft()

    def ring_hint(self, since: float | None) -> Pose2D | None:
        """Newest ring pose, or the newest taken at or before since.

        Args:
            since (float | None): Time of the uncontrolled move, s; None for the newest pose.

        Returns:
            Pose2D | None: The hint, None when the ring has no such pose.
        """
        with self.state_lock:
            for stamp, pose in reversed(self.pose_ring):
                if since is None or stamp <= since:
                    return pose
        return None

    def relocalise(self, command: MapCommand) -> MapResult:
        """Recover the pose on the CURRENT map from the last known pose (design D13, U8).

        Runs the load sequence's steps against the live map serialized to a scratch directory: pause, candidate
        search, deserialize at the refined pose, confirm, verify. No autosave and no POI replace, except that
        ``discard_on_fail`` empties the live POIs when the pose is not recovered. Named bundles are never touched.

        Args:
            command (MapCommand): ``op: relocalise`` with optional since and discard_on_fail.

        Returns:
            MapResult: ok when localised; else the reason, with ``data.discarded_pois`` the count emptied.
        """
        hint = self.ring_hint(command.since)
        if hint is None:
            return MapResult(request_id=command.request_id, ok=False, message=f"cannot relocalise: {NO_POSE_REASON}")
        self.sensors.start_operation()
        scratch = self.store.root / SCRATCH_DIR / uuid.uuid4().hex
        try:
            return self.run_relocalise(command, hint, scratch)
        except OperationFailed as exc:
            LOGGER.warning("map_manager relocalise refused: %s", exc)
            return MapResult(request_id=command.request_id, ok=False, message=str(exc))
        finally:
            shutil.rmtree(scratch, ignore_errors=True)
            try:
                scratch.parent.rmdir()
            except OSError:
                pass
            self.sensors.end_operation()

    def run_relocalise(self, command: MapCommand, hint: Pose2D, scratch: Path) -> MapResult:
        """Serialize, pause, search, deserialize, confirm, verify and apply the verdict.

        Args:
            command (MapCommand): The relocalise command.
            hint (Pose2D): Last known pose from the ring.
            scratch (Path): Scratch directory for the serialized live graph (removed by the caller).

        Returns:
            MapResult: The answer.

        Raises:
            OperationFailed: Serialize or pause failed before slam_toolbox was touched; nothing changed.
        """
        grid = self.sensors.latest_grid()
        if grid is None:
            raise OperationFailed("cannot relocalise: no map grid received")
        scratch.mkdir(parents=True)
        base = str(scratch / POSEGRAPH_BASE)
        if not self.slam.serialize(base):
            raise OperationFailed("cannot relocalise: slam_toolbox serialize_map failed")
        with self.state_lock:
            self.before_load = (self.localisation, self.localised_value)
        self.set_localisation(
            Localisation(state="loading", message="relocalising on the current map", updated_at=self.clock()), False
        )
        search, search_reason = self.refine(grid, hint)
        target = search.pose if search is not None and search.ok else hint
        if not self.set_paused(True):
            self.restore_before_load()
            raise OperationFailed(f"relocalise aborted: {PAUSE_FAILED_REASON}")
        started = self.clock()
        confirmed: Pose2D | None = None
        ratio: float | None = None
        state: LocalisationState
        try:
            if not self.slam.deserialize(base, target):
                state, reason = "not_localised", DESERIALIZE_FAILED_REASON
            else:
                confirmed = self.slam.wait_pose_after(started, self.config.load_timeout_s)
                if confirmed is None:
                    state, reason = "not_localised", UNCONFIRMED_REASON
                else:
                    judge = LocalisationJudge(
                        localised_ratio=self.config.localised_ratio, min_samples=self.config.min_samples
                    )
                    for sample in self.sensors.agreement_samples(self.config.verify_window_s):
                        judge.add(sample)
                    ambiguous = search is not None and search.ambiguous
                    state, ratio, reason = judge.verdict(ambiguous=ambiguous)
        except Exception as exc:
            LOGGER.exception("map_manager relocalise failed after the pause")
            state, reason = "not_localised", f"relocalise failed: {exc}"
        notes = [reason]
        if state == "localised" and not self.set_paused(False):
            notes.append(UNPAUSE_FAILED_REASON)
        discarded = 0
        if state != "localised" and command.discard_on_fail:
            discarded, message = self.discard_live_pois()
            if discarded < 0:
                discarded = 0
                notes.append(f"POIs not discarded: {message}")
        with self.state_lock:
            self.before_load = None
        self.set_localisation(
            Localisation(state=state, ratio=ratio, pose=confirmed, message=reason, updated_at=self.clock()),
            state == "localised",
        )
        data = {
            "state": state,
            "ratio": ratio,
            "hint": hint.model_dump(),
            "pose": target.model_dump(),
            "slam_pose": confirmed.model_dump() if confirmed is not None else None,
            "search_ratio": search.ratio if search is not None else None,
            "search_reason": search_reason,
            "ambiguous": search is not None and search.ambiguous,
            "discarded_pois": discarded,
            "active_map": self.active_map,
            "load_seq": self.load_seq,
        }
        return MapResult(
            request_id=command.request_id,
            ok=state == "localised",
            message=f"relocalise: {state} ({'; '.join(notes)})",
            data=data,
        )

    def discard_live_pois(self) -> tuple[int, str]:
        """Empty the live POI set through poi_store.

        Returns:
            tuple[int, str]: Number of POIs emptied (-1 when the replace failed) and poi_store's message.
        """
        listing = self.poi.latest_list()
        count = len(listing[0]) if listing is not None else 0
        ok, message = self.poi.replace([])
        return (count if ok else -1), message
