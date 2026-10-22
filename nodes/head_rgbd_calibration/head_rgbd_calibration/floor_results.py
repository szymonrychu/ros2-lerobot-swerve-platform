"""Compute, store and apply the head floor fit (design D6, D7, D8).

Layout under state_dir: floor_fit/<sid>/observations.json (written by the sweep) and
floor_fit/<sid>/results/<rid>/{result.json, head_chain.json}. Applying copies head_chain.json into applied/ and adds a
head_chain key to applied/applied.json, which starts the apply path unit (it restarts head_state_publisher).
"""

import json
import logging
import os
import re
import subprocess
import threading
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.head_rgbd_calibration.floor_fit import FloorObservation, PlaneFit, fit_head_chain_floor
from ros2_common.head_rgbd_calibration.urdf_text import (
    HEAD_CHAIN_FILE,
    Origin,
    apply_overrides,
    head_chain_json,
    parse_head_chain,
)

from head_rgbd_calibration.apply import APPLIED_DIR, APPLIED_MARKER, NotApplicable, unit_state
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_sweep import FLOOR_DIR, OBSERVATIONS_FILE
from head_rgbd_calibration.store import (
    RESULT_FILE,
    RESULT_ID_RE,
    TEMP_PREFIX,
    InvalidId,
    NotFound,
    SessionStore,
    write_json_atomic,
)

LOGGER = logging.getLogger(__name__)

FLOOR_SESSION_ID_RE = re.compile(r"^floor-\d{8}-\d{6}(?:-\d+)?$", re.ASCII)
RESULTS_DIR = "results"
HEAD_CHAIN_KEY = "head_chain"
NOTE_DATE_FORMAT = "%Y-%m-%d"
SOURCE_FLOOR = "floor"
LOCK = threading.Lock()

RunFn = Callable[..., subprocess.CompletedProcess]


def floor_session_dir(cfg: NodeConfig, sid: str) -> Path:
    """Directory of a floor fit session.

    Args:
        cfg: Node config (state_dir).
        sid: Floor session id (floor-YYYYmmdd-HHMMSS[-N]).

    Returns:
        The directory path; it need not exist.

    Raises:
        InvalidId: The id does not match the pattern.
    """
    if not FLOOR_SESSION_ID_RE.fullmatch(sid):
        raise InvalidId(f"invalid floor session id: {sid!r}")
    return Path(cfg.state_dir) / FLOOR_DIR / sid


def existing_floor_session_dir(cfg: NodeConfig, sid: str) -> Path:
    """Directory of a floor session that holds an observations.json.

    Raises:
        InvalidId: Malformed id.
        NotFound: No such session.
    """
    path = floor_session_dir(cfg, sid)
    if not (path / OBSERVATIONS_FILE).is_file():
        raise NotFound(f"no such floor session: {sid}")
    return path


def floor_result_dir(cfg: NodeConfig, sid: str, rid: str) -> Path:
    """Directory of a floor fit result (it need not exist).

    Raises:
        InvalidId: Malformed session or result id.
        NotFound: No such session.
    """
    if not RESULT_ID_RE.fullmatch(rid):
        raise InvalidId(f"invalid result id: {rid!r}")
    return existing_floor_session_dir(cfg, sid) / RESULTS_DIR / rid


def result_ids(cfg: NodeConfig, sid: str) -> list[str]:
    """Ids of the results of a floor session that hold a result.json, newest first."""
    results = existing_floor_session_dir(cfg, sid) / RESULTS_DIR
    if not results.is_dir():
        return []
    found = [p.name for p in results.iterdir() if RESULT_ID_RE.fullmatch(p.name) and (p / RESULT_FILE).is_file()]
    return sorted(found, reverse=True)


def base_urdf(cfg: NodeConfig) -> str:
    """The head URDF with the applied overrides (tof_joint and head chain).

    Args:
        cfg: Node config (head_urdf_file, state_dir).

    Returns:
        URDF text; every override warning is logged.
    """
    text, warnings = apply_overrides(Path(cfg.head_urdf_file).read_text(), Path(cfg.state_dir) / APPLIED_DIR)
    for warning in warnings:
        LOGGER.warning("head override skipped: %s", warning)
    return text


def read_observations(path: Path) -> tuple[list[FloorObservation], dict[str, Any]]:
    """Parse a sweep's observations.json.

    Args:
        path: observations.json path.

    Returns:
        (observations, the raw document).
    """
    doc = json.loads(path.read_text())
    observations = [
        FloorObservation(
            pose=int(o["pose"]),
            pan_rad=float(o["pan_rad"]),
            tilt_rad=float(o["tilt_rad"]),
            plane=PlaneFit(
                normal=np.asarray(o["normal"], dtype=float),
                height_m=float(o["height_m"]),
                inliers=int(o["inliers"]),
                candidates=int(o["candidates"]),
                rms_m=float(o["rms_m"]),
            ),
        )
        for o in doc["observations"]
    ]
    return observations, doc


def applied_chain_origins(applied_dir: Path) -> dict[str, Origin]:
    """Origins of the applied head_chain.json, empty when absent or invalid."""
    path = applied_dir / HEAD_CHAIN_FILE
    try:
        return parse_head_chain(json.loads(path.read_text()))
    except (OSError, ValueError):
        return {}


def compute_floor(cfg: NodeConfig, sid: str, now: datetime) -> dict[str, Any]:
    """Fit the head chain to a floor session and store the result.

    Args:
        cfg: Node config (state_dir, head_urdf_file, floor.fit).
        sid: Floor session id.
        now: Result time (names the result and dates the note).

    Returns:
        The result.json content {id, session, created, passed, failed_limits, params, height_rms_mm, normal_rms_deg,
        observation_height_rms_mm, observation_normal_rms_deg, poses, skipped}: height_rms_mm and normal_rms_deg are
        the pose-mean RMS the limits apply to, the observation_* keys the per-observation RMS. head_chain.json is
        written next to it only when the result passed.

    Raises:
        InvalidId, NotFound: unknown or malformed session id.
    """
    session = existing_floor_session_dir(cfg, sid)
    observations, doc = read_observations(session / OBSERVATIONS_FILE)
    fit = fit_head_chain_floor(observations, base_urdf(cfg), cfg.floor.fit)
    rid = SessionStore.new_result_id(now)
    body = fit.to_json()
    result = {
        "id": rid,
        "session": sid,
        "created": now.isoformat(),
        "passed": body["passed"],
        "failed_limits": body["failed_limits"],
        "params": body["params"],
        "height_rms_mm": body["height_rms_mm"],
        "normal_rms_deg": body["normal_rms_deg"],
        "observation_height_rms_mm": body["observation_height_rms_mm"],
        "observation_normal_rms_deg": body["observation_normal_rms_deg"],
        "poses": body["poses"],
        "skipped": doc.get("skipped", []),
    }
    directory = session / RESULTS_DIR / rid
    with LOCK:
        directory.mkdir(parents=True, exist_ok=True)
        chain_path = directory / HEAD_CHAIN_FILE
        chain_path.unlink(missing_ok=True)
        if fit.passed:
            origins = {**applied_chain_origins(Path(cfg.state_dir) / APPLIED_DIR), **fit.origins}
            note = (
                f"floor fit {now.strftime(NOTE_DATE_FORMAT)}, pose-mean height RMS {fit.height_rms_mm:.1f} mm, "
                f"normal RMS {fit.normal_rms_deg:.2f} deg"
            )
            tmp = directory / f"{TEMP_PREFIX}{HEAD_CHAIN_FILE}"
            tmp.write_text(head_chain_json(origins, note, sid, rid))
            os.replace(tmp, chain_path)
        write_json_atomic(directory / RESULT_FILE, result)
    return result


def read_result(cfg: NodeConfig, sid: str, rid: str) -> dict[str, Any]:
    """The result.json of a floor fit result.

    Raises:
        InvalidId, NotFound: malformed ids, unknown session or result.
    """
    path = floor_result_dir(cfg, sid, rid) / RESULT_FILE
    if not path.is_file():
        raise NotFound(f"no such result: {rid}")
    return json.loads(path.read_text())


def read_applied_marker(cfg: NodeConfig) -> dict[str, Any]:
    """applied/applied.json, empty when absent or unreadable."""
    try:
        data = json.loads((Path(cfg.state_dir) / APPLIED_DIR / APPLIED_MARKER).read_text())
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def current_floor_ref(cfg: NodeConfig) -> dict[str, str] | None:
    """The floor result the applied head chain was checked against, from the `head_chain` record.

    Args:
        cfg: Node config (state_dir).

    Returns:
        {session, result}: the record's `floor` entry; for a record without a `floor` key (written by the floor apply
        before the key existed) whose session matches FLOOR_SESSION_ID_RE, that record's own session and result;
        None when there is neither.
    """
    record = read_applied_marker(cfg).get(HEAD_CHAIN_KEY)
    if not isinstance(record, dict):
        return None
    if "floor" in record:
        floor = record["floor"]
        if isinstance(floor, dict) and floor.get("session") and floor.get("result"):
            return {"session": str(floor["session"]), "result": str(floor["result"])}
        return None
    session, result = str(record.get("session", "")), str(record.get("result", ""))
    if FLOOR_SESSION_ID_RE.fullmatch(session) and result:
        return {"session": session, "result": result}
    return None


def apply_head_chain_file(
    cfg: NodeConfig, source: Path, record: dict[str, Any], now: datetime, run: RunFn
) -> dict[str, Any]:
    """Copy a head_chain.json into applied/ and record it in applied.json (the one writer of the applied chain).

    Args:
        cfg: Node config (state_dir, restart_units).
        source: The result's head_chain.json.
        record: The `head_chain` record: {session, result, applied_at, source, floor}.
        now: Apply time, for the unit states.
        run: subprocess.run-compatible callable for the unit state.

    Returns:
        {head_chain: record, units: [{name, active_state, restarted}]}.

    Raises:
        NotApplicable: no RGB-D calibration is applied.
    """
    target = Path(cfg.state_dir) / APPLIED_DIR
    marker = target / APPLIED_MARKER
    with LOCK:
        if not marker.is_file():
            raise NotApplicable("no RGB-D calibration is applied; apply one before the head chain")
        info = json.loads(marker.read_text())
        tmp = target / f"{TEMP_PREFIX}{HEAD_CHAIN_FILE}"
        try:
            tmp.write_bytes(source.read_bytes())
            os.replace(tmp, target / HEAD_CHAIN_FILE)
            write_json_atomic(marker, {**info, HEAD_CHAIN_KEY: record})
        finally:
            tmp.unlink(missing_ok=True)
    return {HEAD_CHAIN_KEY: record, "units": [unit_state(u, now.timestamp(), run) for u in cfg.restart_units]}


def apply_floor(cfg: NodeConfig, sid: str, rid: str, now: datetime, run: RunFn) -> dict[str, Any]:
    """Apply a passing floor result: copy head_chain.json into applied/ and record it in applied.json.

    Args:
        cfg: Node config (state_dir, restart_units).
        sid: Floor session id.
        rid: Result id.
        now: Apply time, recorded as head_chain.applied_at.
        run: subprocess.run-compatible callable for the unit state.

    Returns:
        {head_chain: {session, result, applied_at, source: "floor", floor: {session, result}}, units: [...]}.

    Raises:
        NotApplicable: the result did not pass, or no RGB-D calibration is applied.
        InvalidId, NotFound: unknown or malformed ids.
    """
    directory = floor_result_dir(cfg, sid, rid)
    result_path = directory / RESULT_FILE
    if not result_path.is_file():
        raise NotFound(f"no such result: {rid}")
    source = directory / HEAD_CHAIN_FILE
    if json.loads(result_path.read_text()).get("passed") is not True or not source.is_file():
        raise NotApplicable(f"result {rid} did not pass and cannot be applied")
    record = {
        "session": sid,
        "result": rid,
        "applied_at": now.isoformat(),
        "source": SOURCE_FLOOR,
        "floor": {"session": sid, "result": rid},
    }
    return apply_head_chain_file(cfg, source, record, now, run)


def applied_head_chain(cfg: NodeConfig) -> dict[str, Any] | None:
    """The applied head chain, read from applied/head_chain.json itself.

    Args:
        cfg: Node config (state_dir).

    Returns:
        {session, result, note, source, floor, joints: {name: {xyz, rpy}}}, None when the file is absent or invalid.
        source ("floor" or "head_arm") and floor ({session, result} or None) come from the applied.json record when it
        names the same session and result, else None.
    """
    path = Path(cfg.state_dir) / APPLIED_DIR / HEAD_CHAIN_FILE
    try:
        data = json.loads(path.read_text())
        origins = parse_head_chain(data)
        session, result = str(data.get("session", "")), str(data.get("result", ""))
        record = read_applied_marker(cfg).get(HEAD_CHAIN_KEY)
        matches = isinstance(record, dict) and (record.get("session"), record.get("result")) == (session, result)
        return {
            "session": session,
            "result": result,
            "note": str(data.get("note", "")),
            "source": (record.get("source") or (SOURCE_FLOOR if "floor" not in record else None)) if matches else None,
            "floor": current_floor_ref(cfg) if matches else None,
            "joints": {name: {"xyz": list(o.xyz), "rpy": list(o.rpy)} for name, o in origins.items()},
        }
    except (OSError, ValueError):
        return None
