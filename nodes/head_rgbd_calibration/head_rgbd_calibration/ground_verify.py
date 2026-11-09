"""Verification after a head chain accept (user decision 313): the floor must read flat at the right height in the
ground estimate and the obstacle cloud, at home and two other head poses.

The verification first waits (up to `restart_check_s`) until every unit of `restart_units` reports active and
restarted since the applied head chain's `applied_at` (the apply path unit restarts them), so the ground estimate runs
on the new chain. Then the head visits `ground_board.verify.poses_deg` through the floor sweep's command path and
motion settings (FloorSweep: head only). At each pose, once the head is reached and still and `settle_s` has passed,
the /ground/state documents of ground_estimator_cpp that report head_moving false are collected for `window_s` (longer,
up to `sample_timeout_s`, until `min_samples` arrived) and judged by `evaluate_pose` against the runbook tolerances
(docs/runbooks/head-floor-fit.md): median plane height within `max_height_m` of 0, median plane_vs_base_deg under
`max_plane_vs_base_deg`, median obstacle points under `max_obstacle_points`, median drop points at most
`max_drop_points`. Every exit commands the head back to where it was at start and releases the subscriptions and the
publisher. When the applied chain is a ground board result, the summary is stored with it (verification.json and the
result.json `verification` key).

Telemetry: calib_event lines of flow `ground_verify` (run id: the verification id): the start (head chain, poses,
tolerances) or the refused start, the unit restart wait (restarted or not), each pose (passed or failed at WARNING,
with the medians, maxima, sample counts and every check), the restore and the end (passed, failed, stopped or error).
"""

import json
import logging
import statistics
import threading
from collections import Counter, deque
from collections.abc import Callable
from datetime import datetime
from typing import Any

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import unit_state
from head_rgbd_calibration.arm_camera_results import URDF_SOURCE, head_chain_source
from head_rgbd_calibration.capture import rgb_qos
from head_rgbd_calibration.config import GroundVerifyConfig, NodeConfig
from head_rgbd_calibration.floor_sweep import (
    DONE,
    ERROR,
    POLL_S,
    PUBLISHER_DEPTH,
    STOPPED,
    SUBSCRIPTION_DEPTH,
    Busy,
    FloorSweep,
    JointSample,
    Run,
    Stopped,
    SweepError,
)
from head_rgbd_calibration.ground_results import SOURCE, GroundStore, record_verification
from head_rgbd_calibration.store import InvalidId, NotFound
from head_rgbd_calibration.telemetry import calib_event

LOG = logging.getLogger(__name__)

FLOW = "ground_verify"
VERIFY_STEP = "verify"
VERIFY_FORMAT = "verify-%Y%m%d-%H%M%S"
STARTING = "starting"
RESTARTS = "waiting_for_restarts"
MEASURING = "measuring"
STATE_HISTORY = 4000
STATE_QOS_DEPTH = 1
RESTART_POLL_S = 0.5
NO_CHAIN = "no head chain is applied: accept a calibration first"
NOT_RESTARTED = "units not restarted since the apply within {timeout:g} s: {units}"
NOT_REACHED = "did not reach pan/tilt within {timeout:g} s"


def state_qos() -> QoSProfile:
    """RELIABLE TRANSIENT_LOCAL depth 1, as ground_estimator_cpp publishes /ground/state."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=STATE_QOS_DEPTH,
    )


def median_or_none(values: list[float]) -> float | None:
    """Median of the values, None for none."""
    return float(statistics.median(values)) if values else None


def evaluate_pose(samples: list[dict[str, Any]], cfg: GroundVerifyConfig) -> dict[str, Any]:
    """Judge the /ground/state samples of one pose against the runbook tolerances.

    Args:
        samples: /ground/state documents taken with the head still (head_moving false).
        cfg: Verification config (min_samples and the tolerances).

    Returns:
        dict: {samples, plane_samples, height_m, plane_vs_base_deg, obstacle_points, obstacle_points_max, drop_points,
        drop_points_max, checks [{name, value, limit, ok}], passed}: medians over the samples (the plane ones over the
        samples with a plane; None without any), `checks` in the order samples, plane, height_m, plane_vs_base_deg,
        obstacle_points, drop_points.
    """
    planes = [s for s in samples if isinstance(s.get("plane"), dict)]
    heights = [float(s["plane"]["height_m"]) for s in planes if s["plane"].get("height_m") is not None]
    angles = [float(s["plane_vs_base_deg"]) for s in planes if s.get("plane_vs_base_deg") is not None]
    obstacles = [int(s.get("obstacle_points", 0)) for s in samples]
    drops = [int(s.get("drop_points", 0)) for s in samples]
    height = median_or_none(heights)
    angle = median_or_none(angles)
    obstacle = median_or_none(obstacles)
    drop = median_or_none(drops)
    checks = [
        {"name": "samples", "value": len(samples), "limit": cfg.min_samples, "ok": len(samples) >= cfg.min_samples},
        {"name": "plane", "value": len(planes), "limit": cfg.min_samples, "ok": len(planes) >= cfg.min_samples},
        {
            "name": "height_m",
            "value": height,
            "limit": cfg.max_height_m,
            "ok": height is not None and abs(height) <= cfg.max_height_m,
        },
        {
            "name": "plane_vs_base_deg",
            "value": angle,
            "limit": cfg.max_plane_vs_base_deg,
            "ok": angle is not None and angle < cfg.max_plane_vs_base_deg,
        },
        {
            "name": "obstacle_points",
            "value": obstacle,
            "limit": cfg.max_obstacle_points,
            "ok": obstacle is not None and obstacle < cfg.max_obstacle_points,
        },
        {
            "name": "drop_points",
            "value": drop,
            "limit": cfg.max_drop_points,
            "ok": drop is not None and drop <= cfg.max_drop_points,
        },
    ]
    return {
        "samples": len(samples),
        "plane_samples": len(planes),
        "height_m": height,
        "plane_vs_base_deg": angle,
        "obstacle_points": obstacle,
        "obstacle_points_max": max(obstacles) if obstacles else None,
        "drop_points": drop,
        "drop_points_max": max(drops) if drops else None,
        "checks": checks,
        "passed": all(c["ok"] for c in checks),
    }


class GroundVerify(FloorSweep):
    """Runs one verification at a time on a worker thread (FloorSweep provides the head motion)."""

    flow = FLOW

    def __init__(
        self,
        node: Any,
        cfg: NodeConfig,
        store: GroundStore,
        clock: Callable[[], float],
        sleep: Callable[[float], None],
        run: floor_results.RunFn,
        blocked: Callable[[], str | None] | None = None,
        now: Callable[[], datetime] = datetime.now,
    ) -> None:
        """Create an idle verification (nothing is subscribed or published until start).

        Args:
            node: The rclpy node the subscriptions and the publisher are created on.
            cfg: Node config (floor motion, restart_units, restart_check_s, ground_board.verify, state_dir).
            store: Ground store (where a ground board result's verification is recorded).
            clock: Monotonic seconds; all waits are measured on it.
            sleep: Sleeps seconds (the injected clock's sleep in tests).
            run: subprocess.run-compatible callable for the unit states.
            blocked: Returns a reason while something else forbids a verification, else None.
            now: Wall time, names the verification.
        """
        super().__init__(node, cfg, lambda: "", clock, sleep, now, blocked)
        self.verify = cfg.ground_board.verify
        self.store = store
        self.run_command = run
        self.documents: deque[tuple[float, dict[str, Any]]] = deque(maxlen=STATE_HISTORY)
        self.head_chain: dict[str, Any] | None = None
        self.result_ref: dict[str, str] | None = None
        self.poses: list[dict[str, Any]] = []
        self.passed: bool | None = None

    def state(self) -> dict[str, Any]:
        """The verification state for the HTTP API.

        Returns:
            {running, status, id, head_chain, result, pose, total, poses, passed, error}: pose is the 1-based pose
            being measured, poses the judged poses so far (`evaluate_pose` plus pose, pan_deg, tilt_deg, reached),
            result the ground board result the summary is stored with (None for another chain), passed None until
            every pose was judged.
        """
        with self.lock:
            return {
                "running": self.running,
                "status": self.status,
                "id": self.session_id,
                "head_chain": self.head_chain,
                "result": self.result_ref,
                "pose": self.pose,
                "total": self.total,
                "poses": [dict(p) for p in self.poses],
                "passed": self.passed,
                "error": self.error,
            }

    def start(self) -> str:
        """Start verifying the applied head chain.

        Returns:
            The verification id.

        Raises:
            Busy: a verification runs, or `blocked` names a reason.
            SweepError: no head chain is applied, or no head joint state arrives within move_timeout_s.
        """
        with self.lock:
            if self.running:
                raise Busy("a ground verification is already running")
            reason = self.blocked() if self.blocked is not None else None
            if reason:
                raise Busy(reason)
            self.running = True
            self.status = STARTING
            self.error = self.session_id = self.head_chain = self.result_ref = self.passed = None
            self.poses = []
            self.pose = 0
            self.total = len(self.verify.poses_deg)
        handles: list[Any] = []
        publisher: Any = None
        try:
            chain = head_chain_source(self.cfg)
            if chain["source"] == URDF_SOURCE:
                raise SweepError(NO_CHAIN)
            self.stop_event.clear()
            self.samples.clear()
            self.documents.clear()
            handles += [
                self.node.create_subscription(
                    JointState, self.floor.joint_states_topic, self.on_joint_state, rgb_qos(SUBSCRIPTION_DEPTH)
                ),
                self.node.create_subscription(String, self.verify.state_topic, self.on_state, state_qos()),
            ]
            publisher = self.node.create_publisher(JointState, self.floor.command_topic, PUBLISHER_DEPTH)
            start = self.wait_first_sample()
        except BaseException as exc:
            self.release(handles, publisher, None)
            with self.lock:
                self.running = False
                self.status = ERROR
                self.error = str(exc)
            calib_event(LOG, FLOW, None, VERIFY_STEP, ERROR, logging.WARNING, str(exc), error_type=type(exc).__name__)
            raise
        created = self.now()
        run = Run(
            session_id=created.strftime(VERIFY_FORMAT),
            directory=self.store.root,
            created=created,
            grid=[(float(pan), float(tilt)) for pan, tilt in self.verify.poses_deg],
            start_pan=start.pan,
            start_tilt=start.tilt,
            subscriptions=handles,
            publisher=publisher,
            observations=[],
            skipped=[],
            dropped=Counter(),
        )
        ref = (
            {"session": str(chain["session"]), "result": str(chain["result"])}
            if chain["source"] == SOURCE and chain["session"] and chain["result"]
            else None
        )
        with self.lock:
            self.session_id = run.session_id
            self.head_chain = chain
            self.result_ref = ref
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            VERIFY_STEP,
            "start",
            head_chain=chain,
            result=ref,
            poses=run.grid,
            tolerances=self.verify.model_dump(),
            units=self.cfg.restart_units,
        )
        self.thread = threading.Thread(target=self.verify_run, args=(run,), name="ground-verify", daemon=True)
        self.thread.start()
        return run.session_id

    def on_state(self, msg: Any) -> None:
        """Keep a /ground/state document with its arrival time; one that does not parse is dropped."""
        try:
            doc = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(doc, dict):
            with self.lock:
                self.documents.append((self.clock(), doc))

    def verify_run(self, run: Run) -> None:
        """Worker: wait for the unit restarts, measure every pose, then always restore the head and release."""
        final, error = DONE, None
        try:
            self.wait_restarts(run)
            for index, (pan, tilt) in enumerate(run.grid):
                if self.stop_event.is_set():
                    raise Stopped
                with self.lock:
                    self.pose = index + 1
                    self.status = MEASURING
                self.measure(run, index, pan, tilt)
        except Stopped:
            final = STOPPED
        except SweepError as exc:
            final, error = ERROR, str(exc)
        except Exception as exc:  # reported as the verification error, the finally block still restores the head
            final, error = ERROR, f"{type(exc).__name__}: {exc}"
            LOG.exception("ground verification %s raised", run.session_id)
        finally:
            self.restore(run)
            self.release(run.subscriptions, run.publisher, run.session_id)
            with self.lock:
                complete = final == DONE and len(self.poses) == len(run.grid)
                self.passed = all(p["passed"] for p in self.poses) if complete else None
                self.running = False
                self.status = final
                self.error = error
            summary = {key: value for key, value in self.state().items() if key != "running"}
            self.record(run, summary)
        outcome = final if self.passed is None else ("passed" if self.passed else "failed")
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            VERIFY_STEP,
            outcome,
            logging.ERROR if error else (logging.INFO if self.passed else logging.WARNING),
            error,
            passed=self.passed,
            poses=[{"pose": p["pose"], "passed": p["passed"]} for p in summary["poses"]],
        )

    def record(self, run: Run, summary: dict[str, Any]) -> None:
        """Store the summary with the verified ground board result (when the applied chain is one)."""
        ref = summary["result"]
        if ref is None:
            return
        try:
            record_verification(self.store, ref["session"], ref["result"], summary)
        except (InvalidId, NotFound, OSError, ValueError) as exc:
            calib_event(LOG, FLOW, run.session_id, "record", ERROR, logging.WARNING, str(exc), result=ref)

    def wait_restarts(self, run: Run) -> None:
        """Wait until every restart unit is active and restarted since the head chain apply.

        Raises:
            SweepError: not within restart_check_s.
            Stopped: stop() was called.
        """
        with self.lock:
            self.status = RESTARTS
        record = floor_results.read_applied_marker(self.cfg).get(floor_results.HEAD_CHAIN_KEY) or {}
        try:
            applied_at = datetime.fromisoformat(str(record["applied_at"])).timestamp()
        except (KeyError, TypeError, ValueError):
            calib_event(
                LOG,
                FLOW,
                run.session_id,
                "units",
                "skipped",
                logging.WARNING,
                "the head chain record has no applied_at",
            )
            return
        began = self.clock()
        while True:
            if self.stop_event.is_set():
                raise Stopped
            units = [unit_state(unit, applied_at, self.run_command) for unit in self.cfg.restart_units]
            waiting = [u["name"] for u in units if not (u["restarted"] and u["active_state"] == "active")]
            if not waiting:
                calib_event(LOG, FLOW, run.session_id, "units", "restarted", units=units)
                return
            if self.clock() - began >= self.cfg.restart_check_s:
                reason = NOT_RESTARTED.format(timeout=self.cfg.restart_check_s, units=", ".join(waiting))
                calib_event(LOG, FLOW, run.session_id, "units", "not_restarted", logging.WARNING, reason, units=units)
                raise SweepError(reason)
            self.sleep(RESTART_POLL_S)

    def measure(self, run: Run, index: int, pan: float, tilt: float) -> None:
        """Move to one pose, collect its still samples and judge them."""
        entry: dict[str, Any] = {"pose": index, "pan_deg": pan, "tilt_deg": tilt, "reached": None}
        still = self.move_to(run, pan, tilt)
        if still is None:
            result = evaluate_pose([], self.verify)
            entry.update(result, error=NOT_REACHED.format(timeout=self.floor.move_timeout_s))
        else:
            entry["reached"] = self.measured_deg(still)
            entry.update(self.collect(still), error=None)
        with self.lock:
            self.poses.append(entry)
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            "pose",
            "passed" if entry["passed"] else "failed",
            logging.INFO if entry["passed"] else logging.WARNING,
            entry["error"] or "; ".join(c["name"] for c in entry["checks"] if not c["ok"]) or None,
            **{key: value for key, value in entry.items() if key != "error"},
        )

    def collect(self, still: JointSample) -> dict[str, Any]:
        """The judged samples of a pose: settle, then the head-still documents over window_s (up to
        sample_timeout_s until min_samples)."""
        settled = still.received_s + self.verify.settle_s
        while self.clock() < settled:
            self.check_alive()
            self.sleep(POLL_S)
        since = self.clock()
        while True:
            self.check_alive()
            with self.lock:
                samples = [doc for t, doc in self.documents if t >= since and doc.get("head_moving") is False]
            elapsed = self.clock() - since
            if (elapsed >= self.verify.window_s and len(samples) >= self.verify.min_samples) or (
                elapsed >= self.verify.sample_timeout_s
            ):
                return evaluate_pose(samples, self.verify)
            self.sleep(POLL_S)
