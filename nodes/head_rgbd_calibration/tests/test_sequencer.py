"""Tests for the guided head-from-arm capture sequence (design D37, D38, D46).

The robot is the real RobotClient over a fake mcp_server transport, so every assertion is on the tool calls and their
arguments as mcp_server would receive them. The fake arm reports the commanded joints as measured. The capture is a
fake still-capture controller; the clock is injected and advanced by the injected sleep. The pose check runs on the
shipped arm and head URDFs (nodes/web_ui/urdf) with the shipped pose set.
"""

import math
import threading
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseChecker
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_centre

from head_rgbd_calibration.capture import StillOutcome
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.robot_client import RawToolResult, RobotClient
from head_rgbd_calibration.sequencer import (
    HOLDING,
    IDLE,
    WAITING_BIG_JOINTS,
    WAITING_BOARD,
    Busy,
    SequenceError,
    Sequencer,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
ARM_URDF = (URDF_DIR / "so101_arm.urdf").read_text()
HEAD_URDF = (URDF_DIR / "head.urdf").read_text()
FOOTPRINT_YAML = REPO_ROOT / "scripts" / "gen_nav_footprint.yaml"
HOME_POSE: dict[str, float] = {
    name: float(value)
    for name, value in yaml.safe_load(FOOTPRINT_YAML.read_text())["parked_arm_joints"].items()
    if name in ARM_JOINT_NAMES
}
TOKEN = "seq-token"
SESSION = "head-arm-20261022-120000"
STATE_KEYS = {
    "running",
    "phase",
    "session_id",
    "tier",
    "arm_pose",
    "total_poses",
    "captured",
    "skipped",
    "status",
    "error",
    "big_joints",
    "waiting_since",
}
BIG_JOINTS = ["elbow_flex", "shoulder_lift", "shoulder_pan"]
RADIUS_KEY = "carried_object_radius_m"
WAIT_S = 20.0
GRIPPER = "gripper"


class FakeClock:
    """Injected monotonic clock; sleep advances it and yields to the other threads."""

    def __init__(self) -> None:
        self.now = 1000.0
        self.sleeps: list[float] = []

    def clock(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds
        time.sleep(0.0005)


class FakeRobot:
    """Fake mcp_server transport: records every tool call; the arm reports the commanded joints as measured.

    Attributes:
        errors: Tool name -> 1-based call number answered with a tool error.
        move_status: 1-based move_arm_joints call number -> status other than converged.
        head_status: 1-based set_head_angles call number -> status other than converged.
        measured: Overrides the measured positions get_arm_state reports (None: the commanded ones).
        carrying: Reasons get_control_state reports in arm.carrying (empty: not carrying).
        reset_arm: reset.arm of a park release.
    """

    def __init__(self, log: list[tuple[str, str, dict[str, Any]]]) -> None:
        self.log = log
        self.calls: list[tuple[str, dict[str, Any]]] = []
        self.positions: dict[str, float] = {**HOME_POSE, GRIPPER: 0.05}
        self.errors: dict[str, int] = {}
        self.move_status: dict[int, str] = {}
        self.head_status: dict[int, str] = {}
        self.measured: dict[str, float] | None = None
        self.carrying: list[str] = []
        self.reset_arm = "homed"

    def count(self, name: str) -> int:
        return sum(1 for called, _ in self.calls if called == name)

    def named(self, name: str) -> list[dict[str, Any]]:
        return [args for called, args in self.calls if called == name]

    async def __call__(self, url: str, token: str, name: str, args: dict[str, Any]) -> RawToolResult:
        assert token == TOKEN
        self.calls.append((name, dict(args)))
        self.log.append(("robot", name, dict(args)))
        number = self.count(name)
        if self.errors.get(name) == number:
            return RawToolResult(True, f"{name} failed: injected tool error")
        if name == "take_control":
            return RawToolResult(False, "ok", {"arm": {"held": True}, "base": {"held": False}})
        if name == "get_arm_state":
            positions = self.measured if self.measured is not None else self.positions
            return RawToolResult(False, "ok", {"positions": dict(positions), "control_held": True, "home_stored": True})
        if name == "move_arm_joints":
            status = self.move_status.get(number, "converged")
            if status == "converged":
                self.positions.update(args["targets"])
            message = "" if status == "converged" else "not converged (error 0.200 rad); holding measured pose"
            return RawToolResult(False, "ok", {"status": status, "message": message, "positions": self.positions})
        if name == "set_head_angles":
            status = self.head_status.get(number, "converged")
            angles = {"pan_deg": args["pan_deg"], "tilt_deg": args["tilt_deg"]}
            return RawToolResult(False, "ok", {"status": status, "commanded": angles, "measured": angles})
        if name == "release_control":
            keep = args["keep_position"]
            reset = {"arm": "kept", "head": "kept"} if keep else {"arm": self.reset_arm, "head": "homed"}
            return RawToolResult(False, "ok", {"released": ["arm"], "reset": reset, "state": {}})
        if name == "get_control_state":
            carrying = {"carrying": bool(self.carrying), "reasons": list(self.carrying), "radius_m": None}
            return RawToolResult(False, "ok", {"arm": {"held": False, "carrying": carrying}, "base": {"held": False}})
        raise AssertionError(f"unexpected tool {name}")


class FakeCapture:
    """Fake still-capture controller: stores every view unless told otherwise.

    Attributes:
        outcomes: 1-based capture_still call number -> StillOutcome other than stored.
        raise_at: 1-based capture_still call number that raises RuntimeError.
        on_capture: Called with the call number on every capture_still (e.g. to press Stop).
    """

    def __init__(self, log: list[tuple[str, str, dict[str, Any]]]) -> None:
        self.log = log
        self.stills: list[tuple[int, int, int]] = []
        self.stops = 0
        self.sessions: list[str | None] = []
        self.outcomes: dict[int, StillOutcome] = {}
        self.raise_at: int | None = None
        self.on_capture: Callable[[int], None] | None = None

    def start_still_session(self, session_id: str | None) -> str:
        self.sessions.append(session_id)
        self.log.append(("capture", "start_still_session", {"session_id": session_id}))
        return SESSION

    def capture_still(self, arm_pose: int, tier: int, head_offset: int) -> StillOutcome:
        self.stills.append((arm_pose, tier, head_offset))
        self.log.append(("capture", "capture_still", {"arm_pose": arm_pose, "tier": tier, "head_offset": head_offset}))
        number = len(self.stills)
        if self.on_capture is not None:
            self.on_capture(number)
        if self.raise_at == number:
            raise RuntimeError("injected capture failure")
        return self.outcomes.get(number, StillOutcome(True, f"pose-{number:03d}", "stored"))

    def stop(self) -> dict[str, Any]:
        self.stops += 1
        self.log.append(("capture", "stop", {}))
        return {}


class Rig:
    """A sequencer wired to the fakes."""

    def __init__(self, tmp_path: Path, cfg: NodeConfig | None = None, blocked: str | None = None) -> None:
        token = tmp_path / "token"
        token.write_text(f"MCP_SERVER_TOKEN={TOKEN}\n")
        base = cfg or NodeConfig()
        self.cfg = base.model_copy(update={"robot": base.robot.model_copy(update={"token_file": str(token)})})
        self.log: list[tuple[str, str, dict[str, Any]]] = []
        self.robot = FakeRobot(self.log)
        self.capture = FakeCapture(self.log)
        self.clock = FakeClock()
        self.blocked_reason = blocked
        self.head_urdf = HEAD_URDF
        self.seq = Sequencer(
            self.capture,
            RobotClient(self.cfg.robot, call=self.robot),
            self.cfg,
            lambda: self.head_urdf,
            lambda: ARM_URDF,
            lambda: self.blocked_reason,
            sleep=self.clock.sleep,
            clock=self.clock.clock,
        )

    def wait_phase(self, phase: str) -> dict[str, Any]:
        return self.wait_for(lambda s: s["phase"] == phase)

    def wait_for(self, predicate: Callable[[dict[str, Any]], bool]) -> dict[str, Any]:
        end = time.monotonic() + WAIT_S
        while time.monotonic() < end:
            state = self.seq.state()
            if predicate(state):
                return state
            time.sleep(0.002)
        raise AssertionError(f"timed out waiting, state {self.seq.state()}")

    def finish(self) -> dict[str, Any]:
        self.seq.wait(WAIT_S)
        state = self.seq.state()
        assert not state["running"]
        return state

    def run_to_holding(self, allow: bool = True) -> dict[str, Any]:
        self.seq.move_to_base(confirmed=True)
        self.wait_phase(WAITING_BOARD)
        self.seq.board_attached()
        self.wait_phase(WAITING_BIG_JOINTS)
        self.seq.allow_big_joints(allow)
        return self.finish()

    def names(self) -> list[str]:
        return [name for _, name, _ in self.log]


def small_cfg(**seq_updates: Any) -> NodeConfig:
    """Node config with the first two shipped poses of each tier."""
    cfg = NodeConfig()
    seq = cfg.arm_sequence
    update: dict[str, Any] = {
        "tier0_poses": seq.tier0_poses[:2],
        "tier1_poses": seq.tier1_poses[:2],
        "tier2_poses": seq.tier2_poses[:2],
    }
    update.update(seq_updates)
    arm_sequence = type(seq).model_validate({**seq.model_dump(), **update})
    return cfg.model_copy(update={"arm_sequence": arm_sequence})


def tier_poses(cfg: NodeConfig) -> list[tuple[int, dict[str, float]]]:
    seq = cfg.arm_sequence
    return [(tier, pose) for tier in (0, 1, 2) for pose in getattr(seq, f"tier{tier}_poses")]


def expected_aim(cfg: NodeConfig, joints: dict[str, float], head_urdf: str = HEAD_URDF) -> tuple[float, float]:
    """(pan, tilt) deg aimed at the nominal board centre at the measured joints, computed independently."""
    checker = PoseChecker(ARM_URDF, head_urdf, cfg.fit, cfg.arm_sequence.pose_check)
    mount = pose_from_xyz_rpy(cfg.fit.board_mount.xyz_init, cfg.fit.board_mount.rpy_init)
    board = checker.link_poses(joints)["gripper_link"] @ mount
    centre = (board @ np.append(board_centre(cfg.fit.board), 1.0))[:3]
    pan, tilt = aim_head(UrdfChain.from_text(head_urdf), centre)
    return math.degrees(pan), math.degrees(tilt)


def assert_held_not_parked(rig: Rig) -> None:
    releases = rig.robot.named("release_control")
    assert releases, "the sequence never released the arm"
    assert all(args["keep_position"] is True for args in releases)
    assert releases[-1] == {"subsystem": "arm", "keep_position": True}
    assert rig.capture.stops >= 1
    assert rig.names().index("stop") < len(rig.names()) - 1 - rig.names()[::-1].index("release_control")


def test_move_to_base_needs_confirm(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    with pytest.raises(ValueError, match="confirm"):
        rig.seq.move_to_base(confirmed=False)
    assert rig.robot.calls == []
    assert rig.capture.sessions == [] and rig.capture.stops == 0
    state = rig.seq.state()
    assert state["running"] is False and state["phase"] == IDLE


def test_full_run_call_order(tmp_path: Path) -> None:
    cfg = NodeConfig()
    rig = Rig(tmp_path, cfg)
    seq_cfg = cfg.arm_sequence
    rig.seq.move_to_base(confirmed=True)
    state = rig.wait_phase(WAITING_BOARD)
    assert state["running"] is True
    assert "Clip the board to the fixed jaw, then press Board attached" in state["status"]
    assert state["waiting_since"] is not None
    assert rig.names() == ["take_control", "get_arm_state", "move_arm_joints"]
    assert rig.robot.named("move_arm_joints")[0] == {
        "targets": {**seq_cfg.base_pose, GRIPPER: seq_cfg.gripper_rad},
        "speed_scale": seq_cfg.speed_scale,
    }
    assert rig.capture.sessions == []
    rig.seq.board_attached()
    state = rig.wait_phase(WAITING_BIG_JOINTS)
    for name in BIG_JOINTS:
        assert name in state["status"]
    rig.seq.allow_big_joints(True)
    state = rig.finish()
    assert state["phase"] == HOLDING
    assert "Remove the board, then press Park" in state["status"]
    assert state["error"] is None
    assert state["session_id"] == SESSION

    poses = tier_poses(cfg)
    offsets = seq_cfg.head_offsets_deg
    expected: list[str] = ["take_control", "get_arm_state", "move_arm_joints", "start_still_session"]
    for _ in poses:
        expected += ["get_arm_state", "move_arm_joints", "get_arm_state"]
        expected += ["set_head_angles", "capture_still"] * len(offsets)
    expected += ["get_arm_state", "move_arm_joints", "stop", "release_control"]
    assert rig.names() == expected
    assert rig.capture.sessions == [None]
    moves = rig.robot.named("move_arm_joints")
    assert [m["targets"] for m in moves[1:-1]] == [pose for _, pose in poses]
    assert moves[-1]["targets"] == seq_cfg.base_pose
    assert all(m["speed_scale"] == seq_cfg.speed_scale for m in moves)
    assert rig.robot.named("release_control") == [{"subsystem": "arm", "keep_position": True}]
    stills = [(index, tier, k) for index, (tier, _) in enumerate(poses) for k in range(len(offsets))]
    assert rig.capture.stills == stills
    assert state["captured"] == len(stills)
    assert state["skipped"] == {}
    assert state["total_poses"] == len(poses)
    assert rig.clock.sleeps.count(seq_cfg.settle_s) >= len(stills)


@pytest.mark.parametrize("allow", [True, False])
def test_big_joints_need_confirm(tmp_path: Path, allow: bool) -> None:
    cfg = small_cfg()
    rig = Rig(tmp_path, cfg)
    with pytest.raises(Busy):
        rig.seq.allow_big_joints(True)
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    with pytest.raises(Busy):
        rig.seq.allow_big_joints(True)
    rig.seq.board_attached()
    state = rig.wait_phase(WAITING_BIG_JOINTS)
    assert state["big_joints"] == BIG_JOINTS
    tier2 = [pose for tier, pose in tier_poses(cfg) if tier == 2]
    moved = [m["targets"] for m in rig.robot.named("move_arm_joints")]
    assert not any(target in moved for target in tier2)
    assert {tier for _, tier, _ in rig.capture.stills} == {0, 1}
    with pytest.raises(Busy):
        rig.seq.board_attached()
    rig.seq.allow_big_joints(allow)
    state = rig.finish()
    assert state["phase"] == HOLDING
    moved = [m["targets"] for m in rig.robot.named("move_arm_joints")]
    if allow:
        assert all(target in moved for target in tier2)
        assert 2 in {tier for _, tier, _ in rig.capture.stills}
    else:
        assert not any(target in moved for target in tier2)
        assert 2 not in {tier for _, tier, _ in rig.capture.stills}
        assert moved[-1] == cfg.arm_sequence.base_pose
        assert "tier 2 skipped" in state["status"]


def test_gripper_only_in_base_move(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    moves = rig.robot.named("move_arm_joints")
    assert len(moves) > 2
    assert moves[0]["targets"][GRIPPER] == rig.cfg.arm_sequence.gripper_rad
    assert all(GRIPPER not in m["targets"] for m in moves[1:])


def test_path_check_refusal_sends_nothing(tmp_path: Path) -> None:
    # Measured arm at a pose that breaks the joint limits: the base move is refused before any motion.
    rig = Rig(tmp_path, small_cfg())
    rig.robot.measured = {**HOME_POSE, "shoulder_lift": 3.0, GRIPPER: 0.05}
    rig.seq.move_to_base(confirmed=True)
    state = rig.finish()
    assert rig.robot.count("move_arm_joints") == 0
    assert rig.robot.count("set_head_angles") == 0
    assert state["phase"] == IDLE
    assert "base pose" in state["error"] and "joint_limits" in state["error"]
    assert_held_not_parked(rig)

    # A measured arm joint missing: nothing is commanded.
    rig = Rig(tmp_path, small_cfg())
    rig.robot.measured = {name: value for name, value in HOME_POSE.items() if name != "elbow_flex"}
    rig.seq.move_to_base(confirmed=True)
    state = rig.finish()
    assert rig.robot.count("move_arm_joints") == 0
    assert "elbow_flex" in state["error"]
    assert_held_not_parked(rig)

    # A scripted pose whose path fails the check (wrist_roll beyond the URDF limit): stop before that pose.
    seq = NodeConfig().arm_sequence
    bad = {**seq.base_pose, "wrist_roll": 4.0}
    cfg = small_cfg(tier0_poses=[seq.tier0_poses[0], bad], max_roll_delta_rad=5.0)
    rig = Rig(tmp_path, cfg)
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    rig.seq.board_attached()
    state = rig.finish()
    moved = [m["targets"] for m in rig.robot.named("move_arm_joints")]
    assert bad not in moved
    assert moved[-1] == seq.tier0_poses[0]
    assert state["phase"] == HOLDING
    assert "tier 0 pose 1" in state["error"] and "joint_limits" in state["error"]
    assert state["arm_pose"] == 1 and state["tier"] == 0
    assert_held_not_parked(rig)


def press_stop_on(rig: Rig, number: int) -> None:
    def hook(call: int) -> None:
        if call == number:
            rig.seq.stop()

    rig.capture.on_capture = hook


EXITS = [
    "end",
    "end-skipped-tier2",
    "stop-waiting-board",
    "stop-in-tier",
    "stop-waiting-big-joints",
    "tool-error",
    "head-tool-error",
    "not-converged",
    "exception",
    "timeout-waiting-board",
    "timeout-waiting-big-joints",
]


@pytest.mark.parametrize("exit_kind", EXITS)
def test_every_exit_holds_position(tmp_path: Path, exit_kind: str) -> None:
    cfg = small_cfg(confirm_timeout_s=2.0)
    rig = Rig(tmp_path, cfg)
    seq = rig.seq
    expected_phase = HOLDING
    if exit_kind == "tool-error":
        rig.robot.errors["move_arm_joints"] = 3
    if exit_kind == "head-tool-error":
        rig.robot.errors["set_head_angles"] = 2
    if exit_kind == "not-converged":
        rig.robot.move_status[2] = "timeout"
    if exit_kind == "exception":
        rig.capture.raise_at = 3
    if exit_kind == "stop-in-tier":
        press_stop_on(rig, 2)
    seq.move_to_base(confirmed=True)
    if exit_kind in ("stop-waiting-board", "timeout-waiting-board"):
        rig.wait_phase(WAITING_BOARD)
        if exit_kind == "stop-waiting-board":
            seq.stop()
        expected_phase = IDLE
    else:
        rig.wait_phase(WAITING_BOARD)
        seq.board_attached()
        if exit_kind in ("end", "end-skipped-tier2", "stop-waiting-big-joints", "timeout-waiting-big-joints"):
            rig.wait_phase(WAITING_BIG_JOINTS)
            if exit_kind == "end":
                seq.allow_big_joints(True)
            elif exit_kind == "end-skipped-tier2":
                seq.allow_big_joints(False)
            elif exit_kind == "stop-waiting-big-joints":
                seq.stop()
    state = rig.finish()
    assert state["phase"] == expected_phase
    assert_held_not_parked(rig)
    assert rig.robot.count("get_control_state") == 0
    if exit_kind.startswith("end"):
        assert state["error"] is None
    elif exit_kind.startswith("stop"):
        assert "stopped" in state["status"]
    elif exit_kind == "tool-error":
        assert "injected tool error" in state["error"]
    elif exit_kind == "head-tool-error":
        assert "set_head_angles" in state["error"] and "injected tool error" in state["error"]
    elif exit_kind == "not-converged":
        assert "timeout" in state["error"] and "not converged" in state["error"]
        assert rig.capture.stills == []
    elif exit_kind == "exception":
        assert "RuntimeError: injected capture failure" in state["error"]
    else:
        assert "confirm_timeout_s" in state["error"]
    if expected_phase == HOLDING:
        assert "Remove the board, then press Park" in state["status"]


def test_park_needs_board_removed(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    with pytest.raises(ValueError, match="board"):
        rig.seq.park(board_removed=False)
    assert rig.robot.calls == []
    rig.run_to_holding()
    calls_before = len(rig.robot.calls)
    with pytest.raises(ValueError, match="board"):
        rig.seq.park(board_removed=False)
    assert len(rig.robot.calls) == calls_before
    assert all(args["keep_position"] is True for args in rig.robot.named("release_control"))

    rig.seq.park(board_removed=True)
    state = rig.finish()
    assert state["phase"] == IDLE
    assert state["error"] is None
    assert [name for name, _ in rig.robot.calls[calls_before:]] == [
        "get_control_state",
        "take_control",
        "release_control",
    ]
    assert rig.robot.calls[-1][1] == {"subsystem": "arm", "keep_position": False}
    assert "homed" in state["status"]


def test_park_failure_keeps_holding(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.reset_arm = "failed: timeout"
    rig.seq.park(board_removed=True)
    state = rig.finish()
    assert state["phase"] == HOLDING
    assert "failed: timeout" in state["error"]


def test_park_refused_while_carrying(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = ["holding load on the gripper (effort 0.31)"]
    calls_before = len(rig.robot.calls)
    with pytest.raises(SequenceError, match="holding load on the gripper") as refused:
        rig.seq.park(board_removed=True)
    assert not isinstance(refused.value, Busy)
    assert [name for name, _ in rig.robot.calls[calls_before:]] == ["get_control_state"]
    state = rig.seq.state()
    assert state["phase"] == HOLDING and state["running"] is False
    assert "holding load on the gripper" in state["error"]

    rig.robot.carrying = []
    rig.seq.park(board_removed=True)
    assert rig.finish()["phase"] == IDLE
    for _, args in rig.robot.calls:
        assert RADIUS_KEY not in args


def test_no_call_carries_a_radius(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding(allow=False)
    rig.seq.park(board_removed=True)
    rig.finish()
    assert rig.robot.calls
    for _, args in rig.robot.calls:
        assert RADIUS_KEY not in args
        assert RADIUS_KEY not in args.get("targets", {})


def test_head_aim_offsets(tmp_path: Path) -> None:
    offsets = [(0.0, 0.0), (8.0, -4.0), (500.0, 0.0), (0.0, -200.0)]
    cfg = small_cfg(head_offsets_deg=offsets)
    rig = Rig(tmp_path, cfg)
    rig.run_to_holding()
    heads = rig.robot.named("set_head_angles")
    poses = tier_poses(cfg)
    assert len(heads) == 2 * len(poses)
    for index, (_, pose) in enumerate(poses):
        pan, tilt = expected_aim(cfg, pose)
        for k, (dpan, dtilt) in enumerate(offsets[:2]):
            command = heads[2 * index + k]
            assert command["pan_deg"] == pytest.approx(pan + dpan, abs=1e-6)
            assert command["tilt_deg"] == pytest.approx(tilt + dtilt, abs=1e-6)
    state = rig.seq.state()
    assert state["captured"] == 2 * len(poses)
    assert sum(state["skipped"].values()) == 2 * len(poses)
    assert any("head_pan" in reason and "limit" in reason for reason in state["skipped"])
    assert any("head_tilt" in reason and "limit" in reason for reason in state["skipped"])
    assert {k for _, _, k in rig.capture.stills} == {0, 1}


def test_capture_and_head_outcomes_are_counted(tmp_path: Path) -> None:
    cfg = small_cfg(head_offsets_deg=[(0.0, 0.0), (5.0, 0.0)])
    rig = Rig(tmp_path, cfg)
    rig.capture.outcomes[1] = StillOutcome(False, None, "no board")
    rig.capture.outcomes[2] = StillOutcome(False, None, "no board")
    rig.robot.head_status[3] = "timeout"
    state = rig.run_to_holding()
    total = 2 * len(tier_poses(cfg))
    assert state["skipped"]["no board"] == 2
    assert any("timeout" in reason for reason in state["skipped"])
    assert state["captured"] == total - 3
    assert len(rig.capture.stills) == total - 1


def test_aim_uses_the_applied_head_chain(tmp_path: Path) -> None:
    # A head chain whose mount sits 5 cm further left: the aim follows the URDF the sequencer is given.
    cfg = small_cfg(head_offsets_deg=[(0.0, 0.0)])
    rig = Rig(tmp_path, cfg)
    rig.head_urdf = applied_head()
    rig.run_to_holding()
    first = rig.robot.named("set_head_angles")[0]
    pan, tilt = expected_aim(cfg, tier_poses(cfg)[0][1], applied_head())
    shipped_pan, _ = expected_aim(cfg, tier_poses(cfg)[0][1])
    assert first["pan_deg"] == pytest.approx(pan, abs=1e-6)
    assert first["tilt_deg"] == pytest.approx(tilt, abs=1e-6)
    assert abs(pan - shipped_pan) > 0.5


def applied_head() -> str:
    """The shipped head URDF with head_mount_joint moved 5 cm left (+y)."""
    root = ET.fromstring(HEAD_URDF)
    joint = next(j for j in root.findall("joint") if j.get("name") == "head_mount_joint")
    origin = joint.find("origin")
    assert origin is not None
    x, y, z = (float(v) for v in origin.get("xyz", "0 0 0").split())
    origin.set("xyz", f"{x} {y + 0.05} {z}")
    return ET.tostring(root, encoding="unicode")


def test_busy(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg(), blocked="an RGB-D capture is running")
    with pytest.raises(Busy, match="RGB-D capture"):
        rig.seq.move_to_base(confirmed=True)
    with pytest.raises(Busy, match="RGB-D capture"):
        rig.seq.park(board_removed=True)
    assert rig.robot.calls == []
    assert rig.seq.state()["running"] is False

    rig.blocked_reason = None
    with pytest.raises(Busy):
        rig.seq.board_attached()
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    with pytest.raises(Busy, match="running"):
        rig.seq.move_to_base(confirmed=True)
    with pytest.raises(Busy, match="running"):
        rig.seq.park(board_removed=True)
    rig.seq.board_attached()
    rig.wait_phase(WAITING_BIG_JOINTS)
    rig.seq.allow_big_joints(False)
    assert rig.finish()["phase"] == HOLDING
    # Holding with the board on: a new base move waits for the board removal and Park.
    with pytest.raises(Busy, match="Park"):
        rig.seq.move_to_base(confirmed=True)


def test_concurrent_starts_run_one_sequence(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    outcomes: list[str] = []
    barrier = threading.Barrier(4)

    def start() -> None:
        barrier.wait()
        try:
            rig.seq.move_to_base(confirmed=True)
            outcomes.append("started")
        except Busy:
            outcomes.append("busy")

    threads = [threading.Thread(target=start) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert sorted(outcomes) == ["busy", "busy", "busy", "started"]
    rig.wait_phase(WAITING_BOARD)
    rig.seq.stop()
    rig.finish()
    assert rig.robot.count("take_control") == 1


def test_unusable_urdf_refuses_before_any_call(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.head_urdf = "<robot"
    with pytest.raises(SequenceError, match="URDF"):
        rig.seq.move_to_base(confirmed=True)
    assert rig.robot.calls == []
    assert rig.seq.state()["running"] is False


def test_state_keys(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    state = rig.seq.state()
    assert set(state) == STATE_KEYS
    assert state["phase"] == IDLE and state["running"] is False
    assert state["big_joints"] == BIG_JOINTS
    assert state["captured"] == 0 and state["skipped"] == {}
    assert state["total_poses"] == len(tier_poses(rig.cfg))
    rig.seq.move_to_base(confirmed=True)
    state = rig.wait_phase(WAITING_BOARD)
    assert set(state) == STATE_KEYS
    assert state["waiting_since"] == pytest.approx(rig.clock.now, abs=rig.cfg.arm_sequence.confirm_timeout_s)
    rig.seq.stop()
    state = rig.finish()
    assert set(state) == STATE_KEYS
    assert state["waiting_since"] is None
    assert state["phase"] == IDLE
