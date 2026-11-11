"""Tests for the guided head-from-arm capture sequence (design D37, D38, D46).

The robot is the real RobotClient over a fake mcp_server transport, so every assertion is on the tool calls and their
arguments as mcp_server would receive them. The fake arm reports the commanded joints as measured. The capture is a
fake still-capture controller; the clock is injected and advanced by the injected sleep. The pose check runs on the
shipped arm and head URDFs (nodes/web_ui/urdf) with the shipped pose set. The fake capture's colour camera_info is a
very wide camera unless a test sets the D435 one, so only the min-Z and FOV tests see views skipped by those rules.
"""

import json
import logging
import math
import re
import threading
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseChecker
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_centre

from head_rgbd_calibration.capture import StillOutcome
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.robot_client import RawToolResult, RobotClient
from head_rgbd_calibration.sequencer import (
    CAPTURING,
    EVENT_LIMIT,
    GRIP_CONTACT_MAX_RAD,
    GRIP_CONTACT_MIN_RAD,
    GRIP_RECORD_FILE,
    HOLDING,
    HOMING,
    IDLE,
    MESSAGE_CLEARED,
    MESSAGE_GRIP_CONFIRM,
    MESSAGE_GRIP_RESET_CONFIRM,
    MESSAGE_NOTHING_TO_CLEAR,
    MESSAGE_SLACK_GRIP,
    MESSAGE_UNSEATED,
    REASON_TOO_OLD,
    REASON_UNKNOWN_SESSION,
    SOURCE_CONFIG,
    SOURCE_RECORDED,
    WAITING_BIG_JOINTS,
    WAITING_BOARD,
    Busy,
    SequenceError,
    Sequencer,
    read_grip_record,
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
# The home stored by the servo calibration on 2026-11-11 (client /var/lib/ros2/arm/home.yaml).
CALIBRATED_HOME: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -math.pi / 2,
    "elbow_flex": math.pi / 2,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}
STORED_HOME_GRIPPER = -0.14419419406125028
TOKEN = "seq-token"
NOW = datetime(2026, 10, 23, 8, 0, 0, tzinfo=UTC)
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
    "board_on",
    "board_record",
    "grip",
    "events",
}
EVENT_KEYS = {"time", "phase", "level", "message"}
SEQUENCER_LOGGER = "head_rgbd_calibration.sequencer"
BIG_JOINTS = ["elbow_flex", "shoulder_lift", "shoulder_pan"]
RADIUS_KEY = "carried_object_radius_m"
WAIT_S = 20.0
GRIPPER = "gripper"
ADMIN_PARK_BOARD = "/admin/park_calibration_board"
BOARD_PARK_TOKEN = "board-park-token"
COLOR_FRAME = "d435_color_optical_frame"
MIN_Z_REASON = "inside D435 min-Z"
FOV_REASON = "outside colour FOV"
MIN_DEPTH_M = 0.20  # D435 min-Z at 640x360 (0.15 m) plus the 0.05 m margin
# d435_min_z_m of the near-pose test (shipped 0.15 m): the CAD handle mount keeps the board 0.25 m or more from the D435
# at every pose within the tier bounds and the head tilt limit, so that test raises the min-Z instead
MIN_Z_M = 0.22
NEAR_MIN_DEPTH_M = MIN_Z_M + 0.05
# A tier-2 pose (within the tier bounds) that puts the board (edge handle CAD mount) within 0.27 m of the D435 at every
# head offset, the head within its limits: inside the NEAR_MIN_DEPTH_M limit above.
NEAR_POSE = {"shoulder_pan": -0.47, "shoulder_lift": -0.05, "elbow_flex": 0.0, "wrist_flex": -0.5, "wrist_roll": 0.5}
WIDE_K = np.array([[100.0, 0.0, 320.0], [0.0, 100.0, 180.0], [0.0, 0.0, 1.0]])  # 145 deg across 640 px
D435_FOCAL_PX = 320.0 / math.tan(math.radians(69.0 / 2))  # nominal D435 colour, 69 deg across 640 px
D435_K = np.array([[D435_FOCAL_PX, 0.0, 320.0], [0.0, D435_FOCAL_PX, 180.0], [0.0, 0.0, 1.0]])
IMAGE_SIZE = (640, 360)


def camera_info(k: np.ndarray) -> CameraInfoData:
    """A 640x360 plumb_bob colour camera_info without distortion."""
    p = np.hstack([k, np.zeros((3, 1))]).reshape(12)
    return CameraInfoData(IMAGE_SIZE[0], IMAGE_SIZE[1], "plumb_bob", np.zeros(5), k.reshape(9), p)


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
        reset_arm: reset.arm of a home release.
        jaw_after_close: Gripper angle get_arm_state reports after the gripper-only handle close (None: the gripper is
            not reported after it).
        on_call: Called with the tool name before every call is answered (e.g. to start a move during a measure).
    """

    def __init__(self, log: list[tuple[str, str, dict[str, Any]]], jaw_after_close: float | None) -> None:
        self.log = log
        self.calls: list[tuple[str, dict[str, Any]]] = []
        self.positions: dict[str, float] = {**HOME_POSE, GRIPPER: 0.05}
        self.errors: dict[str, int] = {}
        self.move_status: dict[int, str] = {}
        self.head_status: dict[int, str] = {}
        self.measured: dict[str, float] | None = None
        self.carrying: list[str] = []
        self.reset_arm = "homed"
        self.jaw_after_close = jaw_after_close
        self.on_call: Callable[[str], None] | None = None

    def count(self, name: str) -> int:
        return sum(1 for called, _ in self.calls if called == name)

    def named(self, name: str) -> list[dict[str, Any]]:
        return [args for called, args in self.calls if called == name]

    async def __call__(self, url: str, token: str, name: str, args: dict[str, Any]) -> RawToolResult:
        assert token == (BOARD_PARK_TOKEN if name == ADMIN_PARK_BOARD else TOKEN)
        self.calls.append((name, dict(args)))
        self.log.append(("robot", name, dict(args)))
        if self.on_call is not None:
            self.on_call(name)
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
                if set(args["targets"]) == {GRIPPER}:  # the handle close: the jaw stops where the handle is
                    if self.jaw_after_close is None:
                        self.positions.pop(GRIPPER)
                    else:
                        self.positions[GRIPPER] = self.jaw_after_close
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
        if name == ADMIN_PARK_BOARD:  # the internal admin route of mcp_server, not an MCP tool
            assert set(args) == {"mount"}  # mcp_server owns the outline and margins: the request names the mount only
            assert set(args["mount"]) == {"xyz", "rpy"}
            reset = {"arm": self.reset_arm, "head": "homed"}
            return RawToolResult(False, "ok", {"ok": True, "result": {"released": ["arm"], "reset": reset}})
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
        camera_info: The colour camera_info still_camera_info reports (None: none has arrived).
        info_after: still_camera_info reports None for its first info_after calls.
    """

    def __init__(self, log: list[tuple[str, str, dict[str, Any]]]) -> None:
        self.log = log
        self.stills: list[tuple[int, int, int]] = []
        self.stops = 0
        self.sessions: list[str | None] = []
        self.outcomes: dict[int, StillOutcome] = {}
        self.raise_at: int | None = None
        self.on_capture: Callable[[int], None] | None = None
        self.camera_info: CameraInfoData | None = camera_info(WIDE_K)
        self.info_after = 0
        self.info_calls = 0

    def still_camera_info(self) -> CameraInfoData | None:
        self.info_calls += 1
        return self.camera_info if self.info_calls > self.info_after else None

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

    def __init__(
        self,
        tmp_path: Path,
        cfg: NodeConfig | None = None,
        blocked: str | None = None,
        fitted_mount: Callable[[str | None], dict[str, list[float]] | None] | None = None,
        board_file: Path | None = None,
        session_exists: Callable[[str | None], bool] | None = None,
        now: Callable[[], datetime] | None = None,
        home: dict[str, float] | None = None,
        grip_file: Path | None = None,
    ) -> None:
        token = tmp_path / "token"
        token.write_text(f"MCP_SERVER_TOKEN={TOKEN}\n")
        self.home_file = tmp_path / "home.yaml"  # mcp_server's stored home, as its home_store writes it
        self.write_home(home or HOME_POSE)
        base = (cfg or NodeConfig()).model_copy(update={"arm_home_file": str(self.home_file)})
        board_token = tmp_path / "board_park_token"
        board_token.write_text("board-park-token\n")
        robot_cfg = base.robot.model_copy(update={"token_file": str(token), "board_park_token_file": str(board_token)})
        self.cfg = base.model_copy(update={"robot": robot_cfg})
        self.log: list[tuple[str, str, dict[str, Any]]] = []
        self.robot = FakeRobot(self.log, self.cfg.arm_sequence.handle_contact_rad)  # the jaw stops on the handle
        self.capture = FakeCapture(self.log)
        self.clock = FakeClock()
        self.blocked_reason = blocked
        self.head_urdf = HEAD_URDF
        self.seq = Sequencer(
            self.capture,
            RobotClient(self.cfg.robot, call=self.robot, admin_call=self.robot),
            self.cfg,
            lambda: self.head_urdf,
            lambda: ARM_URDF,
            lambda: self.blocked_reason,
            sleep=self.clock.sleep,
            clock=self.clock.clock,
            fitted_mount=fitted_mount or (lambda _session_id: None),
            board_file=board_file,
            session_exists=session_exists or (lambda session_id: session_id == SESSION),
            now=now or (lambda: NOW),
            grip_file=grip_file,
        )

    def write_home(self, home: dict[str, float]) -> None:
        """Store an arm home (with a gripper entry, as mcp_server's Set home does) in the rig's home file.

        Args:
            home (dict[str, float]): The five arm joints, rad.
        """
        self.home_file.write_text(yaml.safe_dump({"joints": {**home, GRIPPER: STORED_HOME_GRIPPER}}, sort_keys=False))

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

    def run_to_capture(self, jaw_after_close: float | None) -> None:
        """Run to the board confirm, press Board attached with the jaw reporting `jaw_after_close` after the close.

        Args:
            jaw_after_close: Gripper angle (rad, or None for not reported) get_arm_state reports after the close.
        """
        self.robot.jaw_after_close = jaw_after_close
        self.seq.move_to_base(confirmed=True)
        self.wait_phase(WAITING_BOARD)
        self.seq.board_attached()

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


def assert_held_not_homed(rig: Rig) -> None:
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
    assert (
        "Seat the arm board handle in the gripper jaws up to the shoulder, then press Board attached" in state["status"]
    )
    assert state["waiting_since"] is not None
    assert rig.names() == ["take_control", "get_arm_state", "move_arm_joints"]
    assert rig.robot.named("move_arm_joints")[0] == {
        "targets": {**seq_cfg.base_pose, GRIPPER: seq_cfg.insert_gripper_rad},
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
    assert "Press Home" in state["status"]
    assert state["error"] is None
    assert state["session_id"] == SESSION

    poses = tier_poses(cfg)
    offsets = seq_cfg.head_offsets_deg
    expected: list[str] = [
        "take_control",
        "get_arm_state",
        "move_arm_joints",
        "get_arm_state",
        "move_arm_joints",
        "get_arm_state",
        "start_still_session",
    ]
    for _ in poses:
        expected += ["get_arm_state", "move_arm_joints", "get_arm_state"]
        expected += ["set_head_angles", "capture_still"] * len(offsets)
    expected += ["get_arm_state", "move_arm_joints", "stop", "release_control"]
    assert rig.names() == expected
    assert rig.capture.sessions == [None]
    moves = rig.robot.named("move_arm_joints")
    assert moves[1] == {"targets": {GRIPPER: pytest.approx(seq_cfg.grip_rad)}, "speed_scale": seq_cfg.speed_scale}
    assert [m["targets"] for m in moves[2:-1]] == [pose for _, pose in poses]
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


def test_gripper_opens_at_base_and_closes_on_the_handle(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    seq = rig.cfg.arm_sequence
    rig.run_to_holding()
    moves = [m["targets"] for m in rig.robot.named("move_arm_joints")]
    assert len(moves) > 3
    assert moves[0][GRIPPER] == seq.insert_gripper_rad
    assert moves[1] == {GRIPPER: pytest.approx(seq.grip_rad)}
    assert all(GRIPPER not in m for m in moves[2:])


def test_unseated_handle_stops_the_sequence(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    seq = rig.cfg.arm_sequence
    rig.run_to_capture(jaw_after_close=seq.grip_rad)  # the jaw closed past the contact: nothing in it
    state = rig.finish()
    assert MESSAGE_UNSEATED in state["error"]
    assert f"{seq.grip_rad:.4f}" in state["error"]
    assert rig.robot.calls[-1] == ("release_control", {"subsystem": "arm", "keep_position": True})
    assert rig.capture.sessions == [] and rig.capture.stills == []
    assert rig.robot.count("move_arm_joints") == 2


def test_handle_close_at_the_unseated_threshold(tmp_path: Path) -> None:
    seq = small_cfg().arm_sequence
    limit = seq.handle_contact_rad - seq.handle_squeeze_rad / 2
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_capture(jaw_after_close=limit + 1e-3)  # a seated handle with the gripper zero offset of the real arm
    rig.wait_phase(WAITING_BIG_JOINTS)
    rig.seq.stop()
    rig.finish()
    assert rig.capture.sessions == [None]
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_capture(jaw_after_close=limit - 1e-3)
    assert MESSAGE_UNSEATED in rig.finish()["error"]


def test_slack_grip_after_the_handle_close_stops_the_sequence(tmp_path: Path) -> None:
    # The jaw stopped on a handle thicker than the CAD contact: mcp_server reports the residual as converged, then
    # relaxes the hold to the measured pose, so the board would hang with no squeeze. The sequence stops holding.
    seq = small_cfg().arm_sequence
    for residual in (seq.grip_converge_tolerance_rad, seq.grip_converge_tolerance_rad + 0.005):
        rig = Rig(tmp_path, small_cfg())
        rig.run_to_capture(jaw_after_close=seq.grip_rad + residual)
        state = rig.finish()
        assert MESSAGE_SLACK_GRIP in state["error"]
        assert f"{seq.grip_rad + residual:.4f}" in state["error"]
        assert rig.robot.calls[-1] == ("release_control", {"subsystem": "arm", "keep_position": True})
        assert rig.capture.sessions == [] and rig.capture.stills == []
        assert rig.robot.count("move_arm_joints") == 2


def test_handle_close_with_a_residual_inside_the_converge_tolerance_holds(tmp_path: Path) -> None:
    seq = small_cfg().arm_sequence
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_capture(jaw_after_close=seq.grip_rad + seq.grip_converge_tolerance_rad - 1e-3)
    rig.wait_phase(WAITING_BIG_JOINTS)
    rig.seq.stop()
    rig.finish()
    assert rig.capture.sessions == [None]


def test_handle_close_without_a_measured_gripper_stops(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_capture(jaw_after_close=None)
    state = rig.finish()
    assert "reports no gripper" in state["error"]
    assert rig.robot.calls[-1] == ("release_control", {"subsystem": "arm", "keep_position": True})
    assert rig.capture.sessions == []


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
    assert_held_not_homed(rig)

    # A measured arm joint missing: nothing is commanded.
    rig = Rig(tmp_path, small_cfg())
    rig.robot.measured = {name: value for name, value in HOME_POSE.items() if name != "elbow_flex"}
    rig.seq.move_to_base(confirmed=True)
    state = rig.finish()
    assert rig.robot.count("move_arm_joints") == 0
    assert "elbow_flex" in state["error"]
    assert_held_not_homed(rig)

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
    assert_held_not_homed(rig)


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
    "grip-tool-error",
    "head-tool-error",
    "not-converged",
    "grip-not-converged",
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
    # move_arm_joints calls: 1 open to the base pose, 2 the gripper close on the handle, 3 and up the tier moves
    if exit_kind == "tool-error":
        rig.robot.errors["move_arm_joints"] = 4
    if exit_kind == "grip-tool-error":
        rig.robot.errors["move_arm_joints"] = 2
    if exit_kind == "head-tool-error":
        rig.robot.errors["set_head_angles"] = 2
    if exit_kind == "not-converged":
        rig.robot.move_status[3] = "timeout"
    if exit_kind == "grip-not-converged":
        rig.robot.move_status[2] = "timeout"
    if exit_kind == "exception":
        rig.capture.raise_at = 3
    if exit_kind == "stop-in-tier":
        press_stop_on(rig, 2)
    seq.move_to_base(confirmed=True)
    if exit_kind in ("stop-waiting-board", "timeout-waiting-board"):
        # The fake clock runs the confirm timeout out in milliseconds, so a slow poller may only see the end state.
        rig.wait_for(lambda s: s["phase"] == WAITING_BOARD or not s["running"])
        if exit_kind == "stop-waiting-board":
            seq.stop()
        expected_phase = IDLE
    else:
        rig.wait_phase(WAITING_BOARD)
        seq.board_attached()
        if exit_kind in ("end", "end-skipped-tier2", "stop-waiting-big-joints", "timeout-waiting-big-joints"):
            rig.wait_for(lambda s: s["phase"] == WAITING_BIG_JOINTS or not s["running"])
            if exit_kind == "end":
                seq.allow_big_joints(True)
            elif exit_kind == "end-skipped-tier2":
                seq.allow_big_joints(False)
            elif exit_kind == "stop-waiting-big-joints":
                seq.stop()
    state = rig.finish()
    assert state["phase"] == expected_phase
    assert_held_not_homed(rig)
    assert rig.robot.count("get_control_state") == 0
    if exit_kind.startswith("end"):
        assert state["error"] is None
    elif exit_kind.startswith("stop"):
        assert "stopped" in state["status"]
    elif exit_kind in ("tool-error", "grip-tool-error"):
        assert "injected tool error" in state["error"]
    elif exit_kind == "head-tool-error":
        assert "set_head_angles" in state["error"] and "injected tool error" in state["error"]
    elif exit_kind in ("not-converged", "grip-not-converged"):
        assert "timeout" in state["error"] and "not converged" in state["error"]
        assert rig.capture.stills == []
    elif exit_kind == "exception":
        assert "RuntimeError: injected capture failure" in state["error"]
    else:
        assert "confirm_timeout_s" in state["error"]
    if expected_phase == HOLDING:
        assert "Press Home" in state["status"]


def test_home_needs_confirm(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    with pytest.raises(ValueError, match="confirm"):
        rig.seq.home(confirmed=False)
    assert rig.robot.calls == []
    rig.run_to_holding()
    calls_before = len(rig.robot.calls)
    with pytest.raises(ValueError, match="confirm"):
        rig.seq.home(confirmed=False)
    assert len(rig.robot.calls) == calls_before
    assert all(args["keep_position"] is True for args in rig.robot.named("release_control"))

    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert state["phase"] == IDLE
    assert state["error"] is None
    # The sequence clipped the board: the Home is the checked board home, never the plain one.
    assert [name for name, _ in rig.robot.calls[calls_before:]] == [
        "get_control_state",
        "get_arm_state",
        "take_control",
        "move_arm_joints",
        ADMIN_PARK_BOARD,
    ]
    assert "homed" in state["status"]


def test_home_failure_keeps_holding(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.reset_arm = "failed: timeout"
    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert state["phase"] == HOLDING
    assert "failed: timeout" in state["error"]


def test_home_refused_while_carrying(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = ["last grasp held an object"]
    calls_before = len(rig.robot.calls)
    with pytest.raises(SequenceError, match="last grasp held an object") as refused:
        rig.seq.home(confirmed=True)
    assert not isinstance(refused.value, Busy)
    assert [name for name, _ in rig.robot.calls[calls_before:]] == ["get_control_state"]
    state = rig.seq.state()
    assert state["phase"] == HOLDING and state["running"] is False
    assert "last grasp held an object" in state["error"]

    rig.robot.carrying = []
    rig.seq.home(confirmed=True)
    assert rig.finish()["phase"] == IDLE
    for _, args in rig.robot.calls:
        assert RADIUS_KEY not in args


BOARD_LOAD = "holding load: jaw -0.040 rad (0.125 rad short of closed), gripper load 200"


def home_calls(rig: Rig, since: int) -> list[str]:
    return [name for name, _ in rig.robot.calls[since:]]


def test_home_with_the_board_on_is_checked_moved_then_homed_through_the_internal_route(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]  # the jaw clamps the board's stick: the live run's refusal
    before = len(rig.robot.calls)
    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert state["phase"] == IDLE and state["error"] is None
    assert home_calls(rig, before) == [
        "get_control_state",
        "get_arm_state",
        "take_control",
        "move_arm_joints",
        ADMIN_PARK_BOARD,
    ]
    seq_cfg = rig.cfg.arm_sequence
    move = rig.robot.named("move_arm_joints")[-1]
    assert move == {"targets": HOME_POSE, "speed_scale": seq_cfg.speed_scale} and seq_cfg.speed_scale == 0.1
    # No release_control after the sequence's hold: the Home is the admin route, which checks the board itself.
    assert all(args["keep_position"] is True for args in rig.robot.named("release_control"))
    mount = rig.cfg.fit.board_mount
    assert rig.robot.named(ADMIN_PARK_BOARD)[-1] == {
        "mount": {"xyz": list(mount.xyz_init), "rpy": list(mount.rpy_init)}
    }
    assert "homed" in state["status"]


def test_home_with_the_board_on_moves_to_the_stored_home_read_at_each_press(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg(), home=CALIBRATED_HOME)
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    rig.seq.home(confirmed=True)
    assert rig.finish()["phase"] == IDLE
    # The five arm joints of the stored home; its gripper entry is no target (the jaw keeps holding the handle).
    assert rig.robot.named("move_arm_joints")[-1]["targets"] == CALIBRATED_HOME
    # A re-calibration stores a new home: the next press reads it.
    rig.write_home(HOME_POSE)
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    rig.seq.home(confirmed=True)
    assert rig.finish()["phase"] == IDLE
    assert rig.robot.named("move_arm_joints")[-1]["targets"] == HOME_POSE


@pytest.mark.parametrize("stored", ["missing", "invalid"])
def test_home_with_the_board_on_refuses_without_a_valid_stored_home(tmp_path: Path, stored: str) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    if stored == "missing":
        rig.home_file.unlink()
        expected = f"no stored arm home at {rig.home_file}: store the arm home (Calibrate > General, Set home) first"
    else:
        rig.home_file.write_text(yaml.safe_dump({"joints": {"shoulder_pan": 0.0}}))
        expected = f"invalid arm home {rig.home_file}: missing joint shoulder_lift"
    before = len(rig.robot.calls)
    with pytest.raises(SequenceError) as refused:
        rig.seq.home(confirmed=True)
    assert not isinstance(refused.value, Busy)
    assert expected in str(refused.value)
    assert home_calls(rig, before) == ["get_control_state"]  # nothing moved, the arm keeps holding
    state = rig.seq.state()
    assert state["phase"] == HOLDING and state["running"] is False
    assert expected in state["error"]


def test_home_with_the_board_on_is_checked_even_when_no_load_is_reported(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg(park_board_sag_margin_m=1.0))
    rig.run_to_holding()
    assert rig.robot.carrying == []  # board_on from the sequence, but the load reads zero: still the checked flow
    before = len(rig.robot.calls)
    with pytest.raises(SequenceError, match="Home refused by the path check"):
        rig.seq.home(confirmed=True)
    assert home_calls(rig, before) == ["get_control_state", "get_arm_state"]
    rig2 = Rig(tmp_path, small_cfg())
    rig2.run_to_holding()
    rig2.seq.home(confirmed=True)
    assert rig2.finish()["phase"] == IDLE
    assert rig2.robot.named(ADMIN_PARK_BOARD) and rig2.robot.count("move_arm_joints") > 0


def test_home_without_any_board_is_the_plain_home(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())  # nothing carried and no board stated: mcp_server's plain home (release_control)
    rig.seq.home(confirmed=True)
    assert rig.finish()["phase"] == IDLE
    assert rig.robot.calls[-1] == ("release_control", {"subsystem": "arm", "keep_position": False})
    assert rig.robot.count(ADMIN_PARK_BOARD) == 0


@pytest.mark.parametrize("sag", [0.2, 1.0])
def test_home_with_the_board_refused_when_the_board_path_hits_the_floor_body_or_head(
    tmp_path: Path, sag: float
) -> None:
    rig = Rig(tmp_path, small_cfg(park_board_sag_margin_m=sag))
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    before = len(rig.robot.calls)
    with pytest.raises(SequenceError, match="Home refused by the path check") as refused:
        rig.seq.home(confirmed=True)
    assert not isinstance(refused.value, Busy)
    assert re.search(r"min_board_height_m|board_clearance|board_link_clearance", str(refused.value))
    assert home_calls(rig, before) == ["get_control_state", "get_arm_state"]  # the arm stays holding
    state = rig.seq.state()
    assert state["phase"] == HOLDING and state["running"] is False
    assert "path check" in state["error"]


def test_home_with_the_board_refused_by_mcp_server_keeps_holding(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    rig.robot.errors[ADMIN_PARK_BOARD] = 1
    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert state["phase"] == HOLDING and "injected tool error" in state["error"]
    assert rig.robot.named("release_control")[-1] == {"subsystem": "arm", "keep_position": True}


def test_home_with_the_board_failing_move_keeps_holding(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    moves = rig.robot.count("move_arm_joints")
    rig.robot.move_status[moves + 1] = "timeout"
    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert state["phase"] == HOLDING and "did not converge" in state["error"]
    assert rig.robot.count(ADMIN_PARK_BOARD) == 0
    assert rig.robot.named("release_control")[-1] == {"subsystem": "arm", "keep_position": True}


def test_home_with_the_board_sends_the_fitted_mount_to_mcp_server(tmp_path: Path) -> None:
    mount = small_cfg().fit.board_mount
    fitted = {"xyz": [mount.xyz_init[0] + 0.004, mount.xyz_init[1], mount.xyz_init[2]], "rpy": list(mount.rpy_init)}
    rig = Rig(tmp_path, small_cfg(), fitted_mount=lambda session_id: fitted)
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    rig.seq.home(confirmed=True)
    assert rig.finish()["phase"] == IDLE
    assert rig.robot.named(ADMIN_PARK_BOARD)[-1] == {"mount": fitted}


def test_home_with_the_board_uses_the_fitted_mount_when_a_result_exists(tmp_path: Path) -> None:
    seen: list[str | None] = []
    far = {"xyz": [2.0, 0.0, 0.5], "rpy": [0.0, 0.0, 0.0]}  # a mount that puts the board far off the gripper

    def fitted(session_id: str | None) -> dict[str, list[float]] | None:
        seen.append(session_id)
        return far

    rig = Rig(tmp_path, small_cfg(), fitted_mount=fitted)
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    with pytest.raises(SequenceError, match="path check"):
        rig.seq.home(confirmed=True)
    assert seen == [SESSION]


@pytest.mark.parametrize(
    "reasons",
    [
        ["last grasp held an object"],
        ["carried_object_radius_m 0.08 m set at the last arm release"],
        [BOARD_LOAD, "last grasp held an object"],
    ],
)
def test_home_with_another_carried_object_is_still_refused(tmp_path: Path, reasons: list[str]) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = reasons
    before = len(rig.robot.calls)
    with pytest.raises(SequenceError, match="mcp_server reports carrying"):
        rig.seq.home(confirmed=True)
    assert home_calls(rig, before) == ["get_control_state"]


def test_home_with_a_load_but_no_board_from_this_sequence_is_refused(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())  # no sequence ran and the user did not state the board
    rig.robot.carrying = [BOARD_LOAD]
    with pytest.raises(SequenceError, match="mcp_server reports carrying"):
        rig.seq.home(confirmed=True)
    assert [name for name, _ in rig.robot.calls] == ["get_control_state"]


def after_restart(rig: Rig, tmp_path: Path, board_file: Path, **kwargs: Any) -> Rig:
    """A new sequencer (a node restart) on the same state file; the arm still holds the clamped board."""
    fresh = Rig(tmp_path, kwargs.pop("cfg", rig.cfg), board_file=board_file, **kwargs)
    fresh.robot.positions = dict(rig.robot.positions)
    fresh.robot.carrying = list(rig.robot.carrying)
    return fresh


def test_board_record_is_written_with_the_session_and_pre_fills_after_a_restart(tmp_path: Path) -> None:
    board_file = tmp_path / "state" / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    record = json.loads(board_file.read_text())
    assert record["session_id"] == SESSION and record["clipped_at"] and set(record["mount"]) == {"xyz", "rpy"}
    rig.robot.carrying = [BOARD_LOAD]
    fresh = after_restart(rig, tmp_path, board_file)
    state = fresh.seq.state()
    assert state["phase"] == IDLE  # the record is not the state: the user confirms it again
    assert state["board_record"] == {
        "file": str(board_file),
        "session_id": SESSION,
        "clipped_at": record["clipped_at"],
        "age_s": 0.0,
        "offered": True,
        "problem": None,
    }
    with pytest.raises(SequenceError, match="state that the board is on") as refused:
        fresh.seq.home(confirmed=True)
    assert "mcp_server reports carrying" in str(refused.value)
    assert [name for name, _ in fresh.robot.calls] == ["get_control_state"]
    fresh.seq.home(confirmed=True, board_on=True)  # the fresh confirmation in the UI
    assert fresh.finish()["phase"] == IDLE
    assert fresh.robot.named(ADMIN_PARK_BOARD)
    assert not board_file.exists()  # a completed Home clears it


def test_stale_board_record_is_ignored_and_a_non_board_load_is_refused(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    max_age = rig.cfg.arm_sequence.board_record_max_age_s
    assert max_age == 7200.0
    fresh = after_restart(rig, tmp_path, board_file, now=lambda: NOW + timedelta(seconds=max_age + 60))
    shown = fresh.seq.state()["board_record"]  # still shown (mcp_server refuses Home while it exists), not offered
    assert (shown["session_id"], shown["age_s"]) == (SESSION, max_age + 60)
    assert (shown["offered"], shown["problem"]) == (False, REASON_TOO_OLD)
    with pytest.raises(SequenceError, match="mcp_server reports carrying"):
        fresh.seq.home(confirmed=True)
    assert [name for name, _ in fresh.robot.calls] == ["get_control_state"]


def test_board_record_of_another_session_is_ignored(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    fresh = after_restart(rig, tmp_path, board_file, session_exists=lambda session_id: False)
    shown = fresh.seq.state()["board_record"]
    assert (shown["session_id"], shown["offered"], shown["problem"]) == (SESSION, False, REASON_UNKNOWN_SESSION)
    board_file.write_text("not json")
    unreadable = after_restart(rig, tmp_path, board_file).seq.state()["board_record"]
    assert unreadable["file"] == str(board_file) and unreadable["offered"] is False
    assert (unreadable["session_id"], unreadable["clipped_at"], unreadable["age_s"]) == (None, None, None)
    assert unreadable["problem"].startswith("unreadable: ")


def test_restored_board_home_is_still_path_checked(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    fresh = after_restart(rig, tmp_path, board_file, cfg=small_cfg(park_board_sag_margin_m=1.0))
    fresh.robot.carrying = [BOARD_LOAD]
    with pytest.raises(SequenceError, match="path check"):
        fresh.seq.home(confirmed=True, board_on=True)
    assert board_file.exists()  # the arm still holds the board: the record stays for the retry


def test_user_states_the_board_is_on_and_it_is_recorded_then_path_checked(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)  # a node that never saw the clip (deploy while holding)
    rig.robot.carrying = [BOARD_LOAD]
    with pytest.raises(SequenceError, match="state that the board is on"):
        rig.seq.home(confirmed=True)
    assert not board_file.exists()
    rig.seq.home(confirmed=True, board_on=True)
    state = rig.finish()
    assert state["phase"] == IDLE and rig.robot.named(ADMIN_PARK_BOARD)
    assert not board_file.exists()
    rig2 = Rig(tmp_path, small_cfg(park_board_sag_margin_m=1.0), board_file=board_file)
    rig2.robot.carrying = [BOARD_LOAD]
    with pytest.raises(SequenceError, match="path check"):
        rig2.seq.home(confirmed=True, board_on=True)
    assert json.loads(board_file.read_text())["session_id"] is None


def test_board_record_is_shown_from_the_attach_on_and_none_without_a_file(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    assert rig.seq.state()["board_record"] is None
    rig.run_to_holding()
    shown = rig.seq.state()["board_record"]
    assert (shown["session_id"], shown["offered"]) == (SESSION, True)
    assert Rig(tmp_path, small_cfg()).seq.state()["board_record"] is None  # no board_file: nothing kept


def test_clear_board_record_needs_confirm_and_changes_nothing(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    with pytest.raises(ValueError, match="off the gripper"):
        rig.seq.clear_board_record(confirmed=False)
    assert board_file.exists() and rig.seq.state()["phase"] == HOLDING


def test_clear_board_record_removes_an_old_record_without_any_motion(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    max_age = rig.cfg.arm_sequence.board_record_max_age_s
    fresh = after_restart(rig, tmp_path, board_file, now=lambda: NOW + timedelta(days=1))
    assert fresh.seq.state()["board_record"]["problem"] == REASON_TOO_OLD and max_age < 86400
    caplog.set_level(logging.INFO, logger=SEQUENCER_LOGGER)
    state = fresh.seq.clear_board_record(confirmed=True)
    assert not board_file.exists()
    assert state["board_record"] is None and state["status"] == MESSAGE_CLEARED
    assert state["events"][0]["message"] == MESSAGE_CLEARED
    assert fresh.robot.calls == []  # nothing moved, mcp_server was not called
    lines = [r.getMessage() for r in caplog.records if r.name == SEQUENCER_LOGGER]
    cleared = [line for line in lines if '"step":"board_record_clear"' in line and '"outcome":"cleared"' in line]
    assert len(cleared) == 1 and '"run_id":"board-' in cleared[0] and str(board_file) in cleared[0]


def test_clear_board_record_after_a_held_sequence_returns_to_idle_and_home_is_plain(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    assert rig.run_to_holding()["phase"] == HOLDING
    state = rig.seq.clear_board_record(confirmed=True)
    assert state["phase"] == IDLE and not board_file.exists()
    since = len(rig.robot.calls)
    rig.seq.home(confirmed=True)  # the board is off: the plain home, no board route
    rig.finish()
    assert ADMIN_PARK_BOARD not in home_calls(rig, since)


def test_clear_board_record_without_a_record_says_so(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg(), board_file=tmp_path / "board.json")
    state = rig.seq.clear_board_record(confirmed=True)
    assert state["status"] == MESSAGE_NOTHING_TO_CLEAR and state["events"][0]["message"] == MESSAGE_NOTHING_TO_CLEAR


def test_clear_board_record_is_refused_while_a_sequence_runs(tmp_path: Path) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    rig.seq.board_attached()
    rig.wait_phase(WAITING_BIG_JOINTS)
    assert board_file.exists()
    with pytest.raises(Busy, match="running"):
        rig.seq.clear_board_record(confirmed=True)
    assert board_file.exists()
    rig.seq.allow_big_joints(False)
    rig.finish()


def test_no_call_carries_a_radius(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding(allow=False)
    rig.seq.home(confirmed=True)
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
        rig.seq.home(confirmed=True)
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
        rig.seq.home(confirmed=True)
    rig.seq.board_attached()
    rig.wait_phase(WAITING_BIG_JOINTS)
    rig.seq.allow_big_joints(False)
    assert rig.finish()["phase"] == HOLDING
    # Holding with the board on: a new base move waits for the board removal and Home.
    with pytest.raises(Busy, match="Home"):
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


def corners_in_camera(cfg: NodeConfig, head_urdf: str, joints: dict[str, float], pan_deg: float, tilt_deg: float):
    """The ChArUco corners (nominal mount) in the colour optical frame, FK and head chain straight from the URDFs."""
    fit = cfg.fit
    gripper = arm_in_robot_base(fit.base_in_base_link.model_dump()) @ UrdfChain.from_text(ARM_URDF).transform(
        "gripper_link", joints, offsets=fit.joint_offsets_rad
    )
    board = gripper @ pose_from_xyz_rpy(fit.board_mount.xyz_init, fit.board_mount.rpy_init)
    angles = {"head_pan": math.radians(pan_deg), "head_tilt": math.radians(tilt_deg)}
    camera = UrdfChain.from_text(head_urdf).transform(COLOR_FRAME, angles)
    corners = board_corners_3d(fit.board)
    return (np.linalg.solve(camera, board) @ np.column_stack([corners, np.ones(len(corners))]).T).T[:, :3]


def expected_views(
    cfg: NodeConfig, head_urdf: str, k: np.ndarray, min_depth: float = MIN_DEPTH_M
) -> list[tuple[int, int, float, float, str | None]]:
    """(arm pose, head offset, pan deg, tilt deg, skip reason) of every view of the pose set, computed independently:
    outside the head limits, then any corner nearer than `min_depth` (m), then any corner outside the 640x360 image."""
    chain = UrdfChain.from_text(head_urdf)
    out = []
    for index, (_, pose) in enumerate(tier_poses(cfg)):
        pan0, tilt0 = expected_aim(cfg, pose, head_urdf)
        for offset, (dpan, dtilt) in enumerate(cfg.arm_sequence.head_offsets_deg):
            pan, tilt = pan0 + dpan, tilt0 + dtilt
            reason = None
            for joint, value in (("head_pan", pan), ("head_tilt", tilt)):
                lower, upper = chain.limits(joint)
                if reason is None and not lower <= math.radians(value) <= upper:
                    reason = f"{joint} outside the URDF head limits"
            if reason is None:
                points = corners_in_camera(cfg, head_urdf, pose, pan, tilt)
                pixels = (k @ points.T).T
                pixels = pixels[:, :2] / pixels[:, 2:3]
                inside = (
                    (pixels[:, 0] >= 0)
                    & (pixels[:, 0] <= IMAGE_SIZE[0] - 1)
                    & (pixels[:, 1] >= 0)
                    & (pixels[:, 1] <= IMAGE_SIZE[1] - 1)
                )
                if points[:, 2].min() < min_depth:
                    reason = MIN_Z_REASON
                elif not inside.all():
                    reason = FOV_REASON
            out.append((index, offset, pan, tilt, reason))
    return out


def assert_rule_skips(rig: Rig, views: list[tuple[int, int, float, float, str | None]], reason: str) -> None:
    """The run skipped exactly the expected views with their reason, captured the others, and never moved the arm to
    a pose whose every view is skipped."""
    cfg = rig.cfg
    state = rig.run_to_holding()
    assert state["error"] is None, state
    poses = tier_poses(cfg)
    skipped = [(i, k) for i, k, _, _, why in views if why is not None]
    kept = [(i, k, pan, tilt) for i, k, pan, tilt, why in views if why is None]
    assert state["skipped"].get(reason, 0) == sum(1 for *_, why in views if why == reason) > 0
    assert sum(state["skipped"].values()) == len(skipped)
    assert [(i, k) for i, _, k in rig.capture.stills] == [(i, k) for i, k, _, _ in kept]
    assert state["captured"] == len(kept) > 0
    heads = rig.robot.named("set_head_angles")
    assert len(heads) == len(kept)
    for command, (_, _, pan, tilt) in zip(heads, kept, strict=True):
        assert (command["pan_deg"], command["tilt_deg"]) == (
            pytest.approx(pan, abs=1e-6),
            pytest.approx(tilt, abs=1e-6),
        )
    visited = sorted({i for i, *_ in kept})
    moved = [m["targets"] for m in rig.robot.named("move_arm_joints")]
    assert moved[2:-1] == [poses[i][1] for i in visited]


def test_poses_inside_min_z_are_skipped(tmp_path: Path) -> None:
    """A board pose with a ChArUco corner nearer than 0.20 m to the D435 colour camera (0.15 m min-Z + 0.05 m margin; the
    near-pose half raises the min-Z to 0.22 m, as no pose within the tier bounds gets the CAD handle board that close)
    or outside its colour FOV is skipped with the reason, and a pose whose every view is skipped is not visited; the
    tiers and the big-joint confirm are unchanged."""
    seq = NodeConfig().arm_sequence
    pose_check = seq.pose_check.model_copy(update={"d435_min_z_m": MIN_Z_M})
    cfg = small_cfg(tier2_poses=[seq.tier2_poses[0], NEAR_POSE, seq.tier2_poses[1]], pose_check=pose_check)
    rig = Rig(tmp_path, cfg)
    views = expected_views(rig.cfg, HEAD_URDF, WIDE_K, NEAR_MIN_DEPTH_M)
    near = [why for i, *_, why in views if tier_poses(cfg)[i][1] == NEAR_POSE]
    assert near == [MIN_Z_REASON] * len(cfg.arm_sequence.head_offsets_deg)
    assert {why for *_, why in views} == {None, MIN_Z_REASON}
    assert_rule_skips(rig, views, MIN_Z_REASON)
    assert NEAR_POSE not in [m["targets"] for m in rig.robot.named("move_arm_joints")]
    assert {tier for _, tier, _ in rig.capture.stills} == {0, 1, 2}

    # The nominal D435 colour camera (69 deg across 640 px): views with a corner outside the image are skipped, the
    # others of the same pose are captured. A +30 deg pan offset pushes the (smaller) CAD handle board out of the 640 px
    # image width.
    rig = Rig(tmp_path, small_cfg(head_offsets_deg=[*seq.head_offsets_deg, (30.0, 0.0)]))
    rig.capture.camera_info = camera_info(D435_K)
    views = expected_views(rig.cfg, HEAD_URDF, D435_K)
    assert {why for *_, why in views} == {None, FOV_REASON}
    assert_rule_skips(rig, views, FOV_REASON)


def test_sequence_needs_the_colour_camera_info(tmp_path: Path) -> None:
    """The FOV rule needs the colour camera_info: the sequence waits for it up to still.timeout_s after the still
    session starts, then fails holding the arm, before any tier move."""
    rig = Rig(tmp_path, small_cfg())
    rig.capture.info_after = 3
    state = rig.run_to_holding()
    assert state["error"] is None and state["captured"] > 0

    rig = Rig(tmp_path, small_cfg())
    rig.capture.camera_info = None
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    rig.seq.board_attached()
    state = rig.finish()
    assert state["phase"] == HOLDING
    assert "no colour camera_info" in state["error"] and "6 s" in state["error"]
    assert rig.robot.count("move_arm_joints") == 2 and rig.capture.stills == []  # base pose and handle grip
    assert_held_not_homed(rig)


def event_messages(state: dict[str, Any], level: str | None = None) -> list[str]:
    """Event messages of a state, oldest first, optionally of one level."""
    return [e["message"] for e in reversed(state["events"]) if level is None or e["level"] == level]


def test_step_failure_is_logged_with_its_message(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    seq = rig.cfg.arm_sequence
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.run_to_capture(jaw_after_close=seq.grip_rad)
        rig.finish()
    failures = [r for r in caplog.records if r.name == SEQUENCER_LOGGER and r.levelno >= logging.WARNING]
    assert any(MESSAGE_UNSEATED in r.getMessage() for r in failures)


def test_state_events_cover_a_full_run_newest_first(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    assert rig.seq.state()["events"] == []
    state = rig.run_to_holding()
    events = state["events"]
    assert all(set(e) == EVENT_KEYS for e in events)
    assert all(e["time"] == NOW.isoformat(timespec="seconds") for e in events)
    messages = event_messages(state)
    assert messages[0].startswith("sequence started")
    assert events[0]["message"].startswith("sequence done")  # newest first
    assert events[0]["level"] == "info"
    for text in (
        "base pose reached",
        "board attached",
        "handle gripped",
        f"still session {SESSION} started",
        "tier 2 allowed",
        "back at the base pose",
    ):
        assert any(text in m for m in messages), text
    pose_events = [m for m in messages if m.startswith("tier ") and " captured" in m]
    assert len(pose_events) == len(tier_poses(rig.cfg))
    assert event_messages(state, "error") == []


def test_failed_sequence_records_the_error_event(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_capture(jaw_after_close=rig.cfg.arm_sequence.grip_rad)
    state = rig.finish()
    newest = state["events"][0]
    assert newest["level"] == "error"
    assert newest["message"].startswith("sequence error: ") and MESSAGE_UNSEATED in newest["message"]
    assert newest["phase"] == "moving_to_base"  # the phase the step failed in


def test_stopped_sequence_records_a_warning_event(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    rig.seq.stop()
    state = rig.finish()
    assert state["events"][0]["level"] == "warning"
    assert state["events"][0]["message"] == "sequence stopped"
    assert event_messages(state, "warning")[-1] == "sequence stopped"


def test_home_outcomes_are_recorded_as_events(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.reset_arm = "failed: timeout"
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.seq.home(confirmed=True)
        state = rig.finish()
    assert state["events"][0]["level"] == "error"
    assert (
        state["events"][0]["message"].startswith("home failed: ") and "failed: timeout" in state["events"][0]["message"]
    )
    assert any(r.levelno >= logging.WARNING and "failed: timeout" in r.getMessage() for r in caplog.records)
    assert any(m.startswith("home started") for m in event_messages(state))
    assert [e["phase"] for e in state["events"] if e["message"].startswith("home started")] == [HOMING]
    rig.robot.reset_arm = "homed"
    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert state["events"][0]["level"] == "info" and state["events"][0]["message"].startswith("home done: homed")


def test_home_refusal_is_recorded_as_an_error_event(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.robot.carrying = ["last grasp held an object"]
    with pytest.raises(SequenceError, match="mcp_server reports carrying"):
        rig.seq.home(confirmed=True)
    newest = rig.seq.state()["events"][0]
    assert newest["level"] == "error" and "mcp_server reports carrying" in newest["message"]


def test_refused_start_is_recorded_as_an_error_event(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.head_urdf = "<robot"
    with pytest.raises(SequenceError, match="URDF"):
        rig.seq.move_to_base(confirmed=True)
    newest = rig.seq.state()["events"][0]
    assert newest["level"] == "error" and "URDF" in newest["message"]


def test_events_are_bounded_to_the_newest(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.robot.carrying = ["last grasp held an object"]
    for _ in range(EVENT_LIMIT + 5):
        with pytest.raises(SequenceError):
            rig.seq.home(confirmed=True)
    rig.robot.carrying = []
    rig.seq.home(confirmed=True)
    state = rig.finish()
    assert len(state["events"]) == EVENT_LIMIT
    assert state["events"][0]["message"].startswith("home done")


JAW_EPS_RAD = 1e-9


def record_confirm_phases(rig: Rig) -> list[str]:
    """Record the phase of every confirm wait of the sequencer (wraps wait_confirm).

    Args:
        rig (Rig): The rig.

    Returns:
        list[str]: The phases waited in, appended as the worker enters them.
    """
    phases: list[str] = []
    original = rig.seq.wait_confirm

    def wrapped(phase: str, message: str, ready: Callable[[], bool]) -> None:
        phases.append(phase)
        original(phase, message, ready)

    rig.seq.wait_confirm = wrapped  # type: ignore[method-assign]
    return phases


def test_resume_with_board_is_refused_unless_holding_with_the_board_on(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    with pytest.raises(ValueError, match="confirmed=True"):
        rig.seq.resume_with_board(confirmed=False)
    with pytest.raises(ValueError, match="board_gripped=True"):  # idle needs the user's statement the board is gripped
        rig.seq.resume_with_board(confirmed=True)
    assert rig.robot.calls == []

    # Running (the board confirm of a normal sequence): refused, the sequence goes on.
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    with pytest.raises(Busy, match="running"):
        rig.seq.resume_with_board(confirmed=True)
    rig.seq.stop()
    assert rig.finish()["phase"] == IDLE
    with pytest.raises(ValueError, match="board_gripped=True"):
        rig.seq.resume_with_board(confirmed=True)

    # Holding without the board on: refused.
    with rig.seq.lock:
        rig.seq.phase = HOLDING
        rig.seq.board_on = False
    calls = len(rig.robot.calls)
    with pytest.raises(Busy, match="holding with the board on"):
        rig.seq.resume_with_board(confirmed=True)
    assert len(rig.robot.calls) == calls

    # Holding with the board on but blocked: refused, still holding.
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.blocked_reason = "an RGB-D capture is running"
    calls = len(rig.robot.calls)
    with pytest.raises(Busy, match="RGB-D capture"):
        rig.seq.resume_with_board(confirmed=True)
    state = rig.seq.state()
    assert (state["phase"], state["board_on"], state["running"]) == (HOLDING, True, False)
    assert len(rig.robot.calls) == calls

    # An unusable URDF: refused before any call, still holding.
    rig.blocked_reason = None
    rig.head_urdf = "<robot"
    with pytest.raises(SequenceError, match="URDF"):
        rig.seq.resume_with_board(confirmed=True)
    state = rig.seq.state()
    assert (state["phase"], state["board_on"], state["running"]) == (HOLDING, True, False)
    assert len(rig.robot.calls) == calls


def test_resume_with_board_keeps_the_jaws_and_skips_the_board_confirm(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    seq_cfg = rig.cfg.arm_sequence
    first = rig.run_to_holding()
    assert (first["phase"], first["board_on"]) == (HOLDING, True)
    jaw = seq_cfg.handle_contact_rad - 0.01  # the taped board: the jaw rests a bit closed past the CAD contact
    rig.robot.positions[GRIPPER] = jaw
    first_run = rig.seq.run_id
    moves_before = rig.robot.count("move_arm_joints")
    sessions_before = len(rig.capture.sessions)
    phases = record_confirm_phases(rig)

    started = rig.seq.resume_with_board(confirmed=True)
    assert started["running"] is True and started["session_id"] is None
    assert rig.seq.run_id != first_run and rig.seq.run_id.startswith("seq-")
    rig.wait_phase(WAITING_BIG_JOINTS)
    rig.seq.allow_big_joints(True)
    state = rig.finish()

    assert WAITING_BOARD not in phases and phases == [WAITING_BIG_JOINTS]
    moves = [m["targets"] for m in rig.robot.named("move_arm_joints")[moves_before:]]
    assert moves[0] == {**seq_cfg.base_pose, GRIPPER: pytest.approx(jaw)}
    gripper_targets = [m[GRIPPER] for m in moves if GRIPPER in m]
    assert gripper_targets == [pytest.approx(jaw)]
    assert all(target <= jaw + JAW_EPS_RAD for target in gripper_targets)
    assert seq_cfg.insert_gripper_rad not in gripper_targets
    assert all(set(m) != {GRIPPER} for m in moves)  # no handle grip
    assert moves[-1] == seq_cfg.base_pose  # back to the base pose, the gripper untouched
    assert len(rig.capture.sessions) == sessions_before + 1
    assert rig.capture.stills
    assert (state["phase"], state["board_on"], state["error"]) == (HOLDING, True, None)
    assert state["session_id"] == SESSION and state["captured"] > 0
    assert "Press Home" in state["status"]
    assert any("resumed with the board on" in message for message in event_messages(state))
    assert_held_not_homed(rig)
    assert rig.robot.count("get_control_state") == 0


def test_resume_with_board_path_check_refusal_moves_nothing(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    moves_before = rig.robot.count("move_arm_joints")
    rig.robot.measured = {**HOME_POSE, "shoulder_lift": 3.0, GRIPPER: 0.05}
    rig.seq.resume_with_board(confirmed=True)
    state = rig.finish()
    assert rig.robot.count("move_arm_joints") == moves_before
    assert (state["phase"], state["board_on"]) == (HOLDING, True)
    assert "base pose (board on)" in state["error"] and "joint_limits" in state["error"]
    assert_held_not_homed(rig)

    # No measured gripper angle: the jaws cannot be kept, nothing moves.
    rig.robot.measured = {**HOME_POSE}
    rig.seq.resume_with_board(confirmed=True)
    state = rig.finish()
    assert rig.robot.count("move_arm_joints") == moves_before
    assert (state["phase"], state["board_on"]) == (HOLDING, True)
    assert "no gripper angle" in state["error"]


def test_resume_with_board_stopped_keeps_holding_with_the_board_on(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    stills = len(rig.capture.stills)
    press_stop_on(rig, stills + 1)
    rig.seq.resume_with_board(confirmed=True)
    state = rig.finish()
    assert (state["phase"], state["board_on"]) == (HOLDING, True)
    assert "stopped" in state["status"]
    assert_held_not_homed(rig)


def test_resume_from_idle_is_refused_without_board_gripped_or_while_running_or_blocked(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    with pytest.raises(ValueError, match="confirmed=True"):
        rig.seq.resume_with_board(confirmed=False, board_gripped=True)
    with pytest.raises(ValueError, match="board_gripped=True"):
        rig.seq.resume_with_board(confirmed=True)
    assert rig.robot.calls == []

    # Running (the board confirm of a normal sequence): refused, the sequence goes on.
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    with pytest.raises(Busy, match="running"):
        rig.seq.resume_with_board(confirmed=True, board_gripped=True)
    assert rig.seq.state()["phase"] == WAITING_BOARD
    rig.seq.stop()
    assert rig.finish()["phase"] == IDLE

    # Blocked: refused before any call, still idle without the board.
    rig = Rig(tmp_path, small_cfg(), blocked="an RGB-D capture is running")
    with pytest.raises(Busy, match="RGB-D capture"):
        rig.seq.resume_with_board(confirmed=True, board_gripped=True)
    state = rig.seq.state()
    assert (state["phase"], state["board_on"], state["running"]) == (IDLE, False, False)
    assert rig.robot.calls == []


@pytest.mark.parametrize("jaw", ["insert", "just open", None])
def test_resume_from_idle_refuses_open_jaws_and_moves_nothing(tmp_path: Path, jaw: str | None) -> None:
    board_file = tmp_path / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    seq_cfg = rig.cfg.arm_sequence
    closed_max = seq_cfg.handle_contact_rad + seq_cfg.grip_converge_tolerance_rad
    if jaw is None:
        rig.robot.positions.pop(GRIPPER)
    else:
        rig.robot.positions[GRIPPER] = seq_cfg.insert_gripper_rad if jaw == "insert" else closed_max + 0.001
    with pytest.raises(SequenceError, match="gripper is open" if jaw else "no gripper angle"):
        rig.seq.resume_with_board(confirmed=True, board_gripped=True)
    state = rig.seq.state()
    assert (state["phase"], state["board_on"], state["running"]) == (IDLE, False, False)
    assert [name for name, _ in rig.robot.calls] == ["get_arm_state"]
    assert not board_file.exists()
    assert (
        state["events"][0]["level"] == "error" and "resume with the board on refused" in state["events"][0]["message"]
    )


def test_resume_from_idle_after_a_restart_keeps_the_jaws_and_skips_the_board_confirm(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    board_file = tmp_path / "state" / "board.json"
    rig = Rig(tmp_path, small_cfg(), board_file=board_file)
    rig.run_to_holding()
    record = json.loads(board_file.read_text())
    fresh = after_restart(rig, tmp_path, board_file)
    seq_cfg = fresh.cfg.arm_sequence
    jaw = seq_cfg.handle_contact_rad + seq_cfg.grip_converge_tolerance_rad  # the closed limit itself is accepted
    fresh.robot.positions[GRIPPER] = jaw
    assert fresh.seq.state()["phase"] == IDLE
    phases = record_confirm_phases(fresh)

    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        started = fresh.seq.resume_with_board(confirmed=True, board_gripped=True)
        assert started["running"] is True and started["board_on"] is True
        fresh.wait_phase(WAITING_BIG_JOINTS)
        fresh.seq.allow_big_joints(False)
        state = fresh.finish()

    assert WAITING_BOARD not in phases and phases == [WAITING_BIG_JOINTS]
    moves = [m["targets"] for m in fresh.robot.named("move_arm_joints")]
    assert moves[0] == {**seq_cfg.base_pose, GRIPPER: pytest.approx(jaw)}
    gripper_targets = [m[GRIPPER] for m in moves if GRIPPER in m]
    assert gripper_targets == [pytest.approx(jaw)]
    assert seq_cfg.insert_gripper_rad not in gripper_targets
    assert all(set(m) != {GRIPPER} for m in moves)  # no handle grip
    assert len(fresh.capture.sessions) == 1 and fresh.capture.stills
    assert (state["phase"], state["board_on"], state["error"]) == (HOLDING, True, None)
    assert "Press Home" in state["status"]
    assert fresh.seq.recorded_mount == record["mount"]  # the home path check falls back to the restored mount
    assert json.loads(board_file.read_text())["session_id"] == SESSION
    assert any("board confirmed gripped by the user (from idle" in m for m in event_messages(state))
    confirmed = [r.getMessage() for r in caplog.records if "board_confirmed" in r.getMessage()]
    assert confirmed and "from_phase" in confirmed[0] and "idle" in confirmed[0]
    assert_held_not_homed(fresh)


def test_resume_from_idle_path_check_refusal_moves_nothing(tmp_path: Path) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.robot.positions = {**HOME_POSE, "shoulder_lift": 3.0, GRIPPER: 0.05}
    rig.seq.resume_with_board(confirmed=True, board_gripped=True)
    state = rig.finish()
    assert rig.robot.count("move_arm_joints") == 0
    assert (state["phase"], state["board_on"]) == (HOLDING, True)  # the user stated the board is gripped
    assert "base pose (board on)" in state["error"] and "joint_limits" in state["error"]
    assert_held_not_homed(rig)


# grip record: the confirmed grip becomes the active handle contact


def grip_path(tmp_path: Path) -> Path:
    """The grip record file of a rig (in the state dir, as __main__ wires it)."""
    return tmp_path / "state" / GRIP_RECORD_FILE


def grip_rig(tmp_path: Path, cfg: NodeConfig | None = None) -> Rig:
    """A rig that keeps the grip record in `grip_path(tmp_path)`."""
    return Rig(tmp_path, cfg or small_cfg(), grip_file=grip_path(tmp_path))


def recorded_rig(tmp_path: Path, contact: float) -> Rig:
    """A rig with `contact` recorded from idle, then the robot back at its home pose with the jaws open.

    Args:
        tmp_path (Path): Test dir.
        contact (float): The jaw angle the confirm measures (rad).

    Returns:
        Rig: The rig, idle, with the grip recorded.
    """
    tmp_path.mkdir(parents=True, exist_ok=True)
    rig = grip_rig(tmp_path)
    rig.robot.positions[GRIPPER] = contact
    rig.seq.confirm_grip(True)
    rig.robot.positions = {**HOME_POSE, GRIPPER: 0.05}
    return rig


def write_grip(path: Path, contact: float, recorded_at: str = "2026-10-22T12:00:00+00:00") -> None:
    """Write a grip record file as the node does."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"handle_contact_rad": contact, "recorded_at": recorded_at}))


def test_confirm_grip_from_idle_records_measured_jaw(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    seq_cfg = rig.cfg.arm_sequence
    contact = seq_cfg.handle_contact_rad - 0.005
    rig.robot.positions[GRIPPER] = contact
    state = rig.seq.confirm_grip(True)
    assert state["grip"] == {
        "handle_contact_rad": contact,
        "grip_rad": pytest.approx(contact - seq_cfg.handle_squeeze_rad),
        "source": SOURCE_RECORDED,
        "recorded_at": NOW.isoformat(timespec="seconds"),
        "config_handle_contact_rad": seq_cfg.handle_contact_rad,
    }
    record = read_grip_record(grip_path(tmp_path))
    assert record.handle_contact_rad == contact and record.recorded_at == NOW
    assert json.loads(grip_path(tmp_path).read_text()) == {
        "handle_contact_rad": contact,
        "recorded_at": NOW.isoformat(timespec="seconds"),
    }
    names = [name for name, _ in rig.robot.calls]
    assert names == ["get_arm_state"]
    assert not {"move_arm_joints", "set_head_angles", "take_control"} & set(names)
    assert state["phase"] == IDLE and state["running"] is False
    assert rig.seq.state()["running"] is False


def test_confirm_grip_from_holding_after_unseated_refusal(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    seq_cfg = rig.cfg.arm_sequence
    rig.run_to_capture(jaw_after_close=seq_cfg.grip_rad)
    state = rig.finish()
    assert state["phase"] == HOLDING and state["error"].startswith(MESSAGE_UNSEATED)
    state = rig.seq.confirm_grip(True)
    assert state["grip"]["handle_contact_rad"] == pytest.approx(seq_cfg.grip_rad)
    assert state["grip"]["source"] == SOURCE_RECORDED
    assert state["phase"] == HOLDING and state["board_on"] is True and state["running"] is False


def test_confirm_grip_needs_confirm(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    with pytest.raises(ValueError, match="confirmed=True") as refused:
        rig.seq.confirm_grip(False)
    assert str(refused.value) == MESSAGE_GRIP_CONFIRM
    assert rig.robot.calls == []
    assert not grip_path(tmp_path).exists()
    assert rig.seq.grip_state()["source"] == SOURCE_CONFIG


def test_confirm_grip_refused_while_running(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    calls = len(rig.robot.calls)
    with pytest.raises(Busy):
        rig.seq.confirm_grip(True)
    assert len(rig.robot.calls) == calls
    assert not grip_path(tmp_path).exists()
    rig.seq.stop()
    rig.finish()


def test_confirm_grip_refused_while_thread_alive(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    release = threading.Event()
    thread = threading.Thread(target=release.wait, daemon=True)
    thread.start()
    rig.seq.thread = thread
    try:
        assert rig.seq.state()["running"] is False
        with pytest.raises(Busy):
            rig.seq.confirm_grip(True)
        with pytest.raises(Busy):
            rig.seq.reset_grip(True)
    finally:
        release.set()
        thread.join()
    assert rig.robot.calls == []


@pytest.mark.parametrize("phase", [WAITING_BOARD, CAPTURING, WAITING_BIG_JOINTS])
def test_confirm_grip_refused_in_other_phases(tmp_path: Path, phase: str) -> None:
    rig = grip_rig(tmp_path)
    rig.seq.phase = phase
    with pytest.raises(Busy):
        rig.seq.confirm_grip(True)
    assert rig.robot.calls == []
    assert rig.seq.state()["phase"] == phase and rig.seq.state()["running"] is False


def test_confirm_grip_refuses_missing_gripper(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    rig.robot.positions.pop(GRIPPER)
    with pytest.raises(SequenceError, match="no gripper angle"):
        rig.seq.confirm_grip(True)
    assert not grip_path(tmp_path).exists()
    state = rig.seq.state()
    assert state["grip"]["source"] == SOURCE_CONFIG and state["running"] is False
    assert state["events"][0]["level"] == "warning"


def test_confirm_grip_refuses_a_failed_measure(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    rig.robot.errors["get_arm_state"] = 1
    with pytest.raises(SequenceError, match="injected tool error"):
        rig.seq.confirm_grip(True)
    assert not grip_path(tmp_path).exists()
    assert rig.seq.state()["running"] is False


@pytest.mark.parametrize("jaw", [GRIP_CONTACT_MIN_RAD - 0.0001, GRIP_CONTACT_MAX_RAD + 0.0001])
def test_confirm_grip_refuses_out_of_range(tmp_path: Path, jaw: float) -> None:
    assert (GRIP_CONTACT_MIN_RAD, GRIP_CONTACT_MAX_RAD) == (0.03, 0.30)
    rig = grip_rig(tmp_path)
    rig.robot.positions[GRIPPER] = jaw
    with pytest.raises(SequenceError, match="outside"):
        rig.seq.confirm_grip(True)
    assert not grip_path(tmp_path).exists()
    assert rig.seq.grip_state()["source"] == SOURCE_CONFIG
    assert rig.seq.state()["running"] is False


@pytest.mark.parametrize("jaw", [GRIP_CONTACT_MIN_RAD, GRIP_CONTACT_MAX_RAD])
def test_confirm_grip_accepts_the_range_boundaries(tmp_path: Path, jaw: float) -> None:
    rig = grip_rig(tmp_path)
    rig.robot.positions[GRIPPER] = jaw
    state = rig.seq.confirm_grip(True)
    assert state["grip"]["handle_contact_rad"] == jaw and state["grip"]["source"] == SOURCE_RECORDED
    assert read_grip_record(grip_path(tmp_path)).handle_contact_rad == jaw


def test_confirm_grip_holds_a_claim_while_measuring(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    seen: list[Exception] = []
    running: list[bool] = []

    def start_a_move(name: str) -> None:
        if name != "get_arm_state":
            return
        running.append(rig.seq.state()["running"])
        try:
            rig.seq.move_to_base(True)
        except Busy as exc:
            seen.append(exc)

    rig.robot.on_call = start_a_move
    state = rig.seq.confirm_grip(True)
    assert running == [True]
    assert len(seen) == 1
    assert state["running"] is False and state["phase"] == IDLE
    assert [name for name, _ in rig.robot.calls] == ["get_arm_state"]


def test_confirm_grip_write_failure_keeps_config(tmp_path: Path) -> None:
    (tmp_path / "state").write_text("not a directory")
    rig = grip_rig(tmp_path)
    contact = rig.cfg.arm_sequence.handle_contact_rad - 0.005
    rig.robot.positions[GRIPPER] = contact
    with pytest.raises(SequenceError, match="cannot write"):
        rig.seq.confirm_grip(True)
    grip = rig.seq.grip_state()
    assert grip["source"] == SOURCE_CONFIG
    assert grip["handle_contact_rad"] == rig.cfg.arm_sequence.handle_contact_rad
    assert rig.seq.seq is rig.seq.config_seq
    assert rig.seq.state()["running"] is False


def test_startup_loads_grip_record(tmp_path: Path) -> None:
    seq_cfg = small_cfg().arm_sequence
    contact = seq_cfg.handle_contact_rad + 0.004
    write_grip(grip_path(tmp_path), contact)
    rig = grip_rig(tmp_path)
    grip = rig.seq.grip_state()
    assert grip["source"] == SOURCE_RECORDED
    assert grip["handle_contact_rad"] == contact
    assert grip["recorded_at"] == "2026-10-22T12:00:00+00:00"
    assert grip["config_handle_contact_rad"] == seq_cfg.handle_contact_rad
    assert rig.seq.seq.grip_rad == pytest.approx(contact - seq_cfg.handle_squeeze_rad)
    assert rig.seq.config_seq.handle_contact_rad == seq_cfg.handle_contact_rad


INVALID_SQUEEZE_RAD = 0.05  # a valid config with this squeeze refuses a contact at or below it
INVALID_TOLERANCE_RAD = 0.06


@pytest.mark.parametrize(
    "content",
    [
        "not json",
        json.dumps({"recorded_at": "2026-10-22T12:00:00+00:00"}),
        json.dumps({"handle_contact_rad": 0.31, "recorded_at": "2026-10-22T12:00:00+00:00"}),
        json.dumps({"handle_contact_rad": 0.08, "recorded_at": "2026-10-22T12:00:00"}),
        json.dumps({"handle_contact_rad": INVALID_SQUEEZE_RAD, "recorded_at": "2026-10-22T12:00:00+00:00"}),
    ],
    ids=["not-json", "missing-key", "out-of-range", "naive-datetime", "refused-by-config"],
)
def test_startup_ignores_invalid_grip_record(tmp_path: Path, caplog: pytest.LogCaptureFixture, content: str) -> None:
    cfg = small_cfg(handle_squeeze_rad=INVALID_SQUEEZE_RAD, grip_converge_tolerance_rad=INVALID_TOLERANCE_RAD)
    assert cfg.arm_sequence.handle_contact_rad > INVALID_SQUEEZE_RAD
    path = grip_path(tmp_path)
    path.parent.mkdir(parents=True)
    path.write_text(content)
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig = grip_rig(tmp_path, cfg)
    grip = rig.seq.grip_state()
    assert grip["source"] == SOURCE_CONFIG and grip["recorded_at"] is None
    assert grip["handle_contact_rad"] == cfg.arm_sequence.handle_contact_rad
    assert any(r.levelno == logging.WARNING and "grip_record" in r.getMessage() for r in caplog.records)


def test_reset_grip_restores_config_and_deletes_file(tmp_path: Path) -> None:
    rig = recorded_rig(tmp_path, small_cfg().arm_sequence.handle_contact_rad - 0.01)
    assert grip_path(tmp_path).exists()
    calls = len(rig.robot.calls)
    state = rig.seq.reset_grip(True)
    assert not grip_path(tmp_path).exists()
    assert state["grip"]["source"] == SOURCE_CONFIG and state["grip"]["recorded_at"] is None
    assert state["grip"]["handle_contact_rad"] == rig.cfg.arm_sequence.handle_contact_rad
    assert rig.seq.seq is rig.seq.config_seq
    assert len(rig.robot.calls) == calls


def test_reset_grip_without_record_is_ok(tmp_path: Path) -> None:
    rig = grip_rig(tmp_path)
    state = rig.seq.reset_grip(True)
    assert state["grip"]["source"] == SOURCE_CONFIG
    corrupt = grip_path(tmp_path)
    corrupt.parent.mkdir(parents=True)
    corrupt.write_text("not json")
    state = rig.seq.reset_grip(True)
    assert state["grip"]["source"] == SOURCE_CONFIG and not corrupt.exists()
    assert rig.robot.calls == []


def test_reset_grip_refused_while_running(tmp_path: Path) -> None:
    rig = recorded_rig(tmp_path, small_cfg().arm_sequence.handle_contact_rad - 0.01)
    rig.seq.move_to_base(confirmed=True)
    rig.wait_phase(WAITING_BOARD)
    with pytest.raises(Busy):
        rig.seq.reset_grip(True)
    assert grip_path(tmp_path).exists()
    assert rig.seq.grip_state()["source"] == SOURCE_RECORDED
    rig.seq.stop()
    rig.finish()


def test_reset_grip_needs_confirm(tmp_path: Path) -> None:
    rig = recorded_rig(tmp_path, small_cfg().arm_sequence.handle_contact_rad - 0.01)
    with pytest.raises(ValueError, match="confirmed=True") as refused:
        rig.seq.reset_grip(False)
    assert str(refused.value) == MESSAGE_GRIP_RESET_CONFIRM
    assert grip_path(tmp_path).exists() and rig.seq.grip_state()["source"] == SOURCE_RECORDED


def test_recorded_contact_drives_handle_grip(tmp_path: Path) -> None:
    seq_cfg = small_cfg().arm_sequence
    contact = seq_cfg.handle_contact_rad - 0.01
    squeeze = seq_cfg.handle_squeeze_rad
    tolerance = seq_cfg.grip_converge_tolerance_rad

    rig = recorded_rig(tmp_path / "seated", contact)
    rig.run_to_capture(jaw_after_close=contact)
    rig.wait_phase(WAITING_BIG_JOINTS)
    moves = [m["targets"] for m in rig.robot.named("move_arm_joints")]
    assert moves[1] == {GRIPPER: pytest.approx(contact - squeeze)}
    rig.seq.stop()
    rig.finish()

    limit = contact - squeeze / 2
    rig = recorded_rig(tmp_path / "unseated", contact)
    rig.run_to_capture(jaw_after_close=limit - 1e-3)
    error = rig.finish()["error"]
    assert MESSAGE_UNSEATED in error and f"limit {limit:.4f} rad" in error

    rig = recorded_rig(tmp_path / "slack", contact)
    rig.run_to_capture(jaw_after_close=contact - squeeze + tolerance)
    assert MESSAGE_SLACK_GRIP in rig.finish()["error"]


def test_recorded_contact_drives_resume_from_idle_limit(tmp_path: Path) -> None:
    seq_cfg = small_cfg().arm_sequence
    contact = seq_cfg.handle_contact_rad - 0.01
    tolerance = seq_cfg.grip_converge_tolerance_rad

    rig = recorded_rig(tmp_path / "open", contact)
    rig.robot.positions[GRIPPER] = contact + tolerance + 1e-3
    with pytest.raises(SequenceError, match="gripper is open"):
        rig.seq.resume_with_board(confirmed=True, board_gripped=True)
    assert rig.robot.count("move_arm_joints") == 0

    rig = recorded_rig(tmp_path / "closed", contact)
    rig.robot.positions[GRIPPER] = contact + tolerance - 1e-3
    started = rig.seq.resume_with_board(confirmed=True, board_gripped=True)
    assert started["running"] is True
    rig.seq.stop()
    rig.finish()
