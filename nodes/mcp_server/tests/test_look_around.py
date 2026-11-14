"""Tests for mcp_server.look_around: step planning, clearance refusal, the rotate-capture loop and the montage."""

import math
from collections.abc import Callable
from pathlib import Path
from typing import Any

import anyio
import cv2
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import ValidationError

from mcp_server import camera_scene, look_around, perception_tools
from mcp_server.base_motion import SpinOutcome
from mcp_server.config import FootprintSettings, LookAroundSettings, McpServerConfig
from mcp_server.head import AppliedHeadKinematics, HeadController
from mcp_server.look_around import (
    build_montage,
    check_clearance,
    head_pan_label,
    heading_label,
    plan_look_around,
    required_clearance_m,
    run_head_mode,
    run_look_around,
)
from mcp_server.models import (
    BaseMotionBusyError,
    BasePose,
    CameraFrame,
    MapSummary,
    NavigationResult,
    RobotError,
    SectorObstacle,
)
from mcp_server.tools import build_mcp_server

from .fakes import FakeHeadBackend
from .test_perception_tools import RotatingRobot
from .test_tools import TOKEN, call, take

FOOTPRINT = FootprintSettings()
SETTINGS = LookAroundSettings()
HEAD_SETTINGS = McpServerConfig().head
LEVEL = {"head_pan": 0.0, "head_tilt": 0.0}


def sectors(front: float | None = 2.0, **others: float | None) -> list[SectorObstacle]:
    names = ("front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right")
    values = {"front": front} | others
    return [SectorObstacle(sector=n, nearest_m=values.get(n), bearing_rad=None) for n in names]


def jpeg(color: int = 128, size: tuple[int, int] = (64, 48)) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((size[1], size[0], 3), color, np.uint8))
    assert ok
    return buf.tobytes()


class ScriptRobot:
    """Minimal RobotApi for run_look_around: records calls, scripted rotation results."""

    def __init__(
        self,
        statuses: list[str] | None = None,
        obstacles: list[list[SectorObstacle] | None] | None = None,
        head_positions: dict[str, float] | None = None,
        head_speed: float | None = None,
    ):
        self.head_backend = FakeHeadBackend(head_positions, head_speed)
        self.head = HeadController(
            self.head_backend, AppliedHeadKinematics(HEAD_SETTINGS.urdf_path, None), HEAD_SETTINGS
        )
        self.head_commands_at_capture: list[int] = []  # head commands sent before each camera_image call
        self.rehome_head_after_moves = False  # head_aim homes the head when a Nav2 goal ends
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=0.3)
        self.statuses = list(statuses or [])
        self.obstacles = list(obstacles or [])
        self.calls: list[tuple] = []
        self.stops = 0
        self.event_no = 0  # event sequence number; raise_event bumps it
        self.raised_event: str | None = None
        self.precise_flags: list[bool] = []  # the precise argument of every move_relative call
        self.stop_after_moves: int | None = None
        self.camera_fail_at: set[int] = set()
        self.frames = 0
        self.event_after_cameras: tuple[int, str] | None = None  # (camera calls so far, critical event type)
        self.move_errors: dict[int, RobotError] = {}  # 1-based move_relative call number -> raised error
        self.yaw_error_rad = 0.0  # added to every successful rotation (a controller that stops short or long)

    def raise_event(self, event_type: str) -> None:
        self.raised_event = event_type
        self.event_no += 1

    def robot_pose(self) -> BasePose | None:
        return self.pose

    def stop_count(self) -> int:
        return self.stops

    def event_seq(self) -> int:
        return self.event_no

    def interrupt_since(self, seq: int) -> str | None:
        if self.raised_event is not None and self.event_no > seq:
            return self.raised_event
        if self.event_after_cameras is not None and self.frames >= self.event_after_cameras[0]:
            return self.event_after_cameras[1]
        return None

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        self.calls.append(("scan",))
        obstacles = self.obstacles.pop(0) if self.obstacles else sectors()
        return MapSummary(obstacles=obstacles), None

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        index = self.frames
        self.frames += 1
        self.head_commands_at_capture.append(len(self.head_backend.commands))
        self.calls.append(("camera", camera, max_px))
        if index in self.camera_fail_at:
            raise RobotError("no frame")
        return CameraFrame(
            camera=camera, topic="/x", jpeg=jpeg(40 * (index + 1)), width=64, height=48, stamp_s=1.0, age_s=0.1
        )

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.precise_flags.append(precise)
        self.calls.append(("move", dx, dy, round(dyaw, 6), timeout_s))
        moves = sum(1 for c in self.calls if c[0] == "move")
        if moves in self.move_errors:
            raise self.move_errors[moves]
        status = self.statuses.pop(0) if self.statuses else "succeeded"
        if self.stop_after_moves == moves:
            self.stops += 1
        if status == "succeeded":
            self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=self.pose.yaw + dyaw + self.yaw_error_rad)
        if self.rehome_head_after_moves:
            home = {"head_pan": HEAD_SETTINGS.home_pan_rad, "head_tilt": HEAD_SETTINGS.home_tilt_rad}
            self.head_backend.positions.update(home)
            self.head_backend.target.update(home)
        interrupted_by = "collision_stop" if status == "interrupted" else None
        return NavigationResult(status=status, final_pose=self.pose, interrupted_by=interrupted_by)


def test_plan_four_captures_cover_360_in_equal_steps() -> None:
    plan = plan_look_around(4)
    assert plan.step_rad == pytest.approx(math.pi / 2)
    assert [round(math.degrees(h)) for h in plan.headings_rad] == [0, 90, 180, 270]
    assert plan.total_rad == pytest.approx(2 * math.pi)


@pytest.mark.parametrize("captures", [3, 5, 6, 12])
def test_plan_steps_sum_to_a_full_turn(captures: int) -> None:
    plan = plan_look_around(captures)
    assert plan.step_rad * captures == pytest.approx(2 * math.pi)
    assert len(plan.headings_rad) == captures


@pytest.mark.parametrize("captures", [0, 1, 2, 13])
def test_plan_rejects_out_of_range_captures(captures: int) -> None:
    with pytest.raises(ValueError, match="captures"):
        plan_look_around(captures)


def test_required_clearance_is_circumscribed_radius_plus_margin() -> None:
    assert required_clearance_m(0.47, 0.386, 0.10) == pytest.approx(math.hypot(0.235, 0.193) + 0.10)


def test_check_clearance_refuses_close_obstacles_and_missing_lidar() -> None:
    required = required_clearance_m(0.47, 0.386, 0.10)
    assert check_clearance(sectors(2.0, left=1.0), required) is None
    msg = check_clearance(sectors(2.0, rear_left=0.35), required)
    assert msg is not None and "rear_left" in msg and "0.35" in msg and "rotation" in msg
    assert check_clearance(None, required) is not None
    assert check_clearance(sectors(None), required) is None  # no returns at all = free


def test_heading_label_is_degrees_counter_clockwise() -> None:
    assert heading_label(0.0) == "0 deg (start)"
    assert heading_label(math.pi / 2) == "+90 deg"
    assert heading_label(3 * math.pi / 2) == "+270 deg"


def test_montage_tiles_every_frame_and_marks_missing_ones() -> None:
    tiles = [("0 deg (start)", jpeg(200)), ("+90 deg", None), ("+180 deg", jpeg(50)), ("+270 deg", jpeg(90))]
    montage = build_montage(tiles, tile_width=64)
    assert montage.shape[1] == 2 * 64 and montage.shape[0] > 2 * 48 - 2
    # The missing frame's tile is flat grey with a label, not a copy of another image.
    top_right = montage[:, 64:]
    assert top_right.std() > 0  # label text drawn
    assert not np.array_equal(montage[: montage.shape[0] // 2, 64:], montage[: montage.shape[0] // 2, :64])


def test_run_completes_a_full_turn_returning_to_start_heading() -> None:
    robot = ScriptRobot()
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    moves = [c for c in robot.calls if c[0] == "move"]
    assert [m[3] for m in moves] == [round(math.pi / 2, 6)] * 4  # three between stops + the closing step
    assert all(m[1] == 0 and m[2] == 0 for m in moves)
    assert [c for c in robot.calls if c[0] == "camera"] == [("camera", "head", SETTINGS.frame_max_px)] * 4
    res = run.result
    assert res.status == "completed" and res.interrupted_by is None and res.returned_to_start
    assert len(run.frames) == 4 and all(jpeg_bytes for _, jpeg_bytes in run.frames)
    assert [h.heading_deg for h in res.headings] == [0, 90, 180, 270]
    assert res.headings[0].nearest_m == 2.0 and res.headings[0].nearest_sector == "front"
    assert res.expected["rotation_deg"] == 360 and res.expected["stops"] == 4
    assert res.achieved["rotation_deg"] == pytest.approx(360.0)
    assert res.achieved["heading_error_deg"] == pytest.approx(0.0, abs=1e-6)
    assert [s.status for s in res.steps] == ["succeeded"] * 4


def test_run_refuses_without_moving_when_obstacle_too_close() -> None:
    robot = ScriptRobot(obstacles=[sectors(2.0, left=0.3)])
    with pytest.raises(RobotError, match="rotation"):
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert not any(c[0] in ("move", "camera") for c in robot.calls)


def test_run_refuses_without_lidar() -> None:
    robot = ScriptRobot(obstacles=[None])
    with pytest.raises(RobotError, match="lidar"):
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert not any(c[0] == "move" for c in robot.calls)


def test_run_without_pose_is_an_error() -> None:
    robot = ScriptRobot()
    robot.pose = None  # type: ignore[assignment]
    with pytest.raises(RobotError, match="pose"):
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")


def test_interrupted_step_ends_early_and_reports_the_event() -> None:
    robot = ScriptRobot(statuses=["succeeded", "interrupted"])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    res = run.result
    assert res.status == "interrupted" and res.interrupted_by == "collision_stop"
    assert not res.returned_to_start
    assert len([c for c in robot.calls if c[0] == "move"]) == 2  # nothing after the interruption, no return rotation
    assert len(run.frames) == 2  # stops 0 and 1 were captured before the second rotation
    assert res.achieved["rotation_deg"] == pytest.approx(90.0)


def test_failed_step_stops_the_sequence() -> None:
    robot = ScriptRobot(statuses=["aborted"])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "failed" and len(run.frames) == 1
    assert len([c for c in robot.calls if c[0] == "move"]) == 1


def test_stop_between_steps_aborts_before_the_next_rotation() -> None:
    robot = ScriptRobot()
    robot.stop_after_moves = 1  # stop() lands while the first rotation finishes
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "stopped"
    assert len([c for c in robot.calls if c[0] == "move"]) == 1


def test_obstacle_appearing_mid_scan_aborts_before_rotating() -> None:
    robot = ScriptRobot(obstacles=[sectors(2.0), sectors(2.0), sectors(0.3)])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "aborted_obstacle"
    assert len([c for c in robot.calls if c[0] == "move"]) == 1


def test_camera_failure_is_recorded_not_fabricated() -> None:
    robot = ScriptRobot()
    robot.camera_fail_at = {1}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "completed"
    assert run.frames[1][1] is None
    assert any("no frame" in n for n in run.result.notes)
    assert run.result.headings[1].image_captured is False


# --- a critical event raised between rotations (not during a navigate call) ---------------------------------------


@pytest.mark.parametrize(
    ("cameras_before_event", "expected_moves", "expected_frames"), [(1, 0, 1), (2, 1, 2), (4, 3, 4)]
)
def test_critical_event_during_capture_stops_before_the_next_rotation(
    cameras_before_event: int, expected_moves: int, expected_frames: int
) -> None:
    robot = ScriptRobot()
    robot.event_after_cameras = (cameras_before_event, "bump")
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    res = run.result
    assert res.status == "interrupted" and res.interrupted_by == "bump"
    assert len([c for c in robot.calls if c[0] == "move"]) == expected_moves  # no rotation, no return rotation
    assert len(run.frames) == expected_frames  # frames captured so far are kept
    assert not res.returned_to_start
    assert "bump" in res.message and "NOT return" in res.message


# --- RobotError from a rotation step ---------------------------------------------------------------------------


def test_robot_error_in_a_step_returns_failed_with_the_frames_and_tries_to_return_to_start() -> None:
    robot = ScriptRobot()
    robot.move_errors = {2: RobotError("robot pose (map->base_link) unavailable; cannot plan a relative move")}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")  # must not raise
    res = run.result
    assert res.status == "failed" and "pose" in res.message and "step 2" in res.message
    assert len(run.frames) == 2
    moves = [c for c in robot.calls if c[0] == "move"]
    assert [m[3] for m in moves] == [round(math.pi / 2, 6), round(math.pi / 2, 6), round(-math.pi / 2, 6)]
    assert res.returned_to_start  # the return step succeeded and the heading is back within tolerance
    assert res.achieved["heading_error_deg"] == pytest.approx(0.0, abs=1e-6)


def test_robot_error_because_another_motion_runs_attempts_no_return_rotation() -> None:
    robot = ScriptRobot()
    robot.move_errors = {2: BaseMotionBusyError("another base motion is running; call stop first")}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    res = run.result
    assert res.status == "failed" and "another base motion" in res.message
    assert len([c for c in robot.calls if c[0] == "move"]) == 2  # nothing sent while someone else drives
    assert len(run.frames) == 2 and not res.returned_to_start


def test_failed_return_attempt_is_reported_not_raised() -> None:
    robot = ScriptRobot()
    robot.move_errors = {2: RobotError("pose stale"), 3: RobotError("still stale")}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "failed" and not run.result.returned_to_start
    assert any("return to start failed" in n and "still stale" in n for n in run.result.notes)


def test_robot_error_on_the_first_step_leaves_the_robot_where_it_is() -> None:
    robot = ScriptRobot()
    robot.move_errors = {1: RobotError("pose stale")}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "failed" and len(run.frames) == 1
    assert len([c for c in robot.calls if c[0] == "move"]) == 1  # already at the start heading: no return step


# --- returned_to_start is verified against the measured heading --------------------------------------------------


def test_returned_to_start_requires_the_final_heading_within_the_yaw_tolerance() -> None:
    robot = ScriptRobot()
    robot.yaw_error_rad = math.radians(1.5)  # every step stops 1.5 deg long: 6 deg off after four steps
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    res = run.result
    assert res.status == "completed" and not res.returned_to_start
    assert res.achieved["heading_error_deg"] == pytest.approx(6.0)
    assert any("heading error" in n and "5 deg tolerance" in n for n in res.notes)


def test_returned_to_start_within_tolerance_and_tolerance_is_configurable() -> None:
    robot = ScriptRobot()
    robot.yaw_error_rad = math.radians(0.4)  # 1.6 deg in total
    assert run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head").result.returned_to_start
    robot = ScriptRobot()
    robot.yaw_error_rad = math.radians(0.4)
    strict = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head", yaw_tolerance_deg=1.0)
    assert not strict.result.returned_to_start


def test_returned_to_start_is_false_when_the_final_pose_is_unknown() -> None:
    robot = ScriptRobot()
    poses = [robot.pose]

    def pose_then_none() -> BasePose | None:
        return poses.pop(0) if poses else None

    robot.robot_pose = pose_then_none  # type: ignore[method-assign]
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert run.result.status == "completed" and not run.result.returned_to_start
    assert run.result.achieved["heading_error_deg"] is None


# --- sweeps must use the precise goal tolerance so the heading-return guarantee holds ----------------------------


def test_every_rotation_step_requests_the_precise_goal_tolerance() -> None:
    robot = ScriptRobot()
    run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert robot.precise_flags and all(robot.precise_flags)


def test_corrective_return_rotation_requests_the_precise_goal_tolerance() -> None:
    robot = ScriptRobot()
    robot.move_errors[2] = RobotError("tf lost")  # second step cannot start, so the corrective rotation runs
    robot.yaw_error_rad = math.radians(10.0)
    run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert len(robot.precise_flags) >= 3 and all(robot.precise_flags)


# --- spin mode (continuous slow rotation, frames at angular marks) and return_to_start ---------------------------------


class SpinScriptRobot(ScriptRobot):
    """ScriptRobot with RobotApi.spin: the heading jumps to each mark, then to the full angle."""

    def __init__(self, spin_status: str = "completed", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.spins: list[tuple[float, float, list[float]]] = []
        self.spin_status = spin_status

    def spin(
        self, wz: float, angle_rad: float, marks_rad: list[float], on_mark: Callable[[int], bool], timeout_s: float
    ) -> SpinOutcome:
        self.spins.append((wz, angle_rad, list(marks_rad)))
        start = self.pose.yaw
        for index, mark in enumerate(marks_rad):
            if self.spin_status == "interrupted" and index == 1:
                return SpinOutcome(
                    status="interrupted", commanded_wz=wz, rotated_rad=mark, marks_reached=1, interrupted_by="bump"
                )
            self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=start + mark)
            if not on_mark(index):
                return SpinOutcome(status="aborted", commanded_wz=wz, rotated_rad=mark, marks_reached=index + 1)
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=start + angle_rad)
        return SpinOutcome(status="completed", commanded_wz=wz, rotated_rad=angle_rad, marks_reached=len(marks_rad))


def test_spin_mode_captures_the_same_montage_as_steps_mode() -> None:
    steps_robot = ScriptRobot()
    by_steps = run_look_around(steps_robot, SETTINGS, FOOTPRINT, 4, "head", mode="steps", return_to_start=False)
    spin_robot = SpinScriptRobot()
    by_spin = run_look_around(spin_robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin", return_to_start=False)
    assert [label for label, _ in by_spin.frames] == [label for label, _ in by_steps.frames]
    assert all(data is not None for _, data in by_spin.frames)
    assert [(h.index, h.heading_deg, h.label) for h in by_spin.result.headings] == [
        (h.index, h.heading_deg, h.label) for h in by_steps.result.headings
    ]
    assert by_spin.result.status == by_steps.result.status == "completed"
    assert not by_spin.result.returned_to_start and not by_steps.result.returned_to_start
    assert len(spin_robot.spins) == 1 and not any(c[0] == "move" for c in spin_robot.calls)
    wz, angle, marks = spin_robot.spins[0]
    assert angle == pytest.approx(3 * math.pi / 2) and marks == pytest.approx([math.pi / 2, math.pi, 3 * math.pi / 2])
    assert wz == pytest.approx(SETTINGS.spin_speed_rps)
    assert by_spin.result.expected["rotation_deg"] == 270
    assert by_spin.result.achieved["rotation_deg"] == pytest.approx(270.0)


def test_steps_mode_without_return_skips_the_closing_rotation() -> None:
    robot = ScriptRobot()
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="steps", return_to_start=False)
    assert sum(1 for c in robot.calls if c[0] == "move") == 3
    assert run.result.expected["rotation_deg"] == 270 and not run.result.returned_to_start


def test_spin_mode_returns_to_start_only_when_asked() -> None:
    robot = SpinScriptRobot()
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin", return_to_start=True)
    assert robot.spins[0][1] == pytest.approx(2 * math.pi)
    assert run.result.returned_to_start and run.result.status == "completed"
    assert run.result.expected["rotation_deg"] == 360


def test_spin_mode_corrects_a_final_heading_error_when_returning() -> None:
    robot = SpinScriptRobot()
    original = robot.spin

    def coast(*args: Any) -> SpinOutcome:
        out = original(*args)
        robot.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=robot.pose.yaw + 0.2)  # overshoot by ~11 deg
        return out

    robot.spin = coast  # type: ignore[method-assign]
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin", return_to_start=True)
    moves = [c for c in robot.calls if c[0] == "move"]
    assert len(moves) == 1 and moves[0][3] == pytest.approx(-0.2, abs=1e-6)
    assert run.result.returned_to_start


def test_spin_mode_obstacle_at_a_mark_ends_the_spin() -> None:
    robot = SpinScriptRobot(obstacles=[sectors(), sectors(), sectors(front=0.1)])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin")
    assert run.result.status == "aborted_obstacle"
    assert len(run.frames) == 2


def test_spin_mode_interrupted_by_a_critical_event() -> None:
    robot = SpinScriptRobot(spin_status="interrupted")
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin", return_to_start=True)
    assert run.result.status == "interrupted" and run.result.interrupted_by == "bump"
    assert not run.result.returned_to_start
    assert not any(c[0] == "move" for c in robot.calls)  # never a corrective rotation after a critical event


# --- camera: the D435 head camera only -------------------------------------------------------------------------


def test_look_around_uses_the_head_d435(tmp_path: Path) -> None:
    assert look_around.Camera is camera_scene.Camera and perception_tools.Camera is camera_scene.Camera
    robot = RotatingRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)

    async def run() -> Any:
        return await server.list_tools()

    tool = next(t for t in anyio.run(run) if t.name == "look_around")
    prop = tool.input_schema["properties"]["camera"]
    assert prop.get("enum", [prop.get("const")]) == ["head"] and prop["default"] == "head"
    assert "gripper" not in prop["description"] and "D435" in prop["description"]
    take(server, "base")
    with pytest.raises(ToolError):
        call(server, "look_around", {"captures": 4, "camera": "gripper"})
    call(server, "look_around", {"captures": 4, "mode": "steps"})
    assert [c[1][0] for c in robot.calls if c[0] == "camera_image"] == ["head"] * 4


# --- the head is levelled for the base turn and its aim restored afterwards -----------------------------------------

AIMED = {"head_pan": 0.3, "head_tilt": 0.5}


def make_robot(mode: str, **kwargs: Any) -> ScriptRobot:
    return SpinScriptRobot(**kwargs) if mode == "spin" else ScriptRobot(**kwargs)


@pytest.mark.parametrize("mode", ["spin", "steps"])
def test_look_around_levels_the_head_before_the_first_capture(mode: str) -> None:
    robot = make_robot(mode, head_positions=dict(AIMED))
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode=mode)  # type: ignore[arg-type]
    assert run.result.status == "completed"
    assert robot.head_backend.commands[0] == LEVEL
    assert robot.head_commands_at_capture[0] == 1  # the level command went out before the first frame


@pytest.mark.parametrize("mode", ["spin", "steps"])
def test_look_around_restores_the_previous_head_aim(mode: str) -> None:
    robot = make_robot(mode, head_positions=dict(AIMED))
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode=mode)  # type: ignore[arg-type]
    assert robot.head_backend.commands[-1] == pytest.approx(AIMED)
    assert robot.head_backend.positions == pytest.approx(AIMED)
    head = run.result.head
    assert head is not None and head.restored is True
    assert head.before_deg == pytest.approx((math.degrees(0.3), math.degrees(0.5)), abs=0.01)


def test_look_around_without_head_sample_restores_home() -> None:
    robot = SpinScriptRobot()
    robot.head_backend.no_samples = True
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin")
    res = run.result
    assert res.status == "completed" and len(run.frames) == 4
    assert robot.head_backend.commands[0] == LEVEL
    assert robot.head_backend.commands[-1] == pytest.approx(
        {"head_pan": HEAD_SETTINGS.home_pan_rad, "head_tilt": HEAD_SETTINGS.home_tilt_rad}
    )
    assert any("home" in n for n in res.notes)
    assert res.head is not None and res.head.before_deg is None and res.head.restored is False


def test_look_around_head_timeout_is_a_note() -> None:
    robot = SpinScriptRobot(head_positions=dict(AIMED), head_speed=0.0)  # a stuck head follower
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="spin")
    res = run.result
    assert res.status == "completed"
    assert len(run.frames) == 4 and all(data is not None for _, data in run.frames)
    assert any("head" in n and "timeout" in n for n in res.notes)


def test_look_around_steps_mode_relevels_before_each_capture() -> None:
    robot = ScriptRobot(head_positions=dict(AIMED))
    robot.rehome_head_after_moves = True  # head_aim homes the head after every Nav2 rotation
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="steps", return_to_start=False)
    assert run.result.status == "completed"
    assert robot.head_commands_at_capture == [1, 2, 3, 4]  # one level command before each of the 4 captures
    assert robot.head_backend.commands[:4] == [LEVEL] * 4
    assert robot.head_backend.commands[-1] == pytest.approx(AIMED)


def test_look_around_clearance_refusal_leaves_the_head_alone() -> None:
    robot = ScriptRobot(obstacles=[sectors(2.0, left=0.3)], head_positions=dict(AIMED))
    with pytest.raises(RobotError, match="rotation"):
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="head")
    assert robot.head_backend.commands == []


@pytest.mark.parametrize("mode", ["spin", "steps"])
def test_look_around_level_head_false_never_moves_the_head(mode: str) -> None:
    robot = make_robot(mode, head_positions=dict(AIMED))
    settings = LookAroundSettings(level_head=False)
    run = run_look_around(robot, settings, FOOTPRINT, 4, "head", mode=mode)  # type: ignore[arg-type]
    assert run.result.status == "completed"
    assert robot.head_backend.commands == []
    assert run.result.head is None


def test_look_around_stop_leaves_the_head_where_it_is() -> None:
    robot = ScriptRobot(head_positions=dict(AIMED))
    robot.stop_after_moves = 1
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode="steps")
    assert run.result.status == "stopped"
    assert robot.head_backend.commands == [LEVEL]  # levelled, never restored after the stop
    assert run.result.head is not None and run.result.head.restored is False


@pytest.mark.parametrize("mode", ["spin", "steps"])
@pytest.mark.parametrize("kind", ["stop", "event"])
def test_look_around_keeps_a_stop_or_event_raised_while_the_head_levels(
    mode: str, kind: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = make_robot(mode, head_positions=dict(AIMED))
    publish = robot.head_backend.publish_head_command

    def publish_then_signal(positions: dict[str, float]) -> None:
        publish(positions)
        if kind == "stop":
            robot.stops += 1
        else:
            robot.raise_event("collision_stop")

    monkeypatch.setattr(robot.head_backend, "publish_head_command", publish_then_signal)
    run = run_look_around(robot, SETTINGS, FOOTPRINT, 4, "head", mode=mode)  # type: ignore[arg-type]
    assert not any(c[0] == "move" for c in robot.calls)
    assert not getattr(robot, "spins", [])
    assert run.result.status == ("stopped" if kind == "stop" else "interrupted")
    if kind == "event":
        assert robot.head_backend.commands == [LEVEL, pytest.approx(AIMED)]  # only a stop skips the restore
    else:
        assert robot.head_backend.commands == [LEVEL]  # the head stays where it is after a stop
        assert run.result.head is not None and run.result.head.restored is False


# --- head mode: pan the D435 head, the base never moves -----------------------------------------------------------

DEFAULT_PANS = [90.0, 45.0, 0.0, -45.0, -90.0]


class HeadScriptRobot(ScriptRobot):
    """ScriptRobot whose frames carry the head clock as their stamp and feed the head joint-state history."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.moving_frames: set[int] = set()  # camera calls whose frame shows the head still moving

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        index = self.frames
        frame = super().camera_image(camera, max_px)
        stamp = self.head_backend.t
        positions = dict(self.head_backend.positions)
        if index in self.moving_frames:  # a sample just before the frame with another pan: the head was moving
            self.head.history.add(stamp - 0.05, {**positions, "head_pan": positions["head_pan"] - 0.1})
        self.head.history.add(stamp, positions)
        return frame.model_copy(update={"stamp_s": stamp})


def commanded_pans_deg(robot: ScriptRobot) -> list[float]:
    return [round(math.degrees(c["head_pan"]), 6) for c in robot.head_backend.commands]


@pytest.mark.parametrize(
    ("pan", "label"),
    [
        (90.0, "pan +90 deg (left)"),
        (45.4, "pan +45 deg (left)"),
        (0.0, "pan 0 deg (ahead)"),
        (-0.3, "pan 0 deg (ahead)"),
        (-45.0, "pan -45 deg (right)"),
        (-89.6, "pan -90 deg (right)"),
    ],
)
def test_head_pan_label(pan: float, label: str) -> None:
    assert head_pan_label(pan) == label


def test_head_mode_pans_without_base_motion() -> None:
    robot = HeadScriptRobot(head_positions=dict(AIMED))
    run = run_head_mode(robot, SETTINGS, SETTINGS.head_pans_deg, SETTINGS.head_tilt_deg, "head")
    res = run.result
    assert res.status == "completed"
    assert not [c for c in robot.calls if c[0] == "move"] and robot.precise_flags == []
    assert not hasattr(robot, "spins")
    assert commanded_pans_deg(robot)[:5] == pytest.approx(DEFAULT_PANS)
    assert all(c["head_tilt"] == pytest.approx(0.0) for c in robot.head_backend.commands[:5])
    assert robot.head_commands_at_capture == [1, 2, 3, 4, 5]  # one pan command before each frame
    assert [label for label, _ in run.frames] == [head_pan_label(p) for p in DEFAULT_PANS]
    assert all(data is not None for _, data in run.frames)
    assert [h.heading_deg for h in res.headings] == [90, 45, 0, -45, -90]
    assert [h.label for h in res.headings] == [head_pan_label(p) for p in DEFAULT_PANS]
    assert res.expected == {"mode": "head", "rotation_deg": 0, "pans_deg": DEFAULT_PANS}
    assert res.achieved["pans_deg"] == pytest.approx(DEFAULT_PANS)
    assert res.returned_to_start is False


def test_head_mode_settles_before_each_frame() -> None:
    robot = HeadScriptRobot()
    settings = LookAroundSettings(head_settle_s=0.5)
    start = robot.head_backend.t
    run_head_mode(robot, settings, [30.0], 0.0, "head")
    assert robot.head_backend.t - start >= 0.5


def test_head_mode_restores_aim_and_reports_head() -> None:
    robot = HeadScriptRobot(head_positions=dict(AIMED))
    run = run_head_mode(robot, SETTINGS, [45.0, -45.0], 10.0, "head")
    assert robot.head_backend.commands[-1] == pytest.approx(AIMED)
    assert robot.head_backend.positions == pytest.approx(AIMED)
    head = run.result.head
    assert head is not None and head.restored is True
    assert head.before_deg == pytest.approx((math.degrees(0.3), math.degrees(0.5)), abs=0.01)
    assert [c["head_tilt"] for c in robot.head_backend.commands[:2]] == pytest.approx([math.radians(10.0)] * 2)


def test_head_mode_without_head_sample_sends_the_head_home() -> None:
    robot = HeadScriptRobot()
    robot.head_backend.no_samples = True
    run = run_head_mode(robot, SETTINGS, [45.0], 0.0, "head")
    assert robot.head_backend.commands[-1] == pytest.approx(
        {"head_pan": HEAD_SETTINGS.home_pan_rad, "head_tilt": HEAD_SETTINGS.home_tilt_rad}
    )
    assert run.result.head is not None and run.result.head.before_deg is None


@pytest.mark.parametrize("kind", ["stop", "event"])
def test_head_mode_stop_stops_panning_and_leaves_head(kind: str, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = HeadScriptRobot(head_positions=dict(AIMED))
    publish = robot.head_backend.publish_head_command

    def publish_then_signal(positions: dict[str, float]) -> None:
        publish(positions)
        if len(robot.head_backend.commands) == 2:  # while the head pans to the second angle
            if kind == "stop":
                robot.stops += 1
            else:
                robot.raise_event("collision_stop")

    monkeypatch.setattr(robot.head_backend, "publish_head_command", publish_then_signal)
    run = run_head_mode(robot, SETTINGS, DEFAULT_PANS, 0.0, "head")
    res = run.result
    assert len(run.frames) == 1  # no capture after the stop / event
    assert robot.frames == 1
    if kind == "stop":
        assert res.status == "stopped"
        assert commanded_pans_deg(robot) == pytest.approx([90.0, 45.0])  # no further pan, no restore
        assert res.head is not None and res.head.restored is False
        assert any("stop" in n for n in res.notes)
    else:
        assert res.status == "interrupted" and res.interrupted_by == "collision_stop"
        assert robot.head_backend.commands[-1] == pytest.approx(AIMED)  # only a stop skips the restore
        assert len(robot.head_backend.commands) == 3


def test_head_mode_retakes_a_frame_taken_while_moving() -> None:
    robot = HeadScriptRobot()
    robot.moving_frames = {0}
    run = run_head_mode(robot, SETTINGS, [45.0, -45.0], 0.0, "head")
    assert robot.frames == 3  # the first pan was retaken once
    assert [label for label, _ in run.frames] == ["pan +45 deg (left)", "pan -45 deg (right)"]
    assert not any("moving" in n for n in run.result.notes)
    still = HeadScriptRobot()
    still.moving_frames = {0, 1}
    run = run_head_mode(still, SETTINGS, [45.0], 0.0, "head")
    assert still.frames == 2  # retaken once only
    assert len(run.frames) == 1 and run.frames[0][1] is not None
    assert any("moving" in n for n in run.result.notes)


def test_head_mode_head_timeout_is_a_note() -> None:
    robot = HeadScriptRobot(head_positions=dict(AIMED), head_speed=0.0)  # a stuck head follower
    run = run_head_mode(robot, SETTINGS, DEFAULT_PANS, 0.0, "head")
    res = run.result
    assert res.status == "completed"
    assert len(run.frames) == 5 and all(data is not None for _, data in run.frames)
    assert any("timeout" in n for n in res.notes)
    stuck = head_pan_label(math.degrees(0.3))  # the tiles say where the head really pointed at each frame
    assert [label for label, _ in run.frames] == [stuck] * 5
    assert [s.status for s in res.steps] == ["timeout"] * 5


def test_head_mode_camera_failure_is_a_missing_tile() -> None:
    robot = HeadScriptRobot()
    robot.camera_fail_at = {1}
    run = run_head_mode(robot, SETTINGS, [45.0, 0.0, -45.0], 0.0, "head")
    assert run.result.status == "completed"
    assert [data is None for _, data in run.frames] == [False, True, False]
    assert run.result.headings[1].image_captured is False
    assert any("no camera frame" in n for n in run.result.notes)


def test_head_mode_unknown_pan_at_the_frame_uses_the_commanded_pan_with_a_note() -> None:
    robot = ScriptRobot()  # frames that no head joint-state sample covers
    run = run_head_mode(robot, SETTINGS, [45.0], 0.0, "head")
    assert run.frames[0][0] == "pan +45 deg (left)"
    assert any("commanded" in n for n in run.result.notes)
    assert run.result.achieved["pans_deg"] == [None]


def test_look_around_settings_head_keys() -> None:
    settings = LookAroundSettings()
    assert settings.head_pans_deg == DEFAULT_PANS
    assert settings.head_tilt_deg == 0.0 and settings.head_settle_s == 0.2
    assert LookAroundSettings(mode="head").mode == "head"
    for bad in (
        {"head_pans_deg": []},
        {"head_pans_deg": [91.0]},
        {"head_pans_deg": [-90.5]},
        {"head_tilt_deg": -31.0},
        {"head_tilt_deg": 46.0},
        {"head_settle_s": -0.1},
    ):
        with pytest.raises(ValidationError):
            LookAroundSettings.model_validate(bad)
