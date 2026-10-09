"""Tests for mcp_server.look_around: step planning, clearance refusal, the rotate-capture loop and the montage."""

import math

import cv2
import numpy as np
import pytest

from mcp_server.config import FootprintSettings, LookAroundSettings
from mcp_server.look_around import (
    build_montage,
    check_clearance,
    heading_label,
    plan_look_around,
    required_clearance_m,
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

FOOTPRINT = FootprintSettings()
SETTINGS = LookAroundSettings()


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

    def __init__(self, statuses: list[str] | None = None, obstacles: list[list[SectorObstacle] | None] | None = None):
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=0.3)
        self.statuses = list(statuses or [])
        self.obstacles = list(obstacles or [])
        self.calls: list[tuple] = []
        self.stops = 0
        self.precise_flags: list[bool] = []  # the precise argument of every move_relative call
        self.stop_after_moves: int | None = None
        self.camera_fail_at: set[int] = set()
        self.frames = 0
        self.event_after_cameras: tuple[int, str] | None = None  # (camera calls so far, critical event type)
        self.move_errors: dict[int, RobotError] = {}  # 1-based move_relative call number -> raised error
        self.yaw_error_rad = 0.0  # added to every successful rotation (a controller that stops short or long)

    def robot_pose(self) -> BasePose | None:
        return self.pose

    def stop_count(self) -> int:
        return self.stops

    def event_seq(self) -> int:
        return 0

    def interrupt_since(self, seq: int) -> str | None:
        assert seq == 0
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
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    moves = [c for c in robot.calls if c[0] == "move"]
    assert [m[3] for m in moves] == [round(math.pi / 2, 6)] * 4  # three between stops + the closing step
    assert all(m[1] == 0 and m[2] == 0 for m in moves)
    assert [c for c in robot.calls if c[0] == "camera"] == [("camera", "front", SETTINGS.frame_max_px)] * 4
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
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert not any(c[0] in ("move", "camera") for c in robot.calls)


def test_run_refuses_without_lidar() -> None:
    robot = ScriptRobot(obstacles=[None])
    with pytest.raises(RobotError, match="lidar"):
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert not any(c[0] == "move" for c in robot.calls)


def test_run_without_pose_is_an_error() -> None:
    robot = ScriptRobot()
    robot.pose = None  # type: ignore[assignment]
    with pytest.raises(RobotError, match="pose"):
        run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")


def test_interrupted_step_ends_early_and_reports_the_event() -> None:
    robot = ScriptRobot(statuses=["succeeded", "interrupted"])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    res = run.result
    assert res.status == "interrupted" and res.interrupted_by == "collision_stop"
    assert not res.returned_to_start
    assert len([c for c in robot.calls if c[0] == "move"]) == 2  # nothing after the interruption, no return rotation
    assert len(run.frames) == 2  # stops 0 and 1 were captured before the second rotation
    assert res.achieved["rotation_deg"] == pytest.approx(90.0)


def test_failed_step_stops_the_sequence() -> None:
    robot = ScriptRobot(statuses=["aborted"])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert run.result.status == "failed" and len(run.frames) == 1
    assert len([c for c in robot.calls if c[0] == "move"]) == 1


def test_stop_between_steps_aborts_before_the_next_rotation() -> None:
    robot = ScriptRobot()
    robot.stop_after_moves = 1  # stop() lands while the first rotation finishes
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert run.result.status == "stopped"
    assert len([c for c in robot.calls if c[0] == "move"]) == 1


def test_obstacle_appearing_mid_scan_aborts_before_rotating() -> None:
    robot = ScriptRobot(obstacles=[sectors(2.0), sectors(2.0), sectors(0.3)])
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert run.result.status == "aborted_obstacle"
    assert len([c for c in robot.calls if c[0] == "move"]) == 1


def test_camera_failure_is_recorded_not_fabricated() -> None:
    robot = ScriptRobot()
    robot.camera_fail_at = {1}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="gripper")
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
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
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
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")  # must not raise
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
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    res = run.result
    assert res.status == "failed" and "another base motion" in res.message
    assert len([c for c in robot.calls if c[0] == "move"]) == 2  # nothing sent while someone else drives
    assert len(run.frames) == 2 and not res.returned_to_start


def test_failed_return_attempt_is_reported_not_raised() -> None:
    robot = ScriptRobot()
    robot.move_errors = {2: RobotError("pose stale"), 3: RobotError("still stale")}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert run.result.status == "failed" and not run.result.returned_to_start
    assert any("return to start failed" in n and "still stale" in n for n in run.result.notes)


def test_robot_error_on_the_first_step_leaves_the_robot_where_it_is() -> None:
    robot = ScriptRobot()
    robot.move_errors = {1: RobotError("pose stale")}
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert run.result.status == "failed" and len(run.frames) == 1
    assert len([c for c in robot.calls if c[0] == "move"]) == 1  # already at the start heading: no return step


# --- returned_to_start is verified against the measured heading --------------------------------------------------


def test_returned_to_start_requires_the_final_heading_within_the_yaw_tolerance() -> None:
    robot = ScriptRobot()
    robot.yaw_error_rad = math.radians(1.5)  # every step stops 1.5 deg long: 6 deg off after four steps
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    res = run.result
    assert res.status == "completed" and not res.returned_to_start
    assert res.achieved["heading_error_deg"] == pytest.approx(6.0)
    assert any("heading error" in n and "2" in n for n in res.notes)


def test_returned_to_start_within_tolerance_and_tolerance_is_configurable() -> None:
    robot = ScriptRobot()
    robot.yaw_error_rad = math.radians(0.4)  # 1.6 deg in total
    assert run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front").result.returned_to_start
    robot = ScriptRobot()
    robot.yaw_error_rad = math.radians(0.4)
    strict = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front", yaw_tolerance_deg=1.0)
    assert not strict.result.returned_to_start


def test_returned_to_start_is_false_when_the_final_pose_is_unknown() -> None:
    robot = ScriptRobot()
    poses = [robot.pose]

    def pose_then_none() -> BasePose | None:
        return poses.pop(0) if poses else None

    robot.robot_pose = pose_then_none  # type: ignore[method-assign]
    run = run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert run.result.status == "completed" and not run.result.returned_to_start
    assert run.result.achieved["heading_error_deg"] is None


# --- sweeps must use the precise goal tolerance so the heading-return guarantee holds ----------------------------


def test_every_rotation_step_requests_the_precise_goal_tolerance() -> None:
    robot = ScriptRobot()
    run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert robot.precise_flags and all(robot.precise_flags)


def test_corrective_return_rotation_requests_the_precise_goal_tolerance() -> None:
    robot = ScriptRobot()
    robot.move_errors[2] = RobotError("tf lost")  # second step cannot start, so the corrective rotation runs
    robot.yaw_error_rad = math.radians(10.0)
    run_look_around(robot, SETTINGS, FOOTPRINT, captures=4, camera="front")
    assert len(robot.precise_flags) >= 3 and all(robot.precise_flags)
