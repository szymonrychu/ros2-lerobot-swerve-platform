"""MCP tools suggest_approach_pose and approach_for_grasp (approach_tools.py) on fakes: gates, dry run, modes, results."""

import logging
import math
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server import approach_tools, camera_scene
from mcp_server.approach import CostGrid, PathInfo
from mcp_server.base_dock import DockGoal, DockResult
from mcp_server.config import McpServerConfig
from mcp_server.models import BaseMotionBusyError, BasePose, CameraFrame, NavigationResult, RobotError
from mcp_server.monitor import RobotMonitor
from mcp_server.reach_map import REGENERATE_COMMAND, CellSet, ReachMapStale
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, QUEUE_EXCLUSIVE_TOOLS, TOOL_SUBSYSTEM, build_mcp_server

from .camera_helpers import HEIGHT, WIDTH, camera_config
from .test_battery_gate import LOW_V, make_guard
from .test_tilt_guard import state
from .test_tools import TOKEN, FakeRobot, call, take, tool_schema_text

SMALL_SQUARE = [(0.04, 0.04), (-0.04, 0.04), (-0.04, -0.04), (0.04, -0.04)]
GRID_RES = 0.05
GRID_SIZE_M = 12.0
# A target the synthetic front camera sees from the robot's pose (1, 2, 0.5): 0.74 m ahead of it.
VISIBLE_TARGET = {"x": 1.0 + 0.74 * math.cos(0.5), "y": 2.0 + 0.74 * math.sin(0.5), "z": 0.02}
FAR_TARGET = {"x": 2.5, "y": 2.0, "z": 0.05}
REACH_OK = {
    "reachable": True,
    "collision_free": True,
    "pitch_range_deg": [0.0, 15.0],
    "pitches_deg": [0.0, 15.0],
}
REACH_NO = {
    "suggestion": "raise the point above z 0.30 m",
    "reachable": False,
    "collision_free": False,
    "pitch_range_deg": None,
    "pitches_deg": [],
}


class StubReachMap:
    """ReachMap double: one reachable cell straight ahead of the pan axis, 25 cm out."""

    key = "stubkey"

    def describe(self) -> dict[str, Any]:
        return {"key": self.key, "created_at": "2026-10-11T00:00:00+00:00", "grid": {"z_step_m": 0.025}}

    def cells(self, z: float, pitch_deg: float | None) -> CellSet:
        return CellSet(
            z_m=z,
            heading_deg=np.array([0.0]),
            r_m=np.array([0.25]),
            pitches=[[0.0, 15.0]],
            clearance=np.array([0.05]),
            margin_m=np.array([0.08]),
        )


class FakePointReach:
    """point_reach double: answers REACH_OK unless the test queued other answers (consumed in call order)."""

    def __init__(self) -> None:
        self.points: list[dict[str, Any]] = []
        self.answers: list[dict[str, Any]] = []

    def __call__(self, checker: Any, point: dict[str, Any], seed: Any, envelope: Any) -> dict[str, Any]:
        self.points.append(dict(point))
        return {"point": point, **(self.answers.pop(0) if self.answers else REACH_OK)}


def free_grid(age_s: float = 0.5) -> CostGrid:
    n = int(GRID_SIZE_M / GRID_RES)
    return CostGrid(
        data=np.zeros((n, n), dtype=np.int8),
        resolution=GRID_RES,
        origin_x=-GRID_SIZE_M / 2.0,
        origin_y=-GRID_SIZE_M / 2.0,
        age_s=age_s,
    )


def textured_jpeg() -> bytes:
    rng = np.random.default_rng(7)
    img = cv2.GaussianBlur(rng.integers(0, 255, (HEIGHT, WIDTH, 3), dtype=np.uint8), (0, 0), 2.0)
    ok, buf = cv2.imencode(".png", img)
    assert ok
    return buf.tobytes()


class Rig:
    """A server on a FakeRobot with the reach map, footprint and point_reach faked and the base lease taken."""

    def __init__(
        self,
        tmp_path: Path,
        monkeypatch: pytest.MonkeyPatch,
        config: McpServerConfig | None = None,
        guard: Any = None,
        monitor: RobotMonitor | None = None,
        costmap: bool = True,
        lease: bool = True,
        textured: bool = False,
    ) -> None:
        self.robot = FakeRobot(tmp_path)
        self.robot.costmap = free_grid() if costmap else None
        self.reach = FakePointReach()
        monkeypatch.setattr(approach_tools, "load_reach_map", lambda config, checker: StubReachMap())
        monkeypatch.setattr(approach_tools, "footprint_points", lambda ctx: SMALL_SQUARE)
        monkeypatch.setattr(approach_tools, "point_reach", self.reach)
        if textured:
            png = textured_jpeg()
            robot = self.robot

            def camera_image(camera: str, max_px: int) -> CameraFrame:
                robot.calls.append(("camera_image", (camera, max_px)))
                stamp_s = 10.0 + 0.01 * len([c for c in robot.calls if c[0] == "camera_image"])  # every frame is newer
                return CameraFrame(
                    camera=camera, topic="/x", jpeg=png, width=WIDTH, height=HEIGHT, stamp_s=stamp_s, age_s=0.0
                )

            monkeypatch.setattr(self.robot, "camera_image", camera_image)
        self.server = build_mcp_server(self.robot, config or McpServerConfig(), TOKEN, guard, monitor)
        if lease:
            take(self.server, "base")

    def suggest(self, **args: Any) -> dict[str, Any]:
        return call(self.server, "suggest_approach_pose", {**FAR_TARGET, **args}).structured_content

    def approach(self, **args: Any) -> dict[str, Any]:
        return call(self.server, "approach_for_grasp", {**FAR_TARGET, **args}).structured_content

    def moved(self) -> bool:
        """Whether anything that moves or publishes was touched."""
        sent = [c for c in self.robot.calls if c[0] in ("navigate", "move_relative", "drive", "spin")]
        return bool(self.robot.dock_goals or self.robot.detections or sent or self.robot.base_resets)


def calibrated() -> McpServerConfig:
    return camera_config()


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Rig:
    return Rig(tmp_path, monkeypatch)


# --- classification --------------------------------------------------------------------------------------------


def test_tool_sets_and_subsystem() -> None:
    assert approach_tools.TOOL_NAMES == ("suggest_approach_pose", "approach_for_grasp")
    assert "approach_for_grasp" in MOTION_TOOLS and "approach_for_grasp" not in ALWAYS_ALLOWED_TOOLS
    assert "suggest_approach_pose" in ALWAYS_ALLOWED_TOOLS and "suggest_approach_pose" not in MOTION_TOOLS
    assert TOOL_SUBSYSTEM["approach_for_grasp"] == "base"
    assert "suggest_approach_pose" not in TOOL_SUBSYSTEM
    assert "approach_for_grasp" in QUEUE_EXCLUSIVE_TOOLS


# --- suggest_approach_pose -------------------------------------------------------------------------------------


def test_suggest_ranks_candidates_verifies_and_plans_paths(rig: Rig) -> None:
    rig.robot.path_info = PathInfo(status="ok", length_m=0.1)  # shorter than the straight line: planned ones rank first
    res = rig.suggest(max_results=3)
    assert res["target"]["source"] == "xyz" and res["target"]["z"] == pytest.approx(FAR_TARGET["z"])
    cands = res["candidates"]
    assert 1 <= len(cands) <= 3
    assert cands[0]["verified"]["reachable"] is True
    assert cands[0]["path"]["status"] == "ok"
    assert res["docking"] == {"server_ready": True, "action": "/grasp/dock_robot"}
    assert res["reach_map"]["key"] == "stubkey"
    assert res["costmap_age_s"] == pytest.approx(0.5)
    assert len(rig.robot.planned_poses) == 3  # path_check_k: planning only
    assert len(rig.reach.points) == 5  # verify_top_k
    assert not rig.moved()


def test_suggest_needs_no_lease_and_reports_server_down(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, lease=False)
    rig.robot.dock_server_up = False
    res = rig.suggest()
    assert res["docking"]["server_ready"] is False and res["candidates"]


def test_suggest_check_path_false_plans_nothing(rig: Rig) -> None:
    res = rig.suggest(check_path=False)
    assert rig.robot.planned_poses == []
    assert {c["path"]["status"] for c in res["candidates"]} == {"skipped"}


def test_suggest_refuses_without_a_fresh_costmap(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    none = Rig(tmp_path, monkeypatch, costmap=False, lease=False)
    with pytest.raises(ToolError, match="no global costmap"):
        none.suggest()
    stale = Rig(tmp_path, monkeypatch, lease=False)
    stale.robot.costmap = free_grid(age_s=60.0)
    with pytest.raises(ToolError, match="stale"):
        stale.suggest()


def test_stale_reach_map_logs_at_registration_and_the_tool_refuses_with_the_command(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    def stale(config: Any, checker: Any) -> Any:
        raise ReachMapStale("expected-key", "found-key")

    robot = FakeRobot(tmp_path)
    robot.costmap = free_grid()
    monkeypatch.setattr(approach_tools, "load_reach_map", stale)
    monkeypatch.setattr(approach_tools, "footprint_points", lambda ctx: SMALL_SQUARE)
    with caplog.at_level(logging.ERROR):
        server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    assert any(r.levelno == logging.ERROR and "reach map" in r.getMessage() for r in caplog.records)
    take(server, "base")
    for name in ("suggest_approach_pose", "approach_for_grasp"):
        with pytest.raises(ToolError) as info:
            call(server, name, FAR_TARGET)
        assert "expected-key" in str(info.value) and REGENERATE_COMMAND in str(info.value)
    assert robot.dock_goals == [] and robot.planned_poses == []


def test_missing_reach_map_file_refuses_with_the_command(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = FakeRobot(tmp_path)
    robot.costmap = free_grid()
    config = McpServerConfig(reach_map={"file": tmp_path / "absent.npz"})
    monkeypatch.setattr(approach_tools, "footprint_points", lambda ctx: SMALL_SQUARE)
    server = build_mcp_server(robot, config, TOKEN)
    with pytest.raises(ToolError, match="no reach map") as info:
        call(server, "suggest_approach_pose", FAR_TARGET)
    assert REGENERATE_COMMAND in str(info.value)


def test_target_forms_and_validation(rig: Rig) -> None:
    rig.robot.pois.append({"id": "o1", "kind": "object", "name": "cup", "x": 2.5, "y": 2.0, "status": "open"})
    rig.robot.pois.append({"id": "p1", "kind": "area", "name": "table", "x": 2.4, "y": 2.1, "status": "open"})
    no_xyz = {"max_results": 1}
    by_object = call(rig.server, "suggest_approach_pose", {"object_id": "o1", "z_m": 0.05, **no_xyz})
    assert by_object.structured_content["target"] | {"z": 0} == {
        "x": 2.5,
        "y": 2.0,
        "z": 0,
        "frame": "map",
        "source": "object:o1",
    }
    by_poi = call(rig.server, "suggest_approach_pose", {"poi_id": "p1", "z_m": 0.05, **no_xyz})
    assert by_poi.structured_content["target"]["source"] == "poi:p1"
    with pytest.raises(ToolError, match="z_m is required"):
        call(rig.server, "suggest_approach_pose", {"object_id": "o1"})
    with pytest.raises(ToolError, match="unknown object"):
        call(rig.server, "suggest_approach_pose", {"object_id": "nope", "z_m": 0.05})
    with pytest.raises(ToolError, match="exactly one"):
        call(rig.server, "suggest_approach_pose", {**FAR_TARGET, "object_id": "o1", "z_m": 0.05})
    with pytest.raises(ToolError, match="x, y and z"):
        call(rig.server, "suggest_approach_pose", {"x": 1.0, "y": 2.0})
    with pytest.raises(ToolError):
        call(rig.server, "suggest_approach_pose", {})


def test_pixel_target_goes_through_the_calibrated_camera(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), lease=False)
    res = call(
        rig.server,
        "suggest_approach_pose",
        {"camera": "front", "u": WIDTH / 2, "v": HEIGHT / 2, "surface_height_m": 0.02, "max_results": 1},
    ).structured_content
    assert res["target"]["source"] == "pixel:front" and res["target"]["z"] == pytest.approx(0.02)
    # the synthetic front camera sees the floor 0.74 m ahead at the image centre (robot pose 1, 2, 0.5)
    assert (res["target"]["x"], res["target"]["y"]) == pytest.approx(
        (VISIBLE_TARGET["x"], VISIBLE_TARGET["y"]), abs=0.02
    )
    with pytest.raises(ToolError, match="not calibrated"):
        call(
            Rig(tmp_path, monkeypatch, lease=False).server, "suggest_approach_pose", {"camera": "front", "u": 1, "v": 1}
        )


def test_no_robot_pose_still_ranks_but_plans_no_path(rig: Rig) -> None:
    rig.robot.pose = None  # type: ignore[assignment]
    res = rig.suggest()
    assert res["candidates"] and rig.robot.planned_poses == []
    assert res["candidates"][0]["path"]["status"] == "unavailable"


# --- approach_for_grasp: refusals ------------------------------------------------------------------------------


def test_approach_refused_without_the_base_lease(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, lease=False)
    with pytest.raises(ToolError, match="base"):
        rig.approach()
    take(rig.server, "arm")  # the arm lease is not enough
    with pytest.raises(ToolError, match="base"):
        rig.approach()
    assert not rig.moved() and rig.robot.planned_poses == []


def test_approach_refused_below_battery_cutoff(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, guard=make_guard(LOW_V))
    with pytest.raises(ToolError, match="battery below cut-off"):
        rig.approach()
    with pytest.raises(ToolError, match="battery below cut-off"):
        rig.approach(dry_run=True)
    assert not rig.moved() and rig.robot.planned_poses == []
    assert rig.suggest()["candidates"]  # the read-only planner keeps working


@pytest.mark.parametrize("level", ["warning", "error"])
def test_approach_refused_by_the_tilt_guard(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, level: str) -> None:
    config = McpServerConfig()
    monitor = RobotMonitor(config.monitor, cpu_temp_reader=lambda: 50.0, throttled_reader=lambda: None)
    monitor.on_tilt_guard(state(level, roll=9.0 if level == "warning" else 16.0))
    rig = Rig(tmp_path, monkeypatch, config=config, monitor=monitor)
    with pytest.raises(ToolError, match="approach_for_grasp refused.*tilt guard"):
        rig.approach()
    assert not rig.moved()


def test_another_base_motion_running_is_refused_before_anything_is_sent(rig: Rig) -> None:
    def busy(*args: Any, **kwargs: Any) -> NavigationResult:
        raise BaseMotionBusyError("another base motion is running; call stop first")

    rig.robot.navigate = busy  # type: ignore[method-assign]
    with pytest.raises(ToolError, match="another base motion"):
        rig.approach()
    assert rig.robot.dock_goals == [] and rig.robot.detections == []


def test_approach_refused_without_a_pose_or_a_candidate(rig: Rig) -> None:
    rig.robot.pose = None  # type: ignore[assignment]
    with pytest.raises(ToolError, match="robot pose"):
        rig.approach()
    rig.robot.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=0.5)
    rig.robot.costmap = CostGrid(
        data=np.full((240, 240), 100, dtype=np.int8), resolution=GRID_RES, origin_x=-6.0, origin_y=-6.0, age_s=0.1
    )
    with pytest.raises(ToolError, match="no valid approach candidate") as info:
        rig.approach()
    assert "collision" in str(info.value) and not rig.moved()


def test_a_given_pose_is_revalidated(rig: Rig) -> None:
    best = rig.suggest()["candidates"][0]["pose"]
    ok = rig.approach(dry_run=True, pose={"x": best["x"], "y": best["y"], "yaw": best["yaw"]})
    assert ok["plan"]["pose"]["x"] == pytest.approx(best["x"])
    with pytest.raises(ToolError, match="not a valid candidate"):
        rig.approach(dry_run=True, pose={"x": best["x"] + 1.0, "y": best["y"], "yaw": best["yaw"]})


# --- approach_for_grasp: dry run and mode selection ------------------------------------------------------------


def test_dry_run_navigate_sends_nothing(rig: Rig) -> None:
    res = rig.approach(dry_run=True)
    assert res["status"] == "planned" and res["dry_run"] is True
    assert res["mode"] == "navigate" and "not calibrated" in res["mode_reason"]
    pose = res["plan"]["pose"]
    nav = res["would_send"]["navigate_to_pose"]
    assert (nav["x"], nav["y"], nav["yaw"], nav["precise"]) == (
        pytest.approx(pose["x"]),
        pytest.approx(pose["y"]),
        pytest.approx(pose["yaw"]),
        True,
    )
    assert res["docking"]["server_ready"] is True and res["tracker"] is None
    assert not rig.moved() and rig.robot.stops == 0


def test_dry_run_servoed_reports_the_goal_and_one_tracker_step_without_publishing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    res = rig.approach(**VISIBLE_TARGET, dry_run=True)
    assert res["mode"] == "servoed" and "front" in res["mode_reason"]
    goal = res["would_send"]["dock_goal"]
    assert goal["dock_type"] == "grasp_servoed" and goal["navigate_to_staging_pose"] is True
    assert goal["dock_pose"]["x"] == pytest.approx(res["plan"]["pose"]["x"])
    assert res["would_send"]["action"] == "/grasp/dock_robot"
    tracker = res["tracker"]
    assert tracker["camera"] == "front" and tracker["seeded"] is True
    assert tracker["detection"]["pose"]["x"] == pytest.approx(res["plan"]["pose"]["x"], abs=0.02)
    assert tracker["detection"]["pose"]["yaw"] == pytest.approx(res["plan"]["pose"]["yaw"])
    assert not rig.moved()  # no goal, no detection published, no motion
    assert rig.robot.detections == []


def test_dry_run_servoed_with_the_target_out_of_the_current_view_has_no_detection(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    rig.robot.pose = BasePose(frame="map", x=-2.0, y=-2.0, yaw=3.0)
    res = rig.approach(**VISIBLE_TARGET, dry_run=True, prefer_mode="servoed")
    assert res["tracker"]["detection"] is None and res["tracker"]["seeded"] is False
    assert "not in view" in res["tracker"]["note"]
    assert rig.robot.detections == []


def test_mode_is_navigate_when_the_camera_does_not_see_the_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # calibrated camera, but the target is 3 m away from the dock pose's view: behind the camera for most yaws is
    # avoided by the planner, so use a cell that puts the target outside the image from the staging pose
    rig = Rig(tmp_path, monkeypatch, config=calibrated())
    monkeypatch.setattr(approach_tools, "visible", lambda scene, point, margin: False)
    res = rig.approach(**VISIBLE_TARGET, dry_run=True)
    assert res["mode"] == "navigate" and "front" in res["mode_reason"] and "staging" in res["mode_reason"]


def test_mode_is_servoed_only_when_visible_from_both_poses(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated())
    seen: list[tuple[float, float]] = []

    def visible_from_staging_only(scene: Any, point: Any, margin: float) -> bool:
        seen.append((float(point[0]), float(point[1])))
        return len(seen) == 1  # staging yes, dock no

    monkeypatch.setattr(approach_tools, "visible", visible_from_staging_only)
    res = rig.approach(**VISIBLE_TARGET, dry_run=True)
    assert res["mode"] == "navigate" and "dock pose" in res["mode_reason"]
    assert len(seen) >= 2
    # the staging pose is 0.30 m behind the dock pose, so the target is 0.30 m farther ahead from it
    assert seen[0][0] == pytest.approx(seen[1][0] + 0.30)


def test_prefer_mode(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated())
    forced = rig.approach(**VISIBLE_TARGET, dry_run=True, prefer_mode="navigate")
    assert forced["mode"] == "navigate" and "prefer_mode" in forced["mode_reason"]
    monkeypatch.setattr(approach_tools, "visible", lambda scene, point, margin: False)
    with pytest.raises(ToolError, match="servoed.*not possible"):
        rig.approach(**VISIBLE_TARGET, dry_run=True, prefer_mode="servoed")
    assert not rig.moved()


def test_uncalibrated_front_camera_is_named_in_the_reason(rig: Rig) -> None:
    res = rig.approach(dry_run=True)
    assert res["mode"] == "navigate" and res["mode_reason"] == "front: not calibrated"


def test_approach_offers_only_front(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    assert approach_tools.CAMERAS is camera_scene.CAMERAS == ("front",)
    for name in approach_tools.TOOL_NAMES:
        text = tool_schema_text(rig.server, name)
        assert '"enum": ["front"]' in text or '"const": "front"' in text
        assert "gripper camera" not in text and "'gripper'" not in text
        with pytest.raises(ToolError):
            call(rig.server, name, {"camera": "gripper", "u": 1, "v": 1, "dry_run": True})
    assert not rig.moved()
    res = rig.approach(**VISIBLE_TARGET, dry_run=True)
    assert res["mode"] == "servoed" and res["mode_reason"].startswith("front camera")
    assert res["tracker"]["camera"] == "front"
    assert {c[1][0] for c in rig.robot.calls if c[0] == "camera_image"} == {"front"}


# --- approach_for_grasp: running -------------------------------------------------------------------------------


def test_navigate_mode_runs_a_precise_goal_and_rechecks_reach(rig: Rig) -> None:
    res = rig.approach()
    nav = [c for c in rig.robot.calls if c[0] == "navigate"]
    assert len(nav) == 1
    x, y, yaw, frame, timeout_s, precise = nav[0][1]
    assert frame == "map" and precise is True
    assert (x, y, yaw) == pytest.approx((res["goal"]["x"], res["goal"]["y"], res["goal"]["yaw"]))
    assert res["status"] == "succeeded" and res["mode"] == "navigate"
    assert res["reach_after"]["reachable"] is True and res["reach_after"]["collision_free"] is True
    assert rig.robot.dock_goals == [] and rig.robot.detections == []
    # the reach re-check is point_reach of the target in the measured pose's base_link (rig pose 1, 2, 0.5)
    dx, dy = FAR_TARGET["x"] - 1.0, FAR_TARGET["y"] - 2.0
    after = rig.reach.points[-1]
    assert (after["x"], after["y"], after["z"]) == pytest.approx(
        (dx * math.cos(0.5) + dy * math.sin(0.5), -dx * math.sin(0.5) + dy * math.cos(0.5), FAR_TARGET["z"])
    )


def test_arrived_out_of_reach_is_never_a_success(rig: Rig) -> None:
    rig.reach.answers = [REACH_OK] * 5 + [REACH_NO]  # verification of the top 5 passes, the re-check fails
    res = rig.approach()
    assert res["status"] == "arrived_out_of_reach"
    assert res["reach_after"]["reachable"] is False
    assert "out of reach" in res["message"]
    assert res["reach_after"]["suggestion"] == REACH_NO["suggestion"]


def test_failed_navigation_is_not_rechecked(rig: Rig) -> None:
    rig.robot.navigate = lambda *a, **k: NavigationResult(status="aborted", message="no plan")  # type: ignore[method-assign]
    res = rig.approach()
    assert res["status"] == "failed" and res["reach_after"] is None
    assert len(rig.reach.points) == 5


def test_servoed_mode_runs_the_dock_goal_with_the_detection_loop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    alive: list[bool] = []

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        rig.robot.dock_goals.append((goal, timeout_s))
        alive.append(any(t.name == "target-tracker" and t.is_alive() for t in threading.enumerate()))
        return DockResult(status="succeeded", goal=goal, final_pose=BasePose(frame="map", x=1.0, y=2.0, yaw=0.5))

    monkeypatch.setattr(rig.robot, "dock", dock)
    res = rig.approach(**VISIBLE_TARGET, timeout_s=90.0)
    assert res["status"] == "succeeded" and res["mode"] == "servoed"
    ((goal, timeout_s),) = rig.robot.dock_goals
    assert goal.dock_type == "grasp_servoed" and goal.navigate_to_staging_pose is True
    assert goal.max_staging_time_s == pytest.approx(90.0) and timeout_s == pytest.approx(90.0)
    assert alive == [True]  # the loop runs while the dock goal runs
    assert not any(t.name == "target-tracker" and t.is_alive() for t in threading.enumerate())  # stopped after
    assert [c for c in rig.robot.calls if c[0] == "navigate"] == []
    assert res["reach_after"]["reachable"] is True


def wait_for(condition: Any, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return False


def test_servoed_detections_reach_publish_detection_with_pose_and_capture_stamp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        wait_for(lambda: rig.robot.detections)
        return DockResult(status="succeeded", goal=goal, final_pose=BasePose(frame="map", x=1.0, y=2.0, yaw=0.5))

    monkeypatch.setattr(rig.robot, "dock", dock)
    res = rig.approach(**VISIBLE_TARGET)
    assert rig.robot.detections, "the loop published nothing to robot.publish_detection"
    pose, stamp_ns = rig.robot.detections[0]
    assert isinstance(pose, BasePose) and pose.frame == "map"
    assert (pose.x, pose.y) == pytest.approx((res["goal"]["x"], res["goal"]["y"]), abs=0.1)  # a dock pose near the plan
    assert 10_000_000_000 < stamp_ns < 11_000_000_000  # the frame's capture stamp (10 s + 10 ms per frame)
    assert res["detections_published"] == len(rig.robot.detections) >= 1
    assert res["detections_rejected"] == 0


def test_servoed_result_counts_rejected_detections(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config = calibrated()
    config.approach.tracker.max_correction_m = 1e-9  # every match is a jump beyond the correction bound
    rig = Rig(tmp_path, monkeypatch, config=config, textured=True)

    def camera_calls() -> int:
        return len([c for c in rig.robot.calls if c[0] == "camera_image"])

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        wait_for(lambda: camera_calls() >= 6)
        return DockResult(status="succeeded", goal=goal, final_pose=BasePose(frame="map", x=1.0, y=2.0, yaw=0.5))

    monkeypatch.setattr(rig.robot, "dock", dock)
    res = rig.approach(**VISIBLE_TARGET)
    assert res["detections_rejected"] >= 1 and res["detections_published"] == 0 and rig.robot.detections == []


def test_stop_during_dock_cancels_and_stops_the_loop(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        rig.robot.dock_goals.append((goal, timeout_s))
        call(rig.server, "stop", {})  # the agent's stop arrives mid-dock; run_dock cancels the goal
        return DockResult(status="canceled", message="stop requested; dock cancelled", goal=goal)

    monkeypatch.setattr(rig.robot, "dock", dock)
    res = rig.approach(**VISIBLE_TARGET)
    assert res["status"] == "canceled" and res["mode"] == "servoed"
    assert res["reach_after"] is None and len(rig.reach.points) == 5  # verification only
    assert ("stop", ()) in rig.robot.calls
    assert not any(t.name == "target-tracker" and t.is_alive() for t in threading.enumerate())


def test_lost_target_inside_min_z_stops_with_reason(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    yaw = rig.robot.pose.yaw
    # base_link with the target 0.78 m behind it: about 0.10 m of depth along the tilted-down optical axis of the
    # synthetic front camera (0.8 m high, pitched 1 rad down, 0.25 m ahead of base_link), inside the D435 min-Z.
    overshot = BasePose(
        frame="map", x=VISIBLE_TARGET["x"] + 0.78 * math.cos(yaw), y=VISIBLE_TARGET["y"] + 0.78 * math.sin(yaw), yaw=yaw
    )
    published: list[int] = []

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        rig.robot.dock_goals.append((goal, timeout_s))
        assert wait_for(lambda: rig.robot.detections), "the tracker never matched before the overshoot"
        published.append(len(rig.robot.detections))
        rig.robot.pose = overshot
        assert wait_for(lambda: rig.robot.base_resets), "the lost target did not stop the dock"
        return DockResult(status="canceled", message="stop requested; dock cancelled", goal=goal, final_pose=overshot)

    monkeypatch.setattr(rig.robot, "dock", dock)
    res = rig.approach(**VISIBLE_TARGET)
    assert res["status"] == "target_lost" and res["mode"] == "servoed"
    assert "D435 min-Z" in res["target_lost"] and "0.15 m" in res["target_lost"]
    assert res["target_lost"] in res["message"]
    assert rig.robot.base_resets == 1
    assert len(rig.robot.detections) == published[0]  # nothing published after the loss: no placeholder pixel
    assert res["reach_after"] is None
    assert not any(t.name == "target-tracker" and t.is_alive() for t in threading.enumerate())


def test_loop_is_stopped_when_the_dock_raises(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    rig.robot.dock_server_up = False

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        raise RobotError("docking server not available")

    monkeypatch.setattr(rig.robot, "dock", dock)
    with pytest.raises(ToolError, match="docking server not available"):
        rig.approach(**VISIBLE_TARGET)
    assert not any(t.name == "target-tracker" and t.is_alive() for t in threading.enumerate())


def test_failed_servoed_dock_suggests_navigate_and_reports_the_code(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)

    def dock(goal: DockGoal, timeout_s: float) -> DockResult:
        return DockResult(
            status="failed",
            error_code=904,
            error_name="FAILED_TO_DETECT_DOCK",
            num_retries=1,
            message="docking aborted",
            goal=goal,
        )

    monkeypatch.setattr(rig.robot, "dock", dock)
    res = rig.approach(**VISIBLE_TARGET)
    assert res["status"] == "failed" and res["error_code"] == 904 and res["num_retries"] == 1
    assert "prefer_mode='navigate'" in res["message"]
    assert res["reach_after"] is None


def test_interrupted_dock_passes_the_event_through(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), textured=True)
    monkeypatch.setattr(
        rig.robot,
        "dock",
        lambda goal, timeout_s: DockResult(status="interrupted", interrupted_by="tilt_warning", goal=goal),
    )
    res = rig.approach(**VISIBLE_TARGET)
    assert res["status"] == "interrupted" and res["interrupted_by"] == "tilt_warning"
