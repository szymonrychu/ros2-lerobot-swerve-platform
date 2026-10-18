"""verify.py pure parts: geometry ahead of a pose, path/polygon crossing, mask and costmap checks, speed-limit
expectation, POI payloads, pass/fail summary and config; plus a guard that the verifier never publishes cmd_vel."""

import json
import math
import sys
import types
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from .ros_stubs import MapMetaData, Pose, install_ros_stubs

VERIFY_ROS_MODULES = (
    "rclpy.action",
    "rclpy.duration",
    "rclpy.signals",
    "rclpy.time",
    "tf2_ros",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
)
install_ros_stubs(*VERIFY_ROS_MODULES)
for _name in ("SpeedLimit",):
    if not hasattr(sys.modules["nav2_msgs.msg"], _name):
        setattr(sys.modules["nav2_msgs.msg"], _name, MagicMock(name=f"nav2_msgs.msg.{_name}"))

from costmap_zones import verify  # noqa: E402
from costmap_zones.raster import MapGeometry  # noqa: E402

NODE_DIR = Path(__file__).resolve().parent.parent


def geometry(width: int = 40, height: int = 40, resolution: float = 0.1, ox: float = -2.0, oy: float = -2.0):
    return MapGeometry(
        frame="map",
        resolution=resolution,
        origin_x=ox,
        origin_y=oy,
        origin_yaw=0.0,
        width=width,
        height=height,
    )


# --- geometry ahead of a pose ---


def test_point_ahead_follows_the_heading() -> None:
    x, y = verify.point_ahead(verify.Pose2D(1.0, 2.0, math.pi / 2), 1.2)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(3.2)


def test_rectangle_ahead_is_centred_on_the_line_depth_along_width_across() -> None:
    rect = verify.rectangle_ahead(verify.Pose2D(0.0, 0.0, 0.0), ahead_m=0.6, depth_m=0.3, width_m=1.0)
    assert len(rect) == 4
    xs = sorted(p[0] for p in rect)
    ys = sorted(p[1] for p in rect)
    assert xs[0] == pytest.approx(0.45) and xs[-1] == pytest.approx(0.75)
    assert ys[0] == pytest.approx(-0.5) and ys[-1] == pytest.approx(0.5)


def test_rectangle_ahead_rotates_with_the_pose() -> None:
    rect = verify.rectangle_ahead(verify.Pose2D(1.0, 1.0, math.pi / 2), ahead_m=0.6, depth_m=0.3, width_m=1.0)
    xs = sorted(p[0] for p in rect)
    ys = sorted(p[1] for p in rect)
    assert xs[0] == pytest.approx(0.5) and xs[-1] == pytest.approx(1.5)
    assert ys[0] == pytest.approx(1.45) and ys[-1] == pytest.approx(1.75)


def test_box_around_pose_is_centred_on_the_robot() -> None:
    box = verify.rectangle_ahead(verify.Pose2D(2.0, -1.0, 0.3), ahead_m=0.0, depth_m=0.6, width_m=0.6)
    assert verify.point_in_polygon((2.0, -1.0), box)
    assert sum(p[0] for p in box) / 4 == pytest.approx(2.0)
    assert sum(p[1] for p in box) / 4 == pytest.approx(-1.0)


def test_yaw_from_quaternion() -> None:
    yaw = 0.7
    assert verify.yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


# --- polygon / path intersection ---

SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


def test_point_in_polygon_inside_outside_and_edge() -> None:
    assert verify.point_in_polygon((0.5, 0.5), SQUARE)
    assert not verify.point_in_polygon((1.5, 0.5), SQUARE)
    assert verify.point_in_polygon((1.0, 0.5), SQUARE)


def test_path_with_a_pose_inside_crosses() -> None:
    assert verify.path_crosses_polygon([(-1.0, 0.5), (0.5, 0.5)], SQUARE)


def test_path_jumping_over_the_polygon_crosses() -> None:
    # No pose inside, but the segment between two poses passes straight through the square.
    assert verify.path_crosses_polygon([(-1.0, 0.5), (2.0, 0.5)], SQUARE)


def test_path_going_around_does_not_cross() -> None:
    path = [(-1.0, 0.5), (-1.0, 2.0), (2.0, 2.0), (2.0, 0.5)]
    assert not verify.path_crosses_polygon(path, SQUARE)


def test_empty_and_single_pose_paths() -> None:
    assert not verify.path_crosses_polygon([], SQUARE)
    assert verify.path_crosses_polygon([(0.5, 0.5)], SQUARE)
    assert not verify.path_crosses_polygon([(3.0, 3.0)], SQUARE)


def test_segments_intersect_parallel_and_touching() -> None:
    assert verify.segments_intersect((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0))
    assert not verify.segments_intersect((0.0, 0.0), (2.0, 0.0), (0.0, 1.0), (2.0, 1.0))
    assert verify.segments_intersect((0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 1.0))
    assert not verify.segments_intersect((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0))
    assert verify.segments_intersect((0.0, 0.0), (2.0, 0.0), (1.0, 0.0), (3.0, 0.0))


# --- grid cells and checks ---


def test_cell_value_row_major_from_lowest_y() -> None:
    geo = geometry(width=4, height=3, resolution=1.0, ox=0.0, oy=0.0)
    data = list(range(12))
    assert verify.cell_value(geo, data, 0.5, 0.5) == 0
    assert verify.cell_value(geo, data, 3.5, 0.5) == 3
    assert verify.cell_value(geo, data, 1.5, 2.5) == 9
    assert verify.cell_value(geo, data, -0.1, 0.5) is None
    assert verify.cell_value(geo, data, 4.0, 0.5) is None
    assert verify.cell_value(geo, data, 0.5, 3.2) is None


def test_mask_problems_none_for_matching_valid_masks() -> None:
    geo = geometry(width=2, height=2)
    assert verify.mask_problems(geo, geo, [-1, 100, -1, -1], geo, [0, 40, 100, 0]) == []


def test_mask_problems_reports_geometry_mismatch() -> None:
    geo = geometry(width=2, height=2)
    other = geometry(width=3, height=2)
    problems = verify.mask_problems(geo, other, [-1] * 6, geo, [0] * 4)
    assert len(problems) == 1 and "keepout" in problems[0]
    problems = verify.mask_problems(geo, geo, [-1] * 4, geometry(width=2, height=2, ox=0.0), [0] * 4)
    assert len(problems) == 1 and "speed" in problems[0]


def test_mask_problems_reports_bad_values() -> None:
    geo = geometry(width=2, height=2)
    problems = verify.mask_problems(geo, geo, [-1, 0, 100, -1], geo, [0, -1, 40, 0])
    assert len(problems) == 2
    assert "keepout" in problems[0] and "0" in problems[0]
    assert "speed" in problems[1] and "-1" in problems[1]


def test_mask_problems_reports_wrong_data_length() -> None:
    geo = geometry(width=2, height=2)
    problems = verify.mask_problems(geo, geo, [-1] * 3, geo, [0] * 4)
    assert len(problems) == 1 and "keepout" in problems[0]


def test_keepout_marked_values() -> None:
    # OccupancyGrid translation of the costmap: 254 lethal -> 100, 253 inscribed -> 99, 252 override -> 98.
    assert verify.keepout_marked(100, robot_inside=False)
    assert verify.keepout_marked(99, robot_inside=False)
    assert not verify.keepout_marked(98, robot_inside=False)
    assert verify.keepout_marked(98, robot_inside=True)
    assert not verify.keepout_marked(50, robot_inside=True)
    assert not verify.keepout_marked(None, robot_inside=False)
    assert not verify.keepout_marked(-1, robot_inside=True)


def test_geometry_from_grid_message_info() -> None:
    info = MapMetaData(resolution=0.025, width=7, height=5, origin=Pose())
    grid = MagicMock()
    grid.header.frame_id = "map"
    grid.info = info
    assert verify.grid_geometry(grid) == MapGeometry.from_map_info("map", info)


# --- speed limit expectation ---


def test_speed_limit_expected_percent() -> None:
    assert verify.speed_limit_matches(40.0, True, 40)
    assert not verify.speed_limit_matches(40.0, False, 40)
    assert not verify.speed_limit_matches(100.0, True, 40)
    assert not verify.speed_limit_matches(0.0, True, 40)


def test_speed_limit_expected_no_limit() -> None:
    assert verify.speed_limit_matches(0.0, True, 0)
    assert verify.speed_limit_matches(0.0, False, 0)
    assert not verify.speed_limit_matches(40.0, True, 0)


# --- navigate_to_pose activity ---


def test_goal_is_active() -> None:
    assert not verify.goal_is_active([])
    assert not verify.goal_is_active([4, 5, 6])  # succeeded, canceled, aborted
    assert verify.goal_is_active([4, 2])  # executing
    assert verify.goal_is_active([1])  # accepted
    assert verify.goal_is_active([3])  # canceling


# --- POI payloads ---


def test_zone_poi_command_payload() -> None:
    polygon = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    raw = verify.poi_command("add", "req-1", verify.zone_poi("E7-verify-keepout", "keepout", polygon))
    data = json.loads(raw)
    assert data["op"] == "add" and data["request_id"] == "req-1"
    poi = data["poi"]
    assert poi["kind"] == "area" and poi["zone"] == "keepout" and poi["created_by"] == "user"
    assert poi["name"] == "E7-verify-keepout" and poi["status"] == "open"
    assert poi["polygon"] == [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]
    assert "speed_limit_pct" not in poi


def test_slow_zone_poi_carries_the_percent() -> None:
    poi = verify.zone_poi("E7-verify-slow", "slow", [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], speed_limit_pct=40)
    assert poi["zone"] == "slow" and poi["speed_limit_pct"] == 40


def test_verify_poi_ids_selects_by_prefix() -> None:
    raw = json.dumps(
        {
            "pois": [
                {"id": "a", "name": "E7-verify-keepout"},
                {"id": "b", "name": "kitchen"},
                {"id": "c", "name": "E7-verify-slow"},
                {"id": "d"},
            ],
            "revision": 3,
        }
    )
    assert verify.verify_poi_ids(raw, "E7-verify-") == ["a", "c"]


def test_verify_poi_ids_bad_list() -> None:
    assert verify.verify_poi_ids("not json", "E7-verify-") is None
    assert verify.verify_poi_ids(json.dumps({"x": 1}), "E7-verify-") is None


def test_result_matches_request() -> None:
    assert verify.parse_result(json.dumps({"request_id": "r", "ok": True, "message": "added", "poi": {"id": "x"}}), "r")
    assert verify.parse_result(json.dumps({"request_id": "other", "ok": True}), "r") is None
    assert verify.parse_result("garbage", "r") is None


# --- pass/fail summary ---


def test_format_result_lines() -> None:
    assert verify.format_result(verify.StepResult(2, "control path", True, "ok")) == "PASS step 2 control path: ok"
    assert verify.format_result(verify.StepResult(3, "keepout", False, "cell 0")) == "FAIL step 3 keepout: cell 0"


def test_summarise_all_pass_exits_zero() -> None:
    results = [verify.StepResult(n, f"s{n}", True, "") for n in range(1, 7)]
    lines, code = verify.summarise(results)
    assert code == 0
    assert lines[-1] == "RESULT: PASS (6/6 steps)"
    assert len(lines) == 7


def test_summarise_any_fail_exits_non_zero() -> None:
    results = [verify.StepResult(1, "a", True, ""), verify.StepResult(2, "b", False, "x")]
    lines, code = verify.summarise(results)
    assert code == 1
    assert lines[1].startswith("FAIL step 2")
    assert lines[-1] == "RESULT: FAIL (1 of 2 steps failed: 2)"


def test_summarise_nothing_ran_fails() -> None:
    lines, code = verify.summarise([])
    assert code == 1
    assert lines == ["RESULT: FAIL (no steps ran)"]


# --- config ---


def test_shipped_verify_yaml_has_the_design_values() -> None:
    config = verify.load_verify_config(NODE_DIR / "verify.yaml")
    assert config.goal_ahead_m == 1.2
    assert config.rect_ahead_m == 0.6
    assert config.rect_depth_m == 0.3
    assert config.rect_width_m == 1.0
    assert config.robot_box_m == 0.6
    assert config.slow_pct == 40
    assert config.timeouts.speed_limit_s == 3.0
    assert config.poi_name_prefix == "E7-verify-"


def test_config_rejects_unknown_keys_and_bad_values(tmp_path: Path) -> None:
    path = tmp_path / "verify.yaml"
    path.write_text("goal_ahead_m: 1.2\nbogus: 1\n")
    with pytest.raises(ValidationError):
        verify.load_verify_config(path)
    path.write_text("slow_pct: 5\n")
    with pytest.raises(ValidationError):
        verify.load_verify_config(path)
    path.write_text("rect_ahead_m: 1.5\ngoal_ahead_m: 1.2\n")
    with pytest.raises(ValidationError):
        verify.load_verify_config(path)


def test_config_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        verify.load_verify_config(tmp_path / "missing.yaml")


def test_config_path_from_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv(verify.ENV_VERIFY_CONFIG_KEY, raising=False)
    assert verify.verify_config_path() == NODE_DIR / "verify.yaml"
    monkeypatch.setenv(verify.ENV_VERIFY_CONFIG_KEY, str(tmp_path / "x.yaml"))
    assert verify.verify_config_path() == tmp_path / "x.yaml"


# --- never cmd_vel ---


class RecordingNode:
    """Records every publisher, subscription and action client topic the verifier creates."""

    def __init__(self) -> None:
        self.publishers: list[str] = []
        self.subscriptions: list[str] = []

    def create_publisher(self, _msg_type: Any, topic: str, _qos: Any) -> MagicMock:
        self.publishers.append(topic)
        return MagicMock()

    def create_subscription(self, _msg_type: Any, topic: str, _callback: Any, _qos: Any) -> MagicMock:
        self.subscriptions.append(topic)
        return MagicMock()

    def get_logger(self) -> MagicMock:
        return MagicMock()


def test_verifier_publishes_only_poi_commands_never_cmd_vel() -> None:
    node = RecordingNode()
    config = verify.load_verify_config(NODE_DIR / "verify.yaml")
    verify.Verifier(node, config)
    assert node.publishers == [config.poi_command_topic]
    assert not any("cmd_vel" in topic for topic in node.publishers + node.subscriptions)
    assert "cmd_vel" not in (NODE_DIR / "costmap_zones" / "verify.py").read_text()


# --- slow zone path cost (step 5) ---


@pytest.mark.parametrize(("mask", "cost"), [(1, 3), (60, 152), (99, 251), (100, 254)])
def test_keepout_cost_is_the_filter_translation(mask: int, cost: int) -> None:
    assert verify.keepout_cost(mask) == cost


@pytest.mark.parametrize(("cost", "grid"), [(0, 0), (1, 1), (152, 59), (252, 98), (253, 99), (254, 100), (255, -1)])
def test_published_grid_value_is_the_costmap_publisher_table(cost: int, grid: int) -> None:
    assert verify.published_grid_value(cost) == grid


def test_slow_value_translates_to_a_non_lethal_grid_value() -> None:
    assert verify.published_grid_value(verify.keepout_cost(60)) == 59


def test_mask_values_accept_the_slow_value_and_reject_others() -> None:
    geo = geometry(width=2, height=2)
    assert verify.mask_problems(geo, geo, [-1, 60, 100, -1], geo, [0] * 4, slow_value=60) == []
    problems = verify.mask_problems(geo, geo, [-1, 50, 100, -1], geo, [0] * 4, slow_value=60)
    assert len(problems) == 1 and "keepout" in problems[0] and "50" in problems[0]


def test_slow_value_comes_from_verify_yaml_default_60() -> None:
    config = verify.load_verify_config(NODE_DIR / "verify.yaml")
    assert config.slow_zone_mask_value == 60
    assert verify.VerifyConfig(slow_zone_mask_value=70).slow_zone_mask_value == 70
    with pytest.raises(ValidationError):
        verify.VerifyConfig(slow_zone_mask_value=100)


def test_resolve_slow_value_prefers_the_node_config(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    config = verify.VerifyConfig(slow_zone_mask_value=70)
    node_cfg = tmp_path / "config.yaml"
    node_cfg.write_text("slow_zone_mask_value: 45\n")
    monkeypatch.setenv("COSTMAP_ZONES_CONFIG", str(node_cfg))
    assert verify.resolve_slow_value(config) == 45
    monkeypatch.setenv("COSTMAP_ZONES_CONFIG", str(tmp_path / "missing.yaml"))
    assert verify.resolve_slow_value(config) == 70


class FakeSlowVerifier:
    """Verifier stand-in: the slow step run against canned cells and plans."""

    step_slow_cost = verify.Verifier.step_slow_cost
    rectangle = verify.Verifier.rectangle

    def __init__(self, keepout: int | None, costmap: int | None, plan_ok: bool, crosses: bool = False) -> None:
        self.config = verify.VerifyConfig(
            timeouts=verify.Timeouts(mask_s=0.01, costmap_s=0.01, plan_s=0.01),
        )
        self.slow_value = 60
        self.pose = verify.Pose2D(0.0, 0.0, 0.0)
        self.cells = {"keepout": keepout, "costmap": costmap}
        self.deleted: list[str] = []
        self.added: list[tuple[str, str, int | None]] = []
        x = 0.6
        self.outcome = verify.PlanOutcome(
            plan_ok, [(0.0, 0.0), (x, 0.0), (1.2, 0.0)] if crosses else [(0.0, 2.0), (1.2, 2.0)], "detail"
        )

    def require_pose(self) -> verify.Pose2D:
        return self.pose

    def goal(self) -> tuple[float, float]:
        return (1.2, 0.0)

    def wait_until(self, predicate: Any, _timeout_s: float) -> Any:
        return predicate() or None

    def latest_cell(self, key: str, _point: tuple[float, float]) -> int | None:
        return self.cells[key]

    def add_zone(self, suffix: str, zone: str, _polygon: Any, speed_limit_pct: int | None = None) -> str:
        self.added.append((suffix, zone, speed_limit_pct))
        return "slow-id"

    def delete_poi(self, poi_id: str) -> None:
        self.deleted.append(poi_id)

    def plan(self, _goal: Any, _yaw: float) -> verify.PlanOutcome:
        return self.outcome


def test_slow_step_passes_on_59_and_a_successful_plan() -> None:
    fake = FakeSlowVerifier(keepout=60, costmap=59, plan_ok=True, crosses=True)
    detail = fake.step_slow_cost()
    assert "59" in detail and "crosses" in detail
    assert fake.deleted == ["slow-id"]
    assert fake.added[0][1] == "slow"


@pytest.mark.parametrize("costmap", [99, 100])
def test_slow_step_fails_when_the_costmap_is_lethal(costmap: int) -> None:
    fake = FakeSlowVerifier(keepout=60, costmap=costmap, plan_ok=True)
    with pytest.raises(verify.VerifyError, match="59"):
        fake.step_slow_cost()
    assert fake.deleted == ["slow-id"]


def test_slow_step_fails_when_the_plan_fails() -> None:
    fake = FakeSlowVerifier(keepout=60, costmap=59, plan_ok=False)
    with pytest.raises(verify.VerifyError, match="plan"):
        fake.step_slow_cost()
    assert fake.deleted == ["slow-id"]


def test_slow_step_fails_when_the_mask_lacks_the_slow_value() -> None:
    fake = FakeSlowVerifier(keepout=100, costmap=59, plan_ok=True)
    with pytest.raises(verify.VerifyError, match="mask"):
        fake.step_slow_cost()
    assert fake.deleted == ["slow-id"]


def test_cleanup_deletes_the_slow_poi() -> None:
    listing = json.dumps(
        {
            "pois": [
                {"id": "slow-id", "name": "E7-verify-slow-cost"},
                {"id": "kitchen-id", "name": "kitchen"},
            ]
        }
    )

    class Cleanup:
        step_cleanup = verify.Verifier.step_cleanup
        delete_verify_pois = verify.Verifier.delete_verify_pois
        pose = None

        def __init__(self) -> None:
            self.deleted: list[str] = []
            self.latest: dict[str, Any] = {"poi_list": types.SimpleNamespace(data=listing)}
            self.config = types.SimpleNamespace(
                poi_name_prefix="E7-verify-",
                poi_list_topic="/poi/list",
                timeouts=types.SimpleNamespace(discovery_s=1.0, poi_s=1.0),
            )

        def wait_until(self, check: Any, _timeout: float) -> Any:
            return check()

        def delete_poi(self, poi_id: str) -> None:
            self.deleted.append(poi_id)
            self.latest["poi_list"] = types.SimpleNamespace(
                data=json.dumps({"pois": [{"id": "kitchen-id", "name": "kitchen"}]})
            )

    fake = Cleanup()
    assert "deleted 1" in fake.step_cleanup()
    assert fake.deleted == ["slow-id"]


def test_run_orders_the_slow_cost_step_between_keepout_and_speed() -> None:
    names: list[str] = []

    class Runner:
        run = verify.Verifier.run
        run_step = verify.Verifier.run_step
        results: list[Any] = []
        config = verify.VerifyConfig()

        def log(self, _text: str) -> None:
            pass

        def delete_verify_pois(self) -> int:
            return 0

        def __getattr__(self, name: str) -> Any:
            if name.startswith("step_"):
                return lambda: names.append(name) or "ok"
            raise AttributeError(name)

    Runner().run()
    assert names == [
        "step_masks",
        "step_control",
        "step_keepout_ahead",
        "step_keepout_robot",
        "step_slow_cost",
        "step_slow_robot",
        "step_cleanup",
    ]
