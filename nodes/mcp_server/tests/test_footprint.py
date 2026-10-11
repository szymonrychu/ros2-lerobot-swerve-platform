"""Dynamic navigation footprint (footprint.py, control.py): geometry (static H body + parked arm, ground shadow of the
released arm, carried-object circle at the tool point, conservative disc without joint states), the triggers
(take_control / release_control, mcp_server start, the republish tick following the released arm, a converged home),
the park refusal while carrying an object and what reaches the consumers (Nav2 costmap footprint topics and inflation
parameters, the collision_monitor ArmStop polygon and its enabled parameter, the laser_filter polygon parameter), with
the per-consumer outcome in FootprintState.consumers. No ROS."""

import math
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from mcp.server.mcpserver.exceptions import ToolError
from shapely.geometry import Point, Polygon

from mcp_server.arm import ArmController
from mcp_server.config import McpServerConfig, NavFootprintSettings
from mcp_server.control import ControlManager
from mcp_server.footprint import NavFootprint, arm_shadow, carried_circle, dynamic_outline, parked_arm_shadow
from mcp_server.footprint_geometry import (
    circumscribed_radius,
    grow_extension,
    inscribed_radius,
    offset_polygon,
    polygon_text,
)
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, call

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
GEN_CONFIG = REPO_ROOT / "scripts" / "gen_nav_footprint.yaml"
# Arm folded over the crossbar (the measured stored home pose, test_arm_home_tracking.HOME).
HOME = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.35,
    "elbow_flex": 1.578,
    "wrist_flex": -0.1,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
# Arm turned to the left (+y; negative shoulder_pan with the deployed mount) and stretched out level.
SIDEWAYS = {
    "shoulder_pan": -math.pi / 2,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.1,
}
# Arm turned to the right (-y) and stretched out level.
RIGHT = {**SIDEWAYS, "shoulder_pan": math.pi / 2}
COSTMAP_TOPICS = ["/local_costmap/footprint", "/global_costmap/footprint"]
COSTMAP_NODES = ["/local_costmap/local_costmap", "/global_costmap/global_costmap"]
PUBLISHED_TOPICS = ["/local_costmap/published_footprint", "/global_costmap/published_footprint"]
INFLATION_RADIUS = "inflation_layer.inflation_radius"
COST_SCALING = "inflation_layer.cost_scaling_factor"
# Nav2 InflationLayer: cost 252 * exp(-k (d - inscribed)) beyond the inscribed radius.
INFLATED_COST = 252
ARM_STOP_TOPIC = "/collision_monitor/arm_stop_polygon"
CARRIED = 0.1
TOL = 1.5e-4


def deployed_config() -> McpServerConfig:
    """The mcp_server config deployed by Ansible (client.yml)."""
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "mcp_server")
    return McpServerConfig.model_validate(yaml.safe_load(entry["config"]))


def settings() -> NavFootprintSettings:
    nav = deployed_config().nav_footprint
    assert nav is not None, "client.yml mcp_server config needs the nav_footprint block"
    return nav


def footprint_robot(tmp_path: Path) -> FakeRobot:
    """FakeRobot whose arm runs on the deployed config (mount, joint offsets, tool point, body, nav_footprint)."""
    robot = FakeRobot(tmp_path)
    cfg = deployed_config()
    cfg.arm.urdf_path = McpServerConfig().arm.urdf_path
    cfg.arm.home_file = tmp_path / "home.yaml"
    kinematics = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    robot.arm = ArmController(robot.arm_backend, kinematics, load_joint_limits(cfg.arm.urdf_path), cfg)
    return robot


def manager(robot: FakeRobot) -> ControlManager:
    nav = NavFootprint(robot.arm.cfg.nav_footprint, robot.arm.cfg.body, robot)
    return ControlManager(robot.arm, robot, robot.arm.cfg, footprint=nav)


def parts_at(robot: FakeRobot, joints: dict[str, float]) -> dict[str, np.ndarray]:
    return robot.arm.body_check.part_points(joints)


def static_points() -> list[tuple[float, float]]:
    doc = yaml.safe_load(NAV2_PARAMS.read_text())
    return [tuple(p) for p in yaml.safe_load(doc["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])]


def last_polygon(robot: FakeRobot, topic: str) -> list[tuple[float, float]]:
    return [points for t, points in robot.footprint_polygons if t == topic][-1]


def last_param(robot: FakeRobot, node: str, name: str) -> Any:
    return [value for n, k, value in robot.node_params if n == node and k == name][-1]


def move_arm(robot: FakeRobot, joints: dict[str, float]) -> None:
    robot.arm_backend.positions.update(joints)


def make_stale(robot: FakeRobot) -> None:
    """Joint states stop arriving: the last sample ages past follower_stale_s."""
    robot.arm_backend.stale_after = robot.arm_backend.t
    robot.arm_backend.t += 10.0 * robot.arm.cfg.timeouts.follower_stale_s


def nav_for(robot: FakeRobot) -> NavFootprint:
    return NavFootprint(settings(), robot.arm.cfg.body, robot)


def static_inflation() -> tuple[float, float]:
    layer = nav2_doc()["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    return layer["inflation_radius"], layer["cost_scaling_factor"]


def rigid(points: list[tuple[float, float]], x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    """Points moved into a costmap global frame (what `<costmap>/published_footprint` carries), float32-rounded."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(float(np.float32(x + c * px - s * py)), float(np.float32(y + s * px + c * py))) for px, py in points]


# --- geometry --------------------------------------------------------------------------------------------------------


def test_static_footprint_is_the_generated_nav2_footprint_and_laser_polygon(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    assert nav.static_points == static_points()
    laser = yaml.safe_load(LASER_FILTER_CONFIG.read_text())["polygon"]
    assert polygon_text(nav.current.laser) == laser


def test_arm_out_sideways_extends_the_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    cfg = settings()
    static = Polygon(static_points())
    parts = parts_at(robot, SIDEWAYS)
    shadow = arm_shadow(parts, cfg.circle_segments)
    outline = dynamic_outline(static, shadow, cfg.simplify_m)
    assert outline.buffer(TOL).contains(static)
    assert outline.bounds[3] > static.bounds[3] + 0.1  # reaches well beyond the left side frame
    tool = parts["tool_point"][0]
    assert outline.buffer(cfg.simplify_m + TOL).contains(Point(tool[0], tool[1]))
    assert outline.geom_type == "Polygon" and not outline.interiors


def test_parked_arm_shadow_within_static_footprint(tmp_path: Path) -> None:
    """The parked gripper overhangs the crossbar front by ~10 cm: the static footprint covers it."""
    robot = footprint_robot(tmp_path)
    cfg = settings()
    static = Polygon(static_points())
    parts = parts_at(robot, HOME)
    assert static.buffer(cfg.simplify_m).contains(arm_shadow(parts, cfg.circle_segments))
    tool = parts["tool_point"][0]
    assert tool[0] > 0.3 and static.contains(Point(tool[0], tool[1]))


def test_generated_parked_shadow_is_the_home_pose_shadow() -> None:
    """client.yml parked_arm_shadow (scripts/gen_nav_footprint.py) is mcp_server's own shadow of the parked pose."""
    cfg = settings()
    joints = yaml.safe_load(GEN_CONFIG.read_text())["parked_arm_joints"]
    config = deployed_config()
    config.arm.urdf_path = McpServerConfig().arm.urdf_path
    shadow = parked_arm_shadow(config, joints, cfg.circle_segments, cfg.simplify_m, cfg.decimals)
    assert [tuple(p) for p in cfg.parked_arm_shadow] == shadow


def test_simplified_arm_extension_contains_the_raw_shadow(tmp_path: Path) -> None:
    """The arm shadow is grown by simplify_m before it is simplified, so simplification never trims it inward."""
    robot = footprint_robot(tmp_path)
    cfg = settings()
    for joints in (HOME, SIDEWAYS, RIGHT):
        shadow = arm_shadow(parts_at(robot, joints), cfg.circle_segments)
        grown = grow_extension(shadow, cfg.simplify_m)
        assert grown.buffer(1e-9).contains(shadow)
        assert shadow.buffer(cfg.simplify_m + 1e-6).contains(grown)
        assert len(grown.exterior.coords) <= len(shadow.exterior.coords) + 8


def test_carried_circle_is_centred_at_the_tool_point(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    cfg = settings()
    static = Polygon(static_points())
    parts = parts_at(robot, SIDEWAYS)
    tool = parts["tool_point"][0]
    circle = carried_circle(parts, CARRIED, cfg.circle_segments)
    assert circle.centroid.distance(Point(tool[0], tool[1])) < 1e-6
    assert circle.contains(Point(tool[0], tool[1]).buffer(CARRIED - 1e-3))
    shadow = arm_shadow(parts, cfg.circle_segments)
    with_object = dynamic_outline(static, shadow.union(circle), cfg.simplify_m)
    without = dynamic_outline(static, shadow, cfg.simplify_m)
    disc = Point(tool[0], tool[1]).buffer(CARRIED - cfg.simplify_m - 1e-3)
    assert with_object.contains(disc)
    assert not without.contains(disc)


# --- triggers ---------------------------------------------------------------------------------------------------------


def test_mcp_server_start_publishes_the_static_footprint_everywhere(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    move_arm(robot, HOME)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    nav.start()
    for topic in COSTMAP_TOPICS:
        assert last_polygon(robot, topic) == static_points()
    laser = offset_polygon(static_points(), settings().laser_filter_margin_m, settings().decimals)
    assert last_param(robot, "/laser_filter", "polygon") == polygon_text(laser)
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is False
    assert robot.stamped_polygons == []
    radius, factor = static_inflation()
    for node in COSTMAP_NODES:
        assert last_param(robot, node, INFLATION_RADIUS) == radius
        assert last_param(robot, node, COST_SCALING) == factor
    assert nav.state().kind == "static" and nav.state().vertices == len(static_points())


def test_keep_position_release_publishes_the_dynamic_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    move_arm(robot, SIDEWAYS)
    result = ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    assert result.reset == {"arm": "kept"}
    fp = result.state.footprint
    assert fp is not None and fp.kind == "dynamic" and fp.carried_object_radius_m == CARRIED
    published = last_polygon(robot, COSTMAP_TOPICS[0])
    assert last_polygon(robot, COSTMAP_TOPICS[1]) == published
    assert fp.vertices == len(published)
    assert Polygon(published).bounds[3] > Polygon(static_points()).bounds[3] + 0.1
    tool = parts_at(robot, SIDEWAYS)["tool_point"][0]
    assert Polygon(published).contains(Point(tool[0], tool[1]).buffer(CARRIED - settings().simplify_m - 1e-3))
    laser = offset_polygon(published, settings().laser_filter_margin_m, settings().decimals)
    assert last_param(robot, "/laser_filter", "polygon") == polygon_text(laser)
    topic, frame, arm_stop = robot.stamped_polygons[-1]
    assert topic == ARM_STOP_TOPIC and frame == "base_link"
    assert Polygon(arm_stop).contains(Point(tool[0], tool[1]).buffer(CARRIED))  # swept by stopbox_margin_m
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is True


def test_take_control_arm_reverts_to_the_static_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    move_arm(robot, SIDEWAYS)
    ctl.release("arm", keep_position=True)
    assert ctl.state().footprint.kind == "dynamic"
    state = ctl.take("arm")
    assert state.footprint is not None and state.footprint.kind == "static"
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == static_points()
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is False


def test_parked_release_publishes_the_static_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm.move_joints({"elbow_flex": 0.4})  # commanded (a teleported follower would be a tracking fault)
    result = ctl.release("arm", keep_position=False)
    assert result.reset == {"arm": "homed"}
    assert result.state.footprint is not None and result.state.footprint.kind == "static"
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == static_points()


def test_republish_resends_the_current_footprint_while_no_costmap_reports(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    move_arm(robot, HOME)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    nav.start()
    period, tick = robot.footprint_timers[-1]
    assert period == settings().republish_s
    robot.footprint_polygons.clear()
    tick()
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == static_points()


def test_parked_release_is_refused_while_carrying(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    ctl.take("arm")
    with pytest.raises(ToolError, match=r"put the object down first \(release_object\)"):
        ctl.release("arm", keep_position=False)
    assert ctl.release("all", keep_position=False).reset["arm"].startswith("refused: carrying (")
    assert robot.arm.control_held is True
    # Mid-job release keeping the position stays allowed while carrying.
    assert ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED).reset == {"arm": "kept"}


def test_parked_release_is_refused_while_the_last_grasp_is_held(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm.grasp_held = True
    with pytest.raises(ToolError, match="last grasp"):
        ctl.release("arm", keep_position=False)
    robot.arm.grasp_held = False
    assert ctl.release("arm", keep_position=False).released == ["arm"]


def test_parked_release_is_refused_while_the_gripper_holds_a_load(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    cfg = robot.arm.cfg
    robot.arm_backend.positions["gripper"] = cfg.arm.gripper_closed_rad + 0.2 * (
        cfg.arm.gripper_open_rad - cfg.arm.gripper_closed_rad
    )
    robot.arm_backend.efforts["gripper"] = cfg.motion_queue.holding_min_effort + 50.0
    with pytest.raises(ToolError, match="holding load"):
        ctl.release("arm", keep_position=False)


def test_opening_the_gripper_clears_the_carried_object(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    ctl.take("arm")
    robot.arm.set_gripper(open_fraction=0.8)
    assert robot.arm.grasp_held is False
    assert ctl.release("arm", keep_position=False).reset == {"arm": "homed"}


def test_carried_radius_is_rejected_without_keep_position(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    with pytest.raises(ToolError, match="only valid with keep_position=true"):
        ctl.release("arm", keep_position=False, carried_object_radius_m=CARRIED)
    with pytest.raises(ToolError, match=r"0\.\.0\.3"):
        ctl.release("arm", keep_position=True, carried_object_radius_m=0.31)
    with pytest.raises(ToolError, match="subsystem='arm' or 'all'"):
        ctl.release("base", keep_position=True, carried_object_radius_m=CARRIED)
    assert robot.arm.control_held is True


def test_get_control_state_reports_the_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    move_arm(robot, HOME)
    for topic in PUBLISHED_TOPICS:
        robot.observed_footprints[topic] = rigid(static_points(), 1.0, -2.0, 0.3)
    server = build_mcp_server(robot, robot.arm.cfg, TOKEN)
    state = call(server, "get_control_state", {}).structured_content
    assert state["footprint"] == {
        "kind": "static",
        "vertices": len(static_points()),
        "carried_object_radius_m": None,
        "consumers": {
            "local_costmap": "ok",
            "global_costmap": "ok",
            "laser_filter": "ok",
            "collision_monitor": "ok",
        },
    }
    call(server, "take_control", {"subsystem": "arm"})
    move_arm(robot, SIDEWAYS)
    result = call(
        server, "release_control", {"subsystem": "arm", "keep_position": True, "carried_object_radius_m": CARRIED}
    ).structured_content
    footprint = result["state"]["footprint"]
    assert footprint["kind"] == "dynamic" and footprint["carried_object_radius_m"] == CARRIED
    assert footprint["vertices"] == len(last_polygon(robot, COSTMAP_TOPICS[0]))
    with pytest.raises(ToolError):
        call(server, "release_control", {"subsystem": "arm", "keep_position": True, "carried_object_radius_m": 0.5})


def test_without_a_nav_footprint_block_nothing_is_published(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    assert call(server, "get_control_state", {}).structured_content["footprint"] is None
    assert robot.footprint_polygons == [] and robot.node_params == []


# --- consumer config ------------------------------------------------------------------------------------------------


def nav2_doc() -> dict[str, Any]:
    return yaml.safe_load(NAV2_PARAMS.read_text())


def test_costmap_footprint_topics_are_the_costmaps_footprint_inputs() -> None:
    """Costmap2DROS subscribes its relative `footprint` topic (geometry_msgs/Polygon) in its own namespace, so the
    local and global costmaps listen on /local_costmap/footprint and /global_costmap/footprint."""
    doc = nav2_doc()
    expected = []
    for costmap in ("local_costmap", "global_costmap"):
        params = doc[costmap][costmap]["ros__parameters"]
        assert params["footprint_padding"] == 0.0  # the published polygon is used as is
        expected.append(f"/{costmap}/footprint")
    assert settings().costmap_footprint_topics == expected


def test_collision_monitor_arm_stop_follows_the_published_arm_shadow() -> None:
    """The StopBox velocity polygons are static parameters (read once at configure): the released arm is covered by
    ArmStop, a stop polygon with no static points that takes its shape from mcp_server (transient local) and is
    switched on and off through its enabled parameter."""
    cm = nav2_doc()["collision_monitor"]["ros__parameters"]
    nav = settings()
    assert cm["polygons"][0] == "StopBox" and nav.arm_stop_polygon in cm["polygons"]
    arm_stop = cm[nav.arm_stop_polygon]
    # approach: direction-aware (the robot can still back away from an obstacle next to the arm).
    assert arm_stop["type"] == "polygon" and arm_stop["action_type"] == "approach"
    assert arm_stop["time_before_collision"] > 0.0 and arm_stop["simulation_time_step"] > 0.0
    assert "points" not in arm_stop
    assert arm_stop["polygon_sub_topic"] == nav.arm_stop_topic
    assert arm_stop["polygon_subscribe_transient_local"] is True
    assert arm_stop["enabled"] is False  # on only while the footprint is dynamic
    assert arm_stop["min_points"] == cm["StopBox"]["min_points"]
    assert nav.collision_monitor_node == "/collision_monitor"


def test_laser_filter_is_the_runtime_settable_self_filter() -> None:
    """laser_filters' polygon filters cannot change their polygon at runtime in Jazzy (2.0.10: the chain parameter
    is read-only, the reconfigure callback and footprint topic live on an unspun node): nodes/laser_filter is the
    repo's own node whose `polygon` parameter mcp_server sets."""
    cfg = yaml.safe_load(LASER_FILTER_CONFIG.read_text())
    assert cfg["polygon_frame"] == settings().frame_id
    assert "type" not in cfg and "scan_to_scan_filter_chain" not in cfg
    node_source = (REPO_ROOT / "nodes" / "laser_filter" / "laser_filter" / "node.py").read_text()
    assert f'NODE_NAME = "{settings().laser_filter_node.lstrip("/")}"' in node_source
    assert f'POLYGON_PARAM = "{settings().laser_polygon_param}"' in node_source


# --- inflation (H5) ---------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(("joints", "carried"), [(HOME, None), (SIDEWAYS, None), (SIDEWAYS, 0.3)])
def test_dynamic_footprint_sets_inflation_covering_circumscribed_radius(
    tmp_path: Path, joints: dict[str, float], carried: float | None
) -> None:
    """MPPI / Smac need the inflation radius >= the footprint circumscribed radius and a cost >= 1 there: every
    published footprint sets both costmaps' inflation to cover it, the static one restores the generated values."""
    robot = footprint_robot(tmp_path)
    nav = nav_for(robot)
    nav.publish_dynamic(parts_at(robot, joints), carried)
    published = last_polygon(robot, COSTMAP_TOPICS[0])
    rc, ri = circumscribed_radius(published), inscribed_radius(published)
    res = settings().costmap_resolution
    for node in COSTMAP_NODES:
        radius = last_param(robot, node, INFLATION_RADIUS)
        factor = last_param(robot, node, COST_SCALING)
        assert rc <= radius < rc + res + 1e-9
        assert 0.0 < factor <= math.log(INFLATED_COST) / (rc - ri)
        assert int(INFLATED_COST * math.exp(-factor * (rc - ri))) >= 1
    nav.publish_static()
    radius, factor = static_inflation()
    for node in COSTMAP_NODES:
        assert last_param(robot, node, INFLATION_RADIUS) == radius
        assert last_param(robot, node, COST_SCALING) == factor


# --- live arm pose (H7, H8) -------------------------------------------------------------------------------------------


def test_released_footprint_follows_arm_moved_by_other_source(tmp_path: Path) -> None:
    """After a keep release the leader arm / web UI may move the arm: the republish tick recomputes the footprint from
    the measured pose; with stale joint states it keeps the last one; while autonomy holds the arm it stays."""
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    static = Polygon(static_points())
    ctl.take("arm")
    move_arm(robot, SIDEWAYS)
    ctl.release("arm", keep_position=True)
    nav = ctl.footprint
    assert nav is not None
    move_arm(robot, RIGHT)
    nav.tick()
    moved = last_polygon(robot, COSTMAP_TOPICS[0])
    assert Polygon(moved).bounds[1] < static.bounds[1] - 0.1  # out to the right now
    assert Polygon(moved).bounds[3] < static.bounds[3] + 0.01  # no longer to the left
    assert ctl.state().footprint.kind == "dynamic"
    make_stale(robot)
    move_arm(robot, SIDEWAYS)
    nav.tick()
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == moved
    robot2 = footprint_robot(tmp_path / "held")
    ctl2 = manager(robot2)
    ctl2.take("arm")
    move_arm(robot2, SIDEWAYS)
    ctl2.footprint.tick()
    assert last_polygon(robot2, COSTMAP_TOPICS[0]) == static_points()
    assert ctl2.state().footprint.kind == "static"


def test_start_with_arm_out_publishes_dynamic(tmp_path: Path) -> None:
    """An mcp_server restart with the arm released out publishes the footprint of the measured pose, not static."""
    robot = footprint_robot(tmp_path)
    move_arm(robot, HOME)
    robot.arm.set_home()
    move_arm(robot, SIDEWAYS)
    assert robot.arm.at_home() is False
    nav = nav_for(robot)
    nav.start()
    assert nav.state().kind == "dynamic"
    assert Polygon(last_polygon(robot, COSTMAP_TOPICS[0])).bounds[3] > Polygon(static_points()).bounds[3] + 0.1
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is True


def test_start_without_joint_states_publishes_conservative(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    make_stale(robot)
    nav = nav_for(robot)
    nav.start()
    assert nav.state().kind == "conservative"
    mount = Point(robot.arm_mount_xy())
    assert (
        Polygon(last_polygon(robot, COSTMAP_TOPICS[0]))
        .buffer(TOL)
        .contains(mount.buffer(settings().conservative_reach_m - 1e-3))
    )


def test_keep_release_without_fresh_joints_publishes_conservative_footprint(tmp_path: Path) -> None:
    """No fresh joint states at a keep release: the arm pose is unknown, so the footprint becomes a max-reach disc
    around the arm mount (plus the carried object), not the bare body."""
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    move_arm(robot, SIDEWAYS)
    make_stale(robot)
    nav = ctl.footprint
    assert nav is not None
    outcome = ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    assert "footprint unchanged" not in str(outcome)
    # published at once by the release itself, with the carried circle, before any tick.
    state = nav.state()
    assert state.kind == "conservative" and state.carried_object_radius_m == CARRIED
    reach = settings().conservative_reach_m
    mount = Point(robot.arm_mount_xy())
    published = Polygon(last_polygon(robot, COSTMAP_TOPICS[0]))
    assert published.buffer(TOL).contains(mount.buffer(reach + CARRIED - 1e-3))
    assert published.buffer(TOL).contains(Polygon(static_points()))
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is True
    # the republish tick keeps the conservative footprint and its carried circle.
    nav.tick()
    state = nav.state()
    assert state.kind == "conservative" and state.carried_object_radius_m == CARRIED


# --- publishing (M7, M8, M9, M10, L1, L12) ----------------------------------------------------------------------------


class BlockingSink(FakeRobot):
    """FakeRobot whose first costmap publish from the blocking thread waits for `release`."""

    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path)
        self.blocked = threading.Event()
        self.release = threading.Event()
        self.block_thread: int | None = None

    def publish_polygon(self, topic: str, points: list[tuple[float, float]]) -> None:
        if threading.get_ident() == self.block_thread and not self.release.is_set():
            self.blocked.set()
            self.release.wait(5.0)
        super().publish_polygon(topic, points)


def test_apply_is_serialised_against_state_change(tmp_path: Path) -> None:
    """A tick that is sending the dynamic footprint while take_control publishes static must not finish last."""
    base = footprint_robot(tmp_path)
    robot = BlockingSink(tmp_path / "blocking")
    robot.arm = base.arm
    robot.arm_backend = base.arm_backend
    move_arm(robot, SIDEWAYS)
    nav = nav_for(robot)
    nav.publish_dynamic(parts_at(robot, SIDEWAYS), None)

    def run_tick() -> None:
        robot.block_thread = threading.get_ident()
        nav.tick()

    tick = threading.Thread(target=run_tick)
    tick.start()
    assert robot.blocked.wait(5.0)
    take = threading.Thread(target=nav.publish_static)
    take.start()
    time.sleep(0.1)
    robot.release.set()
    tick.join(5.0)
    take.join(5.0)
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == static_points()
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is False
    assert nav.state().kind == "static"


def test_republish_only_on_consumer_mismatch(tmp_path: Path) -> None:
    """Re-publishing an unchanged footprint makes every costmap re-inflate: the tick republishes only when a
    costmap's published_footprint differs from the current footprint."""
    robot = footprint_robot(tmp_path)
    move_arm(robot, SIDEWAYS)
    nav = nav_for(robot)
    nav.start()
    published = last_polygon(robot, COSTMAP_TOPICS[0])
    for topic in PUBLISHED_TOPICS:
        robot.observed_footprints[topic] = rigid(published, 3.0, -1.0, 2.1)
    robot.footprint_polygons.clear()
    sent = len(robot.node_params)
    nav.tick()
    assert robot.footprint_polygons == []
    assert [p for p in robot.node_params[sent:] if p[1].startswith("inflation_layer.")] == []
    consumers = nav.state().consumers
    assert consumers["local_costmap"] == "ok" and consumers["global_costmap"] == "ok"


def test_footprint_recovers_after_a_costmap_restart(tmp_path: Path) -> None:
    """A restarted Nav2 costmap falls back to the static footprint from its params: its published_footprint no longer
    matches, so the next tick republishes there (footprint and inflation), and only there."""
    robot = footprint_robot(tmp_path)
    move_arm(robot, SIDEWAYS)
    nav = nav_for(robot)
    nav.start()
    dynamic = last_polygon(robot, COSTMAP_TOPICS[0])
    robot.observed_footprints[PUBLISHED_TOPICS[0]] = rigid(static_points(), 0.5, 0.5, -1.0)
    robot.observed_footprints[PUBLISHED_TOPICS[1]] = rigid(dynamic, 0.5, 0.5, -1.0)
    robot.footprint_polygons.clear()
    sent = len(robot.node_params)
    nav.tick()
    assert robot.footprint_polygons == [(COSTMAP_TOPICS[0], dynamic)]
    resent = {(n, k) for n, k, _ in robot.node_params[sent:] if k.startswith("inflation_layer.")}
    assert resent == {(COSTMAP_NODES[0], INFLATION_RADIUS), (COSTMAP_NODES[0], COST_SCALING)}
    assert nav.state().consumers["local_costmap"].startswith("republished")
    robot.observed_footprints[PUBLISHED_TOPICS[0]] = rigid(dynamic, 0.5, 0.5, -1.0)
    nav.tick()
    assert nav.state().consumers["local_costmap"] == "ok"


def test_control_state_reports_consumer_failures(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    move_arm(robot, HOME)
    robot.param_outcomes["/laser_filter"] = "unavailable: /laser_filter/set_parameters not ready"
    robot.param_outcomes["/collision_monitor"] = "refused: ArmStop.enabled: parameter not declared"
    ctl = manager(robot)
    ctl.footprint.start()
    ctl.footprint.tick()
    consumers = ctl.state().footprint.consumers
    assert consumers["laser_filter"] == "unavailable: /laser_filter/set_parameters not ready"
    assert consumers["collision_monitor"] == "refused: ArmStop.enabled: parameter not declared"
    assert consumers["local_costmap"] == "unavailable: nothing received on /local_costmap/published_footprint"
    robot.param_outcomes.clear()
    for topic in PUBLISHED_TOPICS:
        robot.observed_footprints[topic] = rigid(static_points(), 0.0, 0.0, 0.0)
    ctl.footprint.tick()
    assert set(ctl.state().footprint.consumers.values()) == {"ok"}


def test_arm_stop_excludes_body_and_is_off_when_shadow_inside_static(tmp_path: Path) -> None:
    """ArmStop covers only the part of the arm outside the body (+ margin); a shadow inside the static outline (the
    parked arm) publishes the static footprint with ArmStop off."""
    robot = footprint_robot(tmp_path)
    nav = nav_for(robot)
    margin = settings().stopbox_margin_m
    static = Polygon(static_points())
    # Arm out to the right: one piece outside the body, ArmStop stays out of the body (but for the margin).
    nav.publish_dynamic(parts_at(robot, RIGHT), None)
    arm_stop = Polygon(robot.stamped_polygons[-1][2])
    assert arm_stop.intersection(static.buffer(-1.5 * margin - TOL)).area == 0.0
    tool = parts_at(robot, RIGHT)["tool_point"][0]
    assert arm_stop.contains(Point(tool[0], tool[1]))
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is True
    # Arm out to the left: it also spans the front notch, ArmStop covers every part outside the body.
    nav.publish_dynamic(parts_at(robot, SIDEWAYS), None)
    arm_stop = Polygon(robot.stamped_polygons[-1][2])
    outside = arm_shadow(parts_at(robot, SIDEWAYS), settings().circle_segments).difference(static)
    assert outside.geom_type == "MultiPolygon"
    assert arm_stop.buffer(TOL).contains(outside)
    state = nav.publish_dynamic(parts_at(robot, HOME), None)
    assert state.kind == "static"
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == static_points()
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is False


def test_consumer_ready_edge_resends_its_parameters(tmp_path: Path) -> None:
    """A restarted laser_filter / collision_monitor / costmap loads its static yaml values: the moment its parameter
    service comes up again it gets the current footprint, without waiting for the next tick."""
    robot = footprint_robot(tmp_path)
    move_arm(robot, SIDEWAYS)
    nav = nav_for(robot)
    nav.start()
    laser = last_param(robot, "/laser_filter", "polygon")
    dynamic = last_polygon(robot, COSTMAP_TOPICS[0])
    ready = robot.consumer_ready_callbacks[-1]
    robot.node_params.clear()
    robot.footprint_polygons.clear()
    ready("/laser_filter")
    assert robot.node_params == [("/laser_filter", "polygon", laser)]
    assert robot.footprint_polygons == []
    robot.node_params.clear()
    ready(COSTMAP_NODES[1])
    assert {k for n, k, _ in robot.node_params if n == COSTMAP_NODES[1]} == {INFLATION_RADIUS, COST_SCALING}
    assert robot.footprint_polygons == [(COSTMAP_TOPICS[1], dynamic)]


# --- arm_outside_static_m (E4-T2: guard field arm.outside_footprint_m) ----------------------------------------------


def test_arm_outside_static_m_is_zero_for_the_parked_arm(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    assert nav.arm_outside_static_m(parts_at(robot, HOME)) == 0.0


def test_arm_outside_static_m_is_the_protrusion_distance(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    parts = parts_at(robot, SIDEWAYS)
    outside = nav.arm_outside_static_m(parts)
    shadow = arm_shadow(parts, settings().circle_segments)
    expected = max(Point(x, y).distance(nav.static) for x, y in shadow.exterior.coords)
    assert outside > 0.1
    assert outside == pytest.approx(expected, abs=1e-9)


def test_arm_outside_static_m_five_cm_protrusion_within_5_mm(tmp_path: Path) -> None:
    """A synthetic part cloud sticking 5 cm out of the middle of the longest static edge measures 0.05."""
    robot = footprint_robot(tmp_path)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    ring = np.asarray(nav.static.exterior.coords)
    edges = list(zip(ring[:-1], ring[1:], strict=True))
    (x0, y0), (x1, y1) = max(edges, key=lambda e: math.dist(e[0], e[1]))
    mid = Point((x0 + x1) / 2, (y0 + y1) / 2)
    length = math.dist((x0, y0), (x1, y1))
    normal = np.array([(y1 - y0) / length, -(x1 - x0) / length])
    if nav.static.contains(Point(mid.x + 1e-3 * normal[0], mid.y + 1e-3 * normal[1])):
        normal = -normal
    tip = np.array([mid.x, mid.y]) + 0.05 * normal
    parts = {"tool_point": np.array([[tip[0], tip[1], 0.1], [mid.x, mid.y, 0.1]])}
    assert nav.arm_outside_static_m(parts) == pytest.approx(0.05, abs=0.005)
    inside = nav.static.representative_point()
    assert nav.arm_outside_static_m({"tool_point": np.array([[inside.x, inside.y, 0.1]])}) == 0.0
