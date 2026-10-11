"""Dynamic navigation footprint at the arm release (footprint.py, control.py): geometry (static H body, ground shadow
of the frozen arm, carried-object circle at the tool point), the triggers (take_control / release_control), the
park refusal while carrying an object and what reaches the consumers (Nav2 costmap footprint topics, the
collision_monitor ArmStop polygon and its enabled parameter, the laser_filter polygon parameter). No ROS."""

import math
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
from mcp_server.footprint import NavFootprint, arm_shadow, carried_circle, dynamic_outline
from mcp_server.footprint_geometry import offset_polygon, polygon_text
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, call

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
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
COSTMAP_TOPICS = ["/local_costmap/footprint", "/global_costmap/footprint"]
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


# --- geometry --------------------------------------------------------------------------------------------------------


def test_static_footprint_is_the_generated_nav2_footprint_and_laser_polygon(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    assert nav.static_points == static_points()
    laser = yaml.safe_load(LASER_FILTER_CONFIG.read_text())["polygon"]
    assert polygon_text(nav.static_laser_points) == laser


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


def test_parked_arm_stays_within_the_static_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    cfg = settings()
    static = Polygon(static_points())
    shadow = arm_shadow(parts_at(robot, HOME), cfg.circle_segments)
    outline = dynamic_outline(static, shadow, cfg.simplify_m)
    # The folded arm overhangs the body at most by a few cm (the gripper above the crossbar front).
    assert outline.area < static.area * 1.35
    assert outline.buffer(TOL).contains(static)


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
    nav = NavFootprint(settings(), robot.arm.cfg.body, robot)
    nav.publish_static()
    for topic in COSTMAP_TOPICS:
        assert last_polygon(robot, topic) == static_points()
    laser = offset_polygon(static_points(), settings().laser_filter_margin_m, settings().decimals)
    assert last_param(robot, "/laser_filter", "polygon") == polygon_text(laser)
    assert last_param(robot, "/collision_monitor", "ArmStop.enabled") is False
    assert robot.stamped_polygons == []
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
    move_arm(robot, {"elbow_flex": 0.4})
    result = ctl.release("arm", keep_position=False)
    assert result.reset == {"arm": "homed"}
    assert result.state.footprint is not None and result.state.footprint.kind == "static"
    assert last_polygon(robot, COSTMAP_TOPICS[0]) == static_points()


def test_republish_resends_the_current_footprint(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
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
    for target in ("arm", "all"):
        with pytest.raises(ToolError, match=r"put the object down first \(release_object\)"):
            ctl.release(target, keep_position=False)
    assert robot.arm.control_held is True
    # Mid-job release keeping the position stays allowed while carrying.
    assert ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED).reset == {"arm": "kept"}


def test_parked_release_is_refused_while_the_last_grasp_is_held(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm.grasp_held = True
    with pytest.raises(ToolError, match="last grasp"):
        ctl.release("all", keep_position=False)
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
    server = build_mcp_server(robot, robot.arm.cfg, TOKEN)
    state = call(server, "get_control_state", {}).structured_content
    assert state["footprint"] == {
        "kind": "static",
        "vertices": len(static_points()),
        "carried_object_radius_m": None,
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
    assert arm_stop["type"] == "polygon" and arm_stop["action_type"] == "stop"
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
