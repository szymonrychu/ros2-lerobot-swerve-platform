"""Tool centre point (jaw closing point) of the sim jaws and its calibration to the robot's measured tool_offset_m.

mcp_server expresses the tool point (`arm.tool_offset_m` in ansible/group_vars/client.yml) in the URDF frame
`gripper_frame_link`: gripper_link turned pi about its y axis and moved (-0.0079, -0.000218, -0.0981) m. The MuJoCo
`gripper` body frame is URDF gripper_link (tests/test_model.py), so the same transform applies here.

The vendored Menagerie jaws close at STOCK_CLOSING_POINT_GFL; the real jaws (measured 2026-10-08) close at
tool_offset_m. scene.build_model translates the finger collision geoms and the moving jaw body by the difference
(JAW_SHIFT), so the sim jaws close where the real ones do. See sim/README.md, "TCP calibration".
"""

from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
CLIENT_GROUP_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MCP_SERVER_NODE = "mcp_server"
GRIPPER_BODY = "gripper"
MOVING_JAW_BODY = "moving_jaw_so101_v1"
GRIPPER_JOINT = "gripper"
# Measured closed gripper position (mcp_server arm.gripper_closed_rad; the gripper joint has no zero offset).
GRIPPER_CLOSED_RAD = -0.165
# gripper_frame_link in gripper_link (URDF joint gripper_frame_joint: xyz -0.0079 -0.000218121 -0.0981274, rpy 0 pi 0).
GFL_POS_IN_GRIPPER = np.array([-0.0079, -0.000218121, -0.0981274])
GFL_ROT_IN_GRIPPER = np.diag([-1.0, 1.0, -1.0])
# Collision geoms of the fixed finger (the gripper body also carries the wrist housing, which stays in place).
FIXED_FINGER_GEOMS = (
    "fixed_jaw_box2",
    "fixed_jaw_box3",
    "fixed_jaw_box4",
    "fixed_jaw_box5",
    "fixed_jaw_box6",
    "fixed_jaw_box7",
    "fixed_jaw_sph_tip1",
    "fixed_jaw_sph_tip2",
    "fixed_jaw_sph_tip3",
)
FIXED_FINGER_MESH = "wrist_roll_follower_so101_gripper_part0_v1"
# Where the unmodified Menagerie jaws close (closing_point_gfl of the stock model), in gripper_frame_link (m).
STOCK_CLOSING_POINT_GFL = (-0.00158, 0.00022, 0.00336)
CLOSING_DISTMAX_M = 0.05


def gfl_to_gripper(point: np.ndarray) -> np.ndarray:
    """Point in gripper_frame_link to the gripper body (URDF gripper_link) frame.

    Args:
        point (np.ndarray): (3,) point in gripper_frame_link (m).

    Returns:
        np.ndarray: (3,) point in the gripper frame (m).
    """
    return GFL_POS_IN_GRIPPER + GFL_ROT_IN_GRIPPER @ np.asarray(point, dtype=float)


def gripper_to_gfl(point: np.ndarray) -> np.ndarray:
    """Point in the gripper body frame to gripper_frame_link.

    Args:
        point (np.ndarray): (3,) point in the gripper frame (m).

    Returns:
        np.ndarray: (3,) point in gripper_frame_link (m).
    """
    return GFL_ROT_IN_GRIPPER.T @ (np.asarray(point, dtype=float) - GFL_POS_IN_GRIPPER)


def jaw_shift(tool_offset_m: tuple[float, float, float] | None) -> np.ndarray:
    """Translation (gripper frame, m) that moves the stock jaws' closing point onto tool_offset_m.

    Args:
        tool_offset_m (tuple[float, float, float] | None): Wanted closing point in gripper_frame_link; None = stock.

    Returns:
        np.ndarray: (3,) shift in the gripper frame; zeros for None.
    """
    if tool_offset_m is None:
        return np.zeros(3)
    return gfl_to_gripper(np.array(tool_offset_m)) - gfl_to_gripper(np.array(STOCK_CLOSING_POINT_GFL))


def is_fixed_finger(geom: mujoco.MjsGeom) -> bool:
    """Whether a gripper-body geom spec is part of the fixed finger's collision geometry.

    Args:
        geom (mujoco.MjsGeom): Geom spec on the gripper body.

    Returns:
        bool: True for the finger boxes, tip spheres and the finger collision mesh.
    """
    return geom.name in FIXED_FINGER_GEOMS or geom.meshname == FIXED_FINGER_MESH


def shift_jaws(spec: mujoco.MjSpec, shift: np.ndarray) -> None:
    """Translate the fixed finger collision geoms and the moving jaw body by shift (gripper frame).

    The moving jaw (hinge, collision and visual geoms) moves rigidly, so the jaw opening per gripper angle is
    unchanged; only where the jaws meet moves. The vendored XML file is not touched (spec edits only).

    Args:
        spec (mujoco.MjSpec): Spec loaded from the vendored so101.xml.
        shift (np.ndarray): (3,) translation in the gripper frame (m).
    """
    gripper = spec.body(GRIPPER_BODY)
    for geom in gripper.geoms:
        if is_fixed_finger(geom):
            geom.pos = list(np.asarray(geom.pos) + shift)
    jaw = spec.body(MOVING_JAW_BODY)
    jaw.pos = list(np.asarray(jaw.pos) + shift)


def finger_geom_ids(model: mujoco.MjModel) -> tuple[list[int], list[int]]:
    """Collision geom ids of the fixed finger and of the moving jaw.

    Args:
        model (mujoco.MjModel): Compiled model.

    Returns:
        tuple[list[int], list[int]]: (fixed finger geoms, moving jaw geoms).
    """
    finger_mesh = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, FIXED_FINGER_MESH)
    gripper_id = model.body(GRIPPER_BODY).id
    jaw_id = model.body(MOVING_JAW_BODY).id
    fixed: list[int] = []
    moving: list[int] = []
    for gid in range(model.ngeom):
        if not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
            continue
        body = int(model.geom_bodyid[gid])
        named = model.geom(gid).name in FIXED_FINGER_GEOMS
        is_mesh = model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH and model.geom_dataid[gid] == finger_mesh
        if body == gripper_id and (named or is_mesh):
            fixed.append(gid)
        elif body == jaw_id:
            moving.append(gid)
    return fixed, moving


def closing_point_gfl(model: mujoco.MjModel) -> np.ndarray:
    """Where the jaws close: midpoint of the nearest points of the fixed finger and the moving jaw at the closed
    gripper angle, in gripper_frame_link (the frame of mcp_server's tool_offset_m).

    Args:
        model (mujoco.MjModel): Compiled model (stock or calibrated).

    Returns:
        np.ndarray: (3,) closing point in gripper_frame_link (m).
    """
    data = mujoco.MjData(model)
    data.qpos[model.joint(GRIPPER_JOINT).qposadr[0]] = GRIPPER_CLOSED_RAD
    mujoco.mj_forward(model, data)
    fixed, moving = finger_geom_ids(model)
    best, best_mid = np.inf, np.zeros(3)
    fromto = np.zeros(6)
    for a in fixed:
        for b in moving:
            dist = mujoco.mj_geomDistance(model, data, a, b, CLOSING_DISTMAX_M, fromto)
            if dist < best:
                best, best_mid = dist, (fromto[:3] + fromto[3:]) / 2.0
    body = data.body(GRIPPER_BODY)
    local = body.xmat.reshape(3, 3).T @ (best_mid - body.xpos)
    return gripper_to_gfl(local)


@lru_cache(maxsize=1)
def client_mcp_config() -> dict:
    """The mcp_server node config block of ansible/group_vars/client.yml (parsed YAML).

    Returns:
        dict: mcp_server configuration mapping (arm, grasp, floor_guard, ...).

    Raises:
        KeyError: If client.yml has no mcp_server node.
    """
    group_vars = yaml.safe_load(CLIENT_GROUP_VARS.read_text())
    for node in group_vars["ros2_nodes"]:
        if node["name"] == MCP_SERVER_NODE:
            config = node["config"]
            return yaml.safe_load(config) if isinstance(config, str) else config
    raise KeyError(f"no {MCP_SERVER_NODE} node in {CLIENT_GROUP_VARS}")


def client_tool_offset() -> tuple[float, float, float]:
    """The deployed arm.tool_offset_m (gripper_frame_link, m) from ansible/group_vars/client.yml.

    Returns:
        tuple[float, float, float]: (x, y, z) in m.
    """
    offset = client_mcp_config()["arm"]["tool_offset_m"]
    return (float(offset["x"]), float(offset["y"]), float(offset["z"]))
