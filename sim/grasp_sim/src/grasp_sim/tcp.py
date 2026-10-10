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
# Moving jaw inner silhouette (moving_jaw_inner_profile): depth samples into the jaws from just beyond the tips, and
# the depth band around each sample in which primitive geom surface points count.
PROFILE_DEPTH_START_M = -0.0014
PROFILE_DEPTH_END_M = 0.077
PROFILE_DEPTH_STEP_M = 0.002
PROFILE_BAND_M = 0.0008
PRIMITIVE_SAMPLES = 24


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


def geom_surface_points(model: mujoco.MjModel, data: mujoco.MjData, gid: int) -> np.ndarray:
    """World points on the surface of a primitive collision geom (box, sphere, capsule), sampled on a grid.

    Args:
        model (mujoco.MjModel): Model.
        data (mujoco.MjData): Data after mj_forward.
        gid (int): Geom id.

    Returns:
        np.ndarray: (n, 3) points (m).

    Raises:
        ValueError: For a geom type without a sampler.
    """
    kind, size = model.geom_type[gid], model.geom_size[gid]
    pos, rot = data.geom_xpos[gid], data.geom_xmat[gid].reshape(3, 3)
    u = np.linspace(-1.0, 1.0, PRIMITIVE_SAMPLES)
    ang = np.linspace(0.0, 2.0 * np.pi, PRIMITIVE_SAMPLES, endpoint=False)
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        a, b = (g.ravel() for g in np.meshgrid(u, u))
        faces = []
        for axis in range(3):
            for side in (-1.0, 1.0):
                local = np.zeros((a.size, 3))
                others = [i for i in range(3) if i != axis]
                local[:, axis] = side * size[axis]
                local[:, others[0]] = a * size[others[0]]
                local[:, others[1]] = b * size[others[1]]
                faces.append(local)
        local = np.vstack(faces)
    elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        theta, phi = (g.ravel() for g in np.meshgrid(ang, np.linspace(0.0, np.pi, PRIMITIVE_SAMPLES)))
        local = size[0] * np.stack([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)], 1)
    elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        theta, height = (g.ravel() for g in np.meshgrid(ang, u))
        ring = np.stack([size[0] * np.cos(theta), size[0] * np.sin(theta), height * size[1]], 1)
        theta, phi = (g.ravel() for g in np.meshgrid(ang, np.linspace(0.0, np.pi, PRIMITIVE_SAMPLES)))
        ball = size[0] * np.stack([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)], 1)
        local = np.vstack([ring, ball + [0.0, 0.0, size[1]], ball - [0.0, 0.0, size[1]]])
    else:
        raise ValueError(f"no surface sampler for geom type {kind}")
    return pos + local @ rot.T


def mesh_min_x_at(triangles: np.ndarray, z: float) -> float:
    """Smallest x where a horizontal (constant z) line cuts a set of triangles in the x-z plane.

    Args:
        triangles (np.ndarray): (n, 3, 2) triangle corners as (x, z).
        z (float): Line height.

    Returns:
        float: Smallest x of the cut, inf when no triangle reaches z.
    """
    best = np.inf
    for a, b in ((0, 1), (1, 2), (2, 0)):
        za, zb = triangles[:, a, 1], triangles[:, b, 1]
        hit = (np.minimum(za, zb) <= z) & (np.maximum(za, zb) >= z) & (za != zb)
        if hit.any():
            f = (z - za[hit]) / (zb[hit] - za[hit])
            xs = triangles[hit, a, 0] + f * (triangles[hit, b, 0] - triangles[hit, a, 0])
            best = min(best, float(xs.min()))
    return best


def moving_jaw_inner_profile(model: mujoco.MjModel) -> list[tuple[float, float]]:
    """Inner silhouette of the closed moving jaw: how far its face toward the fixed jaw stands off at each depth.

    In the gripper body (URDF gripper_link) frame the moving jaw opens along +x and the jaws point along -z, so depth
    into the jaws is +z from the closing point. At the closed angle every collision geom of the moving jaw is cut at
    depths PROFILE_DEPTH_START_M .. PROFILE_DEPTH_END_M (meshes exactly, primitives by surface samples) and the
    smallest x is kept. mcp_server rotates these points about the gripper pivot to get the opening at any depth
    (mcp_server/jaw_profile.py MOVING_JAW_INNER_PROFILE, which tests/test_tcp.py checks against this).

    Args:
        model (mujoco.MjModel): Compiled model (stock jaws: the profile is relative to their closing point).

    Returns:
        list[tuple[float, float]]: (offset along the opening direction, depth) in m relative to the closing point
        STOCK_CLOSING_POINT_GFL, for the depths where the jaw has material, ordered by depth.
    """
    data = mujoco.MjData(model)
    data.qpos[model.joint(GRIPPER_JOINT).qposadr[0]] = GRIPPER_CLOSED_RAD
    mujoco.mj_forward(model, data)
    body = data.body(GRIPPER_BODY)
    rot = body.xmat.reshape(3, 3)
    tip = gfl_to_gripper(np.array(STOCK_CLOSING_POINT_GFL))
    _, moving = finger_geom_ids(model)
    clouds: list[np.ndarray] = []
    meshes: list[np.ndarray] = []
    for gid in moving:
        if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = int(model.geom_dataid[gid])
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            first, faces = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            world = data.geom_xpos[gid] + model.mesh_vert[start : start + count] @ data.geom_xmat[gid].reshape(3, 3).T
            local = (world - body.xpos) @ rot
            meshes.append(local[model.mesh_face[first : first + faces]][:, :, [0, 2]])
        else:
            clouds.append((geom_surface_points(model, data, gid) - body.xpos) @ rot)
    cloud = np.vstack(clouds)
    out: list[tuple[float, float]] = []
    steps = round((PROFILE_DEPTH_END_M - PROFILE_DEPTH_START_M) / PROFILE_DEPTH_STEP_M)
    for k in range(steps + 1):
        depth = PROFILE_DEPTH_START_M + k * PROFILE_DEPTH_STEP_M
        z = tip[2] + depth
        near = cloud[np.abs(cloud[:, 2] - z) <= PROFILE_BAND_M]
        best = min([mesh_min_x_at(tri, z) for tri in meshes] + [float(near[:, 0].min()) if len(near) else np.inf])
        if np.isfinite(best):
            out.append((round(best - float(tip[0]), 5), round(depth, 5)))
    return out
