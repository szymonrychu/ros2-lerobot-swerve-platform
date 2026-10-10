"""Shared kinematic + camera model for the 2026-10-10 live calibration."""

from pathlib import Path

import numpy as np
import yaml
from ros2_common.camera_geometry import optical_from_mount
from scipy.spatial.transform import Rotation

from mcp_server.ik import ARM_CHAIN_JOINTS, ArmKinematics

REPO = Path("/Users/szymonri/Documents/ros2-lerobot-sverve-platform")
D = Path(
    "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2"
)
MOUNT_BL = np.array([0.0592, -0.05, 0.104])
FLOOR_Z = -0.104
JOINTS = list(ARM_CHAIN_JOINTS)

gv = yaml.safe_load((REPO / "ansible/group_vars/client.yml").read_text())
_c = next(n for n in gv["ros2_nodes"] if n["name"] == "mcp_server")["config"]
CFG = yaml.safe_load(_c) if isinstance(_c, str) else _c
URDF = REPO / CFG["arm"]["urdf_path"]
KIN0 = ArmKinematics(URDF, margin=0.0)  # zero offsets, zero tool
CUR = {
    "offsets": dict(CFG["arm"]["joint_offsets_rad"]),
    "mount": {k: CFG["cameras"]["gripper"]["mount"][k] for k in ("x", "y", "z", "roll", "pitch", "yaw")},
    "f": 367.83,
    "k1": -0.1523,
    "tool": [CFG["arm"]["tool_offset_m"][k] for k in ("x", "y", "z")],
}
W, H, CX, CY = 640, 480, 320.0, 240.0


def frames(raw: dict, offsets: dict) -> dict:
    """Link frames for raw measured joints + offsets (urdf = raw + off)."""
    urdf = {j: raw[j] + offsets.get(j, 0.0) for j in JOINTS}
    fk = KIN0.chain.forward_kinematics(KIN0.urdf_vector(urdf), full_kinematics=True)
    return {"gripper_link": np.array(fk[KIN0.link_index["gripper_link"]]), "tool": np.array(fk[-1])}


def mount_matrix(m: dict) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", [m["roll"], m["pitch"], m["yaw"]]).as_matrix()
    t[:3, 3] = [m["x"], m["y"], m["z"]]
    return t


def t_arm_optical(raw: dict, p: dict) -> np.ndarray:
    return frames(raw, p["offsets"])["gripper_link"] @ optical_from_mount(mount_matrix(p["mount"]))


def project(pts_arm: np.ndarray, t_opt: np.ndarray, f: float, k1: float) -> tuple[np.ndarray, np.ndarray]:
    """pts (N,3) arm frame -> (N,2) pixels, depth (N,)."""
    inv = np.linalg.inv(t_opt)
    pc = (inv[:3, :3] @ pts_arm.T).T + inv[:3, 3]
    z = pc[:, 2]
    xn, yn = pc[:, 0] / z, pc[:, 1] / z
    r2 = xn * xn + yn * yn
    d = 1 + k1 * r2
    return np.stack([f * xn * d + CX, f * yn * d + CY], 1), z


def tool_point(raw: dict, p: dict) -> np.ndarray:
    t = frames(raw, p["offsets"])["tool"]
    return t[:3, 3] + t[:3, :3] @ np.array(p["tool"])


def bl_to_arm(xy_mm) -> np.ndarray:
    xy = np.asarray(xy_mm, float) / 1000.0
    return np.array([xy[0] - MOUNT_BL[0], xy[1] - MOUNT_BL[1], FLOOR_Z])


def grid_points_arm(xs=range(200, 701, 50), ys=range(-200, 201, 50)):
    lab, pts = [], []
    for X in xs:
        for Y in ys:
            lab.append((X, Y))
            pts.append(bl_to_arm((X, Y)))
    return lab, np.array(pts)


def backproject(uv: np.ndarray, t_opt: np.ndarray, f: float, k1: float, z: float = FLOOR_Z) -> np.ndarray:
    """raw pixels (N,2) -> floor points (N,3) arm frame (k1 inverted iteratively)."""
    d = (np.asarray(uv, float) - [CX, CY]) / f
    n = d.copy()
    for _ in range(30):
        n = d / (1 + k1 * (n ** 2).sum(1, keepdims=True))
    rays = np.concatenate([n, np.ones((len(n), 1))], 1) @ t_opt[:3, :3].T
    o = t_opt[:3, 3]
    s = (z - o[2]) / rays[:, 2]
    return o + s[:, None] * rays


def arm_to_bl_mm(p: np.ndarray) -> np.ndarray:
    return (np.asarray(p)[..., :2] + MOUNT_BL[:2]) * 1000.0
