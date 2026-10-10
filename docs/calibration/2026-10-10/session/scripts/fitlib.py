"""Joint fit of joint offsets, gripper camera mount, intrinsics (f, k1) and tool point."""

import json
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(
    0,
    "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2",
)
from model import *

NAMES = (
    [f"off_{j}" for j in JOINTS]
    + [f"m_{k}" for k in ("x", "y", "z", "roll", "pitch", "yaw")]
    + ["f", "k1"]
    + ["tool_x", "tool_y", "tool_z"] + ["tilt_x", "tilt_y", "floor_dz", "base_dx", "base_dy"]
)


def to_vec(P: dict) -> np.ndarray:
    return np.array(
        [P["offsets"][j] for j in JOINTS]
        + [P["mount"][k] for k in ("x", "y", "z", "roll", "pitch", "yaw")]
        + [P["f"], P["k1"]]
        + list(P["tool"])
        + list(P.get("extra", [0.0] * 5))
    )


def to_params(v: np.ndarray) -> dict:
    return {
        "offsets": {j: float(v[i]) for i, j in enumerate(JOINTS)},
        "mount": {k: float(v[5 + i]) for i, k in enumerate(("x", "y", "z", "roll", "pitch", "yaw"))},
        "f": float(v[11]),
        "k1": float(v[12]),
        "tool": [float(x) for x in v[13:16]],
        "extra": [float(x) for x in v[16:21]],
    }


def load_images(names, src="caps", key="dets"):
    out = []
    for n in names:
        d = json.load(open(D / src / f"{n}.json"))
        pts = [(x["uv"], x["label_mm"]) for x in d[key] if x.get("ok", True)]
        if not pts:
            continue
        uv = np.array([p[0] for p in pts], float)
        g = np.array([bl_to_arm(p[1]) for p in pts])
        out.append({"name": n, "joints": d["joints"], "uv": uv, "ground": g, "labels": [p[1] for p in pts]})
    return out


def world(P: dict, g: np.ndarray) -> np.ndarray:
    """Floor points (nominal arm frame) -> arm frame with base tilt / floor offset / base shift."""
    e = P.get("extra", [0.0] * 5)
    if not any(e):
        return g
    from scipy.spatial.transform import Rotation as Rr
    c = np.array([0.0, 0.0, FLOOR_Z])
    R = Rr.from_euler("xy", [e[0], e[1]]).as_matrix()
    return (g - c) @ R.T + c + np.array([e[3], e[4], e[2]])


def cam_residuals(P: dict, imgs) -> np.ndarray:
    res = []
    for im in imgs:
        uv, z = project(world(P, im["ground"]), t_arm_optical(im["joints"], P), P["f"], P["k1"])
        r = uv - im["uv"]
        r[z <= 0.01] = 500.0
        res.append(r.ravel())
    return np.concatenate(res) if res else np.zeros(0)


def floor_err_mm(P: dict, imgs) -> np.ndarray:
    """Per-point floor distance (mm) between backprojected pixel and the labelled point."""
    out = []
    for im in imgs:
        g = backproject(im["uv"], t_arm_optical(im["joints"], P), P["f"], P["k1"])
        out.append(np.linalg.norm(g[:, :2] - im["ground"][:, :2], axis=1) * 1000)
    return np.concatenate(out) if out else np.zeros(0)


def tip_residuals(P: dict, touches, w_mm: float) -> np.ndarray:
    """touches: [{joints, target_arm (3,) known tip position (x,y,z) or with nan for unknown components}] -> mm/w."""
    res = []
    for t in touches:
        p = tool_point(t["joints"], P)
        tgt = np.array(t["tip_arm"], float)
        m = ~np.isnan(tgt)
        res.append((p[m] - tgt[m]) * 1000.0 / w_mm)
    return np.concatenate(res) if res else np.zeros(0)


def fit(P0: dict, imgs, free: list, touches=(), w_px: float = 1.0, w_tip_mm: float = 1.0, loss="soft_l1", f_scale=3.0):
    v0 = to_vec(P0)
    idx = [NAMES.index(n) for n in free]

    def fun(x):
        v = v0.copy()
        v[idx] = x
        P = to_params(v)
        return np.concatenate([cam_residuals(P, imgs) / w_px, tip_residuals(P, touches, w_tip_mm)])

    r = least_squares(fun, v0[idx], loss=loss, f_scale=f_scale, x_scale=0.05)
    v = v0.copy()
    v[idx] = r.x
    return to_params(v), r


def rms(a):
    a = np.asarray(a)
    return float(np.sqrt(np.mean(a**2))) if len(a) else float("nan")


def px_rms(P, imgs):
    r = cam_residuals(P, imgs).reshape(-1, 2)
    return rms(np.linalg.norm(r, axis=1))
