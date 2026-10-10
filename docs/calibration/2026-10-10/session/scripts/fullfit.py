"""Joint fit: camera samples (far field, refined intersections) + near-floor line points + tip touches.

Variants are run from main; everything is evaluated on held-out images / touches and on the user observations.
"""

import json
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(
    0,
    "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2",
)
import refine as R
import tipxy as T
from fitlib import *

from mcp_server.ik import ArmKinematics

ALL = [f"c{i:02d}" for i in range(1, 21)]
HOLD_IMG = ["c03", "c09", "c15", "c19"]
TOUCH_XY = {"t1": None, "t2": None, "t7": None, "t8": None}
TOUCH_Z = ["t1", "t2", "t5", "t7", "t8", "t9"]
HOLD_TOUCH = ["t7"]
SIG_PX, SIG_TIP_MM, SIG_LINE_MM = 3.0, 1.5, 1.0
LINE_SUB = 12


def load_state(p):
    return json.load(open(p))["positions"]


def build_lines(P):
    """Near-floor traced line points per touch image: [{name, joints, uv (N,2), axis, val}]."""
    out = []
    for n in T.LINES:
        img = cv2.imread(str(D / "touch" / f"{n}.last_free.jpg"))
        bh = R.darkness(img)
        j = load_state(D / "touch" / f"{n}.last_free.json")
        for ax, val, p1, p2 in T.LINES[n]:
            Pp, _ = T.trace(bh, p1, p2)
            idx = np.linspace(0, len(Pp) - 1, min(LINE_SUB, len(Pp))).astype(int)
            out.append({"name": n, "joints": j, "uv": Pp[idx], "axis": 0 if ax == "X" else 1, "val": val})
    return out


def label_frame(P, g_arm):
    e = P.get("extra", [0.0] * 5)
    return (g_arm[:, :2] - np.array([e[3], e[4]]) + MOUNT_BL[:2]) * 1000.0


def line_res(P, lines):
    out = []
    for L in lines:
        g = backproject(L["uv"], t_arm_optical(L["joints"], P), P["f"], P["k1"])
        out.append(label_frame(P, g)[:, L["axis"]] - L["val"])
    return np.concatenate(out) if out else np.zeros(0)


def touch_res(P, touches):
    """touches: [{joints, xy_mm (label frame) or None}] -> mm residuals (x, y, z)."""
    e = P.get("extra", [0.0] * 5)
    out = []
    for t in touches:
        p = tool_point(t["joints"], P)
        out.append([(p[2] - FLOOR_Z) * 1000.0])
        if t["xy"] is not None:
            lab = (p[:2] - np.array([e[3], e[4]]) + MOUNT_BL[:2]) * 1000.0
            out.append(lab - np.array(t["xy"]))
    return np.concatenate(out) if out else np.zeros(0)


def run_fit(P0, free, imgs, lines, touches):
    v0 = to_vec(P0)
    idx = [NAMES.index(n) for n in free]

    def fun(x):
        v = v0.copy()
        v[idx] = x
        P = to_params(v)
        return np.concatenate(
            [cam_residuals(P, imgs) / SIG_PX, line_res(P, lines) / SIG_LINE_MM, touch_res(P, touches) / SIG_TIP_MM]
        )

    XS = {"off": 0.05, "m_x": 0.01, "m_y": 0.01, "m_z": 0.01, "m_r": 0.05, "m_p": 0.05, "m_y": 0.01, "f": 10.0, "k1": 0.05, "to": 0.005, "ti": 0.02, "fl": 0.01, "ba": 0.01}
    xs = np.array([XS.get(n[:3], XS.get(n[:2], 0.05)) if n not in ("m_yaw", "m_roll", "m_pitch") else 0.05 for n in free])
    r = least_squares(fun, v0[idx], x_scale=xs, max_nfev=400)
    print("   fit:", r.status, r.nfev, round(float(r.cost), 2))
    v = v0.copy()
    v[idx] = r.x
    Pf = to_params(v)
    Pf["_cost"] = float(r.cost)
    return Pf


def report(P, imgs_tr, imgs_ho, lines, touches_tr, touches_ho, label):
    def tstats(ts):
        if not ts:
            return "-"
        rows = []
        for t in ts:
            r = touch_res(P, [t])
            rows.append(f"{t['name']}: z {r[0]:+.1f}" + (f" xy {r[1]:+.1f},{r[2]:+.1f}" if len(r) > 1 else ""))
        return "; ".join(rows)

    fe_tr, fe_ho = floor_err_mm(P, imgs_tr), floor_err_mm(P, imgs_ho)
    # floor error uses nominal label->arm; correct for registration
    return {
        "label": label,
        "train_px": round(px_rms(P, imgs_tr), 2),
        "hold_px": round(px_rms(P, imgs_ho), 2),
        "train_floor_mm": round(rms(floor_err_reg(P, imgs_tr)), 2),
        "hold_floor_mm": round(rms(floor_err_reg(P, imgs_ho)), 2),
        "line_rms_mm": round(rms(line_res(P, lines)), 2),
        "touch_train": tstats(touches_tr),
        "touch_hold": tstats(touches_ho),
    }


def floor_err_reg(P, imgs):
    out = []
    for im in imgs:
        g = backproject(im["uv"], t_arm_optical(im["joints"], P), P["f"], P["k1"])
        lab = label_frame(P, g)
        gl = (im["ground"][:, :2] + MOUNT_BL[:2]) * 1000.0
        out.append(np.linalg.norm(lab - gl, axis=1))
    return np.concatenate(out) if out else np.zeros(0)


def observations(P):
    """(a) home pose tip height above floor (mm); (c) recovered pose tip in base_link (mm)."""
    home = {
        "shoulder_pan": -0.0138,
        "shoulder_lift": -1.3622,
        "elbow_flex": 1.5769,
        "wrist_flex": -0.0752,
        "wrist_roll": -1.6245,
    }
    a = (tool_point(home, P)[2] - FLOOR_Z) * 1000.0
    c = None
    if OBS_C_JOINTS is not None:
        p = tool_point(OBS_C_JOINTS, P)
        c = ((p[:2] + MOUNT_BL[:2]) * 1000.0).round(1).tolist()  # physical base_link (mount as measured)
    return round(float(a), 1), c


def recover_obs_c():
    kin = ArmKinematics(URDF, margin=0.0, joint_offsets=CUR["offsets"], tool_offset=tuple(CUR["tool"]))
    seed = {"shoulder_pan": 0.5, "shoulder_lift": 0.0, "elbow_flex": 0.5, "wrist_flex": 1.0, "wrist_roll": -1.6}
    try:
        return kin.inverse(0.0949, 0.1785, 0.1055, 1.25, seed)
    except Exception as exc:  # noqa: BLE001
        print("obs (c) not recoverable:", exc)
        return None


OBS_C_JOINTS = None

if __name__ == "__main__":
    OBS_C_JOINTS = recover_obs_c()
    if OBS_C_JOINTS:
        OBS_C_JOINTS = {k: float(v) for k, v in OBS_C_JOINTS.items() if k in JOINTS}
    tipm = {}
    for n in TOUCH_XY:
        tipm[n] = T.measure(n, D / "touch" / f"{n}.last_free.jpg", 380.54, -0.1469)["tip_bl_mm"]
    touches = [
        {"name": n, "joints": load_state(D / "touch" / f"{n}.last_free.json"), "xy": tipm.get(n)} for n in TOUCH_Z
    ]
    tr_t = [t for t in touches if t["name"] not in HOLD_TOUCH]
    ho_t = [t for t in touches if t["name"] in HOLD_TOUCH]
    imgs_all = load_images(ALL, src="r5", key="pts")
    imgs_tr = [i for i in imgs_all if i["name"] not in HOLD_IMG]
    imgs_ho = [i for i in imgs_all if i["name"] in HOLD_IMG]
    P_start = json.load(open(D / "P_r6.json"))
    lines_all = build_lines(P_start)
    lines_tr = [L for L in lines_all if L["name"] not in HOLD_TOUCH]
    print(
        "points: train",
        sum(len(i["uv"]) for i in imgs_tr),
        "hold",
        sum(len(i["uv"]) for i in imgs_ho),
        "line pts",
        sum(len(L["uv"]) for L in lines_all),
        "touches",
        len(touches),
        "tip xy",
        tipm,
    )
    OFF = [n for n in NAMES if n.startswith("off_") and n != "off_wrist_roll"]  # roll offset is degenerate with mount + tool
    MNT = [n for n in NAMES if n.startswith("m_")]
    TOOL = ["tool_x", "tool_y", "tool_z"]
    REG = ["base_dx", "base_dy"]
    cur = dict(CUR, extra=[0.0] * 5)
    results = {}
    print(json.dumps(report(cur, imgs_tr, imgs_ho, lines_all, tr_t, ho_t, "CURRENT (deployed)")), "obs", observations(cur))
    r6 = json.load(open(D / "P_r6.json")); r6["tool"] = list(CUR["tool"])
    r6["mount"]["yaw"] += r6["offsets"]["wrist_roll"] - CUR["offsets"]["wrist_roll"]
    r6["offsets"]["wrist_roll"] = CUR["offsets"]["wrist_roll"]
    r6_noreg = dict(r6, extra=[0.0] * 5)
    def go(name, starts, free, use_lines, use_touch):
        best = None
        for P0 in starts:
            P0 = {k: v for k, v in P0.items() if k != "_cost"}
            Pf = run_fit(P0, free, imgs_tr, lines_tr if use_lines else [], tr_t if use_touch else [])
            if best is None or Pf["_cost"] < best["_cost"]:
                best = Pf
        rep = report(best, imgs_tr, imgs_ho, lines_all, tr_t, ho_t, name)
        print(json.dumps(rep), "obs", observations(best), "cost", round(best["_cost"], 1))
        results[name] = {"params": best, "report": rep, "obs": observations(best)}
        return best
    A = go("A cam only, reg fixed", [r6_noreg], OFF + MNT + ["f", "k1"], False, False)
    C = go("C cam only, reg free", [r6], OFF + MNT + ["f", "k1"] + REG, False, False)
    D_ = go("D cam+lines+touch, reg free", [C], OFF + MNT + ["f", "k1"] + TOOL + REG, True, True)
    B = go("B cam+lines+touch, reg fixed", [A, dict(D_, extra=[0.0] * 5)], OFF + MNT + ["f", "k1"] + TOOL, True, True)
    E = go("E D + floor_dz free (check)", [D_], OFF + MNT + ["f", "k1"] + TOOL + REG + ["floor_dz"], True, True)
    F = go("F D + arm tilt free (check)", [D_], OFF + MNT + ["f", "k1"] + TOOL + REG + ["tilt_x", "tilt_y"], True, True)
    json.dump(
        {"results": results, "tip_xy_mm": tipm, "obs_c_joints": OBS_C_JOINTS},
        open(D / "fit_results.json", "w"),
        indent=1,
    )
