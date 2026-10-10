"""Model-driven grid intersection measurement: predict every lattice node with params P, refine on the raw image.

refine.py <params.json> <out_subdir> <search_px> names...
Writes <out_subdir>/<name>.json ({joints, pts:[{uv, label_mm, pred_uv, fit_rms}]}) and _ref.jpg debug images.
"""

import json
import sys

import cv2
import numpy as np
from scipy.ndimage import map_coordinates

sys.path.insert(
    0,
    "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2",
)
from model import *
from fitlib import world

XS = range(200, 701, 50)
YS = range(-150, 151, 50)
ARM_T = np.arange(14, 54, 3.0)
MIN_CONTRAST = 1.0
MAX_LINE_RMS = 1.2


def paper_mask(img):
    bl = cv2.GaussianBlur(img, (21, 21), 0).astype(int)
    m = (((bl[..., 0] - bl[..., 2]) > 15) & (bl.max(2) > 130)).astype(np.uint8)
    return cv2.erode(m, np.ones((5, 5), np.uint8))


def darkness(img):
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32), (3, 3), 0)
    return cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))


def fit_line_arm(bh, p, d, search):
    """Points on the dark line through ~p with direction d (unit); returns (centroid, dir, rms, npos, nneg) or None."""
    n = np.array([-d[1], d[0]])
    offs = np.arange(-search, search + 0.01, 0.5)
    pts = []
    for sgn in (1, -1):
        for t in ARM_T:
            c = p + sgn * t * d
            prof = np.zeros(len(offs))
            for dt in (-2.0, -1.0, 0.0, 1.0, 2.0):
                xy = (c + dt * d)[None, :] + offs[:, None] * n[None, :]
                prof += map_coordinates(bh, [xy[:, 1], xy[:, 0]], order=1, mode="constant", cval=0) / 5
            k = int(np.argmax(prof))
            if prof[k] - np.median(prof) < MIN_CONTRAST or k == 0 or k == len(prof) - 1:
                continue
            a, b, cc = prof[k - 1], prof[k], prof[k + 1]
            den = a - 2 * b + cc
            sub = 0.5 * (a - cc) / den if den != 0 else 0.0
            pts.append((sgn, c + (offs[k] + sub * 0.5) * n))
    if len(pts) < 8:
        return None
    sg = np.array([s for s, _ in pts])
    P = np.array([q for _, q in pts])
    keep = np.ones(len(P), bool)
    for _ in range(3):
        c = P[keep].mean(0)
        _, _, vt = np.linalg.svd(P[keep] - c)
        dd = vt[0]
        nn = np.array([-dd[1], dd[0]])
        r = (P - c) @ nn
        keep = np.abs(r - np.median(r[keep])) < max(0.8, 3.0 * 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep]))))
    if (sg[keep] > 0).sum() < 4 or (sg[keep] < 0).sum() < 4:
        return None
    r = (P[keep] - c) @ nn
    return c, dd, float(np.sqrt(np.mean(r**2)))


def measure(img, joints, P, search):
    bh = darkness(img)
    pm = paper_mask(img)
    t = t_arm_optical(joints, P)
    out = []
    for X in XS:
        for Y in YS:
            q = np.array([bl_to_arm((X, Y)), bl_to_arm((X + 1, Y)), bl_to_arm((X, Y + 1))])
            q = world(P, q)
            uv, z = project(q, t, P["f"], P["k1"])
            if (z <= 0.02).any():
                continue
            inv = np.linalg.inv(t)
            pc = inv[:3, :3] @ q[0] + inv[:3, 3]
            if 1 + 3 * P["k1"] * ((pc[0] / pc[2]) ** 2 + (pc[1] / pc[2]) ** 2) < 0.5:
                continue
            p = uv[0]
            if not (25 < p[0] < W - 25 and 25 < p[1] < H - 25):
                continue
            if pm[max(0, int(p[1]) - 30) : int(p[1]) + 31, max(0, int(p[0]) - 30) : int(p[0]) + 31].mean() < 0.9:
                continue
            dX = uv[1] - p
            dY = uv[2] - p
            if np.linalg.norm(dX) < 0.4 or np.linalg.norm(dY) < 0.4:  # < 20 px per 50 mm cell
                continue
            l1 = fit_line_arm(bh, p, dX / np.linalg.norm(dX), search)  # line of constant Y (along X)
            l2 = fit_line_arm(bh, p, dY / np.linalg.norm(dY), search)
            if l1 is None or l2 is None or max(l1[2], l2[2]) > MAX_LINE_RMS:
                continue
            A = np.stack([l1[1], -l2[1]], 1)
            if abs(np.linalg.det(A)) < 0.3:
                continue
            s, _ = np.linalg.solve(A, l2[0] - l1[0])
            m = l1[0] + s * l1[1]
            if np.linalg.norm(m - p) > search:
                continue
            out.append(
                {
                    "uv": m.round(3).tolist(),
                    "label_mm": [X, Y],
                    "pred_uv": p.round(2).tolist(),
                    "fit_rms": round(max(l1[2], l2[2]), 3),
                    "ok": True,
                }
            )
    return out


if __name__ == "__main__":
    P = json.load(open(sys.argv[1]))
    sub = D / sys.argv[2]
    sub.mkdir(exist_ok=True)
    search = float(sys.argv[3])
    for n in sys.argv[4:]:
        img = cv2.imread(str(D / "caps" / f"{n}.jpg"))
        joints = json.load(open(D / "caps" / f"{n}.state1.json"))["positions"]
        pts = measure(img, joints, P, search)
        json.dump({"name": n, "joints": joints, "pts": pts}, open(sub / f"{n}.json", "w"), indent=1)
        dbg = img.copy()
        for x in pts:
            u, v = x["uv"]
            pu, pv = x["pred_uv"]
            cv2.circle(dbg, (int(round(u)), int(round(v))), 4, (0, 200, 0), 1)
            cv2.line(dbg, (int(pu), int(pv)), (int(u), int(v)), (0, 0, 255), 1)
            cv2.putText(
                dbg,
                f"{x['label_mm'][0]},{x['label_mm'][1]}",
                (int(u) + 4, int(v) + 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 0, 255),
                1,
            )
        cv2.imwrite(str(sub / f"{n}_ref.jpg"), dbg)
        d = np.array([np.subtract(x["uv"], x["pred_uv"]) for x in pts]) if pts else np.zeros((0, 2))
        print(n, len(pts), "mean |uv-pred|", np.round(np.linalg.norm(d, axis=1).mean(), 2) if len(d) else "-")
