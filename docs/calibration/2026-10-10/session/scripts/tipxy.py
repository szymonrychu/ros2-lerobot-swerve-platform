"""Physical tip floor position from a near-floor gripper image: trace labelled grid lines, fit a line-constrained
homography (undistorted pixels -> base_link mm), map the tip pixel. Also returns the measured intersections."""

import json
import sys

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

sys.path.insert(
    0,
    "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2",
)
import refine as R
from model import CX, CY, D

TIP_UV = (384.0, 286.0)
# (axis, value_mm, p1, p2) per image; read from the blackhat views and labels
LINES = {
    "t1": [
        ("X", 350, (195, 10), (182, 470)),
        ("X", 300, (468, 8), (475, 92)),
        ("Y", -50, (5, 128), (425, 153)),
        ("Y", 0, (5, 472), (450, 428)),
    ],
    "t2": [
        ("X", 300, (298, 5), (152, 475)),
        ("Y", 50, (5, 262), (415, 378)),
        ("Y", 0, (290, 10), (460, 100)),
        ("X", 350, (60, 18), (22, 108)),
    ],
    "t7": [
        ("X", 300, (198, 5), (176, 475)),
        ("Y", -50, (5, 85), (435, 135)),
        ("Y", 0, (5, 435), (440, 405)),
        ("X", 250, (475, 5), (478, 90)),
    ],
    "t8": [
        ("X", 300, (62, 5), (183, 475)),
        ("Y", -100, (5, 238), (430, 145)),
        ("X", 250, (376, 10), (410, 190)),
        ("Y", -50, (312, 478), (450, 420)),
    ],
    "t9": [("X", 350, (262, 35), (186, 475)), ("Y", 0, (130, 2), (445, 115)), ("Y", 50, (5, 337), (435, 400))],
}


def trace(bh, p1, p2, search=10.0, step=4.0):
    p1, p2 = np.array(p1, float), np.array(p2, float)
    L = np.linalg.norm(p2 - p1)
    d = (p2 - p1) / L
    n = np.array([-d[1], d[0]])
    offs = np.arange(-search, search + 0.01, 0.5)
    pts = []
    for t in np.arange(0, L, step):
        c = p1 + t * d
        prof = np.zeros(len(offs))
        for dt in (-1.5, 0, 1.5):
            xy = (c + dt * d)[None] + offs[:, None] * n[None]
            prof += map_coordinates(bh, [xy[:, 1], xy[:, 0]], order=1, mode="constant", cval=0)
        k = int(np.argmax(prof))
        if 0 < k < len(prof) - 1 and prof[k] / 3 - np.median(prof) / 3 > 2.0:
            pts.append((t, offs[k]))
    pts = np.array(pts)
    keep = np.ones(len(pts), bool)
    for _ in range(4):
        c2 = np.polyfit(pts[keep, 0], pts[keep, 1], 2)
        r = pts[:, 1] - np.polyval(c2, pts[:, 0])
        keep = np.abs(r) < max(0.8, 3 * 1.4826 * np.median(np.abs(r[keep])))
    P = p1[None] + pts[keep, 0:1] * d[None] + np.polyval(c2, pts[keep, 0])[:, None] * n[None]
    return P, float(np.sqrt(np.mean(r[keep] ** 2)))


def undist(uv, f, k1):
    uv = np.asarray(uv, float).reshape(-1, 1, 2)
    return cv2.undistortPoints(
        uv, np.array([[f, 0, CX], [0, f, CY], [0, 0, 1.0]]), np.array([k1, 0, 0, 0, 0.0])
    ).reshape(-1, 2)


def happly(h, p):
    Hm = np.append(h, 1.0).reshape(3, 3)
    q = np.c_[p, np.ones(len(p))] @ Hm.T
    return q[:, :2] / q[:, 2:3]


def measure(name, img_path, f, k1, tip=TIP_UV):
    img = cv2.imread(str(img_path))
    bh = R.darkness(img)
    traced = []
    for ax, val, p1, p2 in LINES[name]:
        P, rr = trace(bh, p1, p2)
        traced.append((ax, val, P, undist(P, f, k1), rr))

    # init: straight-line intersections in undistorted coords
    def lfit(Q):
        c = Q.mean(0)
        _, _, vt = np.linalg.svd(Q - c)
        return c, vt[0]

    xs = [t for t in traced if t[0] == "X"]
    ys = [t for t in traced if t[0] == "Y"]
    src, dst, inter = [], [], []
    for a in xs:
        for b in ys:
            (c1, d1), (c2, d2) = lfit(a[3]), lfit(b[3])
            s, _ = np.linalg.solve(np.stack([d1, -d2], 1), c2 - c1)
            src.append(c1 + s * d1)
            dst.append((a[1], b[1]))
    src, dst = np.array(src), np.array(dst, float)
    if len(src) >= 4:
        H0, _ = cv2.findHomography(src, dst, 0)
    else:  # affine init from 3 points + extra
        A = cv2.getAffineTransform(src[:3].astype(np.float32), dst[:3].astype(np.float32)) if len(src) >= 3 else None
        H0 = np.vstack([A, [0, 0, 1]]) if A is not None else None
    if H0 is None:
        return None
    h0 = (H0 / H0[2, 2]).ravel()[:8]

    def res(h):
        out = []
        for ax, val, P, U, _ in traced:
            m = happly(h, U)
            out.append(m[:, 0 if ax == "X" else 1] - val)
        return np.concatenate(out)

    sol = least_squares(res, h0)
    rms = float(np.sqrt(np.mean(sol.fun**2)))
    tipmm = happly(sol.x, undist([tip], f, k1))[0]
    # measured intersections (raw pixels) by curve intersection: nearest traced points
    for a in xs:
        for b in ys:
            da = np.linalg.norm(a[2][:, None] - b[2][None], axis=2)
            i, j = np.unravel_index(np.argmin(da), da.shape)
            if da[i, j] < 3.0:
                inter.append({"label_mm": [a[1], b[1]], "uv": ((a[2][i] + b[2][j]) / 2).round(2).tolist()})
    # local scale (mm per px) at the tip
    e = happly(sol.x, undist([tip, (tip[0] + 1, tip[1]), (tip[0], tip[1] + 1)], f, k1))
    scale = float(np.linalg.norm(e[1] - e[0]) + np.linalg.norm(e[2] - e[0])) / 2
    return {
        "tip_bl_mm": tipmm.round(2).tolist(),
        "line_rms_mm": round(rms, 3),
        "mm_per_px": round(scale, 4),
        "line_trace_rms_px": [round(t[4], 2) for t in traced],
        "intersections": inter,
        "lines": [(t[0], t[1], len(t[2])) for t in traced],
    }


if __name__ == "__main__":
    P = json.load(open(sys.argv[1]))
    for n in sys.argv[2:]:
        r = measure(n, D / "touch" / f"{n}.last_free.jpg", P["f"], P["k1"])
        print(n, json.dumps(r))
