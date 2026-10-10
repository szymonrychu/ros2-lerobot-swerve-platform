"""process.py <img> <joints_json_file> <out_prefix> [sx sy] [params.json]: detect, index the lattice, label, write <out>.json/_lab.jpg"""
import sys, json, cv2, numpy as np
from collections import deque
sys.path.insert(0, "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2")
from model import *
from detect import detect

ANG_TOL = np.deg2rad(14)


def lattice_index(U, dirs):
    """BFS integer indices (ia along dirs[0], ib along dirs[1]) over undistorted points U."""
    n = len(U)
    idx = {}
    start = int(np.argmin(np.linalg.norm(U - [CX, CY], axis=1)))
    idx[start] = (0, 0)
    q = deque([start])
    while q:
        i = q.popleft()
        for k, d in enumerate(dirs):
            for sgn in (1, -1):
                best, bd = None, 1e9
                for j in range(n):
                    if j == i: continue
                    v = U[j] - U[i]; L = np.linalg.norm(v)
                    if L < 30 or L > 260: continue
                    if np.arccos(np.clip(sgn * v @ d / L, -1, 1)) < ANG_TOL and L < bd:
                        best, bd = j, L
                if best is not None and best not in idx:
                    a, b = idx[i]
                    idx[best] = (a + sgn, b) if k == 0 else (a, b + sgn)
                    q.append(best)
    return idx, start


def label(raw_uv, U, dirs, joints, P, shift=(0, 0)):
    idx, start = lattice_index(U, dirs)
    ids = sorted(idx)
    g = arm_to_bl_mm(backproject(raw_uv, t_arm_optical(joints, P), P["f"], P["k1"]))
    A = np.array([[idx[i][0], idx[i][1], 1.0] for i in ids])
    coef, *_ = np.linalg.lstsq(A, g[ids], rcond=None)  # g ~ ia*c0 + ib*c1 + c2
    # which ground axis each index direction follows
    c0, c1 = coef[0], coef[1]
    if abs(c0[0]) * abs(c1[1]) >= abs(c0[1]) * abs(c1[0]):
        ax = {"X": (0, np.sign(c0[0])), "Y": (1, np.sign(c1[1]))}
    else:
        ax = {"X": (1, np.sign(c1[0])), "Y": (0, np.sign(c0[1]))}
    ref = np.round(g[start] / 50.0) * 50.0 + np.array(shift)
    labs = {}
    for i in ids:
        a = idx[i]
        X = ref[0] + 50 * ax["X"][1] * a[ax["X"][0]]
        Y = ref[1] + 50 * ax["Y"][1] * a[ax["Y"][0]]
        labs[i] = (int(X), int(Y))
    return labs, g, coef


if __name__ == "__main__":
    img_p, jf, out = sys.argv[1:4]
    shift = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else (0, 0)
    P = json.load(open(sys.argv[6])) if len(sys.argv) > 6 else CUR
    raw = json.load(open(jf)); raw = raw.get("positions", raw)
    img = cv2.imread(img_p)
    uv, U, dirs = detect(img)
    labs, g, coef = label(uv, U, dirs, raw, P, shift)
    print(f"{len(uv)} dets, {len(labs)} indexed; index steps in ground mm: {coef[0].round(1)}, {coef[1].round(1)}")
    keep = []
    for i in sorted(labs):
        L = labs[i]
        ok = abs(L[1]) <= 150 and 200 <= L[0] <= 700
        keep.append({"uv": uv[i].round(2).tolist(), "label_mm": list(L), "ok": ok, "pred_bl_mm": g[i].round(1).tolist()})
    for i in range(len(uv)):
        c = (0, 160, 0) if i in labs else (0, 0, 255)
        cv2.circle(img, (int(uv[i][0]), int(uv[i][1])), 5, c, 1)
        if i in labs:
            cv2.putText(img, f"{labs[i][0]},{labs[i][1]}", (int(uv[i][0]) + 5, int(uv[i][1]) + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    json.dump({"image": img_p, "joints": raw, "dets": keep, "shift": shift}, open(out + ".json", "w"), indent=1)
    cv2.imwrite(out + "_lab.jpg", img)
    for k in keep: print(k["label_mm"], k["uv"], k["pred_bl_mm"])
