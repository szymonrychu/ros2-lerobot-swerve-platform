"""Grid intersection detection on a gripper image.

detect(img) -> (N,2) raw-pixel intersections. Undistorts with the current intrinsics so the lines are straight,
finds dark thin lines (blackhat + Hough), clusters them into two families, intersects, then refines each
intersection locally on the raw image.
"""

import sys

import cv2
import numpy as np

sys.path.insert(
    0,
    "/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad/calib2",
)
from model import CUR, CX, CY, H, W

K = np.array([[CUR["f"], 0, CX], [0, CUR["f"], CY], [0, 0, 1.0]])
BH_T = 6
HOUGH_T = 90
DIST = np.array([CUR["k1"], 0, 0, 0, 0.0])


def line_mask(gray: np.ndarray) -> np.ndarray:
    bh = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
    m = ((bh >= BH_T) * 255).astype(np.uint8)
    # paper mask: bright regions only (exclude jaws / background)
    paper = PAPER_MASK
    paper = cv2.erode(paper, np.ones((15, 15), np.uint8))
    return m * paper


def hough_lines(mask: np.ndarray):
    lines = cv2.HoughLines(mask, 1, np.pi / 360, HOUGH_T)
    return [] if lines is None else [tuple(l[0]) for l in lines]


def cluster_lines(lines, rho_tol=25.0, th_tol=np.deg2rad(6)):
    """Merge near-duplicate (rho, theta) lines; returns list of (rho, theta, votes-order)."""
    out = []
    for rho, th in lines:
        if rho < 0:
            rho, th = -rho, th - np.pi
        merged = False
        for o in out:
            dth = abs(np.arctan2(np.sin(th - o[1]), np.cos(th - o[1])))
            if dth < th_tol and abs(rho - o[0]) < rho_tol:
                merged = True
                break
        if not merged:
            out.append((rho, th))
    return out


ARM_MIN, ARM_MAX, BAND, MIN_SUPPORT = 10.0, 70.0, 3.0, 30


def seek_thetas(x, y):
    return []


def refine(p0, near, fam):
    """Refit both lines locally around p0 from mask pixels; None when either line lacks support."""
    if len(near) == 0:
        return None
    lines = []
    for f in fam:
        # the family line passing closest to p0
        best = min(f, key=lambda rt: abs(p0[0] * np.cos(rt[1]) + p0[1] * np.sin(rt[1]) - rt[0]))
        n = np.array([np.cos(best[1]), np.sin(best[1])])
        d = np.array([-n[1], n[0]])
        rel = near - p0
        along, perp = rel @ d, rel @ n
        for _ in range(2):
            sel = (np.abs(along) > ARM_MIN) & (np.abs(along) < ARM_MAX) & (np.abs(perp) < BAND * 2)
            if sel.sum() < MIN_SUPPORT or (along[sel] > 0).sum() < 8 or (along[sel] < 0).sum() < 8:
                return None
            a, b = np.polyfit(along[sel], perp[sel], 1)
            perp = perp - (a * along + b)
        sel = (np.abs(along) > ARM_MIN) & (np.abs(along) < ARM_MAX) & (np.abs(perp) < BAND)
        pts = near[sel]
        c = pts.mean(0)
        _, _, vt = np.linalg.svd(pts - c)
        lines.append((c, vt[0]))
    (c1, d1), (c2, d2) = lines
    A = np.stack([d1, -d2], 1)
    if abs(np.linalg.det(A)) < 0.3:
        return None
    s_, _ = np.linalg.solve(A, c2 - c1)
    return c1 + s_ * d1


def detect(img: np.ndarray, debug: str | None = None) -> np.ndarray:
    und = cv2.undistort(img, K, DIST)
    gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    global PAPER_MASK
    bl = cv2.GaussianBlur(und, (21, 21), 0).astype(int)
    PAPER_MASK = (((bl[..., 0] - bl[..., 2]) > 15) & (bl.max(2) > 130)).astype(np.uint8)
    mask = line_mask(gray)
    lines = cluster_lines(hough_lines(mask))
    if len(lines) < 2:
        return np.zeros((0, 2)), np.zeros((0, 2)), None
    # two families by angle (mod pi)
    ang = np.array([(t % np.pi) for _, t in lines])
    a2 = np.stack([np.cos(2 * ang), np.sin(2 * ang)], 1)
    _, lab, _ = cv2.kmeans(a2.astype(np.float32), 2, None, (cv2.TERM_CRITERIA_EPS, 10, 1e-3), 5, cv2.KMEANS_PP_CENTERS)
    lab = lab.ravel()
    fam = [[lines[i] for i in range(len(lines)) if lab[i] == k] for k in (0, 1)]
    pts = []
    for r1, t1 in fam[0]:
        for r2, t2 in fam[1]:
            A = np.array([[np.cos(t1), np.sin(t1)], [np.cos(t2), np.sin(t2)]])
            if abs(np.linalg.det(A)) < 0.3:
                continue
            x, y = np.linalg.solve(A, [r1, r2])
            if (
                5 < x < W - 5
                and 5 < y < H - 5
                and mask[max(0, int(y) - 6) : int(y) + 7, max(0, int(x) - 6) : int(x) + 7].any()
            ):
                pts.append((x, y))
    ys_, xs_ = np.nonzero(mask)
    mp = np.stack([xs_, ys_], 1).astype(float)
    good = []
    for x, y in pts:
        p0 = np.array([x, y])
        near = mp[(np.abs(mp - p0) < ARM_MAX).all(1)]
        dirs, ok = [], True
        for t in seek_thetas(x, y):
            pass
        good.append(refine(p0, near, fam))
    uniq = []
    for g in good:
        if g is not None and all(np.hypot(*(g - u)) > 18 for u in uniq):
            uniq.append(g)
    pts = np.array(uniq) if uniq else np.zeros((0, 2))
    # back to raw distorted pixels
    if len(pts):
        n = (pts - [CX, CY]) / CUR["f"]
        r2 = (n**2).sum(1, keepdims=True)
        raw = n * (1 + CUR["k1"] * r2) * CUR["f"] + [CX, CY]
    else:
        raw = pts
    if debug:
        dbg = img.copy()
        for x, y in raw:
            cv2.circle(dbg, (int(round(x)), int(round(y))), 5, (0, 0, 255), 1)
        dm = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for r, t in fam[0] + fam[1]:
            a, b = np.cos(t), np.sin(t)
            cv2.line(
                dm,
                (int(a * r - 1000 * b), int(b * r + 1000 * a)),
                (int(a * r + 1000 * b), int(b * r - 1000 * a)),
                (0, 255, 0),
                1,
            )
        cv2.imwrite(debug, np.hstack([dbg, dm]))
    dirs = []
    for f in fam:
        ang = np.array([t for _, t in f])
        m2 = np.arctan2(np.sin(2 * ang).mean(), np.cos(2 * ang).mean()) / 2
        dirs.append(np.array([-np.sin(m2), np.cos(m2)]))  # direction ALONG the lines of this family
    return raw, pts, dirs


if __name__ == "__main__":
    p, _, _ = detect(cv2.imread(sys.argv[1]), sys.argv[2])
    print(len(p))
    print(np.round(p, 1))
