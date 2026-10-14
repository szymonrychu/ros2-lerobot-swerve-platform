"""Stereo ChArUco calibration for the Jetson (python3 -m stereo_calibrate board|capture|solve).

Adapted from the Pi-era nodes/stereo_depth/stereo_depth/calibrate.py (git 31de0d5): per-camera calibration, cv2.stereoCalibrate
with CALIB_FIX_INTRINSIC, cv2.stereoRectify alpha 0 and the RMS gate. Differences: the pairs come from depthd's
GET /calibration/capture (urllib), the result is written in the depthd stereo.yaml format (see
jetson/depthd/include/depthd/calibration.hpp) atomically, and depthd is told with POST /calibration/reload.

Exit codes: 0 done, 1 failure, 2 usage or config error, 3 solve refused by the RMS gate (nothing written),
4 file written but depthd did not reload it.
"""

import argparse
import math
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

import cv2
import numpy as np

from .config import CalibrateConfig, ConfigError, load_config

EXIT_FAILURE, EXIT_USAGE, EXIT_REFUSED, EXIT_NOT_RELOADED = 1, 2, 3, 4
PAIR_NAME = "pair_%04d"
LEFT_SUFFIX, RIGHT_SUFFIX = "_left.png", "_right.png"
BOARD_MARGIN_FRACTION = 0.04
BORDER_BITS = 1
STEREO_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
RECTIFY_ALPHA = 0.0
HTTP_TIMEOUT_S = 10.0
MIN_SQUARE_PX = 10  # a marker cell needs about a pixel


class CalibrationError(Exception):
    """The calibration cannot be solved or written."""


class RmsGateError(CalibrationError):
    """The stereo RMS is above rms_limit_px; nothing is written."""


class SharedPoints(NamedTuple):
    """Board corners seen in both images of one pair."""

    objects: np.ndarray  # float32 (N, 3), metres, board frame
    left: np.ndarray  # float32 (N, 1, 2)
    right: np.ndarray  # float32 (N, 1, 2)
    ids: np.ndarray  # int32 (N, 1)


class StereoResult(NamedTuple):
    """A solved, rectified pair (the content of stereo.yaml)."""

    width: int
    height: int
    rms: float
    baseline_m: float
    k_left: np.ndarray
    d_left: np.ndarray
    k_right: np.ndarray
    d_right: np.ndarray
    r: np.ndarray
    t: np.ndarray
    r1: np.ndarray
    r2: np.ndarray
    p1: np.ndarray
    p2: np.ndarray
    q: np.ndarray


# ---- board ----


def charuco_board(config: CalibrateConfig) -> "cv2.aruco.CharucoBoard":
    """The ChArUco board of the config (lengths in metres).

    Args:
        config (CalibrateConfig): Board settings.

    Returns:
        cv2.aruco.CharucoBoard: The board.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.dictionary))
    return cv2.aruco.CharucoBoard(
        (config.squares_x, config.squares_y), config.square_mm / 1000.0, config.marker_mm / 1000.0, dictionary
    )


def render_board(config: CalibrateConfig, width_px: int, height_px: int) -> np.ndarray:
    """The board as an 8-bit grey canvas of exactly width_px x height_px, for an e-reader shown unscaled.

    The board is rotated to portrait on a portrait canvas (landscape on a landscape one), has an integer number of
    pixels per square (the largest leaving a margin of at least 4 % of the short side on every side) and is centred
    on white.

    Args:
        config (CalibrateConfig): Board settings (squares, dictionary, marker/square ratio).
        width_px (int): Canvas width.
        height_px (int): Canvas height.

    Returns:
        np.ndarray: uint8 (height_px, width_px).

    Raises:
        CalibrationError: When a square under 10 px is all that fits with the margin.
    """
    portrait = height_px >= width_px
    long_squares, short_squares = max(config.squares_x, config.squares_y), min(config.squares_x, config.squares_y)
    cols, rows = (short_squares, long_squares) if portrait else (long_squares, short_squares)
    margin = BOARD_MARGIN_FRACTION * min(width_px, height_px)
    square = int(min((width_px - 2 * margin) / cols, (height_px - 2 * margin) / rows))
    if square < MIN_SQUARE_PX:
        raise CalibrationError("a %dx%d canvas is too small for the board" % (width_px, height_px))
    flat = charuco_board(config).generateImage(
        (config.squares_x * square, config.squares_y * square), marginSize=0, borderBits=BORDER_BITS
    )
    if portrait == (config.squares_x > config.squares_y):
        flat = cv2.rotate(flat, cv2.ROTATE_90_CLOCKWISE)
    canvas = np.full((height_px, width_px), 255, dtype=np.uint8)
    top, left = (height_px - flat.shape[0]) // 2, (width_px - flat.shape[1]) // 2
    canvas[top : top + flat.shape[0], left : left + flat.shape[1]] = flat
    return canvas


# ---- detection ----


def detect_charuco(grey: np.ndarray, board: "cv2.aruco.CharucoBoard") -> Dict[int, np.ndarray]:
    """ChArUco corners of one image by corner id.

    Args:
        grey (np.ndarray): uint8 grey image.
        board (cv2.aruco.CharucoBoard): The board.

    Returns:
        Dict[int, np.ndarray]: Corner id to float32 (1, 2) point; empty when nothing is found.
    """
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(grey)
    if ids is None or corners is None:
        return {}
    return {int(i): np.asarray(c, dtype=np.float32).reshape(1, 2) for i, c in zip(ids.ravel(), corners)}


def shared_points(left: np.ndarray, right: np.ndarray, config: CalibrateConfig) -> Optional[SharedPoints]:
    """Board corners found in both images of a pair.

    Args:
        left (np.ndarray): Left uint8 grey image.
        right (np.ndarray): Right uint8 grey image.
        config (CalibrateConfig): Board settings and min_shared_corners.

    Returns:
        Optional[SharedPoints]: The shared corners, or None with fewer than min_shared_corners of them.
    """
    board = charuco_board(config)
    left_found, right_found = detect_charuco(left, board), detect_charuco(right, board)
    common = sorted(set(left_found) & set(right_found))
    if len(common) < config.min_shared_corners:
        return None
    objects = np.asarray(board.getChessboardCorners(), dtype=np.float32)[common]
    return SharedPoints(
        objects,
        np.array([left_found[i] for i in common], dtype=np.float32).reshape(-1, 1, 2),
        np.array([right_found[i] for i in common], dtype=np.float32).reshape(-1, 1, 2),
        np.array(common, dtype=np.int32).reshape(-1, 1),
    )


# ---- pose diversity ----


class PoseFilter:
    """Keeps a pair only when the board pose differs from every kept pose.

    A pose is the board centre in the image (coverage of the image corners), its size, its foreshortening (tilt)
    and its in-plane rotation, taken from an affine fit of the board corners to the left image.
    """

    def __init__(self, config: CalibrateConfig) -> None:
        """Remember the thresholds and the board corner layout.

        Args:
            config (CalibrateConfig): Thresholds and board.
        """
        self.config = config
        self.board_xy = np.asarray(charuco_board(config).getChessboardCorners(), dtype=np.float64)[:, :2] * 1000.0
        self.poses: List[Tuple[float, float, float, float, float, float]] = []

    def __len__(self) -> int:
        return len(self.poses)

    def pose(self, points: np.ndarray, ids: np.ndarray, size: Tuple[int, int]) -> Tuple[float, ...]:
        """Describe the board pose: (cx, cy, log scale, tilt_x, tilt_y, roll_deg).

        Args:
            points (np.ndarray): Left image corners, (N, 1, 2).
            ids (np.ndarray): Their corner ids, (N, 1).
            size (Tuple[int, int]): Image (width, height).

        Returns:
            Tuple[float, ...]: The pose vector.
        """
        image = points.reshape(-1, 2).astype(np.float64)
        design = np.column_stack([self.board_xy[ids.ravel()], np.ones(len(image))])
        solution = np.linalg.lstsq(design, image, rcond=None)[0]  # rows: du/dx dv/dx, du/dy dv/dy, offset
        jac = solution[:2].T  # 2x2 board mm -> pixels
        u, s, _ = np.linalg.svd(jac)
        tilt = 1.0 - s[1] / s[0]
        phi = math.atan2(u[1, 0], u[0, 0])
        roll = math.degrees(math.atan2(jac[1, 0], jac[0, 0]))
        centre = image.mean(axis=0)
        return (
            float(centre[0] / size[0]),
            float(centre[1] / size[1]),
            float(math.log(math.sqrt(s[0] * s[1]))),
            float(tilt * math.cos(2 * phi)),
            float(tilt * math.sin(2 * phi)),
            roll,
        )

    def differs(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """Whether two poses differ by at least one threshold.

        Args:
            a (Sequence[float]): A pose vector.
            b (Sequence[float]): Another pose vector.

        Returns:
            bool: True when the poses are distinct enough.
        """
        c = self.config
        roll = abs((a[5] - b[5] + 180.0) % 360.0 - 180.0)
        return (
            math.hypot(a[0] - b[0], a[1] - b[1]) >= c.min_position_shift
            or abs(a[2] - b[2]) >= math.log1p(c.min_scale_change)
            or math.hypot(a[3] - b[3], a[4] - b[4]) >= c.min_tilt_change
            or roll >= c.min_roll_change_deg
        )

    def accept(self, points: np.ndarray, ids: np.ndarray, size: Tuple[int, int]) -> bool:
        """Keep the pose when it differs from every kept pose.

        Args:
            points (np.ndarray): Left image corners, (N, 1, 2).
            ids (np.ndarray): Their corner ids, (N, 1).
            size (Tuple[int, int]): Image (width, height).

        Returns:
            bool: True when the pose is new (and now kept).
        """
        pose = self.pose(points, ids, size)
        if any(not self.differs(pose, kept) for kept in self.poses):
            return False
        self.poses.append(pose)  # type: ignore[arg-type]
        return True


# ---- solve ----


def solve_points(
    objects: Sequence[np.ndarray],
    left: Sequence[np.ndarray],
    right: Sequence[np.ndarray],
    ids: Sequence[np.ndarray],
    size: Tuple[int, int],
    config: Optional[CalibrateConfig] = None,
) -> StereoResult:
    """Calibrate from shared corners: per-camera intrinsics, stereo extrinsics with fixed intrinsics, rectify (alpha 0).

    Args:
        objects (Sequence[np.ndarray]): Per pair float32 (N, 3) board points, metres.
        left (Sequence[np.ndarray]): Per pair float32 (N, 1, 2) left image points.
        right (Sequence[np.ndarray]): Per pair float32 (N, 1, 2) right image points.
        ids (Sequence[np.ndarray]): Per pair int32 (N, 1) corner ids.
        size (Tuple[int, int]): Image (width, height).
        config (Optional[CalibrateConfig]): Board settings; the default board when None.

    Returns:
        StereoResult: The rectified pair with its stereo RMS.

    Raises:
        CalibrationError: When OpenCV cannot solve the set.
    """
    board = charuco_board(config or CalibrateConfig())
    try:
        _, k1, d1, _, _ = cv2.aruco.calibrateCameraCharuco(list(left), list(ids), board, size, None, None)
        _, k2, d2, _, _ = cv2.aruco.calibrateCameraCharuco(list(right), list(ids), board, size, None, None)
        rms, k1, d1, k2, d2, r, t, _, _ = cv2.stereoCalibrate(
            list(objects), list(left), list(right), k1, d1, k2, d2, size,
            criteria=STEREO_CRITERIA, flags=cv2.CALIB_FIX_INTRINSIC,
        )  # fmt: skip
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            k1, d1, k2, d2, size, r, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=RECTIFY_ALPHA
        )
    except cv2.error as e:
        raise CalibrationError("OpenCV could not solve the pairs: %s" % e) from e
    d1, d2 = d1.reshape(1, -1), d2.reshape(1, -1)
    t = t.reshape(3, 1)
    return StereoResult(size[0], size[1], float(rms), float(np.linalg.norm(t)), k1, d1, k2, d2, r, t, r1, r2, p1, p2, q)


def solve(pairs: Sequence[Tuple[np.ndarray, np.ndarray]], config: CalibrateConfig) -> StereoResult:
    """Detect the board in each (left, right) grey pair and calibrate.

    Args:
        pairs (Sequence[Tuple[np.ndarray, np.ndarray]]): uint8 grey image pairs, one size.
        config (CalibrateConfig): Board settings, min_shared_corners and min_pairs.

    Returns:
        StereoResult: The rectified pair with its stereo RMS.

    Raises:
        CalibrationError: With mixed image sizes, fewer than min_pairs usable pairs or an unsolvable set.
    """
    sizes = {image.shape[:2] for pair in pairs for image in pair}
    if len(sizes) > 1:
        raise CalibrationError("pairs have mixed image sizes: %s" % sorted(sizes))
    found = [s for s in (shared_points(left, right, config) for left, right in pairs) if s is not None]
    if len(found) < config.min_pairs:
        raise CalibrationError(
            "%d usable pairs (board with at least %d shared corners), need min_pairs=%d"
            % (len(found), config.min_shared_corners, config.min_pairs)
        )
    height, width = sizes.pop()
    return solve_points(
        [f.objects for f in found], [f.left for f in found], [f.right for f in found], [f.ids for f in found],
        (width, height), config,
    )  # fmt: skip


def write_calibration(result: StereoResult, config: CalibrateConfig) -> Path:
    """Write stereo.yaml atomically (temp file + rename), refusing a result above the RMS limit.

    Args:
        result (StereoResult): Result of solve.
        config (CalibrateConfig): output_path and rms_limit_px.

    Returns:
        Path: The written file.

    Raises:
        RmsGateError: When the RMS is above rms_limit_px (nothing is written).
        CalibrationError: When the file cannot be written.
    """
    if result.rms > config.rms_limit_px:
        raise RmsGateError(
            "stereo RMS %.3f px is above rms_limit_px=%s; nothing written. Recapture with the board flat, sharp and "
            "covering more of the image." % (result.rms, config.rms_limit_px)
        )
    path = config.output_path
    temp = path.with_name(path.name + ".partial.yaml")  # FileStorage picks the format from the extension
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fs = cv2.FileStorage(str(temp), cv2.FILE_STORAGE_WRITE)
        if not fs.isOpened():
            raise CalibrationError("cannot open %s for writing" % temp)
        fs.write("image_width", int(result.width))
        fs.write("image_height", int(result.height))
        fs.write("rms", result.rms)
        fs.write("baseline_m", result.baseline_m)
        for key, matrix in (
            ("K_left", result.k_left), ("D_left", result.d_left), ("K_right", result.k_right),
            ("D_right", result.d_right), ("R", result.r), ("T", result.t), ("R1", result.r1), ("R2", result.r2),
            ("P1", result.p1), ("P2", result.p2), ("Q", result.q),
        ):  # fmt: skip
            fs.write(key, np.asarray(matrix, dtype=np.float64))
        fs.release()
        with open(str(temp), "rb") as written:
            os.fsync(written.fileno())
        os.replace(str(temp), str(path))
    except OSError as e:
        raise CalibrationError("cannot write %s: %s" % (path, e)) from e
    return path


# ---- depthd HTTP ----


def parse_multipart(body: bytes, content_type: str) -> Dict[str, bytes]:
    """Split a multipart/form-data body into name -> payload.

    Args:
        body (bytes): Response body.
        content_type (str): Content-Type header with the boundary parameter.

    Returns:
        Dict[str, bytes]: Part name to raw payload.

    Raises:
        CalibrationError: When the boundary is missing.
    """
    marker = "boundary="
    if marker not in content_type:
        raise CalibrationError("capture response has no multipart boundary: %r" % content_type)
    delimiter = b"--" + content_type.split(marker, 1)[1].split(";")[0].strip().strip('"').encode()
    parts: Dict[str, bytes] = {}
    for chunk in body.split(delimiter)[1:]:
        if chunk.startswith(b"--"):
            break
        head, sep, payload = chunk.partition(b"\r\n\r\n")
        if not sep:
            continue
        for line in head.decode("latin-1").split("\r\n"):
            if line.lower().startswith("content-disposition") and 'name="' in line:
                name = line.split('name="', 1)[1].split('"', 1)[0]
                parts[name] = payload[:-2] if payload.endswith(b"\r\n") else payload
    return parts


def fetch_pair(base_url: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """GET /calibration/capture: the latest unrectified pair as grey images.

    Args:
        base_url (str): depthd HTTP API base URL.

    Returns:
        Optional[Tuple[np.ndarray, np.ndarray]]: (left, right), or None when depthd has no fresh pair (503).

    Raises:
        CalibrationError: When depthd is unreachable or answers something unusable.
    """
    try:
        with urllib.request.urlopen(base_url.rstrip("/") + "/calibration/capture", timeout=HTTP_TIMEOUT_S) as response:
            body, content_type = response.read(), response.headers.get("Content-Type", "")
    except urllib.error.HTTPError as e:
        if e.code == 503:
            return None
        raise CalibrationError("depthd capture answered HTTP %d" % e.code) from e
    except (urllib.error.URLError, OSError) as e:
        raise CalibrationError("depthd at %s is not reachable: %s" % (base_url, e)) from e
    parts = parse_multipart(body, content_type)
    images = []
    for name in ("left", "right"):
        image = cv2.imdecode(np.frombuffer(parts.get(name, b""), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise CalibrationError("capture response has no decodable %r image" % name)
        images.append(image)
    return images[0], images[1]


def post_reload(base_url: str) -> Optional[str]:
    """POST /calibration/reload.

    Args:
        base_url (str): depthd HTTP API base URL.

    Returns:
        Optional[str]: None when depthd reloaded the file, else why not.
    """
    request = urllib.request.Request(base_url.rstrip("/") + "/calibration/reload", data=b"", method="POST")
    try:
        with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_S):
            return None
    except urllib.error.HTTPError as e:
        return "depthd refused the calibration (HTTP %d): %s" % (e.code, e.read().decode("utf-8", "replace")[:300])
    except (urllib.error.URLError, OSError) as e:
        return "depthd at %s is not reachable: %s" % (base_url, e)


# ---- capture and solve modes ----


def save_pair(captures_dir: Path, index: int, left: np.ndarray, right: np.ndarray) -> None:
    """Save one pair as lossless PNGs.

    Args:
        captures_dir (Path): Directory (created).
        index (int): Pair number.
        left (np.ndarray): Left image.
        right (np.ndarray): Right image.

    Raises:
        CalibrationError: When a file cannot be written.
    """
    captures_dir.mkdir(parents=True, exist_ok=True)
    stem = PAIR_NAME % index
    for suffix, image in ((LEFT_SUFFIX, left), (RIGHT_SUFFIX, right)):
        if not cv2.imwrite(str(captures_dir / (stem + suffix)), image):
            raise CalibrationError("could not write %s" % (captures_dir / (stem + suffix)))


def load_pairs(captures_dir: Path) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Read the saved pairs back in file order; a half pair is skipped.

    Args:
        captures_dir (Path): Directory.

    Returns:
        List[Tuple[np.ndarray, np.ndarray]]: (left, right) grey images.
    """
    pairs = []
    for left_path in sorted(captures_dir.glob("pair_*" + LEFT_SUFFIX)):
        right_path = left_path.with_name(left_path.name[: -len(LEFT_SUFFIX)] + RIGHT_SUFFIX)
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE) if right_path.is_file() else None
        if left is not None and right is not None:
            pairs.append((left, right))
    return pairs


def run_capture(config: CalibrateConfig, sleep: Callable[[float], None] = time.sleep) -> int:
    """Pull pairs from depthd until capture_count are kept or capture_timeout_s passes.

    A pair is kept when the board is seen in both images with at least min_shared_corners shared corners and its pose
    differs from every kept pose. Old pairs in captures_dir are removed first.

    Args:
        config (CalibrateConfig): Settings.
        sleep (Callable[[float], None]): Pause between requests.

    Returns:
        int: 0 when capture_count pairs were kept, else 1.
    """
    try:
        for stale in list(config.captures_dir.glob("pair_*.png")) if config.captures_dir.is_dir() else []:
            stale.unlink()
        pose_filter = PoseFilter(config)
        deadline = time.monotonic() + config.capture_timeout_s
        print(
            "Capturing %d pairs: move the board across the whole view, tilt it, bring it closer and further."
            % config.capture_count
        )
        while len(pose_filter) < config.capture_count:
            if time.monotonic() > deadline:
                print(
                    "capture timed out with %d/%d pairs kept" % (len(pose_filter), config.capture_count),
                    file=sys.stderr,
                )
                return EXIT_FAILURE
            pair = fetch_pair(config.depthd_url)
            shared = shared_points(*pair, config) if pair else None
            if shared is not None:
                height, width = pair[0].shape[:2]
                if pose_filter.accept(shared.left, shared.ids, (width, height)):
                    save_pair(config.captures_dir, len(pose_filter) - 1, *pair)
                    print("kept %d/%d (%d shared corners)" % (len(pose_filter), config.capture_count, len(shared.ids)))
            sleep(config.capture_poll_s)
    except CalibrationError as e:
        print("capture failed: %s" % e, file=sys.stderr)
        return EXIT_FAILURE
    print(
        "Captured %d pairs in %s; now run: python3 -m stereo_calibrate solve"
        % (config.capture_count, config.captures_dir)
    )
    return 0


def run_solve(config: CalibrateConfig, reload: bool = True) -> int:
    """Solve from the kept pairs, write stereo.yaml and tell depthd to reload it.

    Args:
        config (CalibrateConfig): Settings.
        reload (bool): POST /calibration/reload after writing.

    Returns:
        int: 0 done, 1 failure, 3 refused by the RMS gate, 4 written but depthd did not reload.
    """
    pairs = load_pairs(config.captures_dir) if config.captures_dir.is_dir() else []
    try:
        result = solve(pairs, config)
        path = write_calibration(result, config)
    except RmsGateError as e:
        print("stereo calibration refused: %s" % e, file=sys.stderr)
        return EXIT_REFUSED
    except CalibrationError as e:
        print("stereo calibration failed: %s" % e, file=sys.stderr)
        return EXIT_FAILURE
    print(
        "stereo RMS %.3f px, baseline %.1f mm, fx %.1f px; wrote %s"
        % (result.rms, result.baseline_m * 1000.0, result.k_left[0, 0], path)
    )
    if not reload:
        return 0
    problem = post_reload(config.depthd_url)
    if problem:
        print("%s; the file is written, restart depthd or retry the reload" % problem, file=sys.stderr)
        return EXIT_NOT_RELOADED
    print("depthd reloaded the calibration")
    return 0


def run_board(config: CalibrateConfig, width_px: int, height_px: int, out: Path) -> int:
    """Write the e-reader board PNG.

    Args:
        config (CalibrateConfig): Board settings.
        width_px (int): Canvas width.
        height_px (int): Canvas height.
        out (Path): PNG path.

    Returns:
        int: 0 when written, else 1.
    """
    try:
        image = render_board(config, width_px, height_px)
    except CalibrationError as e:
        print("board failed: %s" % e, file=sys.stderr)
        return EXIT_FAILURE
    if not cv2.imwrite(str(out), image):
        print("could not write %s" % out, file=sys.stderr)
        return EXIT_FAILURE
    print("wrote %s (%dx%d); open it unscaled and measure one square with a ruler" % (out, width_px, height_px))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Command line entry: board, capture or solve.

    Args:
        argv (Optional[Sequence[str]]): Arguments; sys.argv[1:] when None.

    Returns:
        int: Process exit code.
    """
    parser = argparse.ArgumentParser(prog="stereo_calibrate", description=__doc__.split("\n")[0])
    parser.add_argument("--config", type=Path, help="calibrate.yaml (default: STEREO_CALIBRATE_CONFIG or config/)")
    sub = parser.add_subparsers(dest="mode", required=True)
    board = sub.add_parser("board", help="render the board PNG for an e-reader")
    board.add_argument("--width-px", type=int, required=True)
    board.add_argument("--height-px", type=int, required=True)
    board.add_argument("--out", type=Path, required=True)
    sub.add_parser("capture", help="pull pairs from depthd while the board is moved across the view")
    solve_parser = sub.add_parser("solve", help="solve the kept pairs, write stereo.yaml, reload depthd")
    solve_parser.add_argument("--no-reload", action="store_true", help="do not POST /calibration/reload")
    try:
        args = parser.parse_args(argv)
        config = load_config(args.config)
    except ConfigError as e:
        print("stereo_calibrate config error: %s" % e, file=sys.stderr)
        return EXIT_USAGE
    except SystemExit as e:
        return int(e.code or 0) if isinstance(e.code, int) else EXIT_USAGE
    if args.mode == "board":
        return run_board(config, args.width_px, args.height_px, args.out)
    if args.mode == "capture":
        return run_capture(config)
    return run_solve(config, reload=not args.no_reload)
