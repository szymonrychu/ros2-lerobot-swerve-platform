#!/usr/bin/env python3
"""Ground truth of the replay scenes from the ChArUco board pose.

The robot is static, so base_link is fixed across the scenes. The board is detected in every RGB frame of a ground
recording (npz of web_ui GroundRecording), its pose is solved with the recorded RGB model (the image is the raw,
distorted mapping frame) and moved to base_link with the pair's base_T_rgb. The median outline of the board in
base_link places the box and the ramp, whose placements the user measured against the board (sheet file
scripts/ground_scenes.yaml, model SceneSheet).

Board frame of the placements: origin at the board's outer corner nearest the robot on the robot's right, u along
the board's near edge toward the robot's left, v along its right edge away from the robot (both on the floor, m).
A placement is the near-right footprint corner (u_m, v_m) of the object in that frame and its near edge's yaw_deg
from u (positive counter-clockwise seen from above); the object's length runs away from the robot.

Usage:
    uv run python -m scripts.ground_scene_truth [sheet]
"""

import argparse
import logging
from dataclasses import dataclass
from datetime import date
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict
from ros2_common.head_rgbd_calibration.charuco import detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig
from scipy.spatial.transform import Rotation

STRICT = ConfigDict(extra="forbid")
LOG = logging.getLogger("ground_scene_truth")

DEFAULT_SHEET = Path("scripts/ground_scenes.yaml")
MIN_DETECTIONS = 5
MIN_CORNERS = 12
MAX_SPREAD_MM = 5.0
MM_PER_M = 1000.0
ROUND_DIGITS = 4
BOARD_KEYS = ("squares_x", "squares_y", "square_mm", "marker_mm", "dictionary")
KEY_JPEG = "pair_jpeg"
KEY_JPEG_OFFSETS = "pair_jpeg_offsets"
KEY_BASE_T_RGB = "pair_base_T_rgb"
KEY_RGB_K = "rgb_K"
KEY_RGB_D = "rgb_D"
RIGHT_ANGLE_DEG = 90.0
ROBOT_LEFT = np.array([0.0, 1.0])


class TruthError(RuntimeError):
    """The recording does not give a trustworthy board pose."""


class Placement(BaseModel):
    """Footprint corner (u_m, v_m) in the board frame and the yaw of the near edge from u, degrees (CCW from above)."""

    model_config = STRICT
    u_m: float
    v_m: float
    yaw_deg: float


class Box(BaseModel):
    """The box: width_m is the near edge, length_m runs away from the robot."""

    model_config = STRICT
    length_m: float
    width_m: float
    height_m: float = 0.05
    placement: Placement


class Ramp(BaseModel):
    """The ramp: low_end names the end at floor level."""

    model_config = STRICT
    length_m: float = 0.25
    width_m: float
    height_m: float = 0.063
    low_end: Literal["near", "far"]
    placement: Placement


class EmptyRect(BaseModel):
    """Bare-floor rectangle in base_link, m."""

    model_config = STRICT
    x_min: float
    x_max: float
    y_min: float
    y_max: float


class SceneSheet(BaseModel):
    """scripts/ground_scenes.yaml: where the recording is and what the user measured against the board."""

    model_config = STRICT
    recording: Path = Path(".logs/ground_recordings/home.npz")
    truth_out: Path = Path("nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml")
    board_config: Path = Path("scripts/markers_generate.yaml")
    box: Box
    ramp: Ramp
    empty: EmptyRect


@dataclass(frozen=True)
class BoardResult:
    """Median board outline in base_link: corners (4, 3) in the board's local order, detections, spread_mm."""

    corners: np.ndarray
    detections: int
    spread_mm: float


def outline_corners(outline: dict[str, float]) -> np.ndarray:
    """Outer corners of the board in its own frame (origin at the grid corner), z = 0.

    :param outline: width_m, height_m (grid size) and margin_m (border around the grid) (dict[str, float]).
    :return: (4, 3) corners in the order (0,0), (W,0), (W,H), (0,H) of the grid, margin included (np.ndarray).
    """
    m, w, h = outline["margin_m"], outline["width_m"], outline["height_m"]
    return np.array([[-m, -m, 0.0], [w + m, -m, 0.0], [w + m, h + m, 0.0], [-m, h + m, 0.0]])


def pose_matrix(row: np.ndarray) -> np.ndarray:
    """4x4 transform of a pose row.

    :param row: x y z qx qy qz qw (np.ndarray, 7).
    :return: homogeneous matrix (np.ndarray, 4x4).
    """
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(row[3:7]).as_matrix()
    matrix[:3, 3] = row[:3]
    return matrix


def solve_board(
    images: list[np.ndarray],
    base_T_rgb: list[list[float]],
    board: cv2.aruco.CharucoBoard,
    k: np.ndarray,
    d: np.ndarray,
    outline: dict[str, float],
) -> BoardResult:
    """Board outline in base_link: median over the frames, failing when it moves.

    :param images: RGB frames, raw (distorted) (list of uint8 arrays).
    :param base_T_rgb: per frame x y z qx qy qz qw of the RGB optical frame in base_link (list of 7 floats).
    :param board: ChArUco board (cv2.aruco.CharucoBoard).
    :param k: RGB camera matrix, 9 values or 3x3 (np.ndarray).
    :param d: RGB distortion coefficients (np.ndarray).
    :param outline: see outline_corners (dict[str, float]).
    :return: the median outline (BoardResult).
    :raises TruthError: fewer than MIN_DETECTIONS frames with MIN_CORNERS corners, or a corner farther than
        MAX_SPREAD_MM from the median.
    """
    camera_matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
    local = outline_corners(outline)
    object_points = board.getChessboardCorners()
    per_frame: list[np.ndarray] = []
    for image, pose in zip(images, base_T_rgb, strict=True):
        found = detect(image, board)
        if found is None or len(found.ids) < MIN_CORNERS:
            continue
        ok, rvec, tvec = cv2.solvePnP(
            object_points[found.ids], found.points.astype(np.float64), camera_matrix, np.asarray(d, dtype=np.float64)
        )
        if not ok:
            continue
        rgb_T_board = np.eye(4)
        rgb_T_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        rgb_T_board[:3, 3] = tvec.ravel()
        base_T_board = pose_matrix(np.asarray(pose, dtype=np.float64)) @ rgb_T_board
        per_frame.append(local @ base_T_board[:3, :3].T + base_T_board[:3, 3])
    if len(per_frame) < MIN_DETECTIONS:
        raise TruthError(f"{len(per_frame)} board detections with >= {MIN_CORNERS} corners, need {MIN_DETECTIONS}")
    stack = np.stack(per_frame)
    median = np.median(stack, axis=0)
    spread_mm = float(np.linalg.norm(stack - median, axis=2).max() * MM_PER_M)
    if spread_mm > MAX_SPREAD_MM:
        raise TruthError(
            f"board moved or was occluded: corner {spread_mm:.1f} mm from the median (max {MAX_SPREAD_MM})"
        )
    return BoardResult(median, len(per_frame), spread_mm)


def board_frame(corners_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Board frame in base_link from the four outer corners, in any order.

    :param corners_xy: (4, 2) corner positions in base_link, m (np.ndarray).
    :return: (origin, u, v): the near-right corner, and unit vectors along the near edge toward the robot's left
        and away from the robot (np.ndarray, each 2).
    """
    centre = corners_xy.mean(axis=0)
    ring = corners_xy[np.argsort(np.arctan2(*(corners_xy - centre).T[::-1]))]
    edges = [(ring[i], ring[(i + 1) % 4]) for i in range(4)]
    start, end = min(edges, key=lambda e: float(np.linalg.norm((e[0] + e[1]) / 2)))
    u = (end - start) / np.linalg.norm(end - start)
    if u @ ROBOT_LEFT < 0:
        start, end, u = end, start, -u
    return start, u, np.array([u[1], -u[0]])


def rotate_ccw(vector: np.ndarray, yaw_deg: float) -> np.ndarray:
    """Rotate a 2D vector counter-clockwise (seen from above, x forward, y left).

    :param vector: (2,) vector (np.ndarray).
    :param yaw_deg: angle, degrees (float).
    :return: rotated vector (np.ndarray, 2).
    """
    c, s = np.cos(np.deg2rad(yaw_deg)), np.sin(np.deg2rad(yaw_deg))
    return np.array([c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]])


def placement_polygon(
    frame: tuple[np.ndarray, np.ndarray, np.ndarray], length_m: float, width_m: float, placement: Placement
) -> np.ndarray:
    """Footprint of a box or ramp in base_link.

    :param frame: board frame from board_frame (tuple of np.ndarray).
    :param length_m: extent running away from the robot (float).
    :param width_m: near edge length (float).
    :param placement: near-right corner and yaw in the board frame (Placement).
    :return: (4, 2) polygon, near-right, near-left, far-left, far-right at yaw 0 (np.ndarray).
    """
    origin, u, v = frame
    corner = origin + placement.u_m * u + placement.v_m * v
    across = rotate_ccw(u, placement.yaw_deg) * width_m
    along = rotate_ccw(v, placement.yaw_deg) * length_m
    return np.array([corner, corner + across, corner + across + along, corner + along])


def to_list(array: np.ndarray) -> list:
    """Round to 0.1 mm and convert for YAML.

    :param array: numeric array (np.ndarray).
    :return: nested lists of floats (list).
    """
    return np.round(array, ROUND_DIGITS).tolist()


def build_truth(sheet: SceneSheet, corners: np.ndarray, detections: int, spread_mm: float, recording: str) -> dict:
    """The truth document.

    :param sheet: measured scene (SceneSheet).
    :param corners: (4, 3) board outer corners in base_link, any order (np.ndarray).
    :param detections: accepted board detections (int).
    :param spread_mm: largest corner distance from the median, mm (float).
    :param recording: recording file name (str).
    :return: board, box, box_front_edge, box_height_m, ramp, ramp_low_end, empty and provenance (dict).
    """
    frame = board_frame(corners[:, :2])
    origin, u, v = frame
    rel = corners[:, :2] - origin
    extent_u, extent_v = float((rel @ u).max()), float((rel @ v).max())
    board = np.array([origin, origin + extent_u * u, origin + extent_u * u + extent_v * v, origin + extent_v * v])
    box = placement_polygon(frame, sheet.box.length_m, sheet.box.width_m, sheet.box.placement)
    ramp = placement_polygon(frame, sheet.ramp.length_m, sheet.ramp.width_m, sheet.ramp.placement)
    return {
        "board": to_list(board),
        "box": to_list(box),
        "box_front_edge": to_list(box[:2]),
        "box_height_m": sheet.box.height_m,
        "ramp": to_list(ramp),
        "ramp_low_end": sheet.ramp.low_end,
        "empty": sheet.empty.model_dump(),
        "provenance": {
            "recording": recording,
            "detections": detections,
            "spread_mm": round(spread_mm, 2),
            "date": date.today().isoformat(),
            "placements": {
                "box": {**sheet.box.model_dump(exclude={"placement"}), **sheet.box.placement.model_dump()},
                "ramp": {**sheet.ramp.model_dump(exclude={"placement"}), **sheet.ramp.placement.model_dump()},
            },
        },
    }


def load_board(path: Path) -> tuple[cv2.aruco.CharucoBoard, dict[str, float]]:
    """Board and outline from the charuco block of a markers_generate-style YAML.

    :param path: YAML file (Path).
    :return: (board, outline) as for solve_board (tuple).
    """
    block = yaml.safe_load(path.read_text())["charuco"]
    cfg = BoardConfig(**{key: block[key] for key in BOARD_KEYS})
    outline = {
        "width_m": cfg.squares_x * cfg.square_mm / MM_PER_M,
        "height_m": cfg.squares_y * cfg.square_mm / MM_PER_M,
        "margin_m": float(block.get("margin_mm", 0.0)) / MM_PER_M,
    }
    return make_board(cfg), outline


def run(sheet_path: Path = DEFAULT_SHEET) -> Path:
    """Detect the board in the recording and write the truth YAML.

    :param sheet_path: scene sheet YAML (Path).
    :return: the truth file written (Path).
    :raises TruthError: see solve_board.
    """
    sheet = SceneSheet.model_validate(yaml.safe_load(sheet_path.read_text()))
    board, outline = load_board(sheet.board_config)
    with np.load(sheet.recording) as data:
        blob, offsets = data[KEY_JPEG].tobytes(), data[KEY_JPEG_OFFSETS]
        poses, k, d = data[KEY_BASE_T_RGB], data[KEY_RGB_K], data[KEY_RGB_D]
    images = [
        cv2.imdecode(np.frombuffer(blob[a:b], dtype=np.uint8), cv2.IMREAD_COLOR)
        for a, b in zip(offsets[:-1], offsets[1:], strict=True)
    ]
    result = solve_board(images, poses.tolist(), board, k, d, outline)
    truth = build_truth(sheet, result.corners, result.detections, result.spread_mm, sheet.recording.name)
    sheet.truth_out.parent.mkdir(parents=True, exist_ok=True)
    sheet.truth_out.write_text(yaml.safe_dump(truth, sort_keys=False))
    LOG.info("truth from %d detections, spread %.2f mm -> %s", result.detections, result.spread_mm, sheet.truth_out)
    return sheet.truth_out


def main() -> None:
    """Parse the optional sheet path and write the truth."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sheet", nargs="?", type=Path, default=DEFAULT_SHEET, help="scene sheet YAML")
    run(parser.parse_args().sheet)


if __name__ == "__main__":
    main()
