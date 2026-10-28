"""Find and decode QR metadata markers in a grey frame (zxing-cpp), and locate undecodable QR quads (OpenCV)."""

from dataclasses import dataclass

import cv2
import numpy as np
import zxingcpp
from ros2_common.marker_payload import MarkerPayload, PayloadError, decode_payload

REASON_PAYLOAD = "payload"
FINDER_MODULES = 7
FINDER_SAMPLE_SCALE = 3
SYMBOL_BASE_MODULES = 17
MODULES_PER_VERSION = 4


@dataclass
class Candidate:
    """One decoded marker symbol.

    payload: validated payload. text: raw decoded text. corners: (4, 2) float, top-left, top-right, bottom-right,
    bottom-left in the symbol's own orientation (bottom-right is the corner without a finder). version: QR version.
    module_px: mean edge length in pixels divided by the module count.
    """

    payload: MarkerPayload
    text: str
    corners: np.ndarray
    version: int
    module_px: float


@dataclass
class Rejection:
    """A QR code that was read but is not a valid marker. reason: category. detail: why. corners: (4, 2) or None."""

    reason: str
    detail: str
    corners: np.ndarray | None = None


def finder_template() -> np.ndarray:
    """7x7 finder pattern, True = dark."""
    ring = np.maximum(*np.abs(np.mgrid[0:FINDER_MODULES, 0:FINDER_MODULES] - 3))
    return ring != 2


def finder_score(grey: np.ndarray, corners: np.ndarray, modules: int) -> float:
    """Agreement of the three expected finder corners with the image under a given corner order.

    Args:
        grey: Grey frame.
        corners: (4, 2) corners assumed top-left, top-right, bottom-right, bottom-left.
        modules: Module count along one edge.

    Returns:
        Mean agreement in [0, 1] of the finder template at top-left, top-right and bottom-left.
    """
    side = modules * FINDER_SAMPLE_SCALE
    target = np.array([[0, 0], [side, 0], [side, side], [0, side]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(corners.astype(np.float32), target)
    patch = cv2.warpPerspective(grey, matrix, (side, side), flags=cv2.INTER_AREA)
    dark = patch < patch.mean()
    template = finder_template()
    cell = FINDER_SAMPLE_SCALE
    scores = []
    for row, col in ((0, 0), (0, modules - FINDER_MODULES), (modules - FINDER_MODULES, 0)):
        window = dark[row * cell : (row + FINDER_MODULES) * cell, col * cell : (col + FINDER_MODULES) * cell]
        centres = window[cell // 2 :: cell, cell // 2 :: cell]
        scores.append(float(np.mean(centres == template)))
    return float(np.mean(scores))


def order_corners(grey: np.ndarray, corners: np.ndarray, modules: int) -> np.ndarray:
    """Return the corner rotation (cyclic shift) whose finders match the QR layout; the reported order wins ties.

    Args:
        grey: Grey frame.
        corners: (4, 2) corners from the decoder.
        modules: Module count along one edge.

    Returns:
        (4, 2) corners ordered top-left, top-right, bottom-right, bottom-left of the symbol.
    """
    best, best_score = corners, finder_score(grey, corners, modules)
    for shift in (1, 2, 3):
        rolled = np.roll(corners, -shift, axis=0)
        score = finder_score(grey, rolled, modules)
        if score > best_score + 1e-9:
            best, best_score = rolled, score
    return best


def decode_frame(grey: np.ndarray) -> tuple[list[Candidate], list[Rejection]]:
    """Decode every QR code in a frame and validate the payloads.

    Args:
        grey: Grey uint8 image.

    Returns:
        (candidates, rejections): valid marker symbols, and QR codes whose text is not a valid marker payload.
    """
    candidates: list[Candidate] = []
    rejections: list[Rejection] = []
    for result in zxingcpp.read_barcodes(grey, formats=zxingcpp.BarcodeFormat.QRCode):
        position = result.position
        corners = np.array(
            [[p.x, p.y] for p in (position.top_left, position.top_right, position.bottom_right, position.bottom_left)],
            dtype=float,
        )
        try:
            payload = decode_payload(result.text)
        except PayloadError as exc:
            rejections.append(Rejection(REASON_PAYLOAD, str(exc), corners))
            continue
        version = int(result.extra["Version"])
        modules = SYMBOL_BASE_MODULES + MODULES_PER_VERSION * version
        corners = order_corners(grey, corners, modules)
        edges = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
        candidates.append(Candidate(payload, result.text, corners, version, float(edges.mean() / modules)))
    return candidates, rejections
