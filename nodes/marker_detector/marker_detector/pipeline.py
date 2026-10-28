"""Marker detection pipeline (no ROS): decode -> pose (depth plane chooser) -> TF -> confidence gate -> commands and report."""

import logging
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import cv2
import numpy as np
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.marker_payload import encode_payload

from marker_detector.config import MarkerDetectorConfig
from marker_detector.decode import Candidate, decode_frame
from marker_detector.depth_plane import fit_plane, nearest_depth, plane_chooser
from marker_detector.geometry import detection_report, marker_in_map, sighting_command
from marker_detector.pose import PoseResult, Solution, estimate_pose

LOG = logging.getLogger(__name__)
REASON_TF = "tf"
REASON_LOW_CONFIDENCE = "low_confidence"
REASON_IMAGE = "image"
CHOOSER_REPROJECTION = "reprojection"
CHOOSER_DEPTH = "depth"
MAX_DEPTH_FRAMES = 8

TfLookup = Callable[[str, int], tuple | None]


@dataclass
class FrameOutput:
    """What one frame produced.

    commands: ``upsert_marker`` dicts for ``/poi/command``. report: ``/markers/detections`` JSON, only when at least one
    detection was accepted. rejections: reason -> count.
    """

    commands: list[dict] = field(default_factory=list)
    report: str | None = None
    rejections: dict[str, int] = field(default_factory=dict)

    def reject(self, reason: str) -> None:
        """Count one rejection.

        Args:
            reason: Rejection category.
        """
        self.rejections[reason] = self.rejections.get(reason, 0) + 1


class MarkerPipeline:
    """Turns camera frames and depth frames into POI upsert commands (placeholder-free)."""

    def __init__(self, cfg: MarkerDetectorConfig) -> None:
        """Create the pipeline.

        Args:
            cfg: Validated detector config.
        """
        self.cfg = cfg
        self.K: np.ndarray | None = None
        self.D: np.ndarray | None = None
        self.size: tuple[int, int] | None = None
        self.frame_id = ""
        self.depth_intrinsics: CameraIntrinsics | None = None
        self.depth_frames: deque[tuple[int, np.ndarray]] = deque(maxlen=MAX_DEPTH_FRAMES)
        self.request_count = 0

    def on_camera_info(self, K: np.ndarray, D: np.ndarray | None, size: tuple[int, int], frame_id: str) -> None:  # noqa: N803
        """Store the stream intrinsics; an all-zero (uncalibrated) matrix counts as no camera_info.

        Args:
            K: 3x3 intrinsics. D: distortion coefficients or None/empty. size: (width, height). frame_id: optical frame.
        """
        k = np.asarray(K, dtype=float).reshape(3, 3)
        calibrated = k[0, 0] > 0 and k[1, 1] > 0
        if calibrated != (self.K is not None):
            LOG.info("camera_info %s", "received" if calibrated else "uncalibrated; processing stopped")
        if not calibrated:
            self.K = None
            return
        dist = None if D is None or len(D) == 0 else np.asarray(D, dtype=float)
        self.K, self.D, self.size, self.frame_id = k, dist, (int(size[0]), int(size[1])), frame_id

    def on_depth(self, stamp_ns: int, depth_mm: np.ndarray, depth_intrinsics: CameraIntrinsics) -> None:
        """Remember an aligned depth frame (16UC1 mm) for the depth plane disambiguation.

        A frame whose size differs from its intrinsics is dropped; new intrinsics drop the frames kept for the old ones.

        Args:
            stamp_ns: Depth stamp. depth_mm: (H, W) depth. depth_intrinsics: intrinsics from the depth camera_info.
        """
        if depth_mm.shape[:2] != (depth_intrinsics.height, depth_intrinsics.width):
            LOG.debug("depth %s does not match its camera_info size; dropped", depth_mm.shape)
            return
        if depth_intrinsics != self.depth_intrinsics:
            self.depth_frames.clear()
            self.depth_intrinsics = depth_intrinsics
        self.depth_frames.append((int(stamp_ns), depth_mm))

    def process_frame(self, stamp_ns: int, jpeg: bytes, lookup_tf: TfLookup) -> FrameOutput:
        """Process one compressed stream image.

        Args:
            stamp_ns: Image stamp. jpeg: compressed image bytes.
            lookup_tf: (frame_id, stamp_ns) -> (translation, quaternion xyzw) of the frame in map, or None.

        Returns:
            FrameOutput (empty without camera_info).
        """
        out = FrameOutput()
        if self.K is None or self.size is None:
            return out
        grey = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        if grey is None:
            out.reject(REASON_IMAGE)
            return out
        candidates, rejected = decode_frame(grey)
        for rejection in rejected:
            out.reject(rejection.reason)
        items = []
        for candidate in candidates:
            item = self.accept(grey, candidate, stamp_ns, lookup_tf, out)
            if item is not None:
                items.append(item)
        if items:
            out.report = detection_report(stamp_ns, items)
        return out

    def accept(
        self, grey: np.ndarray, candidate: Candidate, stamp_ns: int, lookup_tf: TfLookup, out: FrameOutput
    ) -> dict | None:
        """Pose, TF and gate one candidate; append its command to ``out`` when accepted.

        Args:
            grey: Grey stream image. candidate: decoded marker. stamp_ns: image stamp. lookup_tf: TF lookup.
            out: frame output to extend.

        Returns:
            The report item, or None when the candidate was rejected.
        """
        used_depth = []
        choose = self.depth_chooser(candidate, stamp_ns, used_depth)
        pose = estimate_pose(grey, candidate, self.K, self.D, self.cfg.pose, choose)
        if not pose.ok or pose.rvec is None or pose.tvec is None:
            out.reject(pose.reason or "pnp_failed")
            return None
        if pose.confidence < self.cfg.pose.min_confidence:
            out.reject(REASON_LOW_CONFIDENCE)
            return None
        map_from_camera = lookup_tf(self.frame_id, stamp_ns)
        if map_from_camera is None:
            out.reject(REASON_TF)
            return None
        position, quaternion = marker_in_map(pose.rvec, pose.tvec, map_from_camera)
        self.request_count += 1
        out.commands.append(
            sighting_command(candidate, pose, position, quaternion, f"marker-{stamp_ns}-{self.request_count}")
        )
        return self.report_item(candidate, pose, position, quaternion, bool(used_depth))

    def depth_chooser(
        self, candidate: Candidate, stamp_ns: int, used: list[bool]
    ) -> Callable[[list[Solution]], Solution | None] | None:
        """Disambiguation callback using the depth frame nearest in time, or None when the depth plane cannot apply.

        Args:
            candidate: The decoded marker (its corners are the polygon). stamp_ns: image stamp.
            used: appended with True when the chooser settled the ambiguity.

        Returns:
            Callback for ``estimate_pose`` (fits the plane lazily, only when the pose is ambiguous) or None.
        """
        plane_cfg = self.cfg.depth_plane
        if not plane_cfg.enabled or self.depth_intrinsics is None or self.size is None:
            return None
        frame = nearest_depth(self.depth_frames, stamp_ns, plane_cfg.max_dt_s)
        if frame is None:
            return None
        depth, intrinsics, size = frame[1], self.depth_intrinsics, self.size

        def choose(solutions: list[Solution]) -> Solution | None:
            plane = fit_plane(depth, intrinsics, candidate.corners, size, plane_cfg)
            chosen = plane_chooser(plane, plane_cfg)(solutions) if plane is not None else None
            if chosen is not None:
                used.append(True)
            return chosen

        return choose

    def report_item(
        self, candidate: Candidate, pose: PoseResult, position: np.ndarray, quaternion: np.ndarray, by_depth: bool
    ) -> dict:
        """Per marker entry of the detections report.

        Args:
            candidate: decoded marker. pose: accepted pose. position: map position. quaternion: map orientation.
            by_depth: True when the depth plane chose the solution.

        Returns:
            JSON-serialisable dict.
        """
        return {
            "key": candidate.payload.key,
            "payload": encode_payload(candidate.payload),
            "position": [float(c) for c in position],
            "quaternion": [float(c) for c in quaternion],
            "rms_px": float(pose.rms_px),
            "ratio": None if not np.isfinite(pose.ratio) else float(pose.ratio),
            "confidence": float(pose.confidence),
            "module_px": float(candidate.module_px),
            "chooser": CHOOSER_DEPTH if by_depth else CHOOSER_REPROJECTION,
        }
