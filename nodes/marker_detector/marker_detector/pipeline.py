"""Marker detection pipeline (no ROS): decode -> pose (ToF chooser) -> TF -> confidence gate -> commands and report."""

import logging
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import cv2
import numpy as np
from ros2_common.marker_payload import encode_payload

from marker_detector.config import MarkerDetectorConfig
from marker_detector.decode import Candidate, decode_frame, find_undecoded
from marker_detector.geometry import detection_report, marker_in_map, sighting_command, zoom_intrinsics, zoom_request
from marker_detector.pose import PoseResult, Solution, estimate_pose
from marker_detector.tof_plane import fit_plane, nearest_depth, plane_chooser
from marker_detector.zoom import ZoomFrame, ZoomPlanner, ZoomTarget

LOG = logging.getLogger(__name__)
REASON_TF = "tf"
REASON_LOW_CONFIDENCE = "low_confidence"
REASON_IMAGE = "image"
REASON_ZOOM_GEOMETRY = "zoom_geometry"
SOURCE_STREAM = "stream"
SOURCE_ZOOM = "zoom"
CHOOSER_REPROJECTION = "reprojection"
CHOOSER_TOF = "tof"
MAX_DEPTH_FRAMES = 8
MAX_ZOOM_RESULTS = 8
NS_PER_S = 1_000_000_000

TfLookup = Callable[[str, int], tuple | None]


@dataclass
class FrameOutput:
    """What one frame produced.

    commands: ``upsert_marker`` dicts for ``/poi/command``. report: ``/markers/detections`` JSON, only when at least one
    detection was accepted. zoom_request: zoom command dict or None. rejections: reason -> count.
    """

    commands: list[dict] = field(default_factory=list)
    report: str | None = None
    zoom_request: dict | None = None
    rejections: dict[str, int] = field(default_factory=dict)

    def reject(self, reason: str) -> None:
        """Count one rejection.

        Args:
            reason: Rejection category.
        """
        self.rejections[reason] = self.rejections.get(reason, 0) + 1


class MarkerPipeline:
    """Turns camera frames, depth frames and zoom stills into POI upsert commands (placeholder-free)."""

    def __init__(self, cfg: MarkerDetectorConfig, clock: Callable[[], float] = time.monotonic) -> None:
        """Create the pipeline.

        Args:
            cfg: Validated detector config.
            clock: Monotonic seconds source (rate limits of the zoom planner).
        """
        self.cfg = cfg
        self.clock = clock
        self.planner = ZoomPlanner(cfg.zoom, clock)
        self.K: np.ndarray | None = None
        self.D: np.ndarray | None = None
        self.size: tuple[int, int] | None = None
        self.frame_id = ""
        self.depth_K: np.ndarray | None = None
        self.depth_frames: deque[tuple[int, np.ndarray]] = deque(maxlen=MAX_DEPTH_FRAMES)
        self.zoom_results: dict[int, dict] = {}
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

    def on_depth(self, stamp_ns: int, depth_mm: np.ndarray, depth_K: np.ndarray) -> None:  # noqa: N803
        """Remember a registered depth frame (16UC1 mm) for ToF disambiguation.

        Args:
            stamp_ns: Depth stamp. depth_mm: (H, W) depth. depth_K: 3x3 depth intrinsics.
        """
        self.depth_K = np.asarray(depth_K, dtype=float).reshape(3, 3)
        self.depth_frames.append((int(stamp_ns), depth_mm))

    def process_frame(
        self,
        stamp_ns: int,
        jpeg: bytes,
        source: str,
        lookup_tf: TfLookup,
        K_override: np.ndarray | None = None,  # noqa: N803
    ) -> FrameOutput:
        """Process one compressed image.

        Args:
            stamp_ns: Image stamp. jpeg: compressed image bytes. source: ``stream`` or ``zoom``.
            lookup_tf: (frame_id, stamp_ns) -> (translation, quaternion xyzw) of the frame in map, or None.
            K_override: Intrinsics of this image instead of the stream camera_info (zoom stills).

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
        k = self.K if K_override is None else np.asarray(K_override, dtype=float)
        stream = source == SOURCE_STREAM
        candidates, rejected = decode_frame(grey)
        for rejection in rejected:
            out.reject(rejection.reason)
        items = []
        for candidate in candidates:
            item = self.accept(grey, candidate, k, stamp_ns, source, lookup_tf, out)
            if item is not None:
                items.append(item)
        if items:
            out.report = detection_report(stamp_ns, source, items)
        if stream:
            out.zoom_request = self.propose_zoom(grey, candidates)
        return out

    def accept(
        self,
        grey: np.ndarray,
        candidate: Candidate,
        k: np.ndarray,
        stamp_ns: int,
        source: str,
        lookup_tf: TfLookup,
        out: FrameOutput,
    ) -> dict | None:
        """Pose, TF and gate one candidate; append its command to ``out`` when accepted.

        Args:
            grey: Grey image. candidate: decoded marker. k: intrinsics of ``grey``. stamp_ns: image stamp.
            source: stream or zoom. lookup_tf: TF lookup. out: frame output to extend.

        Returns:
            The report item, or None when the candidate was rejected.
        """
        used_tof = []
        choose = self.tof_chooser(candidate, stamp_ns, used_tof) if source == SOURCE_STREAM else None
        pose = estimate_pose(grey, candidate, k, self.D, self.cfg.pose, choose)
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
        return self.report_item(candidate, pose, position, quaternion, bool(used_tof))

    def tof_chooser(
        self, candidate: Candidate, stamp_ns: int, used: list[bool]
    ) -> Callable[[list[Solution]], Solution | None] | None:
        """Disambiguation callback using the depth frame nearest in time, or None when ToF cannot apply.

        Args:
            candidate: The decoded marker (its corners are the polygon). stamp_ns: image stamp.
            used: appended with True when the chooser settled the ambiguity.

        Returns:
            Callback for ``estimate_pose`` (fits the plane lazily, only when the pose is ambiguous) or None.
        """
        tof = self.cfg.tof
        if not tof.enabled or self.depth_K is None or self.size is None:
            return None
        frame = nearest_depth(self.depth_frames, stamp_ns, tof.max_dt_s)
        if frame is None:
            return None
        depth, depth_k, size = frame[1], self.depth_K, self.size

        def choose(solutions: list[Solution]) -> Solution | None:
            plane = fit_plane(depth, depth_k, candidate.corners, size, tof)
            chosen = plane_chooser(plane, tof)(solutions) if plane is not None else None
            if chosen is not None:
                used.append(True)
            return chosen

        return choose

    def report_item(
        self, candidate: Candidate, pose: PoseResult, position: np.ndarray, quaternion: np.ndarray, tof: bool
    ) -> dict:
        """Per marker entry of the detections report.

        Args:
            candidate: decoded marker. pose: accepted pose. position: map position. quaternion: map orientation.
            tof: True when the ToF plane chose the solution.

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
            "chooser": CHOOSER_TOF if tof else CHOOSER_REPROJECTION,
        }

    def propose_zoom(self, grey: np.ndarray, candidates: list[Candidate]) -> dict | None:
        """Ask the planner for one zoom still of a small decoded marker or an undecoded QR quad.

        Args:
            grey: Stream grey image. candidates: markers decoded from it.

        Returns:
            The zoom command dict (already registered as outstanding), or None.
        """
        zoom = self.cfg.zoom
        now = self.clock()
        self.planner.expire(now)
        if not zoom.auto_zoom or self.size is None:
            return None
        width, height = self.size
        targets = [
            ZoomTarget(float(c.corners[:, 0].mean()) / width, float(c.corners[:, 1].mean()) / height, c.module_px)
            for c in candidates
            if c.module_px < zoom.min_module_px
        ]
        for quad in find_undecoded(grey, candidates):
            targets.append(ZoomTarget(float(quad[:, 0].mean()) / width, float(quad[:, 1].mean()) / height))
        target = self.planner.propose(targets, now)
        if target is None:
            return None
        self.request_count += 1
        request = zoom_request(target.u, target.v, target.module_px, zoom, f"marker-zoom-{self.request_count}")
        self.planner.sent(request["request_id"], target, now)
        return request

    def on_zoom_result(self, result: dict) -> tuple[ZoomFrame, dict] | None:
        """Handle a zoom result message.

        Args:
            result: Parsed result dict.

        Returns:
            (frame, result) when the matching image already arrived, else None (the image will complete it).
        """
        pending = self.planner.on_result(result, self.clock())
        if pending is None:
            return None
        if pending.frame is not None:
            return pending.frame, result
        self.zoom_results[pending.stamp_ns] = result
        while len(self.zoom_results) > MAX_ZOOM_RESULTS:
            del self.zoom_results[next(iter(self.zoom_results))]
        return None

    def on_zoom_image(self, stamp_ns: int, jpeg: bytes) -> tuple[ZoomFrame, dict] | None:
        """Handle a zoom still image.

        Args:
            stamp_ns: image stamp (equals the result's ``stamp_ns``). jpeg: compressed bytes.

        Returns:
            (frame, result) when a result matches, else None.
        """
        frame = self.planner.on_image(stamp_ns, jpeg)
        if frame is None:
            return None
        return frame, self.zoom_results.pop(stamp_ns, {})

    def process_zoom(self, frame: ZoomFrame, result: dict, lookup_tf: TfLookup) -> FrameOutput:
        """Process a zoom still with intrinsics derived from the stream camera_info and the result's crop.

        Args:
            frame: still paired with its result. result: result dict (``crop`` [x, y, w, h], ``full`` [w, h]).
            lookup_tf: TF lookup.

        Returns:
            FrameOutput (empty without camera_info; a ``zoom_geometry`` rejection without a usable crop).
        """
        out = FrameOutput()
        if self.K is None or self.size is None:
            return out
        grey = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        crop = result.get("crop")
        if grey is None:
            out.reject(REASON_IMAGE)
            return out
        if not crop or len(crop) != 4 or min(crop[2], crop[3]) <= 0:
            out.reject(REASON_ZOOM_GEOMETRY)
            return out
        full = tuple(result.get("full") or self.cfg.zoom.full_size)
        k = zoom_intrinsics(self.K, self.size, (full[0], full[1]), tuple(crop), (grey.shape[1], grey.shape[0]))
        return self.process_frame(frame.stamp_ns, frame.jpeg, SOURCE_ZOOM, lookup_tf, K_override=k)
