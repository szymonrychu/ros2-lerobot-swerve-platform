"""Head camera target tracker of the servoed grasp approach: visibility precheck and template-matching detections.

Pure module: frames, scenes (camera_scene.Scene), base poses and the surface height are passed in, nothing is read
from ROS here. The tracker turns each camera frame into at most one real detection of the grasp target:

1. predict the target pixel from the current target estimate and the base pose at the frame's capture stamp;
2. rescale the seed template by the predicted camera distance ratio (seed distance / current distance);
3. ``cv2.matchTemplate`` (TM_CCOEFF_NORMED) in a ``search_px`` window around the prediction;
4. accept the best match only with a score >= ``min_score``;
5. cut the matched pixel's ray with the surface plane (``Scene.ground_point``) and take it to the map at that stamp;
6. reject it when it lies farther than ``max_correction_m`` from the planned target (counted in ``rejected``);
7. return the dock pose ``target - R(dock_yaw) t_rel`` stamped with the capture stamp.

No accepted match means no Detection: nothing is published for that frame, and a stored position is never
re-stamped as a fresh detection (CLAUDE.md, no placeholder data).

The frames come from the head D435 colour camera (camera_scene 'head'). Once seeded, a target whose predicted position
lies inside the D435 min-Z (D435_MIN_Z_M of depth along the optical axis) or outside the colour image is lost: the
tracker latches the reason in ``lost`` and returns no detection from then on, and the DetectionLoop reports it once
(``on_lost``) and ends, so the approach stops with that reason instead of servoing on a guessed pixel.
"""

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.arm_pose_check import D435_MIN_Z_M

from .camera_scene import Scene, base_to_map, transform_point
from .config import TrackerSettings
from .models import BasePose
from .topdown import map_to_base

LOGGER = logging.getLogger(__name__)
FRAME_MAP = "map"
MIN_TEMPLATE_PX = 8
MIN_TEMPLATE_STD = 2.0  # grey levels: below this the patch has no texture to match
JOIN_TIMEOUT_S = 2.0
LOST_MIN_Z = f"inside the D435 min-Z ({D435_MIN_Z_M:g} m)"
LOST_FOV = "out of the D435 colour FOV"


@dataclass(frozen=True)
class Detection:
    """One accepted target detection.

    Attributes:
        pose (BasePose): Dock pose in the map (what the docking server consumes).
        stamp_ns (int): Capture stamp of the frame the target was found in (ns since the epoch).
        target_map (tuple[float, float]): Detected target position in the map (m).
        pixel (tuple[float, float]): Matched target pixel (u, v).
        score (float): Normalized cross-correlation score of the match.
    """

    pose: BasePose
    stamp_ns: int
    target_map: tuple[float, float]
    pixel: tuple[float, float]
    score: float


@dataclass(frozen=True)
class TrackerInput:
    """One frame and the geometry valid at its capture stamp, as the DetectionLoop source provides it.

    Attributes:
        frame_bgr (np.ndarray): Camera image (BGR or grayscale).
        stamp_ns (int): Capture stamp (ns since the epoch).
        scene (Scene): Camera scene at the capture stamp.
        base_pose (BasePose): map -> base_link at the capture stamp.
        surface_z (float): Height of the target surface above the floor (m).
    """

    frame_bgr: np.ndarray
    stamp_ns: int
    scene: Scene
    base_pose: BasePose
    surface_z: float


def optical_depth(scene: Scene, point_ref: np.ndarray) -> float:
    """Depth of a reference-frame point along the camera's optical axis (the optical frame z, what min-Z bounds).

    Args:
        scene (Scene): Camera scene.
        point_ref (np.ndarray): Point (3,) in the scene's reference frame.

    Returns:
        float: Depth (m); negative behind the camera.
    """
    return float(transform_point(np.linalg.inv(scene.t_ref_optical), point_ref)[2])


def lost_reason(scene: Scene, point_ref: np.ndarray) -> str | None:
    """Why the D435 colour camera cannot see a reference-frame point, None when it projects into the image.

    Args:
        scene (Scene): Camera scene.
        point_ref (np.ndarray): Point (3,) in the scene's reference frame.

    Returns:
        str | None: LOST_MIN_Z when it lies in front of the camera but closer than D435_MIN_Z_M of depth, LOST_FOV when
            it is behind the camera or off the image, None when in view.
    """
    if 0.0 < optical_depth(scene, point_ref) < D435_MIN_Z_M:
        return LOST_MIN_Z
    if scene.project(point_ref) is None:
        return LOST_FOV
    return None


def visible(scene: Scene, point_base_link: np.ndarray, edge_margin_px: float) -> bool:
    """Whether a base_link point lies beyond the D435 min-Z and projects edge_margin_px inside every image border.

    Args:
        scene (Scene): Camera scene.
        point_base_link (np.ndarray): Point (3,) in base_link (z = 0 is the floor).
        edge_margin_px (float): Required distance from the image borders (px).

    Returns:
        bool: True when in view with the margin; False when behind the camera, inside the min-Z, off the image,
            inside the margin or when the scene cannot express base_link points.
    """
    ref = scene.base_to_ref(np.asarray(point_base_link, dtype=np.float64))
    if ref is None or lost_reason(scene, ref) is not None:
        return False
    pixel = scene.project(ref)
    if pixel is None:
        return False
    u, v = pixel
    w, h = scene.intr.width, scene.intr.height
    return edge_margin_px <= u <= w - 1 - edge_margin_px and edge_margin_px <= v <= h - 1 - edge_margin_px


def dock_pose_from_target(
    target_map: tuple[float, float] | np.ndarray, dock_yaw: float, t_rel: tuple[float, float]
) -> BasePose:
    """Dock pose that puts the target at t_rel in the dock (base_link) frame: target - R(dock_yaw) t_rel.

    Args:
        target_map (tuple[float, float] | np.ndarray): Target (x, y[, z]) in the map.
        dock_yaw (float): Planned dock heading in the map (rad).
        t_rel (tuple[float, float]): Target (x, y) in the planned dock frame (m).

    Returns:
        BasePose: The dock pose in the map frame.
    """
    c, s = math.cos(dock_yaw), math.sin(dock_yaw)
    rx, ry = float(t_rel[0]), float(t_rel[1])
    return BasePose(
        frame=FRAME_MAP,
        x=float(target_map[0]) - (c * rx - s * ry),
        y=float(target_map[1]) - (s * rx + c * ry),
        yaw=dock_yaw,
    )


def camera_distance(scene: Scene, point_ref: np.ndarray) -> float:
    """Distance from the camera's optical centre to a reference-frame point.

    Args:
        scene (Scene): Camera scene.
        point_ref (np.ndarray): Point (3,) in the scene's reference frame.

    Returns:
        float: Distance (m).
    """
    return float(np.linalg.norm(np.asarray(point_ref, dtype=np.float64) - scene.t_ref_optical[:3, 3]))


def to_gray(frame: np.ndarray) -> np.ndarray:
    """Grayscale view of a BGR (or already grayscale) frame.

    Args:
        frame (np.ndarray): HxWx3 BGR or HxW grayscale image.

    Returns:
        np.ndarray: HxW grayscale image.
    """
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def subpixel_peak(res: np.ndarray, x: int, y: int) -> tuple[float, float]:
    """Refine an integer correlation peak with a 1D parabola fit per axis.

    Args:
        res (np.ndarray): matchTemplate result.
        x (int): Peak column.
        y (int): Peak row.

    Returns:
        tuple[float, float]: (x, y) refined.
    """

    def offset(lo: float, mid: float, hi: float) -> float:
        denom = lo - 2.0 * mid + hi
        return 0.0 if denom >= 0.0 else float(np.clip(0.5 * (lo - hi) / denom, -0.5, 0.5))

    fx, fy = float(x), float(y)
    if 0 < x < res.shape[1] - 1:
        fx += offset(float(res[y, x - 1]), float(res[y, x]), float(res[y, x + 1]))
    if 0 < y < res.shape[0] - 1:
        fy += offset(float(res[y - 1, x]), float(res[y, x]), float(res[y + 1, x]))
    return fx, fy


class TargetTracker:
    """Template tracker of one grasp target on a surface, producing dock-pose detections.

    Attributes:
        planned_target (tuple[float, float]): Planned target (x, y) in the map.
        dock_yaw (float): Planned dock heading (rad).
        t_rel (tuple[float, float]): Target in the planned dock frame (m).
        settings (TrackerSettings): Tracker settings.
        estimate (tuple[float, float]): Current target estimate in the map (planned target until a seed or match).
        seeded (bool): Whether a template has been taken.
        frames (int): step() calls.
        accepted (int): Detections returned.
        rejected (int): Matches rejected for lying farther than max_correction_m from the planned target.
        low_score (int): Frames whose best match scored below min_score.
        last_scale (float | None): Template scale used by the last match attempt.
        lost (str | None): Why the seeded target left the camera's view (LOST_MIN_Z, LOST_FOV); latched, None while
            tracked.
    """

    def __init__(
        self,
        planned_target: tuple[float, float],
        dock_yaw: float,
        t_rel: tuple[float, float],
        settings: TrackerSettings,
    ) -> None:
        """Create an unseeded tracker.

        Args:
            planned_target (tuple[float, float]): Planned target (x, y) in the map.
            dock_yaw (float): Planned dock heading (rad).
            t_rel (tuple[float, float]): Target in the planned dock frame (m).
            settings (TrackerSettings): Tracker settings.
        """
        self.planned_target = (float(planned_target[0]), float(planned_target[1]))
        self.dock_yaw = dock_yaw
        self.t_rel = t_rel
        self.settings = settings
        self.estimate = self.planned_target
        self.template: np.ndarray | None = None
        self.seed_distance = 0.0
        self.frames = 0
        self.accepted = 0
        self.rejected = 0
        self.low_score = 0
        self.last_scale: float | None = None
        self.lost: str | None = None

    @property
    def seeded(self) -> bool:
        """Whether a template has been taken.

        Returns:
            bool: True once seeded.
        """
        return self.template is not None

    def target_ref(self, scene: Scene, base_pose: BasePose, surface_z: float) -> np.ndarray | None:
        """The current target estimate in the scene's reference frame.

        Args:
            scene (Scene): Camera scene at the frame's stamp.
            base_pose (BasePose): map -> base_link at the frame's stamp.
            surface_z (float): Surface height above the floor (m).

        Returns:
            np.ndarray | None: Point in the reference frame, None when the scene cannot express base_link points.
        """
        bx, by = map_to_base(*self.estimate, (base_pose.x, base_pose.y, base_pose.yaw))
        return scene.base_to_ref(np.array([bx, by, surface_z]))

    def pixel_to_map(
        self, scene: Scene, base_pose: BasePose, u: float, v: float, surface_z: float
    ) -> tuple[tuple[float, float], np.ndarray] | None:
        """Map point and reference-frame point of a pixel on the target surface.

        Args:
            scene (Scene): Camera scene.
            base_pose (BasePose): map -> base_link at the frame's stamp.
            u (float): Pixel x.
            v (float): Pixel y.
            surface_z (float): Surface height above the floor (m).

        Returns:
            tuple[tuple[float, float], np.ndarray] | None: ((x, y) in the map, point in the reference frame), None when
                the ray misses the surface or the scene cannot reach base_link.
        """
        ground = scene.ground_point(u, v, surface_z)
        if ground is None:
            return None
        base = scene.ref_to_base(ground)
        if base is None:
            return None
        return base_to_map(base_pose, float(base[0]), float(base[1])), ground

    def seed_from_pixel(
        self, frame_bgr: np.ndarray, u: float, v: float, scene: Scene, base_pose: BasePose, surface_z: float
    ) -> bool:
        """Take the template around a pixel (the agent's pixel, or the projected target) of a frame.

        Args:
            frame_bgr (np.ndarray): The frame the pixel refers to.
            u (float): Pixel x.
            v (float): Pixel y.
            scene (Scene): Camera scene at that frame's stamp.
            base_pose (BasePose): map -> base_link at that frame's stamp.
            surface_z (float): Surface height above the floor (m).

        Returns:
            bool: True when seeded; False when the patch leaves the image, has no texture or misses the surface.
        """
        gray = to_gray(frame_bgr)
        size = self.settings.template_px
        u0 = round(u - (size - 1) / 2)
        v0 = round(v - (size - 1) / 2)
        if u0 < 0 or v0 < 0 or u0 + size > gray.shape[1] or v0 + size > gray.shape[0]:
            return False
        patch = gray[v0 : v0 + size, u0 : u0 + size].copy()
        if float(patch.std()) < MIN_TEMPLATE_STD:
            return False
        centre = self.pixel_to_map(scene, base_pose, u0 + (size - 1) / 2, v0 + (size - 1) / 2, surface_z)
        if centre is None:
            return False
        self.estimate, ground = centre
        self.seed_distance = camera_distance(scene, ground)
        self.template = patch
        return True

    def seed_from_projection(self, frame_bgr: np.ndarray, scene: Scene, base_pose: BasePose, surface_z: float) -> bool:
        """Seed around the projected planned target when it is in view with room for the template.

        Args:
            frame_bgr (np.ndarray): Frame.
            scene (Scene): Camera scene at the frame's stamp.
            base_pose (BasePose): map -> base_link at the frame's stamp.
            surface_z (float): Surface height above the floor (m).

        Returns:
            bool: True when seeded.
        """
        bx, by = map_to_base(*self.planned_target, (base_pose.x, base_pose.y, base_pose.yaw))
        point = np.array([bx, by, surface_z])
        margin = max(self.settings.edge_margin_px, self.settings.template_px / 2)
        if not visible(scene, point, margin):
            return False
        ref = scene.base_to_ref(point)
        assert ref is not None  # visible() already required it
        pixel = scene.project(ref)
        assert pixel is not None
        return self.seed_from_pixel(frame_bgr, pixel[0], pixel[1], scene, base_pose, surface_z)

    def step(
        self, frame_bgr: np.ndarray, stamp: int, scene: Scene, base_pose: BasePose, surface_z: float
    ) -> Detection | None:
        """Track the target in one frame.

        An unseeded tracker seeds from this frame when the planned target is in view and returns None (a template
        matched against its own frame would only echo the plan). A seeded tracker whose predicted target lies inside
        the D435 min-Z or outside the image latches the reason in ``lost``; a lost tracker returns None for every
        later frame.

        Args:
            frame_bgr (np.ndarray): Camera image.
            stamp (int): Capture stamp of the frame (ns since the epoch); the detection carries it unchanged.
            scene (Scene): Camera scene at the capture stamp.
            base_pose (BasePose): map -> base_link at the capture stamp.
            surface_z (float): Surface height above the floor (m).

        Returns:
            Detection | None: The detection, or None when nothing was accepted (nothing to publish).
        """
        self.frames += 1
        if self.template is None:
            self.seed_from_projection(frame_bgr, scene, base_pose, surface_z)
            return None
        if self.lost is not None:
            return None
        ref = self.target_ref(scene, base_pose, surface_z)
        if ref is not None:
            self.lost = lost_reason(scene, ref)
            if self.lost is not None:
                return None
        match = self.match(to_gray(frame_bgr), scene, base_pose, surface_z)
        if match is None:
            return None
        (u, v), score = match
        if score < self.settings.min_score:
            self.low_score += 1
            return None
        located = self.pixel_to_map(scene, base_pose, u, v, surface_z)
        if located is None:
            return None
        target, _ = located
        if math.dist(target, self.planned_target) > self.settings.max_correction_m:
            self.rejected += 1
            return None
        self.estimate = target
        self.accepted += 1
        return Detection(
            pose=dock_pose_from_target(target, self.dock_yaw, self.t_rel),
            stamp_ns=int(stamp),
            target_map=target,
            pixel=(u, v),
            score=score,
        )

    def match(
        self, gray: np.ndarray, scene: Scene, base_pose: BasePose, surface_z: float
    ) -> tuple[tuple[float, float], float] | None:
        """Best template match around the predicted target pixel.

        Args:
            gray (np.ndarray): Grayscale frame.
            scene (Scene): Camera scene at the frame's stamp.
            base_pose (BasePose): map -> base_link at the frame's stamp.
            surface_z (float): Surface height above the floor (m).

        Returns:
            tuple[tuple[float, float], float] | None: ((u, v) of the template centre, score), None when the target
                is out of view or the window cannot hold the template.
        """
        assert self.template is not None
        ref = self.target_ref(scene, base_pose, surface_z)
        if ref is None:
            return None
        predicted = scene.project(ref)
        if predicted is None:
            return None
        scale = self.seed_distance / camera_distance(scene, ref)
        self.last_scale = scale
        size = round(self.template.shape[0] * scale)
        if size < MIN_TEMPLATE_PX:
            return None
        template = cv2.resize(
            self.template, (size, size), interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        )
        side = max(self.settings.search_px, size + self.settings.search_px - self.settings.template_px)
        x0 = max(0, round(predicted[0] - side / 2))
        y0 = max(0, round(predicted[1] - side / 2))
        window = gray[y0 : min(gray.shape[0], y0 + side), x0 : min(gray.shape[1], x0 + side)]
        if window.shape[0] < size or window.shape[1] < size:
            return None
        res = cv2.matchTemplate(window, template, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(res)
        if not math.isfinite(score):
            return None
        px, py = subpixel_peak(res, loc[0], loc[1])
        return (x0 + px + (size - 1) / 2, y0 + py + (size - 1) / 2), float(score)


class DetectionLoop:
    """Runs a TargetTracker on fresh frames at rate_hz in a thread and publishes only accepted detections.

    The source returns the newest frame with its geometry (or None when there is none); a frame whose stamp is not
    newer than the last processed one is skipped, so a detection is never re-published with an old stamp. When the
    tracker loses the target the loop calls on_lost once with the reason and ends.

    Attributes:
        tracker (TargetTracker): The tracker.
        published (int): Detections handed to the publish callback.
        errors (int): Source, tracking or publish exceptions (logged, the loop keeps running).
    """

    def __init__(
        self,
        tracker: TargetTracker,
        source: Callable[[], TrackerInput | None],
        publish: Callable[[Detection], None],
        rate_hz: float,
        on_lost: Callable[[str], None] | None = None,
    ) -> None:
        """Create a stopped loop.

        Args:
            tracker (TargetTracker): The tracker.
            source (Callable[[], TrackerInput | None]): Newest frame with its geometry, None when unavailable.
            publish (Callable[[Detection], None]): Publishes one detection (e.g. DetectionPublisher.publish).
            rate_hz (float): Loop rate (Hz).
            on_lost (Callable[[str], None] | None): Called once with TargetTracker.lost when the target is lost.
        """
        if rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz}")
        self.tracker = tracker
        self.source = source
        self.publish = publish
        self.on_lost = on_lost
        self.period_s = 1.0 / rate_hz
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.published = 0
        self.errors = 0
        self.last_stamp: int | None = None

    @property
    def running(self) -> bool:
        """Whether the loop thread is alive.

        Returns:
            bool: True while running.
        """
        return self.thread is not None and self.thread.is_alive()

    def start(self) -> None:
        """Start the loop thread (no-op when already running)."""
        if self.running:
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.run, name="target-tracker", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        """Signal the loop to stop and wait for the thread to end."""
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=JOIN_TIMEOUT_S)

    def __enter__(self) -> "DetectionLoop":
        """Start the loop.

        Returns:
            DetectionLoop: This loop.
        """
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        """Stop the loop."""
        self.stop()

    def tick(self) -> None:
        """Process the newest frame once: track it and publish an accepted detection, or report a lost target and end
        the loop."""
        sample = self.source()
        if sample is None:
            return
        if self.last_stamp is not None and sample.stamp_ns <= self.last_stamp:
            return
        self.last_stamp = sample.stamp_ns
        detection = self.tracker.step(
            sample.frame_bgr, sample.stamp_ns, sample.scene, sample.base_pose, sample.surface_z
        )
        if self.tracker.lost is not None:
            self.stop_event.set()
            if self.on_lost is not None:
                self.on_lost(self.tracker.lost)
            return
        if detection is None:
            return
        self.publish(detection)
        self.published += 1

    def run(self) -> None:
        """Thread body: tick at the loop rate until the stop event is set."""
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                self.tick()
            except Exception:
                self.errors += 1
                LOGGER.exception("target tracker cycle failed")
            self.stop_event.wait(max(0.0, self.period_s - (time.monotonic() - started)))
