"""Frame capture (PNG frames, GIF) and an interactive passive viewer for plan replays."""

import time
from pathlib import Path

import mujoco
import mujoco.viewer
from PIL import Image

CAMERA_AZIMUTH_DEG = 150.0
CAMERA_ELEVATION_DEG = -25.0
CAMERA_DISTANCE_M = 0.85
CAMERA_LOOKAT = (0.15, 0.0, -0.05)
DEFAULT_FPS = 25.0
DEFAULT_SIZE = (640, 480)
VIEWER_POLL_S = 0.05


def make_camera() -> mujoco.MjvCamera:
    """Free camera looking at the arm workspace from the front-left.

    Returns:
        mujoco.MjvCamera: Camera for Renderer.update_scene.
    """
    cam = mujoco.MjvCamera()
    cam.azimuth = CAMERA_AZIMUTH_DEG
    cam.elevation = CAMERA_ELEVATION_DEG
    cam.distance = CAMERA_DISTANCE_M
    cam.lookat[:] = CAMERA_LOOKAT
    return cam


class FrameSaver:
    """simulate() observer that renders offscreen frames at a fixed rate and writes PNGs and/or a GIF."""

    def __init__(
        self,
        frames_dir: Path | None = None,
        gif_path: Path | None = None,
        fps: float = DEFAULT_FPS,
        size: tuple[int, int] = DEFAULT_SIZE,
    ) -> None:
        """Configure outputs; the renderer is created on the first frame.

        Args:
            frames_dir (Path | None): Directory for numbered PNG frames (created if missing).
            gif_path (Path | None): Output animated GIF.
            fps (float): Capture rate in simulated frames per second.
            size (tuple[int, int]): Frame (width, height) in px.
        """
        self.frames_dir = frames_dir
        self.gif_path = gif_path
        self.period = 1.0 / fps
        self.size = size
        self.images: list[Image.Image] = []
        self.next_t: float | None = None
        self.count = 0
        self.renderer: mujoco.Renderer | None = None
        self.camera = make_camera()
        if frames_dir is not None:
            frames_dir.mkdir(parents=True, exist_ok=True)

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, t: float, label: str | None) -> None:
        """Capture a frame when the capture period has elapsed.

        Args:
            model (mujoco.MjModel): Model being simulated.
            data (mujoco.MjData): Current state.
            t (float): Plan time in s.
            label (str | None): Current segment label (unused, part of the observer signature).
        """
        if self.next_t is None:
            self.next_t = t
        if t < self.next_t:
            return
        self.next_t += self.period
        if self.renderer is None:
            self.renderer = mujoco.Renderer(model, self.size[1], self.size[0])
        self.renderer.update_scene(data, self.camera)
        image = Image.fromarray(self.renderer.render())
        if self.frames_dir is not None:
            image.save(self.frames_dir / f"frame_{self.count:05d}.png")
        if self.gif_path is not None:
            self.images.append(image)
        self.count += 1

    def close(self) -> None:
        """Write the GIF (if requested) and release the renderer."""
        if self.gif_path is not None and self.images:
            self.images[0].save(
                self.gif_path,
                save_all=True,
                append_images=self.images[1:],
                duration=int(self.period * 1000),
                loop=0,
            )
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class ViewerObserver:
    """simulate() observer that mirrors the replay into mujoco.viewer.launch_passive, paced to real time.

    On macOS the viewer needs the mjpython launcher (uv run mjpython -m grasp_sim.cli run ... --render).
    """

    def __init__(self, hold_open: bool = True) -> None:
        """Create an observer; the window opens on the first step.

        Args:
            hold_open (bool): Keep the window open after the replay until it is closed.
        """
        self.hold_open = hold_open
        self.viewer: mujoco.viewer.Handle | None = None
        self.last_wall: float | None = None
        self.last_t: float | None = None

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, t: float, label: str | None) -> None:
        """Sync the viewer and sleep so the replay runs in real time.

        Args:
            model (mujoco.MjModel): Model being simulated.
            data (mujoco.MjData): Current state.
            t (float): Plan time in s.
            label (str | None): Current segment label (unused, part of the observer signature).
        """
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(model, data)
            self.last_wall = time.monotonic()
        if self.last_t is not None and self.last_wall is not None:
            wait = (t - self.last_t) - (time.monotonic() - self.last_wall)
            if wait > 0:
                time.sleep(wait)
        self.last_t, self.last_wall = t, time.monotonic()
        self.viewer.sync()

    def close(self) -> None:
        """Wait for the window to be closed (if hold_open) and release it."""
        if self.viewer is None:
            return
        while self.hold_open and self.viewer.is_running():
            time.sleep(VIEWER_POLL_S)
        self.viewer.close()
        self.viewer = None
