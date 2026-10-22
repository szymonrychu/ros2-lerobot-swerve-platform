#!/usr/bin/env python3
"""Record the ground-detection scenes from the Mac through the web_ui of the robot.

Per scene the head is pointed with a websocket publish, the script waits until /head/joint_states reports the
target, then starts a recorder run (POST /api/ground/recordings), captures the configured topics to
`<out_dir>/<name>.ndjson` (`{t, topic, data}` lines, as scripts/ground_noise_report.py reads them), downloads
`<out_dir>/<name>.npz` and deletes it on the robot. Only the head command is ever published; no rclpy, no SSH.

Usage:
    python scripts/ground_record.py [config.yaml]    # default scripts/ground_record.yaml
"""

import argparse
import asyncio
import json
import logging
import math
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Protocol

import httpx
import websockets
import yaml
from pydantic import BaseModel, ConfigDict, Field

LOG = logging.getLogger("ground_record")

DEFAULT_CONFIG = Path(__file__).with_name("ground_record.yaml")
HEAD_MSG_TYPE = "sensor_msgs/JointState"
HEAD_JOINTS = ("head_pan", "head_tilt")
RECORDINGS_PATH = "/api/ground/recordings"
POLL_INTERVAL_S = 2.0
DONE_TIMEOUT_MARGIN_S = 60.0
HTTP_TIMEOUT_S = 60.0
EXIT_OK = 0
EXIT_FAIL = 1

STEP_PUBLISH = "publish"
STEP_SETTLE = "settle"
STEP_START = "start"
STEP_CAPTURE = "capture"
STEP_DOWNLOAD = "download"
STEP_DELETE = "delete"


class Strict(BaseModel):
    """Base model rejecting unknown keys."""

    model_config = ConfigDict(extra="forbid")


class Scene(Strict):
    """One head pose to record."""

    name: str
    pan_deg: float
    tilt_deg: float
    settle_s: float = 10
    duration_s: float = 30
    capture_topics: list[str] = Field(default_factory=lambda: ["/ground/state"])


def default_scenes() -> list[Scene]:
    """Return the home, box, ramp and empty starting scenes."""
    return [
        Scene(name="home", pan_deg=0, tilt_deg=29.4),
        Scene(name="box", pan_deg=35, tilt_deg=29.4),
        Scene(name="ramp", pan_deg=-30, tilt_deg=29.4),
        Scene(name="empty", pan_deg=-60, tilt_deg=29.4),
    ]


class RecordConfig(Strict):
    """Script configuration."""

    web_ui_url: str = "http://client.ros2.lan:8080"
    out_dir: Path = Path(".logs/ground_recordings")
    head_command_topic: str = "/head/joint_commands"
    head_js_topic: str = "/head/joint_states"
    settle_tolerance_deg: float = 1.0
    settle_timeout_s: float = 15
    scenes: list[Scene] = Field(default_factory=default_scenes)


class SceneError(Exception):
    """A scene failed at a named step."""

    def __init__(self, scene: str, step: str, reason: str) -> None:
        super().__init__(f"scene {scene!r} failed at step {step!r}: {reason}")
        self.scene = scene
        self.step = step
        self.reason = reason


class Socket(Protocol):
    """The slice of a websocket connection the script uses."""

    async def send(self, message: str) -> None: ...

    async def recv(self) -> str | bytes: ...


def load_config(path: Path) -> RecordConfig:
    """Load and validate the YAML config.

    Args:
        path (Path): YAML file.

    Returns:
        RecordConfig: validated config.
    """
    return RecordConfig.model_validate(yaml.safe_load(path.read_text()) or {})


def head_command(scene: Scene, topic: str) -> str:
    """Build the websocket publish frame the web UI head tab sends.

    Args:
        scene (Scene): target pose in degrees.
        topic (str): head command topic.

    Returns:
        str: JSON frame with a JointState of head_pan and head_tilt in rad.
    """
    position = [math.radians(scene.pan_deg), math.radians(scene.tilt_deg)]
    data = {"name": list(HEAD_JOINTS), "position": position}
    return json.dumps({"type": "publish", "topic": topic, "msg_type": HEAD_MSG_TYPE, "data": data})


def head_at_target(data: Any, scene: Scene, tolerance_deg: float) -> bool:
    """Return True when a head JointState puts both joints within the tolerance of the scene.

    Args:
        data (Any): JointState payload ({name, position}).
        scene (Scene): target pose.
        tolerance_deg (float): allowed error per joint.

    Returns:
        bool: whether pan and tilt are both on target.
    """
    if not isinstance(data, dict):
        return False
    names, positions = data.get("name"), data.get("position")
    if not isinstance(names, list) or not isinstance(positions, list) or len(names) != len(positions):
        return False
    measured = dict(zip(names, positions, strict=True))
    targets = {"head_pan": scene.pan_deg, "head_tilt": scene.tilt_deg}
    return all(
        isinstance(measured.get(joint), int | float) and abs(math.degrees(measured[joint]) - target) <= tolerance_deg
        for joint, target in targets.items()
    )


async def wait_settled(scene: Scene, config: RecordConfig, ws: Socket) -> None:
    """Wait until the head is on target, then settle_s more.

    Args:
        scene (Scene): target pose.
        config (RecordConfig): script config.
        ws (Socket): websocket subscribed to the head joint states.

    Raises:
        SceneError: on a web_ui error frame (refused command) or when the head misses the timeout.
    """

    async def until_on_target() -> None:
        while True:
            message = json.loads(await ws.recv())
            if message.get("type") == "error":
                raise SceneError(scene.name, STEP_SETTLE, str(message.get("message", message)))
            if message.get("topic") == config.head_js_topic and head_at_target(
                message.get("data"), scene, config.settle_tolerance_deg
            ):
                return

    try:
        await asyncio.wait_for(until_on_target(), timeout=config.settle_timeout_s)
    except TimeoutError:
        raise SceneError(
            scene.name, STEP_SETTLE, f"head not within {config.settle_tolerance_deg} deg of target in time"
        ) from None
    await drain(ws, scene.settle_s)


async def drain(ws: Socket, duration_s: float) -> None:
    """Read and discard websocket frames for a duration so none queue up unread.

    Args:
        ws (Socket): websocket to keep reading.
        duration_s (float): how long to keep draining, in seconds.
    """

    async def discard() -> None:
        while True:
            await ws.recv()

    try:
        await asyncio.wait_for(discard(), timeout=duration_s)
    except TimeoutError:
        pass


async def capture_lines(scene: Scene, ws: Socket, path: Path) -> None:
    """Write the captured topics' websocket lines to an NDJSON file until cancelled.

    Args:
        scene (Scene): scene whose capture_topics are kept.
        ws (Socket): websocket subscribed to the capture topics.
        path (Path): output NDJSON file.
    """
    with path.open("w") as out:
        while True:
            message = json.loads(await ws.recv())
            topic = message.get("topic")
            if topic in scene.capture_topics:
                out.write(json.dumps({"t": time.time(), "topic": topic, "data": message.get("data")}) + "\n")
                out.flush()


def check(response: httpx.Response, scene: Scene, step: str) -> httpx.Response:
    """Raise a SceneError naming the step when the HTTP response is not a success."""
    if response.is_success:
        return response
    raise SceneError(scene.name, step, f"HTTP {response.status_code}: {response.text[:200]}")


async def wait_done(scene: Scene, client: httpx.AsyncClient, poll_interval_s: float) -> None:
    """Poll the recording list until the scene's recording is done.

    Raises:
        SceneError: when it does not finish within duration_s plus a margin.
    """

    async def poll() -> None:
        while True:
            await asyncio.sleep(poll_interval_s)
            listing = check(await client.get(RECORDINGS_PATH), scene, STEP_CAPTURE).json()
            if any(info.get("name") == scene.name and info.get("done") for info in listing):
                return

    try:
        await asyncio.wait_for(poll(), timeout=scene.duration_s + DONE_TIMEOUT_MARGIN_S)
    except TimeoutError:
        raise SceneError(scene.name, STEP_CAPTURE, "recording did not finish") from None


async def record_scene(
    scene: Scene,
    config: RecordConfig,
    ws: Socket,
    client: httpx.AsyncClient,
    poll_interval_s: float = POLL_INTERVAL_S,
) -> None:
    """Point the head, wait for it, record, download and delete one scene.

    Args:
        scene (Scene): scene to record.
        config (RecordConfig): script config.
        ws (Socket): websocket subscribed to the head joint states and the capture topics.
        client (httpx.AsyncClient): client of the web_ui recorder API.
        poll_interval_s (float): seconds between recording-list polls.

    Raises:
        SceneError: naming the failed step; nothing is recorded when the head does not settle.
    """
    await ws.send(head_command(scene, config.head_command_topic))
    await wait_settled(scene, config, ws)

    config.out_dir.mkdir(parents=True, exist_ok=True)
    capture = asyncio.create_task(capture_lines(scene, ws, config.out_dir / f"{scene.name}.ndjson"))
    try:
        body = {"name": scene.name, "duration_s": scene.duration_s}
        check(await client.post(RECORDINGS_PATH, json=body), scene, STEP_START)
        await wait_done(scene, client, poll_interval_s)
    finally:
        capture.cancel()
        await asyncio.gather(capture, return_exceptions=True)

    item = f"{RECORDINGS_PATH}/{scene.name}"
    response = check(await client.get(item), scene, STEP_DOWNLOAD)
    (config.out_dir / f"{scene.name}.npz").write_bytes(response.content)
    check(await client.delete(item), scene, STEP_DELETE)


def websocket_url(web_ui_url: str) -> str:
    """Return the /ws URL for an http(s) web_ui URL."""
    return web_ui_url.replace("http", "ws", 1).rstrip("/") + "/ws"


async def record_all(config: RecordConfig) -> None:
    """Record every configured scene in order, stopping at the first failure.

    Args:
        config (RecordConfig): script config.

    Raises:
        SceneError: for the first failed scene.
    """
    topics = sorted({config.head_js_topic, *(t for s in config.scenes for t in s.capture_topics)})
    async with (
        httpx.AsyncClient(base_url=config.web_ui_url, timeout=HTTP_TIMEOUT_S) as client,
        websockets.connect(websocket_url(config.web_ui_url), max_size=None) as ws,
    ):
        await ws.send(json.dumps({"type": "subscribe", "topics": topics}))
        for scene in config.scenes:
            LOG.info("recording scene %s (pan %s, tilt %s)", scene.name, scene.pan_deg, scene.tilt_deg)
            await record_scene(scene, config, ws, client)
            LOG.info("scene %s -> %s", scene.name, config.out_dir / f"{scene.name}.npz")


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point.

    Args:
        argv (Sequence[str] | None): arguments; one optional config path.

    Returns:
        int: 0 when every scene was recorded, 1 when one failed (named on stderr).
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        asyncio.run(record_all(load_config(args.config)))
    except SceneError as error:
        print(error, file=sys.stderr)
        return EXIT_FAIL
    except (OSError, httpx.HTTPError, websockets.WebSocketException) as error:
        print(f"connection to the web_ui failed: {error}", file=sys.stderr)
        return EXIT_FAIL
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
