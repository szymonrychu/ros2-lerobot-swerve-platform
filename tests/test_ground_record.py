"""Unit tests for scripts/ground_record.py: head scenes recorded through the web_ui websocket and recorder API."""

import asyncio
import json
import math
from pathlib import Path
from typing import Any

import httpx
import pytest
import yaml
from pydantic import ValidationError

from scripts.ground_noise_report import read_capture
from scripts.ground_record import RecordConfig, SceneError, record_scene

CONFIG_PATH = Path(__file__).resolve().parent.parent / "scripts" / "ground_record.yaml"
NPZ_BYTES = b"npz-bytes"


class FakeWebSocket:
    """Websocket double: logs sent frames and serves queued frames to recv()."""

    def __init__(self, calls: list[str]) -> None:
        self.calls = calls
        self.sent: list[dict[str, Any]] = []
        self.inbox: asyncio.Queue[str] = asyncio.Queue()

    async def send(self, frame: str) -> None:
        message = json.loads(frame)
        self.sent.append(message)
        self.calls.append(f"ws:{message['type']}")

    async def recv(self) -> str:
        return await self.inbox.get()

    def push(self, message: dict[str, Any]) -> None:
        self.inbox.put_nowait(json.dumps(message))


def head_js(pan_deg: float, tilt_deg: float) -> dict[str, Any]:
    return {
        "topic": "/head/joint_states",
        "data": {"name": ["head_pan", "head_tilt"], "position": [math.radians(pan_deg), math.radians(tilt_deg)]},
    }


def state_line(index: int) -> dict[str, Any]:
    return {"topic": "/ground/state", "data": json.dumps({"index": index})}


def make_config(tmp_path: Path, **overrides: Any) -> RecordConfig:
    scene = {"name": "box", "pan_deg": 35, "tilt_deg": 29.4, "settle_s": 0, "duration_s": 5}
    return RecordConfig.model_validate(
        {"out_dir": str(tmp_path / "out"), "settle_timeout_s": 0.3, "scenes": [scene], **overrides}
    )


def make_client(calls: list[str], done: bool = True) -> httpx.AsyncClient:
    def handler(request: httpx.Request) -> httpx.Response:
        calls.append(f"{request.method} {request.url.path}")
        if request.method == "POST":
            return httpx.Response(202, json=json.loads(request.content))
        if request.method == "DELETE":
            return httpx.Response(200, json={"ok": True})
        if request.url.path.endswith("/box"):
            return httpx.Response(200, content=NPZ_BYTES)
        return httpx.Response(200, json=[{"name": "box", "done": done}])

    return httpx.AsyncClient(transport=httpx.MockTransport(handler), base_url="http://robot")


def run_scene(config: RecordConfig, ws: FakeWebSocket, client: httpx.AsyncClient) -> None:
    asyncio.run(record_scene(config.scenes[0], config, ws, client, poll_interval_s=0.01))


def test_scene_points_head_then_records_and_downloads(tmp_path: Path) -> None:
    calls: list[str] = []
    ws = FakeWebSocket(calls)
    ws.push(head_js(10, 20))
    ws.push(head_js(35, 29.4))
    ws.push(state_line(1))
    config = make_config(tmp_path)

    run_scene(config, ws, make_client(calls))

    assert calls == [
        "ws:publish",
        "POST /api/ground/recordings",
        "GET /api/ground/recordings",
        "GET /api/ground/recordings/box",
        "DELETE /api/ground/recordings/box",
    ]
    command = ws.sent[0]
    assert command["topic"] == "/head/joint_commands"
    assert command["msg_type"] == "sensor_msgs/JointState"
    assert command["data"]["name"] == ["head_pan", "head_tilt"]
    assert command["data"]["position"] == pytest.approx([math.radians(35), math.radians(29.4)])
    assert (tmp_path / "out" / "box.npz").read_bytes() == NPZ_BYTES
    assert [m["type"] for m in ws.sent] == ["publish"]


def test_head_not_settled_fails_scene_without_recording(tmp_path: Path) -> None:
    calls: list[str] = []
    ws = FakeWebSocket(calls)
    ws.push(head_js(10, 20))
    config = make_config(tmp_path)

    with pytest.raises(SceneError) as error:
        run_scene(config, ws, make_client(calls))

    assert error.value.scene == "box"
    assert error.value.step == "settle"
    assert not any(call.startswith(("POST", "DELETE")) for call in calls)
    assert not (tmp_path / "out" / "box.npz").exists()


def test_refused_head_command_is_the_scene_failure(tmp_path: Path) -> None:
    calls: list[str] = []
    ws = FakeWebSocket(calls)
    ws.push({"type": "error", "source": "head", "message": "head command refused: head_pan out of range"})

    with pytest.raises(SceneError, match="out of range") as error:
        run_scene(make_config(tmp_path), ws, make_client(calls))

    assert error.value.step == "settle"
    assert "POST /api/ground/recordings" not in calls


def test_capture_lines_written_in_report_format(tmp_path: Path) -> None:
    calls: list[str] = []
    ws = FakeWebSocket(calls)
    ws.push(head_js(35, 29.4))
    for index in range(3):
        ws.push(state_line(index))
    ws.push({"topic": "/other", "data": {}})
    config = make_config(tmp_path)

    run_scene(config, ws, make_client(calls))

    lines = [json.loads(line) for line in (tmp_path / "out" / "box.ndjson").read_text().splitlines()]
    assert all(set(line) == {"t", "topic", "data"} for line in lines)
    states, skipped, malformed = read_capture(tmp_path / "out" / "box.ndjson")
    assert [s["index"] for s in states] == [0, 1, 2]
    assert (skipped, malformed) == (0, 0)


def test_settle_period_frames_are_drained_not_captured(tmp_path: Path) -> None:
    calls: list[str] = []
    ws = FakeWebSocket(calls)
    ws.push(head_js(35, 29.4))
    ws.push(state_line(99))
    config = make_config(tmp_path)
    config.scenes[0].settle_s = 0.05

    run_scene(config, ws, make_client(calls))

    assert (tmp_path / "out" / "box.ndjson").read_text() == ""
    assert ws.inbox.empty()


def test_config_validates() -> None:
    config = RecordConfig.model_validate(yaml.safe_load(CONFIG_PATH.read_text()))
    assert config.web_ui_url == "http://client.ros2.lan:8080"
    assert config.out_dir == Path(".logs/ground_recordings")
    assert (config.head_command_topic, config.head_js_topic) == ("/head/joint_commands", "/head/joint_states")
    assert (config.settle_tolerance_deg, config.settle_timeout_s) == (1.0, 15)
    assert [(s.name, s.pan_deg, s.tilt_deg) for s in config.scenes] == [
        ("home", 0, 29.4),
        ("box", 35, 29.4),
        ("ramp", -30, 29.4),
        ("empty", -60, 29.4),
    ]
    assert all((s.settle_s, s.duration_s, s.capture_topics) == (10, 30, ["/ground/state"]) for s in config.scenes)
    with pytest.raises(ValidationError):
        RecordConfig.model_validate({"bogus": 1})
    with pytest.raises(ValidationError):
        RecordConfig.model_validate({"scenes": [{"name": "x", "pan_deg": 0, "tilt_deg": 0, "oops": 1}]})
