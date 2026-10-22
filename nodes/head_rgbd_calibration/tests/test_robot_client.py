"""Tests for the mcp_server client (fake transport)."""

import asyncio
from pathlib import Path
from typing import Any

import pytest

from head_rgbd_calibration.config import RobotConfig
from head_rgbd_calibration.robot_client import RawToolResult, RobotClient, ToolResult

TOKEN = "s3cret-token-value"
URL = "http://127.0.0.1:18200/mcp"


class FakeTransport:
    def __init__(self, result: RawToolResult | None = None, raises: BaseException | None = None, delay: float = 0.0):
        self.calls: list[tuple[str, str, dict[str, Any]]] = []
        self.result = result or RawToolResult(False, "ok", {"a": 1})
        self.raises = raises
        self.delay = delay

    async def __call__(self, url: str, token: str, name: str, args: dict[str, Any]) -> RawToolResult:
        self.calls.append((url, token, name, args))
        if self.delay:
            await asyncio.sleep(self.delay)
        if self.raises:
            raise self.raises
        return self.result


@pytest.fixture
def token_file(tmp_path: Path) -> Path:
    path = tmp_path / "token"
    path.write_text(f"MCP_SERVER_TOKEN={TOKEN}\n")
    return path


def make(token_file: Path, fake: FakeTransport, **kw: float) -> RobotClient:
    cfg = RobotConfig(mcp_url=URL, token_file=str(token_file), **kw)
    return RobotClient(cfg, call=fake)


def test_helpers_tool_names_and_args(token_file: Path) -> None:
    fake = FakeTransport()
    client = make(token_file, fake)
    client.set_head_angles(10.0, 20.0)
    client.take_arm()
    client.move_arm_joints({"shoulder_pan": 1.0}, 0.3)
    client.release_arm(True)
    client.release_arm(False)
    client.arm_state()
    client.control_state()
    assert [(c[2], c[3]) for c in fake.calls] == [
        ("set_head_angles", {"pan_deg": 10.0, "tilt_deg": 20.0}),
        ("take_control", {"subsystem": "arm"}),
        ("move_arm_joints", {"targets": {"shoulder_pan": 1.0}, "speed_scale": 0.3}),
        ("release_control", {"subsystem": "arm", "keep_position": True}),
        ("release_control", {"subsystem": "arm", "keep_position": False}),
        ("get_arm_state", {}),
        ("get_control_state", {}),
    ]
    assert all(c[0] == URL and c[1] == TOKEN for c in fake.calls)


def test_data_returned(token_file: Path) -> None:
    result = make(token_file, FakeTransport()).arm_state()
    assert result == ToolResult(False, "ok", data={"a": 1})


def test_move_uses_move_timeout(token_file: Path) -> None:
    fake = FakeTransport(delay=0.3)
    client = make(token_file, fake, call_timeout_s=0.05, move_timeout_s=2.0)
    assert not client.move_arm_joints({"elbow_flex": 0.0}, 0.2).is_error
    result = client.arm_state()
    assert result.is_error and "timed out after 0.05" in result.text


def test_tool_error_text(token_file: Path) -> None:
    fake = FakeTransport(RawToolResult(True, "floor guard refused", None))
    result = make(token_file, fake).move_arm_joints({}, 0.2)
    assert result.is_error and result.text == "floor guard refused" and not result.unreachable


def test_unreachable(token_file: Path) -> None:
    result = make(token_file, FakeTransport(raises=ConnectionRefusedError("refused"))).take_arm()
    assert result.is_error and result.unreachable
    assert result.text.startswith(f"mcp_server unreachable at {URL}")


def test_unreachable_in_exception_group(token_file: Path) -> None:
    group = BaseExceptionGroup("tg", [ConnectionError("x")])
    result = make(token_file, FakeTransport(raises=group)).take_arm()
    assert result.unreachable


def test_missing_token_file(tmp_path: Path) -> None:
    missing = tmp_path / "nope"
    fake = FakeTransport()
    result = make(missing, fake).take_arm()
    assert result.is_error and str(missing) in result.text and not fake.calls


def test_other_failure_and_token_never_in_text(token_file: Path) -> None:
    result = make(token_file, FakeTransport(raises=RuntimeError(f"boom {TOKEN}"))).take_arm()
    assert result.is_error and not result.unreachable
    assert TOKEN not in result.text


def test_token_not_in_other_texts(token_file: Path) -> None:
    for fake in (FakeTransport(), FakeTransport(delay=1.0), FakeTransport(raises=ConnectionError("c"))):
        client = make(token_file, fake, call_timeout_s=0.05)
        assert TOKEN not in client.arm_state().text
