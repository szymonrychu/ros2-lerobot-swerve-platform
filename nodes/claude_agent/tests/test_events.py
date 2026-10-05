"""Event log ring buffer and normalization of SDK messages into the web UI event contract."""

import base64
import io

from claude_agent_sdk import AssistantMessage, ResultMessage, TextBlock, ToolResultBlock, ToolUseBlock, UserMessage
from PIL import Image

from claude_agent.config import ClaudeAgentConfig
from claude_agent.events import (
    TEXT_TRUNCATE_CHARS,
    EventLog,
    detect_auth_error,
    normalize_message,
    result_to_turn_end,
    thumbnail_image,
)


def jpeg_b64(width: int, height: int) -> str:
    buf = io.BytesIO()
    Image.new("RGB", (width, height), (200, 30, 30)).save(buf, "JPEG")
    return base64.b64encode(buf.getvalue()).decode()


def result(**kw) -> ResultMessage:
    base = dict(subtype="success", duration_ms=1, duration_api_ms=1, is_error=False, num_turns=3, session_id="s")
    base.update(kw)
    return ResultMessage(**base)


def test_event_log_sequence_and_ring_buffer() -> None:
    log = EventLog(size=3)
    for i in range(5):
        event = log.append("assistant_text", text=str(i))
        assert event["seq"] == i + 1
        assert isinstance(event["ts"], float)
        assert event["type"] == "assistant_text"
    history = log.history()
    assert [e["text"] for e in history] == ["2", "3", "4"]
    assert history[-1]["seq"] == 5


async def test_event_log_subscribers_receive_new_events() -> None:
    log = EventLog(size=10)
    queue = log.subscribe()
    log.append("error", message="x")
    event = queue.get_nowait()
    assert event["type"] == "error"
    log.unsubscribe(queue)
    log.append("error", message="y")
    assert queue.empty()


def test_history_returns_copies_safe_to_serialize() -> None:
    log = EventLog(size=2)
    log.append("user_message", text="hi")
    assert log.history() == log.history()


def test_normalize_assistant_text_and_tool_use(config: ClaudeAgentConfig) -> None:
    msg = AssistantMessage(
        content=[
            TextBlock(text="Looking."),
            ToolUseBlock(id="t1", name="mcp__robot__get_camera_image", input={"camera": "front"}),
            ToolUseBlock(id="t2", name="mcp__robot__drive", input={"vx": 0.1}),
            ToolUseBlock(id="t3", name="mcp__robot__stop", input={}),
        ],
        model="opus",
    )
    out = normalize_message(msg, config)
    assert out[0] == ("assistant_text", {"text": "Looking."})
    kinds = {f["name"]: f["kind"] for t, f in out if t == "tool_call"}
    assert kinds == {"get_camera_image": "sensor", "drive": "effector", "stop": "uncapped"}
    drive = next(f for t, f in out if t == "tool_call" and f["name"] == "drive")
    assert drive == {
        "id": "t2",
        "name": "drive",
        "full_name": "mcp__robot__drive",
        "kind": "effector",
        "input": {"vx": 0.1},
    }


def test_normalize_skips_empty_text_and_non_robot_tool_kind(config: ClaudeAgentConfig) -> None:
    msg = AssistantMessage(content=[TextBlock(text="  "), ToolUseBlock(id="t", name="Bash", input={})], model="opus")
    out = normalize_message(msg, config)
    assert [t for t, _ in out] == ["tool_call"]
    assert out[0][1]["name"] == "Bash"
    assert out[0][1]["kind"] == "uncapped"


def test_normalize_tool_result_text_and_image(config: ClaudeAgentConfig) -> None:
    msg = UserMessage(
        content=[
            ToolResultBlock(
                tool_use_id="t1",
                content=[
                    {"type": "text", "text": "frame"},
                    {
                        "type": "image",
                        "source": {"type": "base64", "media_type": "image/png", "data": jpeg_b64(1000, 500)},
                    },
                ],
                is_error=False,
            )
        ]
    )
    ((etype, fields),) = normalize_message(msg, config)
    assert etype == "tool_result"
    assert fields["id"] == "t1" and fields["is_error"] is False
    assert fields["content"][0] == {"type": "text", "text": "frame"}
    image = fields["content"][1]
    assert image["type"] == "image" and image["media_type"] == "image/jpeg"
    decoded = Image.open(io.BytesIO(base64.b64decode(image["data_b64"])))
    assert max(decoded.size) == 480
    assert decoded.size == (480, 240)
    assert fields["truncated"] is False


def test_thumbnail_respects_config_and_never_upscales() -> None:
    media_type, data = thumbnail_image(jpeg_b64(100, 50), 480)
    assert media_type == "image/jpeg"
    assert Image.open(io.BytesIO(base64.b64decode(data))).size == (100, 50)
    _, small = thumbnail_image(jpeg_b64(800, 800), 64)
    assert Image.open(io.BytesIO(base64.b64decode(small))).size == (64, 64)


def test_thumbnail_rgba_png_converts_to_jpeg() -> None:
    buf = io.BytesIO()
    Image.new("RGBA", (600, 300), (0, 0, 0, 128)).save(buf, "PNG")
    _, data = thumbnail_image(base64.b64encode(buf.getvalue()).decode(), 480)
    assert Image.open(io.BytesIO(base64.b64decode(data))).format == "JPEG"


def test_bad_image_becomes_text_note(config: ClaudeAgentConfig) -> None:
    msg = UserMessage(
        content=[
            ToolResultBlock(
                tool_use_id="t",
                content=[{"type": "image", "source": {"type": "base64", "media_type": "image/png", "data": "!!!"}}],
            )
        ]
    )
    ((_, fields),) = normalize_message(msg, config)
    assert fields["content"][0]["type"] == "text"
    assert "image" in fields["content"][0]["text"]


def test_long_text_truncated_with_flag(config: ClaudeAgentConfig) -> None:
    long_text = "x" * (TEXT_TRUNCATE_CHARS + 500)
    msg = UserMessage(content=[ToolResultBlock(tool_use_id="t", content=long_text, is_error=True)])
    ((_, fields),) = normalize_message(msg, config)
    assert fields["is_error"] is True
    assert fields["truncated"] is True
    text = fields["content"][0]["text"]
    assert text.startswith("x" * TEXT_TRUNCATE_CHARS)
    assert len(text) < len(long_text)
    assert TEXT_TRUNCATE_CHARS == 4000


def test_tool_result_none_content_and_string_user_message(config: ClaudeAgentConfig) -> None:
    msg = UserMessage(content=[ToolResultBlock(tool_use_id="t", content=None)])
    ((_, fields),) = normalize_message(msg, config)
    assert fields["content"] == [] and fields["is_error"] is False
    assert normalize_message(UserMessage(content="hello"), config) == []


def test_result_to_turn_end_statuses() -> None:
    assert result_to_turn_end(result(total_cost_usd=0.5), False, 4) == {
        "status": "done",
        "cost_usd": 0.5,
        "num_turns": 3,
        "effector_calls": 4,
    }
    assert result_to_turn_end(result(subtype="error_max_turns"), False, 0)["status"] == "max_turns"
    assert result_to_turn_end(result(terminal_reason="max_turns"), False, 0)["status"] == "max_turns"
    assert result_to_turn_end(result(terminal_reason="aborted_streaming"), False, 0)["status"] == "interrupted"
    assert result_to_turn_end(result(), True, 0)["status"] == "interrupted"
    assert result_to_turn_end(result(is_error=True), False, 0)["status"] == "error"
    assert result_to_turn_end(result(total_cost_usd=None), False, 0)["cost_usd"] == 0.0


def test_detect_auth_error() -> None:
    assert detect_auth_error(result(is_error=True, api_error_status=401, result="x"))
    assert detect_auth_error(result(is_error=True, result="Invalid API key - Please run /login"))
    assert detect_auth_error(result(is_error=True, result="OAuth token has expired"))
    assert detect_auth_error(result(is_error=True, result="authentication_error"))
    assert not detect_auth_error(result(is_error=True, api_error_status=529, result="overloaded"))
    assert not detect_auth_error(result(is_error=False, result="401 is a number"))


def test_detect_auth_error_ignores_incidental_matches() -> None:
    for text in (
        "Robot reached waypoint 401 of 500",
        "authentication of the web UI is out of scope",
        "pose 4012 unreachable",
        "unauthorized area on the map",
        "see /login page notes",
    ):
        assert not detect_auth_error(result(is_error=True, result=text)), text


def test_detect_auth_error_matches_specific_patterns() -> None:
    for text in (
        "API Error: 401 something",
        'API Error: 401 {"type":"error","error":{"type":"authentication_error"}}',
        "invalid x-api-key",
        "OAuth token has expired",
        "Invalid API key - Please run /login",
    ):
        assert detect_auth_error(result(is_error=True, result=text)), text
    assert detect_auth_error(result(is_error=True, errors=["API Error: 401 nope"]))
