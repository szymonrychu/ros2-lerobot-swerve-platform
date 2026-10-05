"""Event ring buffer and normalization of Agent SDK messages into the web UI event contract."""

import asyncio
import base64
import binascii
import io
import time
from collections import deque
from typing import Any

from claude_agent_sdk import (
    AssistantMessage,
    Message,
    ResultMessage,
    TextBlock,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)
from PIL import Image, UnidentifiedImageError

from .config import ClaudeAgentConfig
from .tools import KIND_UNCAPPED, classify_tool, short_name

TEXT_TRUNCATE_CHARS = 4000
THUMBNAIL_QUALITY = 70
THUMBNAIL_MEDIA_TYPE = "image/jpeg"
# Specific wording of Claude Code authentication failures; a bare "401" or "authentication" is not enough (tool or
# model text can mention them).
AUTH_MARKERS = (
    "api error: 401",
    "authentication_error",
    "invalid x-api-key",
    "invalid api key",
    "oauth token",
    "please run /login",
)
STATUS_DONE = "done"
STATUS_ERROR = "error"
STATUS_INTERRUPTED = "interrupted"
STATUS_MAX_TURNS = "max_turns"
STATUS_TIMEOUT = "timeout"


class EventLog:
    """Ring buffer of numbered events with live subscribers (single asyncio loop)."""

    def __init__(self, size: int) -> None:
        """Create the log.

        Args:
            size (int): Maximum number of events kept.
        """
        self.events: deque[dict[str, Any]] = deque(maxlen=size)
        self.seq = 0
        self.subscribers: list[asyncio.Queue[dict[str, Any]]] = []

    def append(self, event_type: str, **fields: Any) -> dict[str, Any]:
        """Add an event and fan it out to subscribers.

        Args:
            event_type (str): Event type, e.g. "assistant_text".
            **fields (Any): Event payload fields.

        Returns:
            dict[str, Any]: The stored event with seq, ts and type.
        """
        self.seq += 1
        event = {"seq": self.seq, "ts": time.time(), "type": event_type, **fields}
        self.events.append(event)
        for queue in self.subscribers:
            queue.put_nowait(event)
        return event

    def history(self) -> list[dict[str, Any]]:
        """Return the buffered events, oldest first.

        Returns:
            list[dict[str, Any]]: Copy of the buffer.
        """
        return list(self.events)

    def subscribe(self) -> asyncio.Queue[dict[str, Any]]:
        """Register a live subscriber.

        Returns:
            asyncio.Queue[dict[str, Any]]: Queue receiving each new event.
        """
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self.subscribers.append(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        """Remove a subscriber.

        Args:
            queue (asyncio.Queue[dict[str, Any]]): Queue returned by subscribe().
        """
        if queue in self.subscribers:
            self.subscribers.remove(queue)


def thumbnail_image(data_b64: str, max_px: int) -> tuple[str, str]:
    """Downscale a base64 image to a JPEG thumbnail.

    Args:
        data_b64 (str): Base64 image data (any format Pillow reads).
        max_px (int): Longest edge of the thumbnail (images smaller than this are not upscaled).

    Returns:
        tuple[str, str]: ("image/jpeg", base64 JPEG data).

    Raises:
        ValueError: When the data is not a decodable image.
    """
    try:
        image = Image.open(io.BytesIO(base64.b64decode(data_b64, validate=True)))
        image.load()
    except (binascii.Error, UnidentifiedImageError, OSError) as exc:
        raise ValueError("undecodable image") from exc
    image.thumbnail((max_px, max_px))
    buf = io.BytesIO()
    image.convert("RGB").save(buf, "JPEG", quality=THUMBNAIL_QUALITY)
    return THUMBNAIL_MEDIA_TYPE, base64.b64encode(buf.getvalue()).decode()


def normalize_tool_result(block: ToolResultBlock, config: ClaudeAgentConfig) -> dict[str, Any]:
    """Convert a ToolResultBlock into a tool_result event payload.

    Args:
        block (ToolResultBlock): SDK block (content is a string, a list of text/image dicts or None).
        config (ClaudeAgentConfig): Supplies image_thumbnail_max_px.

    Returns:
        dict[str, Any]: {id, is_error, content, truncated}; long texts are cut at TEXT_TRUNCATE_CHARS.
    """
    raw = block.content
    items: list[dict[str, Any]] = [{"type": "text", "text": raw}] if isinstance(raw, str) else list(raw or [])
    content: list[dict[str, str]] = []
    truncated = False
    for item in items:
        if item.get("type") == "image":
            source = item.get("source", {})
            try:
                media_type, data = thumbnail_image(source.get("data", ""), config.image_thumbnail_max_px)
                content.append({"type": "image", "media_type": media_type, "data_b64": data})
            except ValueError:
                content.append({"type": "text", "text": "[image could not be decoded]"})
        elif item.get("type") == "text":
            text = str(item.get("text", ""))
            if len(text) > TEXT_TRUNCATE_CHARS:
                text = text[:TEXT_TRUNCATE_CHARS]
                truncated = True
            content.append({"type": "text", "text": text})
    return {"id": block.tool_use_id, "is_error": bool(block.is_error), "content": content, "truncated": truncated}


def normalize_message(message: Message, config: ClaudeAgentConfig) -> list[tuple[str, dict[str, Any]]]:
    """Convert an SDK message into zero or more (event type, fields) pairs.

    Args:
        message (Message): AssistantMessage (text, tool calls) or UserMessage (tool results); others yield nothing.
        config (ClaudeAgentConfig): Tool classification and thumbnail size.

    Returns:
        list[tuple[str, dict[str, Any]]]: Events in order.
    """
    out: list[tuple[str, dict[str, Any]]] = []
    if isinstance(message, AssistantMessage):
        for block in message.content:
            if isinstance(block, TextBlock) and block.text.strip():
                out.append(("assistant_text", {"text": block.text}))
            elif isinstance(block, ToolUseBlock):
                out.append(
                    (
                        "tool_call",
                        {
                            "id": block.id,
                            "name": short_name(block.name),
                            "full_name": block.name,
                            "kind": classify_tool(config, block.name) or KIND_UNCAPPED,
                            "input": block.input,
                        },
                    )
                )
    elif isinstance(message, UserMessage) and isinstance(message.content, list):
        out.extend(
            ("tool_result", normalize_tool_result(b, config)) for b in message.content if isinstance(b, ToolResultBlock)
        )
    return out


def result_to_turn_end(result: ResultMessage, interrupted: bool, effector_calls: int) -> dict[str, Any]:
    """Build the turn_end payload from the final ResultMessage.

    Args:
        result (ResultMessage): Final message of an instruction.
        interrupted (bool): True when the user interrupted it.
        effector_calls (int): Effector calls used in the instruction.

    Returns:
        dict[str, Any]: {status, cost_usd, num_turns, effector_calls}.
    """
    if interrupted or (result.terminal_reason or "").startswith("aborted"):
        status = STATUS_INTERRUPTED
    elif result.subtype == "error_max_turns" or result.terminal_reason == "max_turns":
        status = STATUS_MAX_TURNS
    elif result.is_error:
        status = STATUS_ERROR
    else:
        status = STATUS_DONE
    return {
        "status": status,
        "cost_usd": result.total_cost_usd or 0.0,
        "num_turns": result.num_turns,
        "effector_calls": effector_calls,
    }


def detect_auth_error(result: ResultMessage) -> bool:
    """Tell whether a failed result is clearly an authentication failure (HTTP 401 status or Claude Code's auth wording).

    Args:
        result (ResultMessage): Final message.

    Returns:
        bool: True when is_error and the status is 401 or the text matches AUTH_MARKERS.
    """
    if not result.is_error:
        return False
    if result.api_error_status == 401:
        return True
    text = " ".join([result.result or "", *(result.errors or [])]).lower()
    return any(marker in text for marker in AUTH_MARKERS)
