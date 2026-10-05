"""Persisted event log and normalization of Agent SDK messages into the web UI event contract."""

import asyncio
import base64
import binascii
import io
import json
import logging
import time
from collections import deque
from pathlib import Path
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

from .config import DEFAULT_SESSION_LOG_MAX_BYTES, ClaudeAgentConfig
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
# Put on subscriber queues by EventLog.reset() so live consumers drop their stale view (never stored or sent as-is).
RESET_MARKER: dict[str, Any] = {"type": "reset"}
STATUS_DONE = "done"
STATUS_ERROR = "error"
STATUS_INTERRUPTED = "interrupted"
STATUS_MAX_TURNS = "max_turns"
STATUS_TIMEOUT = "timeout"


class EventLog:
    """Numbered events with live subscribers, persisted as JSON lines (single asyncio loop).

    Every event is appended to ``path`` (one JSON object per line). Only the last ``size`` events stay in RAM; older
    ones are read back from the file by page(). When the file exceeds ``max_bytes`` its oldest half is dropped.

    Attributes:
        path (Path | None): The JSONL file, or None for a memory-only log.
        seq (int): Sequence number of the newest event (0 when empty).
        first_seq (int): Sequence number of the oldest retained event (0 when empty).
    """

    def __init__(self, size: int, path: Path | None = None, max_bytes: int = DEFAULT_SESSION_LOG_MAX_BYTES) -> None:
        """Create the log and load the existing file, if any (seq continues after the last stored event).

        Args:
            size (int): Maximum number of events kept in RAM.
            path (Path | None): JSONL file, or None to keep events in memory only.
            max_bytes (int): File size that triggers the rotation.
        """
        self.size = size
        self.path = path
        self.max_bytes = max_bytes
        self.events: deque[dict[str, Any]] = deque(maxlen=size)
        self.seq = 0
        self.first_seq = 0
        self.file_bytes = 0
        self.subscribers: list[asyncio.Queue[dict[str, Any]]] = []
        self.write_failed = False
        if path is not None:
            self.load()

    def read_file(self) -> list[dict[str, Any]]:
        """Read every stored event (corrupt lines are skipped).

        Returns:
            list[dict[str, Any]]: Events in file order; empty when there is no readable file.
        """
        if self.path is None:
            return []
        try:
            lines = self.path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return []
        out: list[dict[str, Any]] = []
        for line in lines:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if isinstance(event, dict) and isinstance(event.get("seq"), int):
                out.append(event)
        return out

    def load(self) -> None:
        """Load the stored events: the last ``size`` go to RAM and the sequence continues."""
        stored = self.read_file()
        self.events.extend(stored[-self.size :])
        if stored:
            self.first_seq, self.seq = stored[0]["seq"], stored[-1]["seq"]
        try:
            self.file_bytes = self.path.stat().st_size  # type: ignore[union-attr]
        except OSError:
            self.file_bytes = 0

    def persist(self, event: dict[str, Any]) -> None:
        """Append an event to the file and rotate when it grew past the limit; a failing disk never breaks the agent.

        Args:
            event (dict[str, Any]): The event to store.
        """
        if self.path is None:
            return
        line = json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n"
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(line)
            self.file_bytes += len(line.encode("utf-8"))
            if self.file_bytes > self.max_bytes:
                self.rotate()
        except OSError as exc:
            if not self.write_failed:
                logging.getLogger("claude_agent").warning(f"session log not writable ({self.path}): {exc}")
            self.write_failed = True

    def rotate(self) -> None:
        """Drop the oldest half of the stored events (atomic rewrite)."""
        stored = self.read_file()
        kept = stored[len(stored) // 2 :]
        tmp = self.path.with_suffix(".tmp")  # type: ignore[union-attr]
        data = "".join(json.dumps(e, ensure_ascii=False, separators=(",", ":")) + "\n" for e in kept)
        tmp.write_text(data, encoding="utf-8")
        tmp.replace(self.path)  # type: ignore[arg-type]
        self.file_bytes = len(data.encode("utf-8"))
        self.first_seq = kept[0]["seq"] if kept else 0

    def append(self, event_type: str, **fields: Any) -> dict[str, Any]:
        """Add an event, store it and fan it out to subscribers.

        Args:
            event_type (str): Event type, e.g. "assistant_text".
            **fields (Any): Event payload fields.

        Returns:
            dict[str, Any]: The stored event with seq, ts and type.
        """
        self.seq += 1
        event = {"seq": self.seq, "ts": time.time(), "type": event_type, **fields}
        if not self.first_seq:
            self.first_seq = self.seq
        self.events.append(event)
        if self.path is None and len(self.events) == self.size:
            self.first_seq = self.events[0]["seq"]
        self.persist(event)
        for queue in self.subscribers:
            queue.put_nowait(event)
        return event

    def history(self) -> list[dict[str, Any]]:
        """Return the events held in RAM, oldest first.

        Returns:
            list[dict[str, Any]]: Copy of the buffer (the last ``size`` events).
        """
        return list(self.events)

    def page(self, before_seq: int | None, limit: int) -> tuple[list[dict[str, Any]], bool]:
        """Return the newest ``limit`` events older than ``before_seq`` (the newest overall without it).

        Args:
            before_seq (int | None): Only events with a smaller seq; None for no bound.
            limit (int): Maximum number of events.

        Returns:
            tuple[list[dict[str, Any]], bool]: (events with ascending seq, whether older events exist).
        """
        ram = [e for e in self.events if before_seq is None or e["seq"] < before_seq]
        if len(ram) >= limit or self.path is None or (self.events and self.events[0]["seq"] <= self.first_seq):
            chosen = ram[-limit:]
        else:
            stored = [e for e in self.read_file() if before_seq is None or e["seq"] < before_seq]
            chosen = stored[-limit:]
        return chosen, bool(chosen) and chosen[0]["seq"] > self.first_seq

    def reset(self) -> None:
        """Forget everything: delete the file, empty the buffer and restart the sequence at 0 (subscribers stay and get RESET_MARKER)."""
        self.events.clear()
        self.seq = self.first_seq = self.file_bytes = 0
        if self.path is not None:
            self.path.unlink(missing_ok=True)
        for queue in self.subscribers:
            queue.put_nowait(RESET_MARKER)

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
