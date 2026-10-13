"""WebSocket client plumbing: write backpressure in uvicorn's sansio protocol and the per-client mailbox."""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import Callable, Iterable
from typing import Any

import structlog
from uvicorn.protocols.websockets.websockets_sansio_impl import WebSocketsSansIOProtocol

from .metrics import WS_CLIENTS_DROPPED_SLOW

log = structlog.get_logger(__name__)

# Seconds one frame may take to reach a client's socket before the client is dropped as too slow.
SEND_TIMEOUT_S = 10.0
# Ordered, non-coalescable messages (error frames) a client may have pending before it is dropped.
MAX_QUEUED_MESSAGES = 32


class BackpressureWebSocketsProtocol(WebSocketsSansIOProtocol):
    """uvicorn's websockets-sansio protocol with the transport's write flow control wired to send().

    uvicorn 0.42 creates and awaits ``writable`` in send() but never clears it, so send() never blocks on a peer that
    stopped reading. The wsproto implementation does clear it; this subclass does the same.
    """

    def pause_writing(self) -> None:
        """Transport write buffer is above the high-water mark: make send() wait."""
        self.writable.clear()

    def resume_writing(self) -> None:
        """Transport write buffer drained below the low-water mark: let send() continue."""
        self.writable.set()


class ClientConnection:
    """A connected WebSocket client with a latest-wins mailbox drained by its own sender task.

    The broadcaster only offers frames and never awaits the socket. offer() keeps one frame per key (a newer frame
    replaces the older one, so pending memory is bounded by the number of keys); enqueue() keeps ordered messages that
    must not be dropped in a small bounded queue. One sender task writes them with a per-frame timeout; a timeout, send
    error or queue overflow drops the client.
    """

    def __init__(
        self,
        ws: Any,
        on_dead: Callable[[ClientConnection], None],
        send_timeout_s: float = SEND_TIMEOUT_S,
        max_queued: int = MAX_QUEUED_MESSAGES,
    ) -> None:
        """Initialise the connection.

        Args:
            ws (Any): Accepted WebSocket (needs an async send_text).
            on_dead (Callable[[ClientConnection], None]): Called once when the client is dropped.
            send_timeout_s (float): Seconds one send may take.
            max_queued (int): Ordered messages that may be pending.
        """
        self.ws = ws
        self.on_dead = on_dead
        self.send_timeout_s = send_timeout_s
        self.max_queued = max_queued
        # Topics the client wants (None: every topic, the state before it sends a subscribe message) and whether its
        # open tab needs the downscaled color_small JPEG.
        self.topics: frozenset[str] | None = None
        self.color_small = False
        self.mailbox: dict[str, str] = {}
        self.ordered: deque[str] = deque()
        self.wakeup = asyncio.Event()
        self.closed = False
        self.sender_task: asyncio.Task[None] | None = None

    def subscribes(self, topic: str) -> bool:
        """Whether the client wants frames of topic.

        Args:
            topic (str): WS topic name.

        Returns:
            bool: True when the client subscribed to topic or has not narrowed its subscription.
        """
        return self.topics is None or topic in self.topics

    def subscribe(self, topics: Iterable[str] | None, color_small: bool) -> None:
        """Replace the client's subscription.

        Args:
            topics (Iterable[str] | None): Topics to receive, None for every topic.
            color_small (bool): Whether the client's open tab needs the downscaled color JPEG.
        """
        self.topics = None if topics is None else frozenset(topics)
        self.color_small = color_small

    @property
    def pending_frames(self) -> int:
        """Frames waiting to be sent (mailbox plus ordered queue)."""
        return len(self.mailbox) + len(self.ordered)

    @property
    def pending_bytes(self) -> int:
        """Characters waiting to be sent."""
        return sum(map(len, self.mailbox.values())) + sum(map(len, self.ordered))

    def offer(self, key: str, frame: str) -> None:
        """Offer a coalescable frame: it replaces any pending frame with the same key.

        Args:
            key (str): Topic (or message kind) the frame belongs to.
            frame (str): Serialized frame.
        """
        if self.closed:
            return
        self.mailbox[key] = frame
        self.wakeup.set()

    def enqueue(self, frame: str) -> None:
        """Queue a frame that must not be dropped or reordered; the client is dropped if the queue is full.

        Args:
            frame (str): Serialized frame.
        """
        if self.closed:
            return
        if len(self.ordered) >= self.max_queued:
            self.drop("ordered queue full", slow=True)
            return
        self.ordered.append(frame)
        self.wakeup.set()

    def start(self) -> None:
        """Start the sender task."""
        self.sender_task = asyncio.create_task(self.send_loop())

    def close(self) -> None:
        """Stop the sender task and discard pending frames (synchronous, safe during cancellation)."""
        self.closed = True
        self.mailbox.clear()
        self.ordered.clear()
        task = self.sender_task
        if task is not None and task is not asyncio.current_task():
            task.cancel()

    def next_frame(self) -> str | None:
        """Pop the next frame: ordered messages first, then mailbox frames in arrival order.

        Returns:
            str | None: The frame, None when nothing is pending.
        """
        if self.ordered:
            return self.ordered.popleft()
        if self.mailbox:
            return self.mailbox.pop(next(iter(self.mailbox)))
        return None

    async def send_loop(self) -> None:
        """Drain pending frames to the socket; drop the client on a send timeout or error."""
        try:
            while True:
                await self.wakeup.wait()
                self.wakeup.clear()
                while (frame := self.next_frame()) is not None:
                    await asyncio.wait_for(self.ws.send_text(frame), self.send_timeout_s)
        except asyncio.CancelledError:
            raise
        except TimeoutError:
            self.drop(f"send exceeded {self.send_timeout_s:g} s", slow=True)
        except Exception as exc:
            self.drop(f"send failed: {exc!r}", slow=False)

    def drop(self, reason: str, slow: bool) -> None:
        """Drop the client: free its pending frames and tell the owner.

        Args:
            reason (str): Why, for the log.
            slow (bool): Whether this counts as a client dropped for slowness.
        """
        if self.closed:
            return
        self.closed = True
        self.mailbox.clear()
        self.ordered.clear()
        task = self.sender_task
        if task is not None and task is not asyncio.current_task():
            task.cancel()  # ends the session: websocket_endpoint waits on the sender task
        if slow:
            WS_CLIENTS_DROPPED_SLOW.inc()
            log.warning("ws_client_dropped_slow", reason=reason)
        self.on_dead(self)


def wanted_topics(clients: Iterable[ClientConnection]) -> set[str] | None:
    """Union of the topics the clients subscribe to.

    Args:
        clients (Iterable[ClientConnection]): Connected clients.

    Returns:
        set[str] | None: Topics any client wants, None when some client wants every topic.
    """
    wanted: set[str] = set()
    for conn in clients:
        if conn.topics is None:
            return None
        wanted |= conn.topics
    return wanted


def wants_color_small(clients: Iterable[ClientConnection]) -> bool:
    """Whether any client's open tab needs the downscaled color JPEG.

    Args:
        clients (Iterable[ClientConnection]): Connected clients.

    Returns:
        bool: True when at least one client needs color_small.
    """
    return any(conn.color_small for conn in clients)
