"""Client side of the map_manager topics (no ROS): request/response matching by request_id."""

import json
import threading
import uuid
from collections.abc import Callable
from typing import Any

from pydantic import ValidationError
from ros2_common.maps import MapResult

from .models import RobotError


class MapManagerDown(RobotError):
    """Nothing listens on /map_manager/command (map_manager is not running)."""


class MapTimeout(RobotError):
    """map_manager did not answer a command in time."""


class MapRequests:
    """Publishes /map_manager/command messages and matches /map_manager/result answers by request_id."""

    def __init__(
        self,
        publish: Callable[[str], None],
        manager_running: Callable[[], bool],
        timeout_for: Callable[[str], float],
        new_id: Callable[[], str] = lambda: uuid.uuid4().hex,
    ) -> None:
        """Create the client.

        Args:
            publish (Callable[[str], None]): Publishes one JSON string on /map_manager/command.
            manager_running (Callable[[], bool]): Whether map_manager listens (a subscriber exists on the command topic).
            timeout_for (Callable[[str], float]): Seconds to wait for the result of an op.
            new_id (Callable[[], str]): Request id factory.
        """
        self.publish = publish
        self.manager_running = manager_running
        self.timeout_for = timeout_for
        self.new_id = new_id
        self.lock = threading.Lock()
        self.pending: dict[str, tuple[threading.Event, list[MapResult]]] = {}

    def pending_count(self) -> int:
        """Number of requests still waiting for an answer.

        Returns:
            int: Pending request count.
        """
        with self.lock:
            return len(self.pending)

    def on_result(self, payload: str) -> None:
        """Feed one /map_manager/result JSON string; unknown request ids and malformed payloads are dropped.

        Args:
            payload (str): The std_msgs/String data.
        """
        try:
            result = MapResult.model_validate_json(payload)
        except ValidationError:
            return
        with self.lock:
            entry = self.pending.get(result.request_id)
        if entry is not None:
            entry[1].append(result)
            entry[0].set()

    def send(self, command: dict[str, Any]) -> MapResult:
        """Send one command and wait for its result.

        Args:
            command (dict[str, Any]): MapCommand fields without request_id (``op`` selects the timeout).

        Returns:
            MapResult: The accepted result (ok is True).

        Raises:
            MapManagerDown: Nothing listens on the command topic.
            MapTimeout: No answer within the op's timeout.
            RobotError: map_manager answered ok false (its message is carried).
        """
        op = str(command["op"])
        if not self.manager_running():
            raise MapManagerDown("map_manager is not running (nothing listens on /map_manager/command); retry later")
        request_id = self.new_id()
        event: threading.Event = threading.Event()
        answers: list[MapResult] = []
        with self.lock:
            self.pending[request_id] = (event, answers)
        timeout_s = self.timeout_for(op)
        try:
            self.publish(json.dumps({**command, "request_id": request_id}))
            if not event.wait(timeout_s):
                raise MapTimeout(f"map_manager did not answer the {op} within {timeout_s:g} s (request {request_id})")
        finally:
            with self.lock:
                self.pending.pop(request_id, None)
        result = answers[0]
        if not result.ok:
            raise RobotError(f"map_manager rejected the {op}: {result.message or 'no reason given'}")
        return result
