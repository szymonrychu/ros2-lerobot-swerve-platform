"""Client side of the head camera zoom topics (no ROS): command, result and image matched by request_id and stamp.

The contract is nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp. The image carries no request id, so it is
paired with a result by its header stamp (the result's stamp_ns); it may arrive before or after the result.
"""

import json
import threading
import time
import uuid
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .models import RobotError

IMAGE_STATUSES = ("ok", "af_failed")
FAILURE_STATUSES = ("busy", "timeout", "camera_error", "invalid")
KEPT_IMAGES = 4  # image stamps remembered, for an image that arrives before its result


class ZoomNodeDown(RobotError):
    """Nothing listens on the zoom command topic (the head camera node is not running)."""


class ZoomTimeout(RobotError):
    """The head camera did not answer, or its image did not arrive, in time."""


class ZoomFailed(RobotError):
    """The head camera answered with a failure status (busy, timeout, camera_error or invalid)."""

    def __init__(self, status: str, message: str) -> None:
        """Create the error.

        Args:
            status (str): The result status.
            message (str): The node's explanation.
        """
        super().__init__(f"head camera zoom {status}: {message}")
        self.status = status
        self.message = message


@dataclass
class ZoomReply:
    """A successful (ok or af_failed) zoom: the JPEG and the result fields."""

    jpeg: bytes
    status: str
    message: str
    af_state: str
    lens_position: float | None
    crop: list[int] | None
    full: list[int]
    out: list[int] | None
    stamp_ns: int
    stamp_s: float
    phase_ms: dict[str, float]


def valid_result(result: Any) -> bool:
    """Whether a parsed result has the fields the client needs.

    Args:
        result (Any): Parsed JSON.

    Returns:
        bool: True for a well-formed result.
    """
    if not isinstance(result, dict) or not isinstance(result.get("request_id"), str):
        return False
    status = result.get("status")
    if status in FAILURE_STATUSES:
        return True
    return (
        status in IMAGE_STATUSES
        and isinstance(result.get("stamp_ns"), int)
        and isinstance(result.get("full"), list)
        and isinstance(result.get("phase_ms"), dict)
    )


class ZoomRequests:
    """Publishes zoom commands and matches the result and image answers to them."""

    def __init__(
        self,
        publish: Callable[[str], None],
        node_running: Callable[[], bool],
        timeout_s: float,
        image_grace_s: float,
        new_id: Callable[[], str] = lambda: uuid.uuid4().hex,
    ) -> None:
        """Create the client.

        Args:
            publish (Callable[[str], None]): Publishes one JSON string on the zoom command topic.
            node_running (Callable[[], bool]): Whether the node listens (a subscriber exists on the command topic).
            timeout_s (float): How long to wait for the result.
            image_grace_s (float): How long to wait for the image after an ok/af_failed result.
            new_id (Callable[[], str]): Request id factory.
        """
        self.publish = publish
        self.node_running = node_running
        self.timeout_s = timeout_s
        self.image_grace_s = image_grace_s
        self.new_id = new_id
        self.cond = threading.Condition()
        self.pending: dict[str, list[dict[str, Any]]] = {}
        self.images: OrderedDict[int, bytes] = OrderedDict()

    def pending_count(self) -> int:
        """Number of requests still waiting for an answer.

        Returns:
            int: Pending request count.
        """
        with self.cond:
            return len(self.pending)

    def on_result(self, payload: str) -> None:
        """Feed one result JSON string; unknown request ids and malformed payloads are dropped.

        Args:
            payload (str): The std_msgs/String data.
        """
        try:
            result = json.loads(payload)
        except ValueError:
            return
        if not valid_result(result):
            return
        with self.cond:
            answers = self.pending.get(result["request_id"])
            if answers is not None and not answers:
                answers.append(result)
                self.cond.notify_all()

    def on_image(self, stamp_ns: int, jpeg: bytes) -> None:
        """Feed one zoom image; the last KEPT_IMAGES are remembered.

        Args:
            stamp_ns (int): The image header stamp in nanoseconds.
            jpeg (bytes): The JPEG data.
        """
        with self.cond:
            self.images[stamp_ns] = jpeg
            self.images.move_to_end(stamp_ns)
            while len(self.images) > KEPT_IMAGES:
                self.images.popitem(last=False)
            self.cond.notify_all()

    def request(self, u: float, v: float, zoom: int, max_px: int, af: bool = True) -> ZoomReply:
        """Send one zoom command and wait for its result and image.

        Args:
            u (float): Normalised x in the default view, in [0, 1].
            v (float): Normalised y in the default view, in [0, 1].
            zoom (int): Zoom factor (2, 4 or 8).
            max_px (int): Longest side of the output image.
            af (bool): Run autofocus before the still.

        Returns:
            ZoomReply: The image and result fields for an ok or af_failed result.
        """
        if not self.node_running():
            raise ZoomNodeDown("head camera is not running (nothing listens on the zoom command topic)")
        request_id = self.new_id()
        answers: list[dict[str, Any]] = []
        with self.cond:
            self.pending[request_id] = answers
        try:
            self.publish(
                json.dumps({"request_id": request_id, "u": u, "v": v, "zoom": zoom, "max_px": max_px, "af": af})
            )
            with self.cond:
                if not self.cond.wait_for(lambda: bool(answers), self.timeout_s):
                    raise ZoomTimeout(f"head camera did not answer within {self.timeout_s:g} s (request {request_id})")
                result = answers[0]
                if result["status"] in FAILURE_STATUSES:
                    raise ZoomFailed(result["status"], str(result.get("message", "")))
                stamp_ns = result["stamp_ns"]
                deadline = time.monotonic() + self.image_grace_s
                while stamp_ns not in self.images:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise ZoomTimeout(
                            f"zoom image (stamp {stamp_ns}) did not arrive within {self.image_grace_s:g} s of the result"
                        )
                    self.cond.wait(remaining)
                jpeg = self.images[stamp_ns]
        finally:
            with self.cond:
                self.pending.pop(request_id, None)
        return ZoomReply(
            jpeg=jpeg,
            status=result["status"],
            message=str(result.get("message", "")),
            af_state=str(result.get("af_state", "")),
            lens_position=result.get("lens_position"),
            crop=result.get("crop"),
            full=result["full"],
            out=result.get("out"),
            stamp_ns=stamp_ns,
            stamp_s=float(result.get("stamp_s", stamp_ns / 1e9)),
            phase_ms=result["phase_ms"],
        )
