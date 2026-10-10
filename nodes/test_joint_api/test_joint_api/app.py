"""aiohttp app: GET/POST /joint-updates; POST publishes to ROS2 filter input topic."""

import json
import threading
import time
from typing import Any

from aiohttp import web
from ros2_metrics import render_latest

from .config import ApiConfig
from .metrics import REQUESTS

# Set by run.py after ROS2 node and publisher are created.
_publisher: Any = None
_clock: Any = None
_last_updates: dict[str, float] = {}
_last_updates_time: float = 0.0
_lock = threading.Lock()


def set_ros_publisher(publisher: Any, clock: Any) -> None:
    """Set the ROS2 publisher and clock for use in request handlers."""
    global _publisher, _clock
    _publisher = publisher
    _clock = clock


def _publish_joint_state(names: list[str], positions: list[float]) -> None:
    """Publish a JointState message to the filter input topic."""
    if _publisher is None:
        return
    from sensor_msgs.msg import JointState

    msg = JointState()
    if _clock is not None:
        msg.header.stamp = _clock.now().to_msg()
    msg.header.frame_id = ""
    msg.name = names
    msg.position = positions
    msg.velocity = []
    msg.effort = []
    _publisher.publish(msg)


async def get_joint_updates(_request: web.Request) -> web.Response:
    """GET /joint-updates: return latest posted joint map (radians) and metadata."""
    with _lock:
        data = dict(_last_updates)
        ts = _last_updates_time
    return web.json_response({"joints": data, "timestamp": ts})


async def post_joint_updates(request: web.Request) -> web.Response:
    """POST /joint-updates: accept JSON joint map (radians), publish to filter, store for GET."""
    try:
        body = await request.json()
    except json.JSONDecodeError as e:
        return web.json_response({"error": "Invalid JSON", "detail": str(e)}, status=400)
    if not isinstance(body, dict):
        return web.json_response({"error": "Body must be a JSON object (joint name -> radians)"}, status=400)
    # Support single joint or multiple: {"joint_6": 0.1} or {"joint_5": 0.0, "joint_6": -0.2}
    joints: dict[str, float] = {}
    for k, v in body.items():
        name = str(k).strip()
        if not name:
            continue
        try:
            joints[name] = float(v)
        except (TypeError, ValueError):
            return web.json_response(
                {"error": "Values must be numbers (radians)", "key": name},
                status=400,
            )
    if not joints:
        return web.json_response({"error": "At least one joint name and value required"}, status=400)
    names = list(joints.keys())
    positions = [joints[n] for n in names]
    _publish_joint_state(names, positions)
    global _last_updates, _last_updates_time  # noqa: PLW0603
    with _lock:
        _last_updates = dict(joints)
        _last_updates_time = time.time()
    return web.json_response({"ok": True, "joints": joints})


@web.middleware
async def count_requests(request: web.Request, handler: Any) -> web.StreamResponse:
    """Count every answered request by status code (aiohttp HTTP errors such as 404 included).

    Args:
        request (web.Request): Incoming request.
        handler (Any): Next handler in the chain.

    Returns:
        web.StreamResponse: The handler's response.
    """
    try:
        response = await handler(request)
    except web.HTTPException as exc:
        REQUESTS.labels(str(exc.status)).inc()
        raise
    REQUESTS.labels(str(response.status)).inc()
    return response


async def get_metrics(_request: web.Request) -> web.Response:
    """GET /metrics: Prometheus text exposition of the default registry."""
    body, content_type = render_latest()
    return web.Response(body=body, headers={"Content-Type": content_type})


def create_app(_config: ApiConfig) -> web.Application:
    """Create aiohttp Application with GET/POST /joint-updates and GET /metrics routes."""
    app = web.Application(middlewares=[count_requests])
    app.router.add_get("/metrics", get_metrics)
    app.router.add_get("/joint-updates", get_joint_updates)
    app.router.add_post("/joint-updates", post_joint_updates)
    return app
