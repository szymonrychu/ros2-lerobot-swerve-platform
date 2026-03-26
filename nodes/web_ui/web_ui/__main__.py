"""Entry point: configure logging, load config, start rclpy + uvicorn in one asyncio loop."""

from __future__ import annotations

import os
import signal
import threading
from pathlib import Path

import rclpy
import structlog
import uvicorn

from .bridge import BridgeNode, create_bridge_executor
from .config import load_config
from .logging_setup import configure_logging
from .server import build_app

configure_logging()
log = structlog.get_logger(__name__)

STATIC_DIR = Path(__file__).parent / "static"
URDF_DIR = Path(os.environ.get("WEB_UI_URDF_DIR", "/etc/ros2-nodes/web_ui/urdf"))
if not URDF_DIR.exists():
    URDF_DIR = Path(__file__).parent.parent / "urdf"


def main() -> None:
    """Start the web_ui node: rclpy in a background thread + uvicorn in the main thread."""
    config_path_env = os.environ.get("WEB_UI_CONFIG")
    config_path = Path(config_path_env) if config_path_env else None
    config = load_config(config_path)

    log.info(
        "startup",
        port=config.http_port,
        config_path=str(config_path or "default"),
        log_level=os.environ.get("DEBUG", "false"),
        urdf_dir=str(URDF_DIR),
    )

    rclpy.init()
    node = BridgeNode(
        topics=config.all_subscribed_topics(),
        allowed_publish_topics=set(config.publish_topics()),
    )
    executor = create_bridge_executor(node)

    stop_event = threading.Event()

    def spin_ros() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.05)

    ros_thread = threading.Thread(target=spin_ros, daemon=True, name="rclpy-spin")
    ros_thread.start()

    app = build_app(config=config, urdf_dir=URDF_DIR, static_dir=STATIC_DIR, bridge_node=node)

    def handle_shutdown(signum: int, frame: object) -> None:
        log.info("shutdown", reason=f"signal {signum}")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    uvicorn.run(app, host="0.0.0.0", port=config.http_port, log_level="warning")

    stop_event.set()
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
