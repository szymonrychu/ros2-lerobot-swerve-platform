"""Process entry point: one rclpy node, an executor thread and the localhost HTTP API on uvicorn."""

import logging
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
import uvicorn
from pydantic import ValidationError
from rclpy.executors import MultiThreadedExecutor

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.capture import CaptureController
from head_rgbd_calibration.config import CONFIG_ENV, load_config
from head_rgbd_calibration.floor_api import floor_router
from head_rgbd_calibration.floor_results import base_urdf
from head_rgbd_calibration.floor_sweep import FloorSweep
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.store import SessionStore

NODE_NAME = "head_rgbd_calibration"
EXECUTOR_THREADS = 4
SPIN_JOIN_S = 5.0
SWEEP_JOIN_S = 15.0
LOGGER = logging.getLogger(NODE_NAME)


def now() -> datetime:
    """Current local time with its UTC offset."""
    return datetime.now().astimezone()


def blocked_reason(capture: CaptureController, job: ComputeJob) -> str | None:
    """Why a floor sweep may not start now.

    Args:
        capture: RGB-D capture controller.
        job: RGB-D compute job.

    Returns:
        A reason while a capture or a compute runs, else None.
    """
    if capture.state()["running"]:
        return "an RGB-D capture is running; stop it first"
    if job.state()["running"]:
        return "a compute is running; wait for it to finish"
    return None


def main() -> int:
    """Load the config, start the node and serve the HTTP API until uvicorn exits.

    Returns:
        0 on clean shutdown, os.EX_CONFIG (78) when the config path is unset or the config is invalid.
    """
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    path = os.environ.get(CONFIG_ENV)
    if not path:
        LOGGER.error("%s is not set", CONFIG_ENV)
        return os.EX_CONFIG
    try:
        cfg = load_config(path)
    except (OSError, ValidationError, ValueError) as exc:
        LOGGER.error("invalid or missing config %s: %s", path, exc)
        return os.EX_CONFIG
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    spinner.start()
    store = SessionStore(Path(cfg.state_dir), cfg)
    capture = CaptureController(node, store, cfg, now)
    job = ComputeJob(cfg, store, subprocess.Popen, now, path)
    sweep = FloorSweep(
        node,
        cfg,
        lambda: base_urdf(cfg),
        time.monotonic,
        time.sleep,
        now,
        lambda: blocked_reason(capture, job),
    )
    app = build_app(capture, job, store, cfg, subprocess.run)
    app.include_router(floor_router(sweep, capture, job, cfg, subprocess.run))
    LOGGER.info("serving on http://%s:%d", cfg.http_host, cfg.http_port)
    try:
        uvicorn.Server(uvicorn.Config(app, host=cfg.http_host, port=cfg.http_port, log_level="info")).run()
    finally:
        sweep.stop()
        sweep.wait(SWEEP_JOIN_S)
        capture.stop()
        job.stop()
        node.destroy_node()
        executor.shutdown()
        spinner.join(SPIN_JOIN_S)
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
