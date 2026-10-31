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
from head_rgbd_calibration.csi_intrinsics import CsiCalibration, csi_router
from head_rgbd_calibration.floor_api import floor_router
from head_rgbd_calibration.floor_results import base_urdf
from head_rgbd_calibration.floor_sweep import FloorSweep
from head_rgbd_calibration.head_arm_api import head_arm_router
from head_rgbd_calibration.head_arm_results import head_arm_session_exists, latest_board_mount
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.robot_client import RobotClient
from head_rgbd_calibration.sequencer import BOARD_RECORD_FILE, Sequencer
from head_rgbd_calibration.store import SessionStore

NODE_NAME = "head_rgbd_calibration"
EXECUTOR_THREADS = 4
SPIN_JOIN_S = 5.0
SWEEP_JOIN_S = 15.0
SEQUENCE_JOIN_S = 15.0
LOGGER = logging.getLogger(NODE_NAME)


def now() -> datetime:
    """Current local time with its UTC offset."""
    return datetime.now().astimezone()


def blocked_reason(capture: CaptureController, job: ComputeJob, sequencer: Sequencer | None = None) -> str | None:
    """Why a floor sweep may not start now.

    Args:
        capture: RGB-D capture controller.
        job: RGB-D compute job.
        sequencer: Head-from-arm sequencer, None to ignore it.

    Returns:
        A reason while a capture, a compute or a head-arm sequence runs, else None.
    """
    if capture.state()["running"]:
        return "an RGB-D capture is running; stop it first"
    if job.state()["running"]:
        return "a compute is running; wait for it to finish"
    if sequencer is not None and sequencer.state()["running"]:
        return "a head-arm sequence is running; stop it first"
    return None


def sequence_blocked_reason(capture: CaptureController, job: ComputeJob, sweep: FloorSweep) -> str | None:
    """Why a head-arm sequence may not start now.

    Args:
        capture: RGB-D capture controller.
        job: RGB-D compute job.
        sweep: Floor sweep.

    Returns:
        A reason while a capture, a compute or a floor sweep runs, else None.
    """
    if sweep.state()["running"]:
        return "a floor sweep is running; stop it first"
    return blocked_reason(capture, job)


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
    csi = CsiCalibration(node, cfg, now)
    job = ComputeJob(cfg, store, subprocess.Popen, now, path)
    sweep = FloorSweep(
        node,
        cfg,
        lambda: base_urdf(cfg),
        time.monotonic,
        time.sleep,
        now,
        lambda: blocked_reason(capture, job, sequencer),
    )
    sequencer = Sequencer(
        capture,
        RobotClient(cfg.robot),
        cfg,
        lambda: base_urdf(cfg),
        lambda: Path(cfg.arm_urdf_file).read_text(),
        lambda: sequence_blocked_reason(capture, job, sweep),
        fitted_mount=lambda sid: latest_board_mount(store, sid),
        board_file=Path(cfg.state_dir) / BOARD_RECORD_FILE,
        session_exists=lambda sid: head_arm_session_exists(store, sid),
    )
    app = build_app(capture, job, store, cfg, subprocess.run)
    app.include_router(floor_router(sweep, capture, job, cfg, subprocess.run))
    app.include_router(head_arm_router(sequencer, capture, sweep, job, store, cfg, subprocess.run))
    app.include_router(csi_router(csi, subprocess.run))
    LOGGER.info("serving on http://%s:%d", cfg.http_host, cfg.http_port)
    try:
        uvicorn.Server(uvicorn.Config(app, host=cfg.http_host, port=cfg.http_port, log_level="info")).run()
    finally:
        sequencer.stop()
        sequencer.wait(SEQUENCE_JOIN_S)
        capture.stop()
        csi.stop()
        sweep.stop()
        sweep.wait(SWEEP_JOIN_S)
        job.stop()
        node.destroy_node()
        executor.shutdown()
        spinner.join(SPIN_JOIN_S)
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
