"""Process entry point: one rclpy node, an executor thread and the localhost HTTP API on uvicorn."""

import logging
import os
import subprocess
import sys
import threading
import time
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
import uvicorn
from pydantic import ValidationError
from rclpy.executors import MultiThreadedExecutor

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.arm_camera_api import arm_camera_router
from head_rgbd_calibration.arm_camera_results import FLOW as ARM_CAMERA_FLOW
from head_rgbd_calibration.capture import CaptureController
from head_rgbd_calibration.config import CONFIG_ENV, load_config
from head_rgbd_calibration.csi_intrinsics import CsiCalibration, csi_router
from head_rgbd_calibration.floor_api import floor_router
from head_rgbd_calibration.floor_results import base_urdf
from head_rgbd_calibration.floor_sweep import FloorSweep
from head_rgbd_calibration.ground_api import ground_router
from head_rgbd_calibration.ground_results import FLOW as GROUND_FLOW
from head_rgbd_calibration.ground_results import GroundStore
from head_rgbd_calibration.ground_sweep import GroundSweep
from head_rgbd_calibration.ground_verify import GroundVerify
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
ARM_CAMERA_MODULE = "head_rgbd_calibration.arm_camera_compute"
GROUND_MODULE = "head_rgbd_calibration.ground_compute"
LOGGER = logging.getLogger(NODE_NAME)


def now() -> datetime:
    """Current local time with its UTC offset."""
    return datetime.now().astimezone()


REASON_CAPTURE = "an RGB-D capture is running; stop it first"
REASON_COMPUTE = "a compute is running; wait for it to finish"
REASON_ARM_CAMERA = "an arm to camera compute is running; wait for it to finish"
REASON_SEQUENCE = "a head-arm sequence is running; stop it first"
REASON_FLOOR_SWEEP = "a floor sweep is running; stop it first"
REASON_GROUND_SWEEP = "a ground board session is running; stop it first"
REASON_GROUND_VERIFY = "a ground verification is running; stop it first"
REASON_GROUND_COMPUTE = "a ground board compute is running; wait for it to finish"


def first_running(parts: Sequence[tuple[Any | None, str]]) -> str | None:
    """The reason of the first part that runs.

    Args:
        parts: (object with state() -> {running}, or None to skip it; its reason) in check order.

    Returns:
        The reason, or None when none runs.
    """
    for part, reason in parts:
        if part is not None and part.state()["running"]:
            return reason
    return None


def blocked_reason(
    capture: CaptureController,
    job: ComputeJob,
    sequencer: Sequencer | None = None,
    arm_camera_job: ComputeJob | None = None,
    ground: Sequence[tuple[Any, str]] = (),
) -> str | None:
    """Why a floor sweep may not start now.

    Args:
        capture: RGB-D capture controller.
        job: RGB-D compute job.
        sequencer: Head-from-arm sequencer, None to ignore it.
        arm_camera_job: Arm to camera compute job, None to ignore it.
        ground: The ground board parts that block it (session, verification, compute) with their reasons.

    Returns:
        A reason while a capture, a compute, an arm to camera compute, a head-arm sequence or a ground part runs, else
        None.
    """
    return first_running(
        [
            (capture, REASON_CAPTURE),
            (job, REASON_COMPUTE),
            (arm_camera_job, REASON_ARM_CAMERA),
            (sequencer, REASON_SEQUENCE),
            *ground,
        ]
    )


def sequence_blocked_reason(
    capture: CaptureController,
    job: ComputeJob,
    sweep: FloorSweep,
    arm_camera_job: ComputeJob | None = None,
    ground: Sequence[tuple[Any, str]] = (),
) -> str | None:
    """Why a head-arm sequence may not start now.

    Args:
        capture: RGB-D capture controller.
        job: RGB-D compute job.
        sweep: Floor sweep.
        arm_camera_job: Arm to camera compute job, None to ignore it.
        ground: The ground board parts that block it with their reasons.

    Returns:
        A reason while a capture, a compute, an arm to camera compute, a floor sweep or a ground part runs, else None.
    """
    if sweep.state()["running"]:
        return REASON_FLOOR_SWEEP
    return blocked_reason(capture, job, arm_camera_job=arm_camera_job, ground=ground)


def ground_motion_blocked_reason(
    capture: CaptureController,
    job: ComputeJob,
    arm_camera_job: ComputeJob,
    ground_job: ComputeJob,
    sequencer: Sequencer,
    sweep: FloorSweep,
    other: tuple[Any, str],
) -> str | None:
    """Why a ground board session or a ground verification (both move the head) may not start now.

    Args:
        capture: RGB-D capture controller.
        job: RGB-D compute job.
        arm_camera_job: Arm to camera compute job.
        ground_job: Ground board compute job.
        sequencer: Head-from-arm sequencer.
        sweep: Floor sweep.
        other: The other ground head motion (the verification for a session and back) with its reason.

    Returns:
        A reason while any of them runs, else None.
    """
    return first_running(
        [
            (capture, REASON_CAPTURE),
            (job, REASON_COMPUTE),
            (arm_camera_job, REASON_ARM_CAMERA),
            (ground_job, REASON_GROUND_COMPUTE),
            (sequencer, REASON_SEQUENCE),
            (sweep, REASON_FLOOR_SWEEP),
            other,
        ]
    )


def ground_compute_blocked_reason(
    ground_sweep: GroundSweep, verify: GroundVerify, sweep: FloorSweep, sequencer: Sequencer, arm_camera_job: ComputeJob
) -> str | None:
    """Why a ground board compute may not start now (the ground job itself refuses a second compute).

    Args:
        ground_sweep: Ground board session.
        verify: Ground verification.
        sweep: Floor sweep.
        sequencer: Head-from-arm sequencer.
        arm_camera_job: Arm to camera compute job.

    Returns:
        A reason while any of them runs, else None.
    """
    return first_running(
        [
            (ground_sweep, REASON_GROUND_SWEEP),
            (verify, REASON_GROUND_VERIFY),
            (sweep, REASON_FLOOR_SWEEP),
            (sequencer, REASON_SEQUENCE),
            (arm_camera_job, REASON_ARM_CAMERA),
        ]
    )


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
    ground_store = GroundStore(cfg)
    capture = CaptureController(node, store, cfg, now)
    csi = CsiCalibration(node, cfg, now)
    job = ComputeJob(cfg, store, subprocess.Popen, now, path)
    arm_camera_job = ComputeJob(cfg, store, subprocess.Popen, now, path, module=ARM_CAMERA_MODULE, flow=ARM_CAMERA_FLOW)
    ground_job = ComputeJob(cfg, ground_store, subprocess.Popen, now, path, module=GROUND_MODULE, flow=GROUND_FLOW)

    def ground_parts() -> list[tuple[Any, str]]:
        """The ground board parts that block the floor sweep and the head-arm sequence (bound when called)."""
        return [
            (ground_sweep, REASON_GROUND_SWEEP),
            (ground_verify, REASON_GROUND_VERIFY),
            (ground_job, REASON_GROUND_COMPUTE),
        ]

    sweep = FloorSweep(
        node,
        cfg,
        lambda: base_urdf(cfg),
        time.monotonic,
        time.sleep,
        now,
        lambda: blocked_reason(capture, job, sequencer, arm_camera_job, ground_parts()),
    )
    sequencer = Sequencer(
        capture,
        RobotClient(cfg.robot),
        cfg,
        lambda: base_urdf(cfg),
        lambda: Path(cfg.arm_urdf_file).read_text(),
        lambda: sequence_blocked_reason(capture, job, sweep, arm_camera_job, ground_parts()),
        fitted_mount=lambda sid: latest_board_mount(store, sid),
        board_file=Path(cfg.state_dir) / BOARD_RECORD_FILE,
        session_exists=lambda sid: head_arm_session_exists(store, sid),
    )
    ground_sweep = GroundSweep(
        node,
        cfg,
        lambda: base_urdf(cfg),
        time.monotonic,
        time.sleep,
        now,
        lambda: ground_motion_blocked_reason(
            capture, job, arm_camera_job, ground_job, sequencer, sweep, (ground_verify, REASON_GROUND_VERIFY)
        ),
    )
    ground_verify = GroundVerify(
        node,
        cfg,
        ground_store,
        time.monotonic,
        time.sleep,
        subprocess.run,
        lambda: ground_motion_blocked_reason(
            capture, job, arm_camera_job, ground_job, sequencer, sweep, (ground_sweep, REASON_GROUND_SWEEP)
        ),
    )
    app = build_app(capture, job, store, cfg, subprocess.run)
    app.include_router(floor_router(sweep, capture, job, cfg, subprocess.run))
    app.include_router(head_arm_router(sequencer, capture, sweep, job, store, cfg, subprocess.run))
    app.include_router(csi_router(csi, subprocess.run))
    app.include_router(
        arm_camera_router(
            sequencer,
            arm_camera_job,
            store,
            cfg,
            subprocess.run,
            lambda: first_running(
                [
                    (ground_sweep, REASON_GROUND_SWEEP),
                    (ground_verify, REASON_GROUND_VERIFY),
                    (ground_job, REASON_GROUND_COMPUTE),
                ]
            ),
        )
    )
    app.include_router(
        ground_router(
            ground_sweep,
            ground_verify,
            ground_job,
            ground_store,
            cfg,
            subprocess.run,
            lambda: ground_compute_blocked_reason(ground_sweep, ground_verify, sweep, sequencer, arm_camera_job),
        )
    )
    LOGGER.info("serving on http://%s:%d", cfg.http_host, cfg.http_port)
    try:
        uvicorn.Server(uvicorn.Config(app, host=cfg.http_host, port=cfg.http_port, log_level="info")).run()
    finally:
        sequencer.stop()
        sequencer.wait(SEQUENCE_JOIN_S)
        ground_sweep.stop()
        ground_sweep.wait(SWEEP_JOIN_S)
        ground_verify.stop()
        ground_verify.wait(SWEEP_JOIN_S)
        capture.stop()
        csi.stop()
        sweep.stop()
        sweep.wait(SWEEP_JOIN_S)
        job.stop()
        arm_camera_job.stop()
        ground_job.stop()
        node.destroy_node()
        executor.shutdown()
        spinner.join(SPIN_JOIN_S)
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
