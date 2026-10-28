"""Tests for the marker detector pipeline and node wiring on synthetic scenes (no ROS)."""

import json
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.marker_payload import decode_payload

from marker_detector.config import MarkerDetectorConfig
from marker_detector.decode import Candidate
from marker_detector.pipeline import FrameOutput, MarkerPipeline
from marker_detector.synthetic import (
    STREAM_K,
    STREAM_SIZE,
    pose_facing,
    project_points,
    render_marker_scene,
    symbol_corners_m,
    symbol_matrix,
)
from tests.ros_stubs import install_ros_stubs

FRAME = "d435_color_optical_frame"
PAYLOAD = "P1/K:BASE/N:HOME/S:150"
AMBIGUOUS_PAYLOAD = "P1/K:BASE/N:HOME/S:100"
STAMP = 5_000_000_000
IDENTITY = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
DEPTH_SIZE = (288, 162)
DEPTH_SCALE = DEPTH_SIZE[0] / STREAM_SIZE[0]
DEPTH_K = np.array(
    [
        [STREAM_K[0, 0] * DEPTH_SCALE, 0.0, (STREAM_K[0, 2] + 0.5) * DEPTH_SCALE - 0.5],
        [0.0, STREAM_K[1, 1] * DEPTH_SCALE, (STREAM_K[1, 2] + 0.5) * DEPTH_SCALE - 0.5],
        [0.0, 0.0, 1.0],
    ]
)
DEPTH_INTRINSICS = CameraIntrinsics.from_camera_info(
    DEPTH_SIZE[0], DEPTH_SIZE[1], DEPTH_K.ravel(), [0.0] * 5, "plumb_bob"
)


def identity_tf(frame_id: str, stamp_ns: int):
    return IDENTITY


def no_tf(frame_id: str, stamp_ns: int):
    return None


def to_jpeg(grey: np.ndarray) -> bytes:
    ok, data = cv2.imencode(".jpg", grey, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return data.tobytes()


def make_pipeline(calibrated: bool = True, **overrides) -> MarkerPipeline:
    cfg = MarkerDetectorConfig.model_validate(overrides)
    pipeline = MarkerPipeline(cfg)
    if calibrated:
        pipeline.on_camera_info(STREAM_K, np.zeros(5), STREAM_SIZE, FRAME)
    return pipeline


def scene(payload: str, size_mm: float, pose, noise: float = 0.5, seed: int = 0) -> bytes:
    return to_jpeg(render_marker_scene([payload], [size_mm], [pose], STREAM_K, STREAM_SIZE, noise_px=noise, seed=seed))


def depth_of_plane(pose, noise_mm: float = 2.0, seed: int = 0) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(pose[0], dtype=float))
    normal, point = rotation[:, 2], np.asarray(pose[1], dtype=float)
    u, v = np.meshgrid(np.arange(DEPTH_SIZE[0], dtype=float), np.arange(DEPTH_SIZE[1], dtype=float))
    rays = np.stack([(u - DEPTH_K[0, 2]) / DEPTH_K[0, 0], (v - DEPTH_K[1, 2]) / DEPTH_K[1, 1], np.ones_like(u)], -1)
    z_mm = float(normal @ point) / (rays @ normal) * 1000.0 + np.random.default_rng(seed).normal(0, noise_mm, u.shape)
    return np.clip(np.round(z_mm), 0, 65535).astype(np.uint16)


def ambiguous_candidate(seed: int) -> tuple[bytes, Candidate, tuple]:
    """100 mm marker, yaw 12 deg, 1.3 m away, 1 px corner noise: the IPPE flip reprojects almost as well."""
    pose = pose_facing(1.3, yaw_deg=12.0)
    jpeg = scene(AMBIGUOUS_PAYLOAD, 100.0, pose, noise=2.0, seed=seed)
    corners = project_points(symbol_corners_m(100.0), pose, STREAM_K)
    corners = corners + np.random.default_rng(seed + 1000).normal(0.0, 1.0, corners.shape)
    modules = symbol_matrix(AMBIGUOUS_PAYLOAD).shape[0]
    module_px = float(np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1).mean() / modules)
    candidate = Candidate(decode_payload(AMBIGUOUS_PAYLOAD), AMBIGUOUS_PAYLOAD, corners, (modules - 17) // 4, module_px)
    return jpeg, candidate, pose


def patch_decode(monkeypatch, candidates: list[Candidate]) -> None:
    import marker_detector.pipeline as pipeline_module

    monkeypatch.setattr(pipeline_module, "decode_frame", lambda grey: (list(candidates), []))


def test_one_marker_gives_one_upsert_and_report():
    pipeline = make_pipeline()
    pose = pose_facing(1.0, yaw_deg=20.0, offset_m=(0.05, -0.03))
    out = pipeline.process_frame(STAMP, scene(PAYLOAD, 150.0, pose), identity_tf)
    assert isinstance(out, FrameOutput)
    assert len(out.commands) == 1
    cmd = out.commands[0]
    assert cmd["op"] == "upsert_marker"
    assert cmd["poi"]["kind"] == "marker"
    assert cmd["poi"]["marker"]["key"] == decode_payload(PAYLOAD).key
    position = np.array([cmd["poi"]["x"], cmd["poi"]["y"], cmd["poi"]["z"]])
    truth = np.asarray(pose[1])
    assert np.linalg.norm(position - truth) < 0.01 * np.linalg.norm(truth)
    assert out.report is not None
    report = json.loads(out.report)
    assert report["stamp_ns"] == STAMP and report["source"] == "stream"
    assert report["markers"][0]["key"] == decode_payload(PAYLOAD).key
    assert report["markers"][0]["chooser"] == "reprojection"
    assert not hasattr(out, "zoom_request")


def test_no_camera_info_gives_nothing():
    pipeline = make_pipeline(calibrated=False)
    out = pipeline.process_frame(STAMP, scene(PAYLOAD, 150.0, pose_facing(1.0)), identity_tf)
    assert out.commands == [] and out.report is None


def test_zero_camera_matrix_counts_as_no_camera_info():
    pipeline = make_pipeline(calibrated=False)
    pipeline.on_camera_info(np.zeros((3, 3)), np.zeros(5), STREAM_SIZE, FRAME)
    out = pipeline.process_frame(STAMP, scene(PAYLOAD, 150.0, pose_facing(1.0)), identity_tf)
    assert out.commands == []


def test_tf_failure_gives_nothing():
    pipeline = make_pipeline()
    out = pipeline.process_frame(STAMP, scene(PAYLOAD, 150.0, pose_facing(1.0, yaw_deg=15.0)), no_tf)
    assert out.commands == [] and out.report is None
    assert out.rejections.get("tf") == 1


def test_undecodable_jpeg_gives_nothing():
    out = make_pipeline().process_frame(STAMP, b"not a jpeg", identity_tf)
    assert out.commands == [] and out.report is None


def test_invalid_payload_gives_payload_rejection():
    pipeline = make_pipeline()
    grey = render_marker_scene(["HELLO WORLD"], [150.0], [pose_facing(1.0)], STREAM_K, STREAM_SIZE)
    out = pipeline.process_frame(STAMP, to_jpeg(grey), identity_tf)
    assert out.commands == [] and out.report is None
    assert out.rejections.get("payload") == 1


def test_ambiguous_without_depth_gives_nothing(monkeypatch):
    jpeg, candidate, _ = ambiguous_candidate(1)
    patch_decode(monkeypatch, [candidate])
    pipeline = make_pipeline(pose={"min_module_px": 1.0})
    out = pipeline.process_frame(STAMP, jpeg, identity_tf)
    assert out.commands == []
    assert out.rejections.get("ambiguous") == 1


def test_ambiguous_with_depth_plane_gives_depth_command(monkeypatch):
    jpeg, candidate, pose = ambiguous_candidate(1)
    patch_decode(monkeypatch, [candidate])
    pipeline = make_pipeline(pose={"min_module_px": 1.0})
    pipeline.on_depth(STAMP + 50_000_000, depth_of_plane(pose), DEPTH_INTRINSICS)
    out = pipeline.process_frame(STAMP, jpeg, identity_tf)
    assert len(out.commands) == 1
    assert json.loads(out.report)["markers"][0]["chooser"] == "depth"
    position = np.array([out.commands[0]["poi"][axis] for axis in "xyz"])
    assert np.linalg.norm(position - np.asarray(pose[1])) < 0.05


def test_depth_too_old_is_not_used(monkeypatch):
    jpeg, candidate, pose = ambiguous_candidate(1)
    patch_decode(monkeypatch, [candidate])
    pipeline = make_pipeline(pose={"min_module_px": 1.0})
    pipeline.on_depth(STAMP + 2_000_000_000, depth_of_plane(pose), DEPTH_INTRINSICS)
    assert pipeline.process_frame(STAMP, jpeg, identity_tf).commands == []


def test_depth_plane_disabled_ignores_depth(monkeypatch):
    jpeg, candidate, pose = ambiguous_candidate(1)
    patch_decode(monkeypatch, [candidate])
    pipeline = make_pipeline(pose={"min_module_px": 1.0}, depth_plane={"enabled": False})
    pipeline.on_depth(STAMP, depth_of_plane(pose), DEPTH_INTRINSICS)
    assert pipeline.process_frame(STAMP, jpeg, identity_tf).commands == []


def test_depth_of_another_size_than_its_camera_info_is_dropped(monkeypatch):
    jpeg, candidate, pose = ambiguous_candidate(1)
    patch_decode(monkeypatch, [candidate])
    pipeline = make_pipeline(pose={"min_module_px": 1.0})
    pipeline.on_depth(STAMP, depth_of_plane(pose)[:, :-1], DEPTH_INTRINSICS)
    assert len(pipeline.depth_frames) == 0
    assert pipeline.process_frame(STAMP, jpeg, identity_tf).commands == []


def test_new_depth_intrinsics_drop_the_older_frames(monkeypatch):
    jpeg, candidate, pose = ambiguous_candidate(1)
    patch_decode(monkeypatch, [candidate])
    pipeline = make_pipeline(pose={"min_module_px": 1.0})
    pipeline.on_depth(STAMP, depth_of_plane(pose), DEPTH_INTRINSICS)
    half = CameraIntrinsics.from_camera_info(DEPTH_SIZE[0] // 2, DEPTH_SIZE[1] // 2, (DEPTH_K / 2).ravel(), [], "")
    pipeline.on_depth(STAMP + 1, np.zeros((DEPTH_SIZE[1] // 2, DEPTH_SIZE[0] // 2), dtype=np.uint16), half)
    assert [stamp for stamp, _ in pipeline.depth_frames] == [STAMP + 1]
    assert pipeline.depth_intrinsics == half


def test_low_confidence_sends_nothing():
    pipeline = make_pipeline(pose={"min_confidence": 1.0})
    out = pipeline.process_frame(STAMP, scene(PAYLOAD, 150.0, pose_facing(1.0, yaw_deg=20.0)), identity_tf)
    assert out.commands == [] and out.report is None
    assert out.rejections.get("low_confidence") == 1


# --- node wiring --------------------------------------------------------------------------------------------------


def test_node_module_imports_and_wires_topics():
    install_ros_stubs()
    from marker_detector.node import MarkerDetectorRos

    cfg = MarkerDetectorConfig.model_validate(
        {
            "process_hz": 4.0,
            "topics": {
                "image": "/x/image",
                "camera_info": "/x/info",
                "depth": "/x/depth",
                "depth_info": "/x/depth_info",
                "poi_command": "/x/poi",
                "detections": "/x/detections",
            },
        }
    )
    node = MagicMock()
    MarkerDetectorRos(cfg, node, MarkerPipeline(cfg))
    subscribed = {call.args[1] for call in node.create_subscription.call_args_list}
    assert subscribed == {"/x/image", "/x/info", "/x/depth", "/x/depth_info"}
    publishers = {call.args[1]: call.args[2] for call in node.create_publisher.call_args_list}
    assert set(publishers) == {"/x/poi", "/x/detections"}
    command_qos = publishers["/x/poi"]
    assert (command_qos.reliability.name, command_qos.history.name, command_qos.depth) == ("RELIABLE", "KEEP_LAST", 20)
    image_qos = next(c.args[3] for c in node.create_subscription.call_args_list if c.args[1] == "/x/image")
    assert (image_qos.reliability.name, image_qos.history.name, image_qos.depth) == ("BEST_EFFORT", "KEEP_LAST", 1)
    assert node.create_timer.call_args.args[0] == pytest.approx(0.25)


def test_node_skips_depth_subscriptions_when_disabled():
    install_ros_stubs()
    from marker_detector.node import MarkerDetectorRos

    cfg = MarkerDetectorConfig.model_validate({"depth_plane": {"enabled": False}})
    node = MagicMock()
    MarkerDetectorRos(cfg, node, MarkerPipeline(cfg))
    subscribed = [call.args[1] for call in node.create_subscription.call_args_list]
    assert sorted(subscribed) == sorted([cfg.topics.image, cfg.topics.camera_info])


def depth_info_msg(k: list[float], distortion_model: str = "plumb_bob"):
    from types import SimpleNamespace

    return SimpleNamespace(
        width=DEPTH_SIZE[0], height=DEPTH_SIZE[1], k=k, d=[0.0] * 5, distortion_model=distortion_model
    )


def depth_msg(depth: np.ndarray, stamp_ns: int = STAMP):
    from types import SimpleNamespace

    stamp = SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        encoding="16UC1",
        height=depth.shape[0],
        width=depth.shape[1],
        step=depth.shape[1] * 2,
        data=depth.tobytes(),
    )


def test_node_depth_info_gives_shared_intrinsics_to_the_pipeline():
    install_ros_stubs()
    from marker_detector.node import MarkerDetectorRos

    cfg = MarkerDetectorConfig()
    pipeline = MagicMock()
    wrapper = MarkerDetectorRos(cfg, MagicMock(), pipeline)
    depth = np.full((DEPTH_SIZE[1], DEPTH_SIZE[0]), 1000, dtype=np.uint16)
    wrapper.on_depth(depth_msg(depth))
    pipeline.on_depth.assert_not_called()  # no depth camera_info yet
    wrapper.on_depth_info(depth_info_msg(list(DEPTH_K.ravel())))
    wrapper.on_depth(depth_msg(depth))
    stamp, image, intrinsics = pipeline.on_depth.call_args.args
    assert stamp == STAMP
    np.testing.assert_array_equal(image, depth)
    assert intrinsics == DEPTH_INTRINSICS


def test_node_drops_depth_after_an_invalid_depth_info():
    install_ros_stubs()
    from marker_detector.node import MarkerDetectorRos

    pipeline = MagicMock()
    wrapper = MarkerDetectorRos(MarkerDetectorConfig(), MagicMock(), pipeline)
    wrapper.on_depth_info(depth_info_msg(list(DEPTH_K.ravel())))
    wrapper.on_depth_info(depth_info_msg([0.0] * 9))
    wrapper.on_depth(depth_msg(np.full((DEPTH_SIZE[1], DEPTH_SIZE[0]), 1000, dtype=np.uint16)))
    pipeline.on_depth.assert_not_called()


def test_run_node_tf_listener_uses_spin_thread():
    install_ros_stubs()
    import marker_detector.node as node_module

    listener = MagicMock()
    with (
        patch.object(node_module.tf2_ros, "TransformListener", listener),
        patch.object(node_module, "start_metrics_server"),
        patch.object(node_module, "MarkerDetectorRos"),
    ):
        node_module.run_node(MarkerDetectorConfig())
    assert listener.call_args.kwargs.get("spin_thread") is True
