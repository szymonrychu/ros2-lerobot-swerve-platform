"""ToF cloud self filter (laser_filter.self_filter, no ROS): the per-point keep mask against the deployed self polygon
(H body + parked gripper, notches kept, height limit, non-finite removed), the runtime polygon replacement with a
released arm shadow, the cloud frame -> polygon_frame transform helper, the whole cloud filter in the cloud frame, the
newest-wins pending slot for clouds waiting for TF and the cloud config keys."""

import math
from pathlib import Path

import numpy as np
import pytest
import shapely
from pydantic import ValidationError
from shapely.geometry import Polygon, box

from laser_filter.config import LaserFilterConfig, load_config
from laser_filter.self_filter import (
    PendingSlot,
    SelfFilter,
    apply_polygon_param,
    cloud_self_mask,
    parse_polygon,
    quaternion_matrix,
    transform_points,
)

DEPLOYED = Path(__file__).resolve().parent.parent / "config" / "footprint_filter.yaml"
GENERATED_BEGIN = "# >>> gen_nav_footprint: laser_polygon"
GENERATED_END = "# <<< gen_nav_footprint: laser_polygon"
MAX_HEIGHT_M = 1.0
SIDE_FRAME = (0.0, 0.15, 0.10)  # on the side rail of the H
PARKED_GRIPPER = (0.30, -0.05, 0.20)  # the parked arm's gripper in front of the body
FRONT_NOTCH = (0.21, 0.04, 0.10)  # front notch beside the parked gripper: a real obstacle
REAR_NOTCH = (-0.19, 0.0, 0.10)  # rear notch between the rear wheels, behind the crossbar: a real obstacle
ABOVE_BODY = (0.0, 0.15, 1.20)  # over the robot, above cloud_self_max_height_m: a real overhang
CLIFF = (1.0, 0.0, -0.30)  # below the floor in front of the robot: a cliff
ARM_POINT = (0.50, 0.0, 0.30)  # the released arm reaching forward
QUARTER = math.sin(math.pi / 8), math.cos(math.pi / 8)


def deployed_polygon() -> list[tuple[float, float]]:
    return parse_polygon(load_config(DEPLOYED).polygon)


def prepared(points: list[tuple[float, float]]) -> Polygon:
    shape = Polygon(points)
    shapely.prepare(shape)
    return shape


def polygon_string(shape: Polygon) -> str:
    return str([[round(x, 4), round(y, 4)] for x, y in list(shape.exterior.coords)[:-1]])


def head_transform() -> tuple[np.ndarray, np.ndarray]:
    """tof_optical_frame -> base_link: head 0.10 m ahead and 0.60 m up, panned 30 deg and tilted 20 deg down, optical
    axes (z forward, x right, y down)."""
    pan = quaternion_matrix(0.0, 0.0, math.sin(math.radians(15)), math.cos(math.radians(15)))
    tilt = quaternion_matrix(0.0, math.sin(math.radians(10)), 0.0, math.cos(math.radians(10)))
    optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    return pan @ tilt @ optical, np.array([0.10, 0.0, 0.60])


def test_points_on_the_side_frame_and_parked_gripper_are_removed() -> None:
    keep = cloud_self_mask(np.array([SIDE_FRAME, PARKED_GRIPPER]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.dtype == bool and keep.tolist() == [False, False]


def test_points_in_the_front_and_rear_notch_are_kept() -> None:
    keep = cloud_self_mask(np.array([FRONT_NOTCH, REAR_NOTCH]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.tolist() == [True, True]


def test_points_above_the_height_limit_inside_the_polygon_are_kept() -> None:
    at_limit = (SIDE_FRAME[0], SIDE_FRAME[1], MAX_HEIGHT_M)
    keep = cloud_self_mask(np.array([ABOVE_BODY, at_limit]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.tolist() == [True, False]  # z <= max_height_m is the robot, above it a real overhang


def test_below_floor_points_outside_the_polygon_are_kept() -> None:
    under_body = (SIDE_FRAME[0], SIDE_FRAME[1], -0.20)
    keep = cloud_self_mask(np.array([CLIFF, under_body]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.tolist() == [True, False]


def test_non_finite_points_are_removed() -> None:
    points = np.array([[math.nan, 1.0, 0.0], [1.0, math.inf, 0.0], [1.0, 0.0, -math.inf], [1.0, 0.0, 0.0]])
    assert cloud_self_mask(points, prepared(deployed_polygon()), MAX_HEIGHT_M).tolist() == [False, False, False, True]


def test_mask_accepts_a_vertex_list_and_an_empty_cloud() -> None:
    assert cloud_self_mask(np.array([SIDE_FRAME, CLIFF]), deployed_polygon(), MAX_HEIGHT_M).tolist() == [False, True]
    empty = cloud_self_mask(np.zeros((0, 3)), deployed_polygon(), MAX_HEIGHT_M)
    assert empty.shape == (0,) and empty.dtype == bool


def test_replaced_runtime_polygon_with_a_released_arm_shadow_removes_arm_points() -> None:
    """mcp_server sets the dynamic footprint (static outline + released arm shadow) through the `polygon` parameter."""
    flt = SelfFilter(deployed_polygon())
    points = np.array([ARM_POINT, FRONT_NOTCH, CLIFF])
    assert cloud_self_mask(points, flt.base_polygon, MAX_HEIGHT_M).tolist() == [True, True, True]
    dynamic = Polygon(deployed_polygon()).union(box(0.15, -0.06, 0.62, 0.06)).simplify(0.0)
    ok, outcome = apply_polygon_param(flt, polygon_string(dynamic))
    assert ok and outcome == "replaced"
    assert cloud_self_mask(points, flt.base_polygon, MAX_HEIGHT_M).tolist() == [False, False, True]
    ok, _ = apply_polygon_param(flt, polygon_string(Polygon(deployed_polygon())))
    assert ok  # the arm parked again: the static outline is restored and the notch is an obstacle again
    assert cloud_self_mask(points, flt.base_polygon, MAX_HEIGHT_M).tolist() == [True, True, True]


def test_transform_helper_round_trips_a_yawed_and_tilted_transform() -> None:
    rotation = quaternion_matrix(0.0, 0.0, *QUARTER) @ quaternion_matrix(QUARTER[0], 0.0, 0.0, QUARTER[1])
    translation = np.array([0.1, -0.2, 0.6])
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.3, -0.4, 2.0]])
    moved = transform_points(points, rotation, translation)
    # yaw 45 deg after roll 45 deg: x stays a pure yaw, y is rolled up first
    np.testing.assert_allclose(moved[0], [math.sqrt(0.5) + 0.1, math.sqrt(0.5) - 0.2, 0.6], atol=1e-12)
    np.testing.assert_allclose(moved[1], [-0.5 + 0.1, 0.5 - 0.2, math.sqrt(0.5) + 0.6], atol=1e-12)
    back = transform_points(moved, rotation.T, -rotation.T @ translation)
    np.testing.assert_allclose(back, points, atol=1e-12)
    assert transform_points(np.zeros((0, 3)), rotation, translation).shape == (0, 3)


def test_filter_cloud_keeps_obstacles_in_the_cloud_frame() -> None:
    """The node's per-cloud path: cloud frame points -> polygon_frame -> mask -> the kept points as given (float32)."""
    rotation, translation = head_transform()
    base = np.array([SIDE_FRAME, FRONT_NOTCH, PARKED_GRIPPER, CLIFF, ABOVE_BODY, REAR_NOTCH])
    cloud = transform_points(base, rotation.T, -rotation.T @ translation).astype(np.float32)
    cloud = np.vstack([cloud, [[math.nan, math.nan, math.nan]]]).astype(np.float32)
    kept = SelfFilter(deployed_polygon()).filter_cloud(cloud, rotation, translation, MAX_HEIGHT_M)
    assert kept.dtype == np.float32
    np.testing.assert_array_equal(kept, cloud[[1, 3, 4, 5]])  # same frame: points are not moved


def test_filter_cloud_passes_an_empty_cloud_through_as_empty() -> None:
    rotation, translation = head_transform()
    kept = SelfFilter(deployed_polygon()).filter_cloud(np.zeros((0, 3), np.float32), rotation, translation, 1.0)
    assert kept.shape == (0, 3) and kept.dtype == np.float32


def test_pending_slot_is_newest_wins_and_counts_drops() -> None:
    slot: PendingSlot[str] = PendingSlot(timeout_s=0.2)
    assert slot.item is None and slot.dropped == 0
    assert slot.put("a", now_s=10.0) is None
    assert slot.put("b", now_s=10.05) == "a"  # superseded by the newer cloud: dropped
    assert slot.dropped == 1 and slot.item == "b"
    assert slot.expire(now_s=10.2) is None  # 0.15 s old: still waiting for TF
    assert slot.take() == "b" and slot.item is None and slot.take() is None
    assert slot.dropped == 1  # taken (published), not dropped


def test_pending_slot_drops_a_cloud_after_the_timeout() -> None:
    slot: PendingSlot[str] = PendingSlot(timeout_s=0.2)
    slot.put("a", now_s=10.0)
    assert slot.expire(now_s=10.21) == "a"
    assert slot.item is None and slot.dropped == 1
    assert slot.expire(now_s=11.0) is None and slot.dropped == 1


def test_cloud_config_defaults() -> None:
    cfg = LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]")
    assert cfg.cloud_topic == "/tof/points"
    assert cfg.cloud_filtered_topic == "/tof/points_filtered"
    assert cfg.cloud_self_max_height_m == 1.0
    assert cfg.cloud_transform_timeout_s == 0.2
    assert LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]", cloud_topic=None).cloud_topic is None


@pytest.mark.parametrize(
    "extra",
    [
        {"cloud_self_max_height_m": 0.0},
        {"cloud_self_max_height_m": -1.0},
        {"cloud_transform_timeout_s": 0.0},
        {"cloud_frame": "base_link"},
        {"cloud_polygon": "[[0, 0], [1, 0], [1, 1]]"},
    ],
)
def test_cloud_config_rejects_bad_values_and_unknown_keys(extra: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]", **extra)


def test_deployed_config_sets_the_cloud_keys_outside_the_generated_region() -> None:
    text = DEPLOYED.read_text()
    outside = text.split(GENERATED_BEGIN)[0] + text.split(GENERATED_END)[1]
    for line in (
        "cloud_topic: /tof/points",
        "cloud_filtered_topic: /tof/points_filtered",
        "cloud_self_max_height_m: 1.0",
        "cloud_transform_timeout_s: 0.2",
    ):
        assert f"\n{line}\n" in outside, line
    cfg = load_config(DEPLOYED)
    assert (cfg.cloud_topic, cfg.cloud_filtered_topic) == ("/tof/points", "/tof/points_filtered")


def test_pending_slot_drop_counts_the_waiting_cloud() -> None:
    """A newer cloud got its TF first (or the TF lookup failed): the waiting one is dropped and counted."""
    slot: PendingSlot[str] = PendingSlot(timeout_s=0.2)
    assert slot.drop() is None and slot.dropped == 0
    slot.put("a", now_s=10.0)
    assert slot.drop() == "a"
    assert slot.item is None and slot.dropped == 1
