"""Stall telemetry (laser_filter.metrics, design D55): scan receive gap and lag of the /scan input and the /scan_filtered
output, the executor tick gap, their sliding-window maxima (including the open gap of a stream that stopped), the
metrics_port config key, and LaserFilterNode recording them against the ROS doubles of test_cloud_filter."""

import math
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from prometheus_client import CollectorRegistry
from pydantic import ValidationError

from laser_filter.config import LaserFilterConfig, load_config
from laser_filter.metrics import GapLagWindow, LaserFilterMetrics, create_families
from tests.test_cloud_filter import (
    CLOUD_STAMP,
    DEPLOYED,
    LASER_FRAME,
    node_module,  # noqa: F401 (pytest fixture)
    stamp_msg,
    transform_msg,
)

POLYGON = "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
DEPLOYED_METRICS_PORT = 19120
SCAN_PERIOD_S = 0.14  # rplidar A1 at ~7.2 Hz
STALL_S = 1.9  # the 2026-10-22 16:30:35 stall of both laser_filter outputs
STAMP_S = CLOUD_STAMP[0] + CLOUD_STAMP[1] * 1e-9


def sample(registry: CollectorRegistry, name: str, **labels: str) -> float | None:
    """One sample value of the registry (None when absent)."""
    return registry.get_sample_value(name, labels)


def window(registry: CollectorRegistry, stream: str = "input") -> GapLagWindow:
    families = create_families(registry)
    return GapLagWindow(
        families.scan_gap.labels(stream),
        families.scan_gap_max.labels(stream),
        families.scan_lag.labels(stream),
        families.scan_lag_max.labels(stream),
    )


def test_first_record_observes_lag_only_then_gaps_follow() -> None:
    registry = CollectorRegistry()
    win = window(registry)
    win.record(10.0, 0.02)
    assert sample(registry, "laser_filter_scan_gap_seconds_count", stream="input") == 0
    assert sample(registry, "laser_filter_scan_lag_seconds_count", stream="input") == 1
    assert sample(registry, "laser_filter_scan_lag_seconds_max", stream="input") == pytest.approx(0.02)
    win.record(10.0 + SCAN_PERIOD_S, 0.03)
    assert sample(registry, "laser_filter_scan_gap_seconds_count", stream="input") == 1
    assert sample(registry, "laser_filter_scan_gap_seconds_sum", stream="input") == pytest.approx(SCAN_PERIOD_S)
    assert sample(registry, "laser_filter_scan_gap_seconds_max", stream="input") == pytest.approx(SCAN_PERIOD_S)
    assert sample(registry, "laser_filter_scan_lag_seconds_max", stream="input") == pytest.approx(0.03)


def test_a_stall_lands_above_the_source_timeout_bucket_and_in_the_window_max() -> None:
    registry = CollectorRegistry()
    win = window(registry)
    for t in (0.0, SCAN_PERIOD_S, SCAN_PERIOD_S + STALL_S):
        win.record(t, 0.02)
    win.record(SCAN_PERIOD_S + STALL_S + 0.01, 1.5)  # a backlogged scan delivered late
    assert sample(registry, "laser_filter_scan_gap_seconds_bucket", stream="input", le="1.0") == 2
    assert sample(registry, "laser_filter_scan_gap_seconds_count", stream="input") == 3
    assert sample(registry, "laser_filter_scan_gap_seconds_max", stream="input") == pytest.approx(STALL_S)
    assert sample(registry, "laser_filter_scan_lag_seconds_max", stream="input") == pytest.approx(1.5)


def test_refresh_reports_the_open_gap_of_a_stream_that_stopped() -> None:
    registry = CollectorRegistry()
    win = window(registry)
    win.record(0.0, 0.02)
    win.record(SCAN_PERIOD_S, 0.02)
    win.refresh(SCAN_PERIOD_S + 1.2)
    assert sample(registry, "laser_filter_scan_gap_seconds_max", stream="input") == pytest.approx(1.2)
    assert sample(registry, "laser_filter_scan_gap_seconds_count", stream="input") == 1  # not observed until it ends


def test_window_max_forgets_a_stall_older_than_the_window() -> None:
    registry = CollectorRegistry()
    win = window(registry)
    win.record(0.0, 2.0)
    t = STALL_S
    win.record(t, 0.02)
    while t < 70.0:
        t += SCAN_PERIOD_S
        win.record(t, 0.02)
    win.refresh(t)
    assert sample(registry, "laser_filter_scan_gap_seconds_max", stream="input") == pytest.approx(SCAN_PERIOD_S)
    assert sample(registry, "laser_filter_scan_lag_seconds_max", stream="input") == pytest.approx(0.02)


def test_executor_window_tracks_gaps_without_lag() -> None:
    registry = CollectorRegistry()
    metrics = LaserFilterMetrics(create_families(registry))
    metrics.executor.record(5.0)
    metrics.executor.record(5.05)
    metrics.executor.record(5.05 + STALL_S)
    assert sample(registry, "laser_filter_executor_tick_gap_seconds_count") == 2
    assert sample(registry, "laser_filter_executor_tick_gap_seconds_max") == pytest.approx(STALL_S)


def test_metrics_port_config_key() -> None:
    assert LaserFilterConfig(polygon=POLYGON).metrics_port is None
    assert LaserFilterConfig(polygon=POLYGON, metrics_port=19999).metrics_port == 19999
    for bad in (0, 65536):
        with pytest.raises(ValidationError):
            LaserFilterConfig(polygon=POLYGON, metrics_port=bad)
    assert load_config(DEPLOYED).metrics_port == DEPLOYED_METRICS_PORT


def scan_msg() -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=LASER_FRAME, stamp=stamp_msg(CLOUD_STAMP)),
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 2,
        time_increment=0.0,
        scan_time=0.1,
        range_min=0.05,
        range_max=12.0,
        ranges=[0.05, 3.0, 3.0, 3.0],
        intensities=[],
    )


def metered_node(
    node_module: ModuleType,  # noqa: F811
    monkeypatch: pytest.MonkeyPatch,
) -> tuple[object, CollectorRegistry, list[float], list[float]]:
    """LaserFilterNode with its own registry and controllable monotonic and wall clocks."""
    registry = CollectorRegistry()
    node = node_module.LaserFilterNode(load_config(DEPLOYED), LaserFilterMetrics(create_families(registry)))
    mono, wall = [100.0], [STAMP_S]
    monkeypatch.setattr(node_module.time, "monotonic", lambda: mono[0])
    monkeypatch.setattr(node_module.time, "time", lambda: wall[0])
    return node, registry, mono, wall


def test_node_records_input_and_output_gap_and_lag_per_scan(
    node_module: ModuleType,  # noqa: F811
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node, registry, mono, wall = metered_node(node_module, monkeypatch)
    node.tf_buffer.static[(LASER_FRAME, "base_link")] = transform_msg(np.eye(3), np.zeros(3))
    wall[0] = STAMP_S + 0.05
    node.callbacks["/scan"](scan_msg())
    mono[0] += STALL_S
    wall[0] = STAMP_S + 0.05 + STALL_S
    node.callbacks["/scan"](scan_msg())
    assert len(node.published["/scan_filtered"]) == 2
    for stream in ("input", "output"):
        assert sample(registry, "laser_filter_scan_gap_seconds_max", stream=stream) == pytest.approx(STALL_S)
        assert sample(registry, "laser_filter_scan_lag_seconds_max", stream=stream) == pytest.approx(0.05 + STALL_S)
        assert sample(registry, "laser_filter_scan_lag_seconds_count", stream=stream) == 2


def test_node_records_a_dropped_scan_as_input_only(
    node_module: ModuleType,  # noqa: F811
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node, registry, _, _ = metered_node(node_module, monkeypatch)
    node.callbacks["/scan"](scan_msg())  # no TF polygon_frame -> laser yet: dropped
    assert node.published["/scan_filtered"] == []
    assert sample(registry, "laser_filter_scan_lag_seconds_count", stream="input") == 1
    assert sample(registry, "laser_filter_scan_lag_seconds_count", stream="output") == 0


def test_node_tick_records_executor_gap_and_the_open_scan_gap(
    node_module: ModuleType,  # noqa: F811
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node, registry, mono, _ = metered_node(node_module, monkeypatch)
    node.tf_buffer.static[(LASER_FRAME, "base_link")] = transform_msg(np.eye(3), np.zeros(3))
    node.callbacks["/scan"](scan_msg())
    (tick,) = node.timers
    tick()
    mono[0] += 1.2  # the executor ran its tick 1.2 s later and no scan came in between
    tick()
    assert sample(registry, "laser_filter_executor_tick_gap_seconds_max") == pytest.approx(1.2)
    for stream in ("input", "output"):
        assert sample(registry, "laser_filter_scan_gap_seconds_max", stream=stream) == pytest.approx(1.2)
