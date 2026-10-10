"""Unit tests for the camera Prometheus metrics (names, labels, buckets)."""

import pytest
from prometheus_client import REGISTRY

import metrics


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    """Return the current sample value, 0.0 when the series does not exist yet."""
    value = REGISTRY.get_sample_value(name, labels or {})
    return 0.0 if value is None else value


def test_frame_published_counts() -> None:
    before = sample("camera_frames_published_total")
    metrics.FRAMES_PUBLISHED.inc()
    assert sample("camera_frames_published_total") == before + 1


@pytest.mark.parametrize("reason", ["read_fail", "fps_cap"])
def test_dropped_frames_by_reason(reason: str) -> None:
    labels = {"reason": reason}
    before = sample("camera_frames_dropped_total", labels)
    metrics.FRAMES_DROPPED.labels(reason).inc()
    assert sample("camera_frames_dropped_total", labels) == before + 1


def test_encode_histogram_observes_with_jpeg_scale_buckets() -> None:
    before = sample("camera_encode_seconds_count")
    metrics.ENCODE_SECONDS.observe(0.004)
    assert sample("camera_encode_seconds_count") == before + 1
    assert sample("camera_encode_seconds_bucket", {"le": "0.005"}) >= 1
    assert sample("camera_encode_seconds_bucket", {"le": "0.001"}) == 0


def test_reopens_count() -> None:
    before = sample("camera_reopens_total")
    metrics.REOPENS.inc()
    assert sample("camera_reopens_total") == before + 1
