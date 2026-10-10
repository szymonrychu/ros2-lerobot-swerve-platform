"""Unit tests for UVC camera bridge config (env-based)."""

import pytest

from config import (
    DEFAULT_DEVICE,
    DEFAULT_FRAME_ID,
    DEFAULT_ROTATE_DEG,
    DEFAULT_TOPIC,
    get_config,
    get_max_fps,
    get_rotate_deg,
)


def test_get_config_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    """With no env set, get_config returns default device, topic, frame_id."""
    monkeypatch.delenv("UVC_DEVICE", raising=False)
    monkeypatch.delenv("UVC_TOPIC", raising=False)
    monkeypatch.delenv("UVC_FRAME_ID", raising=False)
    device, topic, frame_id = get_config()
    assert device == DEFAULT_DEVICE
    assert topic == DEFAULT_TOPIC
    assert frame_id == DEFAULT_FRAME_ID


def test_get_config_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """get_config reads UVC_DEVICE, UVC_TOPIC, UVC_FRAME_ID from environment."""
    monkeypatch.setenv("UVC_DEVICE", "1")
    monkeypatch.setenv("UVC_TOPIC", "/my/camera")
    monkeypatch.setenv("UVC_FRAME_ID", "my_frame")
    device, topic, frame_id = get_config()
    assert device == 1
    assert topic == "/my/camera"
    assert frame_id == "my_frame"


def test_get_config_device_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """UVC_DEVICE as path string is returned as str."""
    monkeypatch.setenv("UVC_DEVICE", "/dev/video2")
    device, topic, frame_id = get_config()
    assert device == "/dev/video2"
    assert topic == DEFAULT_TOPIC
    assert frame_id == DEFAULT_FRAME_ID


def test_get_config_strips_whitespace(monkeypatch: pytest.MonkeyPatch) -> None:
    """Topic and frame_id are stripped; empty env falls back to default."""
    monkeypatch.setenv("UVC_TOPIC", "  /topic  ")
    monkeypatch.setenv("UVC_FRAME_ID", "  frame  ")
    device, topic, frame_id = get_config()
    assert topic == "/topic"
    assert frame_id == "frame"
    monkeypatch.setenv("UVC_TOPIC", "")
    monkeypatch.setenv("UVC_FRAME_ID", "   ")
    _, topic2, frame_id2 = get_config()
    assert topic2 == DEFAULT_TOPIC
    assert frame_id2 == DEFAULT_FRAME_ID


def test_rotate_deg_defaults_to_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    """Without UVC_ROTATE_DEG no rotation is applied."""
    monkeypatch.delenv("UVC_ROTATE_DEG", raising=False)
    assert get_rotate_deg() == DEFAULT_ROTATE_DEG == 0


@pytest.mark.parametrize("value", [0, 90, 180, 270])
def test_rotate_deg_accepts_allowed_values(monkeypatch: pytest.MonkeyPatch, value: int) -> None:
    """0/90/180/270 are accepted (whitespace tolerated)."""
    monkeypatch.setenv("UVC_ROTATE_DEG", f" {value} ")
    assert get_rotate_deg() == value


@pytest.mark.parametrize("value", ["45", "-90", "360", "abc", "180.5"])
def test_rotate_deg_rejects_invalid_values(monkeypatch: pytest.MonkeyPatch, value: str) -> None:
    """Anything but 0/90/180/270 fails fast."""
    monkeypatch.setenv("UVC_ROTATE_DEG", value)
    with pytest.raises(ValueError, match="UVC_ROTATE_DEG"):
        get_rotate_deg()


def test_max_fps_unset_means_no_cap(monkeypatch: pytest.MonkeyPatch) -> None:
    """Without UVC_MAX_FPS every captured frame is published."""
    monkeypatch.delenv("UVC_MAX_FPS", raising=False)
    assert get_max_fps() is None


def test_max_fps_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """UVC_MAX_FPS is read as a float."""
    monkeypatch.setenv("UVC_MAX_FPS", " 10 ")
    assert get_max_fps() == 10.0


@pytest.mark.parametrize("value", ["0", "-5", "fast"])
def test_max_fps_rejects_invalid_values(monkeypatch: pytest.MonkeyPatch, value: str) -> None:
    """A non-positive or non-numeric UVC_MAX_FPS is a config error."""
    monkeypatch.setenv("UVC_MAX_FPS", value)
    with pytest.raises(ValueError, match="UVC_MAX_FPS"):
        get_max_fps()


def test_metrics_port_comes_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """The bridge resolves its /metrics port through ros2_metrics from METRICS_PORT; unset disables it."""
    from ros2_metrics import resolve_metrics_port

    monkeypatch.delenv("METRICS_PORT", raising=False)
    assert resolve_metrics_port(None) is None
    monkeypatch.setenv("METRICS_PORT", "19105")
    assert resolve_metrics_port(None) == 19105
