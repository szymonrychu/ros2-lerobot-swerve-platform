"""Unit tests for UVC camera bridge config (env-based)."""

import pytest

from config import DEFAULT_DEVICE, DEFAULT_FRAME_ID, DEFAULT_TOPIC, get_config


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
