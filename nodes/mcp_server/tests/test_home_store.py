"""Tests for mcp_server.home_store (arm home pose YAML)."""

from pathlib import Path

import pytest
import yaml

from mcp_server.home_store import HomeStoreError, load_home, save_home


def test_missing_file_returns_none(tmp_path: Path) -> None:
    assert load_home(tmp_path / "nope.yaml") is None


def test_round_trip_creates_parent_dirs(tmp_path: Path) -> None:
    path = tmp_path / "arm" / "home.yaml"
    save_home(path, {"shoulder_pan": 0.1, "gripper": 0.5})
    assert load_home(path) == {"shoulder_pan": pytest.approx(0.1), "gripper": pytest.approx(0.5)}
    doc = yaml.safe_load(path.read_text())
    assert doc["joints"] == {"shoulder_pan": 0.1, "gripper": 0.5}


def test_save_is_atomic_no_tmp_left(tmp_path: Path) -> None:
    path = tmp_path / "home.yaml"
    save_home(path, {"a": 1.0})
    save_home(path, {"a": 2.0})
    assert load_home(path) == {"a": 2.0}
    assert [p.name for p in tmp_path.iterdir()] == ["home.yaml"]


def test_corrupt_file_raises(tmp_path: Path) -> None:
    path = tmp_path / "home.yaml"
    path.write_text("joints: [1, 2]\n")
    with pytest.raises(HomeStoreError):
        load_home(path)


def test_non_finite_values_rejected(tmp_path: Path) -> None:
    with pytest.raises(HomeStoreError):
        save_home(tmp_path / "home.yaml", {"a": float("nan")})
