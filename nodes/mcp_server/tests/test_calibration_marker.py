"""Arm calibration stale marker: loading, clearing items, and its effect on arm_home / get_arm_state."""

import json
import re
from pathlib import Path

import pytest

from mcp_server.arm import ArmController, ArmError
from mcp_server.calibration_marker import STALE_ITEMS, clear_stale, load_marker
from mcp_server.config import McpServerConfig
from mcp_server.home_store import save_home
from mcp_server.ik import ArmKinematics, load_joint_limits

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
SAVED_AT = "2026-10-20T10:00:00+00:00"
POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -0.2,
    "elbow_flex": 0.5,
    "wrist_flex": 0.7,
    "wrist_roll": -1.6,
    "gripper": 0.3,
}


def marker_doc(stale: list[str] | None = None) -> dict:
    """Marker JSON as the bridge writes it.

    Args:
        stale (list[str] | None): Stale items, both when None.

    Returns:
        dict: Marker document.
    """
    return {
        "version": 1,
        "group": "follower",
        "saved_at": SAVED_AT,
        "joints": {"wrist_roll": {"before_steps": 2048, "after_steps": 2100}},
        "stale": list(STALE_ITEMS) if stale is None else stale,
    }


def make(tmp_path: Path, marker: Path | None, write: dict | None = None) -> tuple[ArmController, FakeArmBackend]:
    """Controller with a stored home pose and the marker path configured.

    Args:
        tmp_path (Path): Test directory.
        marker (Path | None): Marker path for the config.
        write (dict | None): Marker content to write first.

    Returns:
        tuple[ArmController, FakeArmBackend]: Controller and backend.
    """
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    cfg.arm.calibration_stale_marker = marker
    save_home(cfg.arm.home_file, POSE)
    if marker is not None and write is not None:
        marker.write_text(json.dumps(write))
    be = FakeArmBackend(dict(POSE))
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def test_load_absent_is_none(tmp_path: Path) -> None:
    assert load_marker(tmp_path / "missing.json") is None


def test_load_valid(tmp_path: Path) -> None:
    path = tmp_path / "m.json"
    path.write_text(json.dumps(marker_doc()))
    marker = load_marker(path)
    assert marker is not None
    assert marker.group == "follower"
    assert marker.saved_at == SAVED_AT
    assert marker.stale == ["joint_offsets", "home"]
    assert marker.joints["wrist_roll"].after_steps == 2100


@pytest.mark.parametrize("content", ["{not json", json.dumps({"version": 1}), "[]"])
def test_load_invalid_reports_everything_stale(tmp_path: Path, content: str) -> None:
    path = tmp_path / "m.json"
    path.write_text(content)
    marker = load_marker(path)
    assert marker is not None
    assert set(marker.stale) == set(STALE_ITEMS)


def test_clear_one_item_keeps_the_other(tmp_path: Path) -> None:
    path = tmp_path / "m.json"
    path.write_text(json.dumps(marker_doc()))
    clear_stale(path, ["home"])
    marker = load_marker(path)
    assert marker is not None
    assert marker.stale == ["joint_offsets"]
    assert marker.saved_at == SAVED_AT
    assert not list(tmp_path.glob(".*tmp*")) and sorted(p.name for p in tmp_path.iterdir()) == ["m.json"]


def test_clear_last_item_deletes_the_file(tmp_path: Path) -> None:
    path = tmp_path / "m.json"
    path.write_text(json.dumps(marker_doc(["home"])))
    clear_stale(path, ["home"])
    assert not path.exists()


def test_clear_absent_file_is_a_noop(tmp_path: Path) -> None:
    clear_stale(tmp_path / "m.json", ["home"])


def test_clear_unreadable_marker_deletes_it_when_everything_is_cleared(tmp_path: Path) -> None:
    path = tmp_path / "m.json"
    path.write_text("{bad")
    clear_stale(path, list(STALE_ITEMS))
    assert not path.exists()


def test_arm_home_refused_while_home_stale_and_allowed_after_set_home(tmp_path: Path) -> None:
    path = tmp_path / "marker.json"
    arm, be = make(tmp_path, path, marker_doc())
    sent = len(be.published) if hasattr(be, "published") else None
    with pytest.raises(
        ArmError,
        match=re.escape(
            f"arm home was recorded before the arm re-zero at {SAVED_AT}; record a new home with arm_set_home"
        ),
    ):
        arm.home()
    if sent is not None:
        assert len(be.published) == sent
    arm.set_home()
    marker = load_marker(path)
    assert marker is not None
    assert marker.stale == ["joint_offsets"]
    arm.home()  # no longer refused (already at the new home)


def test_arm_home_allowed_when_only_offsets_stale(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, tmp_path / "marker.json", marker_doc(["joint_offsets"]))
    arm.home()


def test_state_reports_stale_items(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, tmp_path / "marker.json", marker_doc())
    state = arm.state()
    assert state.calibration_stale == ["joint_offsets", "home"]
    assert state.calibration_stale_since == SAVED_AT


def test_state_without_marker(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, tmp_path / "marker.json")
    state = arm.state()
    assert state.calibration_stale == []
    assert state.calibration_stale_since is None


def test_null_path_disables(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, None)
    state = arm.state()
    assert state.calibration_stale == []
    arm.set_home()
    arm.home()
