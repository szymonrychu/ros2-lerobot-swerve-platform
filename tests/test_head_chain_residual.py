"""The arm-frame residual that head_chain.json carries from the head-from-arm fit to the MCP camera-to-arm tools."""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF_TEXT = REPO_ROOT / "shared" / "ros2_common" / "head_rgbd_calibration" / "urdf_text.py"
URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"


@pytest.fixture
def ut() -> ModuleType:
    """urdf_text.py loaded by path, as the head_state_publisher launch file does."""
    spec = importlib.util.spec_from_file_location("urdf_text_residual_under_test", URDF_TEXT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_head_chain_json_carries_the_arm_frame_residual(ut: ModuleType, tmp_path: Path) -> None:
    origins = {"head_tilt": ut.Origin((0.0, 0.0, 0.0412), (0.0, 0.0123, 0.0))}
    residual = ut.ChainResidual(kind="head_arm", holdout_board_mm=2.9, holdout_rms_px=2.5, rms_px=2.7)
    text = ut.head_chain_json(origins, "note", "s1", "r1", residual)
    data = json.loads(text)
    assert list(data) == sorted(data) == ["joints", "note", "residual", "result", "session"]
    assert data["residual"] == {"holdout_board_mm": 2.9, "holdout_rms_px": 2.5, "kind": "head_arm", "rms_px": 2.7}
    assert ut.parse_head_chain(data) == origins  # the residual does not disturb the chain
    assert ut.parse_residual(data) == residual
    (tmp_path / "head_chain.json").write_text(text)
    urdf, warnings = ut.apply_overrides(URDF.read_text(), tmp_path)
    assert warnings == [] and "head_tilt calibrated: note" in urdf


def test_head_chain_json_without_residual_has_no_residual_key(ut: ModuleType) -> None:
    text = ut.head_chain_json({"head_tilt": ut.Origin((0.0, 0.0, 0.04), (0.0, 0.0, 0.0))}, "n", "s", "r")
    assert "residual" not in json.loads(text)
    assert ut.parse_residual(json.loads(text)) is None


@pytest.mark.parametrize(
    "bad",
    [
        None,
        "x",
        {},
        {"kind": "head_arm"},
        {"kind": "head_arm", "holdout_board_mm": "2", "holdout_rms_px": 1, "rms_px": 1},
        {"kind": "head_arm", "holdout_board_mm": float("nan"), "holdout_rms_px": 1, "rms_px": 1},
        {"kind": "head_arm", "holdout_board_mm": -1.0, "holdout_rms_px": 1, "rms_px": 1},
        {"kind": "", "holdout_board_mm": 1.0, "holdout_rms_px": 1, "rms_px": 1},
        {"kind": "head_arm", "holdout_board_mm": True, "holdout_rms_px": 1, "rms_px": 1},
    ],
)
def test_parse_residual_refuses_unusable_values(ut: ModuleType, bad: object) -> None:
    assert ut.parse_residual({"joints": {}, "residual": bad}) is None


def test_parse_residual_of_a_non_object_is_none(ut: ModuleType) -> None:
    assert ut.parse_residual([1, 2]) is None
