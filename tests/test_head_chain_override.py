"""head_chain.json override format and the one applier (design D1, D7)."""

import ast
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF_TEXT = REPO_ROOT / "shared" / "ros2_common" / "head_rgbd_calibration" / "urdf_text.py"
URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"


def load_module() -> ModuleType:
    """Load urdf_text.py by path, as the launch file does."""
    spec = importlib.util.spec_from_file_location("urdf_text_chain_under_test", URDF_TEXT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def ut() -> ModuleType:
    return load_module()


@pytest.fixture
def urdf_text() -> str:
    return URDF.read_text()


def valid_joint(xyz: object = (0.1, 0.2, 0.3), rpy: object = (0.01, -0.02, 0.03)) -> dict:
    return {"xyz": list(xyz), "rpy": list(rpy)}


def changed_lines(before: str, after: str) -> list[str]:
    return [a for b, a in zip(before.splitlines(), after.splitlines(), strict=False) if a != b]


def test_constants(ut: ModuleType) -> None:
    assert ut.HEAD_CHAIN_FILE == "head_chain.json"
    assert ut.TOF_JOINT_FILE == "tof_joint.json"
    assert ut.APPLIED_MARKER_FILE == "applied.json"
    assert ut.HEAD_CHAIN_JOINTS == ("head_mount_joint", "head_tilt", "head_camera_joint")


def test_writer_parser_round_trip(ut: ModuleType) -> None:
    origins = {
        "head_tilt": ut.Origin((0.0, 0.0, 0.0412), (0.0, 0.0123, 0.0)),
        "head_mount_joint": ut.Origin((0.15, 0.1, 0.25), (0.01, -0.02, 0.0)),
    }
    text = ut.head_chain_json(origins, "note", "s1", "ok")
    data = json.loads(text)
    assert list(data) == sorted(data)
    assert data["note"] == "note" and data["session"] == "s1" and data["result"] == "ok"
    assert list(data["joints"]) == sorted(data["joints"])
    assert "0.041200000" in text
    assert ut.parse_head_chain(data) == origins


@pytest.mark.parametrize("joint", ["head_mount_joint", "head_tilt", "head_camera_joint"])
def test_each_chain_joint_rewritten_only(ut: ModuleType, urdf_text: str, joint: str) -> None:
    out = ut.rewrite_joint_origin(urdf_text, joint, ut.Origin((1.0, 2.0, 3.0), (0.1, 0.2, 0.3)), "n1")
    assert f"<!-- {joint} calibrated: n1 -->" in out
    assert 'xyz="1.000000000 2.000000000 3.000000000" rpy="0.100000000 0.200000000 0.300000000"' in out
    removed = [line for line in urdf_text.splitlines() if line not in out.splitlines()]
    assert len(removed) == 1 and "<origin" in removed[0]
    again = ut.rewrite_joint_origin(out, joint, ut.Origin((1.0, 2.0, 3.0), (0.1, 0.2, 0.3)), "n2")
    assert again.count("calibrated") == 1 and "n2" in again


def test_head_tilt_keeps_axis_and_limit(ut: ModuleType, urdf_text: str) -> None:
    out = ut.rewrite_joint_origin(urdf_text, "head_tilt", ut.Origin((0, 0, 0.05), (0.0, 0.02, 0.0)), "n")
    assert '<axis xyz="0 1 0"/>' in out
    assert '<limit lower="-0.5236" upper="0.5236" effort="2.0" velocity="1.0"/>' in out


def test_rewrite_tof_joint_is_rewrite_joint_origin(ut: ModuleType, urdf_text: str) -> None:
    origin = ut.Origin((0.02, 0.03, 0.001), (0.0, 0.01, 0.0))
    assert ut.rewrite_tof_joint(urdf_text, origin, "n") == ut.rewrite_joint_origin(urdf_text, "tof_joint", origin, "n")
    assert "<!-- tof_joint calibrated: n -->" in ut.rewrite_tof_joint(urdf_text, origin, "n")


def test_rewrite_unknown_joint_raises(ut: ModuleType, urdf_text: str) -> None:
    with pytest.raises(ValueError, match="nope"):
        ut.rewrite_joint_origin(urdf_text, "nope", ut.Origin((0, 0, 0), (0, 0, 0)), "n")


def write_chain(tmp_path: Path, joints: dict) -> None:
    (tmp_path / "head_chain.json").write_text(json.dumps({"joints": joints, "note": "chain"}))


def test_apply_chain_then_tof(ut: ModuleType, urdf_text: str, tmp_path: Path) -> None:
    write_chain(tmp_path, {"head_tilt": valid_joint(), "head_camera_joint": valid_joint((0.5, 0.6, 0.7))})
    (tmp_path / "tof_joint.json").write_text(json.dumps({"xyz": [9, 8, 7], "rpy": [0, 0, 0], "note": "tof"}))
    out, warnings = ut.apply_overrides(urdf_text, tmp_path)
    assert warnings == []
    assert "<!-- head_tilt calibrated: chain -->" in out
    assert "<!-- head_camera_joint calibrated: chain -->" in out
    assert "<!-- tof_joint calibrated: tof -->" in out
    assert 'xyz="9.000000000 8.000000000 7.000000000"' in out
    assert 'xyz="0.500000000 0.600000000 0.700000000"' in out


def test_empty_dir_gives_identical_text(ut: ModuleType, urdf_text: str, tmp_path: Path) -> None:
    assert ut.apply_overrides(urdf_text, tmp_path) == (urdf_text, [])


BAD_CHAINS = {
    "bad_json": "{not json",
    "unknown_joint": json.dumps({"joints": {"base_link": valid_joint()}}),
    "two_vector": json.dumps({"joints": {"head_tilt": {"xyz": [0, 0], "rpy": [0, 0, 0]}}}),
    "nan": '{"joints": {"head_tilt": {"xyz": [NaN, 0, 0], "rpy": [0, 0, 0]}}}',
    "bool": json.dumps({"joints": {"head_tilt": {"xyz": [True, 0, 0], "rpy": [0, 0, 0]}}}),
    "empty_joints": json.dumps({"joints": {}}),
}


@pytest.mark.parametrize("case", sorted(BAD_CHAINS))
def test_invalid_chain_one_warning_and_unchanged(ut: ModuleType, urdf_text: str, tmp_path: Path, case: str) -> None:
    (tmp_path / "head_chain.json").write_text(BAD_CHAINS[case])
    out, warnings = ut.apply_overrides(urdf_text, tmp_path)
    assert out == urdf_text
    assert len(warnings) == 1 and "head_chain.json" in warnings[0]


def test_invalid_chain_does_not_block_tof(ut: ModuleType, urdf_text: str, tmp_path: Path) -> None:
    (tmp_path / "head_chain.json").write_text("{bad")
    (tmp_path / "tof_joint.json").write_text(json.dumps({"xyz": [1, 2, 3], "rpy": [0, 0, 0], "note": "t"}))
    out, warnings = ut.apply_overrides(urdf_text, tmp_path)
    assert len(warnings) == 1 and "tof_joint calibrated" in out


def test_invalid_tof_one_warning(ut: ModuleType, urdf_text: str, tmp_path: Path) -> None:
    (tmp_path / "tof_joint.json").write_text(json.dumps({"xyz": [1, 2], "rpy": [0, 0, 0], "note": "t"}))
    out, warnings = ut.apply_overrides(urdf_text, tmp_path)
    assert out == urdf_text and len(warnings) == 1 and "tof_joint.json" in warnings[0]


def test_parse_head_chain_error_names_key(ut: ModuleType) -> None:
    with pytest.raises(ValueError, match="head_tilt"):
        ut.parse_head_chain({"joints": {"head_tilt": {"xyz": [0, 0], "rpy": [0, 0, 0]}}})


def test_urdf_text_is_stdlib_only() -> None:
    tree = ast.parse(URDF_TEXT.read_text())
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert node.level == 0
            modules.add((node.module or "").split(".")[0])
    assert modules <= set(sys.stdlib_module_names)
