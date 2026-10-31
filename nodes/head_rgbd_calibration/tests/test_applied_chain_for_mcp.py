# ruff: noqa: F811
"""The head chain the Calibrate page computes and applies is what the mcp_server camera-to-arm tools consume.

Page flow: Head from arm -> compute -> Apply (POST .../results/{rid}/apply) writes applied/head_chain.json and
applied/applied.json. mcp_server (`mcp_server/head.py`, `AppliedHeadKinematics`) rebuilds `HeadKinematics` from the repo
head URDF with `apply_overrides` on that directory, so these tests load the MCP head kinematics module and place the
camera through the applied files only, against the truth chain of the synthetic session.
"""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE, HEAD_CHAIN_FILE, apply_overrides

from head_rgbd_calibration import apply as applied_writer
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.head_arm_results import apply_head_arm, compute_head_arm
from head_rgbd_calibration.store import SessionStore
from tests.test_head_arm_results import (  # noqa: F401  (fixtures)
    NOW,
    TRUTH_DELTAS,
    cfg,
    fake_run,
    ready,
    state,
    store,
    synthetic,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
MCP_HEAD_KINEMATICS = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "head_kinematics.py"
CAMERA_FRAME = "d435_color_optical_frame"
POSES = [(0.0, 0.5236), (0.25, 0.5236), (-0.25, 0.45), (0.1, 0.62), (-0.1, 0.40)]
PROBE_POINTS = np.array([[0.0, 0.0, 0.35], [0.06, -0.04, 0.45], [-0.08, 0.05, 0.30]])
ARM_FRAME_TARGET_M = 0.005  # user target 2026-10-29: camera to arm below 5 mm


def mcp_head_kinematics() -> ModuleType:
    """mcp_server/head_kinematics.py loaded by path (the mcp_server package needs the arm stack, this module not)."""
    spec = importlib.util.spec_from_file_location("mcp_head_kinematics_under_test", MCP_HEAD_KINEMATICS)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def truth_chain_of_the_session(cfg: NodeConfig) -> UrdfChain:
    head_urdf = Path(cfg.head_urdf_file).read_text()
    mount = synthetic.nominal_board_mount(cfg.fit)
    truth = synthetic.synth_truth(head_urdf, TRUTH_DELTAS, None, cfg.fit, mount, (0.0, 0.0))
    return synthetic.truth_chain(UrdfChain.from_text(head_urdf), truth.origins)


def test_applied_files_place_the_camera_for_the_mcp_kinematics_within_the_target(
    cfg: NodeConfig, state: Path, store: SessionStore, ready: str
) -> None:
    result = compute_head_arm(cfg, store, ready, NOW)
    assert result["passed"] is True, (result["failed_limits"], result["error"])
    apply_head_arm(cfg, store, ready, result["id"], NOW, fake_run)
    applied = state / "applied"
    assert (applied / HEAD_CHAIN_FILE).is_file() and (applied / APPLIED_MARKER_FILE).is_file()

    text, warnings = apply_overrides(Path(cfg.head_urdf_file).read_text(), applied)
    assert warnings == []
    kinematics = mcp_head_kinematics().HeadKinematics.from_text(text, "applied")
    truth = truth_chain_of_the_session(cfg)

    worst = 0.0
    for pan, tilt in POSES:
        t_mcp = kinematics.link_pose(pan, tilt, CAMERA_FRAME)
        t_truth = truth.transform(CAMERA_FRAME, {"head_pan": pan, "head_tilt": tilt})
        for point in PROBE_POINTS:
            homogeneous = np.append(point, 1.0)
            worst = max(worst, float(np.linalg.norm((t_mcp @ homogeneous)[:3] - (t_truth @ homogeneous)[:3])))
    assert worst < ARM_FRAME_TARGET_M, f"worst camera-point error {worst * 1000:.2f} mm"


def test_the_chain_file_is_in_place_before_the_marker_changes(
    cfg: NodeConfig, state: Path, store: SessionStore, ready: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """mcp_server reloads when applied.json changes, so the new head_chain.json must already be there at that moment."""
    result = compute_head_arm(cfg, store, ready, NOW)
    marker = state / "applied" / APPLIED_MARKER_FILE
    marker.write_text(json.dumps({"session": "calib-1", "result": "r-1", "files": []}))
    before = marker.stat().st_mtime_ns
    chain_when_marker_written: list[str] = []
    real_write = applied_writer.write_json_atomic

    def spy(path: Path, data: object) -> None:
        if path.name == APPLIED_MARKER_FILE:
            chain_when_marker_written.append(json.loads((path.parent / HEAD_CHAIN_FILE).read_text())["result"])
        real_write(path, data)

    monkeypatch.setattr(applied_writer, "write_json_atomic", spy)
    apply_head_arm(cfg, store, ready, result["id"], NOW, fake_run)
    assert chain_when_marker_written == [result["id"]]
    assert marker.stat().st_mtime_ns != before
    assert json.loads(marker.read_text())["head_chain"]["result"] == result["id"]


def test_apply_without_an_accepted_check_tells_the_operator_which_button_to_press(
    cfg: NodeConfig, state: Path, store: SessionStore, ready: str
) -> None:
    """The head chain can only be applied after the RGB-D section's Accept check created applied.json."""
    result = compute_head_arm(cfg, store, ready, NOW)
    (state / "applied" / APPLIED_MARKER_FILE).unlink()
    with pytest.raises(NotApplicable, match=r"Accept check.*RGB-D section"):
        apply_head_arm(cfg, store, ready, result["id"], NOW, fake_run)
    assert not (state / "applied" / HEAD_CHAIN_FILE).exists()


def test_mcp_server_reads_the_applied_directory_the_node_writes() -> None:
    """The node's state_dir/applied and mcp_server's head.applied_dir default are the same directory."""
    import yaml

    nodes = yaml.safe_load((REPO_ROOT / "ansible" / "group_vars" / "client.yml").read_text())["ros2_nodes"]
    node = yaml.safe_load(next(n for n in nodes if n["name"] == "head_rgbd_calibration")["config"])
    source = (REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "config.py").read_text()
    assert f'Path("{node["state_dir"]}/applied")' in source
    web_ui = yaml.safe_load(next(n for n in nodes if n["name"] == "web_ui")["config"])
    assert web_ui["head_applied_calibration_dir"] == f"{node['state_dir']}/applied"


def test_web_ui_waits_long_enough_for_a_synchronous_head_arm_compute() -> None:
    """POST /api/head-arm/sessions/{sid}/compute fits in the request (about 15 s on a laptop for 24 poses, minutes on a
    Pi 5); the web_ui proxy default of 10 s answered 'calibration node unreachable' while the fit still ran."""
    import yaml

    nodes = yaml.safe_load((REPO_ROOT / "ansible" / "group_vars" / "client.yml").read_text())["ros2_nodes"]
    web_ui = yaml.safe_load(next(n for n in nodes if n["name"] == "web_ui")["config"])
    node = yaml.safe_load(next(n for n in nodes if n["name"] == "head_rgbd_calibration")["config"])
    assert web_ui["calibration"]["rgbd_timeout_s"] >= node["compute_timeout_s"]
