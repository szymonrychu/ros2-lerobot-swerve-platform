"""Tests for the arm to camera compute child (head_rgbd_calibration/arm_camera_compute.py), run in process."""

import json
from pathlib import Path

import yaml

from head_rgbd_calibration.arm_camera_compute import main
from head_rgbd_calibration.store import SessionStore
from tests.test_arm_camera_results import NOW, Scene


def test_child_writes_the_result_of_its_session_and_result_id(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    config_path = tmp_path / "node.yaml"
    config_path.write_text(yaml.safe_dump(scene.cfg.model_dump(mode="json")))
    rid = SessionStore.new_result_id(NOW)
    assert main(["--config", str(config_path), "--session", scene.sid, "--result", rid]) == 0
    result = json.loads((scene.store.result_dir(scene.sid, rid) / "result.json").read_text())
    assert (result["id"], result["session"], result["kind"], result["state"]) == (rid, scene.sid, "arm_camera", "done")


def test_child_writes_a_failed_result_without_a_head_chain(tmp_path: Path) -> None:
    scene = Scene(tmp_path, head_chain=False)
    config_path = tmp_path / "node.yaml"
    config_path.write_text(yaml.safe_dump(scene.cfg.model_dump(mode="json")))
    rid = SessionStore.new_result_id(NOW)
    assert main(["--config", str(config_path), "--session", scene.sid, "--result", rid]) == 0
    result = json.loads((scene.store.result_dir(scene.sid, rid) / "result.json").read_text())
    assert result["state"] == "failed" and result["error"]
