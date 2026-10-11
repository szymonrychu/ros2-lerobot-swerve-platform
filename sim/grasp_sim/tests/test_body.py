"""Robot body volumes in the sim scene (grasp_sim.config BodyConfig, scene.add_body, replay body contacts)."""

from pathlib import Path

import mujoco
import pytest

from grasp_sim.config import BodyConfig, SceneConfig, load_client_body
from grasp_sim.matrix import MatrixEntry, MatrixIndex, failure_mode, scene_for, summary_table
from grasp_sim.replay import Rig, body_contacts, classify_contact
from grasp_sim.scene import BODY_GEOM_PREFIX

CLIENT_YML = Path(__file__).resolve().parents[3] / "ansible" / "group_vars" / "client.yml"
NAMES = ("frame_left", "frame_right", "crossbar", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr", "lidar")
# URDF (= MuJoCo) joint angles from nodes/mcp_server/tests/test_self_collision.py (zero offsets): the gripper pointing
# down into the lidar column, and a clear pose reaching forward.
LIDAR_POSE = {"shoulder_pan": -1.0718, "shoulder_lift": -0.5239, "elbow_flex": 0.5169, "wrist_flex": 1.5778}
CLEAR_POSE = {"shoulder_pan": 0.0, "shoulder_lift": -0.7948, "elbow_flex": 1.1787, "wrist_flex": 0.4161}


def body() -> BodyConfig:
    return load_client_body(CLIENT_YML)


def test_client_yml_body_is_loaded_with_the_mount() -> None:
    cfg = body()
    assert [b.name for b in cfg.boxes] + [c.name for c in cfg.cylinders] == list(NAMES)
    assert cfg.mount_xyz_m == pytest.approx((0.0592, -0.05, 0.100))
    lidar = cfg.cylinders[-1]
    assert (lidar.x, lidar.y, lidar.radius, lidar.z_min, lidar.z_max) == pytest.approx(
        (0.150, 0.040, 0.037, 0.10, 0.21)
    )


def test_body_geoms_are_in_the_scene_in_the_arm_frame() -> None:
    model = Rig(SceneConfig(object=None, body=body())).model
    for name in NAMES:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BODY_GEOM_PREFIX + name) >= 0, name
    lidar = model.geom(BODY_GEOM_PREFIX + "lidar")
    # base_link (0.150, 0.040, z 0.10..0.21) minus the mount (0.0592, -0.05, 0.100)
    assert lidar.pos == pytest.approx([0.150 - 0.0592, 0.040 + 0.05, (0.10 + 0.21) / 2 - 0.100])
    assert lidar.size[:2] == pytest.approx([0.037, (0.21 - 0.10) / 2])
    crossbar = model.geom(BODY_GEOM_PREFIX + "crossbar")
    assert crossbar.size == pytest.approx([0.125, 0.1158, 0.02])


def test_scene_without_body_has_no_body_geoms() -> None:
    model = Rig(SceneConfig(object=None)).model
    names = [model.geom(i).name for i in range(model.ngeom)]
    assert not any(n.startswith(BODY_GEOM_PREFIX) for n in names)


def posed(rig: Rig, joints: dict[str, float]) -> None:
    for joint, value in joints.items():
        rig.data.qpos[rig.qpos_adr[joint]] = value
    mujoco.mj_forward(rig.model, rig.data)


def test_a_pose_into_the_lidar_touches_the_body_and_a_clear_one_does_not() -> None:
    rig = Rig(SceneConfig(object=None, body=body()))
    posed(rig, LIDAR_POSE)
    hits = body_contacts(rig)
    assert any(b == BODY_GEOM_PREFIX + "lidar" for _, b in hits)
    posed(rig, CLEAR_POSE)
    assert body_contacts(rig) == []


def test_contacts_with_the_body_are_classified() -> None:
    assert classify_contact("jaw", "body") == "arm_body"
    assert classify_contact("arm", "body") == "arm_body"
    assert classify_contact("body", "floor") is None


def test_matrix_index_carries_the_body_into_the_scene() -> None:
    index = MatrixIndex(base_height_m=0.1, floor_z_m=-0.1, tool_offset_m=(0.0, 0.0, 0.0), entries=[], body=body())
    entry_fields = {
        "key": "k",
        "box": "4x4x4",
        "position": "r20",
        "support": "floor",
        "strategy": "top_down",
        "chosen": None,
        "approach_pitch_deg": None,
        "feasible": True,
        "reasons": [],
        "plan_file": "p.json",
        "x": 0.2,
        "y": 0.0,
        "surface_z": -0.1,
        "support_z": -0.1,
        "support_edge_x": None,
        "size_m": (0.04, 0.04, 0.04),
        "gap_below_m": 0.0,
    }
    entry = MatrixEntry.model_validate({k: v for k, v in entry_fields.items() if k in MatrixEntry.model_fields})
    assert scene_for(entry, index, stock_jaws=False).body == body()


def test_body_contact_rows_are_counted_and_fail_the_lift() -> None:
    row = {
        "key": "a",
        "box": "4x4x4",
        "strategy": "top_down",
        "feasible": True,
        "lifted": False,
        "arm_contact": False,
        "body_contacts": 3,
        "reasons": ["unintended arm_body contact"],
    }
    assert failure_mode(row) == "body contact"
    assert "body contacts: 3 in 1 plans" in summary_table([row])
