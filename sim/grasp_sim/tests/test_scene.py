"""Programmatic scene generation: arm mount height, floor, support and the object."""

import mujoco
import numpy as np
import pytest
from pydantic import ValidationError

from grasp_sim.config import BoxObjectConfig, SceneConfig
from grasp_sim.scene import build_model, object_start_height


def settle(model: mujoco.MjModel, seconds: float = 0.5) -> mujoco.MjData:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return data


def floor_top(model: mujoco.MjModel) -> float:
    gid = model.geom("floor").id
    return float(model.geom_pos[gid][2] + model.geom_size[gid][2])


def test_default_floor_is_base_height_below_arm_origin() -> None:
    cfg = SceneConfig()
    assert cfg.base_height_m == pytest.approx(0.15)
    assert cfg.floor_z == pytest.approx(-0.15)
    model = build_model(cfg)
    assert floor_top(model) == pytest.approx(-0.15)


def test_base_height_is_configurable() -> None:
    model = build_model(SceneConfig(base_height_m=0.165))
    assert floor_top(model) == pytest.approx(-0.165)


def test_support_kind_is_inferred_from_support_z() -> None:
    assert SceneConfig().support_kind == "floor"
    assert SceneConfig(support_z_m=-0.15).support_kind == "floor"
    assert SceneConfig(support_z_m=-0.07).support_kind == "ledge"
    assert SceneConfig(support_z_m=-0.23).support_kind == "stair"


def test_box_on_floor_rests_on_floor_with_configured_mass_and_friction() -> None:
    cfg = SceneConfig(object=BoxObjectConfig(size_m=(0.04, 0.03, 0.05), mass_kg=0.2, friction=(0.7, 0.01, 0.001)))
    model = build_model(cfg)
    assert model.body("object").mass[0] == pytest.approx(0.2)
    assert model.geom("object_box").friction[0] == pytest.approx(0.7)
    assert np.allclose(model.geom("object_box").size, [0.02, 0.015, 0.025])
    data = settle(model)
    assert data.body("object").xpos[2] == pytest.approx(-0.15 + 0.025, abs=2e-3)
    assert "support" not in [model.geom(i).name for i in range(model.ngeom)]


def test_box_on_ledge_rests_at_support_z() -> None:
    cfg = SceneConfig(support_z_m=-0.05, object=BoxObjectConfig(size_m=(0.03, 0.03, 0.04)))
    model = build_model(cfg)
    assert object_start_height(cfg) == pytest.approx(-0.05 + 0.02, abs=1e-3)
    data = settle(model)
    assert data.body("object").xpos[2] == pytest.approx(-0.05 + 0.02, abs=2e-3)
    sid = model.geom("support").id
    assert model.geom_pos[sid][2] + model.geom_size[sid][2] == pytest.approx(-0.05)


def test_box_on_lower_stair_rests_below_the_floor_plane() -> None:
    cfg = SceneConfig(support_z_m=-0.25, object=BoxObjectConfig(size_m=(0.03, 0.03, 0.04)))
    model = build_model(cfg)
    data = settle(model)
    assert data.body("object").xpos[2] == pytest.approx(-0.25 + 0.02, abs=2e-3)
    assert data.body("object").xpos[2] < floor_top(model)


def test_scene_without_object_has_no_object_body() -> None:
    model = build_model(SceneConfig(object=None))
    with pytest.raises(KeyError):
        model.body("object")


def test_ledge_below_floor_is_a_stair_and_floor_slab_is_cut_before_the_object() -> None:
    cfg = SceneConfig(support_z_m=-0.25, object=BoxObjectConfig(x_m=0.2), support_edge_x_m=0.12)
    model = build_model(cfg)
    gid = model.geom("floor").id
    assert model.geom_pos[gid][0] + model.geom_size[gid][0] == pytest.approx(0.12)


def test_invalid_sizes_are_rejected() -> None:
    with pytest.raises(ValidationError):
        BoxObjectConfig(size_m=(0.0, 0.03, 0.04))
    with pytest.raises(ValidationError):
        BoxObjectConfig(mass_kg=-1.0)


def test_gap_below_raises_the_object_on_two_rails_with_a_clear_slot() -> None:
    cfg = SceneConfig(support_z_m=-0.08, object=BoxObjectConfig(size_m=(0.04, 0.04, 0.04), x_m=0.2, gap_below_m=0.02))
    model = build_model(cfg)
    names = [model.geom(i).name for i in range(model.ngeom)]
    assert "support_rail_left" in names and "support_rail_right" in names
    assert object_start_height(cfg) == pytest.approx(-0.08 + 0.02 + 0.02, abs=1e-3)
    data = settle(model)
    assert data.body("object").xpos[2] == pytest.approx(-0.08 + 0.02 + 0.02, abs=2e-3)
    left, right = model.geom("support_rail_left"), model.geom("support_rail_right")
    slot = abs(float(data.geom_xpos[left.id][1] - data.geom_xpos[right.id][1])) - 2 * float(left.size[1])
    assert slot >= 0.04 - 2 * 0.004 - 1e-6


def test_rails_are_support_geoms_for_the_report() -> None:
    from grasp_sim.replay import Rig

    rig = Rig(SceneConfig(object=BoxObjectConfig(gap_below_m=0.015)))
    roles = {rig.model.geom(g).name: r for g, r in rig.role.items()}
    assert roles["support_rail_left"] == "support"
    assert roles["support_rail_right"] == "support"
