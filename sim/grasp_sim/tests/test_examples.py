"""The hand-built example scoop plans: passing floor/ledge/stair scoops and a failing (tipping) one."""

import json

import pytest

from grasp_sim.config import SceneConfig
from grasp_sim.examples import build_plan, example_case
from grasp_sim.plan import LABELS, parse_plan
from grasp_sim.replay import simulate


@pytest.fixture(scope="module", params=["floor", "ledge", "stair"])
def passing_case(request):
    scene, plan = example_case(request.param)
    return request.param, scene, plan, simulate(plan, scene)


def test_example_plans_are_valid_json_with_all_labels() -> None:
    _, plan = example_case("floor")
    parsed = parse_plan(json.loads(json.dumps(plan)))
    assert {s.label for s in parsed if s.label} == set(LABELS)
    assert [s.label for s in parsed if s.label][0] == "open"


def test_passing_scoops_pass_and_lift_the_object(passing_case) -> None:
    name, _, _, report = passing_case
    assert report.passed, (name, report.reasons)
    assert report.grasp_success is True
    assert report.object is not None
    assert report.object.lifted_after_lift is True
    assert report.object.lift_height_m > 0.05
    assert report.first_unintended_contact is None


def test_passing_scoops_skim_the_support_without_touching_it(passing_case) -> None:
    name, _, _, report = passing_case
    approach = next(s for s in report.segments if s.label == "approach")
    key = "jaws_floor" if name == "floor" else "jaws_support"
    clearance = getattr(approach.min_clearance, key)
    assert clearance is not None and 0.0 < clearance < 0.01


def test_stair_scoop_reaches_below_the_floor_plane() -> None:
    scene, plan = example_case("stair")
    report = simulate(plan, scene)
    assert report.object is not None
    assert report.object.start_pos[2] < scene.floor_z
    assert report.passed, report.reasons


def test_tipping_scoop_topples_the_object_and_fails() -> None:
    scene, plan = example_case("tip")
    report = simulate(plan, scene)
    assert not report.passed
    assert report.object is not None
    assert report.object.tipped
    assert report.object.pushed
    assert report.grasp_success is False
    assert report.first_unintended_contact is not None
    assert report.first_unintended_contact.kind == "jaw_object"
    assert report.first_unintended_contact.label in {"pre_grasp", "approach"}
    assert any("tipped" in r for r in report.reasons)


def test_a_sideways_offset_on_a_squat_box_is_still_an_unintended_contact() -> None:
    scene, _ = example_case("floor")
    report = simulate(build_plan(scene, "tip"), scene)
    assert not report.passed
    assert report.first_unintended_contact is not None


def test_closed_gripper_never_lifts_the_object() -> None:
    """Lift detection: the same plan with the gripper held open must not report a grasp."""
    scene, plan = example_case("floor")
    opened = [{**s, "joints": {**s["joints"], "gripper": 0.6}} for s in plan]
    report = simulate(opened, scene)
    assert report.grasp_success is False
    assert any("grasp failed" in r for r in report.reasons)


def test_build_plan_requires_an_object() -> None:
    with pytest.raises(ValueError, match="object"):
        build_plan(SceneConfig(object=None))
