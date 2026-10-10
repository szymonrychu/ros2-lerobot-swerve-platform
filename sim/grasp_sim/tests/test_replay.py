"""Plan replay: trivial holds, contacts, clearance, forces and the report contract."""

import numpy as np
import pytest

from grasp_sim.config import BoxObjectConfig, SceneConfig, SimConfig
from grasp_sim.plan import JOINT_ORDER
from grasp_sim.replay import classify_contact, object_tilt_deg, simulate

HOME = dict.fromkeys(JOINT_ORDER, 0.0)


def hold_plan(label: str | None = "open", seconds: float = 0.6, **joints: float) -> list[dict]:
    pose = {**HOME, **joints}
    first: dict = {"t": 0.0, "joints": pose}
    if label:
        first["label"] = label
    return [first, {"t": seconds, "joints": pose}]


def test_trivial_hold_passes_with_clearance_and_gravity_hold_force() -> None:
    scene = SceneConfig(object=BoxObjectConfig(x_m=0.2))
    report = simulate(hold_plan(), scene)
    assert report.passed, report.reasons
    assert report.reasons == []
    assert report.first_unintended_contact is None
    assert [s.label for s in report.segments] == ["open"]
    seg = report.segments[0]
    assert seg.min_clearance.jaws_floor is not None and seg.min_clearance.jaws_floor > 0.2
    assert seg.min_clearance.jaws_support is None
    assert 0.0 < max(report.max_actuator_force_nm.values()) < 3.0
    assert report.object is not None and not report.object.tipped and not report.object.pushed
    assert report.grasp_success is False


def test_unlabelled_plan_reports_null_label() -> None:
    report = simulate(hold_plan(label=None), SceneConfig())
    assert [s.label for s in report.segments] == [None]


def test_scene_without_object_has_no_object_report() -> None:
    report = simulate(hold_plan(), SceneConfig(object=None))
    assert report.object is None
    assert report.grasp_success is None
    assert report.passed


def test_clearance_tracks_support_height() -> None:
    low = simulate(hold_plan(), SceneConfig(support_z_m=-0.05)).segments[0].min_clearance
    high = simulate(hold_plan(), SceneConfig(support_z_m=0.05)).segments[0].min_clearance
    assert low.jaws_support is not None and high.jaws_support is not None
    assert low.jaws_support - high.jaws_support == pytest.approx(0.10, abs=0.01)


def test_arm_links_in_the_floor_are_an_unintended_contact() -> None:
    # Mount the arm only 5 cm above the floor and hold a pose that drives the arm links into it.
    plan = hold_plan("approach", shoulder_lift=1.9, elbow_flex=-1.0, wrist_flex=-1.5)
    report = simulate(plan, SceneConfig(base_height_m=0.05, object=None))
    assert not report.passed
    assert report.first_unintended_contact is not None
    assert report.first_unintended_contact.kind == "arm_floor"
    assert report.first_unintended_contact.label == "approach"
    assert any("arm_floor" in r for r in report.reasons)


def test_jaw_floor_contact_is_allowed_by_default_and_forbidden_on_request() -> None:
    plan = [
        {"t": 0.0, "label": "approach", "joints": {**HOME, "shoulder_lift": 0.2}},
        {"t": 2.5, "label": "approach", "joints": {**HOME, "shoulder_lift": 1.5, "elbow_flex": 0.0, "wrist_flex": 0.0}},
    ]
    scene = SceneConfig(object=None)
    allowed = simulate(plan, scene)
    strict = simulate(plan, scene, SimConfig(allow_jaw_surface_contact=False))
    assert allowed.event_counts.get("jaw_floor", 0) > 0
    assert allowed.passed, allowed.reasons
    assert not strict.passed
    assert strict.first_unintended_contact is not None and strict.first_unintended_contact.kind == "jaw_floor"
    clearance = allowed.segments[0].min_clearance.jaws_floor
    assert clearance is not None and clearance <= 0.0


def test_report_is_json_serialisable_and_deterministic() -> None:
    first = simulate(hold_plan(), SceneConfig())
    second = simulate(hold_plan(), SceneConfig())
    assert first.model_dump_json() == second.model_dump_json()


def test_plan_may_be_json_text() -> None:
    import json

    report = simulate(json.dumps(hold_plan()), SceneConfig())
    assert report.passed


def test_commands_beyond_joint_limits_are_clipped_with_a_warning() -> None:
    plan = hold_plan(elbow_flex=3.0)
    report = simulate(plan, SceneConfig(object=None))
    assert any("elbow_flex" in w for w in report.warnings)
    assert np.isfinite(max(report.max_actuator_force_nm.values()))


@pytest.mark.parametrize(
    ("a", "b", "kind"),
    [
        ("arm", "floor", "arm_floor"),
        ("wrist", "support", "arm_support"),
        ("jaw", "floor", "jaw_floor"),
        ("jaw", "support", "jaw_support"),
        ("jaw", "object", "jaw_object"),
        ("arm", "object", "arm_object"),
        ("object", "floor", None),
        ("jaw", "wrist", None),
        ("jaw", "jaw", None),
    ],
)
def test_classify_contact(a: str, b: str, kind: str | None) -> None:
    assert classify_contact(a, b) == kind
    assert classify_contact(b, a) == kind


def test_object_tilt_deg_from_rotation_matrix() -> None:
    assert object_tilt_deg(np.eye(3).ravel()) == pytest.approx(0.0)
    tipped = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    assert object_tilt_deg(tipped) == pytest.approx(90.0)
