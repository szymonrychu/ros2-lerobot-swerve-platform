"""E1 grasp verification fusion foundations: mcp_server.grasp_check (band_verdict, fuse, models) and the gripper
current ingestion (JointSample.currents from /follower/joint_currents).

Covers: band_verdict rules (object / empty / ambiguous) with the load-delta vote, the hold_effort_min fallback on
|holding_load| for a profile without contact_load_delta, current voting (None = no vote), unmeasured (None or NaN)
loads never giving object, unknown in_empty_band; the full 3x3 fuse table; GripEvidence / CropInfo / GraspCheck
round-trips (a head camera jaw crop, or the full head frame with the reason the head aim or the crop failed);
ros_iface subscribes topics.follower_joint_currents and fills JointSample.currents only from a joint_currents message
received within timeouts.follower_stale_s of the joint sample (non-finite values dropped);
FakeArmBackend supplies currents.
"""

import math
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any, get_args
from unittest.mock import MagicMock

import pytest

from mcp_server import control
from mcp_server.arm import JointSample
from mcp_server.config import McpServerConfig
from mcp_server.grasp_check import (
    Band,
    CropInfo,
    FusedOutcome,
    GraspCheck,
    GripEvidence,
    GripMeasurements,
    ImageState,
    ImageVerdict,
    band_verdict,
    fuse,
    grip_evidence,
)
from mcp_server.motion_queue import LiveState, read_live_state

from .fakes import FakeArmBackend
from .test_control import manager
from .test_tools import FakeRobot

NAN = math.nan
DELTA = 20.0  # contact_load_delta of the profile
HOLD_MIN = 100.0  # grasp.hold_effort_min


def measured(**overrides: Any) -> GripMeasurements:
    """A verified-looking hold: contact by load delta, hold delta 40 over a baseline of 10, outside the empty band."""
    values: dict[str, Any] = {
        "grasp_id": "abc123def456",
        "baseline_load": 10.0,
        "baseline_source": "free_closing",
        "contact": "load_delta",
        "peak_load_delta": 60.0,
        "hold_load_delta": 40.0,
        "rest_current_a": None,
        "hold_current_a": None,
        "current_delta_a": None,
        "jaw_gap_rad": 0.3,
        "in_empty_band": False,
    }
    values.update(overrides)
    return GripMeasurements(**values)


def verdict(m: GripMeasurements, **kwargs: Any) -> Band:
    args: dict[str, Any] = {
        "contact_load_delta": DELTA,
        "current_contact_delta_a": None,
        "holding_load": None,
        "hold_effort_min": HOLD_MIN,
    }
    args.update(kwargs)
    return band_verdict(m, **args)


# --- literals ---------------------------------------------------------------------------------------------------


def test_literal_values_match_the_design() -> None:
    assert set(get_args(Band)) == {"object", "empty", "ambiguous"}
    assert set(get_args(ImageVerdict)) == {"yes", "no", "unsure"}
    assert set(get_args(ImageState)) == {"pending", "not_attached", "not_requested", "yes", "no", "unsure"}
    assert set(get_args(FusedOutcome)) == {"grasped", "unverified", "missed"}


# --- band_verdict: object ---------------------------------------------------------------------------------------


def test_object_when_contact_hold_delta_reaches_the_profile_delta_outside_the_empty_band() -> None:
    assert verdict(measured()) == "object"
    assert verdict(measured(hold_load_delta=DELTA)) == "object"  # >= is inclusive


@pytest.mark.parametrize("contact", ["load_delta", "absolute", "target_load", "stall"])
def test_object_for_every_contact_kind(contact: str) -> None:
    assert verdict(measured(contact=contact)) == "object"


def test_no_contact_is_not_object() -> None:
    assert verdict(measured(contact=None)) == "ambiguous"


def test_hold_delta_below_the_profile_delta_outside_the_empty_band_is_ambiguous() -> None:
    assert verdict(measured(hold_load_delta=DELTA - 0.1)) == "ambiguous"


def test_in_empty_band_is_never_object_even_with_a_high_load() -> None:
    assert verdict(measured(in_empty_band=True)) == "ambiguous"


def test_unknown_empty_band_is_ambiguous() -> None:
    assert verdict(measured(in_empty_band=None)) == "ambiguous"
    assert verdict(measured(in_empty_band=None, hold_load_delta=0.0, contact=None)) == "ambiguous"


# --- band_verdict: empty ----------------------------------------------------------------------------------------


def test_empty_when_in_the_empty_band_and_the_load_does_not_say_object() -> None:
    m = measured(in_empty_band=True, hold_load_delta=3.0, contact="stall", jaw_gap_rad=0.01)
    assert verdict(m) == "empty"
    assert verdict(m.model_copy(update={"contact": None})) == "empty"  # contact is not required for empty


def test_outside_the_empty_band_with_a_low_load_is_ambiguous_not_empty() -> None:
    assert verdict(measured(hold_load_delta=1.0)) == "ambiguous"


# --- band_verdict: unmeasured loads -----------------------------------------------------------------------------


@pytest.mark.parametrize("hold", [None, NAN, math.inf])
def test_unmeasured_hold_delta_is_never_object_nor_empty(hold: float | None) -> None:
    assert verdict(measured(hold_load_delta=hold)) == "ambiguous"
    assert verdict(measured(hold_load_delta=hold, in_empty_band=True)) == "ambiguous"


# --- band_verdict: hold_effort_min fallback (profile without contact_load_delta) --------------------------------


def test_profile_without_delta_uses_hold_effort_min_on_abs_holding_load() -> None:
    m = measured(hold_load_delta=None, baseline_load=None, baseline_source=None, contact="absolute")
    assert verdict(m, contact_load_delta=None, holding_load=150.0) == "object"
    assert verdict(m, contact_load_delta=None, holding_load=-150.0) == "object"  # |holding_load|
    assert verdict(m, contact_load_delta=None, holding_load=HOLD_MIN) == "object"
    assert verdict(m, contact_load_delta=None, holding_load=50.0) == "ambiguous"
    empty = m.model_copy(update={"in_empty_band": True})
    assert verdict(empty, contact_load_delta=None, holding_load=50.0) == "empty"
    assert verdict(empty, contact_load_delta=None, holding_load=150.0) == "ambiguous"


def test_profile_without_delta_ignores_the_hold_delta() -> None:
    """A large hold delta does not count when the profile has no contact_load_delta: the fallback decides."""
    assert verdict(measured(hold_load_delta=500.0), contact_load_delta=None, holding_load=20.0) == "ambiguous"


@pytest.mark.parametrize("holding", [None, NAN])
def test_fallback_with_unmeasured_holding_load_is_ambiguous(holding: float | None) -> None:
    m = measured(hold_load_delta=None)
    assert verdict(m, contact_load_delta=None, holding_load=holding) == "ambiguous"
    assert verdict(m.model_copy(update={"in_empty_band": True}), contact_load_delta=None, holding_load=holding) == (
        "ambiguous"
    )


# --- band_verdict: current vote ---------------------------------------------------------------------------------


def test_current_without_a_configured_delta_does_not_vote() -> None:
    assert verdict(measured(current_delta_a=0.0)) == "object"
    assert verdict(measured(in_empty_band=True, hold_load_delta=0.0, current_delta_a=5.0)) == "empty"


@pytest.mark.parametrize("current", [None, NAN])
def test_unmeasured_current_does_not_vote(current: float | None) -> None:
    assert verdict(measured(current_delta_a=current), current_contact_delta_a=0.1) == "object"
    m = measured(in_empty_band=True, hold_load_delta=0.0, current_delta_a=current)
    assert verdict(m, current_contact_delta_a=0.1) == "empty"


def test_current_vote_false_blocks_object() -> None:
    assert verdict(measured(current_delta_a=0.05), current_contact_delta_a=0.1) == "ambiguous"
    assert verdict(measured(current_delta_a=0.1), current_contact_delta_a=0.1) == "object"  # inclusive


def test_current_vote_true_blocks_empty() -> None:
    m = measured(in_empty_band=True, hold_load_delta=0.0)
    assert verdict(m.model_copy(update={"current_delta_a": 0.2}), current_contact_delta_a=0.1) == "ambiguous"
    assert verdict(m.model_copy(update={"current_delta_a": 0.01}), current_contact_delta_a=0.1) == "empty"


def test_band_verdict_accepts_a_full_grip_evidence() -> None:
    e = GripEvidence(**measured().model_dump(), band="ambiguous")
    assert verdict(e) == "object"  # the stored band is not an input


def test_grip_evidence_helper_sets_the_band() -> None:
    e = grip_evidence(
        measured(),
        contact_load_delta=DELTA,
        current_contact_delta_a=None,
        holding_load=None,
        hold_effort_min=HOLD_MIN,
    )
    assert isinstance(e, GripEvidence)
    assert e.band == "object"
    assert e.grasp_id == "abc123def456"
    assert e.hold_load_delta == 40.0


# --- fuse -------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("band", "image", "fused"),
    [
        ("object", "yes", "grasped"),
        ("object", "no", "unverified"),
        ("object", "unsure", "unverified"),
        ("ambiguous", "yes", "unverified"),
        ("ambiguous", "no", "unverified"),
        ("ambiguous", "unsure", "unverified"),
        ("empty", "yes", "unverified"),
        ("empty", "no", "missed"),
        ("empty", "unsure", "unverified"),
    ],
)
def test_fuse_table(band: Band, image: ImageVerdict, fused: FusedOutcome) -> None:
    assert fuse(band, image) == fused


def test_fuse_rejects_unknown_values() -> None:
    bad_band: Any = "maybe"
    bad_image: Any = "pending"
    with pytest.raises(ValueError):
        fuse(bad_band, "yes")
    with pytest.raises(ValueError):
        fuse("object", bad_image)


# --- models -----------------------------------------------------------------------------------------------------


def test_grasp_check_round_trips_through_json() -> None:
    evidence = GripEvidence(**measured().model_dump(), band="object")
    check = GraspCheck(
        grasp_id=evidence.grasp_id,
        band=evidence.band,
        evidence=evidence,
        image="pending",
        crop=CropInfo(lookup_id="lk1", box_px=(10, 20, 300, 260), full_frame_reason=None),
        fused=None,
    )
    again = GraspCheck.model_validate_json(check.model_dump_json())
    assert again == check
    assert again.crop is not None and again.crop.box_px == (10, 20, 300, 260)
    full = CropInfo(
        lookup_id=None,
        box_px=None,
        full_frame_reason="head not aimed at the tool point: point lies outside the head limits",
    )
    assert GraspCheck(**{**check.model_dump(), "crop": full.model_dump(), "image": "not_requested"}).crop == full


def test_models_reject_bad_literals() -> None:
    with pytest.raises(ValueError):
        GripEvidence(**measured().model_dump(), band="full")
    with pytest.raises(ValueError):
        measured(contact="squeeze")
    with pytest.raises(ValueError):
        measured(baseline_source="later")
    evidence = GripEvidence(**measured().model_dump(), band="object")
    with pytest.raises(ValueError):
        GraspCheck(grasp_id="x", band="object", evidence=evidence, image="maybe", crop=None, fused=None)


# --- current ingestion ------------------------------------------------------------------------------------------


def test_joint_sample_currents_default_empty() -> None:
    sample = JointSample(positions={"gripper": 0.1}, efforts={}, stamp=1.0)
    assert sample.currents == {}


def test_fake_backend_supplies_currents() -> None:
    be = FakeArmBackend()
    sample = be.joint_sample()
    assert sample is not None and sample.currents == {}
    be.currents["gripper"] = 0.12
    sample = be.joint_sample()
    assert sample is not None and sample.currents == {"gripper": 0.12}
    be.currents["gripper"] = 0.5
    assert sample.currents == {"gripper": 0.12}  # a copy, not the live dict


def ros_robot(module: ModuleType, latest: dict[str, Any]) -> Any:
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.lock = module.threading.Lock()
    robot.cfg = McpServerConfig()
    robot.latest = latest
    return robot


def joint_states(module: ModuleType, stamp: float) -> Any:
    msg = SimpleNamespace(name=["shoulder_pan", "gripper"], position=[0.1, 0.3], effort=[NAN, 120.0])
    return module.Stamped(value=msg, stamp=stamp)


def joint_currents(module: ModuleType, stamp: float, effort: list[float] | None = None) -> Any:
    msg = SimpleNamespace(name=["gripper"], position=[], velocity=[], effort=effort if effort is not None else [0.2])
    return module.Stamped(value=msg, stamp=stamp)


def test_ros_iface_subscribes_follower_joint_currents(ros_iface: ModuleType) -> None:
    robot = ros_iface.RosRobot(McpServerConfig())
    topics = [c.args[1] for c in robot.node.create_subscription.call_args_list]
    assert "/follower/joint_currents" in topics


def test_ros_joint_sample_without_currents_message_has_empty_currents(ros_iface: ModuleType) -> None:
    robot = ros_robot(ros_iface, {"joint_states": joint_states(ros_iface, 10.0)})
    assert robot.joint_sample().currents == {}


def test_ros_joint_sample_takes_a_fresh_currents_message(ros_iface: ModuleType) -> None:
    stale_s = McpServerConfig().timeouts.follower_stale_s
    for current_stamp in (10.0, 10.0 - stale_s + 0.01, 10.0 + 0.05):
        latest = {
            "joint_states": joint_states(ros_iface, 10.0),
            "joint_currents": joint_currents(ros_iface, current_stamp),
        }
        sample = ros_robot(ros_iface, latest).joint_sample()
        assert sample.currents == {"gripper": 0.2}
        assert sample.efforts == {"gripper": 120.0}


def test_ros_joint_sample_ignores_a_stale_currents_message(ros_iface: ModuleType) -> None:
    stale_s = McpServerConfig().timeouts.follower_stale_s
    latest = {
        "joint_states": joint_states(ros_iface, 10.0),
        "joint_currents": joint_currents(ros_iface, 10.0 - stale_s - 0.01),
    }
    assert ros_robot(ros_iface, latest).joint_sample().currents == {}


def test_ros_joint_sample_drops_non_finite_currents(ros_iface: ModuleType) -> None:
    msg = SimpleNamespace(name=["gripper", "wrist_roll"], position=[], velocity=[], effort=[NAN, 0.1])
    latest = {"joint_states": joint_states(ros_iface, 10.0), "joint_currents": ros_iface.Stamped(value=msg, stamp=10.0)}
    assert ros_robot(ros_iface, latest).joint_sample().currents == {"wrist_roll": 0.1}


def test_ros_joint_sample_none_before_joint_states_even_with_currents(ros_iface: ModuleType) -> None:
    robot = ros_robot(ros_iface, {"joint_currents": joint_currents(ros_iface, 10.0)})
    assert robot.joint_sample() is None


def test_ros_currents_topic_follows_the_config(ros_iface: ModuleType) -> None:
    cfg = McpServerConfig.model_validate({"topics": {"follower_joint_currents": "/arm/joint_currents"}})
    robot = ros_iface.RosRobot(cfg)
    topics = [c.args[1] for c in robot.node.create_subscription.call_args_list]
    assert "/arm/joint_currents" in topics
    assert isinstance(robot.node, MagicMock)


# --- LiveState.currents -----------------------------------------------------------------------------------------


def test_live_state_currents_default_empty() -> None:
    assert LiveState().currents == {}


def test_read_live_state_carries_fresh_currents(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    assert read_live_state(robot, None, need_twist=False).currents == {}
    robot.arm_backend.currents["gripper"] = 0.12
    assert read_live_state(robot, None, need_twist=False).currents == {"gripper": 0.12}


def test_read_live_state_without_a_fresh_sample_has_no_currents(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    robot.arm_backend.currents["gripper"] = 0.12
    robot.arm_backend.no_samples = True
    state = read_live_state(robot, None, need_twist=False)
    assert state.joints is None and state.currents == {}


def test_carrying_state_live_state_carries_currents(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = FakeRobot(tmp_path)
    robot.arm_backend.currents["gripper"] = 0.2
    seen: list[LiveState] = []

    def spy(precondition: Any, state: LiveState, config: McpServerConfig) -> tuple[bool, str]:
        seen.append(state)
        return False, "spy"

    monkeypatch.setattr(control, "evaluate_precondition", spy)
    manager(robot).carrying_state()
    assert seen and seen[0].currents == {"gripper": 0.2}
