"""Gravity sag model (mcp_server.sag): URDF gravity torques, deflection, compensation bounds, gain fit, log parsing."""

import json
import math
from pathlib import Path

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.sag import (
    APPROACH_HOLD,
    APPROACH_LIFTING,
    APPROACH_LOWERING,
    SAG_JOINTS,
    GravityModel,
    SagCompensator,
    compensate,
    cross_validate,
    fit_gains,
    parse_settle_records,
    predict_deflection,
    rms_by_joint,
)

URDF = McpServerConfig().arm.urdf_path

ONE_LINK = """<?xml version="1.0"?>
<robot name="one">
  <link name="base_link"/>
  <link name="arm"><inertial><origin xyz="1 0 0" rpy="0 0 0"/><mass value="1.0"/></inertial></link>
  <link name="tip"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.5"/></inertial></link>
  <joint name="lift" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/><parent link="base_link"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="bend" type="revolute">
    <origin xyz="2 0 0" rpy="0 0 0"/><parent link="arm"/><child link="tip"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def one_link(tmp_path: Path) -> GravityModel:
    path = tmp_path / "one.urdf"
    path.write_text(ONE_LINK)
    return GravityModel(path, joints=("lift", "bend"))


# --- gravity torques -------------------------------------------------------------------------------------------


def test_torque_of_a_horizontal_point_mass_about_a_horizontal_axis(tmp_path: Path) -> None:
    model = one_link(tmp_path)
    tau = model.torques({"lift": 0.0, "bend": 0.0})
    # lift carries 1 kg at 1 m and 0.5 kg at 2 m: (1 * 1 + 0.5 * 2) * 9.81 about +y (r x F with F = -z).
    assert tau["lift"] == pytest.approx(2.0 * 9.81)
    # bend carries only the tip mass, sitting on its axis.
    assert tau["bend"] == pytest.approx(0.0, abs=1e-12)


def test_torque_vanishes_when_the_link_hangs_straight_down(tmp_path: Path) -> None:
    model = one_link(tmp_path)
    # +90 deg about +y turns the link's x axis to -z: masses straight below the axis.
    tau = model.torques({"lift": math.pi / 2, "bend": 0.0})
    assert tau["lift"] == pytest.approx(0.0, abs=1e-9)


def test_torque_follows_the_cosine_of_the_link_angle(tmp_path: Path) -> None:
    model = one_link(tmp_path)
    tau = model.torques({"lift": 0.5, "bend": 0.0})
    assert tau["lift"] == pytest.approx(2.0 * 9.81 * math.cos(0.5))


def test_missing_joints_default_to_zero(tmp_path: Path) -> None:
    model = one_link(tmp_path)
    assert model.torques({}) == model.torques({"lift": 0.0, "bend": 0.0})


def test_unknown_model_joint_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "one.urdf"
    path.write_text(ONE_LINK)
    with pytest.raises(ValueError, match="nope"):
        GravityModel(path, joints=("nope",))


def test_so101_vertical_pan_axis_carries_no_gravity_torque() -> None:
    model = GravityModel(URDF, joints=("shoulder_pan", *SAG_JOINTS))
    for pose in ({}, {"shoulder_lift": 0.8, "elbow_flex": -0.5}, {"shoulder_pan": 1.0, "wrist_flex": 1.2}):
        assert model.torques(pose)["shoulder_pan"] == pytest.approx(0.0, abs=1e-4)  # URDF rpy 3.14159, not pi


def test_so101_stretched_arm_loads_the_shoulder_more_than_a_folded_one() -> None:
    model = GravityModel(URDF)
    stretched = model.torques({"shoulder_lift": 1.0, "elbow_flex": -1.2, "wrist_flex": 0.0})
    folded = model.torques({"shoulder_lift": 0.0, "elbow_flex": 1.5, "wrist_flex": 0.0})
    assert abs(stretched["shoulder_lift"]) > abs(folded["shoulder_lift"])
    # The shoulder carries every link beyond it: its load exceeds the elbow's, which exceeds the wrist's.
    assert abs(stretched["shoulder_lift"]) > abs(stretched["elbow_flex"]) > abs(stretched["wrist_flex"])
    total_mass = sum(model.masses.values())
    assert 0.4 < total_mass < 0.8  # URDF inertials (kg) of every link


# --- deflection and compensation -------------------------------------------------------------------------------


def test_deflection_is_gain_times_torque_and_saturates() -> None:
    tau = {"shoulder_lift": 1.0, "elbow_flex": -0.5, "wrist_flex": 0.02}
    out = predict_deflection(tau, {"shoulder_lift": 0.05, "elbow_flex": 0.4}, max_rad=0.12)
    assert out["shoulder_lift"] == pytest.approx(0.05)
    assert out["elbow_flex"] == pytest.approx(-0.12)  # -0.2 saturated
    assert out["wrist_flex"] == 0.0  # no gain configured


def test_compensate_counteracts_the_deflection() -> None:
    target = {"shoulder_lift": 0.5, "elbow_flex": -0.3, "wrist_roll": 0.4}
    limits = {"shoulder_lift": (-2.0, 2.0), "elbow_flex": (-2.0, 2.0), "wrist_roll": (-3.0, 3.0)}
    out = compensate(target, {"shoulder_lift": 0.06, "elbow_flex": -0.02}, limits, margin=0.05)
    assert out == pytest.approx({"shoulder_lift": 0.44, "elbow_flex": -0.28, "wrist_roll": 0.4})


def test_compensate_never_leaves_the_limit_band() -> None:
    limits = {"shoulder_lift": (-1.0, 1.0)}
    out = compensate({"shoulder_lift": -0.92}, {"shoulder_lift": 0.1}, limits, margin=0.05)
    assert out["shoulder_lift"] == pytest.approx(-0.95)


def test_compensate_does_not_push_a_target_already_past_the_band_further_out() -> None:
    limits = {"shoulder_lift": (-1.0, 1.0)}
    out = compensate({"shoulder_lift": -0.97}, {"shoulder_lift": 0.1}, limits, margin=0.05)
    assert out["shoulder_lift"] == pytest.approx(-0.97)  # a hold at a measured pose inside the margin stays put
    back = compensate({"shoulder_lift": -0.97}, {"shoulder_lift": -0.1}, limits, margin=0.05)
    assert back["shoulder_lift"] == pytest.approx(-0.87)  # compensation back into the band is applied


def test_compensate_uses_per_joint_margins() -> None:
    limits = {"shoulder_lift": (-1.0, 1.0)}
    out = compensate(
        {"shoulder_lift": 0.85}, {"shoulder_lift": -0.1}, limits, margin=0.05, overrides={"shoulder_lift": 0.1}
    )
    assert out["shoulder_lift"] == pytest.approx(0.9)


def test_compensator_offsets_and_measured_space() -> None:
    model = GravityModel(URDF)
    offsets = {"shoulder_lift": -0.03, "elbow_flex": -0.1, "wrist_flex": 0.1}
    comp = SagCompensator(model, {"shoulder_lift": 0.1, "elbow_flex": 0.1, "wrist_flex": 0.1}, 0.12, offsets)
    measured = {"shoulder_pan": 0.0, "shoulder_lift": 1.0, "elbow_flex": -1.0, "wrist_flex": 0.2, "wrist_roll": 0.0}
    delta = comp.deflection(measured)
    urdf = {j: v + offsets.get(j, 0.0) for j, v in measured.items()}
    tau = model.torques(urdf)
    for j in SAG_JOINTS:
        assert delta[j] == pytest.approx(max(-0.12, min(0.12, 0.1 * tau[j])))
    assert set(delta) == set(SAG_JOINTS)
    halved = comp.deflection(measured, {"shoulder_lift": 0.05})
    assert halved["shoulder_lift"] == pytest.approx(max(-0.12, min(0.12, 0.05 * tau["shoulder_lift"])))
    assert halved["elbow_flex"] == 0.0


def compensator() -> SagCompensator:
    return SagCompensator(
        GravityModel(URDF), {"shoulder_lift": 0.14, "elbow_flex": 0.16}, 0.12, k_lowering={"shoulder_lift": 0.01}
    )


def test_gains_per_approach_mode() -> None:
    comp = compensator()
    modes = {"shoulder_lift": APPROACH_LIFTING, "elbow_flex": APPROACH_LOWERING, "wrist_flex": APPROACH_HOLD}
    gains = comp.gains_for(modes)
    assert gains == pytest.approx({"shoulder_lift": 0.14, "elbow_flex": 0.0, "wrist_flex": 0.0})
    # A hold at a measured pose sits mid-way in the friction band between the two approach gains.
    assert comp.gains_for(comp.hold_modes()) == pytest.approx(
        {"shoulder_lift": 0.075, "elbow_flex": 0.08, "wrist_flex": 0.0}
    )


def test_approach_mode_follows_the_last_motion_against_or_with_gravity() -> None:
    comp = compensator()
    goal = {"shoulder_pan": 0.0, "shoulder_lift": 0.8, "elbow_flex": -0.3, "wrist_flex": 0.2, "wrist_roll": 0.0}
    tau = comp.torques(goal)
    assert tau["shoulder_lift"] > 0.0 and tau["elbow_flex"] > 0.0
    # shoulder rises toward the goal from above (decreasing: against gravity), elbow comes down with gravity,
    # the wrist does not move.
    points = [
        goal | {"shoulder_lift": 1.0, "elbow_flex": -0.5},
        goal | {"shoulder_lift": 0.9, "elbow_flex": -0.4},
        goal,
    ]
    prior = comp.hold_modes()
    modes = comp.approach_modes(points, goal, prior)
    assert modes == {"shoulder_lift": APPROACH_LIFTING, "elbow_flex": APPROACH_LOWERING, "wrist_flex": APPROACH_HOLD}
    # Only the final approach counts: a reversal before the goal changes the mode.
    reverse = [goal | {"shoulder_lift": 1.0}, goal | {"shoulder_lift": 0.7}, goal]
    assert comp.approach_modes(reverse, goal, prior)["shoulder_lift"] == APPROACH_LOWERING


def test_blend_gains_interpolates_between_two_gain_sets() -> None:
    a = {"shoulder_lift": 0.0, "elbow_flex": 0.2}
    b = {"shoulder_lift": 0.1, "elbow_flex": 0.0}
    assert SagCompensator.blend_gains(a, b, 0.0) == a
    assert SagCompensator.blend_gains(a, b, 1.0) == b
    assert SagCompensator.blend_gains(a, b, 0.25) == pytest.approx({"shoulder_lift": 0.025, "elbow_flex": 0.15})


# --- fit -------------------------------------------------------------------------------------------------------


def test_fit_recovers_the_gain_of_noise_free_data() -> None:
    pairs = [
        ({"shoulder_lift": t, "elbow_flex": -0.5 * t}, {"shoulder_lift": 0.07 * t, "elbow_flex": -0.5 * t * 0.2})
        for t in (0.3, 0.6, 0.9, 1.2)
    ]
    gains = fit_gains(pairs, ("shoulder_lift", "elbow_flex"))
    assert gains == pytest.approx({"shoulder_lift": 0.07, "elbow_flex": 0.2})


def test_fit_skips_joints_without_samples_and_never_returns_a_negative_gain() -> None:
    pairs = [({"shoulder_lift": 1.0}, {"shoulder_lift": -0.05})]  # deflection against gravity: not a sag
    gains = fit_gains(pairs, ("shoulder_lift", "wrist_flex"))
    assert gains == {"shoulder_lift": 0.0, "wrist_flex": 0.0}


def test_rms_before_and_after_compensation() -> None:
    pairs = [({"shoulder_lift": 1.0}, {"shoulder_lift": 0.06}), ({"shoulder_lift": 0.5}, {"shoulder_lift": 0.04})]
    before, after = rms_by_joint(pairs, {"shoulder_lift": 0.06}, max_rad=0.12, joints=("shoulder_lift",))
    assert before["shoulder_lift"] == pytest.approx(math.sqrt((0.06**2 + 0.04**2) / 2))
    assert after["shoulder_lift"] == pytest.approx(math.sqrt((0.0**2 + 0.01**2) / 2))


def test_cross_validation_holds_each_group_out() -> None:
    pairs = [({"shoulder_lift": t}, {"shoulder_lift": 0.05 * t}) for t in (0.4, 0.8, 1.2)]
    report = cross_validate(pairs, ["a", "b", "c"], ("shoulder_lift",), max_rad=0.12)
    assert report["folds"] == 3
    assert report["after"]["shoulder_lift"] == pytest.approx(0.0, abs=1e-12)
    assert report["before"]["shoulder_lift"] > 0.0
    assert report["reduction"]["shoulder_lift"] == pytest.approx(1.0)


# --- structured log records ------------------------------------------------------------------------------------


def test_parse_settle_records_from_journal_lines() -> None:
    record = {
        "status": "converged",
        "target": {"shoulder_lift": 1.0},
        "commanded": {"shoulder_lift": 0.94},
        "measured": {"shoulder_lift": 1.05},
        "residual": {"shoulder_lift": -0.05},
        "compensation": {"shoulder_lift": 0.06},
    }
    lines = [
        "Oct 10 12:05:59 client-rpi5 ros2-mcp_server[1]: 2026 INFO mcp_server.timing: tool_call {}",
        f"Oct 10 12:06:00 client-rpi5 ros2-mcp_server[1]: 2026 INFO mcp_server.sag: arm_settle {json.dumps(record)}",
        "arm_settle {not json",
    ]
    records = parse_settle_records(lines)
    assert records == [record]
