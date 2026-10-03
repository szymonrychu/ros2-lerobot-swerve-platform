"""Unit tests for swerve drive kinematics (IK, FK, safeguard)."""

import math

import pytest

from swerve_drive_controller.kinematics import (
    GROUP_FLIP_HYSTERESIS_RAD,
    STEER_LIMIT_HYSTERESIS_RAD,
    choose_common_flip,
    compute_wheel_commands,
    desaturate_wheel_speeds,
    fold_to_steer_range,
    forward_kinematics,
    integrate_odometry,
    inverse_kinematics,
    normalize_angle,
    should_zero_drive,
    steer_angle_difference,
    wheel_positions,
    wheel_states,
)


def test_wheel_positions() -> None:
    lx, ly = 0.2, 0.15
    pos = wheel_positions(lx, ly)
    assert pos.shape == (4, 2)
    assert pos[0, 0] == lx and pos[0, 1] == ly  # fl
    assert pos[1, 0] == lx and pos[1, 1] == -ly  # fr
    assert pos[2, 0] == -lx and pos[2, 1] == ly  # rl
    assert pos[3, 0] == -lx and pos[3, 1] == -ly  # rr


def test_inverse_kinematics_straight_forward() -> None:
    steer, drive = inverse_kinematics(1.0, 0.0, 0.0, 0.2, 0.15, 0.15)
    assert len(steer) == 4 and len(drive) == 4
    for i in range(4):
        assert abs(steer[i]) < 1e-9
    for i in range(4):
        assert abs(drive[i] - 1.0 / 0.15) < 1e-6


def test_inverse_kinematics_straight_sideways() -> None:
    steer, drive = inverse_kinematics(0.0, 1.0, 0.0, 0.2, 0.15, 0.15)
    for i in range(4):
        assert abs(steer[i] - math.pi / 2) < 1e-6
    for i in range(4):
        assert abs(drive[i] - 1.0 / 0.15) < 1e-6


def test_inverse_kinematics_zero() -> None:
    steer, drive = inverse_kinematics(0.0, 0.0, 0.0, 0.2, 0.15, 0.15)
    for i in range(4):
        assert steer[i] == 0.0 and drive[i] == 0.0


def test_forward_kinematics_roundtrip() -> None:
    vx, vy, omega = 0.5, -0.2, 0.3
    steer, drive = inverse_kinematics(vx, vy, omega, 0.2, 0.15, 0.15)
    vx_fk, vy_fk, omega_fk = forward_kinematics(steer, drive, 0.2, 0.15, 0.15)
    assert abs(vx_fk - vx) < 1e-6 and abs(vy_fk - vy) < 1e-6 and abs(omega_fk - omega) < 1e-6


def test_steer_angle_difference() -> None:
    assert abs(steer_angle_difference(0.0, 0.0)) < 1e-9
    assert abs(steer_angle_difference(0.0, math.pi / 2) - math.pi / 2) < 1e-9
    # -pi and +pi are the same angle (difference 0)
    assert abs(steer_angle_difference(math.pi, -math.pi)) < 1e-9


def test_should_zero_drive() -> None:
    assert should_zero_drive(0.0, 0.0, 0.35) is False
    assert should_zero_drive(0.0, 0.5, 0.35) is True
    assert should_zero_drive(0.0, 0.2, 0.35) is False


def test_normalize_angle_already_normalized() -> None:
    assert abs(normalize_angle(0.5) - 0.5) < 1e-9


def test_normalize_angle_wraps_positive() -> None:
    assert abs(normalize_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-6


def test_normalize_angle_wraps_negative() -> None:
    assert abs(normalize_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-6


# Real platform geometry (Platform dimensions.md): 305 x 266.6 mm yaw-axis rectangle, 60 mm wheels.
LX = 0.1525
LY = 0.1333
R = 0.06
HALF_PI = math.pi / 2


def test_fold_keeps_angle_inside_limit() -> None:
    assert fold_to_steer_range(0.3, 2.0, current_steer=0.0, limit=HALF_PI) == pytest.approx((0.3, 2.0))


def test_fold_flips_backward_angle_and_reverses_drive() -> None:
    steer, drive = fold_to_steer_range(math.pi - 0.2, 2.0, current_steer=0.0, limit=HALF_PI)
    assert steer == pytest.approx(-0.2)
    assert drive == pytest.approx(-2.0)


def test_fold_straight_backward_is_zero_steer_negative_drive() -> None:
    steer, drive = fold_to_steer_range(math.pi, 1.0, current_steer=0.0, limit=HALF_PI)
    assert steer == pytest.approx(0.0, abs=1e-9)
    assert drive == pytest.approx(-1.0)


def test_fold_boundary_picks_side_closer_to_current() -> None:
    steer, drive = fold_to_steer_range(HALF_PI, 1.0, current_steer=-1.4, limit=HALF_PI)
    assert steer == pytest.approx(-HALF_PI)
    assert drive == pytest.approx(-1.0)
    steer, drive = fold_to_steer_range(-HALF_PI, 1.0, current_steer=1.4, limit=HALF_PI)
    assert steer == pytest.approx(HALF_PI)
    assert drive == pytest.approx(-1.0)


def test_desaturate_scales_all_wheels_proportionally() -> None:
    assert desaturate_wheel_speeds([2.0, -8.0, 4.0, 1.0], 4.0) == pytest.approx([1.0, -4.0, 2.0, 0.5])


def test_desaturate_noop_within_limit() -> None:
    assert desaturate_wheel_speeds([1.0, -2.0, 0.0, 3.9], 4.0) == pytest.approx([1.0, -2.0, 0.0, 3.9])


def test_wheel_commands_forward() -> None:
    steer, drive, _ = compute_wheel_commands(0.1, 0.0, 0.0, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    assert steer == pytest.approx([0.0] * 4)
    assert drive == pytest.approx([0.1 / R] * 4)


def test_wheel_commands_backward_keeps_wheels_straight() -> None:
    steer, drive, _ = compute_wheel_commands(-0.1, 0.0, 0.0, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    assert steer == pytest.approx([0.0] * 4, abs=1e-9)
    assert drive == pytest.approx([-0.1 / R] * 4)


def test_wheel_commands_strafe_left_within_limits() -> None:
    steer, drive, _ = compute_wheel_commands(0.0, 0.1, 0.0, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    for s, d in zip(steer, drive):
        assert abs(s) == pytest.approx(HALF_PI)
        assert math.copysign(1.0, s) * d * R == pytest.approx(0.1)


def test_wheel_commands_rotate_in_place() -> None:
    """Pure yaw: wheels tangent to the circle through the yaw axes, all at the same speed."""
    omega = 0.5
    steer, drive, _ = compute_wheel_commands(0.0, 0.0, omega, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    alpha = math.atan2(LX, LY)
    assert steer == pytest.approx([-alpha, alpha, alpha, -alpha])
    speed = omega * math.hypot(LX, LY) / R
    assert [abs(d) for d in drive] == pytest.approx([speed] * 4)
    vx, vy, w = forward_kinematics(steer, drive, LX, LY, R)
    assert (vx, vy, w) == pytest.approx((0.0, 0.0, omega), abs=1e-9)


def test_wheel_commands_stopped_holds_current_steer() -> None:
    current = [0.4, -0.2, 0.1, 0.0]
    steer, drive, _ = compute_wheel_commands(0.0, 0.0, 0.0, current, LX, LY, R, HALF_PI, 10.0)
    assert steer == pytest.approx(current)
    assert drive == pytest.approx([0.0] * 4)


def test_wheel_commands_desaturated_preserves_direction() -> None:
    steer, drive, _ = compute_wheel_commands(1.0, 0.0, 1.0, [0.0] * 4, LX, LY, R, HALF_PI, 4.0)
    assert max(abs(d) for d in drive) == pytest.approx(4.0)
    vx, vy, w = forward_kinematics(steer, drive, LX, LY, R)
    assert w / vx == pytest.approx(1.0)


def test_wheel_commands_roundtrip_random_twists() -> None:
    for vx, vy, omega in [(0.1, 0.05, 0.2), (-0.08, 0.02, -0.4), (0.0, -0.1, 0.3), (0.05, 0.0, -1.0)]:
        steer, drive, _ = compute_wheel_commands(vx, vy, omega, [0.0] * 4, LX, LY, R, HALF_PI, 100.0)
        assert all(abs(s) <= HALF_PI + 1e-9 for s in steer)
        assert forward_kinematics(steer, drive, LX, LY, R) == pytest.approx((vx, vy, omega), abs=1e-9)


def test_wheel_states_requires_all_joints() -> None:
    positions = {"fl_steer": 0.1, "fr_steer": 0.2, "rl_steer": 0.3}
    velocities = {"fl_drive": 1.0, "fr_drive": 1.0, "rl_drive": 1.0, "rr_drive": 1.0}
    steer_joints = ["fl_steer", "fr_steer", "rl_steer", "rr_steer"]
    drive_joints = ["fl_drive", "fr_drive", "rl_drive", "rr_drive"]
    assert wheel_states(positions, velocities, steer_joints, drive_joints) is None
    positions["rr_steer"] = 0.4
    assert wheel_states(positions, velocities, steer_joints, drive_joints) == ([0.1, 0.2, 0.3, 0.4], [1.0] * 4)


def test_integrate_odometry_straight_then_rotated() -> None:
    assert integrate_odometry((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), 1.0) == pytest.approx((0.1, 0.0, 0.0))
    x, y, theta = integrate_odometry((0.0, 0.0, HALF_PI), (0.1, 0.0, 0.0), 1.0)
    assert (x, y, theta) == pytest.approx((0.0, 0.1, HALF_PI), abs=1e-9)


def test_integrate_odometry_arc_uses_midpoint_heading() -> None:
    x, y, theta = integrate_odometry((0.0, 0.0, 0.0), (0.1, 0.0, 0.2), 1.0)
    assert theta == pytest.approx(0.2)
    assert x == pytest.approx(0.1 * math.cos(0.1))
    assert y == pytest.approx(0.1 * math.sin(0.1))


def test_fold_hysteresis_keeps_side_just_past_limit() -> None:
    """Heading slightly past +90 deg with the wheel at the + side: stay at +90 instead of swinging to -90."""
    steer, drive = fold_to_steer_range(HALF_PI + 0.05, 1.0, current_steer=HALF_PI - 0.01, limit=HALF_PI)
    assert steer == pytest.approx(HALF_PI)
    assert drive == pytest.approx(1.0)
    steer, drive = fold_to_steer_range(-HALF_PI - 0.05, 1.0, current_steer=-HALF_PI + 0.01, limit=HALF_PI)
    assert steer == pytest.approx(-HALF_PI)
    assert drive == pytest.approx(1.0)


def test_fold_hysteresis_switches_side_when_far_past_limit() -> None:
    steer, drive = fold_to_steer_range(HALF_PI + 0.5, 1.0, current_steer=HALF_PI, limit=HALF_PI)
    assert steer == pytest.approx(-HALF_PI + 0.5)
    assert drive == pytest.approx(-1.0)


# --- Coordinated (group) steering side choice -------------------------------------------------------------------

TURN_NEAR_LIMIT = (0.0, 0.1, 0.05)  # strafing left while turning: IK headings ~86..94 deg, straddling +90 deg
SWEEP_OMEGA = 0.02  # small yaw rate: headings spread ~4 deg, so a common side stays feasible through 90 deg


def wheel_flips(steer: list[float], drive: list[float], ik_steer: list[float]) -> list[bool]:
    """Per wheel: True when the output uses the flipped equivalent (heading + pi, reversed drive)."""
    return [abs(steer_angle_difference(ik, s)) > HALF_PI for s, ik in zip(steer, ik_steer)]


def assert_wheel_velocities_match_ik(twist: tuple[float, float, float], steer: list[float], drive: list[float]) -> bool:
    """Each wheel's ground velocity equals the IK one (angle off by at most the clamp tolerance); True if exact."""
    ik_steer, ik_drive = inverse_kinematics(*twist, LX, LY, R)
    exact = True
    for s, d, ik_s, ik_d in zip(steer, drive, ik_steer, ik_drive):
        assert abs(d) == pytest.approx(ik_d, abs=1e-9)
        if ik_d == 0.0:
            continue
        heading = s if d >= 0.0 else s + math.pi
        error = abs(steer_angle_difference(ik_s, heading))
        assert error <= STEER_LIMIT_HYSTERESIS_RAD + 1e-9
        exact = exact and error < 1e-9
    if exact:
        assert forward_kinematics(steer, drive, LX, LY, R) == pytest.approx(twist, abs=1e-9)
    return exact


def test_coordinated_turn_near_limit_keeps_all_wheels_on_one_side() -> None:
    """From centred wheels the old per-wheel fold splits the wheels across +-90 deg; the group choice does not."""
    ik_steer, ik_drive = inverse_kinematics(*TURN_NEAR_LIMIT, LX, LY, R)
    per_wheel = [fold_to_steer_range(a, d, 0.0, HALF_PI) for a, d in zip(ik_steer, ik_drive)]
    old_flips = wheel_flips([p[0] for p in per_wheel], [p[1] for p in per_wheel], ik_steer)
    assert len(set(old_flips)) == 2  # the bug: some wheels forward at +86 deg, others reversed at -86 deg

    steer, drive, flip = compute_wheel_commands(*TURN_NEAR_LIMIT, [0.0] * 4, LX, LY, R, HALF_PI, 100.0)
    assert flip is not None
    assert wheel_flips(steer, drive, ik_steer) == [flip] * 4
    assert len({math.copysign(1.0, s) for s in steer}) == 1
    assert len({math.copysign(1.0, d) for d in drive}) == 1
    assert all(abs(s) <= HALF_PI + 1e-9 for s in steer)
    assert_wheel_velocities_match_ik(TURN_NEAR_LIMIT, steer, drive)


def test_coordinated_choice_reunites_wheels_already_split() -> None:
    """Wheels left split by earlier per-wheel decisions are brought back onto one side."""
    current = [1.5, -1.5, 1.5, -1.5]
    ik_steer, _ = inverse_kinematics(*TURN_NEAR_LIMIT, LX, LY, R)
    steer, drive, flip = compute_wheel_commands(*TURN_NEAR_LIMIT, current, LX, LY, R, HALF_PI, 100.0)
    assert flip is not None
    assert wheel_flips(steer, drive, ik_steer) == [flip] * 4


def sweep(thetas: list[float], omega: float, speed: float = 0.1) -> list[tuple[list[float], list[float], bool | None]]:
    """Feed a sequence of travel directions through compute_wheel_commands, carrying steer targets and flip."""
    current = [0.0] * 4
    flip: bool | None = None
    outputs = []
    for theta in thetas:
        twist = (speed * math.cos(theta), speed * math.sin(theta), omega)
        steer, drive, flip = compute_wheel_commands(*twist, current, LX, LY, R, HALF_PI, 100.0, previous_flip=flip)
        assert all(abs(s) <= HALF_PI + 1e-9 for s in steer)
        assert_wheel_velocities_match_ik(twist, steer, drive)
        outputs.append((steer, drive, flip))
        current = steer
    return outputs


def test_sweep_through_90_deg_has_no_single_wheel_swings() -> None:
    """Sweeping the travel direction from 60 to 120 deg while turning: wheels only ever switch side together."""
    thetas = [math.radians(deg) for deg in range(60, 121)]
    outputs = sweep(thetas, omega=SWEEP_OMEGA)
    assert all(flip is not None for _, _, flip in outputs)  # a common choice stays feasible throughout
    group_flips = 0
    for (prev_steer, _, prev_flip), (steer, _, flip) in zip(outputs, outputs[1:]):
        jumps = [abs(s - p) > 1.0 for s, p in zip(steer, prev_steer)]
        assert not any(jumps) or all(jumps), f"single-wheel swing: {prev_steer} -> {steer}"
        if flip != prev_flip:
            assert all(jumps)
            group_flips += 1
    assert group_flips == 1


def test_sweep_old_per_wheel_fold_splits_wheels() -> None:
    """Same sweep with the per-wheel fold: some cycles leave the wheels split (documents the user's bug)."""
    current = [0.0] * 4
    split_cycles = 0
    for deg in range(60, 121):
        theta = math.radians(deg)
        ik_steer, ik_drive = inverse_kinematics(0.1 * math.cos(theta), 0.1 * math.sin(theta), SWEEP_OMEGA, LX, LY, R)
        folded = [fold_to_steer_range(a, d, c, HALF_PI) for a, d, c in zip(ik_steer, ik_drive, current)]
        current = [f[0] for f in folded]
        split_cycles += len(set(wheel_flips(current, [f[1] for f in folded], ik_steer))) == 2
    assert split_cycles > 0


def test_group_hysteresis_prevents_chattering_near_90_deg() -> None:
    """Travel direction dithering around 90 deg keeps the same group side every cycle."""
    thetas = [math.radians(90.0 + (2.0 if k % 2 else -2.0)) for k in range(40)]
    outputs = sweep(thetas, omega=0.02)
    flips = {flip for _, _, flip in outputs}
    assert len(flips) == 1 and None not in flips
    for (prev_steer, _, _), (steer, _, _) in zip(outputs, outputs[1:]):
        assert max(abs(s - p) for s, p in zip(steer, prev_steer)) < 0.2


def test_choose_common_flip_keeps_previous_within_threshold() -> None:
    """Both sides feasible and nearly equal travel: the previous group choice is kept."""
    ik = [math.radians(89.0)] * 4
    moving = [True] * 4
    assert choose_common_flip(ik, moving, [0.0] * 4, HALF_PI, previous_flip=True) is True
    assert choose_common_flip(ik, moving, [0.0] * 4, HALF_PI, previous_flip=False) is False
    assert choose_common_flip(ik, moving, [0.0] * 4, HALF_PI, previous_flip=None) is False  # less travel


def test_choose_common_flip_switches_when_travel_saving_exceeds_threshold() -> None:
    ik = [HALF_PI] * 4
    current = [0.5] * 4  # unflipped (+90) is 4 * 1.07 rad away, flipped (-90) is 4 * 2.07 rad away
    assert 4 * 1.0 > GROUP_FLIP_HYSTERESIS_RAD
    assert choose_common_flip(ik, [True] * 4, current, HALF_PI, previous_flip=True) is False


def test_choose_common_flip_infeasible_side_never_chosen() -> None:
    """Forward driving: the flipped side would need |steer| ~ 180 deg, so it is infeasible even if preferred."""
    ik = [0.1, -0.1, 0.05, 0.0]
    assert choose_common_flip(ik, [True] * 4, [0.0] * 4, HALF_PI, previous_flip=True) is False
    backward = [math.pi - 0.1, -math.pi + 0.1, math.pi, math.pi]
    assert choose_common_flip(backward, [True] * 4, [0.0] * 4, HALF_PI, previous_flip=False) is True


def test_choose_common_flip_hysteresis_only_on_current_side() -> None:
    """A heading just past +90 deg is feasible unflipped only for a wheel already on the + side."""
    ik = [HALF_PI + 0.05] * 4
    assert choose_common_flip(ik, [True] * 4, [1.4] * 4, HALF_PI, previous_flip=None) is False
    assert choose_common_flip(ik, [True] * 4, [-1.4] * 4, HALF_PI, previous_flip=None) is True


def test_choose_common_flip_ignores_stopped_wheels() -> None:
    ik = [0.1, 0.0, 0.1, 0.1]
    moving = [True, False, True, True]
    assert choose_common_flip(ik, moving, [0.0, 1.5, 0.0, 0.0], HALF_PI, previous_flip=None) is False
    assert choose_common_flip(ik, [False] * 4, [0.0] * 4, HALF_PI, previous_flip=True) is True


def test_pure_rotation_falls_back_to_per_wheel_fold() -> None:
    """Rotation in place spans more than 180 deg of headings: no common side, per-wheel fold within limits."""
    omega = 0.5
    ik_steer, ik_drive = inverse_kinematics(0.0, 0.0, omega, LX, LY, R)
    assert choose_common_flip(ik_steer, [True] * 4, [0.0] * 4, HALF_PI, previous_flip=False) is None
    steer, drive, flip = compute_wheel_commands(0.0, 0.0, omega, [0.0] * 4, LX, LY, R, HALF_PI, 100.0, False)
    assert flip is None
    expected = [fold_to_steer_range(a, d, 0.0, HALF_PI) for a, d in zip(ik_steer, ik_drive)]
    assert steer == pytest.approx([e[0] for e in expected])
    assert drive == pytest.approx([e[1] for e in expected])
    assert all(abs(s) <= HALF_PI + 1e-9 for s in steer)
    assert forward_kinematics(steer, drive, LX, LY, R) == pytest.approx((0.0, 0.0, omega), abs=1e-9)


def test_fk_roundtrip_holds_for_all_outputs() -> None:
    """Every output (any start state, any previous group choice) reproduces the commanded wheel velocities."""
    twists = [
        (vx, vy, w)
        for vx in (-0.1, -0.02, 0.0, 0.03, 0.1)
        for vy in (-0.1, -0.01, 0.0, 0.04, 0.1)
        for w in (-0.6, -0.05, 0.0, 0.2, 0.8)
        if (vx, vy, w) != (0.0, 0.0, 0.0)
    ]
    starts = [[0.0] * 4, [1.5, -1.5, 1.5, -1.5], [-1.2] * 4, [0.4, -0.3, 1.0, -1.0]]
    exact_cases = 0
    for twist in twists:
        for current in starts:
            for previous in (None, False, True):
                steer, drive, _ = compute_wheel_commands(
                    *twist, current, LX, LY, R, HALF_PI, 100.0, previous_flip=previous
                )
                assert all(abs(s) <= HALF_PI + 1e-9 for s in steer)
                exact_cases += assert_wheel_velocities_match_ik(twist, steer, drive)
    assert exact_cases > len(twists) * len(starts) * 3 // 2


def test_coordinated_desaturation_keeps_common_side() -> None:
    steer, drive, flip = compute_wheel_commands(0.0, 1.0, 0.5, [1.4] * 4, LX, LY, R, HALF_PI, 4.0, previous_flip=None)
    assert flip is False
    assert max(abs(d) for d in drive) == pytest.approx(4.0)
    assert all(d > 0.0 for d in drive)
