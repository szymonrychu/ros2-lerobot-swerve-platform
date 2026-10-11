"""Resistance control law helpers for haptic force-feedback (no ROS2 dependency)."""

import math


def is_measured_load(follower_load: float) -> bool:
    """Return True when the follower load is a real measurement.

    The feetech bridge publishes NaN effort for non-effort joints and missing load readings;
    such a load is unmeasured and must never drive resistance.

    Args:
        follower_load (float): Effort reported for the follower joint.

    Returns:
        bool: False for NaN and +/-inf, True otherwise.
    """
    return math.isfinite(follower_load)


def should_apply_resistance(
    leader_vel: float,
    follower_load: float,
    load_deadband: float,
    activation_velocity_threshold: float,
) -> bool:
    """Return True when resistance should be rendered.

    Resistance is rendered only when follower load indicates contact and
    leader motion is in the closing direction with sufficient speed.
    """
    if not is_measured_load(follower_load):
        return False
    return follower_load > load_deadband and leader_vel > activation_velocity_threshold


def should_apply_resistance_hysteresis(
    leader_vel: float,
    follower_load: float,
    load_deadband: float,
    activation_velocity_threshold: float,
    load_release_ratio: float,
    was_active: bool,
) -> bool:
    """Like should_apply_resistance but with load hysteresis to reduce chatter.

    Activate when load > load_deadband and leader_vel > threshold.
    Once active, stay active until load < load_deadband * load_release_ratio.
    load_release_ratio should be in (0, 1), e.g. 0.6.
    """
    if not is_measured_load(follower_load):
        return False
    release_threshold = load_deadband * load_release_ratio
    if was_active:
        if follower_load < release_threshold:
            return False
        return leader_vel > activation_velocity_threshold
    return follower_load > load_deadband and leader_vel > activation_velocity_threshold


def compute_resistance_target(
    leader_pos: float,
    leader_vel: float,
    follower_load: float,
    deadband: float,
    stiffness: float,
    max_step: float,
) -> float:
    """Compute target position for resistance mode: oppose closing when follower has load.

    When user is closing (leader_vel > 0 in position-increasing direction) and follower
    reports load above deadband, push back by reducing target position (open direction).
    Returns leader_pos with a resistance delta applied (clamped to max_step per cycle).
    """
    if not is_measured_load(follower_load) or follower_load <= deadband:
        return leader_pos
    # Oppose closing: positive velocity (increasing position) = closing in typical gripper convention
    if leader_vel <= 0:
        return leader_pos
    delta = stiffness * (follower_load - deadband)
    delta = min(delta, max_step)
    return leader_pos - delta
