"""Resistance control law helpers for haptic force-feedback (no ROS2 dependency)."""


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
    if follower_load <= deadband:
        return leader_pos
    # Oppose closing: positive velocity (increasing position) = closing in typical gripper convention
    if leader_vel <= 0:
        return leader_pos
    delta = stiffness * (follower_load - deadband)
    delta = min(delta, max_step)
    return leader_pos - delta
