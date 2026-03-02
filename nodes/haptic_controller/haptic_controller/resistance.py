"""Resistance control law for haptic force-feedback (no ROS2 dependency)."""


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
