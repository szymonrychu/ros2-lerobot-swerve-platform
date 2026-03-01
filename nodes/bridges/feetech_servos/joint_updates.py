"""Helpers for detecting joint position changes for logging."""

LOG_POSITION_EPSILON = 1e-4


def get_position_updates(
    names: list[str],
    positions: list[float],
    last_positions: dict[str, float],
    epsilon: float = LOG_POSITION_EPSILON,
) -> list[tuple[str, float]]:
    """Return changed joint positions and update last_positions in-place.

    Args:
        names: Ordered joint names.
        positions: Ordered joint positions matching names.
        last_positions: Cache of last logged position by joint name.
        epsilon: Minimum absolute delta to treat as a change.

    Returns:
        list[tuple[str, float]]: (joint_name, new_position) pairs that changed.
    """
    changes: list[tuple[str, float]] = []
    for i, name in enumerate(names):
        if i >= len(positions):
            continue
        new_val = float(positions[i])
        old_val = last_positions.get(name)
        if old_val is None or abs(new_val - old_val) > epsilon:
            changes.append((name, new_val))
            last_positions[name] = new_val
    return changes
