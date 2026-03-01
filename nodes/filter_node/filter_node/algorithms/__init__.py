"""Algorithm registry for joint filtering. Use get_algorithm(name) to obtain a filter factory."""

from .kalman import KalmanFilterAlgorithm

ALGORITHMS: dict[str, type] = {
    "kalman": KalmanFilterAlgorithm,
}


def get_algorithm(name: str) -> type:
    """Return algorithm class for the given name.

    Args:
        name: Algorithm key (e.g. 'kalman').

    Returns:
        type: Algorithm class (implements FilterAlgorithm interface).

    Raises:
        KeyError: If name is not registered.
    """
    key = (name or "kalman").strip().lower()
    if key not in ALGORITHMS:
        raise KeyError(f"Unknown algorithm {name!r}; available: {list(ALGORITHMS)}")
    return ALGORITHMS[key]
