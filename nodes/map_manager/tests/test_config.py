"""Tests for the map_manager config: the search acceptance keys."""

import pytest
from pydantic import ValidationError

from map_manager.config import MapManagerConfig


def test_min_evaluated_fraction_defaults_when_omitted() -> None:
    assert MapManagerConfig(**{"search": {}}).search.min_evaluated_fraction == 0.6
    assert MapManagerConfig().search.min_evaluated_fraction == 0.6


def test_min_evaluated_fraction_accepts_one() -> None:
    assert MapManagerConfig(search={"min_evaluated_fraction": 1.0}).search.min_evaluated_fraction == 1.0


@pytest.mark.parametrize("value", [0, 0.0, -0.1, 1.01, 2])
def test_min_evaluated_fraction_rejected(value: float) -> None:
    with pytest.raises(ValidationError):
        MapManagerConfig(search={"min_evaluated_fraction": value})
