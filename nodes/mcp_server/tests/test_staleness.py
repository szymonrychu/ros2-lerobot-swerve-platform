"""Tests for mcp_server.staleness (data age bookkeeping)."""

import pytest

from mcp_server.staleness import Stamped, age_s, is_fresh


def test_age_and_freshness() -> None:
    assert age_s(10.0, 10.25) == pytest.approx(0.25)
    assert is_fresh(10.0, 10.25, 0.3)
    assert not is_fresh(10.0, 10.31, 0.3)


def test_missing_stamp_is_never_fresh() -> None:
    assert age_s(None, 5.0) is None
    assert not is_fresh(None, 5.0, 100.0)


def test_stamped_value_freshness() -> None:
    s = Stamped(value={"x": 1}, stamp=1.0)
    assert s.fresh(1.2, 0.5)
    assert not s.fresh(2.0, 0.5)
    assert s.age(1.5) == pytest.approx(0.5)
