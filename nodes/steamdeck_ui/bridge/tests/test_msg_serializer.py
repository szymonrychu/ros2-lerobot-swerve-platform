"""Tests for msg_serializer — ROS2 message to JSON dict conversion."""

from __future__ import annotations

import array

import pytest
from bridge.msg_serializer import _bytes_per_pixel, _serialize_value, extractField_from_dict


class TestBytesPerPixel:
    def test_rgb8(self) -> None:
        assert _bytes_per_pixel("rgb8") == 3

    def test_mono8(self) -> None:
        assert _bytes_per_pixel("mono8") == 1

    def test_rgba8(self) -> None:
        assert _bytes_per_pixel("rgba8") == 4

    def test_mono16(self) -> None:
        assert _bytes_per_pixel("mono16") == 2

    def test_rgb16(self) -> None:
        assert _bytes_per_pixel("rgb16") == 2


class TestSerializeValue:
    def test_array_array_to_list(self) -> None:
        assert _serialize_value(array.array("d", [1.0, 2.0, 3.0])) == [1.0, 2.0, 3.0]

    def test_bytes_to_list(self) -> None:
        assert _serialize_value(b"\x01\x02") == [1, 2]

    def test_primitives_passthrough(self) -> None:
        assert _serialize_value(1.5) == 1.5
        assert _serialize_value(True) is True
        assert _serialize_value("hi") == "hi"
        assert _serialize_value(None) is None

    def test_nested_list(self) -> None:
        assert _serialize_value([1, 2, array.array("d", [3.0])]) == [1, 2, [3.0]]


class TestExtractFieldFromDict:
    def test_simple_key(self) -> None:
        assert extractField_from_dict({"x": 1.0}, "x") == pytest.approx(1.0)

    def test_nested_dot(self) -> None:
        assert extractField_from_dict({"a": {"b": 2.5}}, "a.b") == pytest.approx(2.5)

    def test_array_index(self) -> None:
        assert extractField_from_dict({"pos": [0.1, 0.2, 0.3]}, "pos[1]") == pytest.approx(0.2)

    def test_missing_key_returns_none(self) -> None:
        assert extractField_from_dict({}, "missing") is None

    def test_non_numeric_returns_none(self) -> None:
        assert extractField_from_dict({"s": "text"}, "s") is None
