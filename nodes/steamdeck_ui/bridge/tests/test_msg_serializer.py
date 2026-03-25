"""Tests for msg_serializer — ROS2 message to JSON dict conversion."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest
from bridge.msg_serializer import _bytes_per_pixel, _serialize_value, extractField_from_dict


# Helper: build a minimal mock ROS2 message
def _make_msg(**fields: Any) -> MagicMock:
    msg = MagicMock()
    msg.get_fields_and_field_types.return_value = {k: "" for k in fields}
    for k, v in fields.items():
        setattr(msg, k, v)
    return msg


class TestSerializeValue:
    def test_int(self) -> None:
        assert _serialize_value(42) == 42

    def test_float(self) -> None:
        assert _serialize_value(3.14) == pytest.approx(3.14)

    def test_bool(self) -> None:
        assert _serialize_value(True) is True

    def test_string(self) -> None:
        assert _serialize_value("hello") == "hello"

    def test_bytes_base64(self) -> None:
        import base64

        result = _serialize_value(b"\x00\x01\x02")
        assert result == base64.b64encode(b"\x00\x01\x02").decode("ascii")

    def test_list_of_ints(self) -> None:
        assert _serialize_value([1, 2, 3]) == [1, 2, 3]

    def test_nested_list(self) -> None:
        assert _serialize_value([[1, 2], [3, 4]]) == [[1, 2], [3, 4]]


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
