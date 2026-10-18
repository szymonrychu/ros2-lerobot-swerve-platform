"""Tests for the map_manager relocalise command and result messages."""

import json

from map_guard.relocalise import RelocaliseResult, parse_result, relocalise_command


def test_command_is_a_relocalise_with_discard_on_fail() -> None:
    payload = json.loads(relocalise_command("map_guard-3", 1700.5))
    assert payload == {"op": "relocalise", "request_id": "map_guard-3", "since": 1700.5, "discard_on_fail": True}


def test_parse_localised_result() -> None:
    text = json.dumps({"request_id": "map_guard-1", "ok": True, "message": "x", "data": {"state": "localised"}})
    assert parse_result(text) == RelocaliseResult("map_guard-1", True, "localised", 0)


def test_parse_failed_result_carries_discarded_pois() -> None:
    text = json.dumps(
        {
            "request_id": "map_guard-2",
            "ok": False,
            "message": "x",
            "data": {"state": "not_localised", "discarded_pois": 4},
        }
    )
    assert parse_result(text) == RelocaliseResult("map_guard-2", False, "not_localised", 4)


def test_parse_result_without_data() -> None:
    assert parse_result('{"request_id": "a", "ok": false, "message": "bad"}') == RelocaliseResult("a", False, "", 0)


def test_parse_garbage_is_none() -> None:
    assert parse_result("not json") is None
    assert parse_result('{"ok": true}') is None
