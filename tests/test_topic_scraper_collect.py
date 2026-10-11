import json
import socket

from scripts.topic_scraper_collect import (
    build_record,
    format_record,
    http_get_json,
    parse_json,
    parse_selector,
    parse_source,
    topic_endpoint,
)


def test_parse_source() -> None:
    src = parse_source("client=http://127.0.0.1:18100")
    assert src.name == "client"
    assert src.base_url == "http://127.0.0.1:18100"


def test_parse_selector() -> None:
    selector = parse_selector("/leader/joint_states:.position[5]")
    assert selector.topic == "/leader/joint_states"
    assert selector.jq_filter == ".position[5]"


def test_topic_endpoint() -> None:
    assert topic_endpoint("http://localhost:18100", "/leader/joint_states") == (
        "http://localhost:18100/topics/leader/joint_states"
    )


def test_build_record() -> None:
    payload = {
        "received_at_ns": 10,
        "header_stamp_ns": 9,
        "sample_seq": 3,
    }
    record = build_record("client", "/leader/joint_states", payload, 0.2)
    assert record["source"] == "client"
    assert record["topic"] == "/leader/joint_states"
    assert record["received_at_ns"] == 10
    assert record["header_stamp_ns"] == 9
    assert record["sample_seq"] == 3
    assert record["value"] == 0.2


def test_connection_refused_prints_the_enable_hint(capsys) -> None:
    """topic_scraper_api is disabled by default: a refused connection tells how to enable it."""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    assert http_get_json(f"http://127.0.0.1:{port}/topics") is None
    err = capsys.readouterr().err
    assert "disabled by default" in err and "enabled: true" in err


def test_parse_json_maps_null_and_legacy_non_finite_constants_to_none() -> None:
    """null parses to None; NaN/Infinity from an older scraper also become None (never re-emitted)."""
    assert parse_json('{"a": null, "b": [NaN, Infinity, -Infinity, 1.5]}') == {"a": None, "b": [None, None, None, 1.5]}


def test_record_with_null_value_is_strict_json() -> None:
    """A collected record whose value holds null serializes without NaN/Infinity."""
    record = build_record("client", "/t", {}, parse_json("[null, 0.1]"))
    text = format_record(record)
    assert text == json.dumps(record, separators=(",", ":"), allow_nan=False)
    assert "NaN" not in text
