from scripts.topic_scraper_collect import build_record, parse_selector, parse_source, topic_endpoint


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
