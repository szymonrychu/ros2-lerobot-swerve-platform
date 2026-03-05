from topic_scraper_api.paths import (
    endpoint_to_topic,
    normalize_topic,
    preview_endpoint_to_topic,
    stream_endpoint_to_topic,
    topic_to_endpoint,
    topic_to_preview_endpoint,
    topic_to_stream_endpoint,
)


def test_normalize_topic() -> None:
    assert normalize_topic("leader/joint_states") == "/leader/joint_states"
    assert normalize_topic("/leader/joint_states/") == "/leader/joint_states"
    assert normalize_topic("") == ""


def test_topic_to_endpoint() -> None:
    assert topic_to_endpoint("/leader/joint_states") == "/topics/leader/joint_states"
    assert topic_to_endpoint("leader/joint_states") == "/topics/leader/joint_states"
    assert topic_to_endpoint("/") == "/topics"


def test_endpoint_to_topic() -> None:
    assert endpoint_to_topic("/topics/leader/joint_states") == "/leader/joint_states"
    assert endpoint_to_topic("/topics") == ""
    assert endpoint_to_topic("/bad/leader/joint_states") == ""


def test_topic_to_stream_endpoint() -> None:
    assert topic_to_stream_endpoint("/camera_0/image_raw") == "/streams/camera_0/image_raw"
    assert topic_to_stream_endpoint("camera_0/image_raw") == "/streams/camera_0/image_raw"


def test_topic_to_preview_endpoint() -> None:
    assert topic_to_preview_endpoint("/camera_0/image_raw") == "/previews/camera_0/image_raw"
    assert topic_to_preview_endpoint("camera_0/image_raw") == "/previews/camera_0/image_raw"


def test_stream_endpoint_to_topic() -> None:
    assert stream_endpoint_to_topic("/streams/camera_0/image_raw") == "/camera_0/image_raw"
    assert stream_endpoint_to_topic("/streams/camera_0/image_raw/mjpg") == "/camera_0/image_raw"
    assert stream_endpoint_to_topic("/streams") == ""
    assert stream_endpoint_to_topic("/other/foo") == ""


def test_preview_endpoint_to_topic() -> None:
    assert preview_endpoint_to_topic("/previews/camera_0/image_raw") == "/camera_0/image_raw"
    assert preview_endpoint_to_topic("/previews/camera_0/image_raw/image.jpg") == "/camera_0/image_raw"
    assert preview_endpoint_to_topic("/previews") == ""
    assert preview_endpoint_to_topic("/other/foo") == ""
