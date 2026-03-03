from topic_scraper_api.paths import endpoint_to_topic, normalize_topic, topic_to_endpoint


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
