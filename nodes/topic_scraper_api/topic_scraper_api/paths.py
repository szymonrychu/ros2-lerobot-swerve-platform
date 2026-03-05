"""Topic path helpers for topic_scraper_api."""

TOPIC_PREFIX = "/topics"
STREAM_PREFIX = "/streams"
PREVIEW_PREFIX = "/previews"


def normalize_topic(topic: str) -> str:
    """Normalize topic names for consistent endpoint mapping.

    Args:
        topic: Raw topic string.

    Returns:
        str: Normalized topic path with leading slash and no trailing slash.
    """

    value = (topic or "").strip()
    if not value:
        return ""
    if not value.startswith("/"):
        value = f"/{value}"
    while len(value) > 1 and value.endswith("/"):
        value = value[:-1]
    return value


def topic_to_endpoint(topic: str) -> str:
    """Map ROS topic path to HTTP endpoint path.

    Args:
        topic: Topic path like `/follower/joint_commands`.

    Returns:
        str: Endpoint path like `/topics/follower/joint_commands`.
    """

    normalized = normalize_topic(topic)
    if not normalized or normalized == "/":
        return TOPIC_PREFIX
    return f"{TOPIC_PREFIX}{normalized}"


def endpoint_to_topic(endpoint: str) -> str:
    """Map endpoint path back to ROS topic path.

    Args:
        endpoint: Endpoint path that starts with `/topics`.

    Returns:
        str: Topic path (normalized), or empty string when invalid.
    """

    value = (endpoint or "").strip()
    if not value.startswith(TOPIC_PREFIX):
        return ""
    suffix = value.removeprefix(TOPIC_PREFIX)
    return normalize_topic(suffix)


def topic_to_stream_endpoint(topic: str) -> str:
    """Map ROS topic path to HTTP stream endpoint (HTML viewer)."""
    normalized = normalize_topic(topic)
    if not normalized or normalized == "/":
        return STREAM_PREFIX
    return f"{STREAM_PREFIX}{normalized}"


def topic_to_preview_endpoint(topic: str) -> str:
    """Map ROS topic path to HTTP preview endpoint (HTML viewer)."""
    normalized = normalize_topic(topic)
    if not normalized or normalized == "/":
        return PREVIEW_PREFIX
    return f"{PREVIEW_PREFIX}{normalized}"


def stream_endpoint_to_topic(endpoint: str) -> str:
    """Map stream endpoint path back to ROS topic path."""
    value = (endpoint or "").strip()
    if not value.startswith(STREAM_PREFIX):
        return ""
    suffix = value.removeprefix(STREAM_PREFIX)
    if suffix.endswith("/mjpg"):
        suffix = suffix[:-5].rstrip("/") or "/"
    return normalize_topic(suffix) if suffix else ""


def preview_endpoint_to_topic(endpoint: str) -> str:
    """Map preview endpoint path back to ROS topic path."""
    value = (endpoint or "").strip()
    if not value.startswith(PREVIEW_PREFIX):
        return ""
    suffix = value.removeprefix(PREVIEW_PREFIX)
    if suffix.endswith("/image.jpg"):
        suffix = suffix[:-10].rstrip("/") or "/"
    return normalize_topic(suffix) if suffix else ""
