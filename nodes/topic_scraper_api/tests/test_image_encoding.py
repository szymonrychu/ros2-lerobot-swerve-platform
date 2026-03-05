"""Tests for image_encoding module."""

from array import array

from topic_scraper_api.image_encoding import (
    IMAGE_TYPE_COMPRESSED,
    IMAGE_TYPE_RAW,
    image_message_to_jpeg_bytes,
    is_image_type,
)


def test_is_image_type() -> None:
    assert is_image_type(IMAGE_TYPE_RAW) is True
    assert is_image_type(IMAGE_TYPE_COMPRESSED) is True
    assert is_image_type("sensor_msgs/msg/JointState") is False
    assert is_image_type("") is False


def test_compressed_image_jpeg_passthrough() -> None:
    """CompressedImage with format jpeg returns data as-is."""
    jpeg_data = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xd9"

    class FakeCompressed:
        format = "jpeg"
        data = array("B", jpeg_data)

    result = image_message_to_jpeg_bytes(FakeCompressed(), IMAGE_TYPE_COMPRESSED)
    assert result == jpeg_data


def test_compressed_image_unknown_format_returns_none() -> None:
    """CompressedImage with non-jpeg format returns None."""

    class FakeCompressed:
        format = "png"
        data = array("B", b"fake")

    assert image_message_to_jpeg_bytes(FakeCompressed(), IMAGE_TYPE_COMPRESSED) is None


def test_raw_image_bgr8_to_jpeg() -> None:
    """Raw Image with bgr8 encoding produces JPEG bytes."""
    # 2x2 BGR image: 12 bytes
    image_data = array("B", [255, 0, 0, 0, 255, 0, 0, 0, 255, 128, 128, 128])

    class FakeImage:
        height = 2
        width = 2
        encoding = "bgr8"
        step = 6
        data = image_data

    result = image_message_to_jpeg_bytes(FakeImage(), IMAGE_TYPE_RAW)
    assert result is not None
    assert result[:2] == b"\xff\xd8"
    assert result[-2:] == b"\xff\xd9"


def test_raw_image_mono8_to_jpeg() -> None:
    """Raw Image with mono8 encoding produces JPEG bytes."""
    image_data = array("B", [0, 128, 255, 64])

    class FakeImage:
        height = 2
        width = 2
        encoding = "mono8"
        step = 2
        data = image_data

    result = image_message_to_jpeg_bytes(FakeImage(), IMAGE_TYPE_RAW)
    assert result is not None
    assert result[:2] == b"\xff\xd8"


def test_raw_image_unsupported_encoding_returns_none() -> None:
    """Raw Image with unsupported encoding returns None."""
    image_data = array("B", [0] * 24)

    class FakeImage:
        height = 2
        width = 4
        encoding = "mono16"
        step = 8
        data = image_data

    assert image_message_to_jpeg_bytes(FakeImage(), IMAGE_TYPE_RAW) is None
