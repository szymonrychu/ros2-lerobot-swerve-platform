"""Prometheus metrics of the UVC camera bridge, registered once in the default registry (no ROS/OpenCV deps)."""

from prometheus_client import Counter, Histogram

ENCODE_BUCKETS = (0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2)

FRAMES_PUBLISHED = Counter("camera_frames_published_total", "Frames published on the raw image topic")
FRAMES_DROPPED = Counter("camera_frames_dropped_total", "Captured frames not published, by reason", ["reason"])
ENCODE_SECONDS = Histogram("camera_encode_seconds", "JPEG encode time per published frame", buckets=ENCODE_BUCKETS)
REOPENS = Counter("camera_reopens_total", "Times the capture device was closed and reopened after read failures")
