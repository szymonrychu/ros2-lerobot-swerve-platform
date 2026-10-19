"""Prometheus metrics of the marker detector (default registry, created once at import)."""

from prometheus_client import Counter, Histogram

FRAMES_PROCESSED = Counter("marker_frames_processed_total", "Camera frames run through the detector", ["source"])
DETECTIONS_ACCEPTED = Counter("marker_detections_accepted_total", "Marker detections that became POI commands")
REJECTIONS = Counter("marker_rejections_total", "Rejected marker candidates by reason", ["reason"])
ZOOM_REQUESTS = Counter("marker_zoom_requests_total", "Zoom stills requested, by sent or result status", ["status"])
PROCESS_SECONDS = Histogram("marker_process_seconds", "Time to process one frame", ["source"])
