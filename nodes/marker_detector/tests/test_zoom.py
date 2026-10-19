"""Tests for the zoom planner (no ROS)."""

from marker_detector.config import ZoomConfig
from marker_detector.zoom import ZoomPlanner, ZoomTarget

SMALL = ZoomTarget(u=0.1, v=0.1, module_px=3.0)
OTHER = ZoomTarget(u=0.9, v=0.9, module_px=3.0)
SAME_REGION = ZoomTarget(u=0.11, v=0.12, module_px=3.0)


def make(**kw: object) -> ZoomPlanner:
    cfg = ZoomConfig(auto_zoom=True, **kw)
    return ZoomPlanner(cfg, clock=lambda: 0.0)


def test_default_never_proposes() -> None:
    planner = ZoomPlanner(ZoomConfig(), clock=lambda: 0.0)
    assert planner.propose([SMALL], 0.0) is None


def test_one_proposal_then_none_until_result() -> None:
    p = make()
    t = p.propose([SMALL], 100.0)
    assert t is not None
    p.sent("r1", t, 100.0)
    assert p.propose([OTHER], 200.0) is None
    p.on_result({"request_id": "r1", "status": "busy"}, 200.0)
    assert p.propose([OTHER], 201.0) is OTHER


def test_two_proposals_5s_apart_give_one_request() -> None:
    p = make()
    t = p.propose([SMALL], 100.0)
    p.sent("r1", t, 100.0)
    p.on_result({"request_id": "r1", "status": "timeout"}, 101.0)
    assert p.propose([OTHER], 105.0) is None
    assert p.propose([OTHER], 110.0) is OTHER


def test_region_cooldown() -> None:
    p = make(min_interval_s=1.0, cooldown_s=30.0)
    p.sent("r1", SMALL, 100.0)
    p.on_result({"request_id": "r1", "status": "camera_error"}, 101.0)
    assert p.propose([SAME_REGION], 120.0) is None
    assert p.propose([SAME_REGION], 131.0) is SAME_REGION


def test_prefers_undecoded_then_smallest_module() -> None:
    p = make()
    a = ZoomTarget(0.1, 0.1, 5.0)
    b = ZoomTarget(0.5, 0.5, 2.0)
    q = ZoomTarget(0.9, 0.1, None)
    assert p.propose([a, b], 0.0) is b
    assert p.propose([a, b, q], 0.0) is q


def test_zoom_factor_reaches_target() -> None:
    assert ZoomTarget(0.5, 0.5, 6.0).zoom_for(12.0) == 2
    assert ZoomTarget(0.5, 0.5, 3.0).zoom_for(12.0) == 4
    assert ZoomTarget(0.5, 0.5, 1.0).zoom_for(12.0) == 8
    assert ZoomTarget(0.5, 0.5, None).zoom_for(12.0) == 4


def test_failed_statuses_give_no_frame() -> None:
    for status in ("busy", "timeout", "camera_error"):
        p = make()
        p.sent("r1", SMALL, 0.0)
        assert p.on_result({"request_id": "r1", "status": status, "stamp_ns": 5}, 1.0) is None
        assert p.on_image(5, b"jpg") is None


def test_result_then_image_pairs() -> None:
    p = make()
    p.sent("r1", SMALL, 0.0)
    pending = p.on_result({"request_id": "r1", "status": "ok", "stamp_ns": 7}, 1.0)
    assert pending is not None and pending.frame is None
    frame = p.on_image(7, b"jpg")
    assert frame is not None and frame.jpeg == b"jpg" and frame.request_id == "r1"
    assert p.propose([OTHER], 50.0) is OTHER


def test_image_then_result_pairs() -> None:
    p = make()
    p.sent("r1", SMALL, 0.0)
    assert p.on_image(7, b"jpg") is None
    pending = p.on_result({"request_id": "r1", "status": "af_failed", "stamp_ns": 7}, 1.0)
    assert pending is not None and pending.frame is not None
    assert pending.frame.jpeg == b"jpg"


def test_stale_request_expires() -> None:
    p = make(timeout_s=8.0)
    p.sent("r1", SMALL, 0.0)
    p.expire(5.0)
    assert p.outstanding is not None
    p.expire(9.0)
    assert p.outstanding is None
    assert p.propose([OTHER], 20.0) is OTHER


def test_external_request_paired_when_process_stills() -> None:
    p = ZoomPlanner(ZoomConfig(process_stills=True), clock=lambda: 0.0)
    pending = p.on_result({"request_id": "ext", "status": "ok", "stamp_ns": 3}, 1.0)
    assert pending is not None and pending.target is None
    frame = p.on_image(3, b"x")
    assert frame is not None and frame.request_id == "ext"


def test_external_request_ignored_when_not_process_stills() -> None:
    p = ZoomPlanner(ZoomConfig(process_stills=False), clock=lambda: 0.0)
    assert p.on_result({"request_id": "ext", "status": "ok", "stamp_ns": 3}, 1.0) is None
    assert p.on_image(3, b"x") is None


def test_own_request_paired_when_not_process_stills() -> None:
    p = ZoomPlanner(ZoomConfig(process_stills=False, auto_zoom=True), clock=lambda: 0.0)
    p.sent("r1", SMALL, 0.0)
    assert p.on_result({"request_id": "r1", "status": "ok", "stamp_ns": 3}, 1.0) is not None
    assert p.on_image(3, b"x") is not None
