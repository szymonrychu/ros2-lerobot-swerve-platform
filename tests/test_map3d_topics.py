"""Map3d topic contract (D2, D3): the shipped yaml loads, every rule rejects a crafted bad document."""

import copy
import importlib.util
import sys
from pathlib import Path
from typing import Any

import pytest
import yaml
from pydantic import ValidationError

from .ros_stubs import install_ros_stubs

NODE_DIR = Path(__file__).resolve().parent.parent / "nodes" / "rtabmap_mapping"
sys.path.insert(0, str(NODE_DIR))

from map3d_topics import Map3dTopics, estimated_mbit_s, load_map3d_topics  # noqa: E402

YAML_PATH = NODE_DIR / "config" / "map3d_topics.yaml"
LAUNCH_FILE = NODE_DIR / "launch" / "rtabmap_mapping.launch.py"
LAUNCH_STUBS = ("launch", "launch.actions", "launch_ros", "launch_ros.actions", "launch_ros.descriptions")
HEAD_RGBD_SOURCES = {
    "/head_rgbd/rgb/image_raw/compressed",
    "/head_rgbd/rgb/camera_info",
    "/head_rgbd/depth/image_raw",
    "/head_rgbd/depth/camera_info",
}


def raw() -> dict[str, Any]:
    """Return a fresh dict of the shipped yaml.

    Returns:
        The parsed document, safe to mutate.
    """
    return copy.deepcopy(yaml.safe_load(YAML_PATH.read_text()))


def uplink(doc: dict[str, Any], source: str) -> dict[str, Any]:
    """Find an uplink entry by source topic.

    Args:
        doc: Parsed document.
        source: Source topic.

    Returns:
        The entry dict.
    """
    return next(s for s in doc["uplink"] if s["source"] == source)


def test_shipped_yaml_loads() -> None:
    topics = load_map3d_topics()
    assert isinstance(topics, Map3dTopics)
    assert load_map3d_topics(YAML_PATH) == topics
    assert topics.frames.map3d_frame == "rtabmap_map"
    assert topics.frames.map_frame == "map"


def test_contract_frames_are_the_d435() -> None:
    topics = load_map3d_topics()
    assert topics.frames.rgb_frame == "d435_color_optical_frame"
    assert topics.frames.depth_frame == "d435_depth_optical_frame"
    text = YAML_PATH.read_text()
    assert "640x360" in text
    assert "320x180" in text
    for stale in ("576x324", "288x162", "head_camera_optical_frame", "tof_optical_frame"):
        assert stale not in text


def load_launch() -> Any:
    """Import the rtabmap_mapping launch file with ROS launch stubs.

    Returns:
        The launch module.
    """
    install_ros_stubs(*LAUNCH_STUBS)
    spec = importlib.util.spec_from_file_location("rtabmap_mapping_launch_sources", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_rtabmap_launch_reads_sources_from_the_contract() -> None:
    assert "/head_rgbd/" not in LAUNCH_FILE.read_text()
    launch = load_launch()
    topics = load_map3d_topics()
    rgb, info, depth = launch.image_sources(topics)
    assert (rgb.source, info.source, depth.source) == (
        "/head_rgbd/rgb/image_raw/compressed",
        "/head_rgbd/rgb/camera_info",
        "/head_rgbd/depth/image_raw",
    )
    # Re-pointing the yaml re-points the launch: no source topic lives in the launch module.
    doc = raw()
    uplink(doc, "/head_rgbd/rgb/image_raw/compressed").update(
        source="/cam/color/compressed", dest="/uplink/cam/color/compressed"
    )
    uplink(doc, "/head_rgbd/rgb/camera_info").update(source="/cam/color/info", dest="/uplink/cam/color/info")
    uplink(doc, "/head_rgbd/depth/image_raw").update(source="/cam/depth", dest="/uplink/cam/depth/compressedDepth")
    moved = Map3dTopics(**doc)
    assert [s.source for s in launch.image_sources(moved)] == ["/cam/color/compressed", "/cam/color/info", "/cam/depth"]
    params = launch.RtabmapMappingConfig(database_dir="/tmp/db", rtabmap={"ros__parameters": {}})
    remaps = dict(launch.build_container(moved, params).components[0].remappings)
    assert remaps == {
        "rgb/image": "/uplink/cam/color",
        "depth/image": "/uplink/cam/depth",
        "rgb/camera_info": "/uplink/cam/color/info",
    }


def test_unknown_keys_forbidden() -> None:
    doc = raw()
    doc["bogus"] = 1
    with pytest.raises(ValidationError, match="bogus"):
        Map3dTopics(**doc)
    doc = raw()
    doc["uplink"][0]["extra"] = 1
    with pytest.raises(ValidationError, match="extra"):
        Map3dTopics(**doc)


def test_uplink_dest_prefix() -> None:
    doc = raw()
    uplink(doc, "/scan_filtered")["dest"] = "/scan_up"
    with pytest.raises(ValidationError, match="dest.*/uplink/"):
        Map3dTopics(**doc)


def test_downlink_dest_prefix() -> None:
    doc = raw()
    doc["downlink"][0]["dest"] = "/cloud"
    with pytest.raises(ValidationError, match="dest.*/map3d/"):
        Map3dTopics(**doc)


def test_dest_equals_source_rejected() -> None:
    doc = raw()
    doc["downlink"][1]["source"] = doc["downlink"][1]["dest"]
    with pytest.raises(ValidationError, match="dest.*source"):
        Map3dTopics(**doc)


def test_dest_equals_other_stream_source_rejected() -> None:
    doc = raw()
    doc["downlink"][0]["dest"] = "/map3d/command"
    doc["uplink"][-1]["source"] = "/map3d/command"
    with pytest.raises(ValidationError, match="dest.*source"):
        Map3dTopics(**doc)


def test_dests_unique() -> None:
    doc = raw()
    uplink(doc, "/scan_filtered")["dest"] = uplink(doc, "/tf")["dest"]
    with pytest.raises(ValidationError, match="dest.*unique"):
        Map3dTopics(**doc)


@pytest.mark.parametrize("bad", [0, -1.0])
def test_rate_positive(bad: float) -> None:
    doc = raw()
    doc["downlink"][0]["rate_hz"] = bad
    with pytest.raises(ValidationError, match="rate_hz"):
        Map3dTopics(**doc)


@pytest.mark.parametrize("bad", [0, -5])
def test_budget_bytes_positive(bad: int) -> None:
    doc = raw()
    uplink(doc, "/scan_filtered")["budget_bytes"] = bad
    with pytest.raises(ValidationError, match="budget_bytes"):
        Map3dTopics(**doc)


def test_null_rate_requires_expected_rate() -> None:
    doc = raw()
    del uplink(doc, "/scan_filtered")["expected_rate_hz"]
    with pytest.raises(ValidationError, match="expected_rate_hz"):
        Map3dTopics(**doc)


def test_tf_static_must_be_transient_local() -> None:
    doc = raw()
    uplink(doc, "/tf_static")["qos"]["durability"] = "volatile"
    with pytest.raises(ValidationError, match="tf_static.*transient_local"):
        Map3dTopics(**doc)


def test_large_stream_with_two_readers_rejected() -> None:
    doc = raw()
    uplink(doc, "/scan_filtered")["server_readers"] = ["rtabmap_mapping", "map3d_manager"]
    with pytest.raises(ValidationError, match="server_readers"):
        Map3dTopics(**doc)


def test_small_stream_may_have_two_readers() -> None:
    topics = load_map3d_topics()
    infos = {s.source: s for s in topics.uplink if s.source.endswith("camera_info")}
    assert len(infos) == 2
    for s in infos.values():
        assert s.small
    assert set(infos["/head_rgbd/rgb/camera_info"].server_readers) == {"rtabmap_mapping", "map3d_manager"}
    # rgbd_sync takes the intrinsics of the registered depth from the RGB camera_info (D5): only map3d_manager reads it.
    assert infos["/head_rgbd/depth/camera_info"].server_readers == ["map3d_manager"]


def test_small_flag_must_be_true_for_small_budget() -> None:
    doc = raw()
    uplink(doc, "/scan_filtered")["small"] = True
    with pytest.raises(ValidationError, match="small"):
        Map3dTopics(**doc)


def test_every_uplink_has_a_reader() -> None:
    doc = raw()
    uplink(doc, "/scan_filtered")["server_readers"] = []
    with pytest.raises(ValidationError, match="server_readers"):
        Map3dTopics(**doc)


def test_bandwidth_within_budget() -> None:
    topics = load_map3d_topics()
    up = estimated_mbit_s(topics.uplink, {})
    down = estimated_mbit_s(topics.downlink, {})
    assert 0 < up <= topics.budget.uplink_mbit_s_max
    assert 0 < down <= topics.budget.downlink_mbit_s_max


def test_estimated_mbit_s_arithmetic() -> None:
    topics = load_map3d_topics()
    scan = [s for s in topics.uplink if s.source == "/scan_filtered"]
    assert estimated_mbit_s(scan, {"/scan_filtered": 10.0}) == pytest.approx(12000 * 10 * 8 / 1e6)


def test_head_rgb_uplink_is_mapping_stream_passthrough() -> None:
    topics = load_map3d_topics()
    sources = {s.source for s in topics.uplink}
    assert "/head_rgbd/rgb/image_raw/compressed" in sources
    for s in sources:
        assert not s.startswith("/head_camera/")
    assert "/head_camera/image_raw" not in sources
    assert not any("/head_camera" in s.dest for s in topics.uplink)


def test_depth_uplink_is_registered_depth_not_tof() -> None:
    topics = load_map3d_topics()
    depth = [s for s in topics.uplink if s.source == "/head_rgbd/depth/image_raw"]
    assert len(depth) == 1
    assert depth[0].transport == "compressedDepth"
    assert not any(s.source.startswith("/tof/") for s in topics.uplink)


def test_head_rgbd_entries_unthrottled() -> None:
    topics = load_map3d_topics()
    entries = [s for s in topics.uplink if s.source in HEAD_RGBD_SOURCES]
    assert {s.source for s in entries} == HEAD_RGBD_SOURCES
    for s in entries:
        assert s.rate_hz is None
        assert s.expected_rate_hz == 2.0


def test_documented_values() -> None:
    topics = load_map3d_topics()
    by = {s.source: s for s in topics.uplink}
    assert by["/head_rgbd/rgb/image_raw/compressed"].budget_bytes == 50000
    assert by["/head_rgbd/depth/image_raw"].budget_bytes == 55000
    assert by["/head_rgbd/depth/image_raw"].dest == "/uplink/head_rgbd/depth/image_raw/compressedDepth"
    assert by["/scan_filtered"].budget_bytes == 12000
    assert by["/tf"].budget_bytes == 200
    assert by["/tf"].expected_rate_hz == 100
    assert by["/tf_static"].qos.depth == 100
    cloud = next(s for s in topics.downlink if s.source == "/rtabmap/cloud_map")
    assert (cloud.dest, cloud.rate_hz, cloud.lazy, cloud.budget_bytes) == ("/map3d/cloud", 0.1, True, 1500000)
    assert topics.budget.uplink_mbit_s_max == 8.0
    assert topics.budget.downlink_mbit_s_max == 2.0
