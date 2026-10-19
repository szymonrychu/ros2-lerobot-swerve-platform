"""rtabmap_mapping launch and parameters (design D1, D5-D9, U1, U2, U6).

No ROS on the dev machine: the launch file is imported with MagicMock stubs for `launch` / `launch_ros`, and the pure
`build_container` is checked directly. The docker probe (scripts/rtabmap-launch-probe.sh) proves the same parameters
load in Jazzy on arm64.
"""

import importlib.util
import os
import re
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
import yaml
from pydantic import ValidationError

from .ros_stubs import install_ros_stubs

REPO_ROOT = Path(__file__).resolve().parent.parent
NODE_DIR = REPO_ROOT / "nodes" / "rtabmap_mapping"
LAUNCH_FILE = NODE_DIR / "launch" / "rtabmap_mapping.launch.py"
PROBE_SCRIPT = REPO_ROOT / "scripts" / "rtabmap-launch-probe.sh"
PARAMS_FILE = NODE_DIR / "config" / "rtabmap_params.yaml"
README = NODE_DIR / "README.md"
LAUNCH_STUBS = ("launch", "launch.actions", "launch_ros", "launch_ros.actions", "launch_ros.descriptions")
BEST_EFFORT = 2
RELIABLE = 1
QOS_VALUES = {"reliable": RELIABLE, "best_effort": BEST_EFFORT}
SYNC_PLUGIN = "rtabmap_sync::RGBDSync"
CORE_PLUGIN = "rtabmap_slam::CoreWrapper"

# Every RTAB-Map value the design fixes (D6, D7, D8). Core parameters are strings, wrapper parameters typed.
EXPECTED_RTABMAP = {
    # frames (D7)
    "frame_id": "base_link",
    "odom_frame_id": "map",
    "map_frame_id": "rtabmap_map",
    "publish_tf": False,
    "wait_for_transform": 0.2,
    "odom_tf_linear_variance": 0.001,
    "odom_tf_angular_variance": 0.001,
    # no own links (D7)
    "Kp/MaxFeatures": "-1",
    "RGBD/ProximityBySpace": "false",
    "RGBD/OptimizeFromGraphEnd": "true",
    "RGBD/NeighborLinkRefining": "false",
    "Reg/Force3DoF": "true",
    # node creation (D6)
    "RGBD/LinearUpdate": "0.1",
    "RGBD/AngularUpdate": "0.1",
    "Rtabmap/DetectionRate": "1",
    # grid: depth only (D6, D13)
    "Grid/Sensor": "1",
    "Grid/3D": "true",
    "Grid/RangeMax": "4.0",
    "Grid/CellSize": "0.05",
    "Grid/DepthDecimation": "2",
    "Grid/MaxObstacleHeight": "2.0",
    "Grid/RayTracing": "false",
    # memory (D8)
    "Rtabmap/MemoryThr": "400",
    "Rtabmap/TimeThr": "0",
    "Mem/STMSize": "10",
    "Mem/ImagePostDecimation": "2",
    "Mem/BinDataKept": "true",
    "Mem/RawDescriptorsKept": "false",
    "Rtabmap/ImageBufferSize": "1",
    # outputs (D10)
    "cloud_output_voxelized": True,
    "map_always_update": False,
    "latch": False,
}
HEAD_RGB = "/uplink/head_rgbd/rgb/image_raw"
HEAD_DEPTH = "/uplink/head_rgbd/depth/image_raw"
HEAD_RGB_INFO = "/uplink/head_rgbd/rgb/camera_info"
SCAN = "/uplink/scan_filtered"
PARKING_WORDING = re.compile(
    r"for later|later runs|accepted limitation|\bTODO\b|follow-up|out of scope|phase 2|runbook measures", re.IGNORECASE
)
VALUED_LINE = re.compile(r"^\s*[^#\s][^#]*:\s+[^#\s]")  # "key: value", not a section header or a comment


@pytest.fixture(scope="module")
def launch() -> ModuleType:
    """Import the launch file with ROS launch stubs.

    Returns:
        ModuleType: The launch module.
    """
    install_ros_stubs(*LAUNCH_STUBS)
    spec = importlib.util.spec_from_file_location("rtabmap_mapping_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def spec(launch: ModuleType) -> Any:
    """Container spec built from the shipped contract and defaults.

    Args:
        launch: The launch module.

    Returns:
        Any: ContainerSpec.
    """
    config = launch.load_config(PARAMS_FILE, Path("/nonexistent/override.yaml"))
    return launch.build_container(launch.load_map3d_topics(), config)


def component(spec: Any, plugin: str) -> Any:
    """Return the component of a plugin.

    Args:
        spec: ContainerSpec.
        plugin: Plugin class name.

    Returns:
        Any: ComponentSpec.
    """
    return next(c for c in spec.components if c.plugin == plugin)


def contract_qos(launch: ModuleType, dest: str) -> int:
    """RTAB-Map reliability value of a contract dest.

    Args:
        launch: The launch module.
        dest: Uplink dest topic.

    Returns:
        int: 1 reliable, 2 best_effort.
    """
    stream = next(s for s in launch.load_map3d_topics().uplink if s.dest == dest)
    return QOS_VALUES[stream.qos.reliability]


# --- parameters --------------------------------------------------------------------------------------------------


def test_params_yaml_sets_every_design_value() -> None:
    params = yaml.safe_load(PARAMS_FILE.read_text())["rtabmap"]["ros__parameters"]
    for key, value in EXPECTED_RTABMAP.items():
        assert key in params, key
        assert params[key] == value, key
        assert type(params[key]) is type(value), key


def test_every_param_value_is_commented() -> None:
    valued_lines = [line for line in PARAMS_FILE.read_text().splitlines() if VALUED_LINE.match(line)]
    assert len(valued_lines) > len(EXPECTED_RTABMAP)
    for line in valued_lines:
        assert "#" in line, f"value without a why-comment: {line.strip()}"


def test_rtabmap_component_gets_every_design_value(spec: Any) -> None:
    params = component(spec, CORE_PLUGIN).parameters
    for key, value in EXPECTED_RTABMAP.items():
        assert params[key] == value, key


def test_grid_is_depth_only_and_scan_is_still_subscribed(spec: Any) -> None:
    params = component(spec, CORE_PLUGIN).parameters
    assert params["Grid/Sensor"] == "1"
    assert params["subscribe_scan"] is True


def test_no_own_links(spec: Any) -> None:
    params = component(spec, CORE_PLUGIN).parameters
    assert params["Kp/MaxFeatures"] == "-1"
    assert params["RGBD/ProximityBySpace"] == "false"
    assert params["RGBD/NeighborLinkRefining"] == "false"
    assert params["odom_frame_id"] == "map"
    assert params["map_frame_id"] == "rtabmap_map"


# --- container shape ---------------------------------------------------------------------------------------------


def test_one_component_container_with_sync_and_core(spec: Any) -> None:
    assert spec.package == "rclcpp_components"
    assert spec.executable == "component_container"
    assert sorted(c.plugin for c in spec.components) == sorted([SYNC_PLUGIN, CORE_PLUGIN])


def test_no_register_node_loaded(spec: Any) -> None:
    for c in spec.components:
        assert "depth_image_proc" not in c.package
        assert "Register" not in c.plugin


def test_rgbd_sync_exact_sync_on_head_rgbd_pair(spec: Any) -> None:
    sync = component(spec, SYNC_PLUGIN)
    assert sync.package == "rtabmap_sync"
    assert dict(sync.remappings) == {
        "rgb/image": HEAD_RGB,
        "depth/image": HEAD_DEPTH,
        "rgb/camera_info": HEAD_RGB_INFO,
    }
    assert sync.parameters["approx_sync"] is False
    assert sync.parameters["image_transport"] == "compressed"
    # 0.23.13 renamed depth_image_transport to depth_transport (rgbd_sync.cpp warns on the old name).
    assert sync.parameters["depth_transport"] == "compressedDepth"
    assert "depth_image_transport" not in sync.parameters


def test_core_wrapper_inputs(spec: Any) -> None:
    core = component(spec, CORE_PLUGIN)
    assert core.package == "rtabmap_slam"
    assert core.namespace == "rtabmap"
    assert core.name == "rtabmap"
    assert dict(core.remappings)["scan"] == SCAN
    assert core.parameters["subscribe_rgbd"] is True
    assert core.parameters["subscribe_scan"] is True
    assert core.parameters["subscribe_rgb"] is False
    assert core.parameters["subscribe_depth"] is False
    assert core.parameters["approx_sync"] is True  # rgbd pair and scan come from different sensors


def test_services_are_at_the_rtabmap_namespace(spec: Any) -> None:
    # CoreWrapper creates its services under its node name (/rtabmap/rtabmap/backup); map3d_manager calls /rtabmap/*.
    core = component(spec, CORE_PLUGIN)
    assert dict(core.service_remappings) == {
        f"/rtabmap/rtabmap/{srv}": f"/rtabmap/{srv}" for srv in ("backup", "reset", "pause", "resume")
    }
    assert component(spec, SYNC_PLUGIN).service_remappings == []


def test_sync_output_feeds_core(spec: Any) -> None:
    sync = component(spec, SYNC_PLUGIN)
    core = component(spec, CORE_PLUGIN)
    assert sync.namespace == core.namespace  # rgbd_image resolves to the same /rtabmap/rgbd_image


def test_qos_follows_the_contract(launch: ModuleType, spec: Any) -> None:
    sync = component(spec, SYNC_PLUGIN).parameters
    core = component(spec, CORE_PLUGIN).parameters
    assert sync["qos"] == BEST_EFFORT == contract_qos(launch, HEAD_RGB + "/compressed")
    assert contract_qos(launch, HEAD_DEPTH + "/compressedDepth") == BEST_EFFORT
    assert sync["qos_camera_info"] == contract_qos(launch, HEAD_RGB_INFO)
    assert core["qos_image"] == sync["qos"]  # rgbd_sync publishes rgbd_image with its qos
    # A reliable subscription never matches a best_effort publisher, so the scan QoS is the relay's.
    assert core["qos_scan"] == contract_qos(launch, SCAN)


# --- topics and TF -----------------------------------------------------------------------------------------------


def test_subscribed_topics_equal_contract_readers(launch: ModuleType, spec: Any) -> None:
    topics = launch.load_map3d_topics()
    readers = {s.dest for s in topics.uplink if "rtabmap_mapping" in s.server_readers}
    assert spec.subscribed_topics() == readers


def test_global_tf_remaps(spec: Any) -> None:
    assert spec.ros_arguments == ["-r", "/tf:=/uplink/tf", "-r", "/tf_static:=/uplink/tf_static"]


def test_no_tf_publisher_configured(spec: Any) -> None:
    core = component(spec, CORE_PLUGIN).parameters
    assert core["publish_tf"] is False
    for c in spec.components:
        assert c.plugin in {SYNC_PLUGIN, CORE_PLUGIN}
        assert c.parameters.get("publish_tf", False) is False


def test_override_cannot_enable_tf(launch: ModuleType, tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("rtabmap:\n  ros__parameters:\n    publish_tf: true\n")
    config = launch.load_config(PARAMS_FILE, override)
    with pytest.raises(ValueError, match="publish_tf"):
        launch.build_container(launch.load_map3d_topics(), config)


def test_override_cannot_change_fixed_inputs(launch: ModuleType, tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("rgbd_sync:\n  ros__parameters:\n    approx_sync: true\n")
    config = launch.load_config(PARAMS_FILE, override)
    with pytest.raises(ValueError, match="approx_sync"):
        launch.build_container(launch.load_map3d_topics(), config)


def test_launch_description_starts_one_container_and_no_other_process(
    launch: ModuleType, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    override = tmp_path / "config.yaml"
    override.write_text(f"database_dir: {tmp_path / 'db'}\n")
    monkeypatch.setenv(launch.CONFIG_ENV, str(override))
    actions = sys.modules["launch_ros.actions"]
    descriptions = sys.modules["launch_ros.descriptions"]
    actions.reset_mock()
    descriptions.reset_mock()
    launch.generate_launch_description()
    assert actions.ComposableNodeContainer.call_count == 1
    assert actions.Node.call_count == 0
    kwargs = actions.ComposableNodeContainer.call_args.kwargs
    assert kwargs["executable"] == "component_container"
    assert kwargs["ros_arguments"] == ["-r", "/tf:=/uplink/tf", "-r", "/tf_static:=/uplink/tf_static"]
    assert descriptions.ComposableNode.call_count == 2
    plugins = {c.kwargs["plugin"] for c in descriptions.ComposableNode.call_args_list}
    assert plugins == {SYNC_PLUGIN, CORE_PLUGIN}
    core = next(c.kwargs for c in descriptions.ComposableNode.call_args_list if c.kwargs["plugin"] == CORE_PLUGIN)
    assert ("scan", SCAN) in core["remappings"]
    assert ("/rtabmap/rtabmap/backup", "/rtabmap/backup") in core["remappings"]
    assert core["parameters"][0]["database_path"] == str(tmp_path / "db" / "live.db")
    assert core["parameters"][0]["publish_tf"] is False


# --- database restore (D9) ---------------------------------------------------------------------------------------


def write_db(path: Path, content: str, mtime: float) -> None:
    """Write a fake database file with a given mtime.

    Args:
        path: File path.
        content: File content.
        mtime: Modification time (epoch seconds).
    """
    path.write_text(content)
    os.utime(path, (mtime, mtime))


def test_restore_copies_newer_saved_db(launch: ModuleType, tmp_path: Path) -> None:
    write_db(tmp_path / "live.db", "live", 1000.0)
    write_db(tmp_path / "saved.db", "saved", 2000.0)
    live = launch.restore_saved_database(tmp_path)
    assert live == tmp_path / "live.db"
    assert live.read_text() == "saved"
    assert (tmp_path / "saved.db").read_text() == "saved"


def test_restore_keeps_newer_live_db(launch: ModuleType, tmp_path: Path) -> None:
    write_db(tmp_path / "live.db", "live", 3000.0)
    write_db(tmp_path / "saved.db", "saved", 2000.0)
    live = launch.restore_saved_database(tmp_path)
    assert live.read_text() == "live"
    assert (tmp_path / "saved.db").read_text() == "saved"


def test_restore_copies_saved_when_live_missing(launch: ModuleType, tmp_path: Path) -> None:
    write_db(tmp_path / "saved.db", "saved", 2000.0)
    assert launch.restore_saved_database(tmp_path).read_text() == "saved"
    assert (tmp_path / "saved.db").exists()


def test_restore_without_any_db_starts_new(launch: ModuleType, tmp_path: Path) -> None:
    db_dir = tmp_path / "missing"
    live = launch.restore_saved_database(db_dir)
    assert live == db_dir / "live.db"
    assert db_dir.is_dir()
    assert not live.exists()  # RTAB-Map creates it


def test_database_path_is_live_db_and_never_deleted(launch: ModuleType, tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text(f"database_dir: {tmp_path}\n")
    spec = launch.build_container(launch.load_map3d_topics(), launch.load_config(PARAMS_FILE, override))
    params = component(spec, CORE_PLUGIN).parameters
    assert params["database_path"] == str(tmp_path / "live.db")
    assert params["delete_db_on_start"] is False


def test_override_cannot_delete_db_on_start(launch: ModuleType, tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("rtabmap:\n  ros__parameters:\n    delete_db_on_start: true\n")
    with pytest.raises(ValueError, match="delete_db_on_start"):
        launch.build_container(launch.load_map3d_topics(), launch.load_config(PARAMS_FILE, override))


def test_default_database_dir(launch: ModuleType) -> None:
    config = launch.load_config(PARAMS_FILE, Path("/nonexistent/override.yaml"))
    assert config.database_dir == "/var/lib/ros2/rtabmap"
    assert launch.LIVE_DB == "live.db"
    assert launch.SAVED_DB == "saved.db"


# --- override merge ----------------------------------------------------------------------------------------------


def test_override_merges_key_by_key(launch: ModuleType, tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text(
        "database_dir: /data/rtabmap\n"
        "rtabmap:\n  ros__parameters:\n    Grid/RangeMax: '3.0'\n    Rtabmap/MemoryThr: '200'\n"
        "rgbd_sync:\n  ros__parameters:\n    sync_queue_size: 20\n"
    )
    config = launch.load_config(PARAMS_FILE, override)
    params = config.rtabmap.ros__parameters
    assert config.database_dir == "/data/rtabmap"
    assert params["Grid/RangeMax"] == "3.0"
    assert params["Rtabmap/MemoryThr"] == "200"
    assert params["Grid/Sensor"] == "1"  # untouched defaults survive
    assert params["odom_frame_id"] == "map"
    assert config.rgbd_sync.ros__parameters["sync_queue_size"] == 20


def test_override_ignored_when_missing_or_empty(launch: ModuleType, tmp_path: Path) -> None:
    empty = tmp_path / "config.yaml"
    empty.write_text("")
    defaults = launch.load_config(PARAMS_FILE, tmp_path / "missing.yaml")
    assert launch.load_config(PARAMS_FILE, empty) == defaults
    assert defaults.rtabmap.ros__parameters == yaml.safe_load(PARAMS_FILE.read_text())["rtabmap"]["ros__parameters"]


def test_override_rejects_unknown_top_level_key(launch: ModuleType, tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("rtabmapp:\n  ros__parameters: {}\n")
    with pytest.raises(ValidationError):
        launch.load_config(PARAMS_FILE, override)


def test_config_env_names_and_default(launch: ModuleType) -> None:
    assert launch.CONFIG_ENV == "RTABMAP_MAPPING_CONFIG"
    assert launch.DEFAULT_CONFIG_PATH == "/etc/ros2/rtabmap_mapping/config.yaml"
    assert launch.PARAMS_ENV == "RTABMAP_MAPPING_PARAMS"
    assert Path(launch.DEFAULT_PARAMS_PATH) == PARAMS_FILE


# --- README ------------------------------------------------------------------------------------------------------


def test_readme_sections() -> None:
    text = README.read_text()
    for section in ("## Pipeline", "## Frames and alignment", "## Parameters", "## Memory", "## Database"):
        assert section in text, section


def test_readme_documents_scan_grid_override_and_rebend_behaviour() -> None:
    text = README.read_text()
    assert re.search(r'Grid/Sensor: "2"', text)
    assert "indistinguishable" in text
    assert "re-bend" in text
    for key in EXPECTED_RTABMAP:
        assert f"`{key}`" in text, key


@pytest.mark.parametrize("path", [README, PARAMS_FILE, LAUNCH_FILE, PROBE_SCRIPT], ids=lambda p: p.name)
def test_no_parking_wording(path: Path) -> None:
    """The node's docs, config, launch and probe state what is done, never what is deferred.

    :param path: Path - file under review.
    :return: None
    """
    hits = [
        f"{number}: {line.strip()}"
        for number, line in enumerate(path.read_text().splitlines(), start=1)
        if PARKING_WORDING.search(line)
    ]
    assert not hits, hits
