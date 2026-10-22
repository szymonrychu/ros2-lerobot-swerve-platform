"""map3d_uplink: the single client process carrying the map3d streams to the server and the 3D map back (D2)."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

from tests.ros_stubs import install_ros_stubs

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
LAUNCH_FILE = REPO_ROOT / "nodes" / "map3d_uplink" / "launch" / "map3d_uplink.launch.py"
README = REPO_ROOT / "nodes" / "map3d_uplink" / "README.md"
MAPPING_DIR = REPO_ROOT / "nodes" / "rtabmap_mapping"

NODE = "map3d_uplink"
THROTTLE = "topic_tools::ThrottleNode"
RELAY = "topic_tools::RelayNode"
REPUBLISHER = "image_transport::Republisher"
TF_STATIC = "/tf_static"

if str(MAPPING_DIR) not in sys.path:
    sys.path.insert(0, str(MAPPING_DIR))
from map3d_topics import load_map3d_topics  # noqa: E402


def load_launch() -> ModuleType:
    """Import the launch file with the launch / launch_ros modules stubbed (they are not installed on dev machines)."""
    install_ros_stubs("launch", "launch_ros", "launch_ros.actions", "launch_ros.descriptions")
    spec = importlib.util.spec_from_file_location("map3d_uplink_launch", LAUNCH_FILE)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def launch() -> ModuleType:
    return load_launch()


@pytest.fixture(scope="module")
def contract():
    return load_map3d_topics()


@pytest.fixture(scope="module")
def specs(launch, contract) -> list:
    return launch.build_uplink_nodes(contract)


def client_vars() -> dict:
    return yaml.safe_load(CLIENT_VARS.read_text())


def spec_for(specs: list, dest: str):
    """The spec publishing a contract dest (the compressedDepth entry publishes its base)."""
    base = dest.removesuffix("/compressedDepth")
    return next(s for s in specs if base in (s.parameters.get("output_topic"), s.remappings_dict.get("out")))


def test_one_node_per_contract_entry_with_unique_names(specs, contract) -> None:
    assert len(specs) == len(contract.uplink) + len(contract.downlink)
    names = [s.name for s in specs]
    assert len(set(names)) == len(names)


def test_every_published_topic_is_a_contract_dest(specs, contract) -> None:
    dests = {s.dest.removesuffix("/compressedDepth") for s in contract.uplink + contract.downlink}
    sources = {s.source for s in contract.uplink + contract.downlink}
    published = {s.parameters.get("output_topic") or s.remappings_dict["out"] for s in specs}
    assert published == dests
    assert not published & sources


def test_plugin_rate_and_lazy_per_entry(specs, contract) -> None:
    for stream in contract.uplink + contract.downlink:
        spec = spec_for(specs, stream.dest)
        if stream.transport == "compressedDepth":
            depth_base = stream.dest.removesuffix("/compressedDepth")
            assert spec.plugin == REPUBLISHER
            assert spec.parameters == {
                "in_transport": "raw",
                "out_transport": "compressedDepth",
                f"{depth_base.strip('/').replace('/', '.')}.compressedDepth.png_level": 1,
            }
            assert spec.remappings_dict == {"in": stream.source, "out": stream.dest.removesuffix("/compressedDepth")}
            continue
        assert spec.parameters["input_topic"] == stream.source
        if stream.rate_hz is not None:
            assert spec.plugin == THROTTLE
            assert spec.parameters["throttle_type"] == "messages"
            assert spec.parameters["msgs_per_sec"] == stream.rate_hz
        else:
            assert spec.plugin == RELAY
            assert "msgs_per_sec" not in spec.parameters
        assert spec.parameters["lazy"] is (stream.source != TF_STATIC)


def test_depth_republisher_has_no_throttle_in_front(specs, contract) -> None:
    depth = next(s for s in contract.uplink if s.transport == "compressedDepth")
    assert not any(s.parameters.get("output_topic", "").startswith("/head_rgbd/depth") for s in specs)
    assert any(s.remappings_dict.get("in") == depth.source for s in specs)
    assert not any(s.plugin == THROTTLE and s.parameters["input_topic"] == depth.source for s in specs)


def test_cloud_downlink_throttled_at_a_tenth_hz_lazy(specs) -> None:
    cloud = spec_for(specs, "/map3d/cloud")
    assert (cloud.plugin, cloud.parameters["msgs_per_sec"], cloud.parameters["lazy"]) == (THROTTLE, 0.1, True)


def test_tf_static_qos_overrides_on_both_sides(specs) -> None:
    spec = spec_for(specs, "/uplink/tf_static")
    for side in ("subscription", "publisher"):
        prefix = (
            f"qos_overrides./tf_static.{side}"
            if side == "subscription"
            else "qos_overrides./uplink/tf_static.publisher"
        )
        assert spec.parameters[f"{prefix}.reliability"] == "reliable"
        assert spec.parameters[f"{prefix}.durability"] == "transient_local"
        assert spec.parameters[f"{prefix}.depth"] == 100


def test_launch_file_defines_one_container_named_map3d_uplink(launch) -> None:
    description = launch.generate_launch_description()
    container = launch.ComposableNodeContainer
    assert container.call_count == 1
    kwargs = container.call_args.kwargs
    assert kwargs["name"] == NODE
    assert kwargs["package"] == "rclcpp_components"
    assert kwargs["executable"] == "component_container"
    assert len(kwargs["composable_node_descriptions"]) == len(launch.build_uplink_nodes(load_map3d_topics()))
    assert description is not None


def test_node_type_env_apt_tier_and_paths() -> None:
    node_type = client_vars()["ros2_node_type_defaults"][NODE]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/map3d_uplink/launch/map3d_uplink.launch.py"
    )
    assert node_type["node_src_dir"] == ""
    assert node_type["src_paths"] == [
        "nodes/map3d_uplink",
        "nodes/rtabmap_mapping/config",
        "nodes/rtabmap_mapping/map3d_topics.py",
    ]
    assert set(node_type["apt_packages"]) >= {
        "ros-jazzy-topic-tools",
        "ros-jazzy-image-transport-plugins",
        "ros-jazzy-rclcpp-components",
        "python3-pydantic",
        "python3-yaml",
    }
    assert (node_type["cpu_weight"], node_type["io_weight"], node_type["oom_score_adjust"]) == (100, 100, 0)
    assert (node_type["memory_low"], node_type["memory_max"]) == ("48M", "256M")
    assert node_type["env"] == ["ROS_STATIC_PEERS={{ ros2_server_hostname }}"]


def test_ros2_nodes_entry_present_and_enabled() -> None:
    entry = next(n for n in client_vars()["ros2_nodes"] if n["name"] == NODE)
    assert entry["node_type"] == NODE and entry["present"] is True and entry["enabled"] is True


def test_head_camera_mapping_stream_and_rgbd_node_present() -> None:
    nodes = {n["name"]: n for n in client_vars()["ros2_nodes"]}
    camera = yaml.safe_load(nodes["head_camera_cpp"]["config"])
    assert camera["mapping"]["enabled"] is True and camera["mapping"]["lock_focus"] is True
    assert camera.get("af_mode", "continuous") == "continuous"
    assert nodes["head_rgbd_cpp"]["present"] is True and nodes["head_rgbd_cpp"]["enabled"] is True


def test_playbook_step_after_head_camera_cpp() -> None:
    block = yaml.safe_load(PLAYBOOK.read_text())[0]["tasks"][0]["block"]
    names = [t["name"] for t in block]
    step = block[names.index("Deploy map3d_uplink")]
    assert names.index("Deploy map3d_uplink") > names.index("Deploy head_camera_cpp")
    assert step["tags"] == [NODE, "apt", "python", "build", "config"]
    assert step["vars"]["_deploy_node_name"] == NODE
    assert step["ansible.builtin.include_tasks"]["file"] == "tasks/resolve_and_deploy.yml"


def test_readme_documents_purpose_lazy_and_measuring() -> None:
    text = README.read_text()
    for needle in ("Cross-host", "map3d_topics", "lazy", "bandwidth", "continuous"):
        assert needle in text, needle


def test_every_topic_tools_relay_publishes_with_its_contract_qos(specs, contract) -> None:
    """topic_tools copies the client publisher QoS (e.g. reliable scans over Wi-Fi) unless the contract QoS is set."""
    for stream in contract.uplink + contract.downlink:
        if stream.transport != "raw":
            continue
        spec = spec_for(specs, stream.dest)
        prefix = f"qos_overrides.{stream.dest}.publisher"
        assert spec.parameters[f"{prefix}.reliability"] == stream.qos.reliability, stream.dest
        assert spec.parameters[f"{prefix}.durability"] == stream.qos.durability, stream.dest
        assert spec.parameters[f"{prefix}.depth"] == stream.qos.depth, stream.dest


def test_depth_republisher_uses_contract_qos_both_sides(specs, contract) -> None:
    """The compressedDepth Republisher must not cross Wi-Fi with the upstream QoS either."""
    depth = next(s for s in contract.uplink if s.transport == "compressedDepth")
    spec = next(s for s in specs if s.remappings_dict.get("in") == depth.source)
    for topic, side in ((depth.dest, "publisher"), (depth.source, "subscription")):
        prefix = f"qos_overrides.{topic}.{side}"
        assert spec.parameters[f"{prefix}.reliability"] == depth.qos.reliability
        assert spec.parameters[f"{prefix}.durability"] == depth.qos.durability
        assert spec.parameters[f"{prefix}.depth"] == depth.qos.depth
