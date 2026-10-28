#!/usr/bin/env python3
"""Launch RTAB-Map as a depth-only 3D mapper locked to slam_toolbox's map frame (design D1, D5-D9).

One component_container process loads rtabmap_sync::RGBDSync (exact sync of the RGB / depth pair the client's
head_rgbd_cpp publishes) and rtabmap_slam::CoreWrapper (namespace rtabmap, RGB-D + scan). Topics and QoS come from the
map3d topic contract (config/map3d_topics.yaml); RTAB-Map parameters from config/rtabmap_params.yaml (override the path
with env RTABMAP_MAPPING_PARAMS), merged key by key with the deployed /etc/ros2/rtabmap_mapping/config.yaml (env
RTABMAP_MAPPING_CONFIG, written by Ansible) when it exists and is non-empty, the same rule as slam_toolbox.

TF is read from /uplink/tf and /uplink/tf_static through process-wide remaps: rtabmap's tf2 TransformListener spins an
internal node that the component's own remaps do not reach. Nothing in this process publishes TF (publish_tf false),
so slam_toolbox stays the only map -> odom owner.

Before the container starts, <database_dir>/saved.db is copied over <database_dir>/live.db when it is newer or live.db
is missing, so after a restart the 3D map comes from the same save as slam_toolbox's resumed posegraph. The database
is never deleted on start.
"""

import copy
import os
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from pydantic import BaseModel, Field

NODE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(NODE_DIR))

from map3d_topics import Map3dTopics, Stream, load_map3d_topics  # noqa: E402

PARAMS_ENV = "RTABMAP_MAPPING_PARAMS"
CONFIG_ENV = "RTABMAP_MAPPING_CONFIG"
DEFAULT_PARAMS_PATH = str(NODE_DIR / "config" / "rtabmap_params.yaml")
DEFAULT_CONFIG_PATH = "/etc/ros2/rtabmap_mapping/config.yaml"
LIVE_DB = "live.db"
SAVED_DB = "saved.db"
RESTORE_TMP_SUFFIX = ".restoring"

CONTAINER_NAME = "rtabmap_container"
CONTAINER_PACKAGE = "rclcpp_components"
CONTAINER_EXECUTABLE = "component_container"
RTABMAP_NAMESPACE = "rtabmap"
READER = "rtabmap_mapping"
CORE_NODE_NAME = "rtabmap"
# CoreWrapper creates its services under its node name (<ns>/rtabmap/backup); these are moved to <ns>/<service>, the
# interface map3d_manager calls (D9).
CORE_SERVICES = ("backup", "reset", "pause", "resume")

# Message types that pick the image uplinks this container reads out of the contract; the topic names live only in
# config/map3d_topics.yaml.
RGB_TYPE = "sensor_msgs/msg/CompressedImage"
RGB_INFO_TYPE = "sensor_msgs/msg/CameraInfo"
DEPTH_TYPE = "sensor_msgs/msg/Image"
# Client topics (contract sources) whose uplink dests this container reads.
SCAN_SOURCE = "/scan_filtered"
TF_SOURCES = ("/tf", "/tf_static")
RGB_TRANSPORT = "compressed"  # the relay passes head_rgbd_cpp's JPEG through as <base>/compressed

# image_transport parameter of each transport-aware input (0.23.13 names; depth_image_transport is deprecated).
TRANSPORT_PARAMS = {"rgb/image": "image_transport", "depth/image": "depth_transport"}
RAW_TRANSPORT = "raw"
# rmw_qos_reliability_policy_t values RTAB-Map's qos* parameters take.
QOS_RELIABILITY = {"reliable": 1, "best_effort": 2}

# Values an override may not change: they define the inputs (U1, D5) and the TF / database invariants (U2, D9).
FIXED_CORE_PARAMS: dict[str, Any] = {
    "subscribe_rgbd": True,
    "subscribe_scan": True,
    "subscribe_rgb": False,
    "subscribe_depth": False,
    "approx_sync": True,  # the rgbd pair and the scan come from different sensors
    "publish_tf": False,
    "delete_db_on_start": False,
}
FIXED_SYNC_PARAMS: dict[str, Any] = {"approx_sync": False}  # each head_rgbd pair carries one stamp


class NodeParams(BaseModel, extra="forbid"):
    """ROS parameters of one component (the ros__parameters block of a ROS params file)."""

    ros__parameters: dict[str, Any] = Field(default_factory=dict)


class RtabmapMappingConfig(BaseModel, extra="forbid"):
    """Merged rtabmap_params.yaml + deployed override.

    Attributes:
        database_dir: directory of live.db and saved.db.
        rgbd_sync: rtabmap_sync::RGBDSync parameters.
        rtabmap: rtabmap_slam::CoreWrapper parameters.
    """

    database_dir: str
    rgbd_sync: NodeParams = Field(default_factory=NodeParams)
    rtabmap: NodeParams


@dataclass(frozen=True)
class ComponentSpec:
    """One composable node of the container.

    Attributes:
        package: ROS package of the plugin.
        plugin: Component class name.
        name: Node name.
        namespace: Node namespace.
        remappings: (from, to) topic remaps of the node.
        parameters: ROS parameters.
        service_remappings: (from, to) service remaps of the node.
    """

    package: str
    plugin: str
    name: str
    namespace: str
    remappings: list[tuple[str, str]]
    parameters: dict[str, Any]
    service_remappings: list[tuple[str, str]] = field(default_factory=list)

    def external_inputs(self) -> set[str]:
        """Topics this node subscribes from outside the process: remap targets plus the image_transport suffix.

        Returns:
            set[str]: Fully resolved topic names.
        """
        topics = set()
        for name, target in self.remappings:
            transport = self.parameters.get(TRANSPORT_PARAMS.get(name, ""), RAW_TRANSPORT)
            topics.add(target if transport == RAW_TRANSPORT else f"{target}/{transport}")
        return topics


@dataclass(frozen=True)
class ContainerSpec:
    """The single rtabmap_mapping process.

    Attributes:
        name: Container node name.
        package: Container package.
        executable: Container executable.
        ros_arguments: Process-wide ROS arguments (global TF remaps).
        components: Loaded composable nodes.
    """

    name: str
    package: str
    executable: str
    ros_arguments: list[str]
    components: list[ComponentSpec] = field(default_factory=list)

    def subscribed_topics(self) -> set[str]:
        """Every topic the process reads from outside: component inputs and the globally remapped TF topics.

        Returns:
            set[str]: Fully resolved topic names.
        """
        topics = {
            rule.split(":=", 1)[1] for flag, rule in zip(self.ros_arguments, self.ros_arguments[1:]) if flag == "-r"
        }
        for component in self.components:
            topics |= component.external_inputs()
        return topics


def merge_config(defaults: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Merge override into defaults key by key (nested dicts recursively, other values replaced).

    Args:
        defaults: Parsed defaults document.
        override: Parsed override document.

    Returns:
        dict[str, Any]: A new merged document.
    """
    merged = copy.deepcopy(defaults)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_config(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def load_config(defaults_path: Path, override_path: Path) -> RtabmapMappingConfig:
    """Load the repo defaults and merge the deployed override on top.

    Args:
        defaults_path: config/rtabmap_params.yaml.
        override_path: Deployed overrides; skipped when missing or empty (Ansible writes an empty file for an empty
            config block).

    Returns:
        RtabmapMappingConfig: The validated merged config.
    """
    doc = yaml.safe_load(defaults_path.read_text()) or {}
    if override_path.is_file() and override_path.stat().st_size > 0:
        doc = merge_config(doc, yaml.safe_load(override_path.read_text()) or {})
    return RtabmapMappingConfig(**doc)


def uplink_stream(topics: Map3dTopics, source: str) -> Stream:
    """Find the uplink stream of a client topic that rtabmap_mapping reads.

    Args:
        topics: The map3d topic contract.
        source: Client topic.

    Returns:
        Stream: The contract entry.
    """
    for stream in topics.uplink:
        if stream.source == source and READER in stream.server_readers:
            return stream
    raise ValueError(f"map3d contract has no uplink of {source} read by {READER}")


def image_sources(topics: Map3dTopics) -> tuple[Stream, Stream, Stream]:
    """Find the RGB, RGB camera_info and depth uplinks that rtabmap_mapping reads, by message type.

    Args:
        topics: The map3d topic contract.

    Returns:
        tuple[Stream, Stream, Stream]: RGB (compressed image), RGB camera_info and depth contract entries.
    """
    readers = [stream for stream in topics.uplink if READER in stream.server_readers]
    found = []
    for msg_type in (RGB_TYPE, RGB_INFO_TYPE, DEPTH_TYPE):
        matches = [stream for stream in readers if stream.type == msg_type]
        if len(matches) != 1:
            raise ValueError(
                f"map3d contract needs exactly one uplink of {msg_type} read by {READER}, has {len(matches)}"
            )
        found.append(matches[0])
    return found[0], found[1], found[2]


def transport_base(dest: str, transport: str) -> str:
    """Base topic an image_transport subscriber resolves to dest.

    Args:
        dest: Transport topic (e.g. /uplink/x/image_raw/compressed).
        transport: Transport name (e.g. compressed).

    Returns:
        str: The base topic (e.g. /uplink/x/image_raw).
    """
    suffix = f"/{transport}"
    if not dest.endswith(suffix):
        raise ValueError(f"{dest} is not a {transport} transport topic")
    return dest[: -len(suffix)]


def lock_params(node: str, params: dict[str, Any], fixed: dict[str, Any]) -> dict[str, Any]:
    """Apply the launch-fixed values, refusing a config that sets one of them differently.

    Args:
        node: Node name for the error message.
        params: Merged config parameters.
        fixed: Values the launch fixes.

    Returns:
        dict[str, Any]: params with the fixed values applied.
    """
    for key, value in fixed.items():
        if key in params and params[key] != value:
            raise ValueError(f"{node}: {key} is fixed to {value!r} by the launch, config sets {params[key]!r}")
    return {**params, **fixed}


def build_container(topics: Map3dTopics, params: RtabmapMappingConfig) -> ContainerSpec:
    """Build the single container: RGBDSync + CoreWrapper with global TF remaps.

    Args:
        topics: The map3d topic contract.
        params: Merged rtabmap_mapping config.

    Returns:
        ContainerSpec: Plugins, remaps, parameters and ros_arguments.
    """
    rgb, rgb_info, depth = image_sources(topics)
    scan = uplink_stream(topics, SCAN_SOURCE)
    image_qos = QOS_RELIABILITY[rgb.qos.reliability]
    if QOS_RELIABILITY[depth.qos.reliability] != image_qos:
        raise ValueError("rgbd_sync has one image qos: the contract must give rgb and depth the same reliability")

    sync_params = lock_params(
        "rgbd_sync",
        params.rgbd_sync.ros__parameters,
        {
            **FIXED_SYNC_PARAMS,
            "image_transport": RGB_TRANSPORT,
            "depth_transport": depth.transport,
            "qos": image_qos,
            "qos_camera_info": QOS_RELIABILITY[rgb_info.qos.reliability],
        },
    )
    core_params = lock_params(
        "rtabmap",
        params.rtabmap.ros__parameters,
        {
            **FIXED_CORE_PARAMS,
            "qos_image": image_qos,  # rgbd_image comes from rgbd_sync, published with its qos
            "qos_scan": QOS_RELIABILITY[scan.qos.reliability],
            "database_path": str(Path(params.database_dir) / LIVE_DB),
        },
    )
    sync = ComponentSpec(
        package="rtabmap_sync",
        plugin="rtabmap_sync::RGBDSync",
        name="rgbd_sync",
        namespace=RTABMAP_NAMESPACE,  # publishes rgbd_image -> /rtabmap/rgbd_image, CoreWrapper's input
        remappings=[
            ("rgb/image", transport_base(rgb.dest, RGB_TRANSPORT)),
            ("depth/image", transport_base(depth.dest, depth.transport)),
            ("rgb/camera_info", rgb_info.dest),
        ],
        parameters=sync_params,
    )
    core = ComponentSpec(
        package="rtabmap_slam",
        plugin="rtabmap_slam::CoreWrapper",
        name=CORE_NODE_NAME,
        namespace=RTABMAP_NAMESPACE,
        remappings=[("scan", scan.dest)],
        parameters=core_params,
        service_remappings=[
            (f"/{RTABMAP_NAMESPACE}/{CORE_NODE_NAME}/{srv}", f"/{RTABMAP_NAMESPACE}/{srv}") for srv in CORE_SERVICES
        ],
    )
    ros_arguments: list[str] = []
    for source in TF_SOURCES:
        ros_arguments += ["-r", f"{source}:={uplink_stream(topics, source).dest}"]
    return ContainerSpec(
        name=CONTAINER_NAME,
        package=CONTAINER_PACKAGE,
        executable=CONTAINER_EXECUTABLE,
        ros_arguments=ros_arguments,
        components=[sync, core],
    )


def saved_is_newer(db_dir: Path) -> bool:
    """Whether saved.db should replace live.db.

    Args:
        db_dir: Database directory.

    Returns:
        bool: True when saved.db exists and live.db is missing or older.
    """
    live, saved = db_dir / LIVE_DB, db_dir / SAVED_DB
    if not saved.is_file():
        return False
    return not live.is_file() or saved.stat().st_mtime > live.stat().st_mtime


def restore_saved_database(db_dir: Path) -> Path:
    """Restore the last saved 3D map before RTAB-Map opens live.db; never deletes a database.

    saved.db is copied (mtime preserved) to a temporary file and renamed over live.db, so a crash mid-copy leaves the
    old live.db intact and the next start does not copy again unless saved.db changes.

    Args:
        db_dir: Database directory, created when missing.

    Returns:
        Path: live.db, the database_path RTAB-Map opens (created by RTAB-Map when absent).
    """
    db_dir.mkdir(parents=True, exist_ok=True)
    live = db_dir / LIVE_DB
    if saved_is_newer(db_dir):
        tmp = live.with_name(live.name + RESTORE_TMP_SUFFIX)
        shutil.copy2(db_dir / SAVED_DB, tmp)
        os.replace(tmp, live)
    return live


def generate_launch_description() -> LaunchDescription:
    """Build the rtabmap_mapping launch description.

    Returns:
        LaunchDescription: One component_container with RGBDSync and CoreWrapper.
    """
    defaults_path = Path(os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH))
    override_path = Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH))
    config = load_config(defaults_path, override_path)
    spec = build_container(load_map3d_topics(), config)
    db_dir = Path(config.database_dir)
    restoring = saved_is_newer(db_dir)
    live = restore_saved_database(db_dir)
    status = f"restored {db_dir / SAVED_DB} over {live}" if restoring else f"opening {live}"
    nodes = [
        ComposableNode(
            package=c.package,
            plugin=c.plugin,
            name=c.name,
            namespace=c.namespace,
            remappings=c.remappings + c.service_remappings,
            parameters=[c.parameters],
        )
        for c in spec.components
    ]
    container = ComposableNodeContainer(
        name=spec.name,
        namespace="",
        package=spec.package,
        executable=spec.executable,
        output="screen",
        ros_arguments=spec.ros_arguments,
        composable_node_descriptions=nodes,
    )
    return LaunchDescription(
        [LogInfo(msg=f"rtabmap_mapping: params {defaults_path} + {override_path}; {status}"), container]
    )
