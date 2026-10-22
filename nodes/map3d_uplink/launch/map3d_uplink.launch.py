#!/usr/bin/env python3
"""Launch the map3d uplink: one component_container carrying every map3d stream across the client <-> server link.

The streams come from the map3d topic contract (nodes/rtabmap_mapping/map3d_topics.py, config/map3d_topics.yaml). Each
contract entry becomes one composable node: topic_tools ThrottleNode (entry with a rate), topic_tools RelayNode (no
rate) or image_transport Republisher (compressedDepth entry). All of them publish the contract dest, never a client
topic under its own name, and are lazy (nothing is subscribed upstream while the server has no subscriber) except
/tf_static.
"""

import sys
from dataclasses import dataclass, field
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "rtabmap_mapping"))
from map3d_topics import TF_STATIC_TOPIC, Map3dTopics, Qos, Stream, load_map3d_topics  # noqa: E402

CONTAINER_NAME = "map3d_uplink"
CONTAINER_PACKAGE = "rclcpp_components"
CONTAINER_EXECUTABLE = "component_container"
TOPIC_TOOLS_PACKAGE = "topic_tools"
IMAGE_TRANSPORT_PACKAGE = "image_transport"
THROTTLE_PLUGIN = "topic_tools::ThrottleNode"
RELAY_PLUGIN = "topic_tools::RelayNode"
REPUBLISHER_PLUGIN = "image_transport::Republisher"
DEPTH_TRANSPORT = "compressedDepth"
DEPTH_PNG_LEVEL = 1


@dataclass(frozen=True)
class ComposableNodeSpec:
    """Plain description of one composable node (convertible to launch_ros ComposableNode).

    Attributes:
        package: package providing the component.
        plugin: component class name.
        name: node name, unique in the container.
        parameters: node parameters.
        remappings: (from, to) topic remappings.
    """

    package: str
    plugin: str
    name: str
    parameters: dict = field(default_factory=dict)
    remappings: tuple[tuple[str, str], ...] = ()

    @property
    def remappings_dict(self) -> dict[str, str]:
        """The remappings as a dict."""
        return dict(self.remappings)


def node_name(dest: str) -> str:
    """Derive a node name from a contract dest.

    Args:
        dest: contract dest topic.

    Returns:
        The dest without leading slash and compressedDepth suffix, slashes replaced by underscores.
    """
    return dest.removesuffix(f"/{DEPTH_TRANSPORT}").strip("/").replace("/", "_")


def qos_override(topic: str, side: str, qos: Qos) -> dict:
    """QoS override parameters of one relay endpoint.

    Args:
        topic: topic the override applies to.
        side: "subscription" or "publisher".
        qos: QoS to apply.

    Returns:
        qos_overrides parameters (reliability, durability, depth).
    """
    prefix = f"qos_overrides.{topic}.{side}"
    return {
        f"{prefix}.reliability": qos.reliability,
        f"{prefix}.durability": qos.durability,
        f"{prefix}.depth": qos.depth,
    }


def build_stream_node(stream: Stream) -> ComposableNodeSpec:
    """Build the composable node relaying one contract entry.

    Args:
        stream: contract entry.

    Returns:
        The Republisher, ThrottleNode or RelayNode spec publishing stream.dest.
    """
    name = node_name(stream.dest)
    if stream.transport == DEPTH_TRANSPORT:
        base = stream.dest.removesuffix(f"/{DEPTH_TRANSPORT}")
        return ComposableNodeSpec(
            package=IMAGE_TRANSPORT_PACKAGE,
            plugin=REPUBLISHER_PLUGIN,
            name=name,
            parameters={
                "in_transport": "raw",
                "out_transport": DEPTH_TRANSPORT,
                f"{base.strip('/').replace('/', '.')}.{DEPTH_TRANSPORT}.png_level": DEPTH_PNG_LEVEL,
                # same contract QoS as the topic_tools relays: no reliable retransmission stall on the Wi-Fi leg
                **qos_override(stream.dest, "publisher", stream.qos),
                **qos_override(stream.source, "subscription", stream.qos),
            },
            remappings=(("in", stream.source), ("out", base)),
        )
    parameters: dict = {"input_topic": stream.source, "output_topic": stream.dest}
    if stream.rate_hz is not None:
        plugin = THROTTLE_PLUGIN
        parameters.update({"throttle_type": "messages", "msgs_per_sec": stream.rate_hz})
    else:
        plugin = RELAY_PLUGIN
    parameters["lazy"] = stream.source != TF_STATIC_TOPIC
    # topic_tools copies the client publisher's QoS unless told otherwise: a reliable source (scan) would cross Wi-Fi
    # reliable and stall behind retransmissions instead of the contract's best_effort.
    parameters.update(qos_override(stream.dest, "publisher", stream.qos))
    if stream.source == TF_STATIC_TOPIC:
        parameters.update(qos_override(stream.source, "subscription", stream.qos))
    return ComposableNodeSpec(package=TOPIC_TOOLS_PACKAGE, plugin=plugin, name=name, parameters=parameters)


def build_uplink_nodes(topics: Map3dTopics) -> list[ComposableNodeSpec]:
    """Build one composable node per contract entry (uplink and downlink).

    Args:
        topics: the map3d topic contract.

    Returns:
        The node specs, uplink entries first.
    """
    return [build_stream_node(s) for s in topics.uplink + topics.downlink]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        LaunchDescription: the single map3d_uplink component_container.
    """
    nodes = [
        ComposableNode(
            package=s.package,
            plugin=s.plugin,
            name=s.name,
            parameters=[s.parameters],
            remappings=list(s.remappings),
        )
        for s in build_uplink_nodes(load_map3d_topics())
    ]
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name=CONTAINER_NAME,
                namespace="",
                package=CONTAINER_PACKAGE,
                executable=CONTAINER_EXECUTABLE,
                composable_node_descriptions=nodes,
                output="screen",
            )
        ]
    )
