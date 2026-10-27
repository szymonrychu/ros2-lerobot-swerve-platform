"""The static-peer Wi-Fi bridges (map3d_uplink, leader_bridge, master2master) must not block on Fast DDS UDP sends.

Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time and 0 (no timeout) once
it has passed, so during a Wi-Fi stall a bridge blocks until the carrier is lost. These nodes therefore get a profile
file with a UDPv4 user transport whose `non_blocking_send` is true (a full socket buffer drops the datagram instead).
"""

from pathlib import Path
from xml.etree import ElementTree as ET

import jinja2
import pytest
import yaml

ANSIBLE_DIR = Path(__file__).resolve().parent.parent / "ansible"
README = Path(__file__).resolve().parent.parent / "nodes" / "map3d_uplink" / "README.md"
BRIDGES = ("master2master", "leader_bridge", "map3d_uplink")
PROFILES_VAR = "ros2_dds_bridge_profiles_file"
PROFILES_ENV = "FASTRTPS_DEFAULT_PROFILES_FILE={{ ros2_dds_bridge_profiles_file }}"
NS = {"p": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}


def load_vars(name: str) -> dict:
    """Load one group_vars file.

    Args:
        name (str): Group name.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load((ANSIBLE_DIR / "group_vars" / f"{name}.yml").read_text())


def bridge_env(name: str) -> list[str]:
    """Env entries of one bridge from the client type defaults.

    Args:
        name (str): Node type.

    Returns:
        list[str]: Env strings.
    """
    return load_vars("client")["ros2_node_type_defaults"][name]["env"]


def render_profile() -> ET.Element:
    """Render the bridge profile the way dds_host_setup.yml installs it.

    Returns:
        ET.Element: Parsed XML root.
    """
    tasks = yaml.safe_load((ANSIBLE_DIR / "playbooks" / "tasks" / "dds_host_setup.yml").read_text())
    task = next(t for t in tasks if t.get("ansible.builtin.template", {}).get("dest") == "{{ " + PROFILES_VAR + " }}")
    src = ANSIBLE_DIR / "playbooks" / "templates" / task["ansible.builtin.template"]["src"]
    env = jinja2.Environment(trim_blocks=True, lstrip_blocks=True, keep_trailing_newline=True)
    return ET.fromstring(env.from_string(src.read_text()).render(**load_vars("all")))


@pytest.mark.parametrize("node", BRIDGES)
def test_map3d_leader_bridge_use_nonblocking_profile(node: str) -> None:
    assert PROFILES_ENV in bridge_env(node)


def test_map3d_leader_bridge_profile_path_is_defined() -> None:
    assert load_vars("all")[PROFILES_VAR] == "/etc/ros2/fastdds_wifi_bridge.xml"


def test_map3d_leader_bridge_profile_sets_nonblocking_udp_transport() -> None:
    root = render_profile()
    descriptor = root.find("p:transport_descriptors/p:transport_descriptor", NS)
    assert descriptor is not None
    assert descriptor.findtext("p:type", namespaces=NS) == "UDPv4"
    assert descriptor.findtext("p:non_blocking_send", namespaces=NS) == "true"
    participant = root.find("p:participant", NS)
    assert participant.get("is_default_profile") == "true"
    assert participant.findtext("p:rtps/p:useBuiltinTransports", namespaces=NS) == "false"
    assert participant.findtext("p:rtps/p:userTransports/p:transport_id", namespaces=NS) == descriptor.findtext(
        "p:transport_id", namespaces=NS
    )


def test_map3d_leader_bridge_profile_keeps_localhost_peers() -> None:
    all_vars = load_vars("all")
    ports = [int(e.text) for e in render_profile().iterfind(".//p:initialPeersList//p:port", NS)]
    first, last = all_vars["ros2_dds_extra_peer_first_id"], all_vars["ros2_dds_extra_peer_last_id"]
    assert len(ports) == last - first + 1


def test_map3d_leader_bridge_readme_documents_nonblocking_send() -> None:
    text = README.read_text()
    assert "non_blocking_send" in text and "SO_SNDTIMEO" in text
