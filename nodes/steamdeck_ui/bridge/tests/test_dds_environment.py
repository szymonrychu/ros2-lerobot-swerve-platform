"""Tests for DDS environment setup of the bridge (FastDDS discovery server client of the robot)."""

from bridge.config import BridgeConfig, apply_dds_environment


def test_sets_discovery_server_and_domain_from_config() -> None:
    environ: dict[str, str] = {}
    apply_dds_environment(BridgeConfig(ros_discovery_server="client.ros2.lan:11811", ros_domain_id="3"), environ)
    assert environ == {"ROS_DISCOVERY_SERVER": "client.ros2.lan:11811", "ROS_DOMAIN_ID": "3"}


def test_existing_environment_wins() -> None:
    environ = {"ROS_DISCOVERY_SERVER": "10.0.0.1:11811"}
    apply_dds_environment(BridgeConfig(), environ)
    assert environ["ROS_DISCOVERY_SERVER"] == "10.0.0.1:11811"
    assert environ["ROS_DOMAIN_ID"] == "0"
