"""The network role's netplan rendering after the retired link support was dropped: DHCP ethernets and the WiFi stanza
render as before, no static link ethernet exists, and no link variable remains. Nothing here runs ansible."""

from pathlib import Path
from typing import Any

import jinja2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "network"
RETIRED_VARS = ("network_link_ethernets", "network_netplan_superseded_files")

WIFI_VARS: dict[str, Any] = {
    "_network_interface": "wlan0",
    "_network_interface_is_wlan": True,
    "network_address": "192.168.1.34/24",
    "network_gateway": "192.168.1.1",
    "network_nameservers": ["192.168.1.1", "1.1.1.1"],
    "network_wifi_ssid": "home",
    "network_wifi_password": "secret",
}


def render_text(other: list[str] | None = None) -> str:
    """Render the netplan template for a WiFi primary.

    Args:
        other: DHCP ethernets (the role's _other_ethernets).

    Returns:
        The rendered netplan text.
    """
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(ROLE_DIR / "templates"), trim_blocks=True, lstrip_blocks=True
    )
    return env.get_template("netplan.yaml.j2").render(**WIFI_VARS, _other_ethernets=other or [])


def render(other: list[str] | None = None) -> dict[str, Any]:
    """Render the netplan template and parse the result.

    Args:
        other: DHCP ethernets (the role's _other_ethernets).

    Returns:
        The parsed netplan document.
    """
    return yaml.safe_load(render_text(other))


def test_client_render_has_no_static_ethernet_and_no_link_address() -> None:
    text = render_text()
    assert "ethernets" not in render()["network"]
    assert "10.77" not in text
    assert "ignore-carrier" not in text


def test_wifi_stanza_is_unchanged() -> None:
    wlan = render()["network"]["wifis"]["wlan0"]
    assert wlan["dhcp4"] is False
    assert wlan["addresses"] == ["192.168.1.34/24"]
    assert wlan["routes"] == [{"to": "default", "via": "192.168.1.1"}]
    assert wlan["nameservers"] == {"addresses": ["192.168.1.1", "1.1.1.1"]}
    assert wlan["access-points"] == {"home": {"password": "secret"}}


def test_dhcp_ethernets_still_render() -> None:
    eths = render(other=["eth0", "usb0"])["network"]["ethernets"]
    for name in ("eth0", "usb0"):
        assert eths[name]["dhcp4"] is True
        assert eths[name]["optional"] is True


def test_role_names_no_retired_variable() -> None:
    paths = (
        ROLE_DIR / "defaults" / "main.yml",
        ROLE_DIR / "tasks" / "main.yml",
        ROLE_DIR / "templates" / "netplan.yaml.j2",
    )
    for path in paths:
        text = path.read_text()
        for var in RETIRED_VARS:
            assert var not in text, f"{path.name} still names {var}"
    assert "network_netplan_superseded" not in (ROLE_DIR / "tasks" / "main.yml").read_text()


def test_group_vars_define_no_link_ethernets() -> None:
    for path in (ANSIBLE_DIR / "group_vars").glob("*.yml"):
        assert "network_link_ethernets" not in path.read_text(), path.name
