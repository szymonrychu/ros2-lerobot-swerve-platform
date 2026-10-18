"""The network role's static point-to-point link (design D8): every network_link_ethernets entry renders as a static
ethernet that is excluded from the DHCP list, the WiFi stanza is untouched, a probe refuses a foreign IPv4 on the link
interface, and all.yml hands each host its entry by group membership. Nothing here runs ansible."""

import json
import re
from pathlib import Path
from typing import Any

import jinja2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "network"

CLIENT_LINK = [{"iface": "eth0", "address": "10.77.0.1/30"}]
JETSON_LINK = [{"iface": "eth0", "address": "10.77.0.2/30", "default_route_via": "10.77.0.1", "route_metric": 1000}]
WIFI_VARS: dict[str, Any] = {
    "_network_interface": "wlan0",
    "_network_interface_is_wlan": True,
    "network_address": "192.168.1.34/24",
    "network_gateway": "192.168.1.1",
    "network_nameservers": ["192.168.1.1", "1.1.1.1"],
    "network_wifi_ssid": "home",
    "network_wifi_password": "secret",
}


def render(link: list[dict[str, Any]], other: list[str] | None = None) -> dict[str, Any]:
    """Render the netplan template and parse the result.

    Args:
        link: The network_link_ethernets value.
        other: DHCP ethernets (the role's _other_ethernets).

    Returns:
        The parsed netplan document.
    """
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(ROLE_DIR / "templates"), trim_blocks=True, lstrip_blocks=True
    )
    text = env.get_template("netplan.yaml.j2").render(
        **WIFI_VARS, network_link_ethernets=link, _other_ethernets=other or []
    )
    return yaml.safe_load(text)


def load_tasks() -> list[dict[str, Any]]:
    """Parse the network role's tasks.

    Returns:
        The task list.
    """
    return yaml.safe_load((ROLE_DIR / "tasks" / "main.yml").read_text())


def named(name: str) -> dict[str, Any]:
    """Return the role task with this exact name.

    Args:
        name: Task name.

    Returns:
        The task.
    """
    return next(t for t in load_tasks() if t["name"] == name)


def test_client_link_renders_a_static_ethernet() -> None:
    eth = render(CLIENT_LINK)["network"]["ethernets"]["eth0"]
    assert eth["dhcp4"] is False
    assert eth["addresses"] == ["10.77.0.1/30"]
    assert eth["ignore-carrier"] is True
    assert eth["optional"] is True
    assert "routes" not in eth


def test_jetson_link_renders_a_high_metric_default_route() -> None:
    eth = render(JETSON_LINK)["network"]["ethernets"]["eth0"]
    assert eth["addresses"] == ["10.77.0.2/30"]
    assert eth["routes"] == [{"to": "default", "via": "10.77.0.1", "metric": 1000}]


def test_link_interface_is_not_in_the_dhcp_list() -> None:
    tasks = load_tasks()
    task = next(t for t in tasks if t["name"] == "Set other ethernet-like interfaces (for DHCP in template)")
    expr = task["ansible.builtin.set_fact"]["_other_ethernets"]
    env = jinja2.Environment()
    env.tests["match"] = lambda value, pattern: re.match(pattern, value) is not None  # ansible's match test
    out = env.from_string(expr).render(_other_interfaces=["eth0", "usb0"], network_link_ethernets=CLIENT_LINK)
    assert yaml.safe_load(out) == ["usb0"]


def test_dhcp_ethernets_still_render_next_to_the_link() -> None:
    eths = render(CLIENT_LINK, other=["usb0"])["network"]["ethernets"]
    assert eths["usb0"]["dhcp4"] is True
    assert eths["eth0"]["dhcp4"] is False


def test_no_link_renders_no_ethernets_for_a_wifi_primary() -> None:
    assert "ethernets" not in render([])["network"]


def test_wifi_stanza_is_identical_with_and_without_a_link() -> None:
    base = render([])["network"]["wifis"]
    assert render(CLIENT_LINK)["network"]["wifis"] == base
    assert render(JETSON_LINK)["network"]["wifis"] == base
    wlan = base["wlan0"]
    assert wlan["addresses"] == ["192.168.1.34/24"]
    assert wlan["routes"] == [{"to": "default", "via": "192.168.1.1"}]
    assert wlan["access-points"] == {"home": {"password": "secret"}}


def probe(facts: dict[str, Any], link: list[dict[str, Any]]) -> list[str]:
    """Evaluate the probe task's foreign-address fact.

    Args:
        facts: ansible_facts to probe.
        link: The network_link_ethernets value.

    Returns:
        The foreign IPv4 addresses found on the link interfaces.
    """
    expr = named("Probe link interfaces for foreign IPv4 addresses")["ansible.builtin.set_fact"][
        "_network_link_foreign_addresses"
    ]
    out = jinja2.Environment().from_string(expr).render(ansible_facts=facts, network_link_ethernets=link)
    return yaml.safe_load(out)


def test_probe_flags_a_foreign_ipv4() -> None:
    facts = {"eth0": {"ipv4": {"address": "192.168.1.50"}}}
    assert probe(facts, CLIENT_LINK) == ["eth0 192.168.1.50"]


def test_probe_flags_a_foreign_secondary_address() -> None:
    facts = {"eth0": {"ipv4": {"address": "10.77.0.1"}, "ipv4_secondaries": [{"address": "172.16.0.9"}]}}
    assert probe(facts, CLIENT_LINK) == ["eth0 172.16.0.9"]


def test_probe_passes_planned_address_and_missing_ipv4() -> None:
    assert probe({"eth0": {"ipv4": {"address": "10.77.0.1"}}}, CLIENT_LINK) == []
    assert probe({"eth0": {"active": True}}, CLIENT_LINK) == []
    assert probe({}, CLIENT_LINK) == []


def test_assert_reads_the_probe_fact_as_a_json_string() -> None:
    """set_fact stores the tojson output as the string "[]" (length 2): the assert must parse it first."""
    env = jinja2.Environment()
    env.filters["from_json"] = json.loads
    env.filters["from_yaml"] = yaml.safe_load
    fail = named("Fail when a link interface holds a foreign IPv4 address")["ansible.builtin.assert"]
    cond = env.from_string("{{ " + fail["that"] + " }}")
    assert cond.render(_network_link_foreign_addresses="[]") == "True"
    assert cond.render(_network_link_foreign_addresses='["eth0 1.2.3.4"]') == "False"
    msg = env.from_string(fail["fail_msg"]).render(_network_link_foreign_addresses='["eth0 1.2.3.4"]')
    assert "(eth0 1.2.3.4)" in msg


def test_assert_accepts_the_probe_fact_already_parsed_to_a_list() -> None:
    """ansible-core 2.18 (the jetson deploy) hands set_fact's JSON back as a list: from_json must not see a list."""
    env = jinja2.Environment()
    env.filters["from_json"] = json.loads
    env.filters["from_yaml"] = yaml.safe_load
    fail = named("Fail when a link interface holds a foreign IPv4 address")["ansible.builtin.assert"]
    cond = env.from_string("{{ " + fail["that"] + " }}")
    assert cond.render(_network_link_foreign_addresses=[]) == "True"
    assert cond.render(_network_link_foreign_addresses=["eth0 1.2.3.4"]) == "False"
    msg = env.from_string(fail["fail_msg"]).render(_network_link_foreign_addresses=["eth0 1.2.3.4"])
    assert "(eth0 1.2.3.4)" in msg


def test_probe_fails_with_a_message_naming_the_address_and_runs_before_netplan() -> None:
    tasks = load_tasks()
    names = [t["name"] for t in tasks]
    fail = named("Fail when a link interface holds a foreign IPv4 address")
    assert "_network_link_foreign_addresses" in str(fail["ansible.builtin.assert"]["that"])
    assert "_network_link_foreign_addresses" in fail["ansible.builtin.assert"]["fail_msg"]
    assert names.index(fail["name"]) < names.index("Deploy netplan configuration")


def test_all_yml_defines_the_link_vars_and_names_d8() -> None:
    text = (ANSIBLE_DIR / "group_vars" / "all.yml").read_text()
    assert "D8" in text
    data = yaml.safe_load(text)
    assert data["jetson_link_interface"] == "eth0"
    assert data["jetson_link_pi_address"] == "10.77.0.1/30"
    assert data["jetson_link_jetson_address"] == "10.77.0.2/30"
    assert data["jetson_link_pi_ip"] == "10.77.0.1"
    assert data["jetson_link_jetson_ip"] == "10.77.0.2"


def resolve(group_names: list[str]) -> Any:
    """Resolve all.yml's network_link_ethernets for a host in these groups.

    Args:
        group_names: The host's inventory groups.

    Returns:
        The rendered list.
    """
    data = yaml.safe_load((ANSIBLE_DIR / "group_vars" / "all.yml").read_text())
    ctx = {k: v for k, v in data.items() if k.startswith("jetson_link_")}
    out = jinja2.Environment().from_string(data["network_link_ethernets"]).render(group_names=group_names, **ctx)
    return yaml.safe_load(out)


def test_all_yml_keys_the_link_by_group_membership() -> None:
    assert resolve(["client"]) == CLIENT_LINK
    assert resolve(["jetson"]) == JETSON_LINK
    assert resolve(["server"]) == []
    assert resolve(["controller"]) == []


def test_no_client_group_vars_edit_defines_the_link() -> None:
    assert "network_link_ethernets" not in (ANSIBLE_DIR / "group_vars" / "client.yml").read_text()


def test_role_default_is_an_empty_list() -> None:
    assert yaml.safe_load((ROLE_DIR / "defaults" / "main.yml").read_text())["network_link_ethernets"] == []
