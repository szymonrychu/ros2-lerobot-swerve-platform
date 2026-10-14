"""The jetson_link role (design D8): chrony with the Pi as server and the Jetson as client, Pi NAT for the Jetson, hosts
entries, and the tagged client play that applies the Pi side. Templates are rendered with jinja2 and the tasks are
checked by structure; nothing here runs ansible."""

from pathlib import Path
from typing import Any

import jinja2
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "jetson_link"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
LINK_VARS: dict[str, Any] = {
    "jetson_link_interface": "eth0",
    "jetson_link_subnet": "10.77.0.0/30",
    "jetson_link_pi_ip": "10.77.0.1",
    "jetson_link_jetson_ip": "10.77.0.2",
    "jetson_link_ntp_pools": ["2.pool.ntp.org", "ntp.ubuntu.com"],
    "_jetson_link_wan_interface": "wlan0",
}


def render(template: str, side: str = "server") -> str:
    """Render one role template.

    Args:
        template: Template file name.
        side: The jetson_link_side value.

    Returns:
        The rendered text.
    """
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(ROLE_DIR / "templates"),
        trim_blocks=True,
        lstrip_blocks=True,
        undefined=jinja2.StrictUndefined,
    )
    return env.get_template(template).render(**LINK_VARS, jetson_link_side=side)


def lines(text: str) -> list[str]:
    """Non-comment, non-blank lines with whitespace collapsed.

    Args:
        text: Rendered config.

    Returns:
        The meaningful lines.
    """
    return [" ".join(raw.split()) for raw in text.splitlines() if raw.strip() and not raw.strip().startswith("#")]


def walk(tasks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Flatten tasks, descending into blocks.

    Args:
        tasks: Parsed task list.

    Returns:
        Every leaf task.
    """
    out: list[dict[str, Any]] = []
    for task in tasks:
        if "block" in task:
            out.extend(walk(task["block"]))
        else:
            out.append(task)
    return out


@pytest.fixture(scope="module")
def tasks() -> list[dict[str, Any]]:
    """The role's leaf tasks.

    Returns:
        Flattened tasks of tasks/main.yml.
    """
    return walk(yaml.safe_load((ROLE_DIR / "tasks" / "main.yml").read_text()))


def module_of(task: dict[str, Any]) -> str:
    """The fully qualified module a task calls.

    Args:
        task: One leaf task.

    Returns:
        The module key.
    """
    skip = {"name", "when", "register", "become", "changed_when", "failed_when", "tags", "notify", "loop", "vars"}
    return next(k for k in task if k not in skip)


def test_server_chrony_keeps_pool_serves_the_link_and_works_offline() -> None:
    got = lines(render("chrony.conf.j2", "server"))
    assert "pool 2.pool.ntp.org iburst" in got
    assert "pool ntp.ubuntu.com iburst" in got
    assert "allow 10.77.0.0/30" in got
    assert "local stratum 10" in got
    assert not any(line.startswith("server 10.77.0.1") for line in got)


def test_client_chrony_prefers_the_pi_and_falls_back_to_the_pool() -> None:
    got = lines(render("chrony.conf.j2", "client"))
    assert "server 10.77.0.1 iburst prefer" in got
    assert "pool 2.pool.ntp.org iburst" in got
    assert "makestep 1.0 3" in got
    assert not any(line.startswith(("allow", "local stratum")) for line in got)


def test_server_chrony_also_steps_on_a_large_offset() -> None:
    assert "makestep 1.0 3" in lines(render("chrony.conf.j2", "server"))


def test_nat_masquerades_the_link_out_of_the_primary_interface() -> None:
    got = lines(render("jetson-nat.nft.j2"))
    assert 'iifname "eth0" oifname "wlan0" accept' in got
    assert 'oifname "wlan0" ip saddr 10.77.0.0/30 masquerade' in got
    assert "type nat hook postrouting priority srcnat; policy accept;" in got


def test_nat_forwards_only_between_the_link_and_the_primary_interface() -> None:
    got = lines(render("jetson-nat.nft.j2"))
    assert 'iifname "wlan0" oifname "eth0" ct state established,related accept' in got
    assert 'iifname "eth0" drop' in got
    assert 'oifname "eth0" drop' in got
    assert got.index('iifname "eth0" oifname "wlan0" accept') < got.index('iifname "eth0" drop')


def test_nat_table_is_replaced_not_appended() -> None:
    got = lines(render("jetson-nat.nft.j2"))
    assert got[:2] == ["table ip jetson_link {}", "delete table ip jetson_link"]


def test_every_service_restart_is_conditional_on_a_registered_change(tasks: list[dict[str, Any]]) -> None:
    registered = {t["register"] for t in tasks if "register" in t}
    restarts = [
        t for t in tasks if (t[module_of(t)] if isinstance(t[module_of(t)], dict) else {}).get("state") == "restarted"
    ]
    assert restarts
    for task in restarts:
        conds = task["when"] if isinstance(task.get("when"), list) else [task.get("when", "")]
        changed = [c for c in conds if str(c).endswith(" is changed")]
        assert changed, task["name"]
        assert all(str(c).split(" is changed")[0].split(" or ")[0].strip() in registered for c in changed), task["name"]


def test_commands_run_only_when_something_changed(tasks: list[dict[str, Any]]) -> None:
    for task in tasks:
        if module_of(task) == "ansible.builtin.command":
            assert "is changed" in str(task.get("when")), task["name"]


def test_timesyncd_is_disabled_before_chrony_is_installed(tasks: list[dict[str, Any]]) -> None:
    names = [t["name"] for t in tasks]
    timesyncd = next(i for i, t in enumerate(tasks) if "systemd-timesyncd" in str(t))
    chrony = next(i for i, t in enumerate(tasks) if module_of(t) == "ansible.builtin.apt" and "chrony" in str(t))
    assert timesyncd < chrony, names
    task = tasks[timesyncd]
    assert task["ansible.builtin.systemd"]["enabled"] is False
    assert task["ansible.builtin.systemd"]["state"] == "stopped"


def test_chrony_is_enabled_and_started(tasks: list[dict[str, Any]]) -> None:
    assert any(
        module_of(t) == "ansible.builtin.systemd"
        and t[module_of(t)].get("name") == "chrony"
        and t[module_of(t)].get("enabled") is True
        for t in tasks
    )


def test_server_enables_ipv4_forwarding_through_a_sysctl_dropin(tasks: list[dict[str, Any]]) -> None:
    drop = next(t for t in tasks if "/etc/sysctl.d/" in str(t) and module_of(t) == "ansible.builtin.copy")
    assert "net.ipv4.ip_forward = 1" in drop["ansible.builtin.copy"]["content"]
    assert "server" in str(drop["when"])


def test_server_installs_the_nat_table_and_loads_it_on_boot(tasks: list[dict[str, Any]]) -> None:
    tmpl = next(t for t in tasks if t.get("ansible.builtin.template", {}).get("src") == "jetson-nat.nft.j2")
    assert tmpl["ansible.builtin.template"]["dest"].startswith("/etc/nftables.d/")
    assert any("nftables.conf" in str(t) and "include" in str(t) for t in tasks)
    assert any(module_of(t) == "ansible.builtin.systemd" and t[module_of(t)].get("name") == "nftables" for t in tasks)


def test_hosts_entries_per_side(tasks: list[dict[str, Any]]) -> None:
    defaults = yaml.safe_load((ROLE_DIR / "defaults" / "main.yml").read_text())
    env = jinja2.Environment()
    hosts = [t for t in tasks if t.get("ansible.builtin.lineinfile", {}).get("path") == "/etc/hosts"]
    by_side = {
        ("server" if "server" in str(t["when"]) else "client"): env.from_string(
            t["ansible.builtin.lineinfile"]["line"]
        ).render({**defaults, **LINK_VARS})
        for t in hosts
    }
    assert by_side == {"server": "10.77.0.2 jetson.ros2.lan", "client": "10.77.0.1 client.ros2.lan"}


def test_side_is_validated(tasks: list[dict[str, Any]]) -> None:
    defaults = yaml.safe_load((ROLE_DIR / "defaults" / "main.yml").read_text())
    assert defaults["jetson_link_side"] == "server"
    assert defaults["jetson_link_subnet"] == "10.77.0.0/30"
    assert any(module_of(t) == "ansible.builtin.assert" and "jetson_link_side" in str(t) for t in tasks)


def link_block() -> dict[str, Any]:
    """The client play's 'Direct link to the Jetson' block.

    Returns:
        The parsed block task.
    """
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]

    def find(items: list[dict[str, Any]]) -> dict[str, Any] | None:
        for item in items:
            if item.get("name") == "Direct link to the Jetson":
                return item
            if "block" in item and (hit := find(item["block"])):
                return hit
        return None

    hit = find(play["tasks"])
    assert hit is not None
    return hit


def test_client_play_applies_network_then_jetson_link_under_one_tag() -> None:
    block = link_block()
    assert block["tags"] == ["jetson_link"]
    inner = block["block"]
    assert [t["ansible.builtin.include_role"]["name"] for t in inner] == ["network", "jetson_link"]
    for task in inner:
        assert task["ansible.builtin.include_role"]["apply"]["tags"] == ["jetson_link"]
    assert inner[1]["vars"]["jetson_link_side"] == "server"


def test_no_other_client_task_carries_the_jetson_link_tag() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert text.count("jetson_link]") == 3
    assert "jetson_link" not in str(yaml.safe_load(text)[0]["pre_tasks"])
