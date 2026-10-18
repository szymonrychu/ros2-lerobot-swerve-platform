"""Removal of the retired Pi-Jetson link from the client (tasks/jetson_link_remove.yml under the jetson_link tag).
Static checks only: the task file and the client playbook are read as YAML; nothing runs ansible or touches a host."""

import re
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
REMOVE_TASKS = ANSIBLE_DIR / "playbooks" / "tasks" / "jetson_link_remove.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
ALL_VARS = ANSIBLE_DIR / "group_vars" / "all.yml"
BLOCK_NAME = "Remove the retired Jetson link"
# The regexp the jetson_link role's hosts task rendered for jetson.ros2.lan.
HOSTS_REGEXP = r"\sjetson\.ros2\.lan(\s|$)"
SYSCTL_DROPIN = "/etc/sysctl.d/90-jetson-link-forward.conf"
NAT_FILE = "/etc/nftables.d/jetson-nat.nft"
NFT_DIR = "/etc/nftables.d"
NFT_INCLUDE = 'include "/etc/nftables.d/*.nft"'
TASK_KEYS = {"name", "when", "register", "become", "changed_when", "failed_when", "tags", "notify", "loop", "vars"}


def module_of(task: dict[str, Any]) -> str:
    """The fully qualified module a task calls.

    Args:
        task: One leaf task.

    Returns:
        The module key.
    """
    return next(k for k in task if k not in TASK_KEYS)


def args_of(task: dict[str, Any]) -> dict[str, Any]:
    """The module arguments of a task.

    Args:
        task: One leaf task.

    Returns:
        The argument mapping (a free-form string becomes {"cmd": ...}).
    """
    args = task[module_of(task)]
    return args if isinstance(args, dict) else {"cmd": args}


def when_of(task: dict[str, Any]) -> str:
    """A task's `when`, joined into one string.

    Args:
        task: One leaf task.

    Returns:
        The condition text ("" when unconditional).
    """
    when = task.get("when", "")
    return " and ".join(when) if isinstance(when, list) else str(when)


@pytest.fixture(scope="module")
def tasks() -> list[dict[str, Any]]:
    """The removal task file.

    Returns:
        Its parsed task list (flat, no blocks).
    """
    data = yaml.safe_load(REMOVE_TASKS.read_text())
    assert isinstance(data, list)
    assert not any("block" in t for t in data)
    return data


def task_on(tasks: list[dict[str, Any]], module: str, **match: str) -> dict[str, Any]:
    """The single task calling a module with these argument values.

    Args:
        tasks: Parsed task list.
        module: Fully qualified module name.
        **match: Argument values the task must carry.

    Returns:
        The matching task.
    """
    hits = [t for t in tasks if module_of(t) == module and all(args_of(t).get(k) == v for k, v in match.items())]
    assert len(hits) == 1, f"expected one {module} task with {match}, got {len(hits)}"
    return hits[0]


def nft_task(tasks: list[dict[str, Any]], cmd: str) -> dict[str, Any]:
    """The command task running this nft command.

    Args:
        tasks: Parsed task list.
        cmd: The command line.

    Returns:
        The matching task.
    """
    return task_on(tasks, "ansible.builtin.command", cmd=cmd)


def link_block() -> dict[str, Any]:
    """The client play's jetson_link block.

    Returns:
        The parsed block task.
    """
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]

    def find(items: list[dict[str, Any]]) -> dict[str, Any] | None:
        for item in items:
            if item.get("name") == BLOCK_NAME:
                return item
            if "block" in item and (hit := find(item["block"])):
                return hit
        return None

    hit = find(play["tasks"])
    assert hit is not None, f"no '{BLOCK_NAME}' block in deploy_nodes_client.yml"
    return hit


def test_removal_tasks_are_state_based_and_guarded(tasks: list[dict[str, Any]]) -> None:
    hosts = task_on(tasks, "ansible.builtin.lineinfile", path="/etc/hosts")
    assert args_of(hosts)["regexp"] == HOSTS_REGEXP
    assert args_of(hosts)["state"] == "absent"

    for path in (SYSCTL_DROPIN, NAT_FILE):
        assert args_of(task_on(tasks, "ansible.builtin.file", path=path))["state"] == "absent"

    include = task_on(tasks, "ansible.builtin.lineinfile", path="/etc/nftables.conf")
    assert args_of(include)["line"] == NFT_INCLUDE
    assert args_of(include)["state"] == "absent"
    assert args_of(include).get("create", False) is False

    listing = nft_task(tasks, "nft list tables")
    assert listing["changed_when"] is False
    assert listing["failed_when"] is False  # a client without the nft binary is not an error
    delete = nft_task(tasks, "nft delete table ip jetson_link")
    assert "table ip jetson_link" in when_of(delete)
    assert listing["register"] in when_of(delete)
    assert "default([])" in when_of(delete)  # a missing binary leaves no stdout_lines

    scan = task_on(tasks, "ansible.builtin.find", paths=NFT_DIR)
    assert args_of(scan)["file_type"] == "any"
    assert args_of(scan)["hidden"] is True
    rmdir = task_on(tasks, "ansible.builtin.file", path=NFT_DIR)
    assert args_of(rmdir)["state"] == "absent"
    assert f"{scan['register']}.matched == 0" in when_of(rmdir)

    names = [t["name"] for t in tasks]
    assert names.index(delete["name"]) > names.index(listing["name"])
    assert (
        names.index(rmdir["name"])
        > names.index(scan["name"])
        > names.index(task_on(tasks, "ansible.builtin.file", path=NAT_FILE)["name"])
    )


def test_hosts_regexp_matches_only_the_jetson_line() -> None:
    pattern = re.compile(HOSTS_REGEXP)
    assert pattern.search("10.77.0.2 jetson.ros2.lan")
    assert pattern.search("10.77.0.2 jetson.ros2.lan # comment")
    assert not pattern.search("127.0.1.1 client.ros2.lan")
    assert not pattern.search("10.0.0.9 myjetson.ros2.lan")
    assert not pattern.search("10.0.0.9 jetson.ros2.lan.example")


def test_ip_forward_reset_is_guarded(tasks: list[dict[str, Any]]) -> None:
    names = [t["name"] for t in tasks]
    dropin = task_on(tasks, "ansible.builtin.file", path=SYSCTL_DROPIN)
    facts = next(t for t in tasks if module_of(t) == "ansible.builtin.service_facts")
    reset = [t for t in tasks if "ip_forward" in str(args_of(t))]
    assert len(reset) == 1
    reset_task = reset[0]
    # Live only: sysctl -w, never a persisted sysctl.conf entry.
    assert module_of(reset_task) == "ansible.builtin.command"
    assert args_of(reset_task)["cmd"] == "sysctl -w net.ipv4.ip_forward=0"
    when = when_of(reset_task)
    assert f"{dropin['register']} is changed" in when
    assert "docker.service" in when and "running" in when and "!=" in when
    assert names.index(reset_task["name"]) > max(names.index(dropin["name"]), names.index(facts["name"]))


def test_every_task_is_registered_and_changes_only_on_removal(tasks: list[dict[str, Any]]) -> None:
    registers = [t.get("register") for t in tasks]
    assert all(registers), "every removal task registers its result"
    assert len(set(registers)) == len(registers)
    for task in tasks:
        module = module_of(task)
        if module == "ansible.builtin.command":
            # A command changes only when it runs, and it runs only behind a guard (or never reports a change).
            assert task.get("changed_when") is False or (task.get("changed_when") is True and when_of(task))
        elif module in ("ansible.builtin.lineinfile", "ansible.builtin.file"):
            assert args_of(task)["state"] == "absent"
        else:
            assert module in ("ansible.builtin.service_facts", "ansible.builtin.find")


def test_nftables_package_and_service_are_left_alone(tasks: list[dict[str, Any]]) -> None:
    for task in tasks:
        module = module_of(task)
        assert module not in (
            "ansible.builtin.apt",
            "ansible.builtin.package",
            "ansible.builtin.systemd",
            "ansible.builtin.systemd_service",
            "ansible.builtin.service",
        )
    text = REMOVE_TASKS.read_text()
    assert "nftables.service" not in text
    assert "nft flush ruleset" not in text


def test_client_block_runs_network_chrony_and_removal() -> None:
    block = link_block()
    assert block["tags"] == ["jetson_link"]
    assert "when" not in block  # an untagged client run applies it as well
    inner = block["block"]
    assert len(inner) == 3
    assert inner[0]["ansible.builtin.include_role"]["name"] == "network"
    assert inner[1]["ansible.builtin.include_role"]["name"] == "chrony"
    assert inner[2]["ansible.builtin.include_tasks"]["file"] == "tasks/jetson_link_remove.yml"
    for task in inner:
        args = task.get("ansible.builtin.include_role") or task["ansible.builtin.include_tasks"]
        assert args["apply"]["tags"] == ["jetson_link"]
        assert "vars" not in task


def test_no_other_client_task_carries_the_jetson_link_tag() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert text.count("jetson_link]") == 4  # the block and its three applies
    assert "Direct link to the Jetson" not in text
    assert "jetson_link_side" not in text
    assert "jetson_link" not in str(yaml.safe_load(text)[0]["pre_tasks"])


def test_role_and_link_vars_are_gone() -> None:
    assert not (ANSIBLE_DIR / "roles" / "jetson_link").exists()
    assert not (REPO_ROOT / "tests" / "test_jetson_link_role.py").exists()
    text = ALL_VARS.read_text()
    assert "jetson_link_" not in text
    assert "D8" not in text
    data = yaml.safe_load(text)
    assert not any(key.startswith("jetson_link") for key in data)
    assert data.get("network_link_ethernets", []) == []
