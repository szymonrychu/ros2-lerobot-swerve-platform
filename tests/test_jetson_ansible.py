"""The Jetson as an Ansible host: its own inventory group and group_vars, kept out of every play that has Pi roles, and
a `jetson` target in scripts/deploy-nodes.sh. Nothing here runs ansible-playbook: the script is run against a stub
ansible-playbook that records its arguments."""

import os
import re
import shutil
import stat
import subprocess
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
DEPLOY_SCRIPT = REPO_ROOT / "scripts" / "deploy-nodes.sh"
PI_ROLES = {"common", "network", "hostname", "system_optimize"}
JETSON_NON_NODE_TARGETS = ("base", "build_deps", "depthd")


def load(path: Path) -> Any:
    """Parse a YAML file.

    Args:
        path: File to parse.

    Returns:
        The parsed document.
    """
    return yaml.safe_load(path.read_text())


def role_names(play: dict[str, Any]) -> set[str]:
    """Names of the roles a play lists, in either the string or the mapping form.

    Args:
        play: One parsed play.

    Returns:
        The role names.
    """
    return {r if isinstance(r, str) else r.get("role", r.get("name", "")) for r in play.get("roles", [])}


def host_patterns(hosts: Any) -> set[str]:
    """Split an Ansible hosts pattern into its group names.

    Args:
        hosts: The play's `hosts` value (string or list).

    Returns:
        The patterns, split on `:` and `,`.
    """
    items = hosts if isinstance(hosts, list) else [hosts]
    return {p.strip() for item in items for p in re.split(r"[:,]", str(item)) if p.strip()}


def test_inventory_example_has_a_jetson_group_with_the_nano() -> None:
    text = (ANSIBLE_DIR / "inventory.example").read_text()
    assert re.search(r"^\[jetson\]\njetson-nano ansible_host=192\.168\.1\.32$", text, re.M)


def test_inventory_example_says_the_jetson_keeps_wifi() -> None:
    text = (ANSIBLE_DIR / "inventory.example").read_text()
    before_group = text.split("[jetson]")[0].splitlines()
    comment = [ln for ln in before_group if ln.startswith("#")][-1].lower()
    assert "jetson" in comment and "wifi" in comment and "development" in comment


def test_jetson_group_vars_pin_user_python_hostname_and_network() -> None:
    gv = load(ANSIBLE_DIR / "group_vars" / "jetson.yml")
    assert gv["ansible_user"] == "jetson"
    assert gv["ansible_python_interpreter"] == "/usr/bin/python3"
    assert gv["hostname"] == "nano"
    assert gv["network_address"] == "192.168.1.32/24"
    assert gv["network_gateway"] == "192.168.1.1"


def test_jetson_nameservers_match_the_client() -> None:
    jetson = load(ANSIBLE_DIR / "group_vars" / "jetson.yml")
    client = load(ANSIBLE_DIR / "group_vars" / "client.yml")
    assert jetson["network_nameservers"] == client["network_nameservers"]
    assert jetson["primary_dns_server"] == client["primary_dns_server"]
    assert jetson["secondary_dns_server"] == client["secondary_dns_server"]


def test_jetson_group_vars_header_says_no_ros_and_no_pi_roles() -> None:
    header = (ANSIBLE_DIR / "group_vars" / "jetson.yml").read_text().split("\n\n")[0].lower()
    assert "no ros" in header and "no pi roles" in header


def test_jetson_group_vars_have_no_ros2_nodes() -> None:
    assert "ros2_nodes" not in load(ANSIBLE_DIR / "group_vars" / "jetson.yml")


@pytest.mark.parametrize("playbook", ["site.yml", "playbooks/optimize.yml"])
def test_provision_plays_target_server_and_client_only(playbook: str) -> None:
    plays = [p for p in load(ANSIBLE_DIR / playbook) if "hosts" in p and role_names(p) & PI_ROLES]
    assert plays
    for play in plays:
        assert host_patterns(play["hosts"]) == {"server", "client"}


def test_no_play_with_pi_roles_has_hosts_all_or_jetson() -> None:
    checked = 0
    for path in ANSIBLE_DIR.rglob("*.yml"):
        if "roles" in path.relative_to(ANSIBLE_DIR).parts or "group_vars" in path.parts:
            continue
        doc = load(path)
        if not isinstance(doc, list):
            continue
        for play in doc:
            if isinstance(play, dict) and "hosts" in play and role_names(play) & PI_ROLES:
                checked += 1
                assert not host_patterns(play["hosts"]) & {"all", "jetson"}, f"{path}: {play.get('name')}"
    assert checked >= 2


def test_deploy_script_uses_no_associative_arrays() -> None:
    assert "declare -A" not in DEPLOY_SCRIPT.read_text()


def test_deploy_script_header_usage_and_errors_list_the_jetson_targets() -> None:
    text = DEPLOY_SCRIPT.read_text()
    assert "client | server | jetson" in text
    assert "jetson_link" in text
    for target in JETSON_NON_NODE_TARGETS:
        assert target in text
    assert "<client|server|jetson>" in text
    assert "Use: client, server or jetson" in text


@pytest.fixture
def stubbed_script(tmp_path: Path) -> tuple[Path, Path]:
    """The deploy script in a scratch tree with minimal group_vars and a recording stub ansible-playbook.

    Args:
        tmp_path: Scratch directory.

    Returns:
        The script path and the file the stub appends its argv to.
    """
    (tmp_path / "scripts").mkdir()
    (tmp_path / "ansible" / "group_vars").mkdir(parents=True)
    script = tmp_path / "scripts" / "deploy-nodes.sh"
    shutil.copy(DEPLOY_SCRIPT, script)
    for target in ("client", "server", "jetson"):
        shutil.copy(ANSIBLE_DIR / "group_vars" / f"{target}.yml", tmp_path / "ansible" / "group_vars" / f"{target}.yml")
    record = tmp_path / "argv.txt"
    stub = tmp_path / "bin" / "ansible-playbook"
    stub.parent.mkdir()
    stub.write_text(f'#!/bin/sh\necho "$@" >> {record}\n')
    stub.chmod(stub.stat().st_mode | stat.S_IEXEC)
    return script, record


def run_script(script: Path, *args: str) -> subprocess.CompletedProcess[str]:
    """Run the deploy script with the stub ansible-playbook first on PATH.

    Args:
        script: The copied script.
        *args: Script arguments.

    Returns:
        The completed process.
    """
    env = {**os.environ, "PATH": f"{script.parent.parent / 'bin'}:{os.environ['PATH']}"}
    return subprocess.run(["bash", str(script), *args], capture_output=True, text=True, env=env, check=False)


@pytest.mark.parametrize("name", JETSON_NON_NODE_TARGETS)
def test_jetson_non_node_targets_run_the_jetson_playbook(stubbed_script: tuple[Path, Path], name: str) -> None:
    script, record = stubbed_script
    result = run_script(script, "jetson", name)
    assert result.returncode == 0, result.stderr
    assert record.read_text().strip() == f"-i inventory playbooks/deploy_nodes_jetson.yml -l jetson --tags {name}"


def test_jetson_accepts_several_targets_in_one_run(stubbed_script: tuple[Path, Path]) -> None:
    script, record = stubbed_script
    assert run_script(script, "jetson", "base", "depthd").returncode == 0
    assert record.read_text().strip().endswith("-l jetson --tags base,depthd")


def test_jetson_all_runs_without_a_tag_filter(stubbed_script: tuple[Path, Path]) -> None:
    script, record = stubbed_script
    assert run_script(script, "jetson", "--all").returncode == 0
    assert record.read_text().strip() == "-i inventory playbooks/deploy_nodes_jetson.yml -l jetson"


def test_jetson_rejects_a_ros2_node_name(stubbed_script: tuple[Path, Path]) -> None:
    script, _ = stubbed_script
    result = run_script(script, "jetson", "web_ui")
    assert result.returncode == 1
    assert "not a ros2_nodes entry" in result.stderr
    for name in JETSON_NON_NODE_TARGETS:
        assert name in result.stderr


def test_jetson_link_is_a_client_target_only(stubbed_script: tuple[Path, Path]) -> None:
    script, record = stubbed_script
    assert run_script(script, "client", "jetson_link").returncode == 0
    assert record.read_text().strip().endswith("playbooks/deploy_nodes_client.yml -l client --tags jetson_link")
    assert run_script(script, "server", "jetson_link").returncode == 1
    assert run_script(script, "jetson", "jetson_link").returncode == 1


def test_unknown_target_error_lists_jetson(stubbed_script: tuple[Path, Path]) -> None:
    script, _ = stubbed_script
    result = run_script(script, "robot", "base")
    assert result.returncode == 1
    assert "jetson" in result.stderr
