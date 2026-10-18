"""The client chrony role: plain internet pools, no Jetson serving. The template is rendered with jinja2 and the
tasks are checked by structure; nothing here runs ansible."""

from pathlib import Path
from typing import Any

import jinja2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "chrony"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "client.yml"
FORBIDDEN = ("allow", "local stratum", "server ", "10.77")


def load_yaml(path: Path) -> Any:
    """Load a YAML file.

    Args:
        path: File to read.

    Returns:
        The parsed document.
    """
    return yaml.safe_load(path.read_text())


def render() -> list[str]:
    """Render chrony.conf.j2 with the role defaults.

    Returns:
        The non-empty rendered lines.
    """
    defaults = load_yaml(ROLE_DIR / "defaults" / "main.yml")
    text = jinja2.Template((ROLE_DIR / "templates" / "chrony.conf.j2").read_text()).render(**defaults)
    return [line.strip() for line in text.splitlines() if line.strip()]


def tasks() -> list[dict[str, Any]]:
    """Return the role's tasks.

    Returns:
        Task dicts from tasks/main.yml.
    """
    return load_yaml(ROLE_DIR / "tasks" / "main.yml")


def test_chrony_conf_is_plain_pools() -> None:
    """Rendered conf has one pool line per pool, the base directives, and no serving lines."""
    lines = render()
    pools = load_yaml(ROLE_DIR / "defaults" / "main.yml")["chrony_ntp_pools"]
    assert pools
    for pool in pools:
        assert f"pool {pool} iburst" in lines
    for line in (
        "driftfile /var/lib/chrony/chrony.drift",
        "ntsdumpdir /var/lib/chrony",
        "logdir /var/log/chrony",
        "rtcsync",
        "makestep 1.0 3",
    ):
        assert line in lines
    for line in lines:
        if line.startswith("#"):
            continue
        for bad in FORBIDDEN:
            assert bad not in line, line


def test_tasks_guard_timesyncd_and_restart_on_change() -> None:
    """timesyncd is stopped only when its unit exists; chrony restarts only on template change."""
    all_tasks = tasks()
    assert any("ansible.builtin.service_facts" in t for t in all_tasks)
    stops = [t for t in all_tasks if "systemd-timesyncd" in str(t.get("ansible.builtin.systemd", {}).get("name", ""))]
    assert stops
    for task in stops:
        assert "systemd-timesyncd" in str(task.get("when", ""))

    # The restart runs only when the registered template task changed.
    template = next(t for t in all_tasks if "ansible.builtin.template" in t)
    assert template["ansible.builtin.template"]["dest"] == "/etc/chrony/chrony.conf"
    var = template["register"]
    restart = next(t for t in all_tasks if t.get("ansible.builtin.systemd", {}).get("state") == "restarted")
    assert f"{var} is changed" in restart["when"]


def test_client_provisioning_applies_chrony() -> None:
    """playbooks/client.yml applies the chrony role."""
    roles = load_yaml(CLIENT_PLAYBOOK)[0]["roles"]
    names = [r if isinstance(r, str) else r.get("role") for r in roles]
    assert "chrony" in names
