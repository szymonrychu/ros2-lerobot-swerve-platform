"""Shared helpers of the monitoring tests: role paths, per-host variables, template rendering and task walking.

The monitoring roles are split by host:
- roles/monitoring_agent (both hosts): Alloy, monitoring.slice, the RPi throttling timer, robot cleanup.
- roles/monitoring (server): Mimir, Loki, Grafana and the dashboards.
- roles/web_proxy (both hosts): nginx on port 80 (robot: web UI + /grafana/ redirect, server: Grafana).
Per-host values come from the monitoring-only keys of ansible/group_vars/<host>.yml on top of the role defaults.
"""

import os
import shutil
import subprocess
from pathlib import Path

import jinja2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLES_DIR = ANSIBLE_DIR / "roles"
AGENT_DIR = ROLES_DIR / "monitoring_agent"
STACK_DIR = ROLES_DIR / "monitoring"
PROXY_DIR = ROLES_DIR / "web_proxy"
DASHBOARDS_DIR = STACK_DIR / "files"
GROUP_VARS_DIR = ANSIBLE_DIR / "group_vars"
PLAYBOOKS_DIR = ANSIBLE_DIR / "playbooks"
SELECT_RUN = PLAYBOOKS_DIR / "tasks" / "select_run.yml"
DEPLOY_SCRIPT = REPO_ROOT / "scripts" / "deploy-nodes.sh"
HOSTS = ("client", "server")
# group_vars keys that belong to the monitoring roles (the rest of group_vars is the ROS2 deploy).
MONITORING_KEY_PREFIXES = ("monitoring_", "web_proxy_")


def load_yaml(path: Path) -> object:
    """Parse one YAML file.

    Args:
        path (Path): File to read.

    Returns:
        object: The parsed document.
    """
    return yaml.safe_load(path.read_text())


def role_defaults(role_dir: Path) -> dict:
    """Return a role's defaults/main.yml.

    Args:
        role_dir (Path): Role directory.

    Returns:
        dict: Variable name to value.
    """
    return load_yaml(role_dir / "defaults" / "main.yml") or {}


def group_monitoring_vars(host: str) -> dict:
    """Return the monitoring-only keys of group_vars/<host>.yml.

    Args:
        host (str): client or server.

    Returns:
        dict: monitoring_* and web_proxy_* keys.
    """
    data = load_yaml(GROUP_VARS_DIR / f"{host}.yml")
    assert isinstance(data, dict)
    return {k: v for k, v in data.items() if k.startswith(MONITORING_KEY_PREFIXES)}


def host_vars(host: str) -> dict:
    """Variables a monitoring role sees on one host: every monitoring role's defaults, then group_vars (which win).

    Args:
        host (str): client or server.

    Returns:
        dict: Merged variables.
    """
    merged: dict = {}
    for role_dir in (AGENT_DIR, STACK_DIR, PROXY_DIR):
        merged.update(role_defaults(role_dir))
    merged.update(group_monitoring_vars(host))
    return merged


def jinja_env(loader: jinja2.BaseLoader | None = None) -> jinja2.Environment:
    """A jinja2 environment that behaves like ansible.builtin.template for our templates.

    Args:
        loader (jinja2.BaseLoader | None): Template loader, when templates are loaded from files.

    Returns:
        jinja2.Environment: Strict environment with the Ansible filters our templates and tasks use.
    """
    env = jinja2.Environment(
        loader=loader,
        undefined=jinja2.StrictUndefined,
        keep_trailing_newline=True,
        trim_blocks=True,  # as ansible.builtin.template
    )
    env.globals["ansible_managed"] = "Managed by Ansible"
    env.filters["basename"] = os.path.basename
    env.filters["bool"] = lambda value: str(value).lower() in ("true", "yes", "1")
    return env


def render(role_dir: Path, template: str, host: str, **extra: object) -> str:
    """Render one role template with the host's variables.

    Args:
        role_dir (Path): Role directory.
        template (str): File name under templates/.
        host (str): client or server.
        **extra (object): Variables added on top (e.g. item for a loop).

    Returns:
        str: Rendered text.
    """
    env = jinja_env(jinja2.FileSystemLoader(role_dir / "templates"))
    return env.get_template(template).render(**{**host_vars(host), "ansible_machine": "aarch64", **extra})


def resolve(value: object, host: str, item: object = None) -> object:
    """Render a task value (a templated string, or a list of them) with the host's variables.

    Args:
        value (object): Jinja2 string or list of strings.
        host (str): client or server.
        item (object): Loop item, when the task loops.

    Returns:
        object: Rendered value (a list stays a list).
    """
    if isinstance(value, list):
        return [resolve(v, host, item) for v in value]
    if not isinstance(value, str):
        return value
    env = jinja_env()
    return env.from_string(value).render(**host_vars(host), item=item, ansible_machine="aarch64")


def walk_tasks(tasks: list[dict]) -> list[dict]:
    """Flatten a task list, descending into block/rescue/always.

    Args:
        tasks (list[dict]): Tasks.

    Returns:
        list[dict]: Every task (blocks included) in file order.
    """
    out: list[dict] = []
    for task in tasks or []:
        out.append(task)
        for key in ("block", "rescue", "always"):
            out.extend(walk_tasks(task.get(key, [])))
    return out


def role_tasks(role_dir: Path, files: tuple[str, ...] | None = None) -> list[dict]:
    """Return every task of a role's task files, flattening blocks.

    Args:
        role_dir (Path): Role directory.
        files (tuple[str, ...] | None): Task file names to read (default: all of tasks/*.yml, sorted).

    Returns:
        list[dict]: Tasks in file order.
    """
    paths = [role_dir / "tasks" / f for f in files] if files else sorted((role_dir / "tasks").glob("*.yml"))
    out: list[dict] = []
    for path in paths:
        out.extend(walk_tasks(load_yaml(path) or []))
    return out


def tasks_using(tasks: list[dict], module: str) -> list[dict]:
    """Tasks that call one module.

    Args:
        tasks (list[dict]): Tasks.
        module (str): Fully qualified module name.

    Returns:
        list[dict]: Matching tasks.
    """
    return [t for t in tasks if module in t]


def unit_settings(text: str) -> dict[str, list[str]]:
    """Collect Key=value lines of a systemd unit (all sections).

    Args:
        text (str): Unit file text.

    Returns:
        dict[str, list[str]]: Key to every value it was given.
    """
    out: dict[str, list[str]] = {}
    for line in text.splitlines():
        if "=" in line and not line.lstrip().startswith(("#", ";", "[")):
            key, value = line.split("=", 1)
            out.setdefault(key.strip(), []).append(value.strip())
    return out


def playbook_role_includes(host: str) -> list[dict]:
    """Every include_role task of playbooks/deploy_nodes_<host>.yml, in order.

    Args:
        host (str): client or server.

    Returns:
        list[dict]: include_role tasks.
    """
    play = load_yaml(PLAYBOOKS_DIR / f"deploy_nodes_{host}.yml")[0]
    tasks = walk_tasks(play.get("pre_tasks", []) + play.get("tasks", []) + play.get("post_tasks", []))
    return [t for t in tasks if "ansible.builtin.include_role" in t]


def docker_image_available(image: str) -> bool:
    """Whether docker runs here and already has an image (the real-binary checks never pull).

    Args:
        image (str): Image reference.

    Returns:
        bool: True when `docker image inspect` succeeds.
    """
    if shutil.which("docker") is None:
        return False
    proc = subprocess.run(["docker", "image", "inspect", image], capture_output=True, check=False)
    return proc.returncode == 0
