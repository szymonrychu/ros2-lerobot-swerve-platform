"""Server monitoring stack (roles/monitoring: Mimir + Loki + Grafana on the server RPi4) and the deploy integration."""

import configparser
import os
import re
import stat
import subprocess
from pathlib import Path

import jinja2
import pytest
import yaml

from tests.monitoring_helpers import (
    DASHBOARDS_DIR,
    DEPLOY_SCRIPT,
    GROUP_VARS_DIR,
    HOSTS,
    SELECT_RUN,
    STACK_DIR,
    load_yaml,
    playbook_role_includes,
    render,
    resolve,
    role_defaults,
    role_tasks,
    tasks_using,
    unit_settings,
)

TEMPLATES_DIR = STACK_DIR / "templates"
STACK_UNITS = {"mimir": "700M", "loki": "400M", "grafana-server": "768M"}
# Bundled datasource plugins Grafana must not start (each runs its own backend process; 13 of them used 302 MB and
# got Grafana OOM-killed on the robot, 2026-10-10). Prometheus (Mimir) and Loki are the two in use.
UNUSED_GRAFANA_PLUGINS = {
    "grafana-pyroscope-datasource", "influxdb", "stackdriver", "elasticsearch", "mssql", "tempo", "jaeger",
    "opentsdb", "zipkin", "mysql", "grafana-postgresql-datasource",
}  # fmt: skip


def stack(template: str, **extra: object) -> str:
    """Render one server stack template with the server's variables.

    Args:
        template (str): Template under roles/monitoring/templates.
        **extra (object): Extra variables.

    Returns:
        str: Rendered text.
    """
    return render(STACK_DIR, template, "server", **extra)


def tasks() -> list[dict]:
    """Every task of the server stack role.

    Returns:
        list[dict]: Tasks in file order.
    """
    return role_tasks(STACK_DIR)


def task_names() -> list[str]:
    """Names of the stack role's tasks, in order.

    Returns:
        list[str]: Task names.
    """
    return [t.get("name", "") for t in tasks()]


def mimir_config() -> dict:
    """Parse the rendered Mimir config.

    Returns:
        dict: Mimir config.
    """
    return yaml.safe_load(stack("mimir.yml.j2"))


def loki_config() -> dict:
    """Parse the rendered Loki config.

    Returns:
        dict: Loki config.
    """
    return yaml.safe_load(stack("loki.yml.j2"))


def grafana_ini() -> configparser.ConfigParser:
    """Parse the rendered grafana.ini.

    Returns:
        configparser.ConfigParser: Parsed ini.
    """
    parser = configparser.ConfigParser(interpolation=None)
    parser.read_string(stack("grafana.ini.j2"))
    return parser


# --- role structure ----------------------------------------------------------------------------------------------


def test_role_files_exist() -> None:
    """The stack role has tasks, defaults, handlers, meta, the Mimir/Loki/Grafana templates and the dashboards."""
    for rel in [
        "tasks/main.yml",
        "defaults/main.yml",
        "handlers/main.yml",
        "meta/main.yml",
        "templates/mimir.yml.j2",
        "templates/mimir.default.j2",
        "templates/loki.yml.j2",
        "templates/grafana.ini.j2",
        "templates/grafana-datasources.yaml.j2",
        "templates/grafana-dashboards.yaml.j2",
        "templates/monitoring-dropin.conf.j2",
        "files/robot-resources.json",
    ]:
        assert (STACK_DIR / rel).is_file(), rel


def test_robot_only_pieces_left_the_stack_role() -> None:
    """Alloy, Prometheus, the slice, throttling and nginx live in the agent and web_proxy roles now."""
    for gone in [
        "templates/prometheus.yml.j2",
        "templates/prometheus.default.j2",
        "templates/config.alloy.j2",
        "templates/monitoring.slice.j2",
        "templates/nginx-robot.conf.j2",
        "files/rpi_throttled_textfile.py",
    ]:
        assert not (STACK_DIR / gone).exists(), gone


def test_every_template_and_file_the_tasks_name_exists() -> None:
    """Every src: of a template/copy task names a file shipped with the role."""
    checked = 0
    for task in tasks():
        for module, folder in (("ansible.builtin.template", TEMPLATES_DIR), ("ansible.builtin.copy", DASHBOARDS_DIR)):
            src = (task.get(module) or {}).get("src")
            if not src:
                continue
            if "with_fileglob" in task:
                items: list = [str(p) for pattern in task["with_fileglob"] for p in sorted(folder.glob(pattern))]
                assert items, f"{task['name']}: {task['with_fileglob']} matches nothing"
            else:
                loop = task.get("loop", [None])
                items = yaml.safe_load(str(resolve(loop, "server"))) if isinstance(loop, str) else loop
            for item in items:
                assert (folder / str(resolve(src, "server", item))).is_file(), f"{task['name']}: {src}"
                checked += 1
    assert checked >= 10


def test_defaults_have_enable_flag_ports_retention_and_limits() -> None:
    """Enable flag, ports, 24 h retention and the per-unit limits live in defaults/main.yml."""
    defaults = role_defaults(STACK_DIR)
    assert defaults["monitoring_enabled"] is True
    assert defaults["monitoring_grafana_port"] == 3000
    assert defaults["monitoring_mimir_port"] == 9009
    assert defaults["monitoring_loki_port"] == 3100
    assert defaults["monitoring_mimir_retention"] == "24h"
    assert defaults["monitoring_loki_retention"] == "24h"
    assert defaults["monitoring_interval"] == "5s"
    assert {u["name"]: u["memory_max"] for u in defaults["monitoring_units"]} == STACK_UNITS


# --- packages ----------------------------------------------------------------------------------------------------


def test_packages_from_the_grafana_apt_repo() -> None:
    """Mimir, Loki and Grafana are the apt.grafana.com arm64 debs (repo tasks shared with the agent role)."""
    repo = [
        t["ansible.builtin.include_role"]
        for t in tasks_using(tasks(), "ansible.builtin.include_role")
        if t["ansible.builtin.include_role"].get("tasks_from") == "grafana_repo.yml"
    ]
    assert repo == [{"name": "monitoring_agent", "tasks_from": "grafana_repo.yml"}]
    apt = [t["ansible.builtin.apt"] for t in tasks_using(tasks(), "ansible.builtin.apt")]
    installs = [a for a in apt if a.get("state") == "present"]
    assert len(installs) == 1
    assert sorted(installs[0]["name"]) == ["grafana", "loki", "mimir"]
    assert installs[0]["install_recommends"] is False
    names = task_names()
    include = next(t["name"] for t in tasks() if t in tasks_using(tasks(), "ansible.builtin.include_role"))
    assert names.index(include) < names.index(next(t["name"] for t in tasks() if "ansible.builtin.apt" in t))


def test_data_directories_belong_to_the_service_users() -> None:
    """/var/lib/mimir and /var/lib/loki exist and belong to the package users (WAL and blocks live there)."""
    dirs = {
        resolve(t["ansible.builtin.file"]["path"], "server"): t["ansible.builtin.file"]
        for t in tasks_using(tasks(), "ansible.builtin.file")
        if t["ansible.builtin.file"].get("state") == "directory" and "loop" not in t
    }
    assert dirs["/var/lib/mimir"]["owner"] == "mimir"
    assert dirs["/var/lib/loki"]["owner"] == "loki"


# --- resource isolation ------------------------------------------------------------------------------------------


@pytest.mark.parametrize("unit", sorted(STACK_UNITS))
def test_dropin_moves_unit_into_the_slice(unit: str) -> None:
    """Each service runs in monitoring.slice, niced, memory capped and first in line for the OOM killer."""
    item = next(u for u in role_defaults(STACK_DIR)["monitoring_units"] if u["name"] == unit)
    settings = unit_settings(stack("monitoring-dropin.conf.j2", item=item))
    assert settings["Slice"] == ["monitoring.slice"]
    assert settings["Nice"] == ["10"]
    assert settings["MemoryMax"] == [STACK_UNITS[unit]]
    assert settings["OOMScoreAdjust"] == ["500"]


def test_dropins_are_installed_for_every_unit() -> None:
    """A template task loops over monitoring_units and writes /etc/systemd/system/<unit>.service.d/."""
    found = [t for t in tasks() if (t.get("ansible.builtin.template") or {}).get("src") == "monitoring-dropin.conf.j2"]
    assert len(found) == 1
    assert found[0]["loop"] == "{{ monitoring_units }}"
    assert "/etc/systemd/system/{{ item.name }}.service.d/" in found[0]["ansible.builtin.template"]["dest"]


# --- mimir -------------------------------------------------------------------------------------------------------


def test_mimir_is_one_monolithic_single_tenant_process() -> None:
    """-target=all, no tenants (Alloy sends no X-Scope-OrgID), HTTP on the LAN, gRPC on localhost only."""
    config = mimir_config()
    assert config["target"] == "all"
    assert config["multitenancy_enabled"] is False
    server = config["server"]
    assert (server["http_listen_address"], server["http_listen_port"]) == ("0.0.0.0", 9009)
    assert server["grpc_listen_address"] == "127.0.0.1"
    assert config["usage_stats"]["enabled"] is False


def test_mimir_stores_blocks_on_the_local_filesystem() -> None:
    """Filesystem blocks storage, TSDB/WAL, compactor and store-gateway dirs all under /var/lib/mimir."""
    config = mimir_config()
    assert config["common"]["storage"]["backend"] == "filesystem"
    assert config["common"]["storage"]["filesystem"]["dir"].startswith("/var/lib/mimir/")
    assert config["blocks_storage"]["tsdb"]["dir"].startswith("/var/lib/mimir/")
    assert config["blocks_storage"]["bucket_store"]["sync_dir"].startswith("/var/lib/mimir/")
    assert config["compactor"]["data_dir"].startswith("/var/lib/mimir/")


def test_mimir_rings_are_in_memory_on_localhost() -> None:
    """One replica, in-memory rings with 127.0.0.1 addresses (no LAN gossip), memberlist bound to localhost."""
    config = mimir_config()
    assert config["ingester"]["ring"]["replication_factor"] == 1
    assert config["store_gateway"]["sharding_ring"]["replication_factor"] == 1
    rings = [
        config["distributor"]["ring"],
        config["ingester"]["ring"],
        config["querier"]["ring"],
        config["compactor"]["sharding_ring"],
        config["store_gateway"]["sharding_ring"],
        config["ruler"]["ring"],
        config["query_scheduler"]["ring"],
    ]
    for ring in rings:
        assert ring["kvstore"]["store"] == "inmemory"
        assert ring["instance_addr"] == "127.0.0.1"
    assert config["frontend"]["address"] == "127.0.0.1"
    assert config["memberlist"]["bind_addr"] == ["127.0.0.1"]


def test_mimir_keeps_one_day_and_accepts_both_hosts() -> None:
    """Compactor drops blocks after 24 h; limits take ~10k series per host at 5 s from two hosts with headroom."""
    limits = mimir_config()["limits"]
    assert limits["compactor_blocks_retention_period"] == "24h"
    hosts, series_per_host, interval_s = 2, 10_000, 5
    assert limits["max_global_series_per_user"] >= 5 * hosts * series_per_host
    # Steady state is 4k samples/s; after an outage Alloy replays its WAL as fast as Mimir lets it.
    assert limits["ingestion_rate"] >= 10 * hosts * series_per_host / interval_s
    assert limits["ingestion_burst_size"] >= 10 * limits["ingestion_rate"]


def test_mimir_is_sized_for_the_pi4() -> None:
    """Small-RAM caps: the ingester refuses series past a bound and the querier runs few queries at once."""
    config = mimir_config()
    assert 0 < config["ingester"]["instance_limits"]["max_series"] <= 200_000
    assert 0 < config["querier"]["max_concurrent"] <= 4


def test_mimir_service_flags() -> None:
    """/etc/default/mimir (read by the deb's unit next to --config.file=/etc/mimir/config.yml): warn-level logs."""
    settings = unit_settings(stack("mimir.default.j2"))
    assert settings["LOG_LEVEL"] == ['"warn"']
    assert settings["RESTART_ON_UPGRADE"] == ['"true"']
    config_task = next(
        t["ansible.builtin.template"]
        for t in tasks()
        if (t.get("ansible.builtin.template") or {}).get("src") == "mimir.yml.j2"
    )
    assert config_task["dest"] == "/etc/mimir/config.yml"


# --- loki --------------------------------------------------------------------------------------------------------


def test_loki_is_monolithic_without_auth() -> None:
    """No tenants, HTTP on the LAN (Alloy on the robot pushes here), gRPC and rings on localhost."""
    config = loki_config()
    assert config["auth_enabled"] is False
    server = config["server"]
    assert (server["http_listen_address"], server["http_listen_port"]) == ("0.0.0.0", 3100)
    assert server["grpc_listen_address"] == "127.0.0.1"
    common = config["common"]
    assert common["replication_factor"] == 1
    assert common["ring"]["kvstore"]["store"] == "inmemory"
    assert common["instance_addr"] == "127.0.0.1"
    assert config["analytics"]["reporting_enabled"] is False


def test_loki_stores_tsdb_v13_on_the_filesystem() -> None:
    """Filesystem chunks and a tsdb v13 schema, everything under /var/lib/loki."""
    config = loki_config()
    assert config["common"]["path_prefix"] == "/var/lib/loki"
    assert config["common"]["storage"]["filesystem"]["chunks_directory"].startswith("/var/lib/loki/")
    schema = config["schema_config"]["configs"]
    assert len(schema) == 1
    assert (schema[0]["store"], schema[0]["object_store"], schema[0]["schema"]) == ("tsdb", "filesystem", "v13")
    assert config["ingester"]["wal"]["dir"].startswith("/var/lib/loki/")


def test_loki_deletes_logs_after_one_day() -> None:
    """retention_period 24h, applied by the compactor (retention_enabled, delete requests on the filesystem)."""
    config = loki_config()
    assert config["limits_config"]["retention_period"] == "24h"
    compactor = config["compactor"]
    assert compactor["retention_enabled"] is True
    assert compactor["delete_request_store"] == "filesystem"
    assert compactor["working_directory"].startswith("/var/lib/loki/")


def test_loki_config_is_verified_before_it_is_installed() -> None:
    """The template task validates with the real binary (loki -verify-config)."""
    task = next(
        t["ansible.builtin.template"]
        for t in tasks()
        if (t.get("ansible.builtin.template") or {}).get("src") == "loki.yml.j2"
    )
    assert task["dest"] == "/etc/loki/config.yml"
    assert task["validate"] == "/usr/bin/loki -config.file=%s -verify-config"


# --- grafana -----------------------------------------------------------------------------------------------------


def test_grafana_behind_nginx_at_subpath_without_login() -> None:
    """Grafana listens on localhost only (nginx fronts it), serves itself from /grafana/, anonymous visitors edit."""
    ini = grafana_ini()
    assert ini["server"]["http_addr"] == "127.0.0.1"
    assert ini["server"]["http_port"] == "3000"
    assert ini["server"]["root_url"] == "%(protocol)s://%(domain)s/grafana/"
    assert ini["server"]["serve_from_sub_path"] == "true"
    assert ini["auth.anonymous"]["enabled"] == "true"
    assert ini["auth.anonymous"]["org_role"] == "Editor"


def test_grafana_phones_nowhere() -> None:
    """Analytics, update checks, news and gravatar are off."""
    ini = grafana_ini()
    assert ini["analytics"]["reporting_enabled"] == "false"
    assert ini["analytics"]["check_for_updates"] == "false"
    assert ini["analytics"]["check_for_plugin_updates"] == "false"
    assert ini["security"]["disable_gravatar"] == "true"
    assert ini["news"]["news_feed_enabled"] == "false"


def test_grafana_runs_only_the_prometheus_and_loki_backend_plugins() -> None:
    """Every other bundled datasource is disabled (one backend process each) and nothing is preinstalled."""
    ini = grafana_ini()
    disabled = {p.strip() for p in ini["plugins"]["disable_plugins"].split(",")}
    assert disabled == UNUSED_GRAFANA_PLUGINS
    assert "prometheus" not in disabled and "loki" not in disabled
    assert ini["plugins"]["preinstall_disabled"] == "true"


def test_grafana_admin_password_from_host_file() -> None:
    """The password file is read by systemd (LoadCredential) and passed through GF_SECURITY_ADMIN_PASSWORD__FILE."""
    item = next(u for u in role_defaults(STACK_DIR)["monitoring_units"] if u["name"] == "grafana-server")
    settings = unit_settings(stack("monitoring-dropin.conf.j2", item=item))
    assert settings["LoadCredential"] == ["admin_password:/etc/grafana/admin-password"]
    assert "GF_SECURITY_ADMIN_PASSWORD__FILE=%d/admin_password" in settings["Environment"]
    assert "admin_password" not in stack("grafana.ini.j2")


def test_grafana_admin_password_generated_on_the_host() -> None:
    """openssl on the server writes it once (creates:), 0600 root:grafana; never an Ansible password lookup."""
    text = "\n".join(p.read_text() for p in (STACK_DIR / "tasks").glob("*.yml"))
    assert "lookup('password'" not in text and 'lookup("password"' not in text
    gen = [t for t in tasks() if "openssl rand" in str(t.get("ansible.builtin.shell", ""))]
    assert len(gen) == 1
    assert resolve(gen[0]["args"]["creates"], "server") == "/etc/grafana/admin-password"
    perms = [
        t["ansible.builtin.file"]
        for t in tasks()
        if "loop" not in t
        and resolve((t.get("ansible.builtin.file") or {}).get("path", ""), "server") == "/etc/grafana/admin-password"
    ]
    assert len(perms) == 1
    assert (perms[0]["owner"], perms[0]["group"], perms[0]["mode"]) == ("root", "grafana", "0600")


def test_grafana_admin_password_command_runs(tmp_path: Path) -> None:
    """The generation command really writes a 48-character hex password (openssl takes options before the count)."""
    task = next(t for t in tasks() if "openssl rand" in str(t.get("ansible.builtin.shell", "")))
    target = tmp_path / "admin-password"
    command = jinja2.Template(task["ansible.builtin.shell"]).render(monitoring_grafana_admin_password_file=str(target))
    subprocess.run(["bash", "-c", command], check=True, capture_output=True)
    assert re.fullmatch(r"[0-9a-f]{48}\n?", target.read_text())
    assert stat.S_IMODE(target.stat().st_mode) == 0o600


def test_grafana_datasources_mimir_and_loki() -> None:
    """prometheus-robot is Mimir's Prometheus API (keeps every dashboard working), loki-robot is Loki."""
    datasources = {d["uid"]: d for d in yaml.safe_load(stack("grafana-datasources.yaml.j2"))["datasources"]}
    assert set(datasources) == {"prometheus-robot", "loki-robot"}
    mimir = datasources["prometheus-robot"]
    assert (mimir["type"], mimir["url"], mimir["isDefault"]) == ("prometheus", "http://127.0.0.1:9009/prometheus", True)
    assert mimir["jsonData"]["prometheusType"] == "Mimir"
    assert mimir["jsonData"]["timeInterval"] == "5s"
    loki = datasources["loki-robot"]
    assert (loki["type"], loki["url"]) == ("loki", "http://127.0.0.1:3100")
    task = next(
        t["ansible.builtin.template"]
        for t in tasks()
        if (t.get("ansible.builtin.template") or {}).get("src") == "grafana-datasources.yaml.j2"
    )
    assert task["dest"] == "/etc/grafana/provisioning/datasources/robot.yaml"


def test_grafana_dashboard_provider() -> None:
    """One task copies every files/*.json into the directory the dashboard provider reads, keeping the file name."""
    providers = yaml.safe_load(stack("grafana-dashboards.yaml.j2"))["providers"]
    assert [p["folder"] for p in providers] == ["Robot"]
    copies = [t for t in tasks() if "ansible.builtin.copy" in t and "*.json" in t.get("with_fileglob", [])]
    assert len(copies) == 1
    copy = copies[0]["ansible.builtin.copy"]
    assert copy["src"] == "{{ item }}"
    names = [p.name for p in sorted(DASHBOARDS_DIR.glob("*.json"))]
    installed = {resolve(copy["dest"], "server", str(DASHBOARDS_DIR / name)) for name in names}
    assert installed == {f"{providers[0]['options']['path']}/{name}" for name in names}
    assert len(names) == 5


# --- start, restart guard, readiness -----------------------------------------------------------------------------


def test_services_restart_when_a_config_is_newer_than_their_start() -> None:
    """A lost handler (a run that failed after writing a config) must not leave a service on its old config: the
    role restarts every stack service whose config files (or the shared slice) changed after it started; the debs
    start Mimir and Loki with package defaults before the role writes their configs."""
    guard = [t for t in tasks() if "newer than" in t.get("name", "")]
    assert len(guard) == 1
    script = guard[0]["ansible.builtin.shell"]
    for unit, path in (
        ("mimir", "/etc/default/mimir"),
        ("mimir", "/etc/mimir/config.yml"),
        ("loki", "/etc/loki/config.yml"),
        ("grafana-server", "/etc/grafana/grafana.ini"),
        ("grafana-server", "/etc/grafana/provisioning/datasources/robot.yaml"),
    ):
        assert unit in script and path in script, (unit, path)
    assert script.count("$d/monitoring.slice") == 3
    assert "ActiveEnterTimestamp" in script and "systemctl restart" in script
    names = task_names()
    start = names.index("Enable and start Mimir, Loki and Grafana")
    ready = names.index("Wait until Mimir, Loki and Grafana answer")
    assert start < names.index(guard[0]["name"]) < ready


def test_readiness_checks() -> None:
    """Mimir /ready, Loki /ready, Grafana /api/health directly and through nginx at /grafana/."""
    task = next(t for t in tasks() if t.get("name") == "Wait until Mimir, Loki and Grafana answer")
    urls = [resolve(u, "server") for u in task["loop"]]
    assert urls == [
        "http://127.0.0.1:9009/ready",
        "http://127.0.0.1:3100/ready",
        "http://127.0.0.1:3000/api/health",
        "http://127.0.0.1/grafana/api/health",
    ]
    assert task["retries"] * task["delay"] >= 120  # Mimir's ingester waits out its ring min_ready_duration


def test_disabled_stack_stops_its_services() -> None:
    """monitoring_enabled false stops and disables Mimir, Loki and Grafana (nginx and Alloy are other roles)."""
    stop = next(t for t in tasks() if t.get("name", "").startswith("Stop and disable the monitoring stack"))
    assert {item["unit"] for item in stop["loop"]} == {"mimir", "loki", "grafana-server"}


# --- deploy integration ------------------------------------------------------------------------------------------


def monitoring_includes(host: str) -> list[dict]:
    """include_role tasks of the host's playbook tagged monitoring.

    Args:
        host (str): client or server.

    Returns:
        list[dict]: Tasks in order.
    """
    return [t for t in playbook_role_includes(host) if "monitoring" in t.get("tags", [])]


@pytest.mark.parametrize(
    ("host", "roles"),
    [("client", ["monitoring_agent", "web_proxy"]), ("server", ["monitoring_agent", "web_proxy", "monitoring"])],
)
def test_playbooks_include_the_host_roles_tagged_monitoring_only(host: str, roles: list[str]) -> None:
    """Each host runs its monitoring roles for --tags monitoring or an untagged run, never for a node or phase tag.

    The server runs the agent first (the slice exists before Mimir/Loki/Grafana start), then nginx, then the stack
    (its readiness check goes through nginx)."""
    found = monitoring_includes(host)
    assert [t["ansible.builtin.include_role"]["name"] for t in found] == roles
    for task in found:
        assert task["tags"] == ["monitoring"]
        assert task["ansible.builtin.include_role"]["apply"]["tags"] == ["monitoring"]


def test_robot_no_longer_includes_the_stack_role() -> None:
    """The robot playbook runs no Mimir/Loki/Grafana."""
    names = [t["ansible.builtin.include_role"]["name"] for t in playbook_role_includes("client")]
    assert "monitoring" not in names


def scope_for(tags: list[str], nodes: list[str]) -> list[str]:
    """Evaluate select_run.yml's set_facts for a run with the given tags.

    Args:
        tags (list[str]): ansible_run_tags.
        nodes (list[str]): ros2_nodes names of the host.

    Returns:
        list[str]: ros2_scope_nodes.
    """
    env = jinja2.Environment()
    facts: dict[str, object] = {"ansible_run_tags": tags, "ros2_nodes": [{"name": n} for n in nodes]}
    for task in load_yaml(SELECT_RUN):
        for key, expr in task["ansible.builtin.set_fact"].items():
            facts[key] = yaml.safe_load(env.from_string(expr).render(**facts).strip())
    scope = facts["ros2_scope_nodes"]
    assert isinstance(scope, list)
    return scope


def host_node_names(host: str) -> list[str]:
    """ros2_nodes names of group_vars/<host>.yml.

    Args:
        host (str): client or server.

    Returns:
        list[str]: Node names.
    """
    data = load_yaml(GROUP_VARS_DIR / f"{host}.yml")
    assert isinstance(data, dict)
    return [n["name"] for n in data["ros2_nodes"]]


@pytest.mark.parametrize("host", HOSTS)
def test_monitoring_only_run_starts_and_verifies_no_nodes(host: str) -> None:
    """--tags monitoring leaves the host's nodes alone; other runs keep their scope (shared select_run.yml)."""
    nodes = host_node_names(host)
    first = nodes[0]
    assert scope_for(["monitoring"], nodes) == []
    assert scope_for(["all"], nodes) == nodes
    assert scope_for(["config"], nodes) == nodes
    assert scope_for([first], nodes) == [first]
    assert scope_for([first, "monitoring"], nodes) == [first]


def run_deploy(tmp_path: Path, *args: str) -> subprocess.CompletedProcess[str]:
    """Run scripts/deploy-nodes.sh with a fake ansible-playbook that prints its arguments.

    Args:
        tmp_path (Path): Scratch directory.
        *args (str): Script arguments.

    Returns:
        subprocess.CompletedProcess[str]: The run.
    """
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    fake = bindir / "ansible-playbook"
    fake.write_text('#!/bin/sh\necho "ARGS: $*"\n')
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    env = {**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}"}
    return subprocess.run(["bash", str(DEPLOY_SCRIPT), *args], env=env, capture_output=True, text=True, check=False)


@pytest.mark.parametrize(("host", "node"), [("client", "web_ui"), ("server", "lerobot_leader")])
def test_deploy_script_accepts_monitoring(tmp_path: Path, host: str, node: str) -> None:
    """`deploy-nodes.sh <host> monitoring` is one run of the host's playbook with --tags monitoring."""
    proc = run_deploy(tmp_path, host, "monitoring")
    assert proc.returncode == 0, proc.stderr
    assert "--tags monitoring" in proc.stdout and f"deploy_nodes_{host}.yml" in proc.stdout
    proc = run_deploy(tmp_path, host, node, "monitoring")
    assert proc.returncode == 0, proc.stderr
    assert f"--tags {node},monitoring" in proc.stdout


@pytest.mark.parametrize(("host", "node"), [("client", "web_ui"), ("server", "lerobot_leader")])
def test_deploy_script_lists_monitoring_on_unknown_name(tmp_path: Path, host: str, node: str) -> None:
    """An unknown name fails and the error lists monitoring among the host's targets."""
    proc = run_deploy(tmp_path, host, "no_such_node")
    assert proc.returncode == 1
    assert "monitoring" in proc.stderr and node in proc.stderr
