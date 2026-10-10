"""Monitoring stack role (Alloy + Prometheus + Grafana on the client RPi): structure, limits and dashboard contract."""

import configparser
import importlib.util
import json
import os
import re
import stat
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import jinja2
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "monitoring"
TEMPLATES_DIR = ROLE_DIR / "templates"
FILES_DIR = ROLE_DIR / "files"
DASHBOARD = FILES_DIR / "robot-resources.json"
THROTTLE_SCRIPT = FILES_DIR / "rpi_throttled_textfile.py"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
SELECT_RUN = ANSIBLE_DIR / "playbooks" / "tasks" / "select_run.yml"
DEPLOY_SCRIPT = REPO_ROOT / "scripts" / "deploy-nodes.sh"

MONITORING_UNITS = {"alloy": "250M", "prometheus": "450M", "grafana-server": "250M"}

# Metric contract (shared with the cgroup-limits task) plus every node_* host metric.
CONTRACT_METRICS = {
    "container_cpu_usage_seconds_total",
    "container_memory_working_set_bytes",
    "container_memory_rss",
    "container_fs_reads_bytes_total",
    "container_fs_writes_bytes_total",
    "container_spec_memory_limit_bytes",
    "container_spec_cpu_quota",
    "container_spec_cpu_period",
    "rpi_throttled_flags",
    "rpi_throttled",
}

# PromQL functions, aggregations and keywords: identifiers in an expression that are not metric names.
PROMQL_WORDS = {
    "abs", "absent", "and", "avg", "avg_over_time", "bool", "by", "ceil", "clamp", "clamp_max", "clamp_min", "count",
    "count_over_time", "delta", "deriv", "floor", "group_left", "group_right", "ignoring", "increase", "irate",
    "label_replace", "last_over_time", "max", "max_over_time", "min", "min_over_time", "offset", "on", "or", "rate",
    "round", "scalar", "sort", "sort_desc", "sum", "sum_over_time", "time", "topk", "bottomk", "unless", "vector",
    "without", "e", "inf", "nan",
}  # fmt: skip

REQUIRED_PANELS = [
    "Host CPU % by mode",
    "Load average",
    "Memory used / available",
    "CPU temperature",
    "Throttling flags",
    "Root disk usage",
    "Root disk IO",
    "Network rx/tx",
    "Per-unit CPU cores used vs CPUQuota",
    "Per-unit memory working set vs MemoryMax",
    "Top 10 units by CPU",
    "Monitoring stack overhead",
]


def load_yaml(path: Path) -> object:
    """Parse one YAML file.

    Args:
        path (Path): File to read.

    Returns:
        object: The parsed document.
    """
    return yaml.safe_load(path.read_text())


def role_defaults() -> dict:
    """Return the role defaults (defaults/main.yml).

    Returns:
        dict: Variable name to value.
    """
    return load_yaml(ROLE_DIR / "defaults" / "main.yml")


def render(template: str, **extra: object) -> str:
    """Render one role template with the role defaults and extra variables, failing on undefined names.

    Args:
        template (str): Template file name under templates/.
        **extra (object): Variables added on top of the defaults (e.g. item for a loop).

    Returns:
        str: Rendered text.
    """
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(TEMPLATES_DIR), undefined=jinja2.StrictUndefined, keep_trailing_newline=True
    )
    env.globals["ansible_managed"] = "Managed by Ansible"
    return env.get_template(template).render(**role_defaults(), **extra)


def resolve(value: str, item: object = None) -> str:
    """Render a task string (e.g. a templated path) with the role defaults.

    Args:
        value (str): Jinja2 string.
        item (object): Loop item, when the task loops.

    Returns:
        str: Rendered string.
    """
    return jinja2.Environment(undefined=jinja2.StrictUndefined).from_string(value).render(**role_defaults(), item=item)


def role_tasks() -> list[dict]:
    """Return every task of the role, flattening blocks.

    Returns:
        list[dict]: Tasks in file order.
    """
    out: list[dict] = []

    def walk(tasks: list[dict]) -> None:
        for task in tasks:
            out.append(task)
            for key in ("block", "rescue", "always"):
                walk(task.get(key, []))

    for path in sorted((ROLE_DIR / "tasks").glob("*.yml")):
        walk(load_yaml(path) or [])
    return out


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


def load_throttle_script() -> ModuleType:
    """Import files/rpi_throttled_textfile.py as a module.

    Returns:
        ModuleType: The script module.
    """
    spec = importlib.util.spec_from_file_location("rpi_throttled_textfile", THROTTLE_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def panel_list(dashboard: dict) -> list[dict]:
    """Return every panel of a dashboard, including the ones nested in rows.

    Args:
        dashboard (dict): Dashboard JSON model.

    Returns:
        list[dict]: Panels.
    """
    out: list[dict] = []
    for panel in dashboard["panels"]:
        out.append(panel)
        out.extend(panel.get("panels", []))
    return out


def metric_names(expr: str) -> set[str]:
    """Extract the metric names a PromQL expression references.

    Args:
        expr (str): PromQL expression (Grafana variables allowed).

    Returns:
        set[str]: Metric names.
    """
    text = re.sub(r'"(?:[^"\\]|\\.)*"', "", expr)  # string literals (label values, label_replace args)
    text = re.sub(r"\{[^}]*\}", "", text)  # label matchers
    text = re.sub(r"\[[^\]]*\]", "", text)  # ranges
    text = re.sub(r"\b(by|without|on|ignoring|group_left|group_right)\s*\([^)]*\)", "", text)  # label lists
    text = re.sub(r"\$\{?\w+\}?", "", text)  # Grafana variables
    tokens = set(re.findall(r"[A-Za-z_:][A-Za-z0-9_:]*", text))
    return {t for t in tokens if t.lower() not in PROMQL_WORDS and not re.fullmatch(r"\d+[smhdwy]", t)}


# --- role structure ----------------------------------------------------------------------------------------------


def test_role_files_exist() -> None:
    """The role has tasks, defaults, handlers, meta and the templates the tasks render."""
    for rel in [
        "tasks/main.yml",
        "defaults/main.yml",
        "handlers/main.yml",
        "meta/main.yml",
        "templates/config.alloy.j2",
        "templates/prometheus.yml.j2",
        "files/robot-resources.json",
        "files/rpi_throttled_textfile.py",
    ]:
        assert (ROLE_DIR / rel).is_file(), rel


def test_every_template_and_file_the_tasks_name_exists() -> None:
    """Every src: of a template/copy task names a file shipped with the role."""
    checked = 0
    for task in role_tasks():
        for module, folder in (("ansible.builtin.template", TEMPLATES_DIR), ("ansible.builtin.copy", FILES_DIR)):
            src = (task.get(module) or {}).get("src")
            if not src:
                continue
            loop = task.get("loop", [None])
            items = yaml.safe_load(resolve(loop)) if isinstance(loop, str) else loop
            for item in items:
                assert (folder / resolve(src, item)).is_file(), f"{task['name']}: {src}"
                checked += 1
    assert checked >= 10


def test_defaults_have_enable_flag_ports_retention_and_limits() -> None:
    """Enable flag, ports, retention and the per-unit limits live in defaults/main.yml."""
    defaults = role_defaults()
    assert defaults["monitoring_enabled"] is True
    assert defaults["monitoring_grafana_port"] == 3000
    assert defaults["monitoring_prometheus_port"] == 9090
    assert defaults["monitoring_alloy_port"] == 12345
    assert defaults["monitoring_prometheus_retention_time"] == "1d"
    assert defaults["monitoring_prometheus_retention_size"] == "2GB"
    assert defaults["monitoring_interval"] == "5s"
    assert {u["name"]: u["memory_max"] for u in defaults["monitoring_units"]} == MONITORING_UNITS


# --- resource isolation ------------------------------------------------------------------------------------------


def test_monitoring_slice_limits() -> None:
    """The slice caps the whole stack: low CPU/IO weight, 40% of one core, 900M hard / 800M soft memory."""
    settings = unit_settings(render("monitoring.slice.j2"))
    assert settings["CPUWeight"] == ["20"]
    assert settings["IOWeight"] == ["20"]
    assert settings["CPUQuota"] == ["40%"]
    assert settings["MemoryMax"] == ["900M"]
    assert settings["MemoryHigh"] == ["800M"]


@pytest.mark.parametrize("unit", sorted(MONITORING_UNITS))
def test_dropin_moves_unit_into_the_slice(unit: str) -> None:
    """Each service runs in monitoring.slice, niced, memory capped and first in line for the OOM killer."""
    item = next(u for u in role_defaults()["monitoring_units"] if u["name"] == unit)
    settings = unit_settings(render("monitoring-dropin.conf.j2", item=item))
    assert settings["Slice"] == ["monitoring.slice"]
    assert settings["Nice"] == ["10"]
    assert settings["MemoryMax"] == [MONITORING_UNITS[unit]]
    assert settings["OOMScoreAdjust"] == ["500"]
    assert settings["Restart"] == ["on-failure"]


def test_dropins_are_installed_for_every_unit() -> None:
    """A template task loops over monitoring_units and writes /etc/systemd/system/<unit>.service.d/."""
    tasks = [
        t for t in role_tasks() if (t.get("ansible.builtin.template") or {}).get("src") == "monitoring-dropin.conf.j2"
    ]
    assert len(tasks) == 1
    assert tasks[0]["loop"] == "{{ monitoring_units }}"
    assert "/etc/systemd/system/{{ item.name }}.service.d/" in tasks[0]["ansible.builtin.template"]["dest"]


# --- prometheus --------------------------------------------------------------------------------------------------


def test_prometheus_flags() -> None:
    """Retention 1d / 2GB on the NVMe root, LAN listener, remote-write receiver for Alloy."""
    args = render("prometheus.default.j2")
    for flag in [
        "--storage.tsdb.retention.time=1d",
        "--storage.tsdb.retention.size=2GB",
        "--storage.tsdb.path=/var/lib/prometheus/metrics2",
        "--web.listen-address=0.0.0.0:9090",
        "--web.enable-remote-write-receiver",
        "--config.file=/etc/prometheus/prometheus.yml",
    ]:
        assert flag in args, flag


def test_prometheus_config_is_the_roles_own() -> None:
    """prometheus.yml: 5 s intervals and no scrape of exporters the robot does not run."""
    config = yaml.safe_load(render("prometheus.yml.j2"))
    assert config["global"]["scrape_interval"] == "5s"
    assert config["global"]["evaluation_interval"] == "5s"
    assert config["global"]["scrape_timeout"] == "4s"  # must not exceed the 5 s interval (default is 10 s)
    assert not config.get("scrape_configs")


def test_node_exporter_is_masked() -> None:
    """Alloy provides host metrics: the prometheus-node-exporter the package may pull in is masked."""
    tasks = [t for t in role_tasks() if "ansible.builtin.systemd_service" in t]
    masked = [t["ansible.builtin.systemd_service"] for t in tasks if t["ansible.builtin.systemd_service"].get("masked")]
    assert any(m["name"] == "prometheus-node-exporter" for m in masked)
    apt = [t["ansible.builtin.apt"] for t in role_tasks() if "prometheus" in str(t.get("ansible.builtin.apt") or {})]
    assert apt and all(a.get("install_recommends") is False for a in apt)


# --- alloy -------------------------------------------------------------------------------------------------------


def test_alloy_config() -> None:
    """Alloy: host + cgroup + self metrics, Prometheus and Grafana scraped at 5 s, all written to local Prometheus."""
    config = render("config.alloy.j2")
    assert 'prometheus.exporter.unix "host"' in config
    assert 'directory = "/var/lib/alloy/textfile"' in config
    assert 'prometheus.exporter.cadvisor "units"' in config
    assert "docker_only" in config and re.search(r"docker_only\s*=\s*false", config)
    assert 'prometheus.exporter.self "alloy"' in config
    assert '"127.0.0.1:9090"' in config and '"127.0.0.1:3000"' in config
    assert 'url = "http://127.0.0.1:9090/api/v1/write"' in config
    scrapes = re.findall(r'prometheus\.scrape "[^"]+"', config)
    assert len(scrapes) >= 4
    assert config.count('scrape_interval = "5s"') == len(scrapes)
    # Alloy rejects a scrape whose timeout (default 10 s) exceeds its interval.
    assert config.count('scrape_timeout  = "4s"') == len(scrapes)


def test_alloy_drops_high_cardinality_cgroups_and_veth() -> None:
    """Only the root, slices and their services are kept; veth/docker interfaces are dropped."""
    config = render("config.alloy.j2")
    keep = re.search(r'source_labels\s*=\s*\["id"\]\s*\n\s*regex\s*=\s*"([^"]+)"\s*\n\s*action\s*=\s*"keep"', config)
    assert keep, "keep rule on id"
    pattern = re.compile(keep.group(1).replace("\\\\", "\\"))
    for kept in ["/", "/system.slice", "/system.slice/ros2-web_ui.service", "/monitoring.slice/alloy.service"]:
        assert pattern.fullmatch(kept), kept
    for dropped in ["/system.slice/docker-abc.scope", "/user.slice/user-1000.slice/session-3.scope", "/init.scope"]:
        assert not pattern.fullmatch(dropped), dropped
    assert re.search(r'source_labels\s*=\s*\["interface"\]\s*\n\s*regex\s*=\s*"[^"]*veth', config)


def test_alloy_listens_on_lan_port() -> None:
    """Alloy's HTTP server (UI, /metrics) is on 0.0.0.0:12345 and the reporting call-home is off."""
    args = render("alloy.default.j2")
    assert "--server.http.listen-addr=0.0.0.0:12345" in args
    assert "--disable-reporting" in args
    assert 'CONFIG_FILE="/etc/alloy/config.alloy"' in args


# --- grafana -----------------------------------------------------------------------------------------------------


def grafana_ini() -> configparser.ConfigParser:
    """Parse the rendered grafana.ini.

    Returns:
        configparser.ConfigParser: Parsed ini.
    """
    parser = configparser.ConfigParser(interpolation=None)
    parser.read_string(render("grafana.ini.j2"))
    return parser


def test_grafana_listens_on_lan_with_anonymous_viewer() -> None:
    """LAN listener on 0.0.0.0:3000; anonymous visitors get Viewer only."""
    ini = grafana_ini()
    assert ini["server"]["http_addr"] == "0.0.0.0"
    assert ini["server"]["http_port"] == "3000"
    assert ini["auth.anonymous"]["enabled"] == "true"
    assert ini["auth.anonymous"]["org_role"] == "Viewer"


def test_grafana_phones_nowhere() -> None:
    """Analytics, update checks, news and gravatar are off."""
    ini = grafana_ini()
    assert ini["analytics"]["reporting_enabled"] == "false"
    assert ini["analytics"]["check_for_updates"] == "false"
    assert ini["analytics"]["check_for_plugin_updates"] == "false"
    assert ini["security"]["disable_gravatar"] == "true"
    assert ini["news"]["news_feed_enabled"] == "false"


def test_grafana_admin_password_from_host_file() -> None:
    """The password file is read by systemd (LoadCredential) and passed through GF_SECURITY_ADMIN_PASSWORD__FILE."""
    item = next(u for u in role_defaults()["monitoring_units"] if u["name"] == "grafana-server")
    settings = unit_settings(render("monitoring-dropin.conf.j2", item=item))
    assert settings["LoadCredential"] == ["admin_password:/etc/grafana/admin-password"]
    assert "GF_SECURITY_ADMIN_PASSWORD__FILE=%d/admin_password" in settings["Environment"]
    assert "admin_password" not in render("grafana.ini.j2")


def test_grafana_admin_password_generated_on_the_host() -> None:
    """openssl on the robot writes it once (creates:), 0600 root:grafana; never an Ansible password lookup."""
    text = "\n".join(p.read_text() for p in (ROLE_DIR / "tasks").glob("*.yml"))
    assert "lookup('password'" not in text and 'lookup("password"' not in text
    gen = [t for t in role_tasks() if "openssl rand" in str(t.get("ansible.builtin.shell", ""))]
    assert len(gen) == 1
    assert resolve(gen[0]["args"]["creates"]) == "/etc/grafana/admin-password"
    perms = [
        t["ansible.builtin.file"]
        for t in role_tasks()
        if "loop" not in t
        and resolve((t.get("ansible.builtin.file") or {}).get("path", "")) == "/etc/grafana/admin-password"
    ]
    assert len(perms) == 1
    assert (perms[0]["owner"], perms[0]["group"], perms[0]["mode"]) == ("root", "grafana", "0600")


def test_grafana_apt_repo_signed_by_keyring() -> None:
    """Grafana's stable apt repo, signed by a key under /etc/apt/keyrings."""
    repo = [t["ansible.builtin.deb822_repository"] for t in role_tasks() if "ansible.builtin.deb822_repository" in t]
    assert len(repo) == 1
    assert repo[0]["uris"] == "https://apt.grafana.com"
    assert repo[0]["suites"] == "stable"
    assert repo[0]["components"] == "main"
    assert repo[0]["signed_by"].startswith("/etc/apt/keyrings/")
    key = [t["ansible.builtin.get_url"] for t in role_tasks() if "ansible.builtin.get_url" in t]
    assert any(k["url"] == "https://apt.grafana.com/gpg.key" and k["dest"] == repo[0]["signed_by"] for k in key)


def test_grafana_datasource_provisioned() -> None:
    """The Prometheus datasource has the fixed uid the dashboard refers to."""
    ds = yaml.safe_load(render("grafana-datasource.yaml.j2"))["datasources"]
    assert [d["uid"] for d in ds] == ["prometheus-robot"]
    assert ds[0]["type"] == "prometheus" and ds[0]["url"] == "http://127.0.0.1:9090"


def test_grafana_dashboard_provider() -> None:
    """The dashboard provider reads the directory the role copies robot-resources.json into."""
    providers = yaml.safe_load(render("grafana-dashboards.yaml.j2"))["providers"]
    copied = [
        t["ansible.builtin.copy"]
        for t in role_tasks()
        if (t.get("ansible.builtin.copy") or {}).get("src") == "robot-resources.json"
    ]
    assert len(copied) == 1
    assert resolve(copied[0]["dest"]).startswith(providers[0]["options"]["path"] + "/")


# --- dashboard ---------------------------------------------------------------------------------------------------


def test_dashboard_is_valid_json_with_required_panels() -> None:
    """robot-resources.json parses and has every required panel by title."""
    dashboard = json.loads(DASHBOARD.read_text())
    assert dashboard["title"] == "Robot resources"
    titles = [p.get("title") for p in panel_list(dashboard)]
    for title in REQUIRED_PANELS:
        assert title in titles, title


def test_dashboard_queries_use_contract_metrics_only() -> None:
    """Every panel query references only metrics from the contract or node_*, on the provisioned datasource."""
    dashboard = json.loads(DASHBOARD.read_text())
    seen: set[str] = set()
    for panel in panel_list(dashboard):
        if panel.get("type") == "row":
            continue
        assert panel["targets"], panel["title"]
        for target in panel["targets"]:
            assert target["datasource"]["uid"] == "prometheus-robot", panel["title"]
            names = metric_names(target["expr"])
            assert names, (panel["title"], target["expr"])
            bad = {n for n in names if n not in CONTRACT_METRICS and not n.startswith("node_")}
            assert not bad, (panel["title"], bad)
            seen |= names
    assert {"container_cpu_usage_seconds_total", "container_spec_cpu_quota", "rpi_throttled"} <= seen


def test_metric_names_helper() -> None:
    """Guard against vacuous passes: the extractor finds metrics and skips functions, labels and ranges."""
    expr = 'topk(10, sum by (id) (rate(container_cpu_usage_seconds_total{id=~"/system.slice/.+"}[$__rate_interval])))'
    assert metric_names(expr) == {"container_cpu_usage_seconds_total"}
    assert metric_names("node_load1 / on() group_left count(node_cpu_seconds_total)") == {
        "node_load1",
        "node_cpu_seconds_total",
    }


# --- throttling textfile -----------------------------------------------------------------------------------------


def test_parse_throttled() -> None:
    """vcgencmd get_throttled output is parsed as hex."""
    mod = load_throttle_script()
    assert mod.parse_throttled("throttled=0x50005\n") == 0x50005
    assert mod.parse_throttled("throttled=0x0") == 0
    with pytest.raises(ValueError):
        mod.parse_throttled('error=1 error_msg="Command not registered"')


def test_throttled_metrics_names_every_bit() -> None:
    """Raw flags gauge plus one gauge per named bit (0-3 now, 16-19 occurred)."""
    mod = load_throttle_script()
    text = mod.throttled_metrics(0x50005)
    assert "rpi_throttled_flags 327685" in text
    expected = {
        "under_voltage_now": 1,
        "freq_capped_now": 0,
        "throttled_now": 1,
        "soft_temp_limit_now": 0,
        "under_voltage_occurred": 1,
        "freq_capped_occurred": 0,
        "throttled_occurred": 1,
        "soft_temp_limit_occurred": 0,
    }
    for bit, value in expected.items():
        assert f'rpi_throttled{{bit="{bit}"}} {value}\n' in text
    assert "# TYPE rpi_throttled_flags gauge" in text and "# TYPE rpi_throttled gauge" in text


def write_fake_vcgencmd(folder: Path, body: str) -> Path:
    """Create an executable fake vcgencmd.

    Args:
        folder (Path): Directory to put it in.
        body (str): Shell body.

    Returns:
        Path: The fake binary.
    """
    fake = folder / "vcgencmd"
    fake.write_text(f"#!/bin/sh\n{body}\n")
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    return fake


def run_script(tmp_path: Path, body: str) -> tuple[subprocess.CompletedProcess[str], Path]:
    """Run the throttling script against a fake vcgencmd on PATH.

    Args:
        tmp_path (Path): Scratch directory.
        body (str): Fake vcgencmd shell body.

    Returns:
        tuple[subprocess.CompletedProcess[str], Path]: The run and the output file path.
    """
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    write_fake_vcgencmd(bindir, body)
    out = tmp_path / "textfile" / "rpi_throttled.prom"
    out.parent.mkdir(exist_ok=True)
    env = {**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}", "RPI_THROTTLED_TEXTFILE": str(out)}
    proc = subprocess.run([sys.executable, str(THROTTLE_SCRIPT)], env=env, capture_output=True, text=True, check=False)
    return proc, out


def test_script_writes_textfile_atomically(tmp_path: Path) -> None:
    """With a working vcgencmd the textfile appears, with no temp file left behind."""
    proc, out = run_script(tmp_path, 'echo "throttled=0x80000"')
    assert proc.returncode == 0, proc.stderr
    assert 'rpi_throttled{bit="soft_temp_limit_occurred"} 1' in out.read_text()
    assert sorted(p.name for p in out.parent.iterdir()) == ["rpi_throttled.prom"]


def test_script_removes_stale_textfile_on_failure(tmp_path: Path) -> None:
    """A failed read publishes nothing: the old file is removed and the script exits non-zero."""
    proc, out = run_script(tmp_path, 'echo "throttled=0x0"')
    assert proc.returncode == 0 and out.exists()
    proc, out = run_script(tmp_path, "exit 1")
    assert proc.returncode != 0
    assert not out.exists()


def test_throttle_timer_runs_every_5s() -> None:
    """The timer fires the oneshot service every 15 s; the service runs in monitoring.slice."""
    timer = unit_settings(render("rpi-throttled.timer.j2"))
    assert timer["OnUnitActiveSec"] == ["5s"]
    service = unit_settings(render("rpi-throttled.service.j2"))
    assert service["Type"] == ["oneshot"]
    assert service["Slice"] == ["monitoring.slice"]
    assert service["Environment"] == ["RPI_THROTTLED_TEXTFILE=/var/lib/alloy/textfile/rpi_throttled.prom"]


# --- deploy integration ------------------------------------------------------------------------------------------


def test_client_playbook_includes_role_tagged_monitoring_only() -> None:
    """The role runs for --tags monitoring or an untagged run, never for a node or phase tag."""
    play = load_yaml(CLIENT_PLAYBOOK)[0]
    found = []

    def walk(tasks: list[dict]) -> None:
        for task in tasks:
            include = task.get("ansible.builtin.include_role")
            if include and include.get("name") == "monitoring":
                found.append(task)
            for key in ("block", "rescue", "always"):
                walk(task.get(key, []))

    walk(play["tasks"])
    assert len(found) == 1
    assert found[0]["tags"] == ["monitoring"]
    assert found[0]["ansible.builtin.include_role"]["apply"]["tags"] == ["monitoring"]


def scope_for(tags: list[str]) -> list[str]:
    """Evaluate select_run.yml's set_facts for a run with the given tags.

    Args:
        tags (list[str]): ansible_run_tags.

    Returns:
        list[str]: ros2_scope_nodes.
    """
    env = jinja2.Environment()
    facts: dict[str, object] = {"ansible_run_tags": tags, "ros2_nodes": [{"name": "web_ui"}, {"name": "mcp_server"}]}
    for task in load_yaml(SELECT_RUN):
        for key, expr in task["ansible.builtin.set_fact"].items():
            facts[key] = yaml.safe_load(env.from_string(expr).render(**facts).strip())
    scope = facts["ros2_scope_nodes"]
    assert isinstance(scope, list)
    return scope


def test_monitoring_only_run_starts_and_verifies_no_nodes() -> None:
    """--tags monitoring leaves the robot nodes alone; other runs keep their scope."""
    assert scope_for(["monitoring"]) == []
    assert scope_for(["all"]) == ["web_ui", "mcp_server"]
    assert scope_for(["config"]) == ["web_ui", "mcp_server"]
    assert scope_for(["web_ui"]) == ["web_ui"]
    assert scope_for(["web_ui", "monitoring"]) == ["web_ui"]


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


def test_deploy_script_accepts_monitoring(tmp_path: Path) -> None:
    """`deploy-nodes.sh client monitoring` is one client run with --tags monitoring, also next to node names."""
    proc = run_deploy(tmp_path, "client", "monitoring")
    assert proc.returncode == 0, proc.stderr
    assert "--tags monitoring" in proc.stdout and "deploy_nodes_client.yml" in proc.stdout
    proc = run_deploy(tmp_path, "client", "web_ui", "monitoring")
    assert proc.returncode == 0, proc.stderr
    assert "--tags web_ui,monitoring" in proc.stdout


def test_deploy_script_lists_monitoring_on_unknown_name(tmp_path: Path) -> None:
    """An unknown name fails and the error lists monitoring among the client targets."""
    proc = run_deploy(tmp_path, "client", "no_such_node")
    assert proc.returncode == 1
    assert "monitoring" in proc.stderr and "web_ui" in proc.stderr


def test_deploy_script_rejects_monitoring_on_server(tmp_path: Path) -> None:
    """The server playbook has no monitoring role, so the name is not accepted there."""
    proc = run_deploy(tmp_path, "server", "monitoring")
    assert proc.returncode == 1
