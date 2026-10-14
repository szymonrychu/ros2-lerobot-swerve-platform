"""The jetson_base role (post-cleanup OS state of the Jetson) and playbooks/deploy_nodes_jetson.yml. The role's defaults
and tasks are read as data and pinned against the cleanup log; nothing here runs ansible."""

import hashlib
import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "jetson_base"
PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_jetson.yml"
CLEANUP_LOG = REPO_ROOT / "docs" / "plans" / "2026-10-14-jetson-depth-offload-briefs" / "jetson-cleanup-log.md"


def load(path: Path) -> Any:
    """Parse a YAML file.

    Args:
        path: File to parse.

    Returns:
        The parsed document.
    """
    return yaml.safe_load(path.read_text())


def defaults() -> dict[str, Any]:
    """The role's defaults.

    Returns:
        The parsed defaults/main.yml.
    """
    return load(ROLE_DIR / "defaults" / "main.yml")


def tasks_text() -> str:
    """The role's task file as text.

    Returns:
        The raw tasks/main.yml.
    """
    return (ROLE_DIR / "tasks" / "main.yml").read_text()


def flat_tasks(tasks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Flatten blocks into one task list.

    Args:
        tasks: Parsed task list, possibly with blocks.

    Returns:
        Every leaf task in order.
    """
    out: list[dict[str, Any]] = []
    for task in tasks:
        if "block" in task:
            out.extend(flat_tasks(task["block"]))
        else:
            out.append(task)
    return out


def role_tasks() -> list[dict[str, Any]]:
    """Every leaf task of the role.

    Returns:
        The flattened tasks/main.yml.
    """
    return flat_tasks(load(ROLE_DIR / "tasks" / "main.yml"))


def task_named(fragment: str) -> dict[str, Any]:
    """The single role task whose name contains a fragment.

    Args:
        fragment: Substring of the task name (case-insensitive).

    Returns:
        The task.
    """
    found = [t for t in role_tasks() if fragment.lower() in t["name"].lower()]
    assert len(found) == 1, f"{fragment!r} matched {[t['name'] for t in found]}"
    return found[0]


def log_units() -> set[str]:
    """Services the cleanup log disabled or masked, parsed from the log text (step 2).

    Returns:
        Unit base names (no .service suffix), with fwupd-refresh.timer for the "(+timer)" note.
    """
    step2 = next(ln for ln in CLEANUP_LOG.read_text().splitlines() if ln.startswith("2. Services:"))
    units = set(re.findall(r"disable --now (\w+)", step2))
    slashed = re.search(r"disable ((?:\w+/)+\w+)\(\+timer\)", step2)
    assert slashed, "cleanup log step 2 no longer lists the disabled services"
    units |= set(slashed.group(1).split("/"))
    units.add("fwupd-refresh.timer")
    masked = re.search(r"; mask ([\w ]+?);", step2)
    assert masked, "cleanup log step 2 no longer lists the masked services"
    units |= set(masked.group(1).split())
    return units


def test_masked_list_covers_every_service_the_cleanup_log_disabled_or_masked() -> None:
    masked = defaults()["jetson_base_masked_units"]
    names = {re.sub(r"\.service$", "", m["unit"]) for m in masked}
    assert names == log_units()


def test_every_masked_unit_carries_a_reason() -> None:
    for entry in defaults()["jetson_base_masked_units"]:
        assert entry["reason"].strip(), entry["unit"]


def test_masking_links_each_unit_to_dev_null_and_reloads_systemd_on_change() -> None:
    mask = task_named("Mask the services")
    assert mask["ansible.builtin.file"]["src"] == "/dev/null"
    assert mask["ansible.builtin.file"]["state"] == "link"
    assert "/etc/systemd/system/" in mask["ansible.builtin.file"]["dest"]
    assert mask["loop"] == "{{ jetson_base_masked_units }}"
    reload = task_named("Reload systemd after masking")
    assert "changed" in reload["when"]


def test_default_target_is_multi_user() -> None:
    assert defaults()["jetson_base_default_target"] == "multi-user.target"
    assert "set-default" in tasks_text()
    assert "get-default" in tasks_text()


def test_journald_is_persistent_with_a_200m_cap() -> None:
    d = defaults()
    assert d["jetson_base_journald_storage"] == "persistent"
    assert d["jetson_base_journald_max_use"] == "200M"
    text = tasks_text()
    assert "/etc/systemd/journald.conf.d/persistent.conf" in text
    assert "Storage={{ jetson_base_journald_storage }}" in text
    assert "SystemMaxUse={{ jetson_base_journald_max_use }}" in text
    assert task_named("journal directory")["ansible.builtin.file"]["path"] == "/var/log/journal"
    restart = task_named("Restart journald")
    assert "changed" in restart["when"]


def test_nvpmodel_is_set_to_maxn_only_when_the_query_differs() -> None:
    assert defaults()["jetson_base_nvpmodel_mode"] == 0
    query = task_named("Query nvpmodel")
    assert query["ansible.builtin.command"] == "nvpmodel -q"
    assert query["changed_when"] is False
    assert query["check_mode"] is False
    setter = task_named("Set nvpmodel")
    assert "nvpmodel -m {{ jetson_base_nvpmodel_mode }}" in setter["ansible.builtin.command"]
    assert "MAXN" in setter["when"]
    assert "not in" in setter["when"] or "!=" in setter["when"] or "is not search" in setter["when"]


def test_jetson_clocks_is_an_enabled_oneshot_after_nvpmodel() -> None:
    unit = task_named("Install the jetson-clocks unit")["ansible.builtin.copy"]
    assert unit["dest"] == "/etc/systemd/system/jetson-clocks.service"
    content = unit["content"]
    assert "Type=oneshot" in content
    assert "After=nvpmodel.service" in content
    assert "ExecStart=/usr/bin/jetson_clocks" in content
    assert "RemainAfterExit=yes" in content
    assert "WantedBy=multi-user.target" in content
    enable = task_named("Enable jetson-clocks")["ansible.builtin.systemd"]
    assert enable["name"] == "jetson-clocks" and enable["enabled"] is True


def test_keep_list_names_the_accelerated_stack() -> None:
    d = defaults()
    packages = d["jetson_base_keep_packages"]
    for required in ("nvidia-l4t-core", "nvidia-l4t-camera", "nvidia-l4t-cuda", "cuda-cudart-10-2", "cuda-nvcc-10-2"):
        assert required in packages
    for required in ("libcudnn8", "libnvinfer8", "libnvvpi1"):
        assert required in packages
    assert sum(p.startswith("nvidia-l4t-") for p in packages) >= 27
    assert d["jetson_base_keep_paths"] == ["/usr/lib/python3/dist-packages/cv2"]


def test_keep_list_assert_fails_on_a_missing_package_or_path() -> None:
    text = tasks_text()
    assert "package_facts" in text
    assert "jetson_base_keep_packages" in text and "jetson_base_keep_paths" in text
    assert_task = task_named("Assert the keep-list packages")["ansible.builtin.assert"]
    assert "difference" in str(assert_task["that"])
    assert "fail_msg" in assert_task
    path_assert = task_named("Assert the keep-list paths")["ansible.builtin.assert"]
    assert "exists" in str(path_assert["that"])


def test_role_never_removes_a_package() -> None:
    for task in role_tasks():
        apt = task.get("ansible.builtin.apt")
        if apt:
            assert apt.get("state") not in ("absent", "purge") and not apt.get("purge")
            assert not apt.get("autoremove")
    assert not re.search(r"apt(-get)?\s+(remove|purge|autoremove)|dpkg\s+(-P|--purge|-r)", tasks_text())


def test_isp_override_is_installed_0664_and_restarts_nvargus_only_on_change() -> None:
    d = defaults()
    isp = ROLE_DIR / "files" / "camera_overrides.isp"
    assert hashlib.sha256(isp.read_bytes()).hexdigest() == d["jetson_base_isp_sha256"]
    header = (ROLE_DIR / "defaults" / "main.yml").read_text()
    assert d["jetson_base_isp_source_url"] in header and d["jetson_base_isp_sha256"] in header
    assert d["jetson_base_isp_source_url"].startswith("https://files.waveshare.com/")
    install = task_named("Install the IMX219")["ansible.builtin.copy"]
    assert install["dest"] == "/var/nvidia/nvcam/settings/camera_overrides.isp"
    assert install["mode"] == "0664"
    restart = task_named("Restart nvargus-daemon")
    assert restart["ansible.builtin.systemd"]["name"] == "nvargus-daemon"
    assert "changed" in restart["when"]


def test_playbook_targets_jetson_and_tags_the_base_roles() -> None:
    plays = load(PLAYBOOK)
    assert len(plays) == 1 and plays[0]["hosts"] == "jetson"
    assert plays[0]["become"] is True
    roles = plays[0]["roles"]
    by_name = {r["role"]: r for r in roles}
    assert [r["role"] for r in roles] == ["network", "jetson_link", "jetson_base", "jetson_depthd"]
    for role in roles[:3]:
        assert "base" in role["tags"]
    assert by_name["jetson_depthd"]["tags"] == ["build_deps", "depthd"]
    assert by_name["jetson_link"]["jetson_link_side"] == "client"


def test_wifi_file_is_removed_only_after_the_ansible_netplan_has_the_wlan0_stanza() -> None:
    network = next(r for r in load(PLAYBOOK)[0]["roles"] if r["role"] == "network")
    assert network["network_netplan_superseded_files"] == ["/etc/netplan/00-wifi.yaml"]
    tasks = flat_tasks(load(ANSIBLE_DIR / "roles" / "network" / "tasks" / "main.yml"))
    names = [t["name"] for t in tasks]
    deploy_i = names.index("Deploy netplan configuration")
    read_i = names.index("Read back the deployed netplan file")
    remove_i = next(i for i, n in enumerate(names) if n.startswith("Remove superseded netplan files"))
    apply_i = next(i for i, n in enumerate(names) if n.startswith("Apply netplan"))
    assert deploy_i < read_i < remove_i < apply_i
    assert tasks[read_i]["ansible.builtin.slurp"]["src"] == "/etc/netplan/{{ network_netplan_file }}"
    remove = tasks[remove_i]
    assert remove["loop"] == "{{ network_netplan_superseded_files }}"
    assert "wifis:" in str(remove["when"]) and "_network_interface" in str(remove["when"])
    assert "network_netplan_superseded is changed" in tasks[apply_i]["when"]
    assert load(ANSIBLE_DIR / "roles" / "network" / "defaults" / "main.yml")["network_netplan_superseded_files"] == []


def test_test_ansible_syntax_checks_the_jetson_playbook() -> None:
    pyproject = (REPO_ROOT / "pyproject.toml").read_text()
    assert "ansible-playbook -i inventory playbooks/deploy_nodes_jetson.yml --syntax-check" in pyproject


def test_plan_verify_uses_numeric_chronyc() -> None:
    """Without -n chronyc prints the Pi's hostname (client.ros2.lan), so the plan's T5 verify must use -n."""
    plan = (REPO_ROOT / "docs" / "plans" / "2026-10-14-jetson-depth-offload-plan.md").read_text()
    verify = [line for line in plan.splitlines() if line.startswith("uv run pytest tests/test_jetson_base_role.py")]
    assert verify, "T5 verify line not found"
    for line in verify:
        assert "chronyc -n sources | grep -q 10.77.0.1" in line


# --- host metrics: prometheus-node-exporter, scraped by the Pi's Alloy (the Jetson has no route to the server) -------


def test_node_exporter_defaults_listen_on_the_link_and_are_capped() -> None:
    d = defaults()
    assert d["jetson_base_node_exporter_listen"] == "{{ jetson_link_jetson_ip }}:9100"
    assert d["jetson_base_node_exporter_cpu_weight"] <= 20
    assert d["jetson_base_node_exporter_memory_max"]
    assert d["jetson_base_node_exporter_oom_score_adjust"] == 500


def test_node_exporter_is_installed_and_started_in_the_background_tier() -> None:
    install = task_named("Install prometheus-node-exporter")
    assert install["ansible.builtin.apt"]["name"] == "prometheus-node-exporter"
    dropin = task_named("Put the node exporter drop-in")
    content = dropin["ansible.builtin.copy"]["content"]
    for key in ("CPUWeight=", "IOWeight=", "MemoryMax=", "Nice=", "OOMScoreAdjust="):
        assert key in content, key
    assert "/etc/systemd/system/prometheus-node-exporter.service.d/" in dropin["ansible.builtin.copy"]["dest"]
    defaults_file = task_named("node exporter settings")
    args = defaults_file["ansible.builtin.copy"]["content"]
    assert "--web.listen-address={{ jetson_base_node_exporter_listen }}" in args
    start = task_named("Enable and start prometheus-node-exporter")
    assert start["ansible.builtin.systemd"]["enabled"] is True
    assert start["ansible.builtin.systemd"]["state"] == "started"
