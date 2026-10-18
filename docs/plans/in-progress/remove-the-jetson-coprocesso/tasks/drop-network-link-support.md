---
title: Drop network link support
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The network role renders only the DHCP ethernets and WiFi it always did; the Jetson-only link machinery is gone."
files: [ansible/roles/network/defaults/main.yml, ansible/roles/network/tasks/main.yml, ansible/roles/network/templates/netplan.yaml.j2, ansible/group_vars/all.yml, tests/test_network_link.py, tests/test_network_netplan.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_network_netplan.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [remove-jetson-link-on-client]
owns: []
uses: []
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Drop network link support

## Requirement

- Network role: the `network_link_ethernets` and `network_netplan_superseded_files` defaults are removed; `tasks/main.yml` drops the link address probe and assert and the superseded-file removal (~43-66); `netplan.yaml.j2` drops the link stanza rendering (~6, ~25-40). The DHCP ethernets list and the WiFi stanza render exactly as before for the server and client vars.
- `ansible/group_vars/all.yml` has no `network_link_ethernets`.
- `tests/test_network_link.py` is deleted. New `tests/test_network_netplan.py`, written first (rendering the template with jinja2 as test_network_link did): the client render has no static ethernet and no `10.77` address; the WiFi stanza equals the stanza `test_network_link.py::test_wifi_stanza_is_identical_with_and_without_a_link` pinned; DHCP ethernets still render; the role defaults, tasks and template name neither variable.
- `ansible/README.md`: the network role docs drop the link variables (~317-319). `tests/README.md`: `test_network_link.py` section removed, new tests listed.

## Acceptance conditions

- a1: the client netplan render has no static ethernet and no 10.77 address [evidence: test: tests/test_network_netplan.py::test_client_render_has_no_static_link]
- a2: the WiFi stanza renders byte-identical to before [evidence: test: tests/test_network_netplan.py::test_wifi_stanza_is_unchanged]
- a3: DHCP ethernets still render [evidence: test: tests/test_network_netplan.py::test_dhcp_ethernets_still_render]
- a4: no role file or group var names network_link_ethernets or network_netplan_superseded_files [evidence: command: ! git grep -n -E 'network_link_ethernets|network_netplan_superseded_files' -- ansible]
- a5: test-ansible passes [evidence: command: uv run poe test-ansible]

## Files

- `ansible/roles/network/defaults/main.yml`
- `ansible/roles/network/tasks/main.yml`
- `ansible/roles/network/templates/netplan.yaml.j2`
- `ansible/group_vars/all.yml`
- `tests/test_network_link.py`
- `tests/test_network_netplan.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces


- Network role inputs after the change: `network_address`, `network_gateway`, `network_nameservers`, the DHCP ethernets and WiFi vars, unchanged.

## Expected outcome


The network role has no extra-link support; client and server netplan renders differ from before only by the missing eth0 static stanza on the client.

## Steps


- Write `tests/test_network_netplan.py` (capture the WiFi golden from the current render first); run it and see the no-link and no-variable cases fail.
- Edit the role files and all.yml; delete `tests/test_network_link.py`.
- Run the verify command; update READMEs; commit `refactor(network): drop the Jetson-only link ethernets and superseded files`.

## Notes


- remove-jetson-link-on-client already set `network_link_ethernets: []`; this task removes the variable and its support code.
