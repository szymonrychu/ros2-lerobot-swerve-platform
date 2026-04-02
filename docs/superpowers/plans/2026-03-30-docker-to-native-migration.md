# Docker to Native Install Migration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace all 18 Docker container-based ROS2 nodes with native systemd services backed by Poetry venvs and direct `ros2 launch` commands, then remove Docker entirely from both RPis and the repository.

**Architecture:** Each node gets a wrapper script at `/usr/local/bin/ros2-<name>` (installed as root) that sources ROS2 Jazzy, optionally activates a Poetry venv, then execs the node. Systemd units call this script with `User=<ansible_user>` for unprivileged execution. The `ros2_node_deploy` role gains a `deploy_mode: native` branch alongside the existing Docker branch for incremental rollout with per-node rollback.

**Tech Stack:** Ansible, systemd cgroup v2 (CPUQuota/MemoryMax), Poetry venvs with `--system-site-packages`, ROS2 Jazzy apt packages, Node.js 22 (nodesource) for web_ui frontend, `poe lint-ansible` / `poe test-ansible` for validation.

---

## File Map

**Create:**
- `ansible/roles/ros2_base/tasks/main.yml` — ROS2 Jazzy apt setup, Poetry, Node.js, groups (replaces `docker` role)
- `ansible/roles/ros2_base/meta/main.yml` — role metadata
- `ansible/roles/docker_cleanup/tasks/main.yml` — stop containers, purge Docker, purge config
- `ansible/roles/docker_cleanup/meta/main.yml` — role metadata
- `ansible/roles/ros2_node_deploy/templates/ros2-node-launcher.j2` — per-node wrapper script
- `ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2` — native systemd unit

**Modify:**
- `ansible/roles/ros2_node_deploy/tasks/main.yml` — add native branch, make image/build_context optional
- `ansible/group_vars/all.yml` — add `ros2_node_user: "{{ ansible_user }}"` (used in native template)
- `ansible/group_vars/server.yml` — add native fields (`deploy_mode`, `node_src_dir`, `node_launch_command`, `apt_packages`, `cpu_quota`, `memory_max`) to all type_defaults
- `ansible/group_vars/client.yml` — same
- `ansible/playbooks/tasks/resolve_and_deploy.yml` — pass new vars, make `node_image`/`node_build_context` optional
- `ansible/playbooks/server.yml` — add `ros2_base` role (Phase 1), remove `docker` (Phase 4)
- `ansible/playbooks/client.yml` — same
- `ansible/site.yml` — same
- `ansible/roles/system_optimize/tasks/main.yml` — remove Docker log rotation block (Phase 4)

**Delete (Phase 4):**
- `ansible/roles/docker/` — entire role
- `ansible/roles/ros2_node_deploy/templates/ros2-node.service.j2` — old Docker template
- All 18 Dockerfiles under `nodes/`

**Update (Phase 4):**
- `CLAUDE.md`, all 14 node READMEs, `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`, `bno055-diagnostics/SKILL.md`, `teleop-diagnostics/SKILL.md`
- `scripts/teleop_diag.sh`, `scripts/bno055_diag.sh`, `scripts/rtk_diag.sh`
- `.github/workflows/ci.yml`

---

## Phase 1: Infrastructure

### Task 1: Create `ros2_base` role

**Files:**
- Create: `ansible/roles/ros2_base/meta/main.yml`
- Create: `ansible/roles/ros2_base/tasks/main.yml`

- [ ] **Step 1: Create role meta**

```bash
mkdir -p ansible/roles/ros2_base/tasks
```

Write `ansible/roles/ros2_base/meta/main.yml`:
```yaml
---
galaxy_info:
  role_name: ros2_base
  description: Install ROS2 Jazzy, Poetry, and system dependencies for native node execution
  min_ansible_version: "2.14"
dependencies: []
```

- [ ] **Step 2: Write the role tasks**

Write `ansible/roles/ros2_base/tasks/main.yml`:
```yaml
---
# Bootstrap ROS2 Jazzy on a fresh Ubuntu 24.04 host.
# Installs: apt prerequisites, ROS2 Jazzy, Python tooling, Poetry,
# hardware support libs, Node.js 22 (client only), user groups.

- name: Update apt cache and upgrade packages
  ansible.builtin.apt:
    update_cache: true
    upgrade: dist
    cache_valid_time: 0
  become: true

- name: Install apt prerequisites
  ansible.builtin.apt:
    name:
      - curl
      - gnupg2
      - lsb-release
      - software-properties-common
      - ca-certificates
      - apt-transport-https
    state: present
  become: true

- name: Add ROS2 GPG key
  ansible.builtin.apt_key:
    url: https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    keyring: /usr/share/keyrings/ros-archive-keyring.gpg
    state: present
  become: true

- name: Add ROS2 Jazzy apt repository
  ansible.builtin.apt_repository:
    repo: >-
      deb [arch={{ ansible_architecture | replace('x86_64', 'amd64') | replace('aarch64', 'arm64') }}
      signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]
      http://packages.ros.org/ros2/ubuntu {{ ansible_distribution_release }} main
    state: present
    filename: ros2
  become: true

- name: Install ROS2 Jazzy base and message packages
  ansible.builtin.apt:
    name:
      - ros-jazzy-ros-base
      - ros-jazzy-ros2cli
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
      - ros-jazzy-geometry-msgs
      - ros-jazzy-nav-msgs
      - ros-jazzy-std-msgs
      - ros-jazzy-tf2-ros
      - ros-jazzy-tf2-geometry-msgs
      - ros-jazzy-rosidl-runtime-py
      - ros-jazzy-robot-localization
      - ros-jazzy-navigation2
      - ros-jazzy-nav2-bringup
      - ros-jazzy-rplidar-ros
      - ros-jazzy-realsense2-camera
      - ros-jazzy-launch-ros
      - python3-rosdep
    state: present
    update_cache: true
  become: true

- name: Install Python tooling
  ansible.builtin.apt:
    name:
      - python3-pip
      - python3-venv
      - python3-numpy
      - pipx
    state: present
  become: true

- name: Install Poetry via pipx (system-wide)
  ansible.builtin.command: pipx install poetry --global
  become: true
  register: _poetry_install
  changed_when: "'already installed' not in _poetry_install.stdout"
  failed_when: _poetry_install.rc != 0 and 'already installed' not in _poetry_install.stderr

- name: Install hardware support libraries
  ansible.builtin.apt:
    name:
      - i2c-tools
      - libi2c-dev
      - python3-smbus2
      - libgl1
      - libglib2.0-0
      - libv4l-dev
      - setserial
    state: present
  become: true

- name: Add NodeSource GPG key (client only)
  when: "'client' in group_names"
  ansible.builtin.apt_key:
    url: https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key
    keyring: /usr/share/keyrings/nodesource.gpg
    state: present
  become: true

- name: Add NodeSource Node.js 22 repository (client only)
  when: "'client' in group_names"
  ansible.builtin.apt_repository:
    repo: >-
      deb [signed-by=/usr/share/keyrings/nodesource.gpg]
      https://deb.nodesource.com/node_22.x nodistro main
    state: present
    filename: nodesource
  become: true

- name: Install Node.js 22 (client only)
  when: "'client' in group_names"
  ansible.builtin.apt:
    name: nodejs
    state: present
    update_cache: true
  become: true

- name: Create /opt/ros2-nodes base directory
  ansible.builtin.file:
    path: /opt/ros2-nodes
    state: directory
    owner: root
    group: root
    mode: "0755"
  become: true

- name: Add ansible_user to device access groups
  ansible.builtin.user:
    name: "{{ ansible_user }}"
    groups: "{{ item }}"
    append: true
  become: true
  loop:
    - dialout
    - i2c
    - video
    - plugdev
    - gpio
  failed_when: false
```

- [ ] **Step 3: Run lint**

```bash
cd /path/to/repo && poetry run poe lint-ansible
poetry run poe test-ansible
```
Expected: PASS (no errors referencing new role yet — it's not included in any playbook)

- [ ] **Step 4: Commit**

```bash
git add ansible/roles/ros2_base/
git commit -m "feat(ansible): add ros2_base role for native ROS2 install"
```

---

### Task 2: Create `docker_cleanup` role

**Files:**
- Create: `ansible/roles/docker_cleanup/meta/main.yml`
- Create: `ansible/roles/docker_cleanup/tasks/main.yml`

- [ ] **Step 1: Create role files**

```bash
mkdir -p ansible/roles/docker_cleanup/tasks
```

Write `ansible/roles/docker_cleanup/meta/main.yml`:
```yaml
---
galaxy_info:
  role_name: docker_cleanup
  description: Stop all Docker containers, purge Docker CE and config, remove docker group membership
  min_ansible_version: "2.14"
dependencies: []
```

Write `ansible/roles/docker_cleanup/tasks/main.yml`:
```yaml
---
# Full Docker removal + config purge. Run ONLY after all nodes verified running natively.

- name: Check if Docker is installed
  ansible.builtin.command: which docker
  register: _docker_present
  changed_when: false
  failed_when: false

- name: Stop all running Docker containers
  ansible.builtin.shell: docker stop $(docker ps -aq) 2>/dev/null || true
  become: true
  when: _docker_present.rc == 0
  changed_when: false

- name: Remove all Docker containers, images, volumes, networks
  ansible.builtin.command: docker system prune -af --volumes
  become: true
  when: _docker_present.rc == 0
  changed_when: true
  failed_when: false

- name: Stop Docker services
  ansible.builtin.systemd:
    name: "{{ item }}"
    state: stopped
    enabled: false
  become: true
  loop:
    - docker.service
    - docker.socket
    - containerd.service
  failed_when: false

- name: Purge Docker packages
  ansible.builtin.apt:
    name:
      - docker-ce
      - docker-ce-cli
      - containerd.io
      - docker-buildx-plugin
      - docker-compose-plugin
    state: absent
    purge: true
    autoremove: true
  become: true
  failed_when: false

- name: Remove Docker data and config directories
  ansible.builtin.file:
    path: "{{ item }}"
    state: absent
  become: true
  loop:
    - /var/lib/docker
    - /etc/docker
    - /var/lib/containerd

- name: Remove Docker systemd overrides
  ansible.builtin.file:
    path: "{{ item }}"
    state: absent
  become: true
  loop:
    - /etc/systemd/system/docker.service.d
    - /etc/systemd/system/docker.socket.d
  failed_when: false

- name: Remove docker apt repository file
  ansible.builtin.file:
    path: /etc/apt/sources.list.d/docker.list
    state: absent
  become: true

- name: Remove docker apt key
  ansible.builtin.apt_key:
    url: https://download.docker.com/linux/ubuntu/gpg
    state: absent
  become: true
  failed_when: false

- name: Remove ansible_user from docker group
  ansible.builtin.user:
    name: "{{ ansible_user }}"
    groups: docker
    remove: true
  become: true
  failed_when: false

- name: Reload systemd after Docker removal
  ansible.builtin.systemd:
    daemon_reload: true
  become: true

- name: Autoremove unused packages
  ansible.builtin.apt:
    autoremove: true
  become: true
```

- [ ] **Step 2: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add ansible/roles/docker_cleanup/
git commit -m "feat(ansible): add docker_cleanup role for full Docker removal"
```

---

### Task 3: Create launcher script and native systemd templates

**Files:**
- Create: `ansible/roles/ros2_node_deploy/templates/ros2-node-launcher.j2`
- Create: `ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2`

- [ ] **Step 1: Write launcher script template**

Write `ansible/roles/ros2_node_deploy/templates/ros2-node-launcher.j2`:
```bash
#!/bin/bash
# Launcher for ROS2 node: {{ node_name }}
# Installed by Ansible. Do not edit manually.
source /opt/ros/jazzy/setup.bash
{% if node_src_dir is defined and node_src_dir | length > 0 %}
source /opt/ros2-nodes/{{ node_name }}/venv/bin/activate
{% endif %}
exec bash -c '{{ node_launch_command }}'
```

Note: `exec bash -c '...'` is used instead of bare `exec` so that compound commands (e.g. `ros2 daemon start && sleep infinity`) work correctly. `exec` replaces the script shell with bash, then bash runs the command — signals propagate properly to systemd.

- [ ] **Step 2: Write native systemd unit template**

Write `ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2`:
```ini
[Unit]
Description=ROS2 node {{ node_name }}
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User={{ ansible_user }}
Restart=on-failure
RestartSec=5
TimeoutStopSec=30
CPUQuota={{ node_cpu_quota | default('50%') }}
MemoryMax={{ node_memory_max | default('256M') }}
{% for env in node_env | default([]) %}
Environment={{ env }}
{% endfor %}
ExecStart=/usr/local/bin/ros2-{{ node_name }}

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 3: Verify templates are syntactically valid Jinja2**

```bash
python3 -c "
from jinja2 import Environment
env = Environment()
for f in ['ansible/roles/ros2_node_deploy/templates/ros2-node-launcher.j2',
          'ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2']:
    with open(f) as fh:
        env.parse(fh.read())
    print(f'OK: {f}')
"
```
Expected: `OK: ...` for both files, no exceptions.

- [ ] **Step 4: Commit**

```bash
git add ansible/roles/ros2_node_deploy/templates/ros2-node-launcher.j2
git add ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2
git commit -m "feat(ansible): add native launcher and systemd templates"
```

---

### Task 4: Add native branch to `ros2_node_deploy` role

**Files:**
- Modify: `ansible/roles/ros2_node_deploy/tasks/main.yml`

- [ ] **Step 1: Rewrite tasks/main.yml with native branch**

Replace `ansible/roles/ros2_node_deploy/tasks/main.yml` entirely:

```yaml
---
# Deploy or uninstall a ROS2 node as a native systemd service (deploy_mode: native)
# or as a Docker container (deploy_mode: docker, legacy).
# Expects from resolve_and_deploy.yml:
#   node_name, node_present, node_enabled, node_deploy_mode
#   Native: node_launch_command, node_src_dir (optional), node_apt_packages, node_cpu_quota, node_memory_max
#   Docker: node_image, node_build_context, node_extra_args
#   Both: node_config_path, node_config, node_env, repo_dest

- name: Validate required variables
  ansible.builtin.assert:
    that:
      - node_name is defined
      - node_name | length > 0
    fail_msg: "ros2_node_deploy requires node_name."
    success_msg: "Required vars present for {{ node_name }}"

- name: Validate Docker-mode required variables
  when: (node_deploy_mode | default('docker')) == 'docker'
  ansible.builtin.assert:
    that:
      - node_image is defined
      - node_image | length > 0
      - node_build_context is defined
      - repo_dest is defined
    fail_msg: "Docker deploy mode requires node_image, node_build_context, repo_dest."

- name: Validate native-mode required variables
  when: (node_deploy_mode | default('docker')) == 'native'
  ansible.builtin.assert:
    that:
      - node_launch_command is defined
      - node_launch_command | length > 0
    fail_msg: "Native deploy mode requires node_launch_command."

- name: Set current node name for handlers
  ansible.builtin.set_fact:
    ros2_node_restart_now: "{{ node_name }}"

- name: Validate feetech_servos node config schema (name+id per joint)
  when:
    - node_present | default(true) | bool
    - node_type is defined
    - node_type == "feetech_servos"
  vars:
    _feetech_cfg: "{{ (node_config | default('') | from_yaml) if (node_config | default('') | trim | length > 0) else {} }}"
  ansible.builtin.assert:
    that:
      - _feetech_cfg is mapping
      - _feetech_cfg.namespace is defined
      - _feetech_cfg.joint_names is defined
      - _feetech_cfg.joint_names is sequence
      - _feetech_cfg.joint_names | length > 0
      - (_feetech_cfg.joint_names | select('mapping') | list | length) == (_feetech_cfg.joint_names | length)
      - (_feetech_cfg.joint_names | selectattr('name', 'defined') | list | length) == (_feetech_cfg.joint_names | length)
      - (_feetech_cfg.joint_names | selectattr('id', 'defined') | list | length) == (_feetech_cfg.joint_names | length)
    fail_msg: >-
      feetech_servos node '{{ node_name }}' requires config.joint_names as list of
      objects with both name and id (e.g. [{name: joint_1, id: 1}, ...]).

# ── UNINSTALL ──────────────────────────────────────────────────────────────────

- name: Uninstall ROS2 node (present is false)
  when: not (node_present | default(true) | bool)
  block:
    - name: Stop ROS2 node service
      ansible.builtin.service:
        name: "ros2-{{ node_name }}"
        state: stopped
      become: true
      ignore_errors: true

    - name: Disable ROS2 node service
      ansible.builtin.service:
        name: "ros2-{{ node_name }}"
        enabled: false
      become: true
      ignore_errors: true

    - name: Remove systemd unit for ROS2 node
      ansible.builtin.file:
        path: "/etc/systemd/system/ros2-{{ node_name }}.service"
        state: absent
      become: true

    - name: Remove launcher script (native)
      ansible.builtin.file:
        path: "/usr/local/bin/ros2-{{ node_name }}"
        state: absent
      become: true

    - name: Remove node venv directory (native)
      ansible.builtin.file:
        path: "/opt/ros2-nodes/{{ node_name }}"
        state: absent
      become: true

    - name: Remove ROS2 node config directory
      ansible.builtin.file:
        path: "{{ ros2_node_config_dir }}/{{ node_name }}"
        state: absent
      become: true

    - name: Reload systemd after uninstall
      ansible.builtin.systemd:
        daemon_reload: true
      become: true

# ── DEPLOY: NATIVE ─────────────────────────────────────────────────────────────

- name: Deploy ROS2 node natively (deploy_mode == native)
  when:
    - node_present | default(true) | bool
    - (node_deploy_mode | default('docker')) == 'native'
  block:
    - name: Install node apt packages
      ansible.builtin.apt:
        name: "{{ node_apt_packages | default([]) }}"
        state: present
        update_cache: false
      become: true
      when: (node_apt_packages | default([])) | length > 0

    - name: Create node venv directory
      ansible.builtin.file:
        path: "/opt/ros2-nodes/{{ node_name }}"
        state: directory
        owner: root
        group: root
        mode: "0755"
      become: true
      when: node_src_dir is defined and node_src_dir | length > 0

    - name: Create Poetry venv with system-site-packages
      ansible.builtin.command:
        cmd: python3 -m venv --system-site-packages /opt/ros2-nodes/{{ node_name }}/venv
        creates: /opt/ros2-nodes/{{ node_name }}/venv/bin/python3
      become: true
      when: node_src_dir is defined and node_src_dir | length > 0

    - name: Install pip + Poetry into venv
      ansible.builtin.command:
        cmd: >-
          /opt/ros2-nodes/{{ node_name }}/venv/bin/pip install --quiet --upgrade pip poetry
      become: true
      when: node_src_dir is defined and node_src_dir | length > 0
      changed_when: true

    - name: Install node Python dependencies via Poetry
      ansible.builtin.command:
        cmd: >-
          /opt/ros2-nodes/{{ node_name }}/venv/bin/poetry install
          --only main --no-root --no-interaction
        chdir: "{{ repo_dest }}/{{ node_src_dir }}"
      become: true
      when: node_src_dir is defined and node_src_dir | length > 0
      notify: "Restart ROS2 node"
      changed_when: true

    - name: Build web_ui React frontend (web_ui only)
      when: node_name == 'web_ui'
      block:
        - name: npm ci in web_ui frontend
          ansible.builtin.command:
            cmd: npm ci
            chdir: "{{ repo_dest }}/nodes/web_ui/frontend"
          become: false
          changed_when: true

        - name: npm run build in web_ui frontend
          ansible.builtin.command:
            cmd: npm run build
            chdir: "{{ repo_dest }}/nodes/web_ui/frontend"
          become: false
          changed_when: true
          notify: "Restart ROS2 node"

        - name: Copy built frontend to static dir
          ansible.builtin.copy:
            src: "{{ repo_dest }}/nodes/web_ui/frontend/dist/"
            dest: "{{ repo_dest }}/nodes/web_ui/web_ui/static/"
            remote_src: true
            mode: preserve
          become: true
          notify: "Restart ROS2 node"

    - name: Ensure ROS2 node config directory exists
      ansible.builtin.file:
        path: "{{ ros2_node_config_dir }}/{{ node_name }}"
        state: directory
        mode: "0755"
      become: true

    - name: Deploy node config file (native)
      ansible.builtin.copy:
        dest: "{{ ros2_node_config_dir }}/{{ node_name }}/config.yaml"
        content: "{{ node_config | default('') }}"
        mode: "0644"
      become: true
      when: node_config_path is defined and node_config_path is string and (node_config_path | length > 0)
      notify: "Restart ROS2 node"

    - name: Deploy launcher script
      ansible.builtin.template:
        src: ros2-node-launcher.j2
        dest: "/usr/local/bin/ros2-{{ node_name }}"
        owner: root
        group: root
        mode: "0755"
      become: true
      notify:
        - "Reload systemd"
        - "Restart ROS2 node"

    - name: Create native systemd unit for ROS2 node
      ansible.builtin.template:
        src: ros2-node-native.service.j2
        dest: "/etc/systemd/system/ros2-{{ node_name }}.service"
        mode: "0644"
      become: true
      notify:
        - "Reload systemd"
        - "Restart ROS2 node"

    - name: Set service state and enabled (native)
      ansible.builtin.service:
        name: "ros2-{{ node_name }}"
        state: "{{ 'started' if (node_enabled | default(true) | bool) else 'stopped' }}"
        enabled: "{{ node_enabled | default(true) | bool }}"
        daemon_reload: true
      become: true

    - name: Flush handlers
      ansible.builtin.meta: flush_handlers

# ── DEPLOY: DOCKER (legacy) ────────────────────────────────────────────────────

- name: Deploy ROS2 node as container (deploy_mode == docker)
  when:
    - node_present | default(true) | bool
    - (node_deploy_mode | default('docker')) == 'docker'
  block:
    - name: Pull node container image from registry
      when: node_build_on_controller | default(false) | bool
      ansible.builtin.command:
        cmd: "docker pull {{ node_image }}"
      become: true
      register: docker_pull_result
      changed_when: docker_pull_result.rc == 0
      notify: "Restart ROS2 node"

    - name: Build node container image from repo
      when: not (node_build_on_controller | default(false) | bool)
      ansible.builtin.command:
        cmd: "docker build --network=host -t {{ node_image }} {{ node_build_context }}"
        chdir: "{{ repo_dest }}"
      become: true
      register: docker_build_result
      changed_when: docker_build_result.rc == 0
      notify: "Restart ROS2 node"

    - name: Ensure ROS2 node config directory exists
      ansible.builtin.file:
        path: "{{ ros2_node_config_dir }}/{{ node_name }}"
        state: directory
        mode: "0755"
      become: true

    - name: Deploy node config file (docker)
      ansible.builtin.copy:
        dest: "{{ ros2_node_config_dir }}/{{ node_name }}/config.yaml"
        content: "{{ node_config | default('') }}"
        mode: "0644"
      become: true
      when: node_config_path is defined and node_config_path is string and (node_config_path | length > 0)
      notify: "Restart ROS2 node"

    - name: Create systemd unit for ROS2 node (docker)
      ansible.builtin.template:
        src: ros2-node.service.j2
        dest: "/etc/systemd/system/ros2-{{ node_name }}.service"
        mode: "0644"
      become: true
      notify:
        - "Reload systemd"
        - "Restart ROS2 node"

    - name: Set service state and enabled (docker)
      ansible.builtin.service:
        name: "ros2-{{ node_name }}"
        state: "{{ 'started' if (node_enabled | default(true) | bool) else 'stopped' }}"
        enabled: "{{ node_enabled | default(true) | bool }}"
        daemon_reload: true
      become: true

    - name: Flush handlers
      ansible.builtin.meta: flush_handlers
```

- [ ] **Step 2: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add ansible/roles/ros2_node_deploy/tasks/main.yml
git commit -m "feat(ansible): add native deploy branch to ros2_node_deploy role"
```

---

### Task 5: Update `resolve_and_deploy.yml` to pass native vars

**Files:**
- Modify: `ansible/playbooks/tasks/resolve_and_deploy.yml`

- [ ] **Step 1: Rewrite resolve_and_deploy.yml**

Replace `ansible/playbooks/tasks/resolve_and_deploy.yml` entirely:

```yaml
---
# Resolve a single node from ros2_nodes by name, then invoke the ros2_node_deploy role.
# Required var: _deploy_node_name   — the node's `name` field in ros2_nodes
# Optional var: _extra_env           — additional env list to merge

- name: "Look up {{ _deploy_node_name }} in ros2_nodes"
  ansible.builtin.set_fact:
    _node: "{{ ros2_nodes | selectattr('name', 'equalto', _deploy_node_name) | first }}"

- name: "Deploy {{ _deploy_node_name }}"
  ansible.builtin.include_role:
    name: ros2_node_deploy
  vars:
    node_name: "{{ _node.name }}"
    node_type: "{{ _node.node_type }}"
    node_present: "{{ _node.present | default(true) }}"
    node_enabled: "{{ _node.enabled | default(true) }}"
    node_config: "{{ _node.config | default('') }}"
    repo_dest: "{{ ros2_repo_dest }}"
    node_deploy_mode: "{{ ros2_node_type_defaults[_node.node_type].deploy_mode | default('docker') }}"
    # Native fields
    node_launch_command: "{{ ros2_node_type_defaults[_node.node_type].node_launch_command | default('') }}"
    node_src_dir: "{{ ros2_node_type_defaults[_node.node_type].node_src_dir | default('') }}"
    node_apt_packages: "{{ ros2_node_type_defaults[_node.node_type].apt_packages | default([]) }}"
    node_cpu_quota: "{{ ros2_node_type_defaults[_node.node_type].cpu_quota | default('50%') }}"
    node_memory_max: "{{ ros2_node_type_defaults[_node.node_type].memory_max | default('256M') }}"
    # Docker fields (omit when native to avoid assert failures)
    node_image: "{{ ros2_node_type_defaults[_node.node_type].image | default('') }}"
    node_build_context: "{{ ros2_node_type_defaults[_node.node_type].build_context | default('') }}"
    # Shared
    node_config_path: "{{ ros2_node_type_defaults[_node.node_type].config_path | default(omit) }}"
    node_env: "{{ (ros2_node_type_defaults[_node.node_type].env | default([])) + (_node.env | default([])) + (_extra_env | default([])) }}"
    node_extra_args: "{{ _node.extra_args | default('') }}"
```

- [ ] **Step 2: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add ansible/playbooks/tasks/resolve_and_deploy.yml
git commit -m "feat(ansible): pass native deploy vars through resolve_and_deploy"
```

---

### Task 6: Add `ros2_base` to provisioning playbooks + `ros2_node_user` to all.yml

**Files:**
- Modify: `ansible/group_vars/all.yml`
- Modify: `ansible/playbooks/server.yml`
- Modify: `ansible/playbooks/client.yml`
- Modify: `ansible/site.yml`

- [ ] **Step 1: Add ros2_node_user to all.yml**

In `ansible/group_vars/all.yml`, append after the existing content:
```yaml

# User that runs ROS2 node systemd services (native deploy mode).
# Must be the same as ansible_user (the SSH/sudo user on each RPi).
ros2_node_user: "{{ ansible_user }}"
```

- [ ] **Step 2: Update server.yml provisioning playbook**

Replace `ansible/playbooks/server.yml`:
```yaml
---
# Provision Server Raspberry Pi (Ubuntu 24.04): bootstrap, network, hostname, ROS2
# Run: ansible-playbook -i inventory playbooks/server.yml -l server
- name: Provision Server
  hosts: server
  become: true
  roles:
    - common
    - role: network
      when: network_address is defined and network_address | length > 0
    - role: hostname
      when: hostname is defined and hostname | length > 0
    - docker
    - ros2_base
    - system_optimize
```

- [ ] **Step 3: Update client.yml provisioning playbook**

Replace `ansible/playbooks/client.yml`:
```yaml
---
# Provision Client Raspberry Pi (Ubuntu 24.04): bootstrap, network, hostname, ROS2
# Run: ansible-playbook -i inventory playbooks/client.yml -l client
- name: Provision Client
  hosts: client
  become: true
  roles:
    - common
    - role: network
      when: network_address is defined and network_address | length > 0
    - role: hostname
      when: hostname is defined and hostname | length > 0
    - docker
    - ros2_base
    - system_optimize
```

- [ ] **Step 4: Update site.yml**

Replace `ansible/site.yml`:
```yaml
---
# Full site: provision all hosts (Server + Client), then deploy ROS2 nodes on each.
# Run from ansible/: ansible-playbook -i inventory site.yml

- name: Provision all
  hosts: all
  become: true
  roles:
    - common
    - role: network
      when: network_address is defined and network_address | length > 0
    - role: hostname
      when: hostname is defined and hostname | length > 0
    - docker
    - ros2_base
    - system_optimize

- name: Deploy Server ROS2 nodes
  ansible.builtin.import_playbook: playbooks/deploy_nodes_server.yml

- name: Deploy Client ROS2 nodes
  ansible.builtin.import_playbook: playbooks/deploy_nodes_client.yml
```

- [ ] **Step 5: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add ansible/group_vars/all.yml ansible/playbooks/server.yml ansible/playbooks/client.yml ansible/site.yml
git commit -m "feat(ansible): add ros2_base to provisioning playbooks"
```

---

## Phase 2: Flip Pure ROS2 Nodes to Native

### Task 7: Update server.yml type_defaults — pure ROS2 and Python nodes

**Files:**
- Modify: `ansible/group_vars/server.yml`

- [ ] **Step 1: Add native fields to server ros2_node_type_defaults**

In `ansible/group_vars/server.yml`, replace the `ros2_node_type_defaults` section:

```yaml
ros2_node_type_defaults:
  ros2_master:
    deploy_mode: native
    node_launch_command: "ros2 daemon start && sleep infinity"
    apt_packages:
      - ros-jazzy-ros2cli
    cpu_quota: "5%"
    memory_max: "64M"
    env:
      - ROS_LOCALHOST_ONLY=0
    # Legacy Docker fields — kept for rollback, removed in Phase 4
    image: harbor.szymonrichert.pl/containers/server-ros2-master:latest
    build_context: nodes/ros2_master

  feetech_servos:
    deploy_mode: native
    node_src_dir: nodes/bridges/feetech_servos
    node_launch_command: "python3 -m feetech_servos"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "25%"
    memory_max: "128M"
    config_path: /etc/ros2/feetech_servos
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - ROS_STATIC_PEERS={{ ros2_client_hostname }}
      - FEETECH_SERVOS_CONFIG=/etc/ros2-nodes/feetech_servos/config.yaml
    image: harbor.szymonrichert.pl/containers/server-feetech-servos-leader:latest
    build_context: nodes/bridges/feetech_servos

  topic_scraper_api:
    deploy_mode: native
    node_src_dir: nodes/topic_scraper_api
    node_launch_command: "python3 -m topic_scraper_api"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
      - ros-jazzy-std-msgs
      - ros-jazzy-rosidl-runtime-py
    cpu_quota: "15%"
    memory_max: "128M"
    config_path: /etc/ros2/topic_scraper_api
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - TOPIC_SCRAPER_API_CONFIG=/etc/ros2-nodes/topic_scraper_api/config.yaml
    image: harbor.szymonrichert.pl/containers/server-topic-scraper-api:latest
    build_context: nodes/topic_scraper_api

  gps_rtk:
    deploy_mode: native
    node_src_dir: nodes/bridges/gps_rtk
    node_launch_command: "python3 -m gps_rtk"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "10%"
    memory_max: "64M"
    config_path: /etc/ros2/gps_rtk
    env:
      - ROS_LOCALHOST_ONLY=0
      - GPS_RTK_CONFIG=/etc/ros2-nodes/gps_rtk/config.yaml
    image: harbor.szymonrichert.pl/containers/server-gps-rtk:latest
    build_context: nodes/bridges/gps_rtk
```

Also update the `ros2_nodes` entries — remove `extra_args` from all server nodes (Docker flags ignored for native, but cleaner to remove):

```yaml
ros2_nodes:
  - name: ros2-master
    node_type: ros2_master
    present: true
    enabled: true
  - name: lerobot_leader
    node_type: feetech_servos
    present: true
    enabled: true
    config: |
      device: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059057-if00
      namespace: leader
      log_joint_updates: true
      enable_torque_on_start: false
      disable_torque_on_start: true
      interpolation_enabled: false
      publish_only_on_change: true
      publish_change_epsilon: 0.01
      control_loop_hz: 180
      joint_names:
        - name: shoulder_pan
          id: 1
        - name: shoulder_lift
          id: 2
        - name: elbow_flex
          id: 3
        - name: wrist_flex
          id: 4
        - name: wrist_roll
          id: 5
        - name: gripper
          id: 6
  - name: topic_scraper_api
    node_type: topic_scraper_api
    present: true
    enabled: true
    config: |
      host: "0.0.0.0"
      port: 18100
      topic_refresh_interval_s: 0.25
      allowed_types:
        - sensor_msgs/msg/JointState
        - sensor_msgs/msg/NavSatFix
        - sensor_msgs/msg/Image
        - sensor_msgs/msg/CompressedImage
        - std_msgs/msg/String
  - name: gps_rtk_base
    node_type: gps_rtk
    present: true
    enabled: true
    config: |
      mode: base
      serial_port: /dev/ttyAMA0
      baud_rate: 115200
      topic: /server/gps/fix
      frame_id: gps_link
      configure_on_start: true
      rtcm_tcp_port: 5016
      ntrip_mountpoint: /rtk
      ntrip_user: ""
      ntrip_password: ""
```

Note: `ros2_node_config_dir` is already `/etc/ros2-nodes` (confirmed in `ansible/roles/ros2_node_deploy/defaults/main.yml`). Config env vars in `type_defaults` are updated to `/etc/ros2-nodes/<name>/config.yaml`. The `config_path` field values (e.g. `/etc/ros2/feetech_servos`) are legacy Docker mount paths — they are only used as a truthy flag to decide whether to write the config file; the actual write destination is always `{{ ros2_node_config_dir }}/{{ node_name }}/config.yaml`. Leave `config_path` values as-is; they will be removed in Phase 4 cleanup.

- [ ] **Step 2: Verify ros2_node_config_dir default**

```bash
cat ansible/roles/ros2_node_deploy/defaults/main.yml
```

If `ros2_node_config_dir` is set to something other than `/etc/ros2-nodes`, update it to `/etc/ros2-nodes`. This is the canonical config location for all nodes.

- [ ] **Step 3: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add ansible/group_vars/server.yml
git commit -m "feat(ansible): add native deploy config to server node type defaults"
```

---

### Task 8: Update client.yml type_defaults — all node types

**Files:**
- Modify: `ansible/group_vars/client.yml`

- [ ] **Step 1: Replace ros2_node_type_defaults in client.yml**

Replace the entire `ros2_node_type_defaults` block in `ansible/group_vars/client.yml`:

```yaml
ros2_node_type_defaults:
  ros2_master:
    deploy_mode: native
    node_launch_command: "ros2 daemon start && sleep infinity"
    apt_packages:
      - ros-jazzy-ros2cli
    cpu_quota: "5%"
    memory_max: "64M"
    env:
      - ROS_LOCALHOST_ONLY=1
    image: harbor.szymonrichert.pl/containers/client-ros2-master:latest
    build_context: nodes/ros2_master

  master2master:
    deploy_mode: native
    node_src_dir: nodes/master2master
    node_launch_command: "python3 -m master2master"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
      - ros-jazzy-geometry-msgs
      - ros-jazzy-nav-msgs
    cpu_quota: "30%"
    memory_max: "256M"
    config_path: /etc/ros2/master2master
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - ROS_STATIC_PEERS={{ ros2_server_hostname }};{{ ros2_controller_hostname }}
      - MASTER2MASTER_CONFIG=/etc/ros2-nodes/master2master/config.yaml
    image: harbor.szymonrichert.pl/containers/client-master2master:latest
    build_context: nodes/master2master

  feetech_servos:
    deploy_mode: native
    node_src_dir: nodes/bridges/feetech_servos
    node_launch_command: "python3 -m feetech_servos"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "25%"
    memory_max: "128M"
    config_path: /etc/ros2/feetech_servos
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - FEETECH_SERVOS_CONFIG=/etc/ros2-nodes/feetech_servos/config.yaml
    image: harbor.szymonrichert.pl/containers/client-feetech-servos-follower:latest
    build_context: nodes/bridges/feetech_servos

  uvc_camera:
    deploy_mode: native
    node_src_dir: nodes/bridges/uvc_camera
    node_launch_command: "python3 -m uvc_camera"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "40%"
    memory_max: "256M"
    env:
      - ROS_LOCALHOST_ONLY=1
    image: harbor.szymonrichert.pl/containers/client-uvc-camera:latest
    build_context: nodes/bridges/uvc_camera

  filter_node:
    deploy_mode: native
    node_src_dir: nodes/filter_node
    node_launch_command: "python3 -m filter_node"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "20%"
    memory_max: "128M"
    config_path: /etc/ros2/filter_node
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - FILTER_NODE_CONFIG=/etc/ros2-nodes/filter_node/config.yaml
    image: harbor.szymonrichert.pl/containers/client-filter-node:latest
    build_context: nodes/filter_node

  test_joint_api:
    deploy_mode: native
    node_src_dir: nodes/test_joint_api
    node_launch_command: "python3 -m test_joint_api"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "10%"
    memory_max: "128M"
    config_path: /etc/ros2/test_joint_api
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - TEST_JOINT_API_CONFIG=/etc/ros2-nodes/test_joint_api/config.yaml
    image: harbor.szymonrichert.pl/containers/client-test-joint-api:latest
    build_context: nodes/test_joint_api

  topic_scraper_api:
    deploy_mode: native
    node_src_dir: nodes/topic_scraper_api
    node_launch_command: "python3 -m topic_scraper_api"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
      - ros-jazzy-std-msgs
      - ros-jazzy-rosidl-runtime-py
    cpu_quota: "15%"
    memory_max: "128M"
    config_path: /etc/ros2/topic_scraper_api
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - TOPIC_SCRAPER_API_CONFIG=/etc/ros2-nodes/topic_scraper_api/config.yaml
    image: harbor.szymonrichert.pl/containers/client-topic-scraper-api:latest
    build_context: nodes/topic_scraper_api

  bno055_imu:
    deploy_mode: native
    node_src_dir: nodes/bridges/bno055_imu
    node_launch_command: "python3 -m bno055_imu"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "5%"
    memory_max: "64M"
    config_path: /etc/ros2/bno055_imu
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - BLINKA_FORCECHIP=BCM2XXX
      - BNO055_IMU_CONFIG=/etc/ros2-nodes/bno055_imu/config.yaml
    image: harbor.szymonrichert.pl/containers/client-bno055-imu:latest
    build_context: nodes/bridges/bno055_imu

  haptic_controller:
    deploy_mode: native
    node_src_dir: nodes/haptic_controller
    node_launch_command: "python3 -m haptic_controller"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "15%"
    memory_max: "128M"
    config_path: /etc/ros2/haptic_controller
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - HAPTIC_CONTROLLER_CONFIG=/etc/ros2-nodes/haptic_controller/config.yaml
    image: harbor.szymonrichert.pl/containers/client-haptic-controller:latest
    build_context: nodes/haptic_controller

  gps_rtk:
    deploy_mode: native
    node_src_dir: nodes/bridges/gps_rtk
    node_launch_command: "python3 -m gps_rtk"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "10%"
    memory_max: "64M"
    config_path: /etc/ros2/gps_rtk
    env:
      - ROS_LOCALHOST_ONLY=0
      - GPS_RTK_CONFIG=/etc/ros2-nodes/gps_rtk/config.yaml
    image: harbor.szymonrichert.pl/containers/client-gps-rtk:latest
    build_context: nodes/bridges/gps_rtk

  rplidar_a1:
    deploy_mode: native
    node_launch_command: >-
      ros2 launch {{ ros2_repo_dest }}/nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py
    apt_packages:
      - ros-jazzy-rplidar-ros
      - ros-jazzy-launch-ros
    cpu_quota: "15%"
    memory_max: "128M"
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    image: harbor.szymonrichert.pl/containers/client-rplidar-a1:latest
    build_context: nodes/bridges/rplidar_a1

  realsense_d435i:
    deploy_mode: native
    node_launch_command: >-
      ros2 launch realsense2_camera rs_launch.py
      enable_accel:=true enable_gyro:=true
      unite_imu_method:=linear_interpolation align_depth:=true
    apt_packages:
      - ros-jazzy-realsense2-camera
      - ros-jazzy-launch-ros
    cpu_quota: "80%"
    memory_max: "512M"
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    image: harbor.szymonrichert.pl/containers/client-realsense-d435i:latest
    build_context: nodes/bridges/realsense_d435i

  swerve_controller:
    deploy_mode: native
    node_src_dir: nodes/swerve_drive_controller
    node_launch_command: "python3 -m swerve_drive_controller"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
      - ros-jazzy-geometry-msgs
      - ros-jazzy-nav-msgs
      - ros-jazzy-tf2-ros
      - ros-jazzy-tf2-geometry-msgs
    cpu_quota: "20%"
    memory_max: "128M"
    config_path: /etc/ros2/swerve_drive_controller
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - SWERVE_DRIVE_CONTROLLER_CONFIG=/etc/ros2-nodes/swerve_drive_controller/config.yaml
    image: harbor.szymonrichert.pl/containers/client-swerve-controller:latest
    build_context: nodes/swerve_drive_controller

  static_tf_publisher:
    deploy_mode: native
    node_src_dir: nodes/static_tf_publisher
    node_launch_command: "python3 -m static_tf_publisher"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-tf2-ros
      - ros-jazzy-geometry-msgs
    cpu_quota: "5%"
    memory_max: "64M"
    config_path: /etc/ros2/static_tf_publisher
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - STATIC_TF_PUBLISHER_CONFIG=/etc/ros2-nodes/static_tf_publisher/config.yaml
    image: harbor.szymonrichert.pl/containers/client-static-tf-publisher:latest
    build_context: nodes/static_tf_publisher

  robot_localization_ekf:
    deploy_mode: native
    node_launch_command: >-
      ros2 launch {{ ros2_repo_dest }}/nodes/robot_localization_ekf/launch/ekf.launch.py
    apt_packages:
      - ros-jazzy-robot-localization
      - ros-jazzy-launch-ros
    cpu_quota: "25%"
    memory_max: "128M"
    config_path: /etc/ros2/robot_localization_ekf
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - ROBOT_LOCALIZATION_EKF_CONFIG=/etc/ros2-nodes/robot_localization_ekf/config.yaml
    image: harbor.szymonrichert.pl/containers/client-robot-localization-ekf:latest
    build_context: nodes/robot_localization_ekf

  nav2_bringup:
    deploy_mode: native
    node_launch_command: >-
      ros2 launch nav2_bringup bringup_launch.py
      use_localization:=False
      params_file:={{ ros2_repo_dest }}/nodes/nav2_bringup/config/nav2_params.yaml
    apt_packages:
      - ros-jazzy-navigation2
      - ros-jazzy-nav2-bringup
      - ros-jazzy-launch-ros
    cpu_quota: "100%"
    memory_max: "512M"
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    image: harbor.szymonrichert.pl/containers/client-nav2-bringup:latest
    build_context: nodes/nav2_bringup

  web_ui:
    deploy_mode: native
    node_src_dir: nodes/web_ui
    node_launch_command: "python3 -m web_ui"
    apt_packages:
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
      - ros-jazzy-nav-msgs
      - ros-jazzy-geometry-msgs
      - ros-jazzy-rosidl-runtime-py
    cpu_quota: "50%"
    memory_max: "512M"
    config_path: /etc/ros2/web_ui
    env:
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      - ROS_STATIC_PEERS={{ ros2_client_hostname }}
      - WEB_UI_CONFIG=/etc/ros2-nodes/web_ui/config.yaml
    image: harbor.szymonrichert.pl/containers/client-web-ui:latest
    build_context: nodes/web_ui
```

Also remove `extra_args` from all `ros2_nodes` entries (the Docker flags are now irrelevant). Keep all `config:` blocks and `env:` per-instance overrides unchanged.

- [ ] **Step 2: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add ansible/group_vars/client.yml
git commit -m "feat(ansible): add native deploy config to client node type defaults"
```

---

### Task 9: Provision both RPis with `ros2_base` and deploy all nodes

- [ ] **Step 1: Run provisioning on server (adds ros2_base alongside existing docker)**

```bash
./scripts/deploy-nodes.sh server --all
```
This runs `ansible-playbook playbooks/deploy_nodes_server.yml` which includes `repo_sync` + all 4 server nodes in native mode.

Expected: All 4 server nodes deploy. Check on server RPi:
```bash
ssh ubuntu@server.ros2.lan "systemctl is-active ros2-ros2-master ros2-lerobot_leader ros2-topic_scraper_api ros2-gps_rtk_base"
```
Expected output: 4 lines of `active`.

- [ ] **Step 2: Verify server ROS2 topics**

```bash
ssh ubuntu@server.ros2.lan "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```
Expected topics: `/leader/joint_states`, `/server/gps/fix`, `/rosout`

- [ ] **Step 3: Deploy all client nodes**

```bash
./scripts/deploy-nodes.sh client --all
```

Expected: All 18 client nodes deploy. On client RPi:
```bash
ssh ubuntu@client.ros2.lan "systemctl is-active ros2-ros2-master ros2-master2master ros2-filter_node ros2-lerobot_follower ros2-bno055_imu ros2-web_ui"
```
Expected: all `active`

- [ ] **Step 4: Verify client ROS2 topics**

```bash
ssh ubuntu@client.ros2.lan "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```
Expected: `/filter/input_joint_updates`, `/follower/joint_commands`, `/follower/joint_states`, `/imu/data`, `/scan`, `/odom`, `/client/gps/fix`

- [ ] **Step 5: Quick teleop pipeline check**

```bash
ssh ubuntu@client.ros2.lan "source /opt/ros/jazzy/setup.bash && ros2 topic echo /follower/joint_commands --once"
```
Expected: one `sensor_msgs/msg/JointState` message with position data (non-zero if leader is moving)

- [ ] **Step 6: Web UI check**

Open `http://client.ros2.lan:8080` in browser. Expected: web dashboard loads with tabs visible.

- [ ] **Step 7: Commit if any fixes needed**

```bash
git add -A
git commit -m "fix(ansible): resolve native deploy issues found during first deploy"
```

---

## Phase 3: Docker Cleanup + Full Repo Cleanup

### Task 10: Run docker_cleanup on both RPis

**Files:**
- Create: `ansible/playbooks/docker_cleanup.yml`

- [ ] **Step 1: Create a one-shot cleanup playbook**

Write `ansible/playbooks/docker_cleanup.yml`:
```yaml
---
# Run ONLY after all nodes verified running natively.
# Stops all containers, purges Docker CE, removes /var/lib/docker and /etc/docker.
- name: Remove Docker from server and client
  hosts: server:client
  become: true
  roles:
    - docker_cleanup
```

- [ ] **Step 2: Run cleanup**

```bash
cd ansible && ansible-playbook -i inventory playbooks/docker_cleanup.yml
```
Expected: runs without error. Verify on each RPi:
```bash
ssh ubuntu@server.ros2.lan "systemctl is-active docker || echo 'docker not running'"
ssh ubuntu@client.ros2.lan "systemctl is-active docker || echo 'docker not running'"
```
Expected: `docker not running` on both.

- [ ] **Step 3: Verify all native services still running after Docker removal**

```bash
ssh ubuntu@server.ros2.lan "systemctl is-active ros2-ros2-master ros2-lerobot_leader ros2-topic_scraper_api ros2-gps_rtk_base"
ssh ubuntu@client.ros2.lan "systemctl is-active ros2-ros2-master ros2-filter_node ros2-web_ui ros2-bno055_imu"
```
Expected: all `active`

- [ ] **Step 4: Commit**

```bash
git add ansible/playbooks/docker_cleanup.yml
git commit -m "feat(ansible): add docker_cleanup playbook"
```

---

### Task 11: Remove Docker role from provisioning, delete Dockerfiles, clean Ansible

**Files:**
- Modify: `ansible/playbooks/server.yml`
- Modify: `ansible/playbooks/client.yml`
- Modify: `ansible/site.yml`
- Modify: `ansible/roles/ros2_node_deploy/tasks/main.yml`
- Modify: `ansible/group_vars/server.yml`
- Modify: `ansible/group_vars/client.yml`
- Modify: `ansible/roles/system_optimize/tasks/main.yml`
- Delete: `ansible/roles/docker/` (entire directory)
- Delete: `ansible/roles/ros2_node_deploy/templates/ros2-node.service.j2`
- Delete: all 18 Dockerfiles

- [ ] **Step 1: Remove `docker` role from provisioning playbooks**

`ansible/playbooks/server.yml`:
```yaml
---
- name: Provision Server
  hosts: server
  become: true
  roles:
    - common
    - role: network
      when: network_address is defined and network_address | length > 0
    - role: hostname
      when: hostname is defined and hostname | length > 0
    - ros2_base
    - system_optimize
```

`ansible/playbooks/client.yml`:
```yaml
---
- name: Provision Client
  hosts: client
  become: true
  roles:
    - common
    - role: network
      when: network_address is defined and network_address | length > 0
    - role: hostname
      when: hostname is defined and hostname | length > 0
    - ros2_base
    - system_optimize
```

`ansible/site.yml`:
```yaml
---
- name: Provision all
  hosts: all
  become: true
  roles:
    - common
    - role: network
      when: network_address is defined and network_address | length > 0
    - role: hostname
      when: hostname is defined and hostname | length > 0
    - ros2_base
    - system_optimize

- name: Deploy Server ROS2 nodes
  ansible.builtin.import_playbook: playbooks/deploy_nodes_server.yml

- name: Deploy Client ROS2 nodes
  ansible.builtin.import_playbook: playbooks/deploy_nodes_client.yml
```

- [ ] **Step 2: Remove Docker branch from `ros2_node_deploy/tasks/main.yml`**

Delete the entire `Deploy ROS2 node as container (deploy_mode == docker)` block from `ansible/roles/ros2_node_deploy/tasks/main.yml`.

Also remove the Docker validation block:
```yaml
- name: Validate Docker-mode required variables
  ...
```

Remove the `node_deploy_mode` default check — all nodes are now native, so the `deploy_mode` field is no longer conditional logic; simplify the native block's `when:` to just `when: node_present | default(true) | bool`.

- [ ] **Step 3: Remove legacy fields from group_vars**

In `ansible/group_vars/server.yml`, remove `image:`, `build_context:` from every `ros2_node_type_defaults` entry.

In `ansible/group_vars/client.yml`, same.

Also remove `deploy_mode: native` — it's the only mode now, so remove the field entirely.

- [ ] **Step 4: Remove Docker log rotation from system_optimize**

In `ansible/roles/system_optimize/tasks/main.yml`, find and remove any task that merges Docker log rotation into `daemon.json` (grep for `daemon.json` or `log-driver`).

- [ ] **Step 5: Delete docker role and old template**

```bash
git rm -r ansible/roles/docker/
git rm ansible/roles/ros2_node_deploy/templates/ros2-node.service.j2
```

- [ ] **Step 6: Delete all 18 Dockerfiles**

```bash
git rm nodes/ros2_master/Dockerfile
git rm nodes/master2master/Dockerfile
git rm nodes/filter_node/Dockerfile
git rm nodes/lerobot_teleop/Dockerfile
git rm nodes/test_joint_api/Dockerfile
git rm nodes/haptic_controller/Dockerfile
git rm nodes/topic_scraper_api/Dockerfile
git rm nodes/swerve_drive_controller/Dockerfile
git rm nodes/static_tf_publisher/Dockerfile
git rm nodes/robot_localization_ekf/Dockerfile
git rm nodes/nav2_bringup/Dockerfile
git rm nodes/web_ui/Dockerfile
git rm nodes/bridges/feetech_servos/Dockerfile
git rm nodes/bridges/uvc_camera/Dockerfile
git rm nodes/bridges/bno055_imu/Dockerfile
git rm nodes/bridges/gps_rtk/Dockerfile
git rm nodes/bridges/rplidar_a1/Dockerfile
git rm nodes/bridges/realsense_d435i/Dockerfile
```

- [ ] **Step 7: Run lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```
Expected: PASS

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "feat(ansible): remove Docker role, old template, Dockerfiles — native only"
```

---

### Task 12: Update diagnostic scripts

**Files:**
- Modify: `scripts/teleop_diag.sh`
- Modify: `scripts/bno055_diag.sh`
- Modify: `scripts/rtk_diag.sh`

- [ ] **Step 1: Update teleop_diag.sh**

Find every occurrence of:
- `docker ps --filter name=ros2-` → replace with `systemctl is-active ros2-`
- `docker logs ros2-<name>` → replace with `journalctl -u ros2-<name> --no-pager -n 50`
- Any `docker inspect` or `docker stats` → replace with `systemctl status ros2-<name>`

Exact pattern to replace (run on dev machine, changes the script):
```bash
# In scripts/teleop_diag.sh:
# "docker ps" health checks → systemctl
sed -i 's|docker ps --filter name=ros2-\([^ ]*\) --format.*|systemctl is-active ros2-\1|g' scripts/teleop_diag.sh
# "docker logs" → journalctl (manual edit — inspect each occurrence)
```

Open the file and do the replacements manually (using Edit tool) after inspecting exact patterns.

- [ ] **Step 2: Update bno055_diag.sh**

Same approach — find and replace `docker ps`/`docker logs`/`docker inspect` with `systemctl`/`journalctl` equivalents.

- [ ] **Step 3: Update rtk_diag.sh**

Same approach.

- [ ] **Step 4: Test scripts (SSH required)**

```bash
./scripts/bno055_diag.sh
./scripts/rtk_diag.sh
```
Expected: scripts run, show service status via systemctl output, no `docker: command not found` errors.

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop_diag.sh scripts/bno055_diag.sh scripts/rtk_diag.sh
git commit -m "fix(scripts): replace docker ps/logs with systemctl/journalctl"
```

---

### Task 13: Update CI, skills, CLAUDE.md, and READMEs

**Files:**
- Modify: `.github/workflows/ci.yml`
- Modify: `.claude/skills/ansible-deploy/SKILL.md`
- Modify: `.claude/skills/bno055-diagnostics/SKILL.md`
- Modify: `.claude/skills/teleop-diagnostics/SKILL.md`
- Modify: `CLAUDE.md`
- Modify: `ansible/README.md`
- Modify: 14 node READMEs under `nodes/`

- [ ] **Step 1: Update CI — remove docker-build job**

In `.github/workflows/ci.yml`, delete the entire `docker-build:` job block (the job that runs `docker/build-push-action` for arm64 builds). Keep all lint/test jobs unchanged.

- [ ] **Step 2: Update ansible-deploy skill**

In `.claude/skills/ansible-deploy/SKILL.md`:
- Remove all references to `docker ps`, `docker logs`, container restart loops, Docker build TLS timeout
- Replace container health check instructions with: `systemctl is-active ros2-<name>` and `journalctl -u ros2-<name> -n 50`
- Replace "restart container" instructions with: `systemctl restart ros2-<name>`
- Remove Docker-specific troubleshooting section
- Update any mention of "containers" to "services"

- [ ] **Step 3: Update bno055-diagnostics skill**

In `.claude/skills/bno055-diagnostics/SKILL.md`:
- Remove `--device=/dev/i2c-1` container arg references
- Remove "check container running" instructions
- Replace with: `systemctl status ros2-bno055_imu` and `journalctl -u ros2-bno055_imu -n 30`

- [ ] **Step 4: Update teleop-diagnostics skill**

In `.claude/skills/teleop-diagnostics/SKILL.md`:
- Remove "Container crashed" troubleshooting section
- Replace all container references with native systemd service commands

- [ ] **Step 5: Update CLAUDE.md**

Make the following changes in `CLAUDE.md`:

**Tech Stack section** — remove "Docker and docker-compose":
```markdown
## Tech Stack

- ROS2 Jazzy (Ubuntu 24.04)
- Ansible
- Ubuntu 24.04
- Client: Raspberry Pi 5
- Server: Raspberry Pi 4b
- Dev: MacBook M4
```

**Replace "Container and Source Changes" section** with:
```markdown
## Node Source Changes

Whenever source code in a node directory changes, redeploy that node with `deploy-nodes.sh` to reinstall the Poetry venv and restart the service. Node services run natively (no Docker); source lives at `/opt/ros2-lerobot-swerve-platform/` on each RPi and venvs at `/opt/ros2-nodes/<name>/venv/`.
```

**Remove numpy dependency guard** Docker-specific text — replace with:
```markdown
### numpy dependency guard (mandatory)

Before finalising any ROS2 Python node, verify that `numpy` is in the node's Poetry dependencies when needed. The venv uses `--system-site-packages` so system numpy is visible, but explicitly declaring it in `pyproject.toml` is required to avoid version conflicts.
```

- [ ] **Step 6: Update all 14 node READMEs**

For each README under `nodes/`, remove or replace:
- "Build" sections with `docker build ...` commands
- "Run" sections with `docker run ...` commands

Replace with standard section:
```markdown
## Deployment

Deploy via Ansible:
```bash
./scripts/deploy-nodes.sh <client|server> <node_name>
```

The node runs as a systemd service `ros2-<name>`. Check status:
```bash
ssh ubuntu@<host>.ros2.lan "systemctl status ros2-<name>"
ssh ubuntu@<host>.ros2.lan "journalctl -u ros2-<name> -n 50"
```
```

- [ ] **Step 7: Update ansible/README.md**

Add a **Node Resource Limits** section after the existing deployment instructions:

```markdown
## Node Resource Limits

Each node runs as a native systemd service with cgroup v2 resource limits (CPUQuota / MemoryMax).

### Server (RPi 4b)

| Node | CPUQuota | MemoryMax | Purpose |
|------|----------|-----------|---------|
| `ros2-master` | 5% | 64M | ROS2 daemon |
| `lerobot_leader` | 25% | 128M | 180 Hz servo control |
| `topic_scraper_api` | 15% | 128M | HTTP topic inspection API |
| `gps_rtk_base` | 10% | 64M | GPS RTK base station |

### Client (RPi 5)

| Node | CPUQuota | MemoryMax | Purpose |
|------|----------|-----------|---------|
| `ros2-master` | 5% | 64M | ROS2 daemon |
| `master2master` | 30% | 256M | Multi-topic relay (server↔client↔controller) |
| `filter_node` | 20% | 128M | Kalman filter 100 Hz |
| `test_joint_api` | 10% | 128M | HTTP joint testing API |
| `topic_scraper_api` | 15% | 128M | HTTP topic inspection API |
| `bno055_imu` | 5% | 64M | I2C IMU at 50 Hz |
| `gps_rtk_rover` | 10% | 64M | GPS RTK rover |
| `haptic_controller` | 15% | 128M | Force feedback (disabled by default) |
| `gripper_uvc_camera` | 40% | 256M | USB camera capture |
| `rplidar_a1` | 15% | 128M | LIDAR scan |
| `realsense_d435i` | 80% | 512M | Depth + color + IMU streams |
| `lerobot_follower` | 25% | 128M | 180 Hz servo control |
| `swerve_drive_servos` | 25% | 128M | 8-servo swerve drive |
| `swerve_controller` | 20% | 128M | Swerve kinematics 50 Hz |
| `static_tf_publisher` | 5% | 64M | Static TF frames |
| `robot_localization_ekf` | 25% | 128M | EKF sensor fusion |
| `nav2_bringup` | 100% | 512M | Nav2 navigation stack |
| `web_ui` | 50% | 512M | Web dashboard (FastAPI + React) |
```

- [ ] **Step 8: Run final lint**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
poetry run poe lint
```
Expected: PASS

- [ ] **Step 9: Commit all documentation and CI changes**

```bash
git add .github/workflows/ci.yml
git add .claude/skills/
git add CLAUDE.md
git add ansible/README.md
git add nodes/
git commit -m "docs: remove Docker references, update skills, CI, and READMEs for native deploy"
```

---

## Final Verification

- [ ] **Verify all services active on server**

```bash
ssh ubuntu@server.ros2.lan "systemctl is-active ros2-ros2-master ros2-lerobot_leader ros2-topic_scraper_api ros2-gps_rtk_base"
```
Expected: `active` for all 4

- [ ] **Verify all services active on client**

```bash
ssh ubuntu@client.ros2.lan "systemctl is-active \
  ros2-ros2-master ros2-master2master ros2-filter_node \
  ros2-test_joint_api ros2-topic_scraper_api ros2-bno055_imu \
  ros2-gps_rtk_rover ros2-gripper_uvc_camera ros2-rplidar_a1 \
  ros2-realsense_d435i ros2-lerobot_follower ros2-swerve_drive_servos \
  ros2-swerve_controller ros2-static_tf_publisher \
  ros2-robot_localization_ekf ros2-nav2_bringup ros2-web_ui"
```
Expected: `active` for all enabled services (haptic_controller is disabled — skip it)

- [ ] **Verify Docker is fully gone from both RPis**

```bash
ssh ubuntu@server.ros2.lan "which docker 2>/dev/null && echo FOUND || echo 'docker not installed'"
ssh ubuntu@client.ros2.lan "which docker 2>/dev/null && echo FOUND || echo 'docker not installed'"
```
Expected: `docker not installed` on both

- [ ] **Verify cgroup limits applied**

```bash
ssh ubuntu@client.ros2.lan "systemctl show ros2-web_ui --property=CPUQuotaPerSecUSec,MemoryMax"
```
Expected: `CPUQuotaPerSecUSec=500000` (50%), `MemoryMax=536870912` (512M)

- [ ] **Full teleop smoke test**

```bash
ssh ubuntu@client.ros2.lan "source /opt/ros/jazzy/setup.bash && ros2 topic echo /follower/joint_states --once"
```
Expected: one `JointState` message with position array

- [ ] **Web UI smoke test**

Open `http://client.ros2.lan:8080`. Expected: dashboard loads, RGBD/IMU/servo tabs visible.
