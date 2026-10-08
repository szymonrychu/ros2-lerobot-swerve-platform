#!/usr/bin/env bash
# Prove, in an Ubuntu 24.04 arm64 container (the RPi OS), that the Ansible node venv recipe works with uv:
#   1. the ros2_base task installs the pinned uv via pipx, replaces a wrong version and is a no-op afterwards;
#   2. the ros2_node_deploy venv task (python3 -m venv --system-site-packages) plus its uv sync task, both read
#      verbatim from the role YAML, sync a node project with an editable ../../shared path dependency;
#   3. the venv keeps include-system-site-packages = true, an apt Python module (python3-yaml, standing in for
#      rclpy) imports from the venv python, the path dependency and the node package import, dev deps stay out.
# Uses nodes/mcp_server + shared/ when nodes/mcp_server/uv.lock exists, else a minimal fixture of the same shape.
# Usage: from repo root, run: ./scripts/probe-uv-venv-docker.sh
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="ubuntu:24.04"

docker run --rm -i --platform linux/arm64 -v "$PWD:/work:ro" "$IMAGE" bash -s <<'PROBE'
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "PASS: $*"; }

echo "== host: $(uname -m), $(. /etc/os-release && echo "$PRETTY_NAME")"
apt-get update -qq >/dev/null
apt-get install -y -qq python3 python3-venv python3-yaml pipx ca-certificates >/dev/null
echo "== system python: $(python3 --version)"

# Read a task out of a role's tasks/main.yml (descending into blocks), print one field of it.
cat > /tmp/task.py <<'PY'
import json, sys, yaml

def walk(tasks):
    for t in tasks:
        for key in ("block", "rescue", "always"):
            yield from walk(t.get(key, []))
        yield t

path, name, field = sys.argv[1:4]
task = next(t for t in walk(yaml.safe_load(open(path))) if t.get("name") == name)
value = task
for part in field.split("/"):
    value = value[part]
print(json.dumps(value) if isinstance(value, dict) else value)
PY
BASE=/work/ansible/roles/ros2_base
DEPLOY=/work/ansible/roles/ros2_node_deploy/tasks/main.yml
UV_VERSION=$(python3 -c "import yaml; print(yaml.safe_load(open('$BASE/defaults/main.yml'))['ros2_uv_version'])")

# --- 1. ros2_base: pinned uv install --------------------------------------------------------------------------
UV_TASK=$(python3 /tmp/task.py "$BASE/tasks/main.yml" "Install the pinned uv via pipx (system-wide)" ansible.builtin.shell)
UV_TASK=${UV_TASK//'{{ ros2_uv_version }}'/$UV_VERSION}
export PIPX_HOME=/opt/pipx PIPX_BIN_DIR=/usr/local/bin
echo "== preinstall a wrong uv (0.11.28) to prove the task replaces it"
pipx install -q "uv==0.11.28" >/dev/null 2>&1
echo "   before: $(/usr/local/bin/uv --version)"
out=$(bash -c "$UV_TASK" 2>&1)
case "$out" in *"already installed"*) fail "wrong uv version was kept";; esac
[ "$(/usr/local/bin/uv --version | cut -d' ' -f2)" = "$UV_VERSION" ] || fail "uv not at $UV_VERSION"
pass "ros2_base task replaced uv 0.11.28 with $(/usr/local/bin/uv --version)"
out=$(bash -c "$UV_TASK" 2>&1)
case "$out" in *"uv $UV_VERSION already installed"*) pass "second run is a no-op: $out";; *) fail "second run reinstalled: $out";; esac
unset PIPX_HOME PIPX_BIN_DIR

# --- 2. the node project ---------------------------------------------------------------------------------------
REPO=/srv/repo
mkdir -p "$REPO"
if [ -f /work/nodes/mcp_server/uv.lock ]; then
  NODE_SRC_DIR=nodes/mcp_server NODE_NAME=mcp_server NODE_MODULE=mcp_server DEV_MODULE=pytest
  mkdir -p "$REPO/nodes"
  cp -r /work/shared "$REPO/shared"
  cp -r /work/nodes/mcp_server "$REPO/nodes/mcp_server"
  rm -rf "$REPO/nodes/mcp_server/.venv" "$REPO/shared/.venv"
  echo "== node: real nodes/mcp_server (its uv.lock exists)"
else
  NODE_SRC_DIR=nodes/probe_node NODE_NAME=probe_node NODE_MODULE=probe_node DEV_MODULE=iniconfig
  echo "== node: fixture nodes/probe_node (nodes/mcp_server/uv.lock not in this tree) - same shape as mcp_server"
  mkdir -p "$REPO/shared/ros2_common" "$REPO/nodes/probe_node/probe_node"
  cat > "$REPO/shared/pyproject.toml" <<'TOML'
[project]
name = "ros2-common"
version = "0.1.0"
requires-python = ">=3.11,<3.15"
dependencies = []

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
TOML
  echo 'SHARED = "ros2_common from shared/"' > "$REPO/shared/ros2_common/__init__.py"
  cat > "$REPO/nodes/probe_node/pyproject.toml" <<'TOML'
[project]
name = "probe-node"
version = "0.1.0"
requires-python = ">=3.12,<3.15"
dependencies = ["ros2-common", "six>=1.16"]

[dependency-groups]
dev = ["iniconfig>=2.0"]

[tool.uv.sources]
ros2-common = { path = "../../shared", editable = true }

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
TOML
  echo 'import ros2_common, six' > "$REPO/nodes/probe_node/probe_node/__init__.py"
  (cd "$REPO/nodes/probe_node" && UV_PYTHON_DOWNLOADS=never /usr/local/bin/uv lock --python /usr/bin/python3 -q)
  echo "   uv.lock packages: $(grep '^name = ' "$REPO/nodes/probe_node/uv.lock" | cut -d'"' -f2 | tr '\n' ' ')"
fi

# --- 3. ros2_node_deploy: venv + uv sync, verbatim from the role -----------------------------------------------
render() { local s=$1; s=${s//'{{ node_name }}'/$NODE_NAME}; s=${s//'{{ node_deps_key }}'/probe-deps-key};
  s=${s//'{{ repo_dest }}'/$REPO}; s=${s//'{{ node_src_dir }}'/$NODE_SRC_DIR}; printf '%s' "$s"; }
VENV_CMD=$(render "$(python3 /tmp/task.py "$DEPLOY" "Create node venv with system-site-packages" ansible.builtin.command/cmd)")
SYNC_CMD=$(render "$(python3 /tmp/task.py "$DEPLOY" "Install node Python dependencies with uv" ansible.builtin.shell)")
CHDIR=$(render "$(python3 /tmp/task.py "$DEPLOY" "Install node Python dependencies with uv" args/chdir)")
ENV_JSON=$(render "$(python3 /tmp/task.py "$DEPLOY" "Install node Python dependencies with uv" environment)")
ENV_ARGS=$(python3 -c "import json, shlex, sys; print(' '.join(shlex.quote(f'{k}={v}') for k, v in json.loads(sys.argv[1]).items()))" "$ENV_JSON")
VENV=/opt/ros2-nodes/$NODE_NAME/venv
mkdir -p "/opt/ros2-nodes/$NODE_NAME"

echo "== venv task:  $VENV_CMD"
eval "$VENV_CMD"
echo "== sync task:  (cd $CHDIR && env $ENV_ARGS bash -c \"$SYNC_CMD\")"
for run in first second; do
  if out=$(cd "$CHDIR" && eval "env $ENV_ARGS bash -c \"\$SYNC_CMD\"" 2>&1); then rc=0; else rc=$?; fi
  printf '%s\n' "$out" | sed 's/^/   uv: /'
  [ "$rc" -eq 0 ] || fail "uv sync ($run run) failed with rc=$rc"
  pass "uv sync succeeded ($run run)"
done

# --- 4. checks ---------------------------------------------------------------------------------------------------
PY="$VENV/bin/python3"
echo "== $VENV/pyvenv.cfg:"; sed 's/^/   /' "$VENV/pyvenv.cfg"
grep -qx 'include-system-site-packages = true' "$VENV/pyvenv.cfg" && pass "venv still has include-system-site-packages = true" \
  || fail "uv sync dropped --system-site-packages"
[ "$("$PY" -c 'import sys; print(sys.prefix)')" = "$VENV" ] && pass "venv python: $("$PY" --version), prefix $VENV" || fail "venv python prefix"
yaml_path=$("$PY" -c 'import yaml; print(yaml.__file__)')
case "$yaml_path" in /usr/lib/python3/dist-packages/*) pass "apt module (rclpy stand-in) imports from the venv: yaml -> $yaml_path";;
  *) fail "yaml came from $yaml_path";; esac
shared_path=$("$PY" -c 'import ros2_common; print(ros2_common.__file__)')
[ "$shared_path" = "$REPO/shared/ros2_common/__init__.py" ] && pass "editable path dependency imports from the checkout: ros2_common -> $shared_path" \
  || fail "ros2_common came from $shared_path"
node_path=$("$PY" -c "import $NODE_MODULE; print($NODE_MODULE.__file__)")
case "$node_path" in "$REPO/$NODE_SRC_DIR/"*) pass "node package installed editable: $NODE_MODULE -> $node_path";; *) fail "node package at $node_path";; esac
"$PY" -c "import $DEV_MODULE" 2>/dev/null && fail "--no-dev installed dev dependency $DEV_MODULE" || pass "--no-dev: dev dependency $DEV_MODULE not installed"
[ "$(cat "$VENV/.uv-deps")" = probe-deps-key ] && pass "stamp $VENV/.uv-deps written after the sync" || fail "stamp missing"
echo "== ALL CHECKS PASSED"
PROBE
