#!/usr/bin/env bash
# Prove, in a ros:jazzy-ros-base container on the host's architecture (arm64 on the M4 Mac = the Pis' Ubuntu 24.04
# arm64), that the rtabmap_mapping launch installs from the split apt packages and loads with every parameter
# accepted, before anything is deployed to the server Pi 4b (design D1):
#   1. apt-install ros-jazzy-rtabmap-{slam,sync,util,msgs,conversions}, ros-jazzy-image-transport-plugins and
#      python3-pydantic (the system python the launch file runs with), never the rtabmap_ros metapackage or rtabmap_viz;
#   2. the map3d contract loads and rejects a crafted relay loop under the apt pydantic;
#   3. launch nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py (repo mounted read-only) for LAUNCH_SECONDS with
#      a temporary database dir passed through an RTABMAP_MAPPING_CONFIG override file;
#   4. fail unless: both components loaded, /rtabmap/backup and /rtabmap/reset exist, every contract input has a
#      subscriber (the compressedDepth topic included), TF is read only from /uplink/tf*, every launch parameter is
#      present on the running node with the launch value (an unknown name is silently dropped by ROS 2, so this is the
#      check that catches it), no log line reports a renamed / unknown / ignored parameter, nothing is published on
#      /tf or /uplink/tf, the container survived the whole run and RTAB-Map created live.db in the database dir.
# Prints PASS/FAIL per check and one final line "rtabmap-launch-probe: PASS|FAIL"; exits non-zero on failure.
# The first run downloads the image and PCL/VTK/gtsam (several minutes); apt archives are cached in the docker volume
# rtabmap-launch-probe-apt, so repeat runs skip the download.
# Usage: from repo root, run: ./scripts/rtabmap-launch-probe.sh
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

IMAGE="ros:jazzy-ros-base"
LAUNCH_SECONDS="${PROBE_LAUNCH_SECONDS:-20}"
LOAD_TIMEOUT_S="${PROBE_LOAD_TIMEOUT_S:-90}"
APT_CACHE_VOLUME="rtabmap-launch-probe-apt"
PACKAGES="ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-sync ros-jazzy-rtabmap-util ros-jazzy-rtabmap-msgs ros-jazzy-rtabmap-conversions ros-jazzy-image-transport-plugins python3-pydantic"
FORBIDDEN_PACKAGES="ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-viz"
LAUNCH_FILE="nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py"

case "$(uname -m)" in
  arm64 | aarch64) PLATFORM="linux/arm64" ;;
  x86_64 | amd64) PLATFORM="linux/amd64" ;;
  *) echo "FAIL: unsupported host architecture $(uname -m)"; echo "rtabmap-launch-probe: FAIL"; exit 1 ;;
esac

if [ ! -f "$LAUNCH_FILE" ]; then
  echo "FAIL: $LAUNCH_FILE not found"
  echo "rtabmap-launch-probe: FAIL"
  exit 1
fi

docker run --rm -i --platform "$PLATFORM" -v "$PWD:/work:ro" -v "$APT_CACHE_VOLUME:/var/cache/apt/archives" \
  -e PACKAGES="$PACKAGES" -e FORBIDDEN_PACKAGES="$FORBIDDEN_PACKAGES" -e LAUNCH_FILE="/work/$LAUNCH_FILE" \
  -e LAUNCH_SECONDS="$LAUNCH_SECONDS" -e LOAD_TIMEOUT_S="$LOAD_TIMEOUT_S" \
  "$IMAGE" bash -s <<'PROBE'
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
FAILED=0
fail() { echo "FAIL: $*"; FAILED=1; }
pass() { echo "PASS: $*"; }
die() { echo "FAIL: $*"; exit 1; }
alive() { [ -r "/proc/$1/status" ] && ! grep -q '^State:.*Z' "/proc/$1/status"; }  # an unreaped child is a zombie

echo "== host: $(uname -m), $(. /etc/os-release && echo "$PRETTY_NAME"), ROS_DISTRO=$ROS_DISTRO"

# --- 1. apt --------------------------------------------------------------------------------------------------------
rm -f /etc/apt/apt.conf.d/docker-clean  # keep downloaded archives in the cache volume
apt-get update -qq >/dev/null || die "apt-get update"
# shellcheck disable=SC2086
apt-get install -y -qq $PACKAGES > /tmp/apt.log 2>&1 || { tail -20 /tmp/apt.log; die "apt-get install $PACKAGES"; }
pass "apt installed: $(dpkg-query -W -f='${Package} ${Version}\n' ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-sync python3-pydantic | tr '\n' ';')"
for pkg in $FORBIDDEN_PACKAGES; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then fail "$pkg got installed"; else pass "$pkg not installed"; fi
done

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

# --- 2. contract under the apt pydantic ----------------------------------------------------------------------------
python3 - <<'PY' && pass "map3d contract loads and validates under the apt pydantic" || fail "map3d contract under the apt pydantic"
import copy
import sys

import pydantic
import yaml

sys.path.insert(0, "/work/nodes/rtabmap_mapping")
from map3d_topics import DEFAULT_YAML, Map3dTopics, load_map3d_topics

print(f"   pydantic {pydantic.VERSION}, {len(load_map3d_topics().uplink)} uplink streams")
doc = yaml.safe_load(DEFAULT_YAML.read_text())
loop = copy.deepcopy(doc)
loop["uplink"][0]["dest"] = loop["uplink"][1]["source"]
try:
    Map3dTopics(**loop)
except pydantic.ValidationError as err:
    print(f"   relay loop rejected: {str(err).splitlines()[-1].strip()}")
else:
    sys.exit("relay loop accepted")
PY

# --- 3. launch -----------------------------------------------------------------------------------------------------
DB_DIR=$(mktemp -d /tmp/rtabmap-db.XXXXXX)
OVERRIDE="$DB_DIR/override.yaml"
printf 'database_dir: %s\n' "$DB_DIR" > "$OVERRIDE"
LOG=/tmp/launch.log
echo "== ros2 launch $LAUNCH_FILE for ${LAUNCH_SECONDS}s, database_dir $DB_DIR"
START=$(date +%s)
export RTABMAP_MAPPING_CONFIG="$OVERRIDE"
setsid ros2 launch "$LAUNCH_FILE" > "$LOG" 2>&1 &
LAUNCH_PID=$!

loaded=0
for _ in $(seq "$LOAD_TIMEOUT_S"); do
  if grep -q "Loaded node '/rtabmap/rgbd_sync'" "$LOG" && grep -q "Loaded node '/rtabmap/rtabmap'" "$LOG"; then
    loaded=1
    break
  fi
  alive "$LAUNCH_PID" || break
  sleep 1
done
if [ "$loaded" = 1 ]; then
  pass "both components loaded in $(( $(date +%s) - START ))s: /rtabmap/rgbd_sync, /rtabmap/rtabmap"
else
  tail -40 "$LOG"
  die "components not loaded within ${LOAD_TIMEOUT_S}s"
fi

# --- 4. checks while running ---------------------------------------------------------------------------------------
# The ros2 CLI daemon starts with an empty graph: wait (bounded) until discovery shows the nodes and their inputs.
for _ in $(seq 30); do
  if ros2 node list 2>/dev/null | grep -qx /rtabmap/rtabmap \
     && ros2 topic info /uplink/scan_filtered 2>/dev/null | grep -q "^Subscription count: [1-9]"; then
    break
  fi
  sleep 1
done
nodes=$(ros2 node list 2>/dev/null)
for node in /rtabmap_container /rtabmap/rgbd_sync /rtabmap/rtabmap; do
  echo "$nodes" | grep -qx "$node" && pass "node $node" || fail "node $node missing (nodes: $(echo "$nodes" | tr '\n' ' '))"
done

services=$(ros2 service list 2>/dev/null)
for srv in /rtabmap/backup /rtabmap/reset /rtabmap/pause /rtabmap/resume; do
  echo "$services" | grep -qx "$srv" && pass "service $srv" || fail "service $srv missing"
done

subs() { ros2 topic info "$1" 2>/dev/null | sed -n 's/^Subscription count: //p'; }
pubs() { ros2 topic info "$1" 2>/dev/null | sed -n 's/^Publisher count: //p'; }
for topic in /uplink/head_rgbd/depth/image_raw/compressedDepth /uplink/head_rgbd/rgb/image_raw/compressed \
             /uplink/head_rgbd/rgb/camera_info /uplink/scan_filtered /uplink/tf /uplink/tf_static; do
  n=$(subs "$topic")
  [ "${n:-0}" -ge 1 ] && pass "$topic has $n subscriber(s)" || fail "$topic has no subscriber"
done
for topic in /tf /tf_static; do
  n=$(subs "$topic")
  [ "${n:-0}" -eq 0 ] && pass "$topic has no subscriber (TF is read from /uplink, process-wide remap)" \
    || fail "$topic has $n subscriber(s): the global TF remap did not reach the listener"
done

python3 - <<'PY' && pass "every launch parameter is set on the running nodes" || fail "launch parameters not applied"
import importlib.util
import math
import os
import subprocess
import sys
from pathlib import Path

import yaml

spec = importlib.util.spec_from_file_location("rtabmap_mapping_launch", os.environ["LAUNCH_FILE"])
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
config = launch.load_config(Path(launch.DEFAULT_PARAMS_PATH), Path(os.environ["RTABMAP_MAPPING_CONFIG"]))
container = launch.build_container(launch.load_map3d_topics(), config)
bad = 0
for component in container.components:
    fq = f"/{component.namespace}/{component.name}"
    out = subprocess.run(["ros2", "param", "dump", fq], capture_output=True, text=True, check=True).stdout
    actual = yaml.safe_load(out)[fq]["ros__parameters"]
    for key, want in component.parameters.items():
        if key not in actual:
            print(f"   {fq}: parameter {key} is not declared (unknown name, dropped)")
            bad += 1
            continue
        got = actual[key]
        same = math.isclose(got, want) if isinstance(want, float) else got == want
        if not same:
            print(f"   {fq}: {key} = {got!r}, launch sets {want!r}")
            bad += 1
    print(f"   {fq}: {len(component.parameters)} launch parameters checked")
sys.exit(1 if bad else 0)
PY

for topic in /tf /uplink/tf; do
  out=$(timeout 6 ros2 topic echo --once "$topic" 2>/dev/null || true)
  if echo "$out" | grep -q "transforms:"; then fail "a transform was published on $topic"; else pass "nothing published on $topic"; fi
done
echo "   publishers (idle allowed): /tf $(pubs /tf), /uplink/tf $(pubs /uplink/tf)"

# --- 5. run the full time, stop, inspect the log --------------------------------------------------------------------
remaining=$(( LAUNCH_SECONDS - ($(date +%s) - START) ))
[ "$remaining" -gt 0 ] && sleep "$remaining"
if alive "$LAUNCH_PID"; then
  pass "launch still running after $(( $(date +%s) - START ))s"
else
  fail "launch exited early"
fi
kill -INT -- "-$LAUNCH_PID" 2>/dev/null
for _ in $(seq 15); do alive "$LAUNCH_PID" || break; sleep 1; done
kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true

if grep -qiE "process has died|Failed to load|exception" "$LOG"; then
  grep -iE "process has died|Failed to load|exception" "$LOG" | head -10
  fail "container crashed or a component failed to load"
else
  pass "no crash or load failure in the log"
fi
PARAM_PATTERN="Parameter name changed|has been renamed|doesn't exist anymore|unknown parameter|not declared|InvalidParameterType|is ignored|parameter.*ignor"
if grep -iE "$PARAM_PATTERN" "$LOG" >/tmp/param_warnings.log; then
  cat /tmp/param_warnings.log
  fail "the log reports renamed, unknown or ignored parameters"
else
  pass "no renamed, unknown or ignored parameter in the log"
fi
applied=$(grep -c "Setting RTAB-Map parameter" "$LOG" || true)
echo "   RTAB-Map applied $applied core parameters from ROS"
[ -s "$DB_DIR/live.db" ] && pass "RTAB-Map opened database_path $DB_DIR/live.db ($(stat -c %s "$DB_DIR/live.db") B)" \
  || fail "no live.db in $DB_DIR: database_path not honoured"

if [ "$FAILED" = 0 ]; then
  echo "== ALL CHECKS PASSED"
  exit 0
fi
echo "== launch log tail:"
tail -30 "$LOG"
exit 1
PROBE
status=$?

if [ "$status" -eq 0 ]; then
  echo "rtabmap-launch-probe: PASS"
else
  echo "rtabmap-launch-probe: FAIL"
fi
exit "$status"
