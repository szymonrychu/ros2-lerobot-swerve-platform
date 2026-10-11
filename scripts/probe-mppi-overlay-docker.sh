#!/usr/bin/env bash
# Prove, in a ros:jazzy-ros-base arm64 container (Ubuntu 24.04, the RPi OS), that the nav2_mppi_controller overlay
# patch (nodes/nav2_bringup/patches/nav2_mppi_controller-0001-model-delay-pr6154.patch, Nav2 PR #6154 ported to
# Jazzy's xtensor MPPI) still applies to the pinned navigation2 source, builds, and passes both its own model-delay
# tests and the unchanged Jazzy MPPI tests:
#   1. clone navigation2 at PIN (tag 1.3.13 = the apt ros-jazzy-nav2-mppi-controller version);
#   2. git apply --check, then git apply the patch;
#   3. rosdep install the deps of nav2_mppi_controller;
#   4. colcon build --packages-select nav2_mppi_controller with -DBUILD_TESTING=ON;
#   5. colcon test the motion model, optimizer unit and optimizer smoke tests, then colcon test-result --verbose.
# Prints PASS/FAIL per step and exits non-zero on any failure. The first run downloads the image and the build deps.
# PIN must equal the commit of nav2_mppi_overlay_source in ansible/group_vars/client.yml; to bump it, change both,
# rerun this probe, then deploy (see nodes/nav2_bringup/README.md, "MPPI overlay (model_delay)").
# Usage: from repo root, run: ./scripts/probe-mppi-overlay-docker.sh
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="ros:jazzy-ros-base"
REPO_URL="https://github.com/ros-navigation/navigation2.git"
PIN="f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501"
PATCH="nodes/nav2_bringup/patches/nav2_mppi_controller-0001-model-delay-pr6154.patch"
TESTS="motion_model_tests|optimizer_unit_tests|optimizer_smoke_test"
JOBS="${PROBE_JOBS:-4}"

[ -f "$PATCH" ] || { echo "FAIL: $PATCH not found"; exit 1; }

docker run --rm -i --platform linux/arm64 -v "$PWD:/work:ro" \
  -e PIN="$PIN" -e REPO_URL="$REPO_URL" -e PATCH="/work/$PATCH" -e TESTS="$TESTS" -e JOBS="$JOBS" \
  "$IMAGE" bash -s <<'PROBE'
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "PASS: $*"; }

echo "== host: $(uname -m), $(. /etc/os-release && echo "$PRETTY_NAME"), ROS_DISTRO=$ROS_DISTRO"
apt-get update -qq >/dev/null
apt-get install -y -qq git ca-certificates python3-colcon-common-extensions >/dev/null

# --- 1. pinned source --------------------------------------------------------------------------------------------
SRC=/ws/src/navigation2
mkdir -p "$SRC"
git -C "$SRC" init -q
git -C "$SRC" remote add origin "$REPO_URL"
git -C "$SRC" fetch -q --depth 1 origin "$PIN" || fail "fetch $PIN from $REPO_URL"
git -C "$SRC" checkout -q FETCH_HEAD
[ "$(git -C "$SRC" rev-parse HEAD)" = "$PIN" ] && pass "navigation2 checked out at $PIN" || fail "checkout is not $PIN"

# --- 2. patch ----------------------------------------------------------------------------------------------------
git -C "$SRC" apply --check "$PATCH" && pass "git apply --check: patch applies cleanly" || fail "patch does not apply to $PIN"
git -C "$SRC" apply "$PATCH" || fail "git apply"
outside=$(git -C "$SRC" status --porcelain | awk '{print $2}' | grep -v '^nav2_mppi_controller/' || true)
[ -z "$outside" ] && pass "patch touches only nav2_mppi_controller/: $(git -C "$SRC" status --porcelain | awk '{print $2}' | tr '\n' ' ')" \
  || fail "patch touches files outside nav2_mppi_controller/: $outside"

# --- 3. deps -----------------------------------------------------------------------------------------------------
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
rosdep update -q >/dev/null 2>&1 || fail "rosdep update"
rosdep install -y -q --from-paths "$SRC/nav2_mppi_controller" --ignore-src --rosdistro "$ROS_DISTRO" >/dev/null \
  || fail "rosdep install for nav2_mppi_controller"
pass "rosdep installed the nav2_mppi_controller deps"

# --- 4. build ----------------------------------------------------------------------------------------------------
cd /ws
echo "== colcon build (MAKEFLAGS=-j$JOBS)"
if MAKEFLAGS="-j$JOBS" colcon build --packages-select nav2_mppi_controller --event-handlers console_cohesion+ \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > /tmp/build.log 2>&1; then
  pass "colcon build nav2_mppi_controller (BUILD_TESTING=ON)"
else
  grep -E "error|Error" /tmp/build.log | head -40
  tail -20 /tmp/build.log
  fail "colcon build nav2_mppi_controller"
fi

# --- 5. tests ----------------------------------------------------------------------------------------------------
echo "== colcon test (ctest -R '$TESTS')"
colcon test --packages-select nav2_mppi_controller --event-handlers console_direct+ \
  --ctest-args -R "$TESTS" > /tmp/test.log 2>&1 || true
grep -E "^ *[0-9]+/[0-9]+ Test|tests passed|\[  (PASSED|FAILED) |\[       OK \] MotionModelTests\.Delay|\[  FAILED  \]" /tmp/test.log || true
ran=$(grep -cE "^ *[0-9]+/[0-9]+ Test" /tmp/test.log || true)
[ "$ran" -eq 3 ] || fail "expected 3 ctest targets ($TESTS), ran $ran"
delay_ok=$(grep -cE "\[       OK \] MotionModelTests\.Delay" /tmp/test.log || true)
[ "$delay_ok" -eq 5 ] || fail "expected 5 passing MotionModelTests.Delay* gtests, got $delay_ok"
if colcon test-result --verbose --test-result-base build/nav2_mppi_controller; then
  pass "colcon test-result: $TESTS all green (5 model-delay gtests)"
else
  fail "colcon test-result reports failures"
fi
echo "== ALL CHECKS PASSED"
PROBE
