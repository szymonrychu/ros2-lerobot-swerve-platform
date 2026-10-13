#!/usr/bin/env bash
# Build and test one in-repo C++ (ament_cmake) package in a ros:jazzy-ros-base container, the same way on the Mac
# (native arm64, the Pi's architecture) and in CI (amd64):
#   1. build or reuse the local image from scripts/cpp-node-test.Dockerfile (colcon plus the rosdep keys of every
#      in-repo C++ package; tagged by a hash of the Dockerfile and all package.xml files);
#   2. mount the repo read-only and copy shared/ros2_common_cpp, nodes/*_cpp and .clang-format into /ws/src,
#      keeping the repo layout (packages find the root .clang-format at ../../.clang-format);
#   3. rosdep install, colcon build --packages-up-to <pkg> (BUILD_TESTING=ON, Release);
#   4. colcon test --packages-select <pkg> (--lint-only: only the ament linters, ctest label "linter"), then
#      colcon test-result --verbose, which must report at least one test and no failures.
# Exits non-zero on any build, test or lint failure and always ends with one line: "cpp-node-test: <pkg> PASS|FAIL".
# An unknown package exits 2 with the list of known ones. The container pattern (docker run --rm -i, repo mounted
# :ro, an inline script on stdin) is the one of scripts/probe-mppi-overlay-docker.sh, generalized to any package.
# Usage: scripts/cpp-node-test.sh <pkg> [--lint-only]
set -euo pipefail
cd "$(dirname "$0")/.."

USAGE="Usage: scripts/cpp-node-test.sh <pkg> [--lint-only]"
DOCKERFILE="scripts/cpp-node-test.Dockerfile"
IMAGE_REPO="cpp-node-test"
EXIT_USAGE=2
PKG="${1:-}"
LINT_ONLY=0

usage() {
  echo "$USAGE" >&2
  exit "$EXIT_USAGE"
}

fail() {
  echo "$*" >&2
  echo "cpp-node-test: ${PKG} FAIL"
  exit 1
}

# Repo-relative directories of every in-repo C++ package; the directory name is the package name.
cpp_package_dirs() {
  local dir
  for dir in shared/ros2_common_cpp nodes/*_cpp; do
    if [ -f "$dir/package.xml" ]; then echo "$dir"; fi
  done
}

sha256() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum; else shasum -a 256; fi
}

[ -n "$PKG" ] || usage
case "${2:-}" in
  "") ;;
  --lint-only) LINT_ONLY=1 ;;
  *) usage ;;
esac
[ "$#" -le 2 ] || usage

PKG_DIRS=$(cpp_package_dirs)
known=0
for dir in $PKG_DIRS; do
  if [ "$(basename "$dir")" = "$PKG" ]; then known=1; fi
done
if [ "$known" = 0 ]; then
  echo "unknown C++ package '${PKG}'; known packages:" >&2
  for dir in $PKG_DIRS; do echo "  $(basename "$dir")" >&2; done
  echo "cpp-node-test: ${PKG} FAIL"
  exit "$EXIT_USAGE"
fi

docker info >/dev/null 2>&1 || fail "docker is not available (is the daemon running?)"

# --- 1. image: reuse when the Dockerfile and every package.xml are unchanged ----------------------------------------
CONTEXT=$(mktemp -d)
trap 'rm -rf "$CONTEXT"' EXIT
cp "$DOCKERFILE" "$CONTEXT/Dockerfile"
for dir in $PKG_DIRS; do
  mkdir -p "$CONTEXT/pkgs/$dir"
  cp "$dir/package.xml" "$CONTEXT/pkgs/$dir/package.xml"
done
TAG=$(cd "$CONTEXT" && find . -type f | LC_ALL=C sort | xargs cat | sha256 | cut -c1-16)
IMAGE="${IMAGE_REPO}:${TAG}"
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "== image $IMAGE (reused)"
else
  echo "== building image $IMAGE from $DOCKERFILE (native $(uname -m))"
  docker build -t "$IMAGE" "$CONTEXT" >"$CONTEXT/build.log" 2>&1 || {
    tail -40 "$CONTEXT/build.log" >&2
    fail "docker build of $IMAGE failed"
  }
fi

# --- 2-4. workspace, build, test ------------------------------------------------------------------------------------
if docker run --rm -i -v "$PWD:/repo:ro" -e PKG="$PKG" -e LINT_ONLY="$LINT_ONLY" -e PKG_DIRS="$PKG_DIRS" \
  -e AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 "$IMAGE" bash -s <<'BUILD'; then
set -eo pipefail
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
mkdir -p /ws/src
cp /repo/.clang-format /ws/src/.clang-format
for dir in $PKG_DIRS; do
  mkdir -p "/ws/src/$(dirname "$dir")"
  cp -r "/repo/$dir" "/ws/src/$dir"
  rm -rf "/ws/src/$dir/build" "/ws/src/$dir/install" "/ws/src/$dir/log"
done
cd /ws
echo "== rosdep install"
rosdep install -y -q --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"
echo "== colcon build --packages-up-to $PKG"
colcon build --packages-up-to "$PKG" --event-handlers console_cohesion+ \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
CTEST_ARGS=()
if [ "$LINT_ONLY" = 1 ]; then CTEST_ARGS=(--ctest-args -L linter); fi
echo "== colcon test --packages-select $PKG ${CTEST_ARGS[*]}"
colcon test --packages-select "$PKG" --event-handlers console_direct+ "${CTEST_ARGS[@]}"
echo "== colcon test-result --verbose"
colcon test-result --verbose --test-result-base "build/$PKG" | tee /tmp/test-result.txt
summary=$(grep -E "^Summary: [0-9]+ tests" /tmp/test-result.txt || true)
[ -n "$summary" ] && ! echo "$summary" | grep -q "^Summary: 0 tests" || { echo "no test results for $PKG"; exit 1; }
BUILD
  echo "cpp-node-test: ${PKG} PASS"
else
  echo "cpp-node-test: ${PKG} FAIL"
  exit 1
fi
