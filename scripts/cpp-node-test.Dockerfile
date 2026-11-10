# Image of scripts/cpp-node-test.sh: ros:jazzy-ros-base (Ubuntu 24.04, the RPi OS) on the native platform - arm64 on
# the Mac (the Pi's architecture), amd64 in CI - with colcon and the rosdep keys of every in-repo C++ package
# preinstalled. The script builds it from a temporary context holding only this file and pkgs/<path>/package.xml of
# each package, and tags it with a hash of those files, so the image is rebuilt exactly when a dependency changes.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Ubuntu apt mirrors, rewritten in the deb822 sources of the noble base image before the first apt-get. The defaults
# are the base image's own URIs (local and Mac builds; on arm64 the image uses ports.ubuntu.com, which these rewrites
# leave untouched). On GitHub Actions scripts/cpp-node-test.sh passes http://azure.archive.ubuntu.com/ubuntu for both,
# the mirror the runner VMs themselves use (actions/runner-images configure-apt-sources.sh): from GitHub-hosted runners
# archive.ubuntu.com and security.ubuntu.com (the same Canonical IP pool) crawled at about 9 kB/s and failed builds
# after 44 min (CI run 42757550349). On amd64 the build fails if the rewritten URIs are missing, so a changed base
# image layout cannot silently fall back to the slow archive.
ARG UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu
ARG UBUNTU_SECURITY_MIRROR=http://security.ubuntu.com/ubuntu
RUN sed -i \
    -e "s|^URIs: http://archive\.ubuntu\.com/ubuntu/\?$|URIs: ${UBUNTU_MIRROR}/|" \
    -e "s|^URIs: http://security\.ubuntu\.com/ubuntu/\?$|URIs: ${UBUNTU_SECURITY_MIRROR}/|" \
    /etc/apt/sources.list.d/ubuntu.sources \
  && if [ "$(dpkg --print-architecture)" = amd64 ]; then \
    grep -qxF "URIs: ${UBUNTU_MIRROR}/" /etc/apt/sources.list.d/ubuntu.sources \
    && grep -qxF "URIs: ${UBUNTU_SECURITY_MIRROR}/" /etc/apt/sources.list.d/ubuntu.sources; \
  fi

# One dropped or stalled connection retries instead of failing the build (apt's default is 3 retries); also applies
# to the rosdep install of every cpp-node-test.sh run in this image.
RUN printf '%s\n' 'Acquire::Retries "5";' 'Acquire::http::Timeout "30";' 'Acquire::https::Timeout "30";' \
  > /etc/apt/apt.conf.d/80-cpp-node-test-retries

# prometheus-cpp (used by ros2_common_cpp metrics) has no upstream rosdep rule; map the key to Ubuntu's apt package.
RUN printf 'prometheus-cpp:\n  ubuntu: [prometheus-cpp-dev]\n' \
  > /etc/ros/rosdep/cpp-node-test.yaml \
  && echo "yaml file:///etc/ros/rosdep/cpp-node-test.yaml" > /etc/ros/rosdep/sources.list.d/10-cpp-node-test.list

RUN apt-get update \
  && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
  && rosdep update --rosdistro "$ROS_DISTRO"

COPY pkgs/ /opt/cpp-node-test/pkgs/
RUN apt-get update \
  && rosdep install -y --from-paths /opt/cpp-node-test/pkgs --ignore-src --rosdistro "$ROS_DISTRO"
