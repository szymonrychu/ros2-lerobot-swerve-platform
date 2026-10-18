# Image of scripts/cpp-node-test.sh: ros:jazzy-ros-base (Ubuntu 24.04, the RPi OS) on the native platform - arm64 on
# the Mac (the Pi's architecture), amd64 in CI - with colcon and the rosdep keys of every in-repo C++ package
# preinstalled. The script builds it from a temporary context holding only this file and pkgs/<path>/package.xml of
# each package, and tags it with a hash of those files, so the image is rebuilt exactly when a dependency changes.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# prometheus-cpp (used by ros2_common_cpp metrics) has no upstream rosdep rule; map the key to Ubuntu's apt package.
# libopencv-contrib-dev (OpenCV ximgproc, the guided filter of head_rgbd_cpp; same 4.6 version as the system OpenCV that
# ros-jazzy-cv-bridge uses) likewise has no rosdep key, so it is mapped to the apt package the same way.
RUN printf 'prometheus-cpp:\n  ubuntu: [prometheus-cpp-dev]\nlibopencv-contrib-dev:\n  ubuntu: [libopencv-contrib-dev]\n' \
  > /etc/ros/rosdep/cpp-node-test.yaml \
  && echo "yaml file:///etc/ros/rosdep/cpp-node-test.yaml" > /etc/ros/rosdep/sources.list.d/10-cpp-node-test.list

RUN apt-get update \
  && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
  && rosdep update --rosdistro "$ROS_DISTRO"

COPY pkgs/ /opt/cpp-node-test/pkgs/
RUN apt-get update \
  && rosdep install -y --from-paths /opt/cpp-node-test/pkgs --ignore-src --rosdistro "$ROS_DISTRO"
