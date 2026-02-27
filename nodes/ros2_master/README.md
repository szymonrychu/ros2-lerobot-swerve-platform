# ROS2 master node

**Purpose:** Runs the ROS2 daemon (DDS master). Used by both Server and Client; same image, different compose image names (server-ros2-master, client-ros2-master).

**Location:** All node source lives under [nodes/](.). See [README.md](../../README.md).

## Build

Ansible clones the repo on each node and builds the container from `nodes/ros2_master`. Image name uses the registry `harbor.szymonrichert.pl/containers/<client-ros2-master|server-ros2-master>:latest`.
