# Roadmap

## Preparation phase

Non functional requirements:
* written in Python3
* Python3 installation managed with `mise`
* Dependencies managed by uv and Poetry
* extensive linting including `pre-commit`, `flake8`, `black`, `isort`, `vulture`, `autoflake`, `coverage`, `pytest` (based on https://github.com/szymonrychu/hikvision-doorbell repository configuration)
* Ansible is able to provision Client and Server raspberry pis running Ubuntu 24.04 (not raspbian)
* Every part of the software developed for the purpose of this project is storedin this reposusitory
* Development is augmented by AI, because of that files like `AGENTS.md`, `MEMORY.md` also exist
* AI builds it's initial rules on it's own, rules must include:
  * developing basic unit tests for new functionality
  * testing code using linters, unit tests
  * preparing commits using semantic commit messages for each features
  * if possible work on multiple branches simultenausly
  * ask developer VERY detailed questions, about everything
  * never hallucinating

Functional requirements:

* there are 2 ros2 master nodes running both inside Server and Client raspberry-pis
* there is a master2master node running inside Client raspberry-pi, which:
  * ensures client's smooth operation even if connection to Server raspberry-pi is lost
  * allows to proxy only specific topics in and out of Client raspberrypi
  * allows to rename topics during proxying
  * is scallable, each topic uses it's own separate thread (or even process if needed)
* all hardware interfaces "bridges" running on RaspberryPIs implenented- meaning:
  * most important hardware capabilities are exposed in form of standard ROS2 topics
  * bridges are written in reusable way for repeating components
* each of the ROS2 nodes (including masters, "bridge nodes", master2master node) are running in separate docker containers
* it's possible to start each of the nodes independently
* there is an Ansible code prepared for:
  * deploying each node as separate systemd service (then enabling and running)
  * managing node's configuration
  * capable of restarting node if configuration, or it's container changes

Optional node (Preparation phase):

* **Lerobot teleoperation node** (Client Raspberry Pi): reads topics from the Server's Lerobot leader arm (via ROS2 / master2master), and sends appropriate ROS2 requests/commands/topic updates to steer the Client's Lerobot follower arm. Enables leader–follower teleop without topic collision (leader and follower use distinct namespaces: `/leader/*`, `/follower/*`).

Note about reusability and what it means: 

Instead of writing separate "bridge" nodes for Lerobot SO-101 leader, follower and separate node for running swerve platform operations, there is single generic "bridge" exposing joints in non-colliding manner.
The same is true for USB Cameras, or GPS modules.

