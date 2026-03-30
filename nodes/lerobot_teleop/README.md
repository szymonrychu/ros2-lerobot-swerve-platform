# Lerobot teleoperation node

**Purpose:** Runs on the Client Raspberry Pi. Subscribes to leader arm topics (which come from the Server via master2master) and publishes corresponding commands to the Client's Lerobot follower arm. Enables leader–follower teleop.

**Audience:** Mid-level Python dev with ROS2.

## Requirements

- **Leader state on client:** Leader joint_states topic must be available on the client (proxied from Server by master2master). Default: `/leader/joint_states`.
- **Follower bridge:** The follower joints bridge must be running and subscribed to the follower joint_commands topic. Default: `/follower/joint_commands`.
- **Environment (optional):** `TELEOP_LEADER_JOINT_STATES_TOPIC`, `TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC` override the default topic names.
- No device access; ROS2 only. Config is in `config.py` (no ROS deps) for testability; run tests with `poetry run poe test` from the node directory.

## Topics

- **Subscribes:** `/leader/joint_states` (sensor_msgs/JointState) — leader state from Server.
- **Publishes:** `/follower/joint_commands` (sensor_msgs/JointState) — commands to local follower arm.

No topic collision: leader and follower use distinct namespaces.

## Build and run

Deploy with `scripts/deploy-nodes.sh client lerobot_teleop` from the repo root. Start master2master and leader/follower bridges (and proxy config) so leader state is available. After editing source, re-run the deploy script to refresh the repo and reinstall the service (see [CLAUDE.md](../../CLAUDE.md)).
