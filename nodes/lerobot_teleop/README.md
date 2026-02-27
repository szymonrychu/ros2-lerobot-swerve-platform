# Lerobot teleoperation node

**Purpose:** Runs on the Client Raspberry Pi. Subscribes to leader arm topics (which come from the Server via master2master) and publishes corresponding commands to the Client's Lerobot follower arm. Enables leader–follower teleop.

**Audience:** Mid-level Python dev with ROS2.

## Requirements

- **Leader state on client:** `/leader/joint_states` must be available on the client (proxied from Server by master2master). Run master2master with a rule that proxies `/leader/joint_states` from server to client.
- **Follower bridge:** The follower joints bridge must be running and subscribed to `/follower/joint_commands`.
- No device access; ROS2 only.

## Topics

- **Subscribes:** `/leader/joint_states` (sensor_msgs/JointState) — leader state from Server.
- **Publishes:** `/follower/joint_commands` (sensor_msgs/JointState) — commands to local follower arm.

No topic collision: leader and follower use distinct namespaces.

## Build and run

Ansible deploys by cloning the repo on the node and building the container from `nodes/lerobot_teleop`. Run the deploy playbook for client. Start master2master and leader/follower bridges (and proxy config) so leader state is available. After editing source, re-run the deploy playbook to refresh the repo and rebuild the image (see [AGENTS.md](../../AGENTS.md)).
