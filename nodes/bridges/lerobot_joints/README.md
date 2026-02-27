# Lerobot joints bridge

**Purpose:** Single generic bridge for Lerobot SO-101 arms (leader and follower). Publishes `sensor_msgs/JointState` as `/<namespace>/joint_states` and subscribes to `/<namespace>/joint_commands`. Namespace is required and avoids topic collision (e.g. `leader` on Server, `follower` on Client).

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Requirements

- **Environment:** `LEROBOT_NAMESPACE` must be set (e.g. `leader` or `follower`). Exits with error if unset.
- **Device:** Optional `LEROBOT_DEVICE` for future serial; stub ignores it.
- Standard SO-101 joint names: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.

## Topic layout (no collision)

- Leader (Server): `/leader/joint_states` (pub), `/leader/joint_commands` (sub).
- Follower (Client): `/follower/joint_states` (pub), `/follower/joint_commands` (sub).

## Build and run

Node source lives under `nodes/`; compose is in `client/` or `server/` with build context `../nodes/bridges/lerobot_joints`.

```bash
# Client follower: cd client && docker compose build lerobot_follower && docker compose --profile lerobot up -d lerobot_follower
# Server leader:   cd server && docker compose build lerobot-leader   && docker compose --profile lerobot-leader up -d lerobot-leader
```

After editing source under `nodes/bridges/lerobot_joints/`, rebuild the image(s) that use it (see [AGENTS.md](../../../AGENTS.md)).
