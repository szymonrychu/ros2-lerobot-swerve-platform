# Feetech servos bridge

**Purpose:** Configurable bridge for Feetech servo arms. Publishes `sensor_msgs/JointState` as `/<namespace>/joint_states` and subscribes to `/<namespace>/joint_commands`. Namespace and joint names come from a config file so the same node can run as leader or follower (or any other role).

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Configuration

- **Config file:** YAML with `namespace` (string) and `joint_names` (list of entries). Each entry must have `name` (joint name for ROS) and `id` (Feetech servo ID, 0–253). Servo IDs need not start from 1 or be sequential. Optional: `device`, `baudrate` for serial.
- **Config path:** Set `FEETECH_SERVOS_CONFIG` to the config file path, or deploy to `/etc/ros2/feetech_servos/config.yaml`.

Example format:

```yaml
namespace: follower
joint_names:
  - name: shoulder_pan
    id: 1
  - name: gripper
    id: 6
```

Example configs are in `config/leader.yaml` and `config/follower.yaml`. Mount one as the config (e.g. in compose) or copy to the default path.

## Topic layout

- With `namespace: leader`: `/leader/joint_states` (pub), `/leader/joint_commands` (sub).
- With `namespace: follower`: `/follower/joint_states` (pub), `/follower/joint_commands` (sub).

## Build and run

Node source lives under `nodes/bridges/feetech_servos`. Ansible deploys by cloning the repo on the node and building the container from this path; run the deploy playbook for client or server to build and start the service.

Set `FEETECH_SERVOS_CONFIG` to the path of the mounted config (e.g. `/etc/ros2/feetech_servos/config.yaml`) or mount `config/follower.yaml` or `config/leader.yaml` there.

## Utility scripts

Run from the node directory after `poetry install`: `poetry run python scripts/set_servo_id.py ...` or `poetry run python scripts/calibrate_servos.py ...`.

### set_servo_id.py

Set a new ID on a single Feetech servo. Requires exactly one servo on the bus (exits with error if 0 or 2+).

- `--device` (required): Serial device path (e.g. `/dev/ttyUSB0`).
- `--baudrate` (default 1000000): Baudrate; the st3215 library uses 1Mbps by default.
- `--new-id` (required): New servo ID (0–253).

Example: `poetry run python scripts/set_servo_id.py --device /dev/ttyUSB0 --new-id 3`

### calibrate_servos.py

Interactive calibration for arm joints (e.g. 6-DOF arm). Disables torque, prompts for neutral then min/max range per joint, writes limits to servos, and outputs JSON.

- `--device` (required): Serial device path.
- `--baudrate` (default 1000000): Baudrate.
- `--output` (required): Path for output JSON file.
- `--expected-joints` (default 6): Expected number of joints; script exits if fewer are found.

Output JSON shape: `{ "<servo_id>": { "min": <steps>, "center": <steps>, "max": <steps> } }` (keys are string IDs).

Example: `poetry run python scripts/calibrate_servos.py --device /dev/ttyUSB0 --output cal.json`
