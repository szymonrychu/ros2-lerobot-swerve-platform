# Feetech servos bridge

**Purpose:** Configurable bridge for Feetech servo arms. Publishes `sensor_msgs/JointState` as `/<namespace>/joint_states` and subscribes to `/<namespace>/joint_commands`. Namespace and joint names come from a config file so the same node can run as leader or follower (or any other role).

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Configuration

- **Config file:** YAML with `namespace` (string) and `joint_names` (list of entries). Each entry must have `name` (joint name for ROS) and `id` (Feetech servo ID, 0–253). Servo IDs need not start from 1 or be sequential. Optional: `device`, `baudrate` for serial; `log_joint_updates` (bool, default false) to print one line per joint_commands update with changing joints as `joint1:val1,joint2:val2,...` to stdout (silent when false); `enable_torque_on_start` (bool, default false); `disable_torque_on_start` (bool, default false); `interpolation_enabled` (bool, default true) and `command_smoothing_time_s` (float, default `0.12`) for smooth follower command transitions with ease-in/ease-out interpolation; `interpolation_target_update_hz` (float, default `40.0`) to pace retargeting when source is steady; `moving_target_update_hz` (float, default `120.0`) to retarget faster while source is moving; `command_deadband_steps` (int, default `3`) to ignore tiny command jitter when source is steady; `moving_command_deadband_steps` (int, default `0`) for moving-source deadband; `source_motion_velocity_threshold_steps_s` (float, default `10.0`) to classify source as moving; `kalman_enabled` (bool, default `true`) for constant-velocity Kalman command tracking; `kalman_process_noise_pos`, `kalman_process_noise_vel`, `kalman_measurement_noise`, `kalman_prediction_lead_s` tune filter responsiveness and prediction lead; `kalman_velocity_decay_per_s` damps extrapolated velocity between command updates; `kalman_max_prediction_time_s` limits how long the bridge predicts without fresh commands (prevents drift); `target_lowpass_alpha` (float in `[0,1]`, default `0.2`) remains available for non-Kalman path; `max_goal_step_rate` (steps/s, default `400.0`, <=0 disables) to limit per-second goal jumps; `control_loop_hz` (float, default `100.0`) for bridge update frequency.
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

- With `namespace: leader`: `/leader/joint_states` (pub), `/leader/joint_commands` (sub), `/leader/servo_registers` (pub, JSON), `/leader/set_register` (sub, JSON).
- With `namespace: follower`: `/follower/joint_states` (pub), `/follower/joint_commands` (sub), `/follower/servo_registers` (pub), `/follower/set_register` (sub).

When `device` is set, the bridge connects to hardware and:

- **Startup:** Reads all Feetech STS registers for each configured joint and prints one compact JSON line per servo to stdout (for debugging): `{"servo_id":1,"joint_name":"shoulder_pan","registers":{...}}`.
- **Publish:** `joint_states` from present position/speed; `servo_registers` (std_msgs/String) as JSON `{ "<joint_name>": { "<register_name>": <value>, ... }, ... }` at ~1 Hz.
- **Subscribe:** `joint_commands` (JointState) writes goal_position (RAM) per joint; `set_register` (std_msgs/String) expects JSON `{"joint_name":"<name>","register":"<name>","value":<int>}` to write any writable register. EPROM registers are written with unlock → write → lock; writes are skipped when the value is unchanged.

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
