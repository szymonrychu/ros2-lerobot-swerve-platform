# Feetech servos bridge

**Purpose:** Configurable bridge for Feetech servo arms. Publishes `sensor_msgs/JointState` as `/<namespace>/joint_states` and subscribes to `/<namespace>/joint_commands`. Namespace and joint names come from a config file so the same node can run as leader or follower (or any other role). Command filtering/smoothing is handled by a separate filter node; this bridge applies incoming joint_commands directly to servo goal_position.

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Configuration

- **Config file:** YAML with `namespace` (string) and `joint_names` (list of entries). Each entry must have `name` (joint name for ROS) and `id` (Feetech servo ID, 0–253). Servo IDs need not start from 1 or be sequential. Optional: `device`, `baudrate` for serial; `log_joint_updates` (bool, default false) to print one line per joint_commands update with changing joints as `joint1:val1,joint2:val2,...` to stdout (silent when false); `enable_torque_on_start` (bool, default false); `disable_torque_on_start` (bool, default false); `control_loop_hz` (float, default `100.0`) for bridge update frequency; `register_publish_interval_s` (float, default `10.0`) interval in seconds for full `servo_registers` JSON dump—use ≥ 10 or 0 (disabled) to avoid blocking the control loop and causing visible stutter (1 Hz is not recommended); `publish_effort_joints` (list of joint names, default empty) to publish `present_load` in `JointState.effort` at control-loop rate for those joints (e.g. for haptic/force-feedback use).
- **Per-joint range mapping (optional):** For each joint you may set `source_min_steps` and `source_max_steps` (0–4095) to define the **incoming** range (e.g. leader’s range). If `source_*` is not configured, the bridge keeps legacy pass-through behavior (`position * 1000` clamped to 0..4095). Set `source_inverted: true` when close/open direction is opposite between leader and follower for that joint. You may also set `command_min_steps` and `command_max_steps` to override the **follower** command range; if omitted, the bridge reads `min_angle_limit` and `max_angle_limit` from the servo at startup. This allows “leader at source_max” to map to “follower at command_max” (or to `command_min` when `source_inverted: true`). Invalid or out-of-range values are ignored (defaults used).
- **Joint mode (optional):** `mode: position` (default; `JointState.position` drives `goal_position`) or `mode: velocity`. Velocity joints run in wheel mode (mode register = 1, set in RAM at every start). `JointState.velocity` in rad/s drives `goal_speed`, clamped to `max_velocity_rad_s` (default 4.71). They are stopped when no command arrives for `velocity_command_timeout_s` (top level, default 0.3 s) and when the bridge exits. An unknown `mode` makes the config invalid.
- **Inversion (optional):** `inverted: true` flips the joint's positive direction for velocities and for positions without `source_*` mapping (e.g. mirrored wheel mounts).
- **Several namespaces on one bus (optional):** `extra_groups: [{namespace, joint_names}]` serves more joints from the same serial port and process, each group with its own `/<namespace>/joint_states` and `/<namespace>/joint_commands`. Servo IDs must be unique across all groups. `servo_registers` / `set_register` stay on the primary namespace (set_register finds joints in any group). Used on the client: the swerve servos (IDs 32-39) share the follower arm bus as group `swerve_drive`.
- **State reads:** present position and speed of all servos are read with one sync read per cycle (per-servo fallback for missing replies). `JointState.velocity` is in rad/s (decoded from the sign-magnitude speed register). If any joint of a group cannot be read, that group's sample is skipped for the cycle, and without a bus nothing is published (no placeholder data). The periodic `servo_registers` dump reads one servo per control cycle, so the loop and the wheel watchdog never stall. `log_joint_updates` skips velocity-mode joints.
- **Config path:** Set `FEETECH_SERVOS_CONFIG` to the config file path, or deploy to `/etc/ros2/feetech_servos/config.yaml`.

Example format:

```yaml
namespace: follower
joint_names:
  - name: shoulder_pan
    id: 1
  - name: gripper
    id: 6
    # Optional: map leader gripper range to follower range so leader "fully closed" -> follower "fully closed"
    # source_min_steps: 1951   # leader open (steps)
    # source_max_steps: 3000   # leader closed (steps) — set to value leader publishes when closed
    # source_inverted: true    # use true when leader/follower close direction is opposite
    # command_min_steps: 1951  # follower open (or omit to read from servo)
    # command_max_steps: 3377  # follower closed (or omit to read from servo)
```

Example configs are in `config/leader.yaml` and `config/follower.yaml`. Mount one as the config (e.g. in compose) or copy to the default path.

## Topic layout

- With `namespace: leader`: `/leader/joint_states` (pub), `/leader/joint_commands` (sub), `/leader/servo_registers` (pub, JSON), `/leader/set_register` (sub, JSON).
- With `namespace: follower`: `/follower/joint_states` (pub), `/follower/joint_commands` (sub), `/follower/servo_registers` (pub), `/follower/set_register` (sub).

When `device` is set, the bridge connects to hardware and:

- **Startup:** Reads all Feetech STS registers for each configured joint and prints one compact JSON line per servo to stdout (for debugging): `{"servo_id":1,"joint_name":"shoulder_pan","registers":{...}}`.
- **Publish:** `joint_states` from present position/speed; `servo_registers` (std_msgs/String) as JSON `{ "<joint_name>": { "<register_name>": <value>, ... }, ... }` at ~1 Hz.
- **Subscribe:** `joint_commands` (JointState) writes goal_position (position joints) or goal_speed (velocity joints) per joint, see [Command formats](#command-formats-joint_commands); `set_register` (std_msgs/String) expects JSON `{"joint_name":"<name>","register":"<name>","value":<int>}`. **At runtime only RAM registers are accepted** (e.g. `torque_enable`, `goal_position`, `acceleration`, `goal_speed`, `goal_time`). EPROM registers (PID, current limits, angle limits, etc.) are **rejected** from ROS; set them once via `calibrate_servos.py load-config` on the host to avoid constant EEPROM wear.

### Command formats (`joint_commands`)

All of these are accepted on any group's `joint_commands`, matched by joint name:

- **Separate messages** (what `swerve_drive_controller` publishes today): a steer-only `JointState` (names + `position`) and a drive-only one (names + `velocity`, rad/s). Arm commands are position-only (`velocity` empty).
- **Combined message:** one `JointState` naming all joints, with `position[i] = NaN` for velocity-mode joints and `velocity[i] = NaN` for position-mode joints.
- **NaN / inf entries are ignored** and never written to a servo (no goal_position / goal_speed write, and a NaN velocity does not feed the wheel watchdog). `command_mapping` additionally refuses non-finite values with `ValueError`; before this check a NaN velocity clamped to full speed.

### Control loop, callback draining and the wheel watchdog

Each loop iteration (`control_loop_hz`): drain **all** pending ROS callbacks (`executor.spin_once(timeout_sec=0)` repeated until no callback ran, at most `MAX_CALLBACKS_PER_CYCLE` = 64), run the velocity watchdog, sync-read and publish state, then sleep the remainder of the period. The per-cycle logic (command handling, NaN filtering, watchdog, write decisions) is in `bridge_cycle.py` (`BridgeCycle`, `drain_callbacks`) and has no rclpy dependency, so `tests/test_bridge_cycle.py` drives it with a fake servo and a simulated KEEP_LAST(10) queue.

The watchdog stops a wheel (goal_speed = 0) only when it is moving and **no drive command for that wheel** was received within `velocity_command_timeout_s`. A received command feeds the watchdog even if its register write fails (the record keeps the last velocity actually written, so a still-moving wheel is stopped once commands stop); a failed watchdog stop is retried every cycle.

**Root cause of the periodic wheel stops (fixed):** the loop used to process only one callback per iteration (`spin_once` at the end of the loop, ~85 Hz). `swerve_drive_controller` publishes a steer message and a drive message back to back (~215 msg/s), so the subscription queue (depth 10) was always full, and after each new pair arrived its oldest entry - the one processed next - was always a steer message. Drive commands were only handled when loop jitter let one through; when none got through for 0.3 s the watchdog wrote goal_speed = 0 to all four wheels, which then stood still for 0.4-0.8 s until the next drive message slipped through.

## Build and run

Node source lives under `nodes/bridges/feetech_servos`. Ansible deploys by cloning the repo on the node and installing the Poetry venv from this path; run the deploy playbook for client or server to install and start the service.

```bash
./scripts/deploy-nodes.sh client lerobot_follower
./scripts/deploy-nodes.sh server lerobot_leader
```

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

CLI with subcommands for calibration, register read/write, and min/max limit management. All single-servo commands accept `--device` (required), `--baudrate` (default 1000000), and `--id` (default 1) for the servo ID.

**Subcommands:**

- **`calibrate`** — Interactive calibration for arm joints (e.g. 6-DOF). Disables torque, prompts for neutral then min/max range per joint, writes limits to servos, outputs JSON.
  - `--device`, `--baudrate`, `--output` (required), `--expected-joints` (default 6).
  - Output JSON: `{ "<servo_id>": { "min", "center", "max" } }` (steps).
- **`read`** — Read one register by name or all registers. Requires either `--register <name>` or `--all`.
- **`get`** — Same as read: get one or all registers (works for both read-only and read-write registers).
- **`write`** — Write one register: `--register <name>` and `--value <int>`.
- **`set`** — Same as write but for read-write registers only; refuses read-only with a detailed error (use `get` to read them).
- **`limits-set`** — Set min/max angle limits (steps 0..4095, min ≤ max): `--min <steps>` and `--max <steps>`.
- **`limits-get`** — Read min/max angle limits from one servo; prints JSON `{"min": <n>, "max": <n>}`.
- **`limits-clear`** — Clear min/max limits to full range (0, 4095).
- **`center`** — Store the servo's current physical position as centre (step 2048, via the STS `torque_enable = 128` offset calibration), verify it reads ~2048, then write `min/max_angle_limit` = 2048 -/+ `--range-steps` (default 1024 = +-90 deg). Prints `{"id", "before", "after", "min", "max"}`. Used for swerve steering servos aligned straight ahead.
- **`load-config`** — Load an extended JSON config file and apply to servos. Refuses if the file contains any read-only register (exits with a detailed message listing them). Legacy keys `min`/`max` map to `min_angle_limit`/`max_angle_limit`; `center` is ignored.
- **`dump-config`** — Read all registers for one or more servos and output extended JSON (`servo_id -> { register_name: value }`). Optional `--output <path>`; otherwise prints to stdout. Use `--id 1 2 3` to dump multiple servos.
- **`list-registers`** — Print register map (name, address, size, read_only, eprom) as JSON. No device needed.

**File-based config (extended JSON):** The format is `{ "<servo_id>": { "<register_name>": <int>, ... } }`. Use `dump-config` to produce it and `load-config --file <path>` to apply it. Only writable registers may appear in a file you load; read-only registers cause load to fail with an error.

Register names match `registers.py` (e.g. `present_position`, `goal_position`, `min_angle_limit`, `max_angle_limit`, `torque_enable`). Use `list-registers` to see all.

**Examples:**

```bash
# Interactive calibration (multi-joint)
poetry run python scripts/calibrate_servos.py calibrate --device /dev/ttyUSB0 --output cal.json

# Read one register from servo ID 1 (default)
poetry run python scripts/calibrate_servos.py read --device /dev/ttyUSB0 --register present_position

# Read all registers from servo ID 2
poetry run python scripts/calibrate_servos.py read --device /dev/ttyUSB0 --id 2 --all

# Write goal_position on servo 1
poetry run python scripts/calibrate_servos.py write --device /dev/ttyUSB0 --register goal_position --value 2048

# Set min/max limits on servo 1
poetry run python scripts/calibrate_servos.py limits-set --device /dev/ttyUSB0 --id 1 --min 100 --max 4000

# Read current min/max limits (JSON)
poetry run python scripts/calibrate_servos.py limits-get --device /dev/ttyUSB0 --id 1

# Clear limits to full range
poetry run python scripts/calibrate_servos.py limits-clear --device /dev/ttyUSB0 --id 1

# Get/set (get any register; set only read-write)
poetry run python scripts/calibrate_servos.py get --device /dev/ttyUSB0 --register present_position
poetry run python scripts/calibrate_servos.py set --device /dev/ttyUSB0 --register goal_position --value 2048

# File-based config: dump all registers to JSON, then load back (file must not contain read-only registers)
poetry run python scripts/calibrate_servos.py dump-config --device /dev/ttyUSB0 --id 1 2 --output config.json
poetry run python scripts/calibrate_servos.py load-config --device /dev/ttyUSB0 --file config.json

# List register map (no device)
poetry run python scripts/calibrate_servos.py list-registers
```

### Leader gripper tuning for haptics

When using the leader arm with the haptic controller (resistance or zero-G), the leader gripper servos (IDs 5 and 6) benefit from reduced stiffness and capped current so the arm stays backdrivable. Tuning is **PID and current only**; apply once via the profile (EEPROM), not via ROS at runtime.

**Profile file:** `leader_gripper_haptic_profile.json` in this directory. It sets only the gripper servo IDs (5 and 6) and only:

- **PID:** `p_coefficient`, `d_coefficient`, `i_coefficient` (lower P, modest D, I=0 for backdrivability and stability).
- **Current limits:** `protection_current`, `max_torque_limit` (capped so the leader does not lock up).

Deadband, acceleration, goal_speed, and protective_torque are left at servo defaults; add them to the JSON if you need to override.

**Workflow:**

1. On the **server** (leader machine), connect the leader arm and identify the serial device (e.g. `/dev/serial/by-id/usb-...`).
2. Apply the profile (writes to EEPROM; servos 5 and 6 only):
   ```bash
   cd nodes/bridges/feetech_servos
   poetry run python scripts/calibrate_servos.py load-config --device /dev/serial/by-id/usb-... --file leader_gripper_haptic_profile.json
   ```
3. Restart the leader feetech_servos node so it runs with the new register values.
4. Tune if needed: use `dump-config --id 5 6` to read current values, edit the JSON (PID and current only), then `load-config` again. Keep haptic tests **gripper-only**.
