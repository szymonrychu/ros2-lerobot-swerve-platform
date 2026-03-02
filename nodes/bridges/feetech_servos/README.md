# Feetech servos bridge

**Purpose:** Configurable bridge for Feetech servo arms. Publishes `sensor_msgs/JointState` as `/<namespace>/joint_states` and subscribes to `/<namespace>/joint_commands`. Namespace and joint names come from a config file so the same node can run as leader or follower (or any other role). Command filtering/smoothing is handled by a separate filter node; this bridge applies incoming joint_commands directly to servo goal_position.

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Configuration

- **Config file:** YAML with `namespace` (string) and `joint_names` (list of entries). Each entry must have `name` (joint name for ROS) and `id` (Feetech servo ID, 0–253). Servo IDs need not start from 1 or be sequential. Optional: `device`, `baudrate` for serial; `log_joint_updates` (bool, default false) to print one line per joint_commands update with changing joints as `joint1:val1,joint2:val2,...` to stdout (silent when false); `enable_torque_on_start` (bool, default false); `disable_torque_on_start` (bool, default false); `control_loop_hz` (float, default `100.0`) for bridge update frequency; `register_publish_interval_s` (float, default `10.0`) interval in seconds for full `servo_registers` JSON dump—use ≥ 10 or 0 (disabled) to avoid blocking the control loop and causing visible stutter (1 Hz is not recommended); `publish_effort_joints` (list of joint names, default empty) to publish `present_load` in `JointState.effort` at control-loop rate for those joints (e.g. for haptic/force-feedback use).
- **Per-joint range mapping (optional):** For each joint you may set `source_min_steps` and `source_max_steps` (0–4095) to define the **incoming** range (e.g. leader’s range). If `source_*` is not configured, the bridge keeps legacy pass-through behavior (`position * 1000` clamped to 0..4095). Set `source_inverted: true` when close/open direction is opposite between leader and follower for that joint. You may also set `command_min_steps` and `command_max_steps` to override the **follower** command range; if omitted, the bridge reads `min_angle_limit` and `max_angle_limit` from the servo at startup. This allows “leader at source_max” to map to “follower at command_max” (or to `command_min` when `source_inverted: true`). Invalid or out-of-range values are ignored (defaults used).
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
- **Subscribe:** `joint_commands` (JointState) writes goal_position (RAM) per joint; `set_register` (std_msgs/String) expects JSON `{"joint_name":"<name>","register":"<name>","value":<int>}` to write any writable register. EPROM registers are written with unlock -> write -> lock; writes are skipped when the value is unchanged.

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
