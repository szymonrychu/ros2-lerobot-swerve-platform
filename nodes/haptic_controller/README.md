# Haptic controller

Client-side ROS2 node for **force-feedback (active resistance)** and **zero-G hold mode** on the leader gripper. Pilot is gripper-only (joint_5, joint_6). When enabled, leader torque must be on so the node can render resistance or hold pose.

## Modes

- **off** — No haptic output; node does not publish. Normal teleop unchanged.
- **resistance** — Subscribes to leader and follower state; when follower gripper reports load (e.g. contact with object), applies a virtual opposing force by commanding the leader gripper toward a slightly more open position. Uses `present_load` from follower bridge (published in `JointState.effort` when `publish_effort_joints` is set).
- **zero_g** — Holds the current leader gripper pose: continuously publishes the leader’s present position as the command so that when the user releases, the gripper stays in place.

## Data flow

- **Leader state:** `/filter/input_joint_updates` (leader joint_states forwarded by master2master to client). During haptic/teleop evaluation, use only one source for this topic (e.g. master2master from leader); do not publish from the test_joint_api to the same topic at the same time to avoid conflicting inputs.
- **Follower state:** `/follower/joint_states` (must include `effort` for gripper joints; set `publish_effort_joints` on the follower feetech bridge).
- **Leader commands:** Node publishes to `/client/haptic_leader_commands`; master2master relays this to `/leader/joint_commands` on the server (direction **out**).

## Configuration

Config file path: `HAPTIC_CONTROLLER_CONFIG` or `/etc/ros2/haptic_controller/config.yaml`.

| Key | Description | Default |
|-----|-------------|---------|
| `mode` | `off`, `resistance`, or `zero_g` | `off` |
| `gripper_joint_names` | Joint names for haptics (e.g. joint_5, joint_6) | `["joint_5", "joint_6"]` |
| `leader_state_topic` | Topic for leader JointState | `/filter/input_joint_updates` |
| `follower_state_topic` | Topic for follower JointState (with effort) | `/follower/joint_states` |
| `leader_cmd_topic` | Topic to publish leader commands | `/client/haptic_leader_commands` |
| `control_loop_hz` | Loop rate when mode is resistance or zero_g | `100.0` |
| `resistance_gains.max_stiffness` | Virtual stiffness (position delta per unit load); keep low for subtle feedback | `0.0008` |
| `resistance_gains.load_deadband` | Follower load below this = no resistance | `60.0` |
| `resistance_gains.max_step_per_cycle` | Max position step per cycle (safety); lower = lighter resistance | `0.02` |
| `resistance_gains.activation_velocity_threshold` | Min closing velocity to apply resistance | `0.01` |
| `resistance_gains.release_delay_s` | Delay after last resistance before torque off | `0.15` |
| `resistance_gains.load_release_ratio` | Hysteresis: release when load < deadband × this | `0.6` |
| `resistance_gains.activation_debounce_cycles` | Consecutive cycles active before enabling torque | `2` |
| `delay_safety_max_skew_s` | Max leader/follower reception time skew; above this, resistance drops to passive | `0.4` |
| `watchdog_timeout_s` | No publish if leader/follower state older than this | `0.5` |

## Deployment

Deployed on the **client** only. Default Ansible config has the node **present** but **enabled: false**. Enable the service and set `mode: resistance` or `mode: zero_g` in config when you want haptics. Ensure the follower feetech bridge has `publish_effort_joints: [joint_5, joint_6]` so follower load is available. For resistance or zero-G, leader torque must be enabled on the server (e.g. manually or via a separate mechanism); the node only publishes position commands. **Feedback is intentionally subtle** (low stiffness, small step per cycle)—you should feel some resistance on contact, not strong locking.

## Tests

From `nodes/haptic_controller`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers config loading (`test_config.py`) and resistance control law `compute_resistance_target` (`test_node.py`), with no ROS2 dependency.
