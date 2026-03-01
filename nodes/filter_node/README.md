# Filter node

**Purpose:** Modular joint command filter for follower teleop. Subscribes to an input `sensor_msgs/JointState` topic, runs a configurable algorithm (e.g. Kalman), and publishes filtered commands to an output topic. Lives on the client; input is typically from master2master or the test joint API node.

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Configuration

- **Config file:** YAML with `input_topic` (default `/filter/input_joint_updates`), `output_topic` (default `/follower/joint_commands`), `algorithm` (default `kalman`), `control_loop_hz` (default `100.0`), optional `joint_names` (order for output), and `algorithm_params` (algorithm-specific keys). For Kalman: `process_noise_pos`, `process_noise_vel`, `measurement_noise`, `prediction_lead_s`, `velocity_decay_per_s`, `max_prediction_time_s`.
- **Config path:** Set `FILTER_NODE_CONFIG` or deploy to `/etc/ros2/filter_node/config.yaml`.

## Topic flow

- **Input:** `sensor_msgs/JointState` on `input_topic`. Each message updates per-joint filter state.
- **Output:** `sensor_msgs/JointState` on `output_topic` at `control_loop_hz`, with filtered positions (and empty velocity/effort).

## Build and run

From repo root, Ansible deploys the node on the client; build context is `nodes/filter_node`. Run the client deploy playbook to build and start the service.
