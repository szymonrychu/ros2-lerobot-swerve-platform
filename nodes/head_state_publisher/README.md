# head_state_publisher

`robot_state_publisher` (node name `head_state_publisher`) on `nodes/web_ui/urdf/head.urdf`. It makes the TF of the
pan/tilt head follow the real servo angles.

## What it publishes

- Fixed joints (mount, rigid head, camera and ToF optical frames) on `/tf_static` once at start.
- `head_pan` and `head_tilt` transforms on `/tf` only from real joint states on `/head/joint_states` (published by the
  `head` group of `base_servos`). With no joint state yet there is no moving transform and nothing is invented.
- The URDF on `/head/robot_description`.

Frames: `base_link` -> `head_post` -> `head_pan_base` -> `head_pan_link` -> `head_link` -> `head_camera_link` ->
`head_camera_optical_frame`, and `head_link` -> `tof_link` -> `tof_optical_frame`.

## Deploy

Ansible node type `head_state_publisher` (native launch file, apt `ros-jazzy-robot-state-publisher`), client host:
`./scripts/deploy-nodes.sh client head_state_publisher`. The launch file reads `head.urdf` relative to itself, so a URDF
change restarts the node.

## Check

```bash
ros2 run tf2_ros tf2_echo base_link head_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link tof_optical_frame
```

Test: `tests/test_head_state_publisher_config.py`.
