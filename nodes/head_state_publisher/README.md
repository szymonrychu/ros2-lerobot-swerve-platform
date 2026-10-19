# head_state_publisher

`robot_state_publisher` (node name `head_state_publisher`) on `nodes/web_ui/urdf/head.urdf`. It makes the TF of the
pan/tilt head follow the real servo angles.

## What it publishes

- Fixed joints (mount, rigid head, camera and ToF optical frames) on `/tf_static` once at start.
- `head_pan` and `head_tilt` transforms on `/tf` only from real joint states on `/head/joint_states` (published by the
  `head` group of `base_servos`). With no joint state yet there is no moving transform and nothing is invented.
- The URDF on `/head/robot_description`.

Frames: `base_link` -> `head_pan_base` -> `head_pan_link` -> `head_link` -> `head_camera_link` ->
`head_camera_optical_frame`, and `head_link` -> `tof_link` -> `tof_optical_frame`. `head_post` is a separate child of
`base_link` (the visual post) and is not an ancestor of the camera or ToF frames.

## Deploy

Ansible node type `head_state_publisher` (native launch file, apt `ros-jazzy-robot-state-publisher`), client host:
`./scripts/deploy-nodes.sh client head_state_publisher`. The launch file reads `head.urdf` relative to itself, so a URDF
change restarts the node.

## Applied tof_joint override

At launch, `launch/head_description.py` (standard library only) builds the `robot_description`. If
`/var/lib/ros2/head_rgbd_calibration/applied/tof_joint.json` exists and holds `{"xyz": [x, y, z], "rpy": [r, p, y],
"note": "..."}`, the `tof_joint` origin in `head.urdf` is rewritten with those values (via
`shared/ros2_common/head_rgbd_calibration/urdf_text.py`, loaded by path); every other line stays as in the repo. A missing
file leaves the repo URDF; unreadable JSON or a wrong shape also leaves it and logs a warning naming the file and the
reason. The Calibrate page Apply button writes that file and restarts the node, so the repo URDF is never edited on the
robot. A change to `urdf_text.py` restarts the node on deploy.

## Check

```bash
ros2 run tf2_ros tf2_echo base_link head_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link tof_optical_frame
```

Tests: `tests/test_head_state_publisher_config.py`, `tests/test_head_tof_joint_override.py`.
