# head_state_publisher

`robot_state_publisher` (node name `head_state_publisher`) on `nodes/web_ui/urdf/head.urdf`. It makes the TF of the
pan/tilt head follow the real servo angles.

## What it publishes

- Fixed joints (mount, rigid head and the D435 body, depth and colour frames) on `/tf_static` once at start.
  realsense_d435 runs with `publish_tf: false`, so `head.urdf` owns every `d435_*` frame (design D4).
- `head_pan` and `head_tilt` transforms on `/tf` only from real joint states on `/head/joint_states` (published by the
  `head` group of `base_servos`). With no joint state yet there is no moving transform and nothing is invented.
- The URDF on `/head/robot_description`.

Frames: `base_link` -> `head_pan_base` -> `head_pan_link` -> `head_link` -> `d435_link` (joint `d435_joint`), then
`d435_link` -> `d435_depth_frame` -> `d435_depth_optical_frame` and `d435_link` -> `d435_color_frame` ->
`d435_color_optical_frame` (`d435_color_joint` is the factory colour-to-depth extrinsic of the head D435). The D435
topics carry `d435_depth_optical_frame` and `d435_color_optical_frame` in their headers. `head_post` is a separate child
of `base_link` (the visual post) and is not an ancestor of the D435 frames.

## Deploy

Ansible node type `head_state_publisher` (native launch file, apt `ros-jazzy-robot-state-publisher`), client host:
`./scripts/deploy-nodes.sh client head_state_publisher`. The launch file reads `head.urdf` relative to itself, so a URDF
change restarts the node.

## Applied head overrides

At launch, `launch/head_description.py` (standard library only) builds the `robot_description` from the repo
`head.urdf` plus the files in `/var/lib/ros2/head_rgbd_calibration/applied/`, using `apply_overrides` from
`shared/ros2_common/head_rgbd_calibration/urdf_text.py` (loaded by path):

- `head_chain.json` rewrites the `head_mount_joint`, `head_pan`, `head_tilt` and `d435_joint` origins. It comes from either fit of the head_rgbd_calibration node, whichever was applied last: the floor-plane fit (mount z, roll, pitch, tilt zero and roll) or the head-from-arm fit (the whole mount, tilt zero and roll and, with the pan axis free, the `head_pan` origin x, y as the pan axis position, with the `head_tilt` origin moved back by the same shift). `applied.json` records which one (`head_chain.source` `floor` or `head_arm`); the file format is the same. The head-from-arm fit starts from and keeps the joints of the chain applied before it, so the file always holds the complete chain. The head-from-arm fit also fits the `d435_joint` x, z, roll and yaw (the D435 mount on the head), so its file overrides the D435 mount; the floor fit keeps the applied `d435_joint`.
- A `tof_joint.json` left in `applied/` by the retired head ToF is ignored (head.urdf has no `tof_joint` any more).

Every other line stays as in the repo. A missing `head_chain.json` is skipped; an unreadable or invalid one is skipped
whole and logs a warning naming the file and reason. The apply path unit restarts the node when
`applied.json` changes, so the repo URDF is never edited on the robot. A change to `urdf_text.py` restarts the node on deploy.

## Check

```bash
ros2 run tf2_ros tf2_echo base_link d435_color_optical_frame
ros2 run tf2_ros tf2_echo base_link d435_depth_optical_frame
```

Tests: `tests/test_head_state_publisher_config.py` (also: the D435 frames resolve from head.urdf), `tests/test_head_chain_launch.py`.
