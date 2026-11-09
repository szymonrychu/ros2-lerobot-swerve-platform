# arm-collision

The rclpy-free arm collision code shared by `nodes/mcp_server` (checks arm motions and grasp plans) and
`nodes/filter_node` (the self-collision guard on every follower command). Import name `arm_collision`; deps numpy,
pydantic, ikpy and the C++ kernels (`nodes/mcp_server/mcp_core_cpp`, path dependency). It imports no `rclpy`, `cv2`,
`mcp_server` or `ros2_common` (`tests/test_import_closure.py`).

## Modules

| Module | Contents |
|---|---|
| `settings.py` | pydantic models: `ArmModelSettings` (URDF path, arm base height and mount, joint offsets, tool offset, joint limit overrides, jaw axis, gripper), `BodySettings` (boxes and cylinders), `BodyCheckSettings` (`hard_margin_m`, `soft_margin_m`, `slow_speed_scale`), `load_robot_body()`, `footprint_body()` |
| `robot_body.yaml` | The one robot body definition (see below) |
| `ik.py` | `ArmKinematics`: URDF limits, ikpy FK/IK with the calibration applied in one place |
| `arm_geometry.py`, `jaw.py`, `jaw_profile.py` | Checked arm parts (capsules, box points, gripper hull) and the jaw model |
| `body_model.py` | Exact signed distances from points and capsules to the body's boxes and vertical cylinders |
| `head.py` | `HeadPose` and `HeadModel`: the D435 head posed from pan/tilt out of `nodes/web_ui/urdf/head.urdf` |
| `self_collision.py` | `SelfCollisionChecker`: clearance of every arm part along a joint path, `posed(head)` and `path(samples, allow_escape)` |

## One body, `robot_body.yaml`

`arm_collision/robot_body.yaml` is the single source of the body the arm must not hit (base_link metres). mcp_server,
filter_node and the head_rgbd_calibration pose check all load it (`settings.load_robot_body()`); there is no copy in
`ansible/group_vars/client.yml`. Parts:

- Two side frames, the crossbar and the lidar column, as in [mcp_server Body collision](../../nodes/mcp_server/README.md#body-collision).
- Wheels: four steering-envelope cylinders, z 0 .. 0.15 (3 cm above the 120 mm tyre top, so the arm keeps off the tyres).
- Head: the post (12 mm radius column to 0.30 m), its horizontal arm at z 0.30, and the head envelope (a cylinder about
  the pan axis, z 0.287 .. 0.396, holding the pan base, pan and head links and the D435 at every pan/tilt inside the
  URDF limits). The post and the arm are fixed parts of the body.
- The D435 head is posed from `/head/joint_states` (`head.py`): with a fresh pan/tilt the envelope is replaced by the
  head's actual parts; with no state or one older than 0.5 s the full envelope is checked (fail closed).
- `footprint_body()` is the floor-level body without the head, for the Nav2 footprint.

One keep-away applies to every part: `BodyCheckSettings.hard_margin_m` and `soft_margin_m` default to 0.005 m.

Edit the YAML, never a second copy. The 5 mm margin, the wheel height and the head parts are pinned by
`tests/test_robot_body.py`, `tests/test_body_poses.py` and `nodes/mcp_server/tests/test_head_envelope_body.py`.

## Runtime calibration swap

`ArmKinematics.set_offsets(offsets)` and `SelfCollisionChecker.set_mount(mount)` replace the joint zero offsets and the
arm mount in place, so every holder of the objects sees the new calibration (mcp_server applies an accepted arm to
camera calibration this way at an idle point). `set_offsets` keeps the `offsets` dict object and clears the IK caches;
`set_mount` rebuilds the C++ body kernel before changing any attribute and forgets the posed checker; `posed()` reuses a
cached posed checker only while it stands on the current mount.

## Tests

| File | Covers |
|---|---|
| `tests/test_import_closure.py` | `import arm_collision.self_collision` loads no rclpy, cv2, mcp_server or ros2_common |
| `tests/test_robot_body.py` | The packaged YAML loads to the deployed boxes and cylinders; the footprint body is the floor-level body without the head |
| `tests/test_body_poses.py` | Gripper 2 cm above a wheel top is rejected and 4 cm clears; touching the head post is rejected; parked home and drive pose clear the margin; a measured head pose replaces the envelope; the posed checker is reused |
| `tests/test_head_pose.py` | Head parts at pan/tilt from the URDF, inside the envelope heights, `posed_body` keeps every other obstacle, `head_pose` needs both finite angles |
| `tests/test_runtime_calibration.py` | Runtime calibration swap: `ArmKinematics.set_offsets` gives the FK and IK of kinematics built with the new offsets, keeps the offsets dict object, defaults missing joints to 0, rejects a non-chain joint without changing anything, clears the IK caches; `SelfCollisionChecker.set_mount` gives the clearances of a checker built on the new mount and forgets the posed checker; a posed checker of the old mount stored late by a concurrent `posed()` is not reused |

## Using it from a node

```toml
[project]
dependencies = ["arm-collision"]

[tool.uv.sources]
arm-collision = { path = "../../shared/arm_collision", editable = true }
```

Then `uv lock`; set `src_shared: true` on the node type so a change under `shared/` redeploys it. The venv builds
`mcp_core_cpp`, so the node type lists `src_extra_paths: [nodes/mcp_server/mcp_core_cpp, nodes/web_ui/urdf]` (the URDFs
are read from the repo checkout).

Run: `cd shared/arm_collision && uv sync && uv run pytest tests/ -q`; lint `uv run poe lint`.
