# Shared libraries

Python code shared by multiple nodes. `shared/` is one installable uv package, `ros2-common` (import name
`ros2_common`, see [pyproject.toml](pyproject.toml)). Node code is in [nodes/](../nodes/).

See [CLAUDE.md](../CLAUDE.md): use Python 3 type hints and extend unit tests as the project grows.

A second, separate package lives in [ros2_metrics/](ros2_metrics/README.md): `ros2-metrics`, the Prometheus
exporter helper of our nodes (only `prometheus-client`).

## Using it from a node

Add ros2-common as an editable path dependency, relative to the node directory:

```toml
[project]
dependencies = ["ros2-common"]

[tool.uv.sources]
ros2-common = { path = "../../shared", editable = true }
```

Then run `uv lock` once. On the Pi the whole repo is cloned to `ros2_repo_dest`, and the `ros2_node_deploy` role
runs `uv sync --frozen --no-dev` inside `<ros2_repo_dest>/nodes/<node>` into `/opt/ros2-nodes/<name>/venv`, so the
same `../../shared` path resolves there; nothing extra is needed in Ansible. A node one level deeper (e.g.
`nodes/bridges/<node>`) uses `../../../shared`.

## Modules

| Module | Contents |
|---|---|
| `ros2_common._utils` | `clamp(value, low, high)` |
| `ros2_common.battery` | `BatteryConfig` (pydantic: `topic` `/battery_state`, `cells` 3, `cutoff_cell_v` 2.8, `resume_cell_v` 2.9, `stale_s` 5.0; `resume_cell_v >= cutoff_cell_v`, `cells >= 1`) and `BatteryGuard` (thread-safe cut-off hysteresis: enter below `cells * cutoff_cell_v`, leave only above `cells * resume_cell_v`; no reading or older than `stale_s` = unknown = not blocked) |
| `ros2_common.camera_geometry` | Pixel <-> ground-plane maths and mount calibration (see below) |

Used by: `nodes/mcp_server` (motion tools refused in cut-off) and `nodes/web_ui` (commands rejected in cut-off; uv path source `ros2-common = { path = "../../shared", editable = true }`).

## Camera geometry (`ros2_common.camera_geometry`)

Conventions (ROS REP-103):

- `MountPose` is the camera BODY frame (x forward, y left, z up) in `parent_frame`, in meters and radians.
- Orientation is fixed-axis RPY like URDF `rpy` (rotate about fixed x, then y, then z).
- The OPTICAL frame (z forward, x right, y down) is the body frame times the standard camera_link -> optical
  rotation (roll -pi/2, yaw -pi/2). `optical_from_mount(T_parent_mount)` applies it, so
  `T_frame_optical = optical_from_mount(T_frame_parent @ T_parent_mount)`.
- Pixels are (u right, v down). `T_a_b` maps points of frame b into frame a.

API: models `CameraIntrinsics` (`from_calibration_yaml(path)`, `from_hfov(w, h, hfov_deg)` - an approximation: square
pixels, centred principal point, no distortion), `MountPose` (`to_matrix()`), `CameraModel` (`calibrated`);
functions `undistort_pixel`, `pixel_to_ray_camera`, `camera_ray_in_frame`, `intersect_ground` (None if parallel or
behind), `pixel_to_ground(intr, T_frame_optical, u, v, ground_z)` (None if the ray misses the floor),
`project_point_to_pixel` (None if behind the camera or off-image), `optical_from_mount`, and
`solve_mount_pose(samples, intr, initial)` returning `(MountPose, rms_px)` (RMS Euclidean reprojection error per
sample; at least 3 samples, each `(T_frame_parent, (u, v), (x, y, z))`).

Calibration procedure:

1. Put a marker at measured floor points in front of the robot (known x, y, z in the ground frame).
2. For the gripper camera, capture the marker pixel plus the parent pose (`T_frame_parent`, e.g. from FK) at several
   arm poses; for the fixed overview camera, capture several marker positions (parent pose is identity).
3. Write a samples JSON (format in the script docstring) and run
   `python scripts/solve_camera_mount.py samples.json --camera gripper`.
4. Paste the printed YAML into the Ansible node config; the RMS should be around 1 px or less.

## Tests

Run from the repo root with the root pytest (`uv run pytest tests -v`): `tests/test_shared_utils.py`,
`tests/test_shared_battery.py` (config and guard logic), `tests/test_shared_camera_geometry.py` and `tests/test_shared_package.py` (packaging and the node
path dependency). They import via `shared.ros2_common`. Documented in [tests/README.md](../tests/README.md).
