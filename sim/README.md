# sim/ - dev-only simulation tooling

`sim/grasp_sim/` replays timed joint plans for the SO-101 arm in MuJoCo on the Mac (arm64, no ROS2) and judges them:
clearance to the floor and to a ledge/stair, unintended contacts, tipping or pushing of the object, lift success and
actuator load. It is a development harness only. It is not a ROS2 node, is not listed in `ansible/` and is never
deployed.

## Install and test

```bash
cd sim/grasp_sim
uv sync                  # Python 3.12, mujoco, numpy, pydantic, pyyaml, pillow
uv run pytest -q         # or: uv run poe test
uv run poe lint          # ruff check + ruff format --check (uv run poe lint-fix to fix)
```

## Run a plan

```bash
cd sim/grasp_sim
uv run grasp-sim example ledge --out /tmp/grasp      # writes ledge.plan.json + ledge.scene.json
uv run grasp-sim run /tmp/grasp/ledge.plan.json --scene /tmp/grasp/ledge.scene.json
uv run grasp-sim run plan.json --scene scene.yaml --sim thresholds.yaml --report report.json   # full SimReport
uv run grasp-sim run grasp_plan.json --grasp-plan --scene scene.yaml                           # via the adapter
```

Exit code 0 = pass, 1 = fail (reasons printed). `--json` prints the full report. Examples: `floor`, `ledge`, `stair`
(passing scoops) and `tip` (a failing scoop that topples a tall box).

Library API:

```python
from grasp_sim.config import SceneConfig, SimConfig
from grasp_sim.replay import simulate
report = simulate(plan_json, SceneConfig(support_z_m=-0.08), SimConfig())   # -> SimReport
```

## View and record

```bash
uv run mjpython -m grasp_sim.cli run plan.json --scene scene.json --render       # interactive, real time
uv run grasp-sim run plan.json --scene scene.json --frames out/frames --gif out/scoop.gif --fps 10 --size 640x480
```

`--render` uses `mujoco.viewer.launch_passive`, which on macOS only works under `mjpython` (installed with the
`mujoco` package; run it through `uv run mjpython`). Under plain `python` the CLI stops with that hint. The window
stays open after the replay until it is closed. `--frames` writes numbered PNGs and `--gif` an animated GIF, both
rendered offscreen (no window needed).

## Model

The robot is the MuJoCo Menagerie `robotstudio_so101` model, vendored unmodified in `grasp_sim/assets/so101/` with
its Apache-2.0 LICENSE and the upstream commit (see the README there). Scenes are generated in Python
(`grasp_sim/scene.py`, `mujoco.MjSpec`), so `so101.xml` stays pristine.

Verified against `nodes/web_ui/urdf/so101_arm.urdf` (so101_new_calib): same joint names, axes, zero poses and link
frames. `tests/test_model.py` compares every body frame with a numpy URDF forward-kinematics chain at six joint
configs (largest position error here: 2 micrometres, tolerance 2 mm). Differences:

| Item | MuJoCo model | URDF / mcp_server | Handling |
|---|---|---|---|
| shoulder_lift range | -1.745 .. 1.745 | mcp_server widens to -1.745 .. 1.9 (`joint_limit_overrides_rad`) | `SceneConfig.limit_overrides_rad` widens joint and actuator range |
| `gripperframe` site | 2 cm off the fixed jaw inner face along the jaw axis | `gripper_frame_link` is the jaw face | the harness does not use the site; the test checks the URDF frame carried by the MuJoCo `gripper` body |
| Actuators | position servos, kp 998, force +-2.94 Nm (STS3215 estimate) | real servos | see limitations |

### Joint convention

Plan joints are in the mcp_server **measured** joint space (`/follower/joint_states`, radians, names
`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`). The MuJoCo actuator target is
`urdf = measured + offset` (mcp_server `to_urdf`), with the offsets of `ansible/group_vars/client.yml`
(`joint_offsets_rad`, solved 2026-10-08) as the defaults of `SceneConfig.joint_offsets_rad`:
pan -0.0619, lift -0.0103, elbow -0.1381, wrist_flex 0.2477, wrist_roll -0.0710, gripper 0. Set them to 0 to
replay raw URDF angles. Targets beyond the actuator range are clipped and reported in `warnings`.

Conventions that the example plans rely on: negative `elbow_flex` stretches the arm; at `wrist_roll` 0 the fixed
jaw is underneath and the moving jaw closes from above; gripper 0 is about 1.6 cm open, 0.6 rad about 6 cm.

## Scene

Frame: the arm base (URDF `base_link` origin) is the world origin, z up, x forward. The floor top is at
`z = -base_height_m` (default 0.15 as specified for this harness; the value measured on the robot and set in
`client.yml` as `arm_base_height_m` is 0.165, use `base_height_m: 0.165` to match it).

```yaml
# SceneConfig (JSON or YAML; unknown keys are rejected)
base_height_m: 0.15
support_z_m: -0.08        # null/floor height = box on the floor; above = ledge/table; below = lower stair
support_edge_x_m: null    # where the ledge/stair starts (default: object x - 0.08); the floor ends there for a stair
support_depth_m: 0.6
support_width_m: 0.6
object:                   # null for no object
  size_m: [0.03, 0.03, 0.04]   # x, y, z full edges
  mass_kg: 0.05
  friction: [1.0, 0.005, 0.0001]
  x_m: 0.2
  y_m: 0.0
  yaw_rad: 0.0
joint_offsets_rad: {shoulder_pan: -0.0619, ...}
limit_overrides_rad: {shoulder_lift: [-1.745, 1.9]}
```

MuJoCo combines two geoms' friction with the larger coefficient, and the jaw geoms use friction 1, so lowering the
object friction below 1 does not make it slippier against the jaws; raise it to make it grippier on the support.
Geoms named `floor`, `support` (ledge/stair only) and `object_box` drive the report.

## Replay plan format

`simulate(plan_json, scene_cfg)` takes JSON text, a path, a list, or `{"samples": [...]}`:

```json
[
  {"t": 0.0, "label": "open", "joints": {"shoulder_pan": 0.0, "shoulder_lift": 0.1, "elbow_flex": -0.3,
                                           "wrist_flex": 0.2, "wrist_roll": -1.5, "gripper": 0.0}},
  {"t": 1.0, "joints": {"gripper": 0.6}},
  {"t": 2.5, "label": "approach", "joints": {"shoulder_lift": 0.5}}
]
```

- `t` strictly increasing seconds; joints linearly interpolated between samples. The first sample lists all six
  joints, later samples may list a subset (others hold).
- `label` (optional): `open, pre_grasp, approach, grasp, close, lift, retreat`. A label applies from its sample
  until the next labelled sample. Samples before the first label are unlabelled and count as approach phase.
- The first sample is teleported in and held for `settle_s` so the object comes to rest before the plan starts.

## SimReport

JSON fields (`grasp_sim/report.py`): `passed`, `reasons[]`, `warnings[]`, `duration_s`, `segments[]` (per label:
`label, t_start, t_end, min_clearance{jaws_floor, jaws_support, wrist_floor, wrist_support}` in metres with negative =
penetration and null when the surface does not exist, `max_actuator_force_nm`, `max_object_tilt_deg`,
`max_object_displacement_m`), `first_unintended_contact{t, label, kind, geom_a, geom_b}`, `event_counts`,
`object{start_pos, final_pos, approach_max_tilt_deg, approach_max_displacement_m, tipped, pushed, lift_height_m,
lifted_after_lift, lifted_at_end}`, `grasp_success`, `max_actuator_force_nm{joint}`, `saturated_actuators[]`.

`passed` is true when `reasons` is empty. Reasons come from:

- unintended contact: arm links (shoulder, upper arm, forearm, wrist) with the floor or support at any time; jaw
  contact with floor/support only if `allow_jaw_surface_contact` is false (default true, a scoop skims the floor);
  any arm contact with the object while the label is not in `contact_allowed_labels` (default grasp, close, lift,
  retreat);
- tipping: object tilt above `tilt_threshold_deg` (10) while the label is outside `contact_allowed_labels`;
- pushing: horizontal object displacement above `push_threshold_m` (0.01) in the same phase;
- grasp failure, only for plans with a `lift` label: the object must be at least `lift_min_height_m` (0.02) above its
  start and within `lift_hold_distance_m` (0.06) of the jaws, both at the end of the lift segment and at the end of
  the plan.

Saturated actuators (torque at the 2.94 Nm limit) are listed but not a failure: a stretched or steep reach holds
against the servo limit, just as the real shoulder stalls above about 1.85 rad.

## GraspPlan adapter (planner contract)

The grasp planner in `nodes/mcp_server` is written in parallel. `grasp_sim.adapter.grasp_plan_to_replay` converts its
output; the schema it expects (aliases in brackets are also accepted):

```json
{
  "units": "rad",
  "waypoints": [
    {"label": "pre_grasp", "joints": {"shoulder_pan": 0.0, "shoulder_lift": 0.2, "elbow_flex": -0.4,
                                       "wrist_flex": 0.6, "wrist_roll": -1.57, "gripper": 0.6}},
    {"label": "approach", "duration_s": 2.0,
     "joint_samples": [{"t": 0.0, "joints": {"...": 0.0}}, {"t": 2.0, "joints": {"...": 0.0}}]},
    {"label": "grasp", "t": 4.5, "joints": {"...": 0.0}}
  ]
}
```

- Waypoint list under `waypoints` [`segments, trajectory, samples, phases, steps`] or a bare list.
- Label under `label` [`phase, name, type`]; case, `-` and spaces are normalised, a few aliases (`descend` ->
  `approach`, `pregrasp`, `retract` -> `retreat`) are mapped, unknown labels are dropped (sample stays unlabelled).
  The label goes on the waypoint's first sample.
- Joints under `joints` [`joint_positions, positions, position, q`] as a dict (`_joint` / `_rad` suffixes stripped) or a
  list in `joint_names` order (default the six joints above); the gripper may be a separate `gripper` / `gripper_rad`.
- Time: absolute `t` [`time, t_s, time_s, time_from_start[_s]`, also `{"sec", "nanosec"}`], or relative `duration_s`
  [`duration, dt, dt_s`] added to the previous waypoint's end, else 1 s spacing. A `joint_samples` list whose times
  restart below the previous time is taken relative to the waypoint start.
- `units`: `rad` (default) or `deg`. Values are in the measured joint space (the offsets above are applied in the sim).
- The first sample must give all six joints, else `ValueError`.

## Example plans

`grasp_sim/examples.py` builds scoop plans with a small damped-least-squares IK on the MuJoCo model (position plus
approach pitch; the 5-DOF arm cannot choose the approach direction freely): jaws opened with `wrist_roll` -1.57 so
the jaws straddle the object sideways, a line approach (5 mm steps) with the jaw tips 2 mm above the support,
close, 6 cm lift, 4 cm retreat. The arm is short: a target about 0.2 m ahead and 0.13 m below the shoulder needs a
steep approach (pitch about 57 deg; the stair case about 69 deg and 0.17 m).

- `floor`, `ledge` (7 cm above the floor), `stair` (6 cm below the floor plane): pass, object lifted about 6 cm.
- `tip`: a 2x2x7 cm box and a 2.5 cm sideways offset: a jaw hits the box in the approach, it tips (about 90 deg) and
  is pushed away, the scoop fails with the reasons in the report.

## Limitations

- Contact fidelity: convex-hull/box collision geoms, soft contacts (`solref 0.01 1`, elliptic cone), one friction
  model. Thin jaw plates and 2 mm clearances are right at the model's resolution; treat clearance numbers as a
  guide of about +-3 mm, not a measurement.
- Servo model: a position actuator with PD gains (kp 998, kv 2.7) and a 2.94 Nm force limit. No servo compliance,
  deadband, backlash (the Menagerie `backlash` class is unused), temperature or voltage sag, no firmware
  acceleration/velocity profile and no 25 Hz command stream; the plan is interpolated and applied every 5 ms step.
- Calibration drift: the measured-to-URDF offsets are one fixed estimate; real arms differ by a few mm at the tool.
- No camera/perception; the object pose is exact; one rigid box per scene.
- Joint-space replay only: the sim does not check the plan's own Cartesian intent, only what the joints do.
