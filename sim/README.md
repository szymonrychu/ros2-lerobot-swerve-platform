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
report = simulate(plan_json, SceneConfig(support_z_m=-0.034), SimConfig())   # -> SimReport
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
| Moving jaw inner face | flares away from the fixed jaw toward the palm (meshes) | `mcp_server/jaw_profile.py` (this silhouette) | `grasp_sim.tcp.moving_jaw_inner_profile`; `tests/test_tcp.py` checks mcp_server's table |
| `gripperframe` site | 2 cm off the fixed jaw inner face along the jaw axis | `gripper_frame_link` is the jaw face | the harness does not use the site; the test checks the URDF frame carried by the MuJoCo `gripper` body |
| Jaw closing point | (-0.0015, 0.0002, 0.003) m in `gripper_frame_link` | `arm.tool_offset_m` (0.0010, -0.0056, -0.0014), the physical fixed-jaw tip fitted 2026-10-10 | `SceneConfig.tool_offset_m` shifts the jaws onto the configured point (8 mm), see TCP calibration |
| Actuators | position servos, kp 998, force +-2.94 Nm (STS3215 estimate) | real servos | see limitations |

### Joint convention

Plan joints are in the mcp_server **measured** joint space (`/follower/joint_states`, radians, names
`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`). The MuJoCo actuator target is
`urdf = measured + offset` (mcp_server `to_urdf`), with the offsets of `ansible/group_vars/client.yml`
(`joint_offsets_rad`, solved 2026-10-10 on the grid sheet, wrist_roll kept from 2026-10-08) as the defaults of
`SceneConfig.joint_offsets_rad`: pan 0.0182, lift -0.0345, elbow -0.1070, wrist_flex 0.0988, wrist_roll -0.0710, gripper 0. Set them to 0 to
replay raw URDF angles. Targets beyond the actuator range are clipped and reported in `warnings`.

Conventions that the example plans rely on: negative `elbow_flex` stretches the arm; at `wrist_roll` 0 the fixed
jaw is underneath and the moving jaw closes from above; gripper 0 is about 1.6 cm open, 0.6 rad about 6 cm.

## TCP calibration

The real robot's tool point (`arm.tool_offset_m` in `ansible/group_vars/client.yml`) is the truth the planner works
with. Since 2026-10-10 it is the physical fixed-jaw tip (0.0010, -0.0056, -0.0014) m in `gripper_frame_link`, fitted
together with the joint offsets and the camera mount on a 50 mm grid sheet
(`docs/calibration/2026-10-10/session/results.yaml`). That is only 8 mm from the stock Menagerie jaw closing point
(-0.0015, 0.0002, 0.003), so the sim jaw shift is now small. Until 2026-10-08..10 the configured point was an
effective TCP (0.0104, -0.0282, -0.0017), 3 cm beside the stock jaws, and the shift was the main sim calibration step.

Frames (checked, not a planner frame bug): mcp_server's IK chain ends at `gripper_frame_link` and applies the offset
as `T_gripper_frame_link @ [offset, 1]` (`ik.py` `forward`/`inverse`); `JawModel` maps it into `gripper_link` with the
same URDF transform. The MuJoCo `gripper` body is URDF `gripper_link` (`tests/test_model.py`, 2 micrometres), and
`grasp_sim.tcp` uses the URDF `gripper_frame_joint` (xyz -0.0079 -0.000218 -0.0981, rpy 0 pi 0). The stock closing
point is computed by `closing_point_gfl` (nearest points of the fixed finger and the moving jaw at the closed angle
-0.165). Re-fit `tool_offset_m` after any change of the joint offsets or the gripper.

What the harness does: `SceneConfig.tool_offset_m` (gripper_frame_link, m; None = stock jaws) translates the fixed
finger collision geoms (`fixed_jaw_box2..7`, the tip spheres and the finger collision mesh) and the moving jaw body
(hinge and all its geoms) by `tool_offset_m` minus the stock closing point, in `gripper_link`. The wrist housing
(`fixed_jaw_box1`) stays; the jaw opening per gripper angle is unchanged; `so101.xml` is untouched (MjSpec edits in
`grasp_sim.tcp.shift_jaws`). Visual meshes are not moved, so renders show the stock jaws while contacts use the shifted
fingers. `tests/test_tcp.py` asserts the calibrated closing point equals client.yml's `tool_offset_m` within 2 mm
(actual: 0.005 mm). `grasp_sim.tcp.client_tool_offset()` reads the deployed value.

Effect (historical, with the 2026-10-08 effective TCP): the 74 feasible plans of the "before" scenario matrix below lift
0 times with the stock jaws (`grasp-sim matrix <dir> --stock-jaws`) and 74 times with the calibrated jaws. With the
2026-10-10 tool point the jaw shift is 8 mm and the stock jaws are close to the calibrated ones.

## Scene

Frame: the arm base (URDF `base_link` origin) is the world origin, z up, x forward. The floor top is at
`z = -base_height_m` (default 0.100, the arm mount height corrected on the robot 2026-10-10 and set in `client.yml` as
`arm_base_height_m`; earlier estimates were 0.165 and 0.15).

```yaml
# SceneConfig (JSON or YAML; unknown keys are rejected)
base_height_m: 0.100
support_z_m: -0.030       # null/floor height = box on the floor; above = ledge/table; below = lower stair
support_edge_x_m: null    # where the ledge/stair starts (default: object x - 0.08); the floor ends there for a stair
support_depth_m: 0.6
support_width_m: 0.6
object:                   # null for no object
  shape: box                # or cylinder (upright; diameter = size_m x = y, height = z)
  size_m: [0.03, 0.03, 0.04]   # x, y, z full edges
  mass_kg: 0.05
  friction: [1.0, 0.005, 0.0001]
  x_m: 0.2
  y_m: 0.0
  yaw_rad: 0.0
  gap_below_m: 0.0          # > 0: the box rests on two 4 mm rails along its x edges, leaving a slot for a scoop
joint_offsets_rad: {shoulder_pan: 0.0182, ...}
limit_overrides_rad: {shoulder_lift: [-1.745, 1.9]}
tool_offset_m: null         # jaw closing point in gripper_frame_link, e.g. [0.0010, -0.0056, -0.0014] (TCP calibration)
```

MuJoCo combines two geoms' friction with the larger coefficient, and the jaw geoms use friction 1, so lowering the
object friction below 1 does not make it slippier against the jaws; raise it to make it grippier on the support.
Geoms named `floor`, `support` (ledge/stair only), `support_rail_left` / `support_rail_right` (gap rails, judged as
support) and `object_box` drive the report.

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

## Planner scenario matrix

Reproducible validation of the deployed planner (`nodes/mcp_server/mcp_server/grasp.py`) with the deployed config.
Two environments: the planner runs in the mcp_server uv env (ikpy, the node's own models), the replay in this one.

```bash
cd nodes/mcp_server
uv run python ../../sim/grasp_sim/scripts/plan_matrix.py --out /tmp/matrix            # about 40 s, 240 plans
uv run python ../../sim/grasp_sim/scripts/plan_matrix.py --out /tmp/m2 --params p.yaml --only 4x4x4_   # tuning
cd ../../sim/grasp_sim
uv run grasp-sim matrix /tmp/matrix               # about 10 s; table + /tmp/matrix/results.json
uv run grasp-sim matrix /tmp/matrix --stock-jaws  # same plans, uncalibrated jaws
```

Run `plan_matrix.py` only from the `nodes/mcp_server` env as above: it imports `mcp_server`, so started from
`sim/grasp_sim` it fails with `ModuleNotFoundError: No module named 'mcp_server'`. The replay needs only the sim env.

`plan_matrix.py` reads the `mcp_server` block of `ansible/group_vars/client.yml` (tool offset, joint offsets, limit
overrides, floor guard, grasp defaults; `--params` overrides grasp values) and plans from the executor's default
folded seed: boxes 4x4x4, 3x3x6, 6x6x3 cm (depth x width x height); centre radius 0.20, 0.25, 0.30 m straight ahead
and 0.25 m at 30 deg; surfaces placed relative to the configured floor (`arm.floor_z_m`, -0.100 in the arm frame):
floor, `ledge+0.07` and `ledge+0.15` (7 and 15 cm above it), `stair-0.10` (10 cm below it); strategies `top_down`,
`angled45`, `scoop` (box flat on the surface), `scoop_gap` (box 2 cm up on rails, `gap_below_m` 0.02) and `auto`. Ledges
and the stair start 8 cm before the box centre along arm x (`support_edge_x`, the scene's support edge). The stair is
passed to the planner as a surface region (`surfaces`: the robot floor up to the edge, a half-plane 10 cm lower beyond
it), so the planner sees the step edge; floor and ledges keep a single surface height. It writes `index.json` (scene constants, executor timing, planner params, one entry per scenario) and one GraspPlan JSON
per scenario. `grasp-sim matrix` replays every feasible plan the way `GraspExecutor.execute` streams it (start at the
pre-grasp with the roll done, gripper open at `gripper_velocity_rps`, approach/grasp/lift/retreat along the planned
joint samples with one quintic profile each at the waypoint `speed_scale`, close to `gripper_closed_rad` and hold)
on a scene with the calibrated jaws, the box yawed to face the shoulder pan axis (`index.json` `pan_axis_xy`, 3.9 cm ahead
of the arm base origin: the planner puts the jaws across the heading from that axis; until 2026-10-10 the box faced the
origin and stood 5 deg off the jaws at r25a30). "Lifted" = the SimReport grasp success (2 cm up and
still between the jaws after the lift and at the end) with no arm-link contact with floor or support. Each result row
also has `descent_tilt_deg` / `descent_push_m`: the worst object tilt and push in the approach and grasp segments, the
open jaws coming down onto the object (the SimReport `approach_*` figures leave out the grasp slide, where contact is
allowed).

```bash
cd nodes/mcp_server
uv run python ../../sim/grasp_sim/scripts/plan_matrix.py --out /tmp/jar --tipover   # the 2026-10-10 jar tip-over set
cd ../../sim/grasp_sim && uv run grasp-sim matrix /tmp/jar
```

`--tipover` plans, instead of the matrix, the jar that tipped on the robot: a light (30 g) upright cylinder 39 mm across
and 6 cm tall on the floor at base_link (0.316, 0.0), top_down, angled 50 (45 cannot reach the approach start there)
and auto, each replayed with the jar where the planner was told (`err0`) and 3 mm toward the fixed jaw (`err3`, a
perception error of the size seen on the robot; `sim_offset_xy_m` in the entry).

Results 2026-10-09 (lifted / feasible of 16 scenarios per cell):

| Strategy | Before 3x3x6 | Before 4x4x4 | Before 6x6x3 | After 3x3x6 | After 4x4x4 | After 6x6x3 |
|---|---|---|---|---|---|---|
| top_down | 7/7 | 7/7 | 6/6 | 6/6 | 7/7 | 6/6 |
| angled45 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| scoop (flat) | 0/0 | 0/0 | 0/0 | 0/0 (no gap) | 0/0 (no gap) | 0/0 (no gap) |
| scoop_gap | 1/1 | 0/0 | 0/0 | 3/3 | 3/3 | 3/3 |
| auto | 12/12 | 12/12 | 11/11 | 11/11 | 12/12 | 11/11 |

Re-run 2026-10-10 after the faster angled planning (secant heading convergence in `ArmKinematics.inverse_flange`,
joints within 2e-3 rad of the table above): the matrix plans 240 scenarios in about 40 s instead of 5 min, 80 feasible,
and replays identically: top_down 19/19, angled45 18/18, scoop_gap 9/9, auto 34/34 lifted, scoop (flat) 0 feasible.

Re-run 2026-10-10 with the measured arm mount (floor at -0.104 instead of -0.15, base 0.104 m above it; supports
kept at the same heights relative to the floor and now named by that height: ledge -0.08 is `ledge+0.07`, ledge 0.0
is `ledge+0.15`, stair -0.25 is `stair-0.10`). Before = the same planner with the 2026-10-09 client.yml (floor -0.15),
reproduced exactly (80 feasible, 80 lifted, no arm contact); after = 93 feasible, 73 lifted, 5 arm contacts
(lifted / feasible):

| Strategy | Floor before | Floor after | Low ledge before | Low ledge after | High ledge before | High ledge after | Stair before | Stair after |
|---|---|---|---|---|---|---|---|---|
| top_down | 9/9 | 9/9 | 9/9 | 0/0 | 0/0 | 0/0 | 1/1 | 3/9 |
| angled45 | 6/6 | 3/3 | 3/3 | 8/8 | 9/9 | 9/9 | 0/0 | 0/5 |
| scoop (flat) | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| scoop_gap | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/4 | 0/0 | 0/0 |
| auto | 12/12 | 12/12 | 12/12 | 8/8 | 9/9 | 9/9 | 1/1 | 3/11 |

Every floor and ledge plan still lifts except one: the 3x3x6 gap scoop at r25a30 on the high ledge (jaw on the ledge,
not lifted). With the higher floor the stair is reachable at r20, r25, r30 and r25a30 (before only r20), and those new
plans expose the missing step-edge model: the jaws touch the upper floor edge (about 0.5 mm) and the box is not lifted
(4x4x4 and 6x6x3 top_down, angled45), and at r30/r25a30 the angled45 wrist hits the floor edge (5 arm contacts, all
on the stair). Coverage moved: top_down no longer reaches the low ledge (3.4 cm below the arm base, pre-grasp too
high) and angled45 loses three floor plans, while angled45 gains five low-ledge plans.

Re-run 2026-10-10 with the grid-sheet calibration (`client.yml` joint offsets pan 0.0182, lift -0.0345, elbow -0.1070,
wrist_flex 0.0988; tool point = physical fixed-jaw tip 0.0010/-0.0056/-0.0014; mount and floor -0.104 unchanged;
`docs/calibration/2026-10-10/session/results.yaml`). Before = the table above (morning, 2026-10-08 calibration: 93
feasible, 73 lifted, 5 arm contacts); after = 89 feasible, 85 lifted, 0 arm contacts, 0 jaw-surface contacts
(lifted / feasible):

| Strategy | Floor before | Floor after | Low ledge before | Low ledge after | High ledge before | High ledge after | Stair before | Stair after |
|---|---|---|---|---|---|---|---|---|
| top_down | 9/9 | 10/10 | 0/0 | 0/0 | 0/0 | 0/0 | 3/9 | 9/9 |
| angled45 | 3/3 | 3/3 | 8/8 | 7/9 | 9/9 | 9/9 | 0/5 | 0/0 |
| scoop (flat) | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| scoop_gap | 3/3 | 3/3 | 3/3 | 3/3 | 3/4 | 4/4 | 0/0 | 0/0 |
| auto | 12/12 | 12/12 | 8/8 | 7/9 | 9/9 | 9/9 | 3/11 | 9/9 |

Re-run 2026-10-10 with the corrected arm mount height (floor at -0.100 instead of -0.104, base 0.100 m above it; supports
keep their heights relative to the floor, so every surface moves up 4 mm; the golden plans are unchanged). Before = the
table above (89 feasible, 85 lifted, 0 arm contacts, 0 jaw-surface contacts); after = 93 feasible, 85 lifted, 4 arm
contacts, 4 jaw-surface contacts (lifted / feasible):

| Strategy | Floor before | Floor after | Low ledge before | Low ledge after | High ledge before | High ledge after | Stair before | Stair after |
|---|---|---|---|---|---|---|---|---|
| top_down | 10/10 | 10/10 | 0/0 | 0/0 | 0/0 | 0/0 | 9/9 | 9/9 |
| angled45 | 3/3 | 3/3 | 7/9 | 7/9 | 9/9 | 9/9 | 0/0 | 0/2 |
| scoop (flat) | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| scoop_gap | 3/3 | 3/3 | 3/3 | 3/3 | 3/4 | 4/4 | 0/0 | 0/0 |
| auto | 12/12 | 12/12 | 7/9 | 7/9 | 9/9 | 9/9 | 9/9 | 9/11 |

The 4 mm change adds one more stair reach at r30 for angled45 (4x4x4 and 3x3x6), and those plans (and the matching auto
plans) hit the floor with the wrist link (4 arm contacts, all on the stair at r30); the high-ledge scoop_gap that did
not lift before now lifts. The floor, low-ledge and top_down results are otherwise unchanged.

Re-run 2026-10-10 with the step-edge model (`mcp_server/surfaces.py`; the stair passed as a step surface at the scene
edge; planner checks of jaw points, gripper body hull and forearm / wrist capsules against surfaces and step faces;
pre-grasp and lift above the step and steeper pitches for an object beyond it). Before = the table above (93 feasible,
85 lifted, 4 arm contacts, 4 jaw-surface contacts); after = 104 feasible, 100 lifted, 0 arm contacts, 0 jaw-surface
contacts (lifted / feasible):

| Strategy | Floor before | Floor after | Low ledge before | Low ledge after | High ledge before | High ledge after | Stair before | Stair after |
|---|---|---|---|---|---|---|---|---|
| top_down | 10/10 | 10/10 | 0/0 | 0/0 | 0/0 | 0/0 | 9/9 | 9/9 |
| angled45 | 3/3 | 3/3 | 7/9 | 7/9 | 9/9 | 9/9 | 0/2 | 12/12 |
| scoop (flat) | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| scoop_gap | 3/3 | 3/3 | 3/3 | 3/3 | 4/4 | 4/4 | 0/0 | 0/0 |
| auto | 12/12 | 12/12 | 7/9 | 7/9 | 9/9 | 9/9 | 9/11 | 12/12 |

Floor and ledge results are identical (no surfaces there, plans unchanged). On the stair the 45 deg candidates that
ran the wrist into the step edge (r30) are now rejected with "wrist link clearance ... to the step edge of 'stair'"
and a steeper candidate over the step is chosen: angled45 plans at 65 deg (75 deg for 6x6x3 at r25a30, where the
fixed jaw body grazed the diagonal edge at 65 deg; 75 deg at r20, where the 65 deg retreat above the step is out of reach), all 12 stair angled
plans lift, and auto lifts all 12 stair cases (top_down where it reaches, angled 65 deg at r30). Calibration of the
clearances from this matrix: wrist capsule (radius 2 cm) clearance 1.1 cm still touched the edge, 1.9 cm and more did
not (`surface_link_clearance_m` 0.015); a 5 cm lift below the upper floor dragged the gripper body over the edge in
the retreat (the lift now clears the upper surface); the jaw tip points alone missed a fixed-jaw body contact at a
diagonal edge (the gripper body hull is checked now).

Re-run 2026-10-10 with the jaw clearance (`nodes/mcp_server/README.md`, "Jaw clearance": the fixed jaw 7.5 mm off the
object's side face instead of on it, the moving jaw 7.5 mm clear over the object's depth in the jaws, the tilted moving
jaw modelled from these jaw meshes) and the box facing the pan axis. Planned 240, 104 feasible (same set), 100 lifted, 0
arm contacts, 0 jaw-surface contacts, no descent tilt above 0.6 deg, exactly what the table above lifts (the same four
6x6x3 angled45 / auto low-ledge plans at r25 and r25a30 are not lifted). Same planner as before on the corrected scene
(box facing the pan axis): 102 lifted; the r25a30 low-ledge 6x6x3 angled45 and auto lift there, because the old fixed jaw
sat on the box face and the moving jaw only closed it. With the clearance the moving jaw first pushes the 6 cm box 7.5 mm
across the ledge with its face tilted 43 deg (6 cm is near the widest opening), and the grip slips in the retreat: every
clearance from 4 to 8 mm loses these two (`--params` sweep of `fixed_jaw_clearance_m`), only the old fixed jaw on the face
keeps them. That is the price of not landing the fixed
jaw on the object; the floor, stair, top_down and every 3x3x6 / 4x4x4 plan lift as before. Golden plans regenerated.

Tip-over set (`--tipover`), descent tilt / push of the jar while the open jaws come down (lifted in brackets):

| Plan | Before err0 | Before err3 | After err0 | After err3 |
|---|---|---|---|---|
| top_down (and auto) | 0.4 deg / 0.2 mm (lifted) | 18.5 deg / 9.7 mm (lifted) | 0.0 / 0.0 (lifted) | 0.0 / 0.0 (lifted) |
| angled 50 | 0.0 / 0.0 (not lifted) | 2.0 deg / 2.6 mm (not lifted) | 0.0 / 0.0 (not lifted) | 0.0 / 0.0 (not lifted) |

Before = the planner of commit 96f413d (fixed jaw on the jar face, tip-only opening 0.53 rad): with the jar 3 mm toward
the fixed jaw the fixed jaw lands on its rim and tips it 18.5 deg on the way down (the robot's tip-over); exactly
placed, the open moving jaw grazes the jar top 2.75 cm into the jaws. After: 7.5 mm on both sides, opened to 0.97 rad for
the 4.2 cm the jar reaches into the jaws, nothing touches the jar until the close, and it lifts in all four top_down
runs. Angled 50 grips the jar but it pivots out of the jaws on the slow lift, before and after (the tall narrow
object failure mode below, not a tip on approach). Clearing only the tips (7.5 mm, opened to 0.53 rad) still touched
the jar top with the moving jaw mesh (3.4 deg, 2.3 mm): that is why the opening is now held over the object's depth.

Per strategy over the whole matrix (before the step-edge model): top_down 19/19 lifted (19 of 19 feasible, 29 infeasible), angled45 19/21, scoop_gap
10/10, auto 37/39, scoop (flat) 0 feasible. The only failures are 6x6x3 angled45 and auto on the low ledge at r25 and
r25a30 (box not lifted, no contact with floor or support). The stair is now fully lifted (the former step-edge
contacts disappeared: all stair plans are top_down, 9/9) and no plan touches the floor or support with an arm link.

Before = the planner and client.yml before this tuning (auto order scoop first, scoop_max_pitch_deg 25), replayed in the
calibrated sim; with the stock jaws the same plans lifted 0 of 74. Target set (4x4x4 and 6x6x3 on floor and both
ledges, top_down and angled45): 24 of 48 scenarios feasible, 24/24 lifted, no arm-floor/support contact in any run.

Tuning runs (`--params`): `scoop_max_pitch_deg` 25 leaves 1 of 48 gap scoops feasible; 40 gives 9 feasible, 9/9
lifted; 45 gives 12 feasible, 9 lifted; 60 gives 29 feasible, 17 lifted (steep scoops swing the moving jaw into the box
top or tip the 3x3x6). Default now 40. Shorter `pre_grasp_clearance_m` / `approach_distance_m` (0.03) added only 1 feasible
plan (a gap scoop) and none for top_down/angled, so those defaults stay. Flat scoops are now infeasible with "no gap under object for the fixed jaw";
`auto` tries top_down, angled, scoop.

Remaining failure modes:

- Reach (2026-10-10 grid calibration): top_down is infeasible at r30 and on both ledges (pre-grasp too high),
  angled45 at r20 on the floor (too close) and on the stair.
- 6x6x3 angled45 and auto on the low ledge at r25 and r25a30: feasible but the box is not lifted (no contact with floor
  or support).
- Stair edge: modelled when the caller passes `surfaces` (the matrix does for the stair); a caller that gives only
  `surface_z_m` for a stair still gets the single-surface plan without edge checks.
- Steep scoops above 40 deg with a gap: moving jaw hits the box top, slips or tips tall boxes (why the default is 40).
- Sim limits apply (see Limitations): exact object pose, a 2.94 Nm gripper squeeze, no compliance.

## Example plans

`grasp_sim/examples.py` builds scoop plans with a small damped-least-squares IK on the MuJoCo model (position plus
approach pitch; the 5-DOF arm cannot choose the approach direction freely): jaws opened with `wrist_roll` -1.57 so
the jaws straddle the object sideways, a line approach (5 mm steps) with the jaw tips 2 mm above the support,
close, 6 cm lift, 4 cm retreat. The arm is short: a target about 0.2 m ahead and 0.13 m below the shoulder needs a
steep approach (pitch about 57 deg; the ledge case, 3.4 cm below the arm base with the measured mount, about 69 deg;
the stair case about 69 deg and 0.17 m).

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
