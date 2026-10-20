---
title: Ground plane tracker
status: done
repo: ros2-lerobot-sverve-platform
value: "The plane survives short occlusions, follows the robot in odom and reports the slope ahead against the tilt_guard limits."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/tracker.hpp, nodes/ground_estimator_cpp/src/core/tracker.cpp, nodes/ground_estimator_cpp/test/core/test_tracker.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-plane-fit]
owns: [ground-tracker]
uses: [ground-config, ground-plane-fit, ground-synthetic-scene]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rgb-d-ground-estimation-and--ground-plane-tracker
branch: "worktree branch of rgb-d-ground-estimation-and--ground-plane-tracker (commit 62feeea2)"
verified_head: 62feeea26d11bc3478cd2242e0672a50632f2cf1
run: rgb-d-ground-estimation-and
provides: [ground-tracker]
acs_for_audit: []
process_gaps: []
---

# Ground plane tracker

## Requirement

- `include/ground_estimator_cpp/tracker.hpp`, `src/core/tracker.cpp` (design D2 prior, D3):
  - `Eigen::Vector3d gravity_up_in_base(const Eigen::Quaterniond &imu_orientation, const Eigen::Quaterniond &imu_to_base, double level_roll_deg, double level_pitch_deg)`: the world up vector expressed in base_link from the IMU orientation (REP-103), with the level reference removed so a base standing level yields +z exactly.
  - `struct Prior { Eigen::Vector3d up; bool from_imu; }`, `Prior choose_prior(std::optional<ImuSample> latest, double now_s, const ImuConfig &)`: the IMU up when the last sample is younger than `imu.timeout_s`, else base_link +z with `from_imu = false`.
  - `class PlaneTracker`: `update(const Plane &plane_base, const Eigen::Isometry3d &base_to_fixed, double stamp_s)` records a successful fit: the plane is stored in the fixed frame with EMA smoothing `fit.smoothing` (normal renormalised, d blended) in a short history (fits newer than `fit.hold_s` plus 1 s, pruned on update); `mark_failed(double stamp_s)` records a cloud that yielded no valid plane. `std::optional<TrackedPlane> current(const Eigen::Isometry3d &base_to_fixed, double stamp_s) const` selects the newest stored fit with fit stamp <= `stamp_s` (never a later fit: no look-ahead) and returns it transformed into base_link at `stamp_s` (via `base_to_fixed` of that stamp, i.e. the plane held in odom is propagated with the robot motion, and the up prior comes from the IMU at that stamp) with `age_s = stamp_s - fit_stamp`; `source` is `fit` when no failed cloud was recorded between that fit and `stamp_s`, `held` when one was (the plane is being held across failed fits). When `age_s > fit.hold_s`, or no fit at or before `stamp_s` exists, it returns nullopt: never extrapolated beyond the hold age. The same rule serves the ToF cloud stamp, the RGB-D cloud stamp (older than the newest fit) and the 2 Hz state stamp (now).
  - `struct Slope { double total_deg, roll_deg, pitch_deg; Level level; }`, `enum class Level { ok, warning, error }`, `Slope slope_vs_gravity(const Plane &plane_base, const Eigen::Vector3d &gravity_up_base, const SlopeConfig &)`: the floor tilt ahead as base roll/pitch would be on that plane (REP-103 signs as tilt_guard: roll > 0 left side up, pitch > 0 nose down), level = the worse axis vs the warn/error limits (tilt_guard semantics, no hysteresis here: the ground estimate is a report).
  - `double plane_vs_base_deg(const Plane &plane_base)`: angle between the plane normal and base_link +z; `double base_tilt_deg(const Eigen::Vector3d &gravity_up_base)`: angle between gravity up and base_link +z.
- Tests first in `test/core/test_tracker.cpp`, seen failing: identity orientation with zero level reference -> +z; the 0.7/1.3 deg level reference applied to an IMU reading exactly that -> +z within 1e-9; IMU pitched 10 deg -> up vector pitched by 10 deg with the REP-103 sign; stale IMU -> base_link +z and from_imu false; tracker holds a plane up to hold_s then returns nullopt; a query stamp between two fits gets the earlier fit with source fit and age = stamp - fit stamp; a query older than every stored fit gets nullopt; a failed cloud between the fit and the query makes the source held; history pruning keeps hold_s + 1 s; a plane stored while the base moved 0.5 m forward and yawed 30 deg in odom comes back with the same height in the new base_link; EMA blends two planes as configured (smoothing 1 = latest only); slope of a 5 deg pitched plane on a level robot = 5 deg pitch with the right sign; level ok/warning/error at the limits for roll and pitch; plane_vs_base for a 3 deg tilted plane = 3 deg.

## Acceptance conditions

- a1: the IMU prior removes the level reference, follows REP-103 signs and falls back to base_link +z when stale [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: current() returns the newest fit at or before the stamp with source fit or held and age = stamp - fit stamp, nullopt beyond hold_s or before the first fit, never a later fit [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: slope, slope level against the tilt_guard limits, plane_vs_base and base tilt are computed as specified [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/tracker.hpp`
- `nodes/ground_estimator_cpp/src/core/tracker.cpp`
- `nodes/ground_estimator_cpp/test/core/test_tracker.cpp`

## Interfaces

`tracker.hpp` (part `ground-tracker`): `gravity_up_in_base`, `choose_prior`, `PlaneTracker` (`update`, `mark_failed`, `current`), `TrackedPlane {Plane plane; std::string source; double age_s;}`, `slope_vs_gravity`, `plane_vs_base_deg`, `base_tilt_deg`, `Level`, `to_string(Level)`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_tracker.

## Steps

- write test_tracker.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): plane tracker, IMU prior and slope level`.

## Notes

tilt_guard (nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py) defines the roll/pitch convention and the level reference handling; mirror its signs. The limits default to the tilt_guard defaults (8/15 roll, 12/20 pitch).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs
