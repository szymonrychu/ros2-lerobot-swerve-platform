# RGB-D ground estimation and plane-relative obstacles implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Ground package config

- Status: done
- Files: `nodes/ground_estimator_cpp/CMakeLists.txt`, `nodes/ground_estimator_cpp/package.xml`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`, `nodes/ground_estimator_cpp/src/core/config.cpp`, `nodes/ground_estimator_cpp/test/core/test_config.cpp`, `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `scripts/cpp-node-test.Dockerfile`, `.github/workflows/ci.yml`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q`
- Verified head: f45710ed598fc2e7db1680835eb1f5fb620303ba

#### Requirement

- New ament_cmake package `nodes/ground_estimator_cpp` (C++17), layout of `nodes/head_rgbd_cpp`: core static library `ground_estimator_core` globbing `src/core/*.cpp` (no ROS node code), one gtest per `test/core/test_*.cpp` (named after the file stem, compile definitions `GROUND_TEST_DATA_DIR` and `GROUND_CONFIG_DIR`), ament linters with the root `.clang-format`, `install(DIRECTORY config ...)`. The node library/executable targets are added later by ground-node-wiring; this task's CMakeLists builds the core library and its tests.
- package.xml dependencies: rclcpp, sensor_msgs, std_msgs, tf2, tf2_ros, tf2_eigen, pcl_conversions, libpcl-all-dev (PCL common, sample_consensus, segmentation, search, kdtree), eigen, OpenCV (core, imgproc), yaml-cpp, nlohmann-json, ros2_common_cpp, prometheus-cpp; test deps ament_cmake_gtest, ament_lint_auto, ament_lint_common. When a key has no rosdep rule, `scripts/cpp-node-test.Dockerfile` maps it to its apt package in the existing rosdep yaml, with a comment (as for prometheus-cpp).
- `include/ground_estimator_cpp/config.hpp`, `src/core/config.cpp`: `struct Config` with nested structs, `Errors validate(const YAML::Node &)`, `Config from_yaml(const YAML::Node &)`, `Config load_config_or_exit()` reading `GROUND_ESTIMATOR_CONFIG` (default `/etc/ros2/ground_estimator_cpp/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (exit 78, unknown keys rejected at every level, each error names its key). Keys and defaults (design D2-D7, D11): `cloud_topic` `/tof/points_filtered`; `imu_topic` `/imu/data`; `rgbd_cloud_topic` `/head_rgbd/points`; `output_namespace` `/ground`; `base_frame` `base_link`; `fixed_frame` `odom`; `tf_timeout_s` 0.1 (0..0.5); `imu.timeout_s` 0.5 (> 0); `imu.level_reference_roll_deg` 0.7 and `imu.level_reference_pitch_deg` 1.3 (|v| <= 30); `fit.distance_m` 0.02 (0 < v <= 0.1); `fit.max_normal_angle_deg` 10.0 (0 < v <= 30); `fit.max_offset_m` 0.08 (0 < v <= 0.3); `fit.min_inliers` 150 (>= 10); `fit.min_inlier_ratio` 0.2 (0 < v <= 1); `fit.max_iterations` 100 (>= 10); `fit.candidate_band_m` 0.15 (> fit.distance_m); `fit.max_range_m` 2.5 (> 0); `fit.hold_s` 1.0 (>= 0); `fit.smoothing` 0.5 (0 < v <= 1, 1 = no smoothing); `bands.obstacle_max_m` 0.6, `bands.drop_max_m` 1.0 (both > traversability max step), `bands.max_range_m` 2.0 (> 0); `traversability.wheel_radius_m` 0.06 (> 0, equals swerve_drive_controller `wheel_radius_m`), `traversability.max_step_fraction` 0.3333 (0 < v < 1; derived `max_step_m() = 2 * wheel_radius_m * max_step_fraction` = 0.04 m, one third of the wheel diameter, must exceed fit.distance_m), `traversability.max_slope_deg` 8.0 (0 < v <= min(slope.roll_warn_deg, slope.pitch_warn_deg)), `traversability.edge_band_m` 0.05 (> 0); `warnings.frame` `map` (frame of the published warning areas); `edges.cluster_tolerance_m` 0.1 (> 0), `edges.min_cluster_points` 10 (>= 3), `edges.max_regions` 8 (1..16), `edges.step_max_height_m` 0.2 (> max_step_m, <= bands.obstacle_max_m), `edges.step_flatness_m` 0.03 (> 0); `slope.roll_warn_deg` 8, `slope.roll_error_deg` 15, `slope.pitch_warn_deg` 12, `slope.pitch_error_deg` 20 (warn < error); `colour.enabled` true, `colour.max_mahalanobis` 3.0 (> 0), `colour.forgetting` 0.1 (0 < v <= 1), `colour.min_floor_samples` 200 (>= 10); `state.rate_hz` 2.0 (0 < v <= 10); `metrics_port` 19119 (int 1..65535 or null; 19119 is free per tests/test_node_metrics_wiring.py). Accessors derive the output topics `<ns>/obstacle_points`, `<ns>/drop_points`, `<ns>/labelled_points`, `<ns>/state`, `<ns>/warning_areas`, and `max_step_m()`.
- `config/ground_estimator.yaml`: the defaults, every key commented with meaning and unit.
- `.github/workflows/ci.yml`: `ground_estimator_cpp` in the C++ package matrix (tests/test_ci_workflow.py and tests/test_node_lint_coverage.py keep it in sync).
- Tests first in `test/core/test_config.cpp`, seen failing: defaults; `ShippedYamlDocumentsEveryKey` (reads `config/ground_estimator.yaml` as text; every key line `^\s*[a-z_]+:` has a `#` comment on the same line or the line above); the shipped yaml loads; every range and cross-key rule above rejected with the key named; `MaxStepIsOneThirdOfWheelDiameter` (0.06 m radius -> 0.04 m, 0.09 m -> 0.06 m); max_slope_deg above the tilt_guard warn limits rejected; unknown nested key rejected; derived topics.

#### Acceptance conditions

- a1: the config loads with the documented defaults and rejects every invalid value and cross-key rule naming the key [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the package builds with PCL, OpenCV and nlohmann-json and its gtests and ament linters pass in the cpp-node-test container (the Pi -j2 build is validated by the runbook deploy) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the CI matrix lists ground_estimator_cpp and the sync tests pass [evidence: command: uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q]
- a4: the shipped config yaml has a comment for every key (gtest ShippedYamlDocumentsEveryKey) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

#### Files

- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/package.xml`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `scripts/cpp-node-test.Dockerfile`
- `.github/workflows/ci.yml`

#### Interfaces

`ground_estimator_cpp::Config` and nested `ImuConfig`, `FitConfig`, `BandsConfig`, `EdgesConfig`, `SlopeConfig`, `ColourConfig`, `StateConfig`; `validate`, `from_yaml`, `load_config_or_exit`; topic accessors `obstacle_points()`, `drop_points()`, `labelled_points()`, `state()` (part `ground-config`).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` ends `cpp-node-test: ground_estimator_cpp PASS`.

#### Steps

- write CMakeLists/package.xml/test_config.cpp, run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement config.cpp and the yaml until PASS
- update ci.yml, run `uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q`
- commit `feat(ground_estimator_cpp): package skeleton and config`.

#### Notes

shared/ros2_common_cpp provides `load_config_or_exit`, `Metrics`, process CPU collector (see nodes/head_rgbd_cpp, nodes/map_guard_cpp). nlohmann-json is already a dependency of map_guard_cpp. PCL on noble is 1.14; PCL headers are heavy, keep `-j2` in mind for the Pi build (deploy is the node type task).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

### S2: Ground plane fit

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/plane_fit.hpp`, `nodes/ground_estimator_cpp/src/core/plane_fit.cpp`, `nodes/ground_estimator_cpp/test/core/test_plane_fit.cpp`, `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: f2211bceb72f6f5fad1e168cf00d5a71870f8796

#### Requirement

- `include/ground_estimator_cpp/plane_fit.hpp`, `src/core/plane_fit.cpp` (design D2): `struct Plane { Eigen::Vector3d normal; double d; }` (unit normal pointing up, `normal.dot(p) + d = 0`), `double height_above(const Plane &, const Eigen::Vector3d &)` (signed, > 0 above), `double height_at_origin(const Plane &)` (plane z at base_link x=y=0); `enum class FitReject { too_few_points, no_plane, normal_off_prior, offset_off_prior, too_few_inliers, low_inlier_ratio }`, `const char *to_string(FitReject)`; `struct FitResult { std::optional<Plane> plane; std::optional<FitReject> reject; std::size_t candidates; std::size_t inliers; double inlier_ratio; }`; `FitResult fit_plane(const pcl::PointCloud<pcl::PointXYZ> &points_base, const Eigen::Vector3d &prior_up, const FitConfig &cfg, unsigned seed)`.
- The fit keeps candidate points with |z| <= `fit.candidate_band_m` and horizontal range <= `fit.max_range_m`, then runs `pcl::SACSegmentation<pcl::PointXYZ>` with `SACMODEL_PERPENDICULAR_PLANE`, `SAC_RANSAC`, axis = `prior_up`, eps angle `fit.max_normal_angle_deg`, distance `fit.distance_m`, `fit.max_iterations`, the given seed (deterministic). The normal is oriented so `normal.dot(prior_up) > 0`. Validity gates in this order, each returning its `FitReject`: fewer candidates than `fit.min_inliers` (too_few_points), no model (no_plane), angle to prior > max (normal_off_prior), |height_at_origin| > `fit.max_offset_m` (offset_off_prior), inliers < `fit.min_inliers` (too_few_inliers), inliers / candidates < `fit.min_inlier_ratio` (low_inlier_ratio).
- `test/core/synthetic_floor.hpp` (part `ground-synthetic-scene`, header-only, used by later tests): deterministic generators in base_link: a floor plane with given tilt (roll, pitch) and height, Gaussian noise, a uniform depth-like bias along the ray from a sensor origin (the ToF long bias), a box (obstacle) at a position and size, a step-up region (flat top at a height over a half-plane), a drop-off (lower floor beyond an edge line), a wall, a table top; points on an image-like grid of a 240x180 sensor footprint subsampled like decimation 4.
- Tests first in `test/core/test_plane_fit.cpp`, seen failing: level floor -> height ~0, normal ~+z (within 0.5 deg, 5 mm); 5 deg pitched floor with a matching prior -> found, angle within 0.5 deg; same floor with a level prior -> found (inside 10 deg); a wall-only cloud -> normal_off_prior or no_plane; a table top at +0.3 m only -> offset_off_prior or too_few_points (outside the band); floor with a box covering half the view -> still found; floor biased by 30 mm along the rays -> plane found and the reported height reflects the bias (within the gate); empty and 10-point clouds -> too_few_points; same input and seed -> identical output.

#### Acceptance conditions

- a1: fit_plane finds level, tilted, partially occluded and biased floors within the stated tolerances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: walls, table tops and sparse clouds are rejected with the named FitReject reason [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the fit is deterministic for a given seed [evidence: symbol: nodes/ground_estimator_cpp/src/core/plane_fit.cpp::fit_plane]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/plane_fit.hpp`
- `nodes/ground_estimator_cpp/src/core/plane_fit.cpp`
- `nodes/ground_estimator_cpp/test/core/test_plane_fit.cpp`
- `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`

#### Interfaces

`plane_fit.hpp` (part `ground-plane-fit`), `synthetic_floor.hpp` (part `ground-synthetic-scene`).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_plane_fit.

#### Steps

- write synthetic_floor.hpp and test_plane_fit.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement plane_fit.cpp until PASS
- commit `feat(ground_estimator_cpp): plane fit with gravity prior`.

#### Notes

PCL `SACSegmentation::setAxis` takes `Eigen::Vector3f`; `setEpsAngle` is radians. `pcl::console::setVerbosityLevel(L_ERROR)` keeps PCL quiet in tests and the node.

### S3: Ground band segmentation

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/segmentation.hpp`, `nodes/ground_estimator_cpp/src/core/segmentation.cpp`, `nodes/ground_estimator_cpp/test/core/test_segmentation.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: d09d5f9132552041e1b9843316fc283707cda304

#### Requirement

- `include/ground_estimator_cpp/segmentation.hpp`, `src/core/segmentation.cpp` (design D4): candidate selection only, no traversability decision (that is ground-traversability-rule). `enum class Band : uint8_t { floor, above, below, none }`; `Band classify(double height, double range, const BandsConfig &, double floor_distance_m)` (floor: |h| <= fit.distance_m; above: fit.distance_m < h <= bands.obstacle_max_m; below: -bands.drop_max_m <= h < -fit.distance_m; none otherwise or range > bands.max_range_m); `struct Segmented { pcl::PointCloud<pcl::PointXYZ> above, below, floor; std::vector<float> above_height, below_height; }` (signed heights parallel to the clouds); `Segmented segment(const pcl::PointCloud<pcl::PointXYZ> &points_base, const Plane &, const BandsConfig &, double floor_distance_m)`; `sensor_msgs::msg::PointCloud2 to_cloud_msg(const pcl::PointCloud<pcl::PointXYZ> &, const std::string &frame, const builtin_interfaces::msg::Time &stamp)` (XYZ float32, dense, an empty cloud is a valid message with width 0).
- Non-finite input points are skipped. Range is horizontal distance from the base_link origin. The output clouds keep the input points' base_link coordinates (x, y, z unchanged); "plane-relative" means only that points are selected by bands of their signed height above the fitted plane (the heights are returned separately).
- Tests first in `test/core/test_segmentation.cpp` (using `synthetic_floor.hpp`), seen failing: level floor with a 10 cm box -> box points in above with heights ~0.10, floor points in floor, nothing in below; a 10 cm drop-off beyond 1 m -> lower floor points in below with heights ~-0.10; a 3 cm sill -> above with heights ~0.03; floor biased 30 mm along the rays with the plane fitted by `fit_plane` on the same cloud -> no floor point lands in below (the plane absorbs the bias), while the raw base_link z of the same floor points reaches below -0.02 m (the margin the fixed bands lose); points beyond bands.max_range_m -> none; 0.7 m overhang -> none; NaN points skipped; to_cloud_msg round trip (fields x,y,z float32, frame, stamp, width) and the empty cloud.

#### Acceptance conditions

- a1: floor, above and below candidate points are split by signed plane height with the configured bands and range [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a 30 mm ToF bias produces no below points relative to the fitted plane [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: to_cloud_msg builds XYZ float32 clouds including the empty cloud [evidence: symbol: nodes/ground_estimator_cpp/src/core/segmentation.cpp::to_cloud_msg]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/segmentation.hpp`
- `nodes/ground_estimator_cpp/src/core/segmentation.cpp`
- `nodes/ground_estimator_cpp/test/core/test_segmentation.cpp`

#### Interfaces

`segmentation.hpp` (part `ground-segmentation`): `Band`, `classify`, `Segmented`, `segment`, `to_cloud_msg`.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_segmentation.

#### Steps

- write test_segmentation.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): plane-relative obstacle and drop bands`.

#### Notes

`pcl_conversions::toROSMsg` builds the PointCloud2; the fixed bands for comparison are in nodes/nav2_bringup/config/nav2_params.yaml (tof_mark 0.06..0.6, tof_cliff_mark -1.0..-0.10).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

### S4: Ground plane tracker

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/tracker.hpp`, `nodes/ground_estimator_cpp/src/core/tracker.cpp`, `nodes/ground_estimator_cpp/test/core/test_tracker.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 196263e30c70e3b317430eb44bb0d8d0a36b13ac

#### Requirement

- `include/ground_estimator_cpp/tracker.hpp`, `src/core/tracker.cpp` (design D2 prior, D3):
  - `Eigen::Vector3d gravity_up_in_base(const Eigen::Quaterniond &imu_orientation, const Eigen::Quaterniond &imu_to_base, double level_roll_deg, double level_pitch_deg)`: the world up vector expressed in base_link from the IMU orientation (REP-103), with the level reference removed so a base standing level yields +z exactly.
  - `struct Prior { Eigen::Vector3d up; bool from_imu; }`, `Prior choose_prior(std::optional<ImuSample> latest, double now_s, const ImuConfig &)`: the IMU up when the last sample is younger than `imu.timeout_s`, else base_link +z with `from_imu = false`.
  - `class PlaneTracker`: `update(const Plane &plane_base, const Eigen::Isometry3d &base_to_fixed, double stamp_s)` records a successful fit: the plane is stored in the fixed frame with EMA smoothing `fit.smoothing` (normal renormalised, d blended) in a short history (fits newer than `fit.hold_s` plus 1 s, pruned on update); `mark_failed(double stamp_s)` records a cloud that yielded no valid plane. `std::optional<TrackedPlane> current(const Eigen::Isometry3d &base_to_fixed, double stamp_s) const` selects the newest stored fit with fit stamp <= `stamp_s` (never a later fit: no look-ahead) and returns it transformed into base_link at `stamp_s` (via `base_to_fixed` of that stamp, i.e. the plane held in odom is propagated with the robot motion, and the up prior comes from the IMU at that stamp) with `age_s = stamp_s - fit_stamp`; `source` is `fit` when no failed cloud was recorded between that fit and `stamp_s`, `held` when one was (the plane is being held across failed fits). When `age_s > fit.hold_s`, or no fit at or before `stamp_s` exists, it returns nullopt: never extrapolated beyond the hold age. The same rule serves the ToF cloud stamp, the RGB-D cloud stamp (older than the newest fit) and the 2 Hz state stamp (now).
  - `struct Slope { double total_deg, roll_deg, pitch_deg; Level level; }`, `enum class Level { ok, warning, error }`, `Slope slope_vs_gravity(const Plane &plane_base, const Eigen::Vector3d &gravity_up_base, const SlopeConfig &)`: the floor tilt ahead as base roll/pitch would be on that plane (REP-103 signs as tilt_guard: roll > 0 left side up, pitch > 0 nose down), level = the worse axis vs the warn/error limits (tilt_guard semantics, no hysteresis here: the ground estimate is a report).
  - `double plane_vs_base_deg(const Plane &plane_base)`: angle between the plane normal and base_link +z; `double base_tilt_deg(const Eigen::Vector3d &gravity_up_base)`: angle between gravity up and base_link +z.
- Tests first in `test/core/test_tracker.cpp`, seen failing: identity orientation with zero level reference -> +z; the 0.7/1.3 deg level reference applied to an IMU reading exactly that -> +z within 1e-9; IMU pitched 10 deg -> up vector pitched by 10 deg with the REP-103 sign; stale IMU -> base_link +z and from_imu false; tracker holds a plane up to hold_s then returns nullopt; a query stamp between two fits gets the earlier fit with source fit and age = stamp - fit stamp; a query older than every stored fit gets nullopt; a failed cloud between the fit and the query makes the source held; history pruning keeps hold_s + 1 s; a plane stored while the base moved 0.5 m forward and yawed 30 deg in odom comes back with the same height in the new base_link; EMA blends two planes as configured (smoothing 1 = latest only); slope of a 5 deg pitched plane on a level robot = 5 deg pitch with the right sign; level ok/warning/error at the limits for roll and pitch; plane_vs_base for a 3 deg tilted plane = 3 deg.

#### Acceptance conditions

- a1: the IMU prior removes the level reference, follows REP-103 signs and falls back to base_link +z when stale [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: current() returns the newest fit at or before the stamp with source fit or held and age = stamp - fit stamp, nullopt beyond hold_s or before the first fit, never a later fit [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: slope, slope level against the tilt_guard limits, plane_vs_base and base tilt are computed as specified [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/tracker.hpp`
- `nodes/ground_estimator_cpp/src/core/tracker.cpp`
- `nodes/ground_estimator_cpp/test/core/test_tracker.cpp`

#### Interfaces

`tracker.hpp` (part `ground-tracker`): `gravity_up_in_base`, `choose_prior`, `PlaneTracker` (`update`, `mark_failed`, `current`), `TrackedPlane {Plane plane; std::string source; double age_s;}`, `slope_vs_gravity`, `plane_vs_base_deg`, `base_tilt_deg`, `Level`, `to_string(Level)`.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_tracker.

#### Steps

- write test_tracker.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): plane tracker, IMU prior and slope level`.

#### Notes

tilt_guard (nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py) defines the roll/pitch convention and the level reference handling; mirror its signs. The limits default to the tilt_guard defaults (8/15 roll, 12/20 pitch).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

### S5: Ground colour labels

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp`, `nodes/ground_estimator_cpp/src/core/colour.cpp`, `nodes/ground_estimator_cpp/test/core/test_colour.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 231ffa0da147e2009a845fb45c76d84b2a4cb555

#### Requirement

- `include/ground_estimator_cpp/colour.hpp`, `src/core/colour.cpp` (design D6): `enum class Label : uint8_t { floor = 0, above_floor = 1, below_floor = 2, floor_colour_mismatch = 3, unlabelled = 4 }`; `class FloorColourModel` with `update(const std::vector<cv::Vec3b> &rgb_floor_samples)` (converted to CIELAB with `cv::cvtColor`, mean and 3x3 covariance blended with weight `colour.forgetting`, total sample count kept), `bool ready() const` (>= `colour.min_floor_samples`), `double mahalanobis(const cv::Vec3b &rgb) const`; `struct LabelStats { std::size_t floor, above, below, mismatch, unlabelled; double mismatch_ratio; }` (mismatch_ratio = mismatch / (floor + mismatch), 0 points -> ratio not reported: std::optional); `LabelStats label_cloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_base, const Plane &, const BandsConfig &, const ColourConfig &, double floor_distance_m, FloorColourModel &model, std::vector<Label> &labels)`: band per point via `classify` (ground-segmentation: floor, above, below, none; the traversability class is decided by ground-traversability-rule, not by labels); floor-band points first update the model, then a floor-band point with model ready and mahalanobis > `colour.max_mahalanobis` is floor_colour_mismatch; before the model is ready floor-band points stay floor; `mismatch_points(cloud_base, labels, plane)` returns the floor_colour_mismatch points with their heights (input of `colour_warning_areas`); `sensor_msgs::msg::PointCloud2 to_labelled_msg(const pcl::PointCloud<pcl::PointXYZRGB> &, const std::vector<Label> &, frame, stamp)`: fields x, y, z (float32), rgb (float32 packed, as head_rgbd_cpp's cloud), label (uint8).
- `colour.enabled: false` makes label_cloud return geometric labels only (no mismatch, model untouched).
- Tests first in `test/core/test_colour.cpp`, seen failing: a grey floor with a red A4 sheet lying flat -> sheet points floor_colour_mismatch, the rest floor, ratio ~ sheet share; a grey box -> above_floor (geometry wins); mismatch_points returns exactly the sheet points; before min_floor_samples nothing is mismatch; forgetting: the model follows a floor colour change within 1/forgetting updates; Lab conversion of pure white/black/grey gives L 100/0/~53; labelled message fields, offsets and label values; colour.enabled false.

#### Acceptance conditions

- a1: geometric floor points with a non-floor colour are labelled floor_colour_mismatch once the model is ready; geometry decides above_floor and below_floor [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the CIELAB floor model updates with forgetting and reports Mahalanobis distances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the labelled PointCloud2 carries x, y, z, rgb and a uint8 label field with values 0..4 [evidence: symbol: nodes/ground_estimator_cpp/src/core/colour.cpp::to_labelled_msg]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp`
- `nodes/ground_estimator_cpp/src/core/colour.cpp`
- `nodes/ground_estimator_cpp/test/core/test_colour.cpp`

#### Interfaces

`colour.hpp` (part `ground-colour-labels`): `Label`, `FloorColourModel`, `LabelStats`, `label_cloud`, `mismatch_points`, `to_labelled_msg`. The label values 0..4 (floor, above_floor, below_floor, floor_colour_mismatch, unlabelled) are a contract read by web_ui (design D6).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_colour.

#### Steps

- write test_colour.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): RGB-D floor colour labels`.

#### Ruling

- 2026-10-20 accepted: Verifier crashed (--prev-head unknown); manual verify at eac98687: clean tree, scripts/cpp-node-test.sh ground_estimator_cpp 80 tests 0 failures PASS. TDD process gap: test and implementation landed in one commit (0343d346); tests are new for a new module and cannot build without it.

#### Notes

head_rgbd_cpp packs rgb as float32 (`nodes/head_rgbd_cpp/src/core/cloud.cpp`); pcl::PointXYZRGB matches that layout via pcl_conversions::fromROSMsg.

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

### S6: Ground step edges

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp`, `nodes/ground_estimator_cpp/src/core/edges.cpp`, `nodes/ground_estimator_cpp/test/core/test_edges.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 33a8ca65e0943cda4be2b304593ae60eb5da191c

#### Requirement

- `include/ground_estimator_cpp/edges.hpp`, `src/core/edges.cpp` (design D5, D14 features): `struct Cluster { std::string kind; pcl::PointCloud<pcl::PointXYZ> points; std::vector<float> heights; double median_height_m; double edge_step_m; double slope_deg; std::vector<Eigen::Vector2d> hull; Eigen::Vector2d edge_point, edge_direction; std::string side; double distance_m; }` (`kind` `step` for above-floor clusters, `drop` for below-floor clusters); `std::vector<Cluster> extract_clusters(const pcl::PointCloud<pcl::PointXYZ> &points, const std::vector<float> &heights, const std::string &kind, const Plane &, const EdgesConfig &, double edge_band_m)`; `struct Region { std::string name, kind, cls; double height_m; Eigen::Vector2d point, direction; std::string side; double distance_m; std::size_t points; }` (`cls` set later by ground-traversability-rule, `obstacle` or `warning`); `std::vector<Region> regions_from_clusters(const std::vector<Cluster> &, const EdgesConfig &)`.
- Clustering: `pcl::EuclideanClusterExtraction` (tolerance `edges.cluster_tolerance_m`, min `edges.min_cluster_points`) on `Segmented::above` and on `Segmented::below`; points in no cluster are returned separately (`unclustered` indices) so the rule can treat them conservatively.
- Features per cluster: edge line = direction from the cluster's principal axis in xy (PCA), placed through the cluster points nearest the base_link origin along the axis normal (the boundary facing the robot, the 10th percentile of the signed normal offset); `side` so the cluster lies on that side and the origin on the other; `distance_m` = origin to line; `edge_step_m` = 90th percentile of |height| of the cluster points within `edge_band_m` of the edge line (the step the wheel meets: ~H for a box or a drop of depth H, ~0 at the foot of a ramp); `slope_deg` = angle between the normal of a least-squares plane through the cluster points (Eigen PCA, smallest eigenvector) and the floor plane normal (~90 for a vertical face, the ramp angle for a ramp, ~0 for a flat top); `hull` = 2D convex hull in base_link xy (counter-clockwise).
- Regions (mcp terrain): drop clusters, and step clusters with |median| <= `edges.step_max_height_m` and height spread (p90 - p10) <= `edges.step_flatness_m`; height_m = median; names `drop_<i>` / `step_<i>` numbered by distance; at most `edges.max_regions`, nearest first.
- Tests first in `test/core/test_edges.cpp` (synthetic_floor.hpp + segment + fit_plane), seen failing: a straight 15 cm drop-off 1.0 m ahead across the robot -> one drop cluster/region, height ~-0.15 (+-1 cm), edge_step ~0.15, line ~x = 1.0 (+-5 cm), direction parallel to y, side such that a point at x = 1.5 is inside and the origin outside (gtest `SideMatchesMcpHalfPlaneEdge`, the same left/right rule as mcp surfaces.py: left = cross(direction, p - point) > 0); a drop-off rotated 30 deg -> direction rotated 30 deg (+-3 deg); a flat plate 60 cm wide, 30 cm deep and 8 cm high -> step region height ~0.08, edge_step ~0.08, slope ~0; a 10 deg ramp starting at the floor -> edge_step < 0.02 and slope 10 deg (+-1.5); a 30 cm box -> cluster with edge_step ~0.3, no region; hull contains the cluster points; noise below min_cluster_points -> unclustered, no cluster; three drops -> regions nearest first and capped at max_regions.

#### Acceptance conditions

- a1: above and below candidates become clusters with edge line, side, distance, edge_step, slope and hull within the stated tolerances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: drop-offs and flat low steps become half-plane regions; noise stays unclustered; regions are nearest first and capped [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the side convention equals mcp HalfPlaneEdge, left = cross(direction, p - point) > 0 (gtest SideMatchesMcpHalfPlaneEdge) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp`
- `nodes/ground_estimator_cpp/src/core/edges.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edges.cpp`

#### Interfaces

`edges.hpp` (part `ground-edges`): `Cluster`, `extract_clusters`, `Region`, `regions_from_clusters`. The Region fields map 1:1 to mcp `SurfaceRegion` (`name`, `height_m`, `frame: base_link`, `edge {point, direction, side}`) plus `kind`, `cls`, `distance_m`, `points`.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_edges.

#### Steps

- write test_edges.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): step and drop-off edge regions`.

#### Notes

mcp SurfaceRegion/HalfPlaneEdge: nodes/mcp_server/mcp_server/surfaces.py:46-118 (side semantics in the HalfPlaneEdge docstring).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

### S7: Ground traversability rule

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`, `nodes/ground_estimator_cpp/src/core/traversability.cpp`, `nodes/ground_estimator_cpp/test/core/test_traversability.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: ce708ba250b3bcda1ee3cffe0369d740f1c2a015

#### Requirement

- `include/ground_estimator_cpp/traversability.hpp`, `src/core/traversability.cpp` (design D14, user rule at the gate): `enum class Cls { obstacle, warning }`; `struct Verdict { Cls cls; std::string reason; }` (reason `step_height`, `slope`, or `traversable`); `Verdict classify_cluster(double edge_step_m, double slope_deg, const TraversabilityConfig &)`: obstacle when `edge_step_m > max_step_m()` (one third of the wheel diameter, 0.04 m for the 0.06 m wheel radius) or `slope_deg > traversability.max_slope_deg` (8 deg), else warning (clear to ride through, slow down). Exactly at the threshold is a warning.
- `struct WarningArea { std::string kind; std::vector<Eigen::Vector2d> polygon; double height_m; double slope_deg; std::string source; }` (`kind` `step`, `drop`, `ramp` (a warning cluster with slope_deg >= 2), or `colour`; `source` `tof` or `colour`; polygon = the cluster hull in base_link); `struct Traversability { pcl::PointCloud<pcl::PointXYZ> obstacle, drop; std::vector<WarningArea> warnings; std::vector<Region> regions; }`; `Traversability apply_rule(const std::vector<Cluster> &above, const std::vector<Cluster> &below, const pcl::PointCloud<pcl::PointXYZ> &unclustered_above, const std::vector<float> &unclustered_above_h, const pcl::PointCloud<pcl::PointXYZ> &unclustered_below, const std::vector<float> &unclustered_below_h, std::vector<Region> regions, const TraversabilityConfig &)`: lethal clouds = points of obstacle clusters plus unclustered points with |h| > max_step_m (conservative: an isolated tall return still marks); warning clusters give WarningAreas and none of their points reach the lethal clouds; each region gets `cls` from its cluster.
- `std::vector<WarningArea> colour_warning_areas(const pcl::PointCloud<pcl::PointXYZ> &mismatch_points_base, const std::vector<float> &heights, const Plane &, const EdgesConfig &, const TraversabilityConfig &)`: floor_colour_mismatch points (ground-colour-labels) clustered like the ToF candidates; each cluster classified by the same `classify_cluster` rule on its edge step and slope (flat objects are warnings, kind `colour`, source `colour`; a colour cluster that fails the rule is reported as an obstacle area with kind `colour` and is added to the lethal obstacle cloud by the node).
- Tests first in `test/core/test_traversability.cpp` (synthetic_floor.hpp, segment, extract_clusters), both sides of every threshold, seen failing: sill 0.035 m -> warning `step`, sill 0.045 m -> obstacle (points in the obstacle cloud); drop 0.035 m -> warning `drop`, drop 0.045 m -> obstacle (points in the drop cloud); ramp 6 deg -> warning `ramp` with no lethal points although it rises above 0.04 m, ramp 10 deg -> obstacle by `slope`; 10 cm box -> obstacle; `classify_cluster` at exactly 0.04 / 8.0 -> warning, 0.0401 / 8.01 -> obstacle; wheel radius 0.09 -> a 0.05 m sill is a warning; a flat red sheet (colour mismatch points at h ~0) -> warning area kind `colour`; a 6 cm colour-flagged plate -> obstacle; an isolated unclustered return at 0.10 m -> in the obstacle cloud, one at 0.03 m -> nowhere; regions carry cls.

#### Acceptance conditions

- a1: steps and drops higher than one third of the wheel diameter and slopes steeper than max_slope_deg are obstacles in the lethal clouds; lower or gentler ones are warning areas with no lethal points, tested on both sides of each threshold [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: colour-flagged flat objects follow the same rule (flat sheet warning, 6 cm plate obstacle) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the threshold derives from the configured wheel radius (0.09 m radius makes a 0.05 m sill a warning) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/test/core/test_traversability.cpp`

#### Interfaces

`traversability.hpp` (part `ground-traversability`): `Cls`, `Verdict`, `classify_cluster`, `WarningArea`, `Traversability`, `apply_rule`, `colour_warning_areas`.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_traversability.

#### Steps

- write test_traversability.cpp with both sides of every threshold
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement traversability.cpp until PASS
- commit `feat(ground_estimator_cpp): obstacle vs warning traversability rule`

#### Notes

tilt_guard warn limits roll 8 / pitch 12 deg: the 8 deg default is the lower one because a swerve base can meet a ramp at any heading (design D14, my call). swerve_drive_controller `wheel_radius_m` 0.06 (client.yml).

### S8: Ground state JSON

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp`, `nodes/ground_estimator_cpp/src/core/state.cpp`, `nodes/ground_estimator_cpp/test/core/test_state.cpp`, `nodes/ground_estimator_cpp/test/data/state_example.json`, `nodes/ground_estimator_cpp/test/data/warning_areas_example.json`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: f1e3584b99342420bfd7bef009d0cf31e83df921

#### Requirement

- `include/ground_estimator_cpp/state.hpp`, `src/core/state.cpp` (design D7, contract `ground-state`): `inline constexpr std::array<std::string_view, N> STATE_KEYS` in this order: `stamp_ns, frame, prior, imu_ok, plane, reason, slope_deg, slope_roll_deg, slope_pitch_deg, slope_level, base_tilt_deg, plane_vs_base_deg, traversability, obstacle_points, drop_points, regions, warnings, colour`; `struct StateInputs` (stamp_ns, frame, Prior, imu_ok, optional TrackedPlane plus inliers and inlier_ratio, optional FitReject or "no_tf"/"no_cloud" reason string, optional Slope, base_tilt_deg, optional plane_vs_base_deg, obstacle/drop counts, regions with cls, warning areas, TraversabilityConfig, optional LabelStats with colour model readiness, sample count and stamp); `nlohmann::json build_state(const StateInputs &)` and `std::string to_state_string(const StateInputs &)` (compact dump, key order = STATE_KEYS, nlohmann::ordered_json).
- Shapes: `plane` = `{normal: [x,y,z], height_m, inliers, inlier_ratio, source: "fit"|"held", age_s}` or null; `reason` null when the plane is present, else the FitReject string or `no_tf` / `no_cloud`; `slope_*`, `slope_level` (`ok|warning|error`), `plane_vs_base_deg` null without a plane; `prior` `imu|base_link`; `traversability` = `{max_step_m, max_slope_deg}` (the configured rule, always present); `regions` a list of `{name, kind, class: "obstacle"|"warning", height_m, frame: "base_link", edge: {point: [x,y], direction: [x,y], side}, distance_m, points}`; `warnings` a list of `{kind: "step"|"drop"|"ramp"|"colour", polygon: [[x,y], ...], height_m, slope_deg, source: "tof"|"colour"}` in base_link; `colour` `{model_ok, floor_samples, mismatch_ratio|null, stamp_ns}` or null when colour is disabled or no RGB-D cloud was labelled yet; floats rounded to 4 decimals. Without a plane, counts are 0 and regions and warnings empty (the absence is stated by `plane: null` and `reason`, no fabricated numbers).
- `std::string to_warning_areas_string(int64_t stamp_ns, const std::string &frame, const std::vector<WarningArea> &areas_in_frame)` (contract `ground-warning-areas`, consumed by costmap_zones): `{stamp_ns, frame, areas: [{kind, polygon: [[x, y], ...], height_m, slope_deg, source}]}`, key order fixed, floats rounded to 4 decimals; an empty `areas` list is valid (real "no warning" state).
- Tests first in `test/core/test_state.cpp`, seen failing: key order equals STATE_KEYS for a full and a plane-less state; `PlaneLessStateHasNullsAndZeroCounts` (plane, slope_deg, slope_roll_deg, slope_pitch_deg, slope_level, plane_vs_base_deg are null; obstacle_points and drop_points are 0; regions is empty; reason is set); region and colour shapes; rounding; the JSON parses back with nlohmann; regions carry class and warnings their shape; to_warning_areas_string key order and an empty list; `test/data/warning_areas_example.json` equals to_warning_areas_string of the test inputs (contract fixture for costmap_zones).
- `nodes/ground_estimator_cpp/test/data/state_example.json`: one full example produced by the test inputs (checked by the test to equal `to_state_string`), used by mcp_server and web_ui tests as the contract fixture.

#### Acceptance conditions

- a1: the state JSON has exactly the STATE_KEYS in order with the specified shapes for full and plane-less states [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the contract fixture test/data/state_example.json equals to_state_string of the test inputs [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: a plane-less state carries no fabricated numbers (gtest PlaneLessStateHasNullsAndZeroCounts) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

- a4: the warning areas JSON has the fixed shape and its fixture test/data/warning_areas_example.json matches [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp`
- `nodes/ground_estimator_cpp/src/core/state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_state.cpp`
- `nodes/ground_estimator_cpp/test/data/state_example.json`

#### Interfaces

`state.hpp` (part `ground-state`): `STATE_KEYS`, `StateInputs`, `build_state`, `to_state_string`; fixture `test/data/state_example.json`; `to_warning_areas_string` and fixture `test/data/warning_areas_example.json` (part `ground-warning-areas`, consumed by costmap_zones).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_state.

#### Steps

- write test_state.cpp and the fixture
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): ground state JSON contract`.

#### Notes

`/tilt_guard/state` (tilt_guard.py STATE_KEYS) is the pattern of a JSON state topic with a fixed key order.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-colour-labels

### S9: Ground node wiring

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`, `nodes/ground_estimator_cpp/src/ground_node.cpp`, `nodes/ground_estimator_cpp/src/main.cpp`, `nodes/ground_estimator_cpp/test/test_node.cpp`, `nodes/ground_estimator_cpp/CMakeLists.txt`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 908cf51a4bb5f60a586343e5c8ee809b5bd99fb2

#### Requirement

- `include/ground_estimator_cpp/ground_node.hpp`, `src/ground_node.cpp`, `src/main.cpp` (design D1, D3, D4, D6, D7, D11): `GroundNode` (ROS node name `ground_estimator`, executable `ground_estimator_node`, single-threaded executor), wired in CMakeLists as `ground_estimator_node_lib` + `ground_estimator_node` (installed to lib/ground_estimator_cpp), plus `test_node` gtest.
- Subscriptions: `cloud_topic` (PointCloud2; QoS reliable depth 1, compatible with the laser_filter publisher), `imu_topic` (sensor_msgs/Imu, sensor-data QoS; orientation used only when its covariance[0] != -1 and the quaternion is finite and normalised), `rgbd_cloud_topic` (PointCloud2 XYZRGB, best_effort depth 1; only when colour.enabled). TF: `tf2_ros::Buffer` + listener; cloud frame -> base_frame at the cloud stamp and base_frame -> fixed_frame at the same stamp with `tf_timeout_s`; the IMU frame -> base_frame static rotation.
- Per ToF cloud (clouds keep base_link coordinates; "plane-relative" = selected by the plane-relative bands): TF failure -> drop (`ground_clouds_dropped_total{reason="tf"}`), nothing published for it, state reason `no_tf` at the next state publish. Otherwise prior (`choose_prior`), `fit_plane` (seed fixed per process), tracker update on success, `mark_failed` on a failed fit, `current(stamp)` plane (fit or held, PlaneTracker stamp rule); with a plane: `segment`, `extract_clusters` (above and below), `regions_from_clusters`, `apply_rule` (design D14: one third of the wheel diameter, max slope), then publish `<ns>/obstacle_points` and `<ns>/drop_points` = the lethal clouds of `apply_rule` only (frame base_frame, the cloud stamp; reliable depth 1; empty clouds allowed; warning-class points never reach them), and `<ns>/warning_areas` (std_msgs/String `to_warning_areas_string`, reliable depth 1) with the ToF warning areas plus the colour warning areas of the latest labelled RGB-D cloud (dropped when older than fit.hold_s), polygons transformed from base_frame to `warnings.frame` (map) at the cloud stamp - no map TF: no warning message for that cloud, counted `reason="no_map_tf"`; colour clusters that fail the rule add their points to the obstacle cloud; slope and cross-check values; without a plane: publish no cloud.
- Per RGB-D cloud: TF to base_frame at its stamp; with `current(rgbd stamp)` (the newest fit at or before that stamp within hold_s, PlaneTracker stamp rule): `label_cloud`, `mismatch_points` -> `colour_warning_areas` (kept for the next ToF cycles), publish `<ns>/labelled_points` (best_effort depth 1, base_frame, the RGB-D stamp); without a plane or TF: nothing published, counted.
- `<ns>/state` (std_msgs/String, reliable + transient_local depth 1): `to_state_string` at `state.rate_hz` (plane from `current(now)`, the same stamp rule: no plane beyond hold_s) and immediately when the plane validity changes; never before the first ToF cloud was processed (no state from no data).
- Metrics (ros2_common_cpp::Metrics on `metrics_port`, part `ground-metrics`): `ground_fits_total{result}` (result `ok` or the FitReject string), `ground_fit_seconds` histogram (fit + segmentation per cloud), `ground_inlier_ratio`, `ground_slope_deg`, `ground_plane_vs_base_deg`, `ground_regions`, `ground_warning_areas`, `ground_obstacle_points`, `ground_drop_points`, `ground_colour_mismatch_ratio` gauges (set only from real values), `ground_clouds_dropped_total{reason}` (`tf`, `no_plane_rgbd`, `no_map_tf`), `ground_labelled_clouds_total`, plus `robot_node_info` and the process collector.
- `test/test_node.cpp` (in-process like nodes/head_rgbd_cpp/test/test_node.cpp: probe publishers, a tf2 buffer fed with static and timed transforms, synthetic_floor.hpp clouds): a level floor with a box -> obstacle_points has the box, drop_points empty, state plane height ~0 and reason null; a 10 cm drop-off scene -> drop_points and a drop region with class obstacle in the state; a 3 cm sill -> no obstacle points, one warning area of kind step on warning_areas in the map frame (map->odom->base_link TF fed) and in the state; a 6 deg ramp -> warning kind ramp, no lethal points; no map TF -> no warning_areas message and the counter; a wall-only cloud -> no obstacle/drop message, state plane null with reason normal_off_prior or no_plane (after hold_s); missing TF -> no output and the tf drop counter; IMU stale -> state prior base_link; an RGB-D cloud with a red sheet -> labelled_points with label 3 points and a colour warning area in the next warning_areas message; no state before the first cloud.

#### Acceptance conditions

- a1: with a valid plane the node publishes plane-relative obstacle and drop clouds and the state with regions and slope [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: without a valid plane, TF or input the node publishes no cloud and the state says plane null with the reason; no state before the first cloud [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: RGB-D clouds are labelled against the tracked plane and published on labelled_points [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: the metrics listed in the requirement are registered [evidence: symbol: nodes/ground_estimator_cpp/src/ground_node.cpp::GroundNode]

- a5: only obstacle-class points reach the lethal clouds; warning areas (ToF and colour) are published in the map frame on warning_areas [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/src/main.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/CMakeLists.txt`

#### Interfaces

`ground_node.hpp` `GroundNode`; topics `/ground/obstacle_points`, `/ground/drop_points`, `/ground/labelled_points`, `/ground/state` (part `ground-topics`); metric names (part `ground-metrics`).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS including test_node.

#### Steps

- write test/test_node.cpp and the CMake test target
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement the node and main until PASS
- commit `feat(ground_estimator_cpp): ground estimator node wiring`.

#### Notes

copy the TF/metrics/QoS patterns of nodes/head_rgbd_cpp/src/head_rgbd_node.cpp. Keep the 10 Hz path allocation-light (reuse clouds). `/imu/data` frame is `imu_link` (client.yml bno055_imu config).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-state-json

### S10: Ground node README

- Status: done
- Files: `nodes/ground_estimator_cpp/README.md`, `tests/test_ground_estimator_readme.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_estimator_readme.py -q && uv run poe lint`
- Verified head: dbfc3bd147f313f89a5c4680819853df4cc87553

#### Requirement

- `nodes/ground_estimator_cpp/README.md` (design D7, D12): purpose; layout (core library, node, tests); `## Topics` table (direction, topic, type, QoS, frame, notes) for `/tof/points_filtered`, `/imu/data`, `/head_rgbd/points`, `/ground/obstacle_points`, `/ground/drop_points`, `/ground/labelled_points`, `/ground/state`, `/ground/warning_areas`, stating that the output clouds keep base_link coordinates and are selected by plane-relative bands; `## State` with every STATE_KEYS key, meaning and unit (contract `ground-state`); `## Labels` with values 0..4; `## Obstacle or warning` (the rule: a step, drop or ramp is an obstacle when its edge step exceeds one third of the wheel diameter or its slope exceeds max_slope_deg, else a warning area that costmap_zones turns into a slow-down; colour-flagged flat objects follow the same rule); `## Configuration` table with every key of `config/ground_estimator.yaml`, default and meaning; `## Metrics` with every metric name registered in `src/ground_node.cpp`; `## Plane tracking` (IMU prior, level reference, hold_s and the PlaneTracker stamp rule: newest fit at or before the stamp, source fit/held, no estimate beyond hold_s); `## No placeholder data` (no cloud without a plane, plane null with reason in the state, no state before the first cloud).
- Test first: new `tests/test_ground_estimator_readme.py`, seen failing: the README has the sections above (including Obstacle or warning); every key of the shipped yaml (nested keys dotted), every STATE_KEYS entry parsed from `include/ground_estimator_cpp/state.hpp`, every `/ground/` output topic and every metric name string in `src/ground_node.cpp` (`ground_*`) appears in the README.
- `tests/README.md`: the new test file described.

#### Acceptance conditions

- a1: the README has the Topics, State, Labels, Obstacle or warning, Configuration, Metrics, Plane tracking and No placeholder data sections [evidence: test: tests/test_ground_estimator_readme.py]
- a2: every config key, state key, output topic and metric name of the node appears in the README [evidence: test: tests/test_ground_estimator_readme.py]
- a3: tests/README.md lists the new test [evidence: command: grep -q test_ground_estimator_readme.py tests/README.md]

#### Files

- `nodes/ground_estimator_cpp/README.md`
- `tests/test_ground_estimator_readme.py`
- `tests/README.md`

#### Interfaces

`nodes/ground_estimator_cpp/README.md` sections Topics, State, Labels, Configuration, Metrics, Plane tracking, No placeholder data; consumes parts `ground-topics`, `ground-state`, `ground-metrics`, `ground-config`.

#### Expected outcome

`uv run pytest tests/test_ground_estimator_readme.py -q` passes.

#### Steps

- write tests/test_ground_estimator_readme.py
- run `uv run pytest tests/test_ground_estimator_readme.py -q`, see FAIL
- write the README until green, add the tests/README entry
- commit `docs(ground_estimator_cpp): node README`

#### Notes

Pattern: nodes/head_rgbd_cpp/README.md. Parse STATE_KEYS and metric names from the C++ sources with regexes in the test; the shipped yaml with yaml.safe_load.

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-wiring

### S11: Ground node type

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_ground_estimator_deploy.py`, `tests/test_unit_cgroups.py`, `tests/test_node_metrics_wiring.py`, `ansible/README.md`, `nodes/README.md`, `.claude/skills/ansible-deploy/SKILL.md`
- Verify: `uv run pytest tests/test_ground_estimator_deploy.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible`
- Verified head: de86d9b1ed6a4e7d13e0d2567d6e5f2fbb604f7d

#### Requirement

- client.yml node type `ground_estimator_cpp` (design D11): native, `node_launch_command: "ros2 run ground_estimator_cpp ground_estimator_node"`, `node_src_dir: ""`, `src_paths: []`, `colcon_source` with `shared/ros2_common_cpp` then `nodes/ground_estimator_cpp` (Release, BUILD_TESTING OFF, `-j2`, base_paths `.`, `stamp_apt_packages` listing the ROS/PCL/OpenCV packages), apt: build-essential, cmake, rsync, python3-colcon-common-extensions, ros-jazzy-ament-cmake, ros-jazzy-rclcpp, ros-jazzy-sensor-msgs, ros-jazzy-std-msgs, ros-jazzy-tf2-ros, ros-jazzy-tf2-eigen, ros-jazzy-pcl-conversions, libpcl-dev, libopencv-dev, libeigen3-dev, libyaml-cpp-dev, nlohmann-json3-dev, prometheus-cpp-dev (tests/test_cpp_node_deploy.py checks these cover every package.xml key); normal tier `cpu_weight: 500`, `io_weight: 500`, `oom_score_adjust: -500`, `memory_low: "48M"`, `memory_max: "256M"` (the runbook replaces them with measured p50 / 2x peak); `config_path: /etc/ros2/ground_estimator_cpp`; env `GROUND_ESTIMATOR_CONFIG=/etc/ros2/ground_estimator_cpp/config.yaml`. No `ROS_STATIC_PEERS`.
- ros2_nodes: `ground_estimator_cpp` present/enabled with the config block (defaults of `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, commented), `metrics_port: 19119`, and the Prometheus scrape entry with the `node` label (client.yml scrape list).
- `deploy_nodes_client.yml`: step `Deploy ground_estimator_cpp` (resolve_and_deploy.yml, tags `[ground_estimator_cpp, apt, build, config]`) after arducam_tof, laser_filter and head_rgbd_cpp.
- Tests first, seen failing: new `tests/test_ground_estimator_deploy.py`: node type keys, apt list, normal tier values, env, colcon source order, ros2_nodes entry, playbook step order; consistency (design D3): the ground config `cloud_topic` equals the laser_filter `cloud_filtered_topic`, `imu_topic` equals the bno055_imu topic, `rgbd_cloud_topic` equals head_rgbd_cpp `output_namespace` + `/points`, `imu.level_reference_*` equal tilt_guard `level_reference_*`, `slope.*_warn_deg` / `*_error_deg` equal the tilt_guard limits, `traversability.wheel_radius_m` equals the swerve_drive_controller `wheel_radius_m` (0.06) and the derived max step is one third of the wheel diameter, `traversability.max_slope_deg` <= the smaller tilt_guard warn limit, the client.yml config keys equal the shipped yaml keys; `tests/test_unit_cgroups.py` covers the new type in the normal tier; `tests/test_node_metrics_wiring.py` knows its metrics port 19119 (unique); `tests/test_cpp_node_deploy.py` passes for the new type.
- Docs: `ansible/README.md` node list entry and the tier rationale (feeds the local costmap); `nodes/README.md` entry; `.claude/skills/ansible-deploy/SKILL.md` lists ground_estimator_cpp among client nodes with its deploy command.

#### Acceptance conditions

- a1: the ground_estimator_cpp node type, normal tier, apt packages and ros2_nodes entry are in client.yml and the playbook deploys it in order [evidence: test: tests/test_ground_estimator_deploy.py]
- a2: the ground config is consistent with laser_filter, bno055_imu, head_rgbd_cpp, the tilt_guard level reference and limits, and the swerve wheel radius [evidence: test: tests/test_ground_estimator_deploy.py]
- a3: cgroup tier, metrics port and C++ deploy checks cover the new type [evidence: command: uv run pytest tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py -q]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe lint-ansible && uv run poe test-ansible]
- a5: ansible README, nodes README and the deploy skill list the node [evidence: command: grep -q ground_estimator_cpp ansible/README.md && grep -q ground_estimator_cpp nodes/README.md && grep -q ground_estimator_cpp .claude/skills/ansible-deploy/SKILL.md]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_ground_estimator_deploy.py`
- `tests/test_unit_cgroups.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `nodes/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

#### Interfaces

deployed unit `ros2-ground_estimator_cpp.service` (part `ground-node-unit`); ansible node type `ground_estimator_cpp`; client.yml `ros2_nodes` entry `ground_estimator_cpp`.

#### Expected outcome

`./scripts/deploy-nodes.sh client ground_estimator_cpp` builds and starts the node with its config (run by the runbook, not this task).

#### Steps

- write tests/test_ground_estimator_deploy.py and the cgroup/metrics test changes
- run `uv run pytest tests/test_ground_estimator_deploy.py -q`, see it fail
- edit client.yml and the playbook until the verify command is green
- update the READMEs and the deploy skill
- commit `feat(ansible): ground_estimator_cpp node type`.

#### Notes

pattern: the head_rgbd_cpp node type in client.yml (around line 599) and tests/test_head_rgbd_deploy.py. Other epics (head-and-arm-calibration-fro, servo-torque-off-and-neutral) edit client.yml concurrently: rebase, never resolve their hunks away. Do not deploy in this task.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-wiring

### S12: Costmap ground warnings

- Status: done
- Files: `nodes/costmap_zones/costmap_zones/config.py`, `nodes/costmap_zones/costmap_zones/node.py`, `nodes/costmap_zones/costmap_zones/raster.py`, `nodes/costmap_zones/costmap_zones/ground_warnings.py`, `nodes/costmap_zones/tests/test_ground_warnings.py`, `nodes/costmap_zones/README.md`, `ansible/group_vars/client.yml`
- Verify: `(cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && uv run poe test-ansible`
- Verified head: 21a4c45f04383ddebccc15b624f0c0d0bbaa8cef

#### Requirement

- costmap_zones (design D15) stays the single writer of the Nav2 speed mask and adds the ground warning areas (slow-down, clear to ride through) next to the POI slow zones:
  - config (`costmap_zones/config.py`, pydantic, and the client.yml costmap_zones config block with comments): `ground_warnings_topic` `/ground/warning_areas`, `ground_warning_ttl_s` 3.0 (> 0), `ground_warning_speed_pct` 40 (10..100), `ground_warning_mask_value` 40 (1..99; KeepoutFilter maps 40 to cost ~101, far below INSCRIBED 253, so the area stays passable), `ground_cost_mask_topic` `/costmap_zones/ground_cost_mask`, `ground_cost_info_topic` `/costmap_filter_info/ground_cost`, `ground_mask_max_rate_hz` 2.0 (> 0).
  - `costmap_zones/ground_warnings.py`: pydantic model of the `/ground/warning_areas` JSON (`{stamp_ns, frame, areas: [{kind, polygon: [[x, y], ...], height_m, slope_deg, source}]}`, extra forbidden, contract `ground-warning-areas` of ground-state-json); `GroundWarningStore` keeping each received message's areas until `ground_warning_ttl_s` after receipt (monotonic), `active()` pruning expired ones; a message whose `frame` is not the `/map` header frame is ignored, logged once per frame and counted.
  - masks: the speed mask gets `ground_warning_speed_pct` inside active warning polygons (lowest percent wins with POI slow zones; keep-outs untouched); a new local-only ground cost mask (background `-1`, `ground_warning_mask_value` inside the polygons) on `ground_cost_mask_topic` with its CostmapFilterInfo (type 0, base 0.0, multiplier 1.0) on `ground_cost_info_topic`, published once with the first mask like the existing infos. Masks are republished when the active area set changes (receipt or expiry), at most `ground_mask_max_rate_hz`; an empty set after expiry republishes the cleared masks (real state); nothing is published before `/map` (no placeholder masks). The keepout mask (global and local costmaps) never carries ground areas: the global costmap stays static + lidar.
  - metrics: `costmap_zones_ground_areas` gauge (active areas), `costmap_zones_ground_ignored_total{reason}`.
- Tests first in `nodes/costmap_zones/tests/test_ground_warnings.py`, seen failing: message parsing and rejection; TTL expiry; frame mismatch ignored; a warning polygon sets the speed mask to 40 and the ground cost mask to 40 inside, -1/0 outside; overlap with a 30 % POI slow zone keeps 30; keepout mask unchanged by ground areas; rate limit; expiry republishes cleared masks; no mask before /map; config defaults equal the ground_estimator_cpp topic (`nodes/ground_estimator_cpp/config/ground_estimator.yaml` output_namespace + `/warning_areas`) and the client.yml block.
- `nodes/costmap_zones/README.md`: the ground warning input, masks, TTL, values and why the keepout mask is not used.

#### Acceptance conditions

- a1: active ground warning areas lower the speed mask to ground_warning_speed_pct (lowest wins with POI slow zones) and fill a local-only ground cost mask with a non-lethal value [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py]
- a2: areas expire after the TTL, frame mismatches are ignored, masks are rate-limited and never published before /map; the keepout mask never carries ground areas [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py]
- a3: the costmap_zones suite and lint pass [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: the README documents the ground warning input and masks [evidence: command: grep -q ground_warning nodes/costmap_zones/README.md]

#### Files

- `nodes/costmap_zones/costmap_zones/config.py`
- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/costmap_zones/raster.py`
- `nodes/costmap_zones/costmap_zones/ground_warnings.py`
- `nodes/costmap_zones/tests/test_ground_warnings.py`
- `nodes/costmap_zones/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

`/costmap_zones/ground_cost_mask`, `/costmap_filter_info/ground_cost` (part `ground-cost-mask`); consumes `/ground/warning_areas` (part `ground-warning-areas`).

#### Expected outcome

`cd nodes/costmap_zones && uv run --frozen pytest tests -q` passes; with ground_estimator_cpp running, /costmap_zones/ground_cost_mask and the speed mask carry the warning areas.

#### Steps

- write tests/test_ground_warnings.py
- run `cd nodes/costmap_zones && uv run --frozen pytest tests/test_ground_warnings.py -q`, see FAIL
- implement ground_warnings.py, config, raster and node wiring until green; add the client.yml keys; run the verify command
- update the README; commit `feat(costmap_zones): ground warning slow-down masks`

#### Notes

costmap_zones README "Slow-zone path cost" (mask 0..100 -> cost 0..254, value 60 -> 152) and raster.py `rasterise_speed` / `fill_zone`. The SpeedFilter of the local costmap turns the speed mask into `/speed_limit`, which controller_server (MPPI) honours (nav2_params.yaml speed_limit_topic).

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-type

### S13: Ground dashboard panels

- Status: done
- Files: `ansible/roles/monitoring/files/robot-slam.json`, `tests/test_monitoring_dashboards.py`, `scripts/node_cpu_compare.yaml`, `tests/test_node_cpu_compare.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_monitoring_dashboards.py tests/test_node_cpu_compare.py -q && uv run poe lint`
- Verified head: 36db64c70608d3194dac54d7eb85f14c725cabc1

#### Requirement

- Grafana (design D11): `ansible/roles/monitoring/files/robot-slam.json` gains a "Ground estimate" row: fit rate by result (`rate(ground_fits_total[1m])` by result), fit duration p50/p95 (`ground_fit_seconds_bucket`), inlier ratio, slope and plane_vs_base (deg), regions, warning areas, obstacle/drop point counts, colour mismatch ratio, cloud drops by reason, and the unit CPU `rate(container_cpu_usage_seconds_total{host="$host",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])` with the 0.10 core budget as a threshold line; every selector host-scoped as the dashboard's other panels.
- `tests/test_monitoring_dashboards.py`: the ground metric names (part `ground-metrics`: `ground_fits_total`, `ground_inlier_ratio`, `ground_slope_deg`, `ground_plane_vs_base_deg`, `ground_regions`, `ground_warning_areas`, `ground_obstacle_points`, `ground_drop_points`, `ground_colour_mismatch_ratio`, `ground_clouds_dropped_total`, `ground_labelled_clouds_total`; histogram `ground_fit_seconds`) in the node metric contract; the slam dashboard test asserts the ground row's queries; test-first, seen failing before the dashboard edit.
- `scripts/node_cpu_compare.yaml`: entry `{name: nav2_bringup_ground_layer, before_unit: ros2-nav2_bringup.service, after_unit: ros2-nav2_bringup.service, switch_time: 1}` with a comment that ros2-ground_estimator_cpp.service is new and measured "after" only by the runbook query (the file's convention); `tests/test_node_cpu_compare.py` stays green (extend it if it pins the entry list).
- `tests/README.md`: the changed tests described.

#### Acceptance conditions

- a1: robot-slam.json has the Ground estimate row with host-scoped queries on contract metrics only, including the unit CPU against the 0.10 core budget [evidence: test: tests/test_monitoring_dashboards.py]
- a2: the ground metric names are in the node metric contract [evidence: test: tests/test_monitoring_dashboards.py]
- a3: node_cpu_compare.yaml has the nav2_bringup ground-layer entry and its test passes [evidence: command: uv run pytest tests/test_node_cpu_compare.py -q]

#### Files

- `ansible/roles/monitoring/files/robot-slam.json`
- `tests/test_monitoring_dashboards.py`
- `scripts/node_cpu_compare.yaml`
- `tests/test_node_cpu_compare.py`
- `tests/README.md`

#### Interfaces

dashboard row "Ground estimate" in robot-slam.json; consumes parts `ground-metrics`, `ground-node-unit`.

#### Expected outcome

`uv run pytest tests/test_monitoring_dashboards.py tests/test_node_cpu_compare.py -q && uv run poe lint` passes.

#### Steps

- extend tests/test_monitoring_dashboards.py
- run it, see FAIL
- edit robot-slam.json and node_cpu_compare.yaml until green
- update tests/README.md
- commit `feat(monitoring): ground estimate dashboard row`.

#### Notes

robot-slam.json already has head_rgbd and tof panels (pattern). The tof epic's escalated final task also lists scripts/node_cpu_compare.yaml: rebase, keep its entries.

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-type

### S14: MCP ground summary

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/models.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/tests/test_ground_summary.py`, `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 4e39a3ba9a62cb26c602093745f94d3790cedb4a

#### Requirement

- mcp_server config (design D9): `topics.ground_state` (default `/ground/state`) and `timeouts.ground_state_stale_s` (default 1.0, > 0) in `mcp_server/config.py`; the client.yml mcp_server config block sets `topics.ground_state: /ground/state` with a comment (like `tof_points`).
- `ros_iface.py`: a permanent subscription to `topics.ground_state` (std_msgs/String, reliable + transient_local depth 1, like `tilt_guard_state`), parsed with a pydantic model `GroundState` (in `models.py`, fields = the ground-state contract keys of `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp` STATE_KEYS, `extra="forbid"`, regions as a list of `GroundRegion` with `name, kind, class, height_m, frame, edge {point, direction, side}, distance_m, points`, warnings as a list of `GroundWarning` with `kind, polygon, height_m, slope_deg, source`, traversability `{max_step_m, max_slope_deg}`); an invalid message is logged once per distinct error and not stored; `ground_state() -> tuple[GroundState, float] | None` returns the latest with its receive age, or None when absent or older than `ground_state_stale_s` (part `mcp-ground-state-client`).
- `models.py` `ground_regions_to_surfaces(state: GroundState) -> list[SurfaceRegion]`: one `SurfaceRegion(name, height_m, frame="base_link", edge=HalfPlaneEdge(point, direction, side))` per region in state order; the GroundRegion fields `kind`, `class`, `distance_m` and `points` are dropped (SurfaceRegion is `extra="forbid"`); at most `surfaces.MAX_REGIONS` (16) regions, the nearest kept; a region SurfaceRegion rejects (e.g. a zero direction or height_m outside -1..1) is skipped and logged, never altered.
- `models.py` `MapSummary.ground: GroundSummary | None` with field descriptions for the agent: floor height relative to base_link (m), plane source (fit/held), slope (deg) and slope_level, plane_vs_base (deg, large means head calibration error), the regions (kind, class obstacle/warning, height, distance, edge), the warning areas (kind, height, slope, nearest distance; "slow down, passable") and the obstacle rule (max step, max slope), colour mismatch ratio, state age (s); null with a note `ground estimate unavailable: no fresh /ground/state` when absent, and with a note `no floor plane: <reason>` when `plane` is null.
- `tools.py` `get_map_summary`: fills `ground` and documents it in the tool docstring (what it means, that it covers only what the head ToF sees, that obstacles_3d stays the fixed-band summary).
- Tests first in `nodes/mcp_server/tests/test_ground_summary.py`, seen failing: ground_regions_to_surfaces drops kind/distance_m/points and yields valid SurfaceRegion objects (frame base_link), caps at 16 keeping the nearest, skips an invalid region; the contract fixture `nodes/ground_estimator_cpp/test/data/state_example.json` parses into GroundState (path read from the repo); unknown or missing keys rejected; stale and absent states give the note; a plane-null state gives the reason note; summary fields mapped and rounded; config defaults and the client.yml value equal `/ground/state` and equal the ground_estimator_cpp `output_namespace` + `/state`.
- `nodes/mcp_server/README.md`: the ground field of get_map_summary, the topic and the stale timeout.

#### Acceptance conditions

- a1: the ground state contract fixture parses into GroundState and malformed states are rejected [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]
- a2: get_map_summary reports the ground estimate, or a note when it is stale, absent or plane-less [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]
- a3: the topic config is consistent with the ground_estimator_cpp output and client.yml [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]
- a4: the full mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: the mcp_server README documents the ground field [evidence: command: grep -q 'ground_state' nodes/mcp_server/README.md]

- a6: ground_regions_to_surfaces converts GroundRegion to SurfaceRegion dropping kind, class, distance_m and points, capped at MAX_REGIONS 16 [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_ground_summary.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

`GroundState`, `GroundRegion`, `GroundSummary`, `ground_regions_to_surfaces` (models.py); `RosInterface.ground_state()` (part `mcp-ground-state-client`); consumes part `ground-state`.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py -q` passes and the full mcp suite stays green.

#### Steps

- write tests/test_ground_summary.py
- run it, see FAIL
- implement config, models, ros_iface, tools
- run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- update the README
- commit `feat(mcp_server): ground estimate in get_map_summary`.

#### Notes

pattern: `tilt_guard_state` subscription (ros_iface.py:131, 798) and `tof_points` (ros_iface.py:1859). Deploying mcp_server kills a running agent session; do not deploy here.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on costmap-ground-warnings

### S15: Nav2 ground voxel layer

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_ground_voxel_layer.py`, `tests/test_nav_tof_voxel_layer.py`, `tests/test_tof_obstacle_wiring.py`, `tests/test_nav_tof_collision.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_tof_voxel_layer.py tests/test_tof_obstacle_wiring.py tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint`
- Verified head: 95e6ecd3934e83c3fa5092609fda5af412e3504f

#### Requirement

- `nodes/nav2_bringup/config/nav2_params.yaml` (design D8, D13, D15): local costmap `plugins: ["obstacle_layer", "tof_voxel_layer", "ground_voxel_layer", "keepout_filter", "ground_cost_filter", "inflation_layer"]`; new local-only `ground_cost_filter` (`nav2_costmap_2d::KeepoutFilter` as a layer below inflation like keepout_filter, `enabled: true`, `filter_info_topic: /costmap_filter_info/ground_cost`, `transform_tolerance: 0.2`, `override_lethal_cost: false`) carrying the costmap_zones ground warning mask: a non-lethal cost (mask 40 -> ~101) that MPPI and the planner avoid when cheap but may drive through, while the unchanged `speed_filter` applies the warning speed percent from the shared speed mask to `/speed_limit`, which controller_server honours (slow-down, not a stop); new `ground_voxel_layer` (`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`, `enabled: true`) with the tof_voxel_layer layer settings (voxel_decay 15.0, decay_model 0, voxel_size 0.05, track_unknown_space false, mark_threshold 0, update_footprint_enabled false, combination_method 1, publish_voxel_map false, transform_tolerance 0.3, mapping_mode false) and `observation_sources: ground_obstacle_mark ground_drop_mark ground_clear`: `ground_obstacle_mark` (topic `/ground/obstacle_points`) and `ground_drop_mark` (topic `/ground/drop_points`), PointCloud2, marking true, clearing false, `min_obstacle_height: -1.5`, `max_obstacle_height: 1.5` (pass-through: ground_estimator_cpp already applied the plane-relative bands), obstacle_range 2.0, filter voxel, voxel_min_points 0, observation_persistence 0.0, clear_after_reading true, expected_update_rate 0.0, inf_is_valid false; `ground_clear` = the tof_clear frustum on `/tof/points_filtered` (model_type 0, min_z 0.2, max_z 3.0, horizontal_fov_angle 1.0, vertical_fov_angle 0.78, decay_acceleration 1.0). Every value commented with its reason, a header comment naming the design (alongside the fixed bands, switch-over criteria in the README, runtime switch `ros2 param set /local_costmap/local_costmap ground_voxel_layer.enabled false`).
- Unchanged and pinned: `tof_voxel_layer` (all keys and the three sources), the collision_monitor `observation_sources` `["scan", "tof_obstacles", "tof_cliff"]` with their bands and the stale-source stop, CliffStop's source set (tof_cliff only), the global costmap plugins `[static_layer, obstacle_layer, keepout_filter, inflation_layer]` with no STVL and no ground topic, and the `gen_nav_footprint` generated blocks (not hand-edited; tests/test_nav_footprint.py stays green). A comment at the collision_monitor ToF sources states that the ground clouds do not feed the monitor (user decision D-A1: the 10 Hz ToF bands with the fail-safe stale stop stay) - the CliffStop review.
- Tests first, seen failing: new `tests/test_nav_ground_voxel_layer.py` (plugin order including ground_cost_filter between keepout_filter and inflation_layer; ground_cost_filter is a KeepoutFilter on `/costmap_filter_info/ground_cost` with override_lethal_cost false, absent from the global costmap, and its info topic equals the costmap_zones `ground_cost_info_topic` default; the costmap_zones `ground_warning_mask_value` maps (value * 254 / 100) below INSCRIBED 253; the local costmap still has exactly one SpeedFilter on `/costmap_filter_info/speed`; layer settings equal the tof layer's; sources, topics and pass-through limits; the ground marking topics equal the ground_estimator_cpp config `output_namespace` + `/obstacle_points` / `/drop_points` in client.yml and in `nodes/ground_estimator_cpp/config/ground_estimator.yaml`; ground_clear equals tof_clear; global costmap untouched; collision monitor sources and CliffStop sources unchanged and free of `/ground/` topics; values commented with their reasons; README section present and free of parking words; tests/README lists the file). `tests/test_nav_tof_voxel_layer.py`: `test_only_one_stvl_layer_in_the_local_costmap` becomes exactly `[tof_voxel_layer, ground_voxel_layer]`; the plugin-order expectations include the ground layer. `tests/test_tof_obstacle_wiring.py`: the STVL-source check stays scoped to `tof_voxel_layer`; the chain adds laser_filter `cloud_filtered_topic` == ground_estimator_cpp `cloud_topic` == `ground_clear` topic. `tests/test_nav_tof_collision.py`: an explicit assertion that no collision_monitor source topic starts with `/ground/`.
- `nodes/nav2_bringup/README.md`: section "Ground layer (plane-relative ToF marks)": what it marks, why a separate layer, the pass-through limits, what stays on the fixed bands (collision monitor, CliffStop), the obstacle vs warning rule (one third of the wheel diameter, max slope) and the slow-down mechanism (ground_cost_filter cost band plus the speed mask / speed_limit from costmap_zones), the runtime switch, and the switch-over criteria: (1) runbook level-floor check passes (plane height within 1 cm, slope and plane_vs_base below 1 deg); (2) the 3 cm sill, 5 cm sill, 10 cm drop at 1 m, box and ramp scenes mark as expected with the ground layer and the fixed-band layer disabled at runtime; (3) no false marks on the floor over a 5 min drive; (4) CPU within the budget; then a separate change disables `tof_mark` and `tof_cliff_mark`.
- `tests/README.md`: the new file and the changed tests described.

#### Acceptance conditions

- a1: the local costmap has ground_voxel_layer after tof_voxel_layer with the specified sources, topics and settings [evidence: test: tests/test_nav_ground_voxel_layer.py]
- a2: tof_voxel_layer, collision monitor sources, CliffStop and the global costmap are unchanged and carry no ground topic [evidence: command: uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_tof_obstacle_wiring.py tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]
- a3: the ground marking topics are consistent with the ground_estimator_cpp config [evidence: test: tests/test_nav_ground_voxel_layer.py]
- a4: the nav2_bringup README documents the ground layer and the switch-over criteria [evidence: doc-section: nodes/nav2_bringup/README.md#Ground layer (plane-relative ToF marks)]

- a5: ground warning areas reach the local costmap only as a non-lethal ground_cost_filter band, and the single speed filter applies the slow-down [evidence: test: tests/test_nav_ground_voxel_layer.py]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_ground_voxel_layer.py`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_tof_obstacle_wiring.py`
- `tests/test_nav_tof_collision.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

nav2_params.yaml `local_costmap.local_costmap.ros__parameters.ground_voxel_layer`; consumes part `ground-topics`.

#### Expected outcome

`uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_tof_voxel_layer.py tests/test_tof_obstacle_wiring.py tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q` passes.

#### Steps

- write tests/test_nav_ground_voxel_layer.py and the test changes
- run the expected-outcome command, see the new tests fail
- edit nav2_params.yaml until green
- write the README section and tests/README entries
- commit `feat(nav2): ground voxel layer alongside the ToF bands`.

#### Ruling

- 2026-10-20 accepted: nav2: flagged text documents the user-approved switch-over criteria (D8), not deferral; floor-guard: orchestrator verify-task.py at fcc0cbc6 passed, clean, a1-a4 met

#### Notes

nav2_params.yaml:242-329 (tof_voxel_layer), 579/686 (CliffStop), 732-750 (collision monitor sources). The tof epic's escalated `final-suite-and-runbook` also edits tests/test_tof_obstacle_wiring.py: rebase onto whatever landed, keep both sets of checks.

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-type, costmap-ground-warnings

### S16: MCP floor guard ground

- Status: done
- Files: `nodes/mcp_server/mcp_server/floor_guard.py`, `nodes/mcp_server/mcp_server/grasp_tools.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/arm.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_floor_guard_ground.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: fcc0cbc6

#### Requirement

- floor_guard (design D9, Q2): `FloorOverride.use_ground_estimate: bool = False`; the arm tools that take `surface_z_m` / `tilt_override_deg` / `surfaces` (tools.py via `grasp_tools.floor_override`) gain a `use_ground_estimate` parameter (default false) documented for the agent: "use the measured floor regions (steps, drop-offs) from the head ToF ground estimate; refused when no fresh estimate exists".
- `FloorGuard.surface(...)` receives the latest ground state (from `RosInterface.ground_state()`, part `mcp-ground-state-client`) and, when `use_ground_estimate` is true: state fresh (age <= `floor_guard.ground_max_age_s`, default 1.0, > 0, in the floor_guard settings) and `plane` not null -> the Terrain is built from `ground_regions_to_surfaces(state)` (models.py, delivered by mcp-ground-summary) followed by the call's explicit `surfaces` (listed later, so they win), at most MAX_REGIONS (explicit ones kept first when trimming); stale, absent or plane-null -> the motion is refused with a `GuardReport`/error whose reason names `/ground/state` and the cause. With `use_ground_estimate` false nothing changes (byte-identical behaviour; existing floor_guard tests unchanged).
- Tests first in `nodes/mcp_server/tests/test_floor_guard_ground.py`, seen failing: a state with a drop region 0.3 m ahead gives a Terrain whose lookup beyond the edge returns the drop height; explicit surfaces override an overlapping ground region; region count trimmed to MAX_REGIONS keeping explicit ones; stale/absent/plane-null refusals with the reason; default false leaves the surface model equal to the current one; the tool parameter is passed through.
- `nodes/mcp_server/README.md`: the floor_guard `use_ground_estimate` option, freshness rule and refusal.

#### Acceptance conditions

- a1: use_ground_estimate builds the floor_guard Terrain from fresh ground regions with explicit surfaces winning [evidence: test: nodes/mcp_server/tests/test_floor_guard_ground.py]
- a2: a requested but stale, absent or plane-less estimate refuses the motion with a reason naming /ground/state [evidence: test: nodes/mcp_server/tests/test_floor_guard_ground.py]
- a3: with the default false the floor_guard behaviour is unchanged and the full suite passes [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: the README documents the option [evidence: command: grep -q 'use_ground_estimate' nodes/mcp_server/README.md]

#### Files

- `nodes/mcp_server/mcp_server/floor_guard.py`
- `nodes/mcp_server/mcp_server/grasp_tools.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/arm.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_floor_guard_ground.py`
- `nodes/mcp_server/README.md`

#### Interfaces

`FloorOverride.use_ground_estimate`; `floor_guard.ground_max_age_s`; uses `ground_regions_to_surfaces` (models.py, from mcp-ground-summary).

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_floor_guard_ground.py -q` passes and the full mcp suite stays green.

#### Steps

- write tests/test_floor_guard_ground.py
- run it, see FAIL
- implement floor_guard, grasp_tools, tools, arm wiring
- run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- update the README
- commit `feat(mcp_server): floor_guard terrain from the ground estimate`.

#### Ruling

- 2026-10-20 accepted: verify-task.py run by the orchestrator at fcc0cbc6: passed, clean, a1-a4 met; mcp_server 2791 passed, lint clean

- 2026-10-20 accepted: nav2: flagged text documents the user-approved switch-over criteria (D8), not deferral; floor-guard: orchestrator verify-task.py at fcc0cbc6 passed, clean, a1-a4 met

#### Notes

surfaces.py `SurfaceRegion`/`Terrain` (46-330), floor_guard.py `surface()` (550-580); arm.py passes `FloorOverride` to the guard (arm.py:1155-1190).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on mcp-ground-summary

- 2026-10-27 done

### S17: Web UI ground backend

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/cloud_serializer.py`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/tests/test_ground_layer.py`, `ansible/group_vars/client.yml`, `nodes/web_ui/README.md`
- Verify: `(cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && uv run poe test-ansible`
- Verified head: e730abde0fc87f067ed377670bec974d8a4f3647

#### Requirement

- web_ui backend (design D10): map_nav tab keys `ground_cloud_topic` (sensor_msgs/PointCloud2 XYZ+rgb+label of ground_estimator_cpp, on demand) and `ground_state_topic` (std_msgs/String JSON) in `web_ui/config.py` (optional, None = no layer), `config/default.yaml` and the client.yml web_ui map_nav block (`/ground/labelled_points`, `/ground/state`, commented like `rgbd_cloud_topic`).
- Roles `GROUND_CLOUD_ROLE = "ground_cloud"` and `GROUND_STATE_ROLE = "ground_state"` (config.py, part `web-ground-roles`) wired in `bridge.py` `ROLE_SPECS`: ground_cloud with SENSOR_SUB_QOS and `serialize_ground_cloud` (cloud_serializer: the map3d serializer plus a `labels_b64` uint8 array aligned with the kept points after voxel downsampling - the label of the voxel's first point - and `frame`), ground_state with a transient_local reliable QoS (like the tilt_guard state) and `serialize_json_object`. Point cap and voxel as the rgbd cloud (RGBD_CLOUD_MAX_POINTS / RGBD_CLOUD_VOXEL_M or own constants with a comment).
- Staleness by backend receive age, like the tilt guard state (`TILT_STALE_S_DEFAULT`, `tilt_guard_stale_s`, `Bridge.mark_tilt_stale`): config `ground_state_stale_s` (default `GROUND_STALE_S_DEFAULT` = 2.0, > 0) in the map_nav tab config; `Bridge.mark_ground_stale()` replaces the cached ground state by a copy with `"stale": true` once its last receive (monotonic) is older than `ground_state_stale_s`, broadcast like any update and cleared by the next real message; it runs on the same timer as `mark_tilt_stale`. No client clock is involved.
- A cloud without a `label` field is serialized without `labels_b64` (logged once), never with fabricated labels.
- Tests first in `nodes/web_ui/tests/test_ground_layer.py`, seen failing: config keys optional and in default.yaml and client.yml with the ground_estimator_cpp topics (consistency with `nodes/ground_estimator_cpp/config/ground_estimator.yaml` output_namespace); roles derive the message types; a synthetic labelled PointCloud2 (x, y, z, rgb, label uint8) serializes with labels aligned to the kept points after voxel downsampling; missing label field; the state fixture `nodes/ground_estimator_cpp/test/data/state_example.json` passes through serialize_json_object unchanged; mark_ground_stale sets `stale: true` after `ground_state_stale_s` without a message and a new message clears it.
- `nodes/web_ui/README.md`: the two keys and roles.

#### Acceptance conditions

- a1: ground_cloud and ground_state roles and their tab keys exist in config, default.yaml and client.yml with the ground_estimator_cpp topics [evidence: test: nodes/web_ui/tests/test_ground_layer.py]
- a2: the labelled cloud serializes with labels aligned to the kept points, and without labels when the field is missing [evidence: test: nodes/web_ui/tests/test_ground_layer.py]
- a3: the full web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

- a4: the backend marks the cached ground state stale by receive age (ground_state_stale_s) and a new message clears it [evidence: test: nodes/web_ui/tests/test_ground_layer.py]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/cloud_serializer.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_ground_layer.py`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/README.md`

#### Interfaces

roles `ground_cloud`, `ground_state` (part `web-ground-roles`); payload `{points_b64, colors_b64, labels_b64, frame, frame_id, ...}`; consumes parts `ground-state`, `ground-topics`, `ground-colour-labels` (label values 0..4).

#### Expected outcome

`cd nodes/web_ui && uv run --frozen pytest tests/test_ground_layer.py -q` passes and the full web_ui suite stays green.

#### Steps

- write tests/test_ground_layer.py
- run it, see FAIL
- implement config, bridge, cloud_serializer, default.yaml, client.yml
- run `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- update the README
- commit `feat(web_ui): ground cloud and state roles`.

#### Notes

pattern: commit ba48647a (rgbd_cloud role: config.py:154/349, bridge.py:277-305, cloud_serializer.py:43-110). Other epics may edit client.yml concurrently: rebase.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on mcp-ground-summary

### S18: Web UI ground frontend

- Status: done
- Files: `nodes/web_ui/frontend/src/map3d/groundLayer.ts`, `nodes/web_ui/frontend/src/map3d/groundLayer.test.ts`, `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`, `nodes/web_ui/frontend/src/map3d/layers.ts`, `nodes/web_ui/frontend/src/map/LayersPanel.tsx`, `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/map3d/groundLayer.test.ts src/map/LayersPanel.test.tsx && npx tsc --noEmit && npm run lint && npm run build`
- Verified head: 175066d79225a8e8f24bae4be246291a4a2bccbd

#### Requirement

- web_ui frontend (design D10): a "Ground" map layer, off by default, subscribed on demand (only while enabled) to the `ground_cloud` and `ground_state` roles.
- `src/map3d/groundLayer.ts` (pure, unit-tested): decode the ground cloud payload (reuse `cloudDecode.ts` for points/colours, decode `labels_b64`); `labelColour(label)` palette: floor green, obstacle red, drop blue, floor_colour_mismatch orange, unlabelled grey; `planePatch(state)` -> the corners of a 2 m x 2 m square around base_link on the state plane (from `plane.normal` and `plane.height_m`, in base_link) or null when `plane` is null; `groundVisible(state)` -> false for an absent, stale (`stale: true`) or plane-null state; `warningPolygons(state)` -> the warning area polygons (base_link) with kind and source; `regionEdges(state)` -> line segments (2 m long, centred on the edge point along the direction) per region with kind for colour.
- `src/map3d/sceneLayers.tsx`: render the labelled points (PointSet, coloured per label, placed by the cloud frame pose like the Head RGB-D layer), the translucent plane patch, the edge lines (red for class obstacle, yellow for warning) and the warning area polygons (translucent yellow, "slow down") placed by the base_link pose; nothing drawn from the plane, edges or read-out when the state is absent, carries `"stale": true` (set by the backend from its receive age, web-ui-ground-backend) or has `plane: null`; the labelled points are hidden with the state when it is stale.
- `src/map/LayersPanel.tsx`: the "Ground" toggle with a legend of the five label colours, the obstacle/warning edge colours and the warning area fill and the slope / slope_level / plane height read-out from the state; `src/map3d/layers.ts` registers the layer id; `src/tabs/MapNavTab.tsx` wires the roles; the payload and state types live in `groundLayer.ts` (state keys equal STATE_KEYS of the ground-state contract).
- Tests first, seen failing: `src/map3d/groundLayer.test.ts` (label decode, palette, plane patch for a level and a 5 deg tilted plane, null plane, region edge segments with class colours, warning polygons, `groundVisible(state)` false for absent, `stale: true` and plane-null states); LayersPanel test for the toggle and legend (extend `src/map/LayersPanel.test.tsx`).
- `nodes/web_ui/README.md`: the Ground layer (what the colours mean, on demand, data source).

#### Acceptance conditions

- a1: ground cloud labels decode and map to the five legend colours; plane patch, class-coloured region edges and warning polygons are computed from the state [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts]
- a2: the Ground toggle, legend and slope read-out are in the layers panel [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/LayersPanel.test.tsx]
- a3: the frontend type-checks, lints and builds [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]
- a4: groundVisible is false for absent, backend-stale and plane-null states, so nothing is drawn [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts]

#### Files

- `nodes/web_ui/frontend/src/map3d/groundLayer.ts`
- `nodes/web_ui/frontend/src/map3d/groundLayer.test.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map/LayersPanel.tsx`
- `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

`groundLayer.ts` `decodeGroundCloud`, `labelColour`, `planePatch`, `regionEdges`; layer id `ground`; consumes part `web-ground-roles`, `ground-state`.

#### Expected outcome

`cd nodes/web_ui/frontend && npx vitest run src/map3d/groundLayer.test.ts src/map/LayersPanel.test.tsx && npx tsc --noEmit && npm run lint && npm run build` passes.

#### Steps

- write groundLayer.test.ts and the LayersPanel test
- run vitest, see FAIL
- implement groundLayer.ts and the wiring until green
- run the full frontend checks
- update the README
- commit `feat(web_ui): ground layer on the map`.

#### Notes

pattern: commit 044c6aae (Head RGB-D cloud layer: layers.ts, sceneLayers.tsx, LayersPanel.tsx, MapNavTab.tsx, obstaclePoints.ts, cloudDecode.ts).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on web-ui-ground-backend

### S19: Ground final suite runbook

- Status: done
- Files: `docs/runbooks/ground-estimation.md`, `MEMORY.md`, `docs/diagrams/topic_flow.puml`, `tests/README.md`, `tests/test_ground_docs.py`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint)`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

- `docs/runbooks/ground-estimation.md` (design D11, D12), executed by the orchestrator on the robot after merge (not by the implementer), sections: `## Preconditions` (head_rgbd_cpp and laser_filter running, head at home tilt 30 deg down, no agent session running before mcp_server is deployed - MEMORY "deploy during agent session"); `## Deploy` (`./scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones nav2_bringup web_ui 2>&1 | tee .logs/deploy-client-ground.log`, then mcp_server after the session check, `ansible-deploy` skill first); `## Topic checks` (rates of /ground/obstacle_points and /ground/drop_points ~10 Hz, /ground/labelled_points ~1 Hz, /ground/state 2 Hz, via topic_scraper_api + scripts/topic_scraper_collect.py with topic_scraper_api enabled per CLAUDE.md; record the TF drop counter and the head joint_states rate); `## Level floor` (plane height within 1 cm, slope and plane_vs_base below 1 deg, prior imu, slope_level ok); `## Scenes`, both sides of the obstacle rule (max step = one third of the wheel diameter = 0.04 m, max slope 8 deg): 3 cm sill -> warning area, /speed_limit at the warning percent while crossing, non-lethal ground cost cells, the robot drives over it; 5 cm sill -> lethal ground mark, the planner routes around; 3 cm and 5 cm drops likewise; 10 cm drop at 1 m: drop region with height within 2 cm and edge distance within 5 cm, ground_drop_mark cells, CliffStop unchanged; box -> obstacle; 6 deg ramp -> warning (slope within 1 deg of the measured angle), driven over slowly; a ramp steeper than 8 deg -> obstacle; red sheet flat on the floor -> floor_colour_mismatch labels in the web UI Ground layer and a colour warning area (slow-down, passable) with the fixed vs ground marks compared in the web UI local costmap with `ground_voxel_layer.enabled` and `tof_voxel_layer.enabled` toggled at runtime; `## CPU and memory` (Pi 5: `rate(container_cpu_usage_seconds_total{host="client",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])` p95 <= 0.10 core over 10 min of driving, nav2_bringup before/after via scripts/node_cpu_compare.py with the `nav2_bringup_ground_layer` entry <= +0.05 core; memory p50 and peak of the unit; client.yml `memory_low` = p50 and `memory_max` >= 2x peak written and committed `chore(ansible): ground_estimator_cpp measured memory`, then redeployed); `## Build and CI validation` (validated in CI and on the robot, not by the implementer: the cpp-node-test container suite of ground_estimator_cpp in the CI matrix, the frontend suite `npx tsc --noEmit && npm test && npm run lint && npm run build`, and the PCL build of the deploy on the Pi 5 with `-j2` within memory, each recorded with its result line); `## Threshold validation` (the defaults `fit.distance_m` 0.02, `fit.max_normal_angle_deg` 10, `fit.max_offset_m` 0.08, `traversability.wheel_radius_m` 0.06 with `max_step_fraction` 1/3 (0.04 m), `traversability.max_slope_deg` 8, costmap_zones `ground_warning_speed_pct` 40 and `ground_warning_mask_value` 40, `edges.step_max_height_m` 0.2, `colour.max_mahalanobis` 3.0 confirmed by the scenes or corrected in client.yml with the measured reason); `## Switch-over checklist` (the nav2_bringup README criteria, each with its measured result; the switch-over itself is a separate change); `## Results` (a table filled by the executor: check, expected, measured, pass).
- `MEMORY.md`: one decision entry (ground plane from the 10 Hz ToF with IMU prior in ground_estimator_cpp; plane-relative bands in a second STVL layer alongside the fixed bands; collision monitor and global costmap unchanged; obstacle vs warning rule (one third of the wheel diameter, 8 deg) with warnings as costmap_zones slow-down (speed mask + local non-lethal ground cost filter), applied to colour-flagged flat objects too; floor_guard opt-in; CPU budget 0.10 core).
- `docs/diagrams/topic_flow.puml`: ground_estimator_cpp with its inputs (/tof/points_filtered, /imu/data, /head_rgbd/points) and outputs (/ground/* to Nav2 local costmap, mcp_server, web_ui).
- `tests/README.md`: every test file added or changed by this epic is listed and described (test_ground_estimator_deploy.py, test_nav_ground_voxel_layer.py, the changed tof/nav/monitoring tests, the mcp and web_ui ground tests, the C++ gtests of ground_estimator_cpp by name, test_ground_docs.py).
- Test first: new `tests/test_ground_docs.py`, seen failing: the runbook has the sections above; the runbook's thresholds equal the design values (0.10 core, 1 cm, 1 deg) and the Threshold validation defaults equal `nodes/ground_estimator_cpp/config/ground_estimator.yaml`; every `scripts/...` path named in the runbook exists (`git ls-files`); every `/ground/` topic named in the runbook is a ground_estimator_cpp output topic of the shipped config; MEMORY.md has the entry; topic_flow.puml names ground_estimator_cpp and the four /ground topics; tests/README.md lists every `tests/test_*ground*.py` file and test_ground_docs.py.
- The verify command passes; the report quotes every summary line (the container C++ suite and the frontend suite are validated in CI per the runbook section).

#### Acceptance conditions

- a1: the runbook has every section with the design thresholds, and the docs test passes [evidence: test: tests/test_ground_docs.py]
- a2: MEMORY.md, the topic flow diagram and tests/README.md cover the epic [evidence: test: tests/test_ground_docs.py]
- a3: root tests, lint and the ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible]
- a4: every scripts/ path and /ground/ topic the runbook names exists, and its threshold defaults equal the shipped config [evidence: test: tests/test_ground_docs.py]

#### Files

- `docs/runbooks/ground-estimation.md`
- `MEMORY.md`
- `docs/diagrams/topic_flow.puml`
- `tests/README.md`
- `tests/test_ground_docs.py`

#### Interfaces

runbook `docs/runbooks/ground-estimation.md`; consumes part `ground-node-unit`.

#### Expected outcome

the full verify command is green; the runbook is ready for the robot run.

#### Steps

- write tests/test_ground_docs.py
- run `uv run pytest tests/test_ground_docs.py -q`, see FAIL
- write the runbook, MEMORY entry, diagram and tests/README entries
- run the full verify
- commit `docs(ground): runbook, memory and diagram`.

#### Ruling

- 2026-10-20 accepted: parking findings are false positives: the runbook documents robot steps executed after merge by design (A6); repo suites 1033 passed, lint green

#### Notes

runbook pattern: docs/runbooks/head-rgbd-fusion.md, docs/runbooks/map3d-mapping.md. MEMORY.md and tests/README.md are also edited by other epics: rebase.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on nav2-ground-voxel-layer, mcp-floor-guard-ground, web-ui-ground-frontend, ground-dashboard-panels, ground-node-readme

- 2026-10-27 done: superseded by realsense-d435-head-switchov/ground-live-acceptance-on-d4

- 2026-10-27 actuals: no agent records

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none

## Rulings (migrated)

- ground-colour-labels: accept true, at 2026-10-20T12:14:41Z: Verifier crashed (--prev-head unknown); manual verify at eac98687: clean tree, scripts/cpp-node-test.sh ground_estimator_cpp 80 tests 0 failures PASS. TDD process gap: test and implementation landed in one commit (0343d346); tests are new for a new module and cannot build without it.
- mcp-floor-guard-ground: accept true, at 2026-10-20T15:06:50Z: verify-task.py run by the orchestrator at fcc0cbc6: passed, clean, a1-a4 met; mcp_server 2791 passed, lint clean
- nav2-ground-voxel-layer: accept true, at 2026-10-20T15:06:58Z: nav2: flagged text documents the user-approved switch-over criteria (D8), not deferral; floor-guard: orchestrator verify-task.py at fcc0cbc6 passed, clean, a1-a4 met
- mcp-floor-guard-ground: accept true, at 2026-10-20T15:06:58Z: nav2: flagged text documents the user-approved switch-over criteria (D8), not deferral; floor-guard: orchestrator verify-task.py at fcc0cbc6 passed, clean, a1-a4 met
- ground-final-suite-runbook: accept true, at 2026-10-20T16:35:12Z: parking findings are false positives: the runbook documents robot steps executed after merge by design (A6); repo suites 1033 passed, lint green
