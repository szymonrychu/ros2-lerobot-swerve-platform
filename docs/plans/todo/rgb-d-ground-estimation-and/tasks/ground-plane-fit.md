---
title: Ground plane fit
status: todo
repo: ros2-lerobot-sverve-platform
value: "The floor plane is found from the ToF cloud with the IMU prior, and walls or table tops are never mistaken for it."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/plane_fit.hpp, nodes/ground_estimator_cpp/src/core/plane_fit.cpp, nodes/ground_estimator_cpp/test/core/test_plane_fit.cpp, nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-package-config]
owns: [ground-plane-fit, ground-synthetic-scene]
uses: [ground-config]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground plane fit

## Requirement

- `include/ground_estimator_cpp/plane_fit.hpp`, `src/core/plane_fit.cpp` (design D2): `struct Plane { Eigen::Vector3d normal; double d; }` (unit normal pointing up, `normal.dot(p) + d = 0`), `double height_above(const Plane &, const Eigen::Vector3d &)` (signed, > 0 above), `double height_at_origin(const Plane &)` (plane z at base_link x=y=0); `enum class FitReject { too_few_points, no_plane, normal_off_prior, offset_off_prior, too_few_inliers, low_inlier_ratio }`, `const char *to_string(FitReject)`; `struct FitResult { std::optional<Plane> plane; std::optional<FitReject> reject; std::size_t candidates; std::size_t inliers; double inlier_ratio; }`; `FitResult fit_plane(const pcl::PointCloud<pcl::PointXYZ> &points_base, const Eigen::Vector3d &prior_up, const FitConfig &cfg, unsigned seed)`.
- The fit keeps candidate points with |z| <= `fit.candidate_band_m` and horizontal range <= `fit.max_range_m`, then runs `pcl::SACSegmentation<pcl::PointXYZ>` with `SACMODEL_PERPENDICULAR_PLANE`, `SAC_RANSAC`, axis = `prior_up`, eps angle `fit.max_normal_angle_deg`, distance `fit.distance_m`, `fit.max_iterations`, the given seed (deterministic). The normal is oriented so `normal.dot(prior_up) > 0`. Validity gates in this order, each returning its `FitReject`: fewer candidates than `fit.min_inliers` (too_few_points), no model (no_plane), angle to prior > max (normal_off_prior), |height_at_origin| > `fit.max_offset_m` (offset_off_prior), inliers < `fit.min_inliers` (too_few_inliers), inliers / candidates < `fit.min_inlier_ratio` (low_inlier_ratio).
- `test/core/synthetic_floor.hpp` (part `ground-synthetic-scene`, header-only, used by later tests): deterministic generators in base_link: a floor plane with given tilt (roll, pitch) and height, Gaussian noise, a uniform depth-like bias along the ray from a sensor origin (the ToF long bias), a box (obstacle) at a position and size, a step-up region (flat top at a height over a half-plane), a drop-off (lower floor beyond an edge line), a wall, a table top; points on an image-like grid of a 240x180 sensor footprint subsampled like decimation 4.
- Tests first in `test/core/test_plane_fit.cpp`, seen failing: level floor -> height ~0, normal ~+z (within 0.5 deg, 5 mm); 5 deg pitched floor with a matching prior -> found, angle within 0.5 deg; same floor with a level prior -> found (inside 10 deg); a wall-only cloud -> normal_off_prior or no_plane; a table top at +0.3 m only -> offset_off_prior or too_few_points (outside the band); floor with a box covering half the view -> still found; floor biased by 30 mm along the rays -> plane found and the reported height reflects the bias (within the gate); empty and 10-point clouds -> too_few_points; same input and seed -> identical output.

## Acceptance conditions

- a1: fit_plane finds level, tilted, partially occluded and biased floors within the stated tolerances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: walls, table tops and sparse clouds are rejected with the named FitReject reason [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the fit is deterministic for a given seed [evidence: symbol: nodes/ground_estimator_cpp/src/core/plane_fit.cpp::fit_plane]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/plane_fit.hpp`
- `nodes/ground_estimator_cpp/src/core/plane_fit.cpp`
- `nodes/ground_estimator_cpp/test/core/test_plane_fit.cpp`
- `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`

## Interfaces

`plane_fit.hpp` (part `ground-plane-fit`), `synthetic_floor.hpp` (part `ground-synthetic-scene`).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_plane_fit.

## Steps

- write synthetic_floor.hpp and test_plane_fit.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement plane_fit.cpp until PASS
- commit `feat(ground_estimator_cpp): plane fit with gravity prior`.

## Notes

PCL `SACSegmentation::setAxis` takes `Eigen::Vector3f`; `setEpsAngle` is radians. `pcl::console::setVerbosityLevel(L_ERROR)` keeps PCL quiet in tests and the node.
