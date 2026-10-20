---
title: Ground colour labels
status: todo
repo: ros2-lerobot-sverve-platform
value: "The RGB-D colour tells floor from obstacle, including flat objects the depth cannot separate."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp, nodes/ground_estimator_cpp/src/core/colour.cpp, nodes/ground_estimator_cpp/test/core/test_colour.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-band-segmentation]
owns: [ground-colour-labels]
uses: [ground-config, ground-segmentation, ground-synthetic-scene]
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

# Ground colour labels

## Requirement

- `include/ground_estimator_cpp/colour.hpp`, `src/core/colour.cpp` (design D6): `enum class Label : uint8_t { floor = 0, obstacle = 1, drop = 2, floor_colour_mismatch = 3, unlabelled = 4 }`; `class FloorColourModel` with `update(const std::vector<cv::Vec3b> &rgb_floor_samples)` (converted to CIELAB with `cv::cvtColor`, mean and 3x3 covariance blended with weight `colour.forgetting`, total sample count kept), `bool ready() const` (>= `colour.min_floor_samples`), `double mahalanobis(const cv::Vec3b &rgb) const`; `struct LabelStats { std::size_t floor, obstacle, drop, mismatch, unlabelled; double mismatch_ratio; }` (mismatch_ratio = mismatch / (floor + mismatch), 0 points -> ratio not reported: std::optional); `LabelStats label_cloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_base, const Plane &, const BandsConfig &, const ColourConfig &, double floor_distance_m, FloorColourModel &model, std::vector<Label> &labels)`: band per point via `classify` (ground-segmentation); floor-band points first update the model, then a floor-band point with model ready and mahalanobis > `colour.max_mahalanobis` is floor_colour_mismatch; before the model is ready floor-band points stay floor; `sensor_msgs::msg::PointCloud2 to_labelled_msg(const pcl::PointCloud<pcl::PointXYZRGB> &, const std::vector<Label> &, frame, stamp)`: fields x, y, z (float32), rgb (float32 packed, as head_rgbd_cpp's cloud), label (uint8).
- `colour.enabled: false` makes label_cloud return geometric labels only (no mismatch, model untouched).
- Tests first in `test/core/test_colour.cpp`, seen failing: a grey floor with a red A4 sheet lying flat -> sheet points floor_colour_mismatch, the rest floor, ratio ~ sheet share; a grey box -> obstacle (geometry wins); before min_floor_samples nothing is mismatch; forgetting: the model follows a floor colour change within 1/forgetting updates; Lab conversion of pure white/black/grey gives L 100/0/~53; labelled message fields, offsets and label values; colour.enabled false.

**Interfaces**: `colour.hpp` (part `ground-colour-labels`): `Label`, `FloorColourModel`, `LabelStats`, `label_cloud`, `to_labelled_msg`. The label values 0..4 are a contract read by web_ui (design D6).

**Expected outcome**: `scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_colour.

**Steps**: write test_colour.cpp; run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL; implement until PASS; commit `feat(ground_estimator_cpp): RGB-D floor colour labels`.

**Notes**: head_rgbd_cpp packs rgb as float32 (`nodes/head_rgbd_cpp/src/core/cloud.cpp`); pcl::PointXYZRGB matches that layout via pcl_conversions::fromROSMsg.

## Acceptance conditions

- a1: geometric floor points with a non-floor colour are labelled floor_colour_mismatch once the model is ready; geometry decides obstacle and drop [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the CIELAB floor model updates with forgetting and reports Mahalanobis distances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the labelled PointCloud2 carries x, y, z, rgb and a uint8 label field with values 0..4 [evidence: symbol: nodes/ground_estimator_cpp/src/core/colour.cpp::to_labelled_msg]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp`
- `nodes/ground_estimator_cpp/src/core/colour.cpp`
- `nodes/ground_estimator_cpp/test/core/test_colour.cpp`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
