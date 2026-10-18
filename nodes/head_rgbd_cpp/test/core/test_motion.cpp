#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "head_rgbd_cpp/motion.hpp"
#include "head_rgbd_cpp/registration.hpp"
#include "synthetic_scene.hpp"

namespace
{

using head_rgbd_cpp::compensated_transform;
using head_rgbd_cpp::decision_label;
using head_rgbd_cpp::DepthConfig;
using head_rgbd_cpp::evaluate_pair;
using head_rgbd_cpp::frame_rate;
using head_rgbd_cpp::MotionConfig;
using head_rgbd_cpp::MotionRate;
using head_rgbd_cpp::PairDecision;
using head_rgbd_cpp::PinholeModel;
using head_rgbd_cpp::RayTable;
using head_rgbd_cpp::register_depth;
using head_rgbd_cpp::registration_transform;
using head_rgbd_cpp::SyncConfig;
namespace synth = head_rgbd_cpp::test;

constexpr double DEG = CV_PI / 180.0;
constexpr int64_t NS_PER_S = 1000000000;
// URDF (nodes/web_ui/urdf/head.urdf): head_mount_joint, head_tilt origin (the pan axis is 0.04 m below the tilt axis
// and the cameras), the camera/ToF origins on head_link and the optical rotation rpy (-pi/2, 0, -pi/2).
const Eigen::Vector3d MOUNT_XYZ(0.15, 0.10, 0.31);
constexpr double PAN_TO_TILT_M = 0.04;
constexpr double CAMERA_FORWARD_M = 0.02;
constexpr double RGB_SIDE_M = -0.025;
constexpr double TOF_SIDE_M = 0.025;
// The head looks down a little so the box's vertical edges converge in the image and their sub-pixel phase varies
// from row to row (an edge error that is not a single quantisation step).
constexpr double HEAD_TILT_RAD = 0.3;
// The capture offset of the scenarios: RGB exposed 33 ms after the ToF.
constexpr double PAIR_DT_S = 0.033;
constexpr double PAN_RATE_DEG_S = 30.0;
constexpr double BASE_YAW_RATE_RAD_S = 0.5;
// The box is everything nearer than this; the wall is beyond it.
constexpr double BOX_MASK_MAX_M = 2.0;
// Edges this close to the image border are ignored (clipped, not located).
constexpr int EDGE_BORDER_PX = 2;
constexpr double COMPENSATED_MAX_MEDIAN_PX = 0.5;
constexpr double STATIC_MIN_MEDIAN_PX = 3.0;
constexpr size_t MIN_EDGE_SAMPLES = 100;

// A rotation about an axis through the origin.
Eigen::Isometry3d rotation(double angle_rad, const Eigen::Vector3d & axis)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(angle_rad, axis.normalized()).toRotationMatrix();
  return T;
}

// A pure translation.
Eigen::Isometry3d translation(const Eigen::Vector3d & t)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = t;
  return T;
}

// The URDF optical rotation rpy (-pi/2, 0, -pi/2): fixed-axis roll, pitch, yaw = Rz(yaw) * Rx(roll).
Eigen::Isometry3d optical_rotation()
{
  return rotation(-CV_PI / 2.0, Eigen::Vector3d::UnitZ()) * rotation(-CV_PI / 2.0, Eigen::Vector3d::UnitX());
}

// The pose of a head camera's optical frame in odom, through the URDF chain base_link -> head_pan_base -> head_pan
// (about z) -> head_tilt (0.04 m up, about y) -> camera (forward, side) -> optical.
// @param base_yaw_rad the base_link yaw in odom (base_link at the odom origin).
// @param pan_rad the head_pan joint position.
// @param side_m the y origin of the camera on head_link (RGB_SIDE_M or TOF_SIDE_M).
// @return odom_T_optical.
Eigen::Isometry3d head_camera_pose(double base_yaw_rad, double pan_rad, double side_m)
{
  return rotation(base_yaw_rad, Eigen::Vector3d::UnitZ()) * translation(MOUNT_XYZ) *
         rotation(pan_rad, Eigen::Vector3d::UnitZ()) * translation(Eigen::Vector3d(0.0, 0.0, PAN_TO_TILT_M)) *
         rotation(HEAD_TILT_RAD, Eigen::Vector3d::UnitY()) *
         translation(Eigen::Vector3d(CAMERA_FORWARD_M, side_m, 0.0)) * optical_rotation();
}

// A wall 3 m ahead of the robot in odom and a 0.5 m wide box 1.2 m ahead, in front of the head.
synth::Scene box_before_wall()
{
  synth::Scene scene;
  scene.planes.push_back(synth::Plane{Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3d(-1.0, 0.0, 0.0), 0.3});
  scene.boxes.push_back(synth::Box{Eigen::Vector3d(1.2, -0.15, -0.2), Eigen::Vector3d(1.5, 0.35, 0.5), 0.8});
  return scene;
}

// The first and last column of the box (depth in (0, BOX_MASK_MAX_M)) in a row, (-1, -1) when the row has none.
template <typename T>
std::pair<int, int> box_span(const cv::Mat & depth_m_or_mm, int v, double to_m)
{
  int first = -1;
  int last = -1;
  for (int u = 0; u < depth_m_or_mm.cols; ++u) {
    const double z = static_cast<double>(depth_m_or_mm.at<T>(v, u)) * to_m;
    if (z > 0.0 && z < BOX_MASK_MAX_M) {
      if (first < 0) {
        first = u;
      }
      last = u;
    }
  }
  return {first, last};
}

// The edge errors of a registered depth against the ground truth: per row where both show the box, the column
// distance between the registered and the true left edge and between the right edges (edges near the border are
// skipped).
// @param registered_mm CV_16UC1 registered depth, mm.
// @param truth_m CV_64FC1 ground truth seen by the RGB camera, metres.
// @return the absolute errors, pixels.
std::vector<double> edge_errors(const cv::Mat & registered_mm, const cv::Mat & truth_m)
{
  std::vector<double> errors;
  const int max_u = truth_m.cols - 1 - EDGE_BORDER_PX;
  for (int v = 0; v < truth_m.rows; ++v) {
    const auto [reg_left, reg_right] = box_span<uint16_t>(registered_mm, v, 1e-3);
    const auto [true_left, true_right] = box_span<double>(truth_m, v, 1.0);
    if (reg_left < 0 || true_left < 0) {
      continue;
    }
    if (std::min(reg_left, true_left) >= EDGE_BORDER_PX) {
      errors.push_back(std::abs(reg_left - true_left));
    }
    if (std::max(reg_right, true_right) <= max_u) {
      errors.push_back(std::abs(reg_right - true_right));
    }
  }
  return errors;
}

// The median of a non-empty sample.
double median(std::vector<double> values)
{
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  return *mid;
}

// The camera poses of one pair: each camera at its own stamp, and the RGB at the ToF stamp (the static rig).
struct PairPoses
{
  Eigen::Isometry3d odom_T_tof_at_ttof;
  Eigen::Isometry3d odom_T_rgb_at_trgb;
  Eigen::Isometry3d odom_T_rgb_at_ttof;
};

// Median edge errors of registering with the compensated and with the static transform.
struct EdgeResult
{
  double compensated_px;
  double static_px;
  size_t samples;
};

// Render the ToF at its stamp and the RGB ground truth at the RGB stamp, register the ToF with the compensated and
// with the static transform and measure both against the ground truth at 288x162.
EdgeResult measure(const PairPoses & poses)
{
  const synth::Scene scene = box_before_wall();
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = synth::default_rgb_out_model();
  const RayTable rays(tof);
  const cv::Mat tof_mm = synth::render_tof_depth_mm(scene, tof, poses.odom_T_tof_at_ttof);
  const cv::Mat truth = synth::render_rgb_truth_m(scene, rgb, poses.odom_T_rgb_at_trgb);

  const Eigen::Isometry3d compensated = compensated_transform(poses.odom_T_rgb_at_trgb, poses.odom_T_tof_at_ttof);
  const Eigen::Isometry3d static_T = compensated_transform(poses.odom_T_rgb_at_ttof, poses.odom_T_tof_at_ttof);
  const std::vector<double> comp_errors =
    edge_errors(register_depth(tof_mm, rays, compensated, rgb, DepthConfig{}), truth);
  const std::vector<double> static_errors =
    edge_errors(register_depth(tof_mm, rays, static_T, rgb, DepthConfig{}), truth);
  if (comp_errors.empty() || static_errors.empty()) {
    return EdgeResult{std::numeric_limits<double>::infinity(), 0.0, 0};
  }
  return EdgeResult{median(comp_errors), median(static_errors), std::min(comp_errors.size(), static_errors.size())};
}

// A head pan at pan_rate_deg_s with a static base; the ToF at pan 0, the RGB exposed dt_s later.
PairPoses head_pan_pair(double pan_rate_deg_s, double dt_s)
{
  const double pan_rgb = pan_rate_deg_s * DEG * dt_s;
  return PairPoses{
    head_camera_pose(0.0, 0.0, TOF_SIDE_M), head_camera_pose(0.0, pan_rgb, RGB_SIDE_M),
    head_camera_pose(0.0, 0.0, RGB_SIDE_M)};
}

// A base rotation at yaw_rate_rad_s with a static head; the ToF at yaw 0, the RGB exposed dt_s later.
PairPoses base_turn_pair(double yaw_rate_rad_s, double dt_s)
{
  const double yaw_rgb = yaw_rate_rad_s * dt_s;
  return PairPoses{
    head_camera_pose(0.0, 0.0, TOF_SIDE_M), head_camera_pose(yaw_rgb, 0.0, RGB_SIDE_M),
    head_camera_pose(0.0, 0.0, RGB_SIDE_M)};
}

// A pose with a rotation about an arbitrary axis and a translation.
Eigen::Isometry3d pose(double angle_deg, const Eigen::Vector3d & axis, const Eigen::Vector3d & t)
{
  return translation(t) * rotation(angle_deg * DEG, axis);
}

int64_t ns(double seconds) { return static_cast<int64_t>(std::llround(seconds * NS_PER_S)); }

}  // namespace

TEST(CompensatedTransform, MapsAToFPointAtTTofToTheSameFixedPointSeenFromTheRgbAtTRgb)
{
  // Two arbitrary poses (both rotated and translated) so that inverting the wrong side cannot pass.
  const Eigen::Isometry3d fixed_T_rgb = pose(25.0, Eigen::Vector3d(0.2, -1.0, 0.4), Eigen::Vector3d(0.3, -0.2, 0.5));
  const Eigen::Isometry3d fixed_T_tof = pose(-40.0, Eigen::Vector3d(1.0, 0.3, -0.2), Eigen::Vector3d(-0.1, 0.4, 0.2));
  const Eigen::Vector3d p_fixed(1.5, -0.7, 2.0);
  const Eigen::Vector3d p_tof = fixed_T_tof.inverse() * p_fixed;
  const Eigen::Vector3d p_rgb = fixed_T_rgb.inverse() * p_fixed;

  const Eigen::Isometry3d rgb_T_tof = compensated_transform(fixed_T_rgb, fixed_T_tof);
  EXPECT_LT((rgb_T_tof * p_tof - p_rgb).norm(), 1e-12);
  // The reverse direction is its inverse: swapping the arguments maps RGB points into the ToF frame.
  const Eigen::Isometry3d tof_T_rgb = compensated_transform(fixed_T_tof, fixed_T_rgb);
  EXPECT_LT((tof_T_rgb * p_rgb - p_tof).norm(), 1e-12);
  EXPECT_GT((tof_T_rgb * p_tof - p_rgb).norm(), 0.1);
  EXPECT_TRUE((rgb_T_tof * tof_T_rgb).isApprox(Eigen::Isometry3d::Identity(), 1e-12));
}

TEST(CompensatedTransform, EqualsTheStaticExtrinsicWhenNothingMoves)
{
  // Same base and head pose at both stamps: the time-travel transform is the URDF ToF -> RGB extrinsic, a pure 0.05 m
  // shift along the optical x axis (the RGB is right of the ToF, so the ToF origin is at x = -0.05 m in the RGB frame).
  const Eigen::Isometry3d rgb_T_tof =
    compensated_transform(head_camera_pose(0.0, 0.0, RGB_SIDE_M), head_camera_pose(0.0, 0.0, TOF_SIDE_M));
  EXPECT_TRUE(rgb_T_tof.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_LT((rgb_T_tof.translation() - Eigen::Vector3d(-0.05, 0.0, 0.0)).norm(), 1e-12);
}

TEST(RegistrationTransform, IsTheCompensatedTransformWhenCompensatingElseTheStaticOne)
{
  const PairPoses poses = head_pan_pair(PAN_RATE_DEG_S, PAIR_DT_S);
  const Eigen::Isometry3d static_T = compensated_transform(poses.odom_T_rgb_at_ttof, poses.odom_T_tof_at_ttof);
  const Eigen::Isometry3d compensated = compensated_transform(poses.odom_T_rgb_at_trgb, poses.odom_T_tof_at_ttof);
  MotionConfig config;
  config.compensate = true;
  EXPECT_TRUE(registration_transform(config, static_T, poses.odom_T_rgb_at_trgb, poses.odom_T_tof_at_ttof)
                .isApprox(compensated, 1e-12));
  config.compensate = false;
  EXPECT_TRUE(registration_transform(config, static_T, poses.odom_T_rgb_at_trgb, poses.odom_T_tof_at_ttof)
                .isApprox(static_T, 1e-12));
  EXPECT_FALSE(compensated.isApprox(static_T, 1e-6));
}

TEST(MotionCompensation, HeadPanAt30DegPerSecondWith33MsOffsetMatchesTheRgbTruthOnlyWhenCompensated)
{
  const EdgeResult r = measure(head_pan_pair(PAN_RATE_DEG_S, PAIR_DT_S));
  std::cout << "head pan 30 deg/s, +33 ms: compensated median " << r.compensated_px << " px, static median "
            << r.static_px << " px, " << r.samples << " edge samples\n";
  EXPECT_GE(r.samples, MIN_EDGE_SAMPLES);
  EXPECT_LE(r.compensated_px, COMPENSATED_MAX_MEDIAN_PX);
  EXPECT_GE(r.static_px, STATIC_MIN_MEDIAN_PX);
}

TEST(MotionCompensation, HeadPanTheOtherWayAndRgbBeforeTofAreCompensatedTheSameWay)
{
  // Both signs of the pan rate and of the capture offset, so a convention bug that happens to cancel for one
  // direction is caught.
  for (const double rate : {PAN_RATE_DEG_S, -PAN_RATE_DEG_S}) {
    for (const double dt : {PAIR_DT_S, -PAIR_DT_S}) {
      SCOPED_TRACE(testing::Message() << "pan rate " << rate << " deg/s, t_rgb - t_tof " << dt << " s");
      const EdgeResult r = measure(head_pan_pair(rate, dt));
      EXPECT_GE(r.samples, MIN_EDGE_SAMPLES);
      EXPECT_LE(r.compensated_px, COMPENSATED_MAX_MEDIAN_PX);
      EXPECT_GE(r.static_px, STATIC_MIN_MEDIAN_PX);
    }
  }
}

TEST(MotionCompensation, BaseRotationAtHalfARadianPerSecondWithAStaticHeadIsCompensated)
{
  for (const double rate : {BASE_YAW_RATE_RAD_S, -BASE_YAW_RATE_RAD_S}) {
    SCOPED_TRACE(testing::Message() << "base yaw rate " << rate << " rad/s");
    const EdgeResult r = measure(base_turn_pair(rate, PAIR_DT_S));
    std::cout << "base yaw " << rate << " rad/s, +33 ms: compensated median " << r.compensated_px
              << " px, static median " << r.static_px << " px, " << r.samples << " edge samples\n";
    EXPECT_GE(r.samples, MIN_EDGE_SAMPLES);
    EXPECT_LE(r.compensated_px, COMPENSATED_MAX_MEDIAN_PX);
    EXPECT_GE(r.static_px, STATIC_MIN_MEDIAN_PX);
  }
}

TEST(FrameRate, TenDegreesOverHalfASecondIsTwentyDegPerSecond)
{
  // About an arbitrary axis, from an arbitrary start pose: the rate is the relative rotation angle.
  const Eigen::Isometry3d a = pose(37.0, Eigen::Vector3d(0.3, 1.0, -0.5), Eigen::Vector3d(1.0, 2.0, 0.3));
  const Eigen::Isometry3d b = a * rotation(10.0 * DEG, Eigen::Vector3d(-0.4, 0.2, 1.0));
  const MotionRate r = frame_rate(a, b, 0.5);
  EXPECT_NEAR(r.angular_deg_s, 20.0, 1e-9);
  EXPECT_NEAR(r.linear_m_s, 0.0, 1e-12);
  // Backwards in time order is the same rate.
  EXPECT_NEAR(frame_rate(b, a, 0.5).angular_deg_s, 20.0, 1e-9);
  // A rotation in the fixed frame (pre-multiplied) has the same angle.
  const Eigen::Isometry3d c = rotation(10.0 * DEG, Eigen::Vector3d::UnitZ()) * a;
  EXPECT_NEAR(frame_rate(a, c, 0.5).angular_deg_s, 20.0, 1e-9);
}

TEST(FrameRate, PointOneMetreOverHalfASecondIsPointTwoMetresPerSecond)
{
  const Eigen::Isometry3d a = pose(20.0, Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0.5, -0.2, 0.1));
  const Eigen::Isometry3d b = translation(Eigen::Vector3d(0.06, 0.0, -0.08)) * a;  // 0.1 m in the fixed frame
  const MotionRate r = frame_rate(a, b, 0.5);
  EXPECT_NEAR(r.linear_m_s, 0.2, 1e-12);
  EXPECT_NEAR(r.angular_deg_s, 0.0, 1e-9);
}

TEST(FrameRate, ThrowsOnANonPositiveOrNonFiniteInterval)
{
  const Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  EXPECT_THROW(frame_rate(a, a, 0.0), std::invalid_argument);
  EXPECT_THROW(frame_rate(a, a, -0.05), std::invalid_argument);
  EXPECT_THROW(frame_rate(a, a, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(frame_rate(a, a, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(EvaluatePair, DropsAbove10DegPerSecondAndAcceptsBelow)
{
  const SyncConfig sync;
  const MotionConfig motion;  // max_angular_rate_deg_s 10, max_linear_rate_m_s 0.3
  ASSERT_DOUBLE_EQ(motion.max_angular_rate_deg_s, 10.0);
  const int64_t t_tof = ns(100.0);
  const int64_t t_rgb = t_tof + ns(0.02);
  EXPECT_EQ(evaluate_pair(t_rgb, t_tof, MotionRate{15.0, 0.0}, sync, motion), PairDecision::motion);
  EXPECT_EQ(evaluate_pair(t_rgb, t_tof, MotionRate{5.0, 0.0}, sync, motion), PairDecision::accept);
  EXPECT_EQ(evaluate_pair(t_rgb, t_tof, MotionRate{10.0, 0.3}, sync, motion), PairDecision::accept);  // at the limit
  EXPECT_EQ(evaluate_pair(t_rgb, t_tof, MotionRate{5.0, 0.4}, sync, motion), PairDecision::motion);   // linear
  EXPECT_EQ(
    evaluate_pair(t_rgb, t_tof, MotionRate{std::numeric_limits<double>::quiet_NaN(), 0.0}, sync, motion),
    PairDecision::motion);
}

TEST(EvaluatePair, DropsAPairDtOf70MsWithThe60MsLimitInEitherOrder)
{
  const SyncConfig sync;  // max_pair_dt_s 0.06
  ASSERT_DOUBLE_EQ(sync.max_pair_dt_s, 0.06);
  const MotionConfig motion;
  const int64_t t_tof = ns(100.0);
  EXPECT_EQ(evaluate_pair(t_tof + ns(0.07), t_tof, MotionRate{}, sync, motion), PairDecision::pair_dt);
  EXPECT_EQ(evaluate_pair(t_tof - ns(0.07), t_tof, MotionRate{}, sync, motion), PairDecision::pair_dt);
  EXPECT_EQ(evaluate_pair(t_tof + ns(0.05), t_tof, MotionRate{}, sync, motion), PairDecision::accept);
  EXPECT_EQ(evaluate_pair(t_tof - ns(0.06), t_tof, MotionRate{}, sync, motion), PairDecision::accept);
  // pair_dt is reported before motion.
  EXPECT_EQ(evaluate_pair(t_tof + ns(0.07), t_tof, MotionRate{15.0, 0.0}, sync, motion), PairDecision::pair_dt);
}

TEST(EvaluatePair, CompensateOffKeepsTheGate)
{
  const SyncConfig sync;
  MotionConfig motion;
  motion.compensate = false;
  const int64_t t_tof = ns(100.0);
  EXPECT_EQ(evaluate_pair(t_tof + ns(0.02), t_tof, MotionRate{15.0, 0.0}, sync, motion), PairDecision::motion);
  EXPECT_EQ(evaluate_pair(t_tof + ns(0.07), t_tof, MotionRate{}, sync, motion), PairDecision::pair_dt);
  EXPECT_EQ(evaluate_pair(t_tof + ns(0.02), t_tof, MotionRate{5.0, 0.0}, sync, motion), PairDecision::accept);
}

TEST(DecisionLabel, DropReasonsAreTheMetricLabelsPairDtAndMotion)
{
  EXPECT_EQ(std::string(decision_label(PairDecision::pair_dt)), "pair_dt");
  EXPECT_EQ(std::string(decision_label(PairDecision::motion)), "motion");
  EXPECT_EQ(std::string(decision_label(PairDecision::accept)), "accept");
}
