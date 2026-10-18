#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

#include "head_rgbd_cpp/registration.hpp"
#include "synthetic_scene.hpp"

namespace
{

using head_rgbd_cpp::clip_footprint;
using head_rgbd_cpp::DepthConfig;
using head_rgbd_cpp::PinholeModel;
using head_rgbd_cpp::RayTable;
using head_rgbd_cpp::register_depth;
using head_rgbd_cpp::register_depth_into;
using head_rgbd_cpp::RegistrationStats;
using head_rgbd_cpp::scale_model;
namespace synth = head_rgbd_cpp::test;

constexpr double BASELINE_M = 0.05;
constexpr uint16_t GUARD_VALUE = 0xBEEF;
constexpr int GUARD_PX = 7;

// A model with plumb_bob distortion.
PinholeModel distorted(PinholeModel m, double k1, double k2, double p1, double p2, double k3)
{
  m.D = cv::Vec<double, 5>(k1, k2, p1, p2, k3);
  return m;
}

// A rotation about the y axis (positive turns +z towards +x).
Eigen::Isometry3d rotation_y(double angle_deg)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(angle_deg * CV_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return T;
}

// cv::projectPoints of one point in the camera frame.
cv::Point2d cv_project(const PinholeModel & m, const Eigen::Vector3d & p)
{
  std::vector<cv::Point3d> object{cv::Point3d(p.x(), p.y(), p.z())};
  std::vector<cv::Point2d> image;
  cv::projectPoints(object, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Mat(m.K), cv::Mat(m.D), image);
  return image.front();
}

// The bounding rectangle of the non-zero pixels of a CV_16UC1 image (empty when all zero).
cv::Rect nonzero_rect(const cv::Mat & img)
{
  std::vector<cv::Point> points;
  cv::findNonZero(img, points);
  return points.empty() ? cv::Rect() : cv::boundingRect(points);
}

// Mean column of the non-zero pixels.
double mean_nonzero_column(const cv::Mat & img)
{
  double sum = 0.0;
  int n = 0;
  for (int v = 0; v < img.rows; ++v) {
    for (int u = 0; u < img.cols; ++u) {
      if (img.at<uint16_t>(v, u) != 0) {
        sum += u;
        ++n;
      }
    }
  }
  return n == 0 ? std::numeric_limits<double>::quiet_NaN() : sum / n;
}

// A CV_8UC1 mask eroded by one pixel (3x3), pixels outside the image count as not set.
cv::Mat erode1(const cv::Mat & mask)
{
  cv::Mat out;
  cv::erode(mask, out, cv::Mat::ones(3, 3, CV_8UC1), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  return out;
}

// A ToF depth with a single valid pixel.
cv::Mat single_pixel_depth(const PinholeModel & tof, int u, int v, uint16_t depth_mm)
{
  cv::Mat depth(tof.height, tof.width, CV_16UC1, cv::Scalar(0));
  depth.at<uint16_t>(v, u) = depth_mm;
  return depth;
}

// Register into a sub-Mat view of a larger Mat filled with GUARD_VALUE and check the guard band stays untouched.
// @return the registered view (a copy).
cv::Mat register_with_guard_band(
  const cv::Mat & tof_depth, const RayTable & rays, const Eigen::Isometry3d & T, const PinholeModel & rgb)
{
  cv::Mat big(rgb.height + 2 * GUARD_PX, rgb.width + 2 * GUARD_PX, CV_16UC1, cv::Scalar(GUARD_VALUE));
  cv::Mat view = big(cv::Rect(GUARD_PX, GUARD_PX, rgb.width, rgb.height));
  register_depth_into(tof_depth, rays, T, rgb, DepthConfig{}, view);
  int touched = 0;
  for (int v = 0; v < big.rows; ++v) {
    for (int u = 0; u < big.cols; ++u) {
      const bool inside = u >= GUARD_PX && u < GUARD_PX + rgb.width && v >= GUARD_PX && v < GUARD_PX + rgb.height;
      if (!inside && big.at<uint16_t>(v, u) != GUARD_VALUE) {
        ++touched;
      }
    }
  }
  EXPECT_EQ(touched, 0) << "pixels written outside the output view";
  return view.clone();
}

}  // namespace

// --- scale_model ---------------------------------------------------------------------------------------------------

TEST(ScaleModel, HalvesTheMappingIntrinsicsWithThePixelCentreConventionAndKeepsDistortion)
{
  const PinholeModel m = distorted(synth::default_rgb_mapping_model(), -0.1, 0.02, 0.001, -0.002, 0.003);
  const PinholeModel s = scale_model(m, 288, 162);
  EXPECT_EQ(s.width, 288);
  EXPECT_EQ(s.height, 162);
  EXPECT_DOUBLE_EQ(s.K(0, 0), 220.0);
  EXPECT_DOUBLE_EQ(s.K(1, 1), 220.0);
  EXPECT_DOUBLE_EQ(s.K(0, 2), (m.K(0, 2) + 0.5) * 0.5 - 0.5);
  EXPECT_DOUBLE_EQ(s.K(1, 2), (m.K(1, 2) + 0.5) * 0.5 - 0.5);
  EXPECT_DOUBLE_EQ(s.K(2, 2), 1.0);
  for (int i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(s.D[i], m.D[i]);
  }
  // The same model at its own size is unchanged.
  const PinholeModel same = scale_model(m, m.width, m.height);
  EXPECT_DOUBLE_EQ(same.K(0, 2), m.K(0, 2));
  EXPECT_DOUBLE_EQ(same.K(0, 0), m.K(0, 0));
}

TEST(ScaleModel, AcceptsRoundingOfOddSizesAndThrowsOnAnAspectChangeAboveOnePixel)
{
  const PinholeModel m = synth::default_rgb_mapping_model();  // 576x324
  EXPECT_NO_THROW(scale_model(m, 192, 108));
  EXPECT_NO_THROW(scale_model(m, 288, 163));  // 1 px of rounding
  EXPECT_THROW(scale_model(m, 288, 165), std::invalid_argument);
  EXPECT_THROW(scale_model(m, 300, 162), std::invalid_argument);
  EXPECT_THROW(scale_model(m, 288, 216), std::invalid_argument);  // 4:3 from 16:9
  EXPECT_THROW(scale_model(m, 0, 162), std::invalid_argument);
  EXPECT_THROW(scale_model(m, 288, -1), std::invalid_argument);
  PinholeModel empty = m;
  empty.width = 0;
  EXPECT_THROW(scale_model(empty, 288, 162), std::invalid_argument);
}

// --- projection and rays -------------------------------------------------------------------------------------------

TEST(ProjectPoint, EqualsCvProjectPointsWithPlumbBobDistortion)
{
  const PinholeModel m = distorted(synth::default_rgb_out_model(), -0.21, 0.07, 0.0015, -0.0012, -0.01);
  double max_err = 0.0;
  for (double x = -0.7; x <= 0.7; x += 0.07) {
    for (double y = -0.45; y <= 0.45; y += 0.05) {
      for (double z : {0.3, 1.0, 3.7}) {
        const Eigen::Vector3d p(x * z, y * z, z);
        const cv::Point2d mine = head_rgbd_cpp::project_point(m, p);
        const cv::Point2d ref = cv_project(m, p);
        max_err = std::max(max_err, std::hypot(mine.x - ref.x, mine.y - ref.y));
      }
    }
  }
  EXPECT_LT(max_err, 1e-6);
}

TEST(RayTable, UndistortsEveryPixelCentreAndCornerSoThatTheyProjectBack)
{
  const PinholeModel m = distorted(synth::default_tof_model(), -0.08, 0.015, 0.0008, -0.0005, 0.0);
  const RayTable rays(m);
  ASSERT_EQ(rays.width(), m.width);
  ASSERT_EQ(rays.height(), m.height);
  ASSERT_EQ(rays.centres().rows, m.height);
  ASSERT_EQ(rays.centres().cols, m.width);
  ASSERT_EQ(rays.corners().rows, m.height + 1);
  ASSERT_EQ(rays.corners().cols, m.width + 1);
  double max_err = 0.0;
  for (int v = 0; v <= m.height; v += 9) {
    for (int u = 0; u <= m.width; u += 8) {
      if (u < m.width && v < m.height) {
        const cv::Vec2d & r = rays.ray(u, v);
        const cv::Point2d back = head_rgbd_cpp::project_point(m, Eigen::Vector3d(r[0], r[1], 1.0));
        max_err = std::max(max_err, std::hypot(back.x - u, back.y - v));
      }
      const cv::Vec2d & c = rays.corner(u, v);
      const cv::Point2d back = head_rgbd_cpp::project_point(m, Eigen::Vector3d(c[0], c[1], 1.0));
      max_err = std::max(max_err, std::hypot(back.x - (u - 0.5), back.y - (v - 0.5)));
    }
  }
  EXPECT_LT(max_err, 1e-4);
  // Without distortion the ray is ((u - cx) / fx, (v - cy) / fy).
  const PinholeModel plain = synth::default_tof_model();
  const RayTable plain_rays(plain);
  EXPECT_NEAR(plain_rays.ray(10, 20)[0], (10 - plain.K(0, 2)) / plain.K(0, 0), 1e-12);
  EXPECT_NEAR(plain_rays.ray(10, 20)[1], (20 - plain.K(1, 2)) / plain.K(1, 1), 1e-12);
  EXPECT_NEAR(plain_rays.corner(0, 0)[0], (-0.5 - plain.K(0, 2)) / plain.K(0, 0), 1e-12);
}

TEST(FoldRadius, IsWhereTheRadialDistortionStopsIncreasingAndInfiniteWithoutDistortion)
{
  // r (1 + k1 r^2) peaks at r^2 = -1 / (3 k1).
  const PinholeModel m = distorted(synth::default_rgb_out_model(), -0.5, 0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(head_rgbd_cpp::fold_radius_squared(m), 1.0 / 1.5, 1e-3);
  EXPECT_TRUE(std::isinf(head_rgbd_cpp::fold_radius_squared(synth::default_rgb_out_model())));
}

// --- clip_footprint ------------------------------------------------------------------------------------------------

TEST(ClipFootprint, CoversThePixelCentresInsideAndClipsAtEachOfTheFourBorders)
{
  const int w = 288;
  const int h = 162;
  // Inside: centres 10..12 x 20..21.
  EXPECT_EQ(clip_footprint(9.6, 19.7, 12.4, 21.2, w, h), cv::Rect(10, 20, 3, 2));
  // One pixel: the footprint of pixel (5, 6) under the identity.
  EXPECT_EQ(clip_footprint(4.5, 5.5, 5.5, 6.5, w, h), cv::Rect(5, 6, 1, 1));
  // Left border crossed.
  EXPECT_EQ(clip_footprint(-3.2, 10.0, 1.7, 11.0, w, h), cv::Rect(0, 10, 2, 2));
  // Right border crossed.
  EXPECT_EQ(clip_footprint(285.6, 10.0, 291.0, 10.4, w, h), cv::Rect(286, 10, 2, 1));
  // Top border crossed.
  EXPECT_EQ(clip_footprint(3.0, -2.5, 3.0, 0.2, w, h), cv::Rect(3, 0, 1, 1));
  // Bottom border crossed.
  EXPECT_EQ(clip_footprint(3.0, 159.5, 4.0, 170.0, w, h), cv::Rect(3, 160, 2, 2));
  // Larger than the image on every side.
  EXPECT_EQ(clip_footprint(-1e9, -1e9, 1e9, 1e9, w, h), cv::Rect(0, 0, w, h));
}

TEST(ClipFootprint, IsEmptyWhenFullyOutsideAnySideOrBetweenPixelCentres)
{
  const int w = 288;
  const int h = 162;
  EXPECT_TRUE(clip_footprint(-5.0, 10.0, -0.1, 12.0, w, h).empty());    // u_max < 0
  EXPECT_TRUE(clip_footprint(288.0, 10.0, 290.0, 12.0, w, h).empty());  // u_min >= width
  EXPECT_TRUE(clip_footprint(287.2, 10.0, 290.0, 12.0, w, h).empty());  // no centre >= u_min inside
  EXPECT_TRUE(clip_footprint(10.0, -4.0, 12.0, -0.5, w, h).empty());    // v_max < 0
  EXPECT_TRUE(clip_footprint(10.0, 162.0, 12.0, 170.0, w, h).empty());  // v_min >= height
  EXPECT_TRUE(clip_footprint(10.2, 10.0, 10.8, 12.0, w, h).empty());    // between two centres
  EXPECT_TRUE(clip_footprint(1e12, 1e12, 2e12, 2e12, w, h).empty());    // far outside, no overflow
  EXPECT_TRUE(clip_footprint(-2e12, -2e12, -1e12, -1e12, w, h).empty());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(clip_footprint(nan, 1.0, 5.0, 5.0, w, h).empty());
  EXPECT_TRUE(clip_footprint(1.0, 1.0, 5.0, nan, w, h).empty());
}

// --- register_depth ------------------------------------------------------------------------------------------------

TEST(RegisterDepth, IdenticalModelsAndIdentityTransformReproduceTheInputExactly)
{
  for (const PinholeModel & m :
       {synth::default_tof_model(), distorted(synth::default_tof_model(), -0.06, 0.012, 0.0007, -0.0004, 0.0)}) {
    cv::Mat depth(m.height, m.width, CV_16UC1);
    cv::RNG rng(42);
    rng.fill(depth, cv::RNG::UNIFORM, 100, 4001);
    for (int v = 0; v < m.height; ++v) {
      for (int u = 0; u < m.width; ++u) {
        if (rng.uniform(0, 10) == 0) {
          depth.at<uint16_t>(v, u) = 0;
        }
      }
    }
    const RayTable rays(m);
    const synth::Rig rig = synth::identity_rig(m);
    RegistrationStats stats;
    const cv::Mat out = register_depth(depth, rays, rig.T_rgb_tof(), rig.rgb, DepthConfig{}, &stats);
    ASSERT_EQ(out.type(), CV_16UC1);
    ASSERT_EQ(out.size(), depth.size());
    EXPECT_EQ(cv::countNonZero(out != depth), 0);
    const auto valid = static_cast<std::size_t>(cv::countNonZero(depth));
    EXPECT_EQ(stats.valid_points, valid);
    EXPECT_EQ(stats.splatted_points, valid);
    EXPECT_EQ(stats.filled_pixels, valid);
    EXPECT_EQ(stats.behind_points + stats.outside_points + stats.folded_points, 0U);
  }
}

TEST(RegisterDepth, FrontoParallelPlaneShiftsByTheBaselineAndKeepsItsDepth)
{
  // A plane patch at 1 m (the front of a box); the RGB 0.05 m right of the ToF sees it fx_out * 0.05 / 1.0 = 11 px
  // further left than an RGB at the ToF position with the same model.
  synth::Scene scene;
  scene.boxes.push_back(synth::Box{Eigen::Vector3d(-0.3, -0.2, 1.0), Eigen::Vector3d(0.3, 0.2, 1.2)});
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = synth::default_rgb_out_model();
  const cv::Mat tof_depth = synth::render_tof_depth_mm(scene, tof, Eigen::Isometry3d::Identity());
  const RayTable rays(tof);

  const synth::Rig at_tof = synth::offset_rig(tof, rgb, Eigen::Isometry3d::Identity());
  const synth::Rig offset = synth::offset_rig(tof, rgb, synth::translation(BASELINE_M, 0.0, 0.0));
  const cv::Mat reference = register_depth(tof_depth, rays, at_tof.T_rgb_tof(), rgb, DepthConfig{});
  const cv::Mat shifted = register_depth(tof_depth, rays, offset.T_rgb_tof(), rgb, DepthConfig{});

  const double expected_shift = -rgb.K(0, 0) * BASELINE_M / 1.0;
  ASSERT_DOUBLE_EQ(expected_shift, -11.0);
  ASSERT_GT(cv::countNonZero(shifted), 1000);
  EXPECT_NEAR(mean_nonzero_column(shifted) - mean_nonzero_column(reference), expected_shift, 0.5);
  const auto shift_px = static_cast<int>(std::lround(-expected_shift));
  int mismatched = 0;
  for (int v = 0; v < rgb.height; ++v) {
    for (int u = 0; u + shift_px < rgb.width; ++u) {
      if ((shifted.at<uint16_t>(v, u) != 0) != (reference.at<uint16_t>(v, u + shift_px) != 0)) {
        ++mismatched;
      }
    }
  }
  EXPECT_EQ(mismatched, 0);
  // Every registered pixel holds the plane's depth in the RGB frame.
  cv::Mat valid = shifted != 0;
  EXPECT_EQ(cv::countNonZero((shifted != 1000) & valid), 0);
}

TEST(RegisterDepth, SlantedPlaneMatchesTheRenderedRgbGroundTruth)
{
  synth::Scene scene;
  scene.planes.push_back(synth::slanted_plane(1.5, 35.0));
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = synth::default_rgb_out_model();
  const synth::Rig rig = synth::offset_rig(tof, rgb, synth::translation(BASELINE_M, 0.01, -0.005) * rotation_y(2.0));
  const cv::Mat tof_depth = synth::render_tof_depth_mm(scene, tof, rig.T_world_tof);
  const cv::Mat truth = synth::render_rgb_truth_m(scene, rgb, rig.T_world_rgb);
  const cv::Mat out = register_depth(tof_depth, RayTable(tof), rig.T_rgb_tof(), rgb, DepthConfig{});

  int covered = 0;
  int within = 0;
  for (int v = 0; v < rgb.height; ++v) {
    for (int u = 0; u < rgb.width; ++u) {
      const uint16_t z = out.at<uint16_t>(v, u);
      const double t = truth.at<double>(v, u);
      if (z == 0 || t <= 0.0) {
        continue;
      }
      ++covered;
      if (std::abs(z / 1000.0 - t) <= 0.005) {
        ++within;
      }
    }
  }
  ASSERT_GT(covered, rgb.width * rgb.height / 2);
  EXPECT_GE(within, 0.95 * covered) << within << " of " << covered;
}

TEST(RegisterDepth, OcclusionKeepsTheNearestDepthAndLeavesTheShadowEmpty)
{
  // A box at 0.8 m in front of a wall at 2.5 m; an RGB 0.05 m right (then left) of the ToF sees a strip of wall right
  // (left) of the box that the box hides from the ToF (the occlusion shadow), and the wall the ToF sees just left
  // (right) of the box lands behind the box in the RGB. Both sides, so the z-buffer cannot pass by scan order.
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(2.5));                                              // index 0
  scene.boxes.push_back(synth::Box{Eigen::Vector3d(-0.15, -0.1, 0.8), Eigen::Vector3d(0.15, 0.1, 0.9)});  // 1
  const int wall = 0;
  const int box = 1;
  const uint16_t box_mm = 800;
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = synth::default_rgb_out_model();
  const RayTable tof_rays(tof);
  const RayTable rgb_rays(rgb);
  for (const double offset : {BASELINE_M, -BASELINE_M}) {
    SCOPED_TRACE(offset);
    const synth::Rig rig = synth::offset_rig(tof, rgb, synth::translation(offset, 0.0, 0.0));
    const synth::Render tof_view = synth::render(scene, tof, rig.T_world_tof);
    const cv::Mat tof_depth = synth::to_depth_mm(tof_view.depth_m);
    const cv::Mat out = register_depth(tof_depth, tof_rays, rig.T_rgb_tof(), rgb, DepthConfig{});

    // Every output pixel a box footprint lands on holds box depth: nearest wins over the wall footprints.
    cv::Mat box_only;
    tof_depth.copyTo(box_only, tof_view.index == box);
    const cv::Mat box_footprints = register_depth(box_only, tof_rays, rig.T_rgb_tof(), rgb, DepthConfig{});
    ASSERT_GT(cv::countNonZero(box_footprints), 500);
    EXPECT_EQ(cv::countNonZero((out != box_mm) & (box_footprints != 0)), 0);

    // Every pixel where the RGB sees the box (away from its outline) holds box depth.
    const synth::Render truth = synth::render(scene, rgb, rig.T_world_rgb);
    const cv::Mat box_seen = erode1(truth.index == box);
    ASSERT_GT(cv::countNonZero(box_seen), 500);
    EXPECT_EQ(cv::countNonZero((out != box_mm) & box_seen), 0);

    // The shadow: wall the RGB sees and the ToF does not.
    cv::Mat shadow(rgb.height, rgb.width, CV_8UC1, cv::Scalar(0));
    for (int v = 0; v < rgb.height; ++v) {
      for (int u = 0; u < rgb.width; ++u) {
        const auto [point, index] = synth::hit_point(scene, rgb_rays, rig.T_world_rgb, u, v);
        if (index == wall && !synth::visible(scene, rig.T_world_tof.translation(), point)) {
          shadow.at<uint8_t>(v, u) = 255;
        }
      }
    }
    // About 0.106 m of wall at 2.5 m: 9 px wide, as tall as the box.
    ASSERT_GT(cv::countNonZero(shadow), 8 * 40);
    int wall_in_shadow = 0;
    for (int v = 0; v < rgb.height; ++v) {
      for (int u = 0; u < rgb.width; ++u) {
        const uint16_t z = out.at<uint16_t>(v, u);
        if (shadow.at<uint8_t>(v, u) != 0 && z != 0 && z != box_mm) {
          ++wall_in_shadow;
        }
      }
    }
    EXPECT_EQ(wall_in_shadow, 0) << "wall depth written into the occlusion shadow";
    const cv::Mat shadow_core = erode1(shadow);
    ASSERT_GT(cv::countNonZero(shadow_core), 6 * 30);
    EXPECT_EQ(cv::countNonZero((out != 0) & shadow_core), 0);
    // The visible wall away from the box holds wall depth.
    EXPECT_EQ(out.at<uint16_t>(rgb.height / 2, 30), 2500);
    EXPECT_EQ(out.at<uint16_t>(rgb.height / 2, rgb.width - 31), 2500);
  }
}

TEST(RegisterDepth, PlanePartlyOutsideTheRgbViewFillsTheBorderColumnsAndWritesNothingOutsideTheMat)
{
  // A wall at 1.5 m seen by an RGB 0.3 m right (then left) of the ToF: the ToF field of view extends past the left
  // (right) RGB border and past the top and bottom borders, and ends inside the image on the other side.
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(1.5));
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = synth::default_rgb_out_model();
  const RayTable tof_rays(tof);
  const RayTable rgb_rays(rgb);
  const cv::Mat tof_depth = synth::render_tof_depth_mm(scene, tof, Eigen::Isometry3d::Identity());
  for (const double offset : {0.3, -0.3}) {
    SCOPED_TRACE(offset);
    const synth::Rig rig = synth::offset_rig(tof, rgb, synth::translation(offset, 0.0, 0.0));
    const cv::Mat out = register_with_guard_band(tof_depth, tof_rays, rig.T_rgb_tof(), rgb);
    EXPECT_EQ(cv::countNonZero(out != register_depth(tof_depth, tof_rays, rig.T_rgb_tof(), rgb, DepthConfig{})), 0);

    const int covered_col = offset > 0 ? 0 : rgb.width - 1;
    const int empty_col = offset > 0 ? rgb.width - 1 : 0;
    for (int v = 0; v < rgb.height; ++v) {
      EXPECT_EQ(out.at<uint16_t>(v, covered_col), 1500) << "row " << v;
      EXPECT_EQ(out.at<uint16_t>(v, empty_col), 0) << "row " << v;
    }
    // Every pixel more than one ToF pixel inside the ToF view holds the wall, every pixel more than one outside is 0.
    int wrong = 0;
    for (int v = 0; v < rgb.height; ++v) {
      for (int u = 0; u < rgb.width; ++u) {
        const auto hit = synth::hit_point(scene, rgb_rays, rig.T_world_rgb, u, v);
        const cv::Point2d px = head_rgbd_cpp::project_point(tof, hit.first);
        const double inside_margin =
          std::min({px.x + 0.5, tof.width - 0.5 - px.x, px.y + 0.5, tof.height - 0.5 - px.y});
        const uint16_t z = out.at<uint16_t>(v, u);
        if ((inside_margin > 1.0 && z != 1500) || (inside_margin < -1.0 && z != 0)) {
          ++wrong;
        }
      }
    }
    EXPECT_EQ(wrong, 0);
  }
}

TEST(RegisterDepth, DefaultOutputFromTheDefaultTofCoversTheOverlapOnAWall)
{
  // 240x180 ToF (f 191) into 288x162 (f 220): about 1:1.15, the footprint splat closes the gaps.
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(1.5));
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = synth::default_rgb_out_model();
  ASSERT_EQ(rgb.width, 288);
  ASSERT_EQ(rgb.height, 162);
  const synth::Rig rig = synth::offset_rig(tof, rgb, synth::translation(BASELINE_M, 0.0, 0.0));
  const cv::Mat tof_depth = synth::render_tof_depth_mm(scene, tof, rig.T_world_tof);
  const cv::Mat out = register_depth(tof_depth, RayTable(tof), rig.T_rgb_tof(), rgb, DepthConfig{});
  const RayTable rgb_rays(rgb);
  int overlap = 0;
  int covered = 0;
  for (int v = 0; v < rgb.height; ++v) {
    for (int u = 0; u < rgb.width; ++u) {
      const auto hit = synth::hit_point(scene, rgb_rays, rig.T_world_rgb, u, v);
      if (hit.second != synth::NO_HIT && synth::in_view(tof, rig.T_world_tof, hit.first)) {
        ++overlap;
        covered += out.at<uint16_t>(v, u) != 0 ? 1 : 0;
      }
    }
  }
  ASSERT_GT(overlap, rgb.width * rgb.height / 2);
  EXPECT_GE(covered, 0.9 * overlap) << covered << " of " << overlap;
}

TEST(RegisterDepth, RgbDistortionMovesPointsToTheCvProjectPointsPositions)
{
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb_plain = synth::default_rgb_out_model();
  const PinholeModel rgb = distorted(rgb_plain, -0.18, 0.04, 0.0, 0.0, 0.0);
  const RayTable rays(tof);
  const synth::Rig rig = synth::offset_rig(tof, rgb, synth::translation(BASELINE_M, 0.0, 0.0));
  const Eigen::Isometry3d T = rig.T_rgb_tof();
  const uint16_t depth_mm = 1200;
  const double z = depth_mm / 1000.0;
  const double tol = 0.05;
  for (const cv::Point & tof_px : {cv::Point(20, 15), cv::Point(220, 160), cv::Point(120, 90), cv::Point(30, 150)}) {
    SCOPED_TRACE(testing::Message() << tof_px);
    const cv::Mat out =
      register_depth(single_pixel_depth(tof, tof_px.x, tof_px.y, depth_mm), rays, T, rgb, DepthConfig{});
    // The footprint expected from cv::projectPoints of the pixel's four corners at the same depth.
    double u_min = 1e9;
    double v_min = 1e9;
    double u_max = -1e9;
    double v_max = -1e9;
    for (const cv::Point & c : {tof_px, tof_px + cv::Point(1, 0), tof_px + cv::Point(0, 1), tof_px + cv::Point(1, 1)}) {
      const cv::Vec2d & r = rays.corner(c.x, c.y);
      const cv::Point2d px = cv_project(rgb, T * Eigen::Vector3d(r[0] * z, r[1] * z, z));
      u_min = std::min(u_min, px.x);
      v_min = std::min(v_min, px.y);
      u_max = std::max(u_max, px.x);
      v_max = std::max(v_max, px.y);
    }
    const cv::Rect inner = clip_footprint(u_min + tol, v_min + tol, u_max - tol, v_max - tol, rgb.width, rgb.height);
    const cv::Rect outer = clip_footprint(u_min - tol, v_min - tol, u_max + tol, v_max + tol, rgb.width, rgb.height);
    const cv::Rect got = nonzero_rect(out);
    ASSERT_FALSE(got.empty());
    EXPECT_EQ(got & inner, inner);
    EXPECT_EQ(got & outer, got);
    EXPECT_EQ(cv::countNonZero(out), got.area());
    // The distortion matters here: the undistorted projection of the centre lies outside the footprint.
    const cv::Vec2d & r = rays.ray(tof_px.x, tof_px.y);
    const Eigen::Vector3d p = T * Eigen::Vector3d(r[0] * z, r[1] * z, z);
    const cv::Point2d distorted_px = cv_project(rgb, p);
    const cv::Point2d plain_px = cv_project(rgb_plain, p);
    if (tof_px != cv::Point(120, 90)) {
      EXPECT_GT(std::hypot(distorted_px.x - plain_px.x, distorted_px.y - plain_px.y), 2.0);
      EXPECT_FALSE(
        got.contains(cv::Point(static_cast<int>(std::lround(plain_px.x)), static_cast<int>(std::lround(plain_px.y)))));
    }
    EXPECT_TRUE(got.contains(
      cv::Point(static_cast<int>(std::lround(distorted_px.x)), static_cast<int>(std::lround(distorted_px.y)))));
  }
}

TEST(RegisterDepth, SkipsPointsWhereTheRgbDistortionFoldsBack)
{
  // k1 = -0.5 folds at r = 0.816; a point 50 deg off the RGB axis (r = 1.19) would be drawn by the polynomial at
  // r_d = 0.34, inside the image.
  const PinholeModel tof = synth::default_tof_model();
  const PinholeModel rgb = distorted(synth::default_rgb_out_model(), -0.5, 0.0, 0.0, 0.0, 0.0);
  const Eigen::Isometry3d T = rotation_y(50.0);
  const cv::Point2d folded = cv_project(rgb, T * Eigen::Vector3d(0.0, 0.0, 1.0));
  ASSERT_GT(folded.x, 0.0);
  ASSERT_LT(folded.x, rgb.width - 1.0);
  RegistrationStats stats;
  const cv::Mat out =
    register_depth(single_pixel_depth(tof, 120, 90, 1000), RayTable(tof), T, rgb, DepthConfig{}, &stats);
  EXPECT_EQ(cv::countNonZero(out), 0);
  EXPECT_EQ(stats.valid_points, 1U);
  EXPECT_EQ(stats.folded_points, 1U);
  EXPECT_EQ(stats.splatted_points, 0U);
}

TEST(RegisterDepth, SkipsDepthsOutsideTheRangeAndPointsBehindTheRgbCamera)
{
  const PinholeModel m = synth::default_tof_model();
  const RayTable rays(m);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  DepthConfig range;  // 0.1 .. 4.0 m
  cv::Mat depth(m.height, m.width, CV_16UC1, cv::Scalar(0));
  depth(cv::Rect(0, 0, 50, 50)).setTo(99);
  depth(cv::Rect(60, 0, 50, 50)).setTo(4001);
  depth(cv::Rect(120, 0, 50, 50)).setTo(100);
  depth(cv::Rect(180, 0, 50, 50)).setTo(4000);
  RegistrationStats stats;
  const cv::Mat out = register_depth(depth, rays, identity, m, range, &stats);
  EXPECT_EQ(cv::countNonZero(out(cv::Rect(0, 0, 50, 50))), 0);
  EXPECT_EQ(cv::countNonZero(out(cv::Rect(60, 0, 50, 50))), 0);
  EXPECT_EQ(cv::countNonZero(out(cv::Rect(120, 0, 50, 50)) != 100), 0);
  EXPECT_EQ(cv::countNonZero(out(cv::Rect(180, 0, 50, 50)) != 4000), 0);
  EXPECT_EQ(stats.valid_points, 2U * 50U * 50U);

  // The RGB 2 m in front of a wall at 1.5 m: every point is behind it.
  cv::Mat wall(m.height, m.width, CV_16UC1, cv::Scalar(1500));
  stats = RegistrationStats{};
  const cv::Mat behind = register_depth(wall, rays, synth::translation(0.0, 0.0, -2.0), m, range, &stats);
  EXPECT_EQ(cv::countNonZero(behind), 0);
  EXPECT_EQ(stats.behind_points, static_cast<std::size_t>(m.width * m.height));
  EXPECT_EQ(stats.filled_pixels, 0U);
}

TEST(RegisterDepth, CountsPointsOutsideTheRgbImage)
{
  const PinholeModel m = synth::default_tof_model();
  cv::Mat wall(m.height, m.width, CV_16UC1, cv::Scalar(1000));
  RegistrationStats stats;
  // The RGB turned 80 deg away: every point lands behind it or outside its image.
  const cv::Mat out = register_depth(wall, RayTable(m), rotation_y(-80.0), m, DepthConfig{}, &stats);
  EXPECT_EQ(cv::countNonZero(out), 0);
  EXPECT_EQ(stats.valid_points, static_cast<std::size_t>(m.width * m.height));
  EXPECT_EQ(stats.outside_points + stats.behind_points, stats.valid_points);
  EXPECT_GT(stats.outside_points, 0U);
  EXPECT_EQ(stats.splatted_points, 0U);
}

TEST(RegisterDepth, RejectsInputsOfTheWrongTypeOrSize)
{
  const PinholeModel m = synth::default_tof_model();
  const RayTable rays(m);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  EXPECT_THROW(
    register_depth(cv::Mat(m.height, m.width, CV_32FC1, cv::Scalar(1.0)), rays, identity, m, DepthConfig{}),
    std::invalid_argument);
  EXPECT_THROW(
    register_depth(cv::Mat(m.height + 1, m.width, CV_16UC1, cv::Scalar(0)), rays, identity, m, DepthConfig{}),
    std::invalid_argument);
  cv::Mat wrong_out(m.height, m.width + 1, CV_16UC1);
  EXPECT_THROW(
    register_depth_into(
      cv::Mat(m.height, m.width, CV_16UC1, cv::Scalar(0)), rays, identity, m, DepthConfig{}, wrong_out),
    std::invalid_argument);
  PinholeModel empty = m;
  empty.height = 0;
  EXPECT_THROW(RayTable{empty}, std::invalid_argument);
}
