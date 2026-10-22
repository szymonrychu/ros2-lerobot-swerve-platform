#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/upsample.hpp"
#include "synthetic_scene.hpp"

namespace
{

using head_rgbd_cpp::guided_upsample;
using head_rgbd_cpp::UpsampleStats;
using head_rgbd_cpp::UpsamplingConfig;
namespace synth = head_rgbd_cpp::test;

constexpr double FRONT_M = 1.0;
constexpr double BACK_M = 2.0;
constexpr double SPLIT_MM = 1500.0;
constexpr double FRONT_ALBEDO = 0.8;
constexpr double BACK_ALBEDO = 0.2;
constexpr int PROBE_ROW = 162;
constexpr int EDGE_COL_FIRST = 250;
constexpr int EDGE_COL_LAST = 350;
constexpr int BORDER_PX = 12;
constexpr double EDGE_PX[] = {299.6, 299.9, 300.2, 300.6, 301.0, 301.4};

struct Inputs
{
  cv::Mat depth_mm;   // 288x162 CV_16UC1
  cv::Mat guide_bgr;  // 576x324 CV_8UC3
  cv::Mat truth_m;    // 576x324 CV_64FC1
};

// The three images of a scene seen by the output (288x162) and the mapping (576x324) cameras at the same pose.
Inputs render_inputs(const synth::Scene & scene)
{
  const auto low = synth::default_rgb_out_model();
  const auto high = synth::default_rgb_mapping_model();
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Inputs in;
  in.depth_mm = synth::to_depth_mm(synth::render_rgb_truth_m(scene, low, pose));
  cv::Mat grey8;
  synth::render_grey(scene, high, pose).convertTo(grey8, CV_8UC1, 255.0);
  cv::cvtColor(grey8, in.guide_bgr, cv::COLOR_GRAY2BGR);
  in.truth_m = synth::render_rgb_truth_m(scene, high, pose);
  return in;
}

// A near box (front face at 1 m) whose right edge projects to column edge_px of the 576 image, in front of a far plane
// (2 m); the box is bright, the plane dark, so the RGB edge is sharp.
synth::Scene step_scene(double edge_px)
{
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(BACK_M, BACK_ALBEDO));
  const double x0 = (edge_px - (synth::RGB_MAPPING_WIDTH - 1) / 2.0) / synth::RGB_MAPPING_F * FRONT_M;
  scene.boxes.push_back(
    synth::Box{Eigen::Vector3d(-10.0, -10.0, FRONT_M), Eigen::Vector3d(x0, 10.0, FRONT_M + 0.1), FRONT_ALBEDO});
  return scene;
}

// Number of near pixels (below 1.5 m) in the probe row between the edge columns; -1 when one of them is 0.
int near_count(const cv::Mat & depth_mm)
{
  int n = 0;
  for (int u = EDGE_COL_FIRST; u <= EDGE_COL_LAST; ++u) {
    const auto z = depth_mm.at<uint16_t>(PROBE_ROW, u);
    if (z == 0) {
      return -1;
    }
    n += z < SPLIT_MM ? 1 : 0;
  }
  return n;
}

cv::Mat nearest_upsample(const cv::Mat & depth_mm)
{
  cv::Mat out;
  cv::resize(depth_mm, out, depth_mm.size() * 2, 0.0, 0.0, cv::INTER_NEAREST);
  return out;
}

int truth_near_count(const cv::Mat & truth_m)
{
  cv::Mat mm = synth::to_depth_mm(truth_m);
  return near_count(mm);
}

double rmse_mm(const cv::Mat & depth_mm, const cv::Mat & truth_m)
{
  double sum = 0.0;
  int n = 0;
  for (int v = BORDER_PX; v < truth_m.rows - BORDER_PX; ++v) {
    for (int u = BORDER_PX; u < truth_m.cols - BORDER_PX; ++u) {
      const double t = truth_m.at<double>(v, u) * 1000.0;
      const double z = depth_mm.at<uint16_t>(v, u);
      if (t <= 0.0 || z == 0.0) {
        continue;
      }
      sum += (z - t) * (z - t);
      ++n;
    }
  }
  return n > 0 ? std::sqrt(sum / n) : 1e9;
}

}  // namespace

TEST(GuidedUpsample, StepEdgeFollowsRgbEdgeWithinOnePixel)
{
  const UpsamplingConfig cfg;
  int nearest_worst = 0;
  int guided_worst = 0;
  int nearest_sum = 0;
  int guided_sum = 0;
  for (const double edge_px : EDGE_PX) {
    const Inputs in = render_inputs(step_scene(edge_px));
    const int truth = truth_near_count(in.truth_m);
    const int nearest = near_count(nearest_upsample(in.depth_mm));
    const int guided = near_count(guided_upsample(in.depth_mm, in.guide_bgr, cfg));
    ASSERT_GE(guided, 0) << "edge " << edge_px << ": guided output has holes at the edge";
    const int nearest_err = std::abs(nearest - truth);
    const int guided_err = std::abs(guided - truth);
    std::printf("edge %.1f truth %d nearest err %d guided err %d\n", edge_px, truth, nearest_err, guided_err);
    nearest_worst = std::max(nearest_worst, nearest_err);
    guided_worst = std::max(guided_worst, guided_err);
    nearest_sum += nearest_err;
    guided_sum += guided_err;
  }
  EXPECT_LE(guided_worst, 1);
  EXPECT_GE(nearest_worst, 1);
  EXPECT_LE(nearest_worst, 2);
  EXPECT_LT(guided_sum, nearest_sum);
}

TEST(GuidedUpsample, HoleInteriorStaysZeroAndNeighboursAreNotPulledToZero)
{
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(1.5, 0.5));
  Inputs in = render_inputs(scene);
  // Hole of 10 x 40 low pixels = 20 x 80 high pixels, wider than 2 * radius (8).
  in.depth_mm(cv::Rect(100, 60, 10, 40)).setTo(0);
  const UpsamplingConfig cfg;
  const cv::Mat out = guided_upsample(in.depth_mm, in.guide_bgr, cfg);
  ASSERT_EQ(out.type(), CV_16UC1);
  ASSERT_EQ(out.size(), cv::Size(576, 324));
  // Hole: high columns 200..219, rows 120..199. Pixels at least radius + 1 from every valid pixel are 0.
  for (int v = 120 + cfg.radius + 1; v < 200 - cfg.radius - 1; ++v) {
    for (int u = 200 + cfg.radius + 1; u < 220 - cfg.radius - 1; ++u) {
      ASSERT_EQ(out.at<uint16_t>(v, u), 0) << "(" << u << ", " << v << ")";
    }
  }
  // Valid pixels next to the hole keep the true depth within 2 %.
  for (int u = 190; u < 200; ++u) {
    const double z = out.at<uint16_t>(160, u);
    ASSERT_GT(z, 0.0) << "col " << u;
    EXPECT_NEAR(z, 1500.0, 30.0) << "col " << u;
  }
  for (int u = 220; u < 230; ++u) {
    const double z = out.at<uint16_t>(160, u);
    ASSERT_GT(z, 0.0) << "col " << u;
    EXPECT_NEAR(z, 1500.0, 30.0) << "col " << u;
  }
}

TEST(GuidedUpsample, SlantedPlaneRmseBelowNearest)
{
  synth::Scene scene;
  scene.planes.push_back(synth::slanted_plane(1.5, 40.0, 0.5));
  const Inputs in = render_inputs(scene);
  const double nearest = rmse_mm(nearest_upsample(in.depth_mm), in.truth_m);
  const double guided = rmse_mm(guided_upsample(in.depth_mm, in.guide_bgr, UpsamplingConfig{}), in.truth_m);
  std::printf("slanted rmse nearest %.3f mm guided %.3f mm\n", nearest, guided);
  EXPECT_LT(guided, nearest);
}

TEST(GuidedUpsample, WrongGuideSizeThrows)
{
  const cv::Mat depth(162, 288, CV_16UC1, cv::Scalar(1000));
  const cv::Mat guide(300, 576, CV_8UC3, cv::Scalar::all(100));
  EXPECT_THROW(guided_upsample(depth, guide, UpsamplingConfig{}), std::invalid_argument);
  const cv::Mat same_size(162, 288, CV_8UC3, cv::Scalar::all(100));
  EXPECT_THROW(guided_upsample(depth, same_size, UpsamplingConfig{}), std::invalid_argument);
}

TEST(GuidedUpsample, AllInvalidInputGivesAllZero)
{
  const cv::Mat depth(162, 288, CV_16UC1, cv::Scalar(0));
  const cv::Mat guide(324, 576, CV_8UC3, cv::Scalar::all(100));
  UpsampleStats stats;
  const cv::Mat out = guided_upsample(depth, guide, UpsamplingConfig{}, &stats);
  ASSERT_EQ(out.size(), cv::Size(576, 324));
  EXPECT_EQ(cv::countNonZero(out), 0);
  EXPECT_EQ(stats.valid_in, 0);
  EXPECT_EQ(stats.valid_out, 0);
}

TEST(GuidedUpsample, StatsCountValidPixels)
{
  const cv::Mat depth(162, 288, CV_16UC1, cv::Scalar(1000));
  const cv::Mat guide(324, 576, CV_8UC3, cv::Scalar::all(100));
  UpsampleStats stats;
  const cv::Mat out = guided_upsample(depth, guide, UpsamplingConfig{}, &stats);
  EXPECT_EQ(stats.valid_in, 162 * 288);
  EXPECT_EQ(stats.valid_out, cv::countNonZero(out));
  EXPECT_EQ(stats.valid_out, 324 * 576);
}

namespace
{

constexpr double BOX_M = 0.95;
constexpr double FLOOR_M = 1.0;
constexpr double GUIDE_EDGE_ROW = 162.3;
constexpr int BAND_EDGE_ROW = 81;  // low-resolution row holding the edge (162.3 / 2); the hole band is centred on it
constexpr double EDGE_MARGIN_PX = 2.0;
constexpr double EDGE_TOLERANCE_MM = 5.0;
constexpr double NOISE_AMPLITUDE_MM = 5.0;
constexpr double FLAT_TOLERANCE_MM = 10.0;
constexpr int CHECKER_PX = 16;
constexpr int SEAM_PERIOD_PX = 64;
constexpr int NOISE_SEED = 7;

// A 5 cm box top edge at 1 m: the box front face at 0.95 m above the floor plane at 1.0 m; the edge projects to row
// GUIDE_EDGE_ROW of the 576x324 image and the guide has a sharp step there.
synth::Scene box_edge_scene()
{
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(FLOOR_M, BACK_ALBEDO));
  const double y0 = (GUIDE_EDGE_ROW - (synth::RGB_MAPPING_HEIGHT - 1) / 2.0) / synth::RGB_MAPPING_F * BOX_M;
  scene.boxes.push_back(
    synth::Box{Eigen::Vector3d(-10.0, -10.0, BOX_M), Eigen::Vector3d(10.0, y0, BOX_M + 0.1), FRONT_ALBEDO});
  return scene;
}

}  // namespace

TEST(GuidedUpsample, BoxEdgeFollowsTheGuideEdge)
{
  const Inputs clean = render_inputs(box_edge_scene());
  for (int band = 1; band <= 3; ++band) {
    Inputs in = clean;
    // The hole band filter_jump_edges leaves along the edge (1..3 low-resolution rows centred on the edge row).
    in.depth_mm(cv::Rect(0, BAND_EDGE_ROW - (band - 1) / 2, in.depth_mm.cols, band)).setTo(0);
    const cv::Mat out = guided_upsample(in.depth_mm, in.guide_bgr, UpsamplingConfig{});
    int checked = 0;
    for (int v = 0; v < out.rows; ++v) {
      const double above = GUIDE_EDGE_ROW - v;  // positive on the box side
      if (std::abs(above) < EDGE_MARGIN_PX + 1.0) {
        continue;
      }
      const double expected_mm = (above > 0 ? BOX_M : FLOOR_M) * 1000.0;
      for (int u = BORDER_PX; u < out.cols - BORDER_PX; u += 7) {
        ASSERT_NEAR(out.at<uint16_t>(v, u), expected_mm, EDGE_TOLERANCE_MM)
          << "band " << band << " (" << u << ", " << v << ")";
        ++checked;
      }
    }
    EXPECT_GT(checked, 1000);
  }
}

TEST(GuidedUpsample, TexturedFlatFloorStaysFlat)
{
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(FLOOR_M, 0.5));
  Inputs in = render_inputs(scene);
  // Checkerboard plus tile seams (dark lines) as the guide.
  cv::Mat grey(in.guide_bgr.size(), CV_8UC1);
  for (int v = 0; v < grey.rows; ++v) {
    for (int u = 0; u < grey.cols; ++u) {
      const bool checker = ((u / CHECKER_PX) + (v / CHECKER_PX)) % 2 == 0;
      const bool seam = u % SEAM_PERIOD_PX < 2 || v % SEAM_PERIOD_PX < 2;
      grey.at<uint8_t>(v, u) = seam ? 20 : (checker ? 220 : 60);
    }
  }
  cv::cvtColor(grey, in.guide_bgr, cv::COLOR_GRAY2BGR);
  cv::RNG rng(NOISE_SEED);
  for (int v = 0; v < in.depth_mm.rows; ++v) {
    for (int u = 0; u < in.depth_mm.cols; ++u) {
      const double z = FLOOR_M * 1000.0 + rng.uniform(-NOISE_AMPLITUDE_MM, NOISE_AMPLITUDE_MM);
      in.depth_mm.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(z));
    }
  }
  const cv::Mat out = guided_upsample(in.depth_mm, in.guide_bgr, UpsamplingConfig{});
  double worst = 0.0;
  for (int v = 0; v < out.rows; ++v) {
    for (int u = 0; u < out.cols; ++u) {
      ASSERT_GT(out.at<uint16_t>(v, u), 0) << "(" << u << ", " << v << ")";
      worst = std::max(worst, std::abs(out.at<uint16_t>(v, u) - FLOOR_M * 1000.0));
    }
  }
  std::printf("textured flat floor worst deviation %.2f mm\n", worst);
  EXPECT_LE(worst, FLAT_TOLERANCE_MM);
}

TEST(GuidedUpsample, LargeHoleStaysEmpty)
{
  synth::Scene scene;
  scene.planes.push_back(synth::fronto_parallel_plane(FLOOR_M, 0.5));
  Inputs in = render_inputs(scene);
  // Black ramp: the guide fades to black across the hole; the hole (60 x 50 low px) is far wider than 2 * radius.
  cv::Mat grey(in.guide_bgr.size(), CV_8UC1);
  for (int v = 0; v < grey.rows; ++v) {
    for (int u = 0; u < grey.cols; ++u) {
      grey.at<uint8_t>(v, u) = static_cast<uint8_t>(std::clamp(255 - std::abs(u - 288) * 2, 0, 255));
    }
  }
  cv::cvtColor(grey, in.guide_bgr, cv::COLOR_GRAY2BGR);
  in.depth_mm(cv::Rect(114, 56, 60, 50)).setTo(0);
  const UpsamplingConfig cfg;
  const cv::Mat out = guided_upsample(in.depth_mm, in.guide_bgr, cfg);
  const cv::Mat valid = nearest_upsample(in.depth_mm) > 0;
  cv::Mat distance;
  cv::distanceTransform(~valid, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
  int far_pixels = 0;
  for (int v = 0; v < out.rows; ++v) {
    for (int u = 0; u < out.cols; ++u) {
      if (distance.at<float>(v, u) > static_cast<float>(cfg.radius)) {
        ASSERT_EQ(out.at<uint16_t>(v, u), 0) << "(" << u << ", " << v << ")";
        ++far_pixels;
      }
    }
  }
  EXPECT_GT(far_pixels, 1000);
}
