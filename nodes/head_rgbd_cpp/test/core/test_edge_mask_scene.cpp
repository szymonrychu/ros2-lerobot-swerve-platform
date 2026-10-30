// The shipped edge_mask settings on synthetic D435 depth of the ground box scene (synthetic_d435.hpp): the mixed
// pixels at the 5 cm box edges go, the floor at 0.3-2.0 m and the box top stay (the numbers of the epic task
// d435-edge-and-flying-pixel-f, mixed pixels >= 95 % removed). Both depths the node filters are covered: the full-FOV
// depth camera (the filtered stream of the obstacle cloud) and the colour camera geometry (the aligned depth of
// /head_rgbd), at the home pose and at the recorded box-scene pose.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

#include "head_rgbd_cpp/config.hpp"
#include "ros2_common_cpp/depth_mask.hpp"
#include "synthetic_d435.hpp"

namespace
{

using d435_test::Frame;
using d435_test::View;

constexpr double MIN_REMOVED_MIXED = 0.95;
constexpr double MIN_KEPT_FLOOR = 0.95;
constexpr double MIN_KEPT_BOX_TOP = 0.95;
constexpr double FLOOR_MIN_M = 0.3;
constexpr double FLOOR_MAX_M = 2.0;
// Enough pixels of each kind that the fractions mean something.
constexpr int MIN_MIXED = 100;
constexpr int MIN_BOX_TOP = 1000;
constexpr int MIN_FLOOR = 10000;

struct Counts
{
  int mixed = 0;
  int mixed_removed = 0;
  int floor = 0;
  int floor_kept = 0;
  int box_top = 0;
  int box_top_kept = 0;
};

head_rgbd_cpp::Config shipped_config()
{
  return head_rgbd_cpp::from_yaml(YAML::LoadFile(std::string(HEAD_RGBD_CONFIG_DIR) + "/head_rgbd.yaml"));
}

// Counts what the shipped mask keeps of each surface of a rendered frame.
Counts filter_and_count(const Frame & frame)
{
  const head_rgbd_cpp::Config config = shipped_config();
  cv::Mat depth = frame.depth_mm.clone();
  if (config.edge_mask.enabled) {
    ros2_common_cpp::apply_depth_edge_mask(depth, config.edge_mask);
  }
  Counts c;
  for (int v = 0; v < depth.rows; ++v) {
    for (int u = 0; u < depth.cols; ++u) {
      const bool kept = depth.at<uint16_t>(v, u) != 0;
      switch (frame.label.at<uint8_t>(v, u)) {
        case d435_test::MIXED:
          ++c.mixed;
          c.mixed_removed += kept ? 0 : 1;
          break;
        case d435_test::BOX_TOP:
          ++c.box_top;
          c.box_top_kept += kept ? 1 : 0;
          break;
        case d435_test::FLOOR: {
          const double z = frame.true_m.at<double>(v, u);
          if (z >= FLOOR_MIN_M && z <= FLOOR_MAX_M) {
            ++c.floor;
            c.floor_kept += kept ? 1 : 0;
          }
          break;
        }
        default:
          break;
      }
    }
  }
  return c;
}

void expect_scene_numbers(const View & view)
{
  const Counts c = filter_and_count(d435_test::render(view));
  ASSERT_GE(c.mixed, MIN_MIXED);
  ASSERT_GE(c.box_top, MIN_BOX_TOP);
  ASSERT_GE(c.floor, MIN_FLOOR);
  std::cout << "pan " << view.pan_deg << " tilt " << view.tilt_deg << (view.colour ? " colour" : " depth")
            << ": mixed removed " << c.mixed_removed << "/" << c.mixed << ", floor kept " << c.floor_kept << "/"
            << c.floor << ", box top kept " << c.box_top_kept << "/" << c.box_top << std::endl;
  EXPECT_GE(static_cast<double>(c.mixed_removed) / c.mixed, MIN_REMOVED_MIXED)
    << c.mixed_removed << " of " << c.mixed << " mixed pixels removed";
  EXPECT_GE(static_cast<double>(c.floor_kept) / c.floor, MIN_KEPT_FLOOR)
    << c.floor_kept << " of " << c.floor << " floor pixels kept";
  EXPECT_GE(static_cast<double>(c.box_top_kept) / c.box_top, MIN_KEPT_BOX_TOP)
    << c.box_top_kept << " of " << c.box_top << " box-top pixels kept";
}

}  // namespace

TEST(EdgeMaskScene, TheShippedMaskIsEnabled) { EXPECT_TRUE(shipped_config().edge_mask.enabled); }

TEST(EdgeMaskScene, WithoutTheMaskEveryMixedPixelIsPublished)
{
  const Frame frame = d435_test::render(View());
  int mixed = 0;
  for (int v = 0; v < frame.label.rows; ++v) {
    for (int u = 0; u < frame.label.cols; ++u) {
      if (frame.label.at<uint8_t>(v, u) == d435_test::MIXED) {
        ++mixed;
        EXPECT_NE(frame.depth_mm.at<uint16_t>(v, u), 0) << u << "," << v;
      }
    }
  }
  EXPECT_GE(mixed, MIN_MIXED);
}

TEST(EdgeMaskScene, DepthCameraAtTheHomePose) { expect_scene_numbers(View{0.0, 30.0, false}); }

TEST(EdgeMaskScene, DepthCameraAtTheBoxScenePose) { expect_scene_numbers(View{35.0, 29.4, false}); }

TEST(EdgeMaskScene, AlignedDepthAtTheHomePose) { expect_scene_numbers(View{0.0, 30.0, true}); }

TEST(EdgeMaskScene, AlignedDepthAtTheBoxScenePose) { expect_scene_numbers(View{35.0, 29.4, true}); }
