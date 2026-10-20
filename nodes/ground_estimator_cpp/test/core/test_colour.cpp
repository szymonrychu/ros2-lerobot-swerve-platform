#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "ground_estimator_cpp/colour.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::BandsConfig;
using ground_estimator_cpp::ColourConfig;
using ground_estimator_cpp::FloorColourModel;
using ground_estimator_cpp::Label;
using ground_estimator_cpp::label_cloud;
using ground_estimator_cpp::mismatch_points;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::rgb_to_lab;
using ground_estimator_cpp::to_labelled_msg;
using ground_test::add_box;
using ground_test::add_table_top;
using ground_test::floor_scene;
using ground_test::render;

constexpr double FLOOR_DISTANCE_M = 0.02;
constexpr double SIGMA_M = 0.001;
constexpr int DECIMATION = 3;
constexpr unsigned SEED = 5;
constexpr uint8_t GREY = 120;
constexpr int GREY_NOISE = 3;
// A4 sheet 0.21 x 0.297 m flat on the floor in front of the robot.
constexpr double SHEET_X0 = 0.6, SHEET_X1 = 0.897, SHEET_Y0 = -0.105, SHEET_Y1 = 0.105;
constexpr double MIN_SHARE = 0.01;  // the sheet is a small share of the floor view
constexpr double MAX_SHARE = 0.15;
constexpr double LAB_L_TOL = 0.5;
constexpr int TRAIN_FRAMES = 5;

bool in_range(const pcl::PointXYZ & p) { return std::hypot(p.x, p.y) <= BandsConfig{}.max_range_m; }

bool on_sheet(const pcl::PointXYZ & p)
{
  return p.x >= SHEET_X0 && p.x <= SHEET_X1 && p.y >= SHEET_Y0 && p.y <= SHEET_Y1;
}

// Deterministic grey with a little texture, a red sheet where on_sheet, the given colour for flagged points.
pcl::PointCloud<pcl::PointXYZRGB> colour(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, bool with_sheet, uint8_t grey = GREY)
{
  pcl::PointCloud<pcl::PointXYZRGB> out;
  std::size_t i = 0;
  for (const auto & p : cloud) {
    pcl::PointXYZRGB q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    if (with_sheet && on_sheet(p)) {
      q.r = 200;
      q.g = 30;
      q.b = 30;
    } else {
      const auto jitter = static_cast<uint8_t>(grey + static_cast<int>(i % (2 * GREY_NOISE + 1)) - GREY_NOISE);
      q.r = q.g = q.b = jitter;
    }
    out.push_back(q);
    ++i;
  }
  return out;
}

pcl::PointCloud<pcl::PointXYZ> floor_cloud() { return render(floor_scene(), SIGMA_M, 0.0, SEED, DECIMATION); }

FloorColourModel trained_model(const ColourConfig & cfg, uint8_t grey = GREY)
{
  FloorColourModel model;
  std::vector<Label> labels;
  const auto cloud = colour(floor_cloud(), false, grey);
  for (int i = 0; i < TRAIN_FRAMES; ++i) {
    label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  }
  return model;
}

std::size_t count(const std::vector<Label> & labels, Label l)
{
  std::size_t n = 0;
  for (auto v : labels) {
    n += v == l ? 1 : 0;
  }
  return n;
}

}  // namespace

TEST(Colour, LabConversionOfWhiteBlackGrey)
{
  EXPECT_NEAR(rgb_to_lab(cv::Vec3b(255, 255, 255))[0], 100.0, LAB_L_TOL);
  EXPECT_NEAR(rgb_to_lab(cv::Vec3b(0, 0, 0))[0], 0.0, LAB_L_TOL);
  const auto grey = rgb_to_lab(cv::Vec3b(128, 128, 128));
  EXPECT_NEAR(grey[0], 53.6, LAB_L_TOL);
  EXPECT_NEAR(grey[1], 0.0, 1.0);
  EXPECT_NEAR(grey[2], 0.0, 1.0);
}

TEST(Colour, RedSheetOnGreyFloorIsMismatch)
{
  const ColourConfig cfg;
  auto model = trained_model(cfg);
  const auto geometry = floor_cloud();
  const auto cloud = colour(geometry, true);
  std::vector<Label> labels;
  const auto stats = label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  ASSERT_EQ(labels.size(), cloud.size());
  std::size_t sheet = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (!in_range(geometry[i])) {
      EXPECT_EQ(labels[i], Label::unlabelled);
    } else if (on_sheet(geometry[i])) {
      ++sheet;
      EXPECT_EQ(labels[i], Label::floor_colour_mismatch);
    } else {
      EXPECT_EQ(labels[i], Label::floor);
    }
  }
  ASSERT_GT(sheet, 20u);
  EXPECT_EQ(stats.mismatch, sheet);
  const std::size_t in_band = stats.floor + stats.mismatch;
  EXPECT_EQ(in_band + stats.unlabelled, cloud.size());
  ASSERT_TRUE(stats.mismatch_ratio.has_value());
  EXPECT_NEAR(*stats.mismatch_ratio, static_cast<double>(sheet) / in_band, 1e-9);
  EXPECT_GT(*stats.mismatch_ratio, MIN_SHARE);
  EXPECT_LT(*stats.mismatch_ratio, MAX_SHARE);
}

TEST(Colour, GreyBoxIsAboveFloorGeometryWins)
{
  const ColourConfig cfg;
  auto model = trained_model(cfg);
  auto scene = floor_scene();
  add_box(scene, Eigen::Vector3d(0.8, 0.0, 0.1), Eigen::Vector3d(0.2, 0.3, 0.2));
  const auto geometry = render(scene, SIGMA_M, 0.0, SEED, DECIMATION);
  // The box is red here: geometry still wins.
  pcl::PointCloud<pcl::PointXYZRGB> cloud = colour(geometry, false);
  for (auto & p : cloud) {
    if (p.z > 0.05f) {
      p.r = 255;
      p.g = 0;
      p.b = 0;
    }
  }
  std::vector<Label> labels;
  label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  std::size_t above = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (cloud[i].z > 0.05f) {
      ++above;
      EXPECT_EQ(labels[i], Label::above_floor);
    }
  }
  EXPECT_GT(above, 20u);
  EXPECT_EQ(count(labels, Label::floor_colour_mismatch), 0u);
}

TEST(Colour, MismatchPointsAreExactlyTheSheet)
{
  const ColourConfig cfg;
  auto model = trained_model(cfg);
  const auto geometry = floor_cloud();
  const auto cloud = colour(geometry, true);
  std::vector<Label> labels;
  label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  const auto points = mismatch_points(cloud, labels, Plane{});
  std::size_t sheet = 0;
  for (const auto & p : geometry) {
    sheet += in_range(p) && on_sheet(p) ? 1 : 0;
  }
  ASSERT_EQ(points.size(), sheet);
  for (const auto & p : points) {
    EXPECT_GE(p.point.x(), SHEET_X0);
    EXPECT_LE(p.point.x(), SHEET_X1);
    EXPECT_NEAR(p.height, p.point.z(), 1e-9);
    EXPECT_LE(std::fabs(p.height), FLOOR_DISTANCE_M);
  }
}

TEST(Colour, NothingIsMismatchBeforeMinFloorSamples)
{
  ColourConfig cfg;
  cfg.min_floor_samples = 1000000;
  FloorColourModel model;
  const auto cloud = colour(floor_cloud(), true);
  std::vector<Label> labels;
  const auto stats = label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  EXPECT_FALSE(model.ready());
  EXPECT_EQ(stats.mismatch, 0u);
  EXPECT_EQ(stats.floor + stats.unlabelled, cloud.size());
}

TEST(Colour, ModelFollowsFloorColourChangeWithinOneOverForgetting)
{
  const ColourConfig cfg;
  auto model = trained_model(cfg, GREY);
  constexpr uint8_t NEW_GREY = 150;
  const auto cloud = colour(floor_cloud(), false, NEW_GREY);
  EXPECT_GT(model.mahalanobis(cv::Vec3b(NEW_GREY, NEW_GREY, NEW_GREY)), cfg.max_mahalanobis);
  std::vector<Label> labels;
  const int updates = static_cast<int>(std::lround(1.0 / cfg.forgetting));
  for (int i = 0; i < updates; ++i) {
    label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  }
  EXPECT_LE(model.mahalanobis(cv::Vec3b(NEW_GREY, NEW_GREY, NEW_GREY)), cfg.max_mahalanobis);
  EXPECT_EQ(count(labels, Label::floor_colour_mismatch), 0u);
}

TEST(Colour, DisabledGivesGeometricLabelsOnly)
{
  ColourConfig cfg;
  cfg.enabled = false;
  FloorColourModel model;
  auto scene = floor_scene();
  add_box(scene, Eigen::Vector3d(0.8, 0.0, 0.1), Eigen::Vector3d(0.2, 0.3, 0.2));
  const auto cloud = colour(render(scene, SIGMA_M, 0.0, SEED, DECIMATION), true);
  std::vector<Label> labels;
  const auto stats = label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  EXPECT_EQ(model.samples(), 0u);
  EXPECT_EQ(stats.mismatch, 0u);
  EXPECT_GT(stats.floor, 0u);
  EXPECT_GT(stats.above, 0u);
}

TEST(Colour, NoFloorPointsGivesNoRatio)
{
  const ColourConfig cfg;
  FloorColourModel model;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB p;
  p.x = 0.5f;
  p.y = 0.0f;
  p.z = 0.3f;
  cloud.push_back(p);
  p.z = 5.0f;  // outside every band
  cloud.push_back(p);
  std::vector<Label> labels;
  const auto stats = label_cloud(cloud, Plane{}, BandsConfig{}, cfg, FLOOR_DISTANCE_M, model, labels);
  EXPECT_FALSE(stats.mismatch_ratio.has_value());
  EXPECT_EQ(labels[0], Label::above_floor);
  EXPECT_EQ(labels[1], Label::unlabelled);
  EXPECT_EQ(stats.unlabelled, 1u);
}

TEST(Colour, LabelledMessageLayout)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB p;
  p.x = 1.0f;
  p.y = 2.0f;
  p.z = 3.0f;
  p.r = 10;
  p.g = 20;
  p.b = 30;
  cloud.push_back(p);
  p.x = 4.0f;
  cloud.push_back(p);
  const std::vector<Label> labels{Label::floor_colour_mismatch, Label::unlabelled};
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  const auto msg = to_labelled_msg(cloud, labels, "base_link", stamp);
  EXPECT_EQ(msg.header.frame_id, "base_link");
  EXPECT_EQ(msg.header.stamp.sec, 7);
  EXPECT_EQ(msg.width, 2u);
  EXPECT_EQ(msg.height, 1u);
  ASSERT_EQ(msg.fields.size(), 5u);
  const char * names[] = {"x", "y", "z", "rgb", "label"};
  const uint32_t offsets[] = {0, 4, 8, 12, 16};
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(msg.fields[i].name, names[i]);
    EXPECT_EQ(msg.fields[i].offset, offsets[i]);
    EXPECT_EQ(msg.fields[i].count, 1u);
  }
  EXPECT_EQ(msg.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(msg.fields[3].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(msg.fields[4].datatype, sensor_msgs::msg::PointField::UINT8);
  ASSERT_EQ(msg.data.size(), msg.point_step * 2);
  float x1 = 0.0f;
  std::memcpy(&x1, &msg.data[msg.point_step], sizeof(float));
  EXPECT_FLOAT_EQ(x1, 4.0f);
  uint32_t packed = 0;
  std::memcpy(&packed, &msg.data[12], sizeof(uint32_t));
  EXPECT_EQ(packed & 0xFFFFFFu, (10u << 16) | (20u << 8) | 30u);
  EXPECT_EQ(msg.data[16], static_cast<uint8_t>(Label::floor_colour_mismatch));
  EXPECT_EQ(msg.data[msg.point_step + 16], static_cast<uint8_t>(Label::unlabelled));
}
