#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "ground_estimator_cpp/guidance.hpp"
#include "synthetic_floor.hpp"

namespace ge = ground_estimator_cpp;

namespace
{

constexpr double RAD_PER_DEG = 0.017453292519943295;

constexpr int IMAGE_W_PX = 320;
constexpr int IMAGE_H_PX = 240;
constexpr int GRID_STEP_PX = 2;
constexpr double HEIGHT_TOL_M = 0.001;
constexpr double HIT_TOL_M = 0.005;

// Optical frame pitched down by pitch_deg: optical z forward, x right, y down.
Eigen::Isometry3d optical_mount(const Eigen::Vector3d & origin, double pitch_deg)
{
  Eigen::Matrix3d optical;
  optical.col(0) = -Eigen::Vector3d::UnitY();
  optical.col(1) = -Eigen::Vector3d::UnitZ();
  optical.col(2) = Eigen::Vector3d::UnitX();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = origin;
  t.linear() = Eigen::AngleAxisd(pitch_deg * RAD_PER_DEG, Eigen::Vector3d::UnitY()).toRotationMatrix() * optical;
  return t;
}

ge::CameraModel make_model(int w, int h, double f)
{
  ge::CameraModel m;
  m.K = cv::Matx33d(f, 0, w / 2.0, 0, f, h / 2.0, 0, 0, 1);
  const double d[5] = {-0.12, 0.05, 0.001, -0.0007, 0.0};
  for (int i = 0; i < 5; ++i) {
    m.D[i] = d[i];
  }
  m.width = w;
  m.height = h;
  return m;
}

sensor_msgs::msg::CameraInfo info_of(const ge::CameraModel & m)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = m.width;
  info.height = m.height;
  info.distortion_model = "plumb_bob";
  for (int i = 0; i < 9; ++i) {
    info.k[i] = m.K(i / 3, i % 3);
  }
  info.d.assign(m.D.val, m.D.val + 5);
  return info;
}

// Everything of one scene: camera, ToF, plane and a depth image of the floor z = 0.
struct World
{
  ge::CameraModel cam = make_model(IMAGE_W_PX, IMAGE_H_PX, 250.0);
  ge::CameraModel tof = make_model(240, 180, 170.0);
  Eigen::Isometry3d base_T_cam = optical_mount(Eigen::Vector3d(0.17, -0.03, 0.47), 30.0);
  Eigen::Isometry3d tof_T_base = optical_mount(Eigen::Vector3d(0.15, 0.0, 0.45), 30.0).inverse();
  ge::Plane plane;
  ge::GuidanceConfig config;
  ge::GuidanceFrame frame;
};

// Camera-frame z of the floor hit of a pixel, 0 when the ray misses the floor.
double floor_depth_m(const World & w, const ge::RayGrid & full, int u, int v)
{
  const Eigen::Vector3d ray(full.ray(u, v)[0], full.ray(u, v)[1], 1.0);
  const Eigen::Vector3d dir = w.base_T_cam.linear() * ray;
  if (dir.z() >= -1e-9) {
    return 0.0;
  }
  return -w.base_T_cam.translation().z() / dir.z();
}

World make_world()
{
  World w;
  w.frame.model = w.cam;
  w.frame.depth_mm = cv::Mat(IMAGE_H_PX, IMAGE_W_PX, CV_16UC1, cv::Scalar(0));
  w.frame.bgr = cv::Mat(IMAGE_H_PX, IMAGE_W_PX, CV_8UC3, cv::Scalar(10, 20, 30));
  const ge::RayGrid full(w.cam, 1);
  for (int v = 0; v < IMAGE_H_PX; ++v) {
    for (int u = 0; u < IMAGE_W_PX; ++u) {
      w.frame.depth_mm.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(floor_depth_m(w, full, u, v) * 1000.0));
    }
  }
  return w;
}

Eigen::Vector3d floor_point(const World & w, const ge::RayGrid & full, int u, int v)
{
  const Eigen::Vector3d ray(full.ray(u, v)[0], full.ray(u, v)[1], 1.0);
  return w.base_T_cam * (ray * floor_depth_m(w, full, u, v));
}

void clear_rect(World & w, const cv::Rect & r) { w.frame.depth_mm(r).setTo(0); }

ge::GuidanceSamples sample(const World & w)
{
  return ge::sample_guidance(
    w.frame, ge::RayGrid(w.cam, GRID_STEP_PX), w.base_T_cam, w.plane, w.tof, w.tof_T_base, w.config);
}

}  // namespace

TEST(Guidance, RayGridMatchesProjection)
{
  const ge::CameraModel m = make_model(IMAGE_W_PX, IMAGE_H_PX, 250.0);
  const ge::RayGrid grid(m, GRID_STEP_PX);
  ASSERT_EQ(grid.cols(), IMAGE_W_PX / GRID_STEP_PX);
  ASSERT_EQ(grid.rows(), IMAGE_H_PX / GRID_STEP_PX);
  for (int j = 0; j < grid.rows(); ++j) {
    for (int i = 0; i < grid.cols(); ++i) {
      // Project the ray with cv::projectPoints (D applied) and expect the grid pixel back.
      std::vector<cv::Point3d> object;
      object.push_back(cv::Point3d(grid.ray(i, j)[0], grid.ray(i, j)[1], 1.0));
      std::vector<cv::Point2d> image;
      const cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);
      cv::projectPoints(object, zero, zero, cv::Mat(m.K), cv::Mat(m.D), image);
      ASSERT_NEAR(image[0].x, i * GRID_STEP_PX, 1e-6);
      ASSERT_NEAR(image[0].y, j * GRID_STEP_PX, 1e-6);
    }
  }
}

TEST(Guidance, RayGridIsRebuiltOnlyWhenModelOrStepChange)
{
  const ge::CameraModel m = make_model(IMAGE_W_PX, IMAGE_H_PX, 250.0);
  ge::RayGrid grid;
  EXPECT_TRUE(grid.ensure(m, GRID_STEP_PX));
  EXPECT_FALSE(grid.ensure(m, GRID_STEP_PX));
  EXPECT_TRUE(grid.ensure(m, GRID_STEP_PX + 1));
  EXPECT_TRUE(grid.ensure(make_model(IMAGE_W_PX, IMAGE_H_PX, 251.0), GRID_STEP_PX + 1));
}

TEST(Guidance, FlatFloorHeightsAreZero)
{
  World w = make_world();
  const auto s = sample(w);
  ASSERT_GT(s.points.size(), 100u);
  ASSERT_EQ(s.heights.size(), s.points.size());
  ASSERT_EQ(s.colours.size(), s.points.size());
  for (const float h : s.heights) {
    ASSERT_LE(std::fabs(h), HEIGHT_TOL_M);
  }
  EXPECT_EQ(s.colours[0], cv::Vec3b(30, 20, 10));  // R G B of the BGR (10, 20, 30)
  EXPECT_TRUE(s.no_return.empty());
}

TEST(Guidance, HoleOnTheFloorBecomesNoReturnSamplesAtTheRightPlace)
{
  World w = make_world();
  const cv::Rect hole(120, 100, 40, 40);
  clear_rect(w, hole);
  const auto s = sample(w);
  const ge::RayGrid full(w.cam, 1);
  ASSERT_GT(s.no_return.size(), 50u);
  ASSERT_EQ(s.no_return_colours.size(), s.no_return.size());
  std::size_t expected = 0;
  for (int v = hole.y; v < hole.y + hole.height; v += GRID_STEP_PX) {
    for (int u = hole.x; u < hole.x + hole.width; u += GRID_STEP_PX) {
      ++expected;
    }
  }
  EXPECT_EQ(s.no_return.size(), expected);
  for (const auto & p : s.no_return) {
    double best = 1e9;
    for (int v = hole.y; v < hole.y + hole.height; v += GRID_STEP_PX) {
      for (int u = hole.x; u < hole.x + hole.width; u += GRID_STEP_PX) {
        const Eigen::Vector3d e = floor_point(w, full, u, v);
        best = std::min(best, (e - Eigen::Vector3d(p.x, p.y, p.z)).norm());
      }
    }
    ASSERT_LE(best, HIT_TOL_M);
  }
}

TEST(Guidance, HitOutsideTofViewIsNoSample)
{
  World w = make_world();
  w.tof = make_model(240, 180, 1000.0);  // about 6 degrees either side: the left of the camera image is outside
  clear_rect(w, cv::Rect(0, 100, 40, 40));
  EXPECT_TRUE(sample(w).no_return.empty());
  clear_rect(w, cv::Rect(140, 110, 20, 20));
  EXPECT_FALSE(sample(w).no_return.empty());
}

TEST(Guidance, HitOutsideRangeIsNoSample)
{
  World w = make_world();
  clear_rect(w, cv::Rect(120, 100, 40, 40));
  const auto base = sample(w);
  ASSERT_FALSE(base.no_return.empty());
  double far = 0.0;
  for (const auto & p : base.no_return) {
    far = std::max(far, std::hypot(p.x - w.base_T_cam.translation().x(), p.y - w.base_T_cam.translation().y()));
  }
  w.config.no_return.min_range_m = far + 0.5;  // nearer than the minimum
  w.config.max_range_m = far + 1.0;
  EXPECT_TRUE(sample(w).no_return.empty());
  w.config.no_return.min_range_m = 0.1;
  w.config.max_range_m = 0.3;  // farther than the maximum
  EXPECT_TRUE(sample(w).no_return.empty());
  EXPECT_TRUE(sample(w).points.empty());
}

TEST(Guidance, BadInputsAreRejectedWithAReason)
{
  const ge::CameraModel m = make_model(8, 6, 10.0);
  sensor_msgs::msg::Image depth;
  depth.width = 8;
  depth.height = 6;
  depth.encoding = "16UC1";
  depth.step = 16;
  depth.data.assign(16 * 6, 0);
  const auto info = info_of(m);
  sensor_msgs::msg::CompressedImage rgb;
  rgb.format = "jpeg";
  std::vector<uint8_t> jpeg;
  ASSERT_TRUE(cv::imencode(".jpg", cv::Mat(6, 8, CV_8UC3, cv::Scalar(1, 2, 3)), jpeg));
  rgb.data = jpeg;

  std::string why;
  const auto ok = ge::decode_guidance(depth, info, rgb, &why);
  ASSERT_TRUE(ok.has_value()) << why;
  EXPECT_EQ(ok->depth_mm.type(), CV_16UC1);
  EXPECT_EQ(ok->bgr.size(), cv::Size(8, 6));

  auto bad_depth = depth;
  bad_depth.encoding = "32FC1";
  EXPECT_FALSE(ge::decode_guidance(bad_depth, info, rgb, &why));
  EXPECT_NE(why.find("16UC1"), std::string::npos) << why;

  bad_depth = depth;
  bad_depth.data.resize(20);
  EXPECT_FALSE(ge::decode_guidance(bad_depth, info, rgb, &why));
  EXPECT_NE(why.find("shorter"), std::string::npos) << why;

  auto bad_rgb = rgb;
  bad_rgb.data.assign(jpeg.begin(), jpeg.begin() + 10);
  why.clear();
  EXPECT_FALSE(ge::decode_guidance(depth, info, bad_rgb, &why));
  EXPECT_NE(why.find("JPEG"), std::string::npos) << why;

  bad_rgb.data.assign(100, 0x42);
  why.clear();
  EXPECT_FALSE(ge::decode_guidance(depth, info, bad_rgb, &why));
  EXPECT_NE(why.find("decode"), std::string::npos) << why;

  ASSERT_TRUE(cv::imencode(".jpg", cv::Mat(6, 10, CV_8UC3, cv::Scalar(1, 2, 3)), jpeg));
  bad_rgb.data = jpeg;
  why.clear();
  EXPECT_FALSE(ge::decode_guidance(depth, info, bad_rgb, &why));
  EXPECT_NE(why.find("JPEG size"), std::string::npos) << why;

  auto bad_info = info;
  bad_info.width = 9;
  why.clear();
  EXPECT_FALSE(ge::decode_guidance(depth, bad_info, rgb, &why));
  EXPECT_NE(why.find("camera_info size"), std::string::npos) << why;

  EXPECT_TRUE(ge::decode_guidance(depth, info, rgb, nullptr).has_value());
}

TEST(Guidance, CameraModelOfRejectsEmptyCalibration)
{
  const auto good = info_of(make_model(8, 6, 10.0));
  ASSERT_TRUE(ge::camera_model_of(good).has_value());

  auto info = good;
  info.k.fill(0.0);
  EXPECT_FALSE(ge::camera_model_of(info).has_value());

  info = good;
  info.distortion_model = "rational_polynomial";
  EXPECT_FALSE(ge::camera_model_of(info).has_value());

  info = good;
  info.distortion_model.clear();
  EXPECT_FALSE(ge::camera_model_of(info).has_value());

  info = good;
  info.width = 0;
  EXPECT_FALSE(ge::camera_model_of(info).has_value());

  info = good;
  info.d.resize(3);
  EXPECT_FALSE(ge::camera_model_of(info).has_value());
}
