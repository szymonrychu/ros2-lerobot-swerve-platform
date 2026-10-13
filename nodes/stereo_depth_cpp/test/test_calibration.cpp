// Calibration files: loading the files the Python calibration tool writes (test/fixtures/<case>/calibration), every
// invalid file set the Python loader rejects (test/fixtures/calibration_invalid.yaml), the rectification maps and the
// file signature that drives the reload check.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>
#include <string>

#include "fixtures.hpp"
#include "stereo_depth_cpp/calibration.hpp"

namespace
{

constexpr int WIDTH = 480;
constexpr int HEIGHT = 360;

void write_case(const YAML::Node & files, const std::string & directory)
{
  fixtures::copy_calibration("ideal", directory);
  for (const auto & entry : files) {
    const std::string name = entry.first.as<std::string>();
    const std::string target = directory + "/" + name;
    if (entry.second.IsNull()) {
      std::filesystem::remove(target);
    } else if (entry.second.IsScalar()) {
      std::ofstream(target) << entry.second.as<std::string>();
    } else {
      YAML::Emitter out;
      out << entry.second;
      std::ofstream(target) << out.c_str() << "\n";
    }
  }
}

}  // namespace

TEST(Calibration, LoadsThePythonWrittenFiles)
{
  const YAML::Node cases = fixtures::yaml("cases.yaml")["cases"];
  for (const char * name : {"ideal", "distorted"}) {
    SCOPED_TRACE(name);
    const auto loaded = stereo_depth_cpp::load_stereo_calibration(fixtures::path(std::string(name) + "/calibration"));
    ASSERT_TRUE(loaded.calibration.has_value()) << loaded.reason;
    EXPECT_TRUE(loaded.reason.empty());
    const auto & c = *loaded.calibration;
    EXPECT_EQ(c.width(), WIDTH);
    EXPECT_EQ(c.height(), HEIGHT);
    EXPECT_DOUBLE_EQ(c.fx(), cases[name]["fx"].as<double>());
    EXPECT_DOUBLE_EQ(c.baseline_m(), cases[name]["baseline_m"].as<double>());
    EXPECT_EQ(c.left.camera_name, "stereo_left");
    EXPECT_EQ(c.right.camera_name, "stereo_right");
    EXPECT_EQ(c.left.distortion_model, "plumb_bob");
    EXPECT_EQ(c.left.d.size(), 5u);
    EXPECT_DOUBLE_EQ(c.extrinsics.rms, 0.2);
  }
}

TEST(Calibration, EveryPythonRejectionIsRejected)
{
  const YAML::Node cases = fixtures::yaml("calibration_invalid.yaml");
  ASSERT_GE(cases.size(), 30u);
  for (const auto & item : cases) {
    const std::string name = item["name"].as<std::string>();
    SCOPED_TRACE(name);
    fixtures::TempDir tmp;
    write_case(item["files"], tmp.path());
    const auto loaded = stereo_depth_cpp::load_stereo_calibration(tmp.path());
    EXPECT_FALSE(loaded.calibration.has_value()) << "accepted although Python rejects it";
    EXPECT_FALSE(loaded.reason.empty());
  }
}

TEST(Calibration, ReasonNamesTheFile)
{
  fixtures::TempDir tmp;
  const auto missing = stereo_depth_cpp::load_stereo_calibration(tmp.path());
  EXPECT_NE(missing.reason.find("stereo_left.yaml is missing"), std::string::npos) << missing.reason;
  fixtures::copy_calibration("ideal", tmp.path());
  tmp.write("stereo.yaml", "just text\n");
  const auto bad = stereo_depth_cpp::load_stereo_calibration(tmp.path());
  EXPECT_NE(bad.reason.find("stereo.yaml: expected a mapping"), std::string::npos) << bad.reason;
}

TEST(Calibration, SingleFileLoaderThrowsCalibrationError)
{
  fixtures::TempDir tmp;
  EXPECT_THROW(stereo_depth_cpp::load_camera_info_yaml(tmp.path() + "/nope.yaml"), stereo_depth_cpp::CalibrationError);
  EXPECT_THROW(
    stereo_depth_cpp::load_stereo_extrinsics_yaml(tmp.write("s.yaml", "rms: 1\n")), stereo_depth_cpp::CalibrationError);
}

TEST(Calibration, IdealMapsAreTheIdentity)
{
  const auto loaded = stereo_depth_cpp::load_stereo_calibration(fixtures::path("ideal/calibration"));
  ASSERT_TRUE(loaded.calibration.has_value());
  const auto maps = stereo_depth_cpp::rectification_maps(*loaded.calibration);
  EXPECT_EQ(maps.left_map1.type(), CV_16SC2);
  EXPECT_EQ(maps.left_map1.size(), cv::Size(WIDTH, HEIGHT));
  const cv::Mat left = fixtures::png("left.png");
  const cv::Mat right = fixtures::png("right.png");
  cv::Mat rect_left;
  cv::Mat rect_right;
  stereo_depth_cpp::rectify(maps, left, right, rect_left, rect_right);
  EXPECT_EQ(cv::countNonZero(rect_left != left), 0);
  EXPECT_EQ(cv::countNonZero(rect_right != right), 0);
}

TEST(Calibration, DistortedMapsMoveThePixels)
{
  const auto loaded = stereo_depth_cpp::load_stereo_calibration(fixtures::path("distorted/calibration"));
  ASSERT_TRUE(loaded.calibration.has_value());
  const auto maps = stereo_depth_cpp::rectification_maps(*loaded.calibration);
  EXPECT_EQ(maps.right_map1.type(), CV_16SC2);
  EXPECT_EQ(maps.right_map2.type(), CV_16UC1);
  const cv::Mat left = fixtures::png("left.png");
  cv::Mat rect_left;
  cv::Mat rect_right;
  stereo_depth_cpp::rectify(maps, left, fixtures::png("right.png"), rect_left, rect_right);
  EXPECT_EQ(rect_left.size(), left.size());
  EXPECT_GT(cv::countNonZero(rect_left != left), static_cast<int>(left.total() / 2));
}

TEST(Calibration, SignatureTracksTheFiles)
{
  fixtures::TempDir tmp;
  const auto empty = stereo_depth_cpp::calibration_signature(tmp.path());
  for (const auto & stamp : empty) {
    EXPECT_FALSE(stamp.has_value());
  }
  fixtures::copy_calibration("ideal", tmp.path());
  const auto full = stereo_depth_cpp::calibration_signature(tmp.path());
  for (const auto & stamp : full) {
    EXPECT_TRUE(stamp.has_value());
  }
  EXPECT_EQ(stereo_depth_cpp::calibration_signature(tmp.path()), full);
  tmp.write("stereo.yaml", "changed: 1\n");
  EXPECT_NE(stereo_depth_cpp::calibration_signature(tmp.path()), full);
}
