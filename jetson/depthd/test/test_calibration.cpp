#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "depthd/calibration.hpp"
#include "depthd/rectify.hpp"

namespace
{
const std::string kFixtures = std::string(DEPTHD_SOURCE_DIR) + "/test/fixtures/";
const std::string kValid = kFixtures + "stereo_valid.yaml";
const cv::Size kSize(640, 480);
constexpr double kMaxRms = 0.5;

depthd::StereoCalibration valid()
{
  auto load = depthd::load_calibration(kValid, kSize, kMaxRms);
  EXPECT_TRUE(load.ok()) << load.reason;
  return *load.calibration;
}

std::string tmp_path(const std::string & name) { return testing::TempDir() + "depthd_cal_" + name; }

// Saves `cal` (already mutated) and loads it back with the given processing size.
depthd::CalibrationLoad roundtrip(const depthd::StereoCalibration & cal, const std::string & name,
                                  cv::Size size = kSize)
{
  const std::string path = tmp_path(name);
  EXPECT_TRUE(depthd::save_calibration(path, cal).empty());
  auto load = depthd::load_calibration(path, size, kMaxRms);
  std::remove(path.c_str());
  return load;
}

void expect_rejected(const depthd::CalibrationLoad & load, const std::string & fragment)
{
  EXPECT_FALSE(load.ok());
  EXPECT_FALSE(load.calibration.has_value());
  EXPECT_NE(load.reason.find(fragment), std::string::npos) << "reason was: " << load.reason;
}
}  // namespace

TEST(Calibration, ValidFixtureLoads)
{
  const auto cal = valid();
  EXPECT_EQ(cal.size, kSize);
  EXPECT_NEAR(cal.baseline_m, 0.06, 1e-12);
  EXPECT_NEAR(cal.rms, 0.21, 1e-12);
  EXPECT_NEAR(cal.P1.at<double>(0, 0), 500.0, 1e-9);
  EXPECT_NEAR(cal.P2.at<double>(0, 3), -30.0, 1e-9);
  EXPECT_NEAR(cal.K1.at<double>(0, 2), 322.0, 1e-9);
  EXPECT_EQ(cal.D1.total(), 5u);
  EXPECT_EQ(cal.Q.rows, 4);
}

TEST(Calibration, MissingFileIsReported)
{
  auto load = depthd::load_calibration(tmp_path("does_not_exist.yaml"), kSize, kMaxRms);
  expect_rejected(load, "missing");
}

TEST(Calibration, GarbageFileIsReported)
{
  const std::string path = tmp_path("garbage.yaml");
  FILE * f = std::fopen(path.c_str(), "w");
  ASSERT_NE(f, nullptr);
  std::fputs("this: [is not\n  opencv", f);
  std::fclose(f);
  auto load = depthd::load_calibration(path, kSize, kMaxRms);
  std::remove(path.c_str());
  EXPECT_FALSE(load.ok());
  EXPECT_FALSE(load.reason.empty());
}

TEST(Calibration, MissingKeyIsRejectedNamingTheKey)
{
  const std::string path = tmp_path("nokey.yaml");
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "image_width" << 640 << "image_height" << 480 << "rms" << 0.2 << "baseline_m" << 0.06;
  fs.release();
  auto load = depthd::load_calibration(path, kSize, kMaxRms);
  std::remove(path.c_str());
  expect_rejected(load, "K_left");
}

TEST(Calibration, EveryRequiredKeyIsChecked)
{
  for (const char * key : {"image_width", "image_height", "rms", "baseline_m", "K_left", "D_left", "K_right", "D_right",
                           "R", "T", "R1", "R2", "P1", "P2", "Q"}) {
    // rename the key in the fixture text so exactly that key is absent
    std::string text;
    FILE * f = std::fopen(kValid.c_str(), "r");
    ASSERT_NE(f, nullptr);
    char buf[512];
    while (std::fgets(buf, sizeof buf, f)) text += buf;
    std::fclose(f);
    const std::string needle = std::string("\n") + key + ":";
    const auto pos = text.find(needle);
    ASSERT_NE(pos, std::string::npos) << key;
    text.replace(pos, needle.size(), std::string("\nzz_") + key + ":");
    const std::string path = tmp_path(std::string("rm_") + key + ".yaml");
    f = std::fopen(path.c_str(), "w");
    std::fputs(text.c_str(), f);
    std::fclose(f);
    auto load = depthd::load_calibration(path, kSize, kMaxRms);
    std::remove(path.c_str());
    expect_rejected(load, key);
  }
}

TEST(Calibration, WrongMatrixShapeIsRejected)
{
  auto cal = valid();
  cal.K1 = cv::Mat::eye(4, 4, CV_64F);
  expect_rejected(roundtrip(cal, "shape_k"), "K_left");
  cal = valid();
  cal.T = cv::Mat::zeros(1, 3, CV_64F);
  cal.T.at<double>(0, 0) = -0.06;
  expect_rejected(roundtrip(cal, "shape_t"), "T");
  cal = valid();
  cal.P2 = cv::Mat::zeros(3, 3, CV_64F);
  expect_rejected(roundtrip(cal, "shape_p"), "P2");
  cal = valid();
  cal.D1 = cv::Mat::zeros(1, 3, CV_64F);
  expect_rejected(roundtrip(cal, "shape_d"), "D_left");
}

TEST(Calibration, RmsAboveLimitIsRejected)
{
  auto load = depthd::load_calibration(kFixtures + "stereo_invalid_rms.yaml", kSize, kMaxRms);
  expect_rejected(load, "rms");
  EXPECT_NE(load.reason.find("max_rms_px"), std::string::npos);
  // the same file passes when the limit is raised
  EXPECT_TRUE(depthd::load_calibration(kFixtures + "stereo_invalid_rms.yaml", kSize, 1.0).ok());
}

TEST(Calibration, BaselineOutsideRangeIsRejected)
{
  for (double b : {0.02, 0.13}) {
    auto cal = valid();
    cal.baseline_m = b;
    cal.T.at<double>(0, 0) = -b;
    expect_rejected(roundtrip(cal, "baseline_range"), "baseline_m");
  }
}

TEST(Calibration, SwappedCamerasAreRejectedWithAdvice)
{
  auto cal = valid();
  cal.T.at<double>(0, 0) = 0.0593;  // left and right are physically swapped: T.x is positive
  cal.baseline_m = 0.0593;
  cal.P2.at<double>(0, 3) = 30.0;
  const auto load = roundtrip(cal, "swapped");
  expect_rejected(load, "swapped");
  EXPECT_NE(load.reason.find("T.x = +59.3 mm"), std::string::npos) << load.reason;
  EXPECT_NE(load.reason.find("left_sensor_id"), std::string::npos) << load.reason;
  EXPECT_NE(load.reason.find("recalibrate"), std::string::npos) << load.reason;

  cal = valid();  // T.x fine but P2[0][3] positive: also swapped
  cal.P2.at<double>(0, 3) = 30.0;
  expect_rejected(roundtrip(cal, "swapped_p2"), "swapped");

  cal = valid();
  cal.T.at<double>(0, 0) = 0.0;  // T.x == 0 is not a left-right rig either
  cal.T.at<double>(1, 0) = 0.06;
  expect_rejected(roundtrip(cal, "swapped_zero"), "swapped");
}

TEST(Calibration, BaselineNotMatchingTranslationIsRejected)
{
  auto cal = valid();
  cal.baseline_m = 0.0615;  // 2.5 % off |T| = 0.06
  expect_rejected(roundtrip(cal, "baseline_t"), "|T|");
  cal = valid();
  cal.baseline_m = 0.0603;  // 0.5 % off: accepted
  EXPECT_TRUE(roundtrip(cal, "baseline_ok").ok());
}

TEST(Calibration, NonPositiveFocalLengthIsRejected)
{
  auto cal = valid();
  cal.K1.at<double>(0, 0) = 0.0;
  expect_rejected(roundtrip(cal, "focal_k"), "focal");
  cal = valid();
  cal.P1.at<double>(1, 1) = -500.0;
  expect_rejected(roundtrip(cal, "focal_p"), "focal");
  cal = valid();
  cal.K2.at<double>(1, 1) = -1.0;
  expect_rejected(roundtrip(cal, "focal_k2"), "focal");
}

TEST(Calibration, AspectRatioMismatchIsRejected)
{
  expect_rejected(depthd::load_calibration(kValid, cv::Size(640, 360), kMaxRms), "aspect");
  // 1 % tolerance: 640x484 differs by 0.8 % and is accepted
  EXPECT_TRUE(depthd::load_calibration(kValid, cv::Size(640, 484), kMaxRms).ok());
}

TEST(Calibration, NonFiniteValueIsRejected)
{
  auto cal = valid();
  cal.P1.at<double>(0, 2) = std::numeric_limits<double>::quiet_NaN();
  expect_rejected(roundtrip(cal, "nan"), "finite");
}

TEST(Calibration, HigherResolutionIsScaledToProcessingSize)
{
  // The 640x480 fixture rewritten at 1280x960: every pixel quantity doubled by hand.
  auto cal = valid();
  cal.size = cv::Size(1280, 960);
  for (cv::Mat * m : {&cal.K1, &cal.K2}) {
    for (int r : {0, 1}) for (int c = 0; c < 3; ++c) m->at<double>(r, c) *= 2.0;
  }
  for (cv::Mat * m : {&cal.P1, &cal.P2}) {
    for (int r : {0, 1}) for (int c = 0; c < 4; ++c) m->at<double>(r, c) *= 2.0;
  }
  cal.Q.at<double>(0, 3) *= 2.0;
  cal.Q.at<double>(1, 3) *= 2.0;
  cal.Q.at<double>(2, 3) *= 2.0;
  cal.Q.at<double>(3, 3) *= 2.0;
  const auto scaled = roundtrip(cal, "hires");
  ASSERT_TRUE(scaled.ok()) << scaled.reason;
  const auto ref = valid();
  EXPECT_EQ(scaled.calibration->size, kSize);
  EXPECT_LT(cv::norm(scaled.calibration->K1, ref.K1, cv::NORM_INF), 1e-9);
  EXPECT_LT(cv::norm(scaled.calibration->K2, ref.K2, cv::NORM_INF), 1e-9);
  EXPECT_LT(cv::norm(scaled.calibration->P1, ref.P1, cv::NORM_INF), 1e-9);
  EXPECT_LT(cv::norm(scaled.calibration->P2, ref.P2, cv::NORM_INF), 1e-9);
  EXPECT_LT(cv::norm(scaled.calibration->Q, ref.Q, cv::NORM_INF), 1e-9);
  EXPECT_NEAR(scaled.calibration->baseline_m, 0.06, 1e-12);
  // the distortion coefficients are resolution independent
  EXPECT_LT(cv::norm(scaled.calibration->D1, ref.D1, cv::NORM_INF), 1e-12);
}

TEST(Rectifier, ValidFileIsCalibratedAndExposesCameraInfoParts)
{
  depthd::Rectifier rect(kValid, kMaxRms, kSize);
  ASSERT_TRUE(rect.calibrated()) << rect.reason();
  EXPECT_EQ(rect.state(), "calibrated");
  EXPECT_TRUE(rect.reason().empty());
  const auto model = rect.snapshot();
  ASSERT_TRUE(model);
  EXPECT_NEAR(model->baseline_m(), 0.06, 1e-12);
  EXPECT_NEAR(model->left_P().at<double>(0, 0), 500.0, 1e-9);
  EXPECT_NEAR(model->left_K().at<double>(0, 0), 510.0, 1e-9);  // unrectified K
  EXPECT_EQ(model->left_D().total(), 5u);
  EXPECT_NEAR(model->Q().at<double>(2, 3), 500.0, 1e-9);
}

TEST(Rectifier, MissingFileIsUncalibratedAndProducesNoOutput)
{
  depthd::Rectifier rect(tmp_path("nope.yaml"), kMaxRms, kSize);
  EXPECT_FALSE(rect.calibrated());
  EXPECT_EQ(rect.state(), "uncalibrated");
  EXPECT_NE(rect.reason().find("missing"), std::string::npos) << rect.reason();
  EXPECT_FALSE(rect.snapshot());
  cv::cuda::GpuMat l(kSize, CV_8UC1, cv::Scalar(7)), r(kSize, CV_8UC1, cv::Scalar(7)), lo, ro;
  EXPECT_FALSE(rect.rectify(l, r, lo, ro));
  EXPECT_TRUE(lo.empty());
  EXPECT_TRUE(ro.empty());
}

TEST(Rectifier, InvalidFileIsUncalibratedWithReason)
{
  depthd::Rectifier rect(kFixtures + "stereo_invalid_rms.yaml", kMaxRms, kSize);
  EXPECT_FALSE(rect.calibrated());
  EXPECT_NE(rect.reason().find("rms"), std::string::npos) << rect.reason();
}

TEST(Rectifier, ReloadSwapsMapsAndFallsBackToUncalibrated)
{
  const std::string path = tmp_path("reload.yaml");
  auto cal = valid();
  ASSERT_TRUE(depthd::save_calibration(path, cal).empty());
  depthd::Rectifier rect(path, kMaxRms, kSize);
  ASSERT_TRUE(rect.calibrated()) << rect.reason();
  const auto before = rect.snapshot();

  // a new calibration with a 70 mm baseline replaces the file
  cal.baseline_m = 0.07;
  cal.T.at<double>(0, 0) = -0.07;
  cal.P2.at<double>(0, 3) = -35.0;
  ASSERT_TRUE(depthd::save_calibration(path, cal).empty());
  EXPECT_NEAR(rect.snapshot()->baseline_m(), 0.06, 1e-12);  // nothing changes until reload()
  EXPECT_TRUE(rect.reload());
  EXPECT_NEAR(rect.snapshot()->baseline_m(), 0.07, 1e-12);
  EXPECT_NEAR(before->baseline_m(), 0.06, 1e-12);  // a holder of the old snapshot keeps a consistent view

  // an invalid file leaves it uncalibrated with the reason
  cal.rms = 5.0;
  ASSERT_TRUE(depthd::save_calibration(path, cal).empty());
  EXPECT_FALSE(rect.reload());
  EXPECT_FALSE(rect.calibrated());
  EXPECT_NE(rect.reason().find("rms"), std::string::npos);

  // and a valid one brings it back
  ASSERT_TRUE(depthd::save_calibration(path, valid()).empty());
  EXPECT_TRUE(rect.reload());
  EXPECT_TRUE(rect.reason().empty());
  std::remove(path.c_str());
}

TEST(Rectifier, GpuRemapMatchesCpuRemapWithinOneGreyLevel)
{
  ASSERT_GT(cv::cuda::getCudaEnabledDeviceCount(), 0);
  depthd::Rectifier rect(kValid, kMaxRms, kSize);
  ASSERT_TRUE(rect.calibrated()) << rect.reason();

  // Synthetic image with smooth structure (gradient + 2D sine, at most ~6 grey levels per pixel): cv::remap
  // quantises sample positions to 1/32 px, so a sharp step would differ from the exact GPU interpolation by design.
  cv::Mat img(kSize, CV_8UC1);
  for (int y = 0; y < img.rows; ++y)
    for (int x = 0; x < img.cols; ++x)
      img.at<uchar>(y, x) = cv::saturate_cast<uchar>(60.0 + x * 80.0 / img.cols + 50.0 * std::sin(x * 0.1) * std::sin(y * 0.1));
  cv::cuda::GpuMat gl, gr, lo, ro;
  gl.upload(img);
  gr.upload(img);
  ASSERT_TRUE(rect.rectify(gl, gr, lo, ro));
  ASSERT_EQ(lo.size(), kSize);
  ASSERT_EQ(lo.type(), CV_8UC1);

  const auto cal = valid();
  for (int side = 0; side < 2; ++side) {
    cv::Mat mx, my, cpu, gpu;
    const cv::Mat & K = side == 0 ? cal.K1 : cal.K2;
    const cv::Mat & D = side == 0 ? cal.D1 : cal.D2;
    const cv::Mat & R = side == 0 ? cal.R1 : cal.R2;
    const cv::Mat & P = side == 0 ? cal.P1 : cal.P2;
    cv::initUndistortRectifyMap(K, D, R, P, kSize, CV_32FC1, mx, my);
    cv::remap(img, cpu, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    (side == 0 ? lo : ro).download(gpu);
    EXPECT_LE(cv::norm(cpu, gpu, cv::NORM_INF), 1.0) << "side " << side;
    // the rectification actually moved pixels
    EXPECT_GT(cv::norm(cpu, img, cv::NORM_INF), 5.0) << "side " << side;
  }
}
