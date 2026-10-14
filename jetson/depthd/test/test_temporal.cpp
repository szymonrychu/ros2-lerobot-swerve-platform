#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <stdexcept>

#include "depthd/temporal.hpp"

namespace
{
constexpr int kW = 640;
constexpr int kH = 480;
constexpr short k16 = 16;

cv::Mat noisy_disp(int disparity_px, int amplitude, cv::RNG & rng)
{
  cv::Mat noise(kH, kW, CV_16SC1);
  rng.fill(noise, cv::RNG::UNIFORM, -amplitude, amplitude + 1);
  return cv::Mat(kH, kW, CV_16SC1, cv::Scalar(disparity_px * k16)) + noise;
}

cv::Mat apply(depthd::TemporalFilter & f, const cv::Mat & disp, const cv::Mat & grey)
{
  cv::cuda::GpuMat d(disp), g(grey);
  f.apply(d, g);
  cv::Mat out;
  d.download(out);
  return out;
}

double stddev(const cv::Mat & m)
{
  cv::Scalar mean, sd;
  cv::meanStdDev(m, mean, sd);
  return sd[0];
}

cv::Mat grey(int v = 100) { return cv::Mat(kH, kW, CV_8UC1, cv::Scalar(v)); }
}  // namespace

TEST(Temporal, FirstFramePassesThrough)
{
  depthd::TemporalFilter f(0.4, 12);
  cv::RNG rng(1);
  cv::Mat in = noisy_disp(40, 8, rng);
  EXPECT_EQ(cv::countNonZero(apply(f, in, grey()) != in), 0);
}

TEST(Temporal, StaticSceneNoiseDrops)
{
  depthd::TemporalFilter f(0.4, 12);
  cv::RNG rng(2);
  double in_sd = 0, out_sd = 0;
  for (int i = 0; i < 40; ++i) {
    cv::Mat in = noisy_disp(40, 8, rng);
    cv::Mat out = apply(f, in, grey());
    if (i == 39) {
      in_sd = stddev(in);
      out_sd = stddev(out);
    }
  }
  EXPECT_LT(out_sd, 0.6 * in_sd);  // EMA with alpha 0.4 settles near sqrt(a/(2-a)) = 0.5 of the noise
}

TEST(Temporal, AlphaOneIsPassThrough)
{
  depthd::TemporalFilter f(1.0, 12);
  cv::RNG rng(3);
  for (int i = 0; i < 5; ++i) {
    cv::Mat in = noisy_disp(40, 8, rng);
    EXPECT_EQ(cv::countNonZero(apply(f, in, grey()) != in), 0);
  }
}

TEST(Temporal, MovedRegionResets)
{
  depthd::TemporalFilter f(0.4, 12);
  cv::RNG rng(4);
  for (int i = 0; i < 30; ++i) apply(f, noisy_disp(40, 4, rng), grey());
  // an object appears: grey changes by 100 > 12 and the disparity jumps
  cv::Mat g = grey();
  const cv::Rect moved(200, 150, 100, 100);
  g(moved).setTo(200);
  cv::Mat in = cv::Mat(kH, kW, CV_16SC1, cv::Scalar(40 * k16));
  in(moved).setTo(25 * k16);
  cv::Mat out = apply(f, in, g);
  EXPECT_EQ(cv::countNonZero(out(moved) != 25 * k16), 0);  // reset: new value passes through untouched
  // outside the region the filter keeps averaging: the old noisy history still weighs 0.6
  // The input is exactly 40 px there, yet the output still carries the noisy history (weight 0.6): it is averaged.
  cv::Mat outside = out(cv::Rect(400, 0, 200, 100));
  EXPECT_GT(cv::countNonZero(outside != 40 * k16), 0);
  EXPECT_EQ(cv::countNonZero(cv::abs(outside - 40 * k16) > 4), 0);
}

TEST(Temporal, SubThresholdChangeDoesNotReset)
{
  depthd::TemporalFilter f(0.4, 12);
  for (int i = 0; i < 5; ++i) apply(f, cv::Mat(kH, kW, CV_16SC1, cv::Scalar(40 * k16)), grey(100));
  // grey changes by 10 (<= 12), disparity jumps 10 px: EMA moves only alpha of the way
  cv::Mat out = apply(f, cv::Mat(kH, kW, CV_16SC1, cv::Scalar(50 * k16)), grey(110));
  EXPECT_EQ(out.at<short>(240, 320), static_cast<short>(40 * k16 + 0.4 * 10 * k16));
  // 13 > 12: reset
  cv::Mat out2 = apply(f, cv::Mat(kH, kW, CV_16SC1, cv::Scalar(60 * k16)), grey(123));
  EXPECT_EQ(out2.at<short>(240, 320), 60 * k16);
}

TEST(Temporal, InvalidStaysInvalid)
{
  depthd::TemporalFilter f(0.4, 12);
  for (int i = 0; i < 5; ++i) apply(f, cv::Mat(kH, kW, CV_16SC1, cv::Scalar(40 * k16)), grey());
  cv::Mat in(kH, kW, CV_16SC1, cv::Scalar(40 * k16));
  const cv::Rect hole(100, 100, 50, 50);
  in(hole).setTo(0);
  in(cv::Rect(300, 300, 10, 10)).setTo(-1);
  cv::Mat out = apply(f, in, grey());
  EXPECT_EQ(cv::countNonZero(out(hole) != 0), 0);
  EXPECT_EQ(cv::countNonZero(out(cv::Rect(300, 300, 10, 10)) > 0), 0);
  // a pixel that was invalid has no history: when valid again it passes through, history is not resurrected
  cv::Mat back(kH, kW, CV_16SC1, cv::Scalar(70 * k16));
  cv::Mat out2 = apply(f, back, grey());
  EXPECT_EQ(out2.at<short>(120, 120), 70 * k16);
  EXPECT_NE(out2.at<short>(10, 10), 70 * k16);  // valid pixels with history are still averaged
}

TEST(Temporal, ResetAndSizeChangeDropHistory)
{
  depthd::TemporalFilter f(0.4, 12);
  apply(f, cv::Mat(kH, kW, CV_16SC1, cv::Scalar(40 * k16)), grey());
  f.reset();
  EXPECT_EQ(apply(f, cv::Mat(kH, kW, CV_16SC1, cv::Scalar(50 * k16)), grey()).at<short>(5, 5), 50 * k16);
  cv::Mat small(240, 320, CV_16SC1, cv::Scalar(30 * k16)), sg(240, 320, CV_8UC1, cv::Scalar(100));
  cv::cuda::GpuMat d(small), g(sg);
  f.apply(d, g);
  cv::Mat out;
  d.download(out);
  EXPECT_EQ(out.at<short>(5, 5), 30 * k16);
}

TEST(Temporal, RejectsBadArguments)
{
  EXPECT_THROW(depthd::TemporalFilter(0.0, 12), std::invalid_argument);
  EXPECT_THROW(depthd::TemporalFilter(1.5, 12), std::invalid_argument);
  EXPECT_THROW(depthd::TemporalFilter(0.4, -1), std::invalid_argument);
  depthd::TemporalFilter f(0.4, 12);
  cv::cuda::GpuMat bad(cv::Mat(kH, kW, CV_8UC1, cv::Scalar(1))), g(grey());
  EXPECT_THROW(f.apply(bad, g), std::invalid_argument);
  cv::cuda::GpuMat d(cv::Mat(kH, kW, CV_16SC1, cv::Scalar(1))), wrong_size(cv::Mat(10, 10, CV_8UC1, cv::Scalar(1)));
  EXPECT_THROW(f.apply(d, wrong_size), std::invalid_argument);
}
