#include <gtest/gtest.h>

#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <vector>

#include "depthd/depth.hpp"
#include "gpu_alloc_counter.hpp"

using depthd::disparity_to_depth_mm;

namespace
{
// Runs the conversion on a 1 x N row of disparity*16 values.
std::vector<uint16_t> convert(const std::vector<int16_t> & d16, double f, double b, double zmin, double zmax)
{
  cv::Mat host(1, static_cast<int>(d16.size()), CV_16SC1);
  for (size_t i = 0; i < d16.size(); ++i) host.at<int16_t>(0, static_cast<int>(i)) = d16[i];
  cv::cuda::GpuMat dev;
  dev.upload(host);
  cv::cuda::GpuMat out = disparity_to_depth_mm(dev, f, b, zmin, zmax);
  EXPECT_EQ(out.type(), CV_16UC1);
  EXPECT_EQ(out.size(), host.size());
  cv::Mat res;
  out.download(res);
  std::vector<uint16_t> v(d16.size());
  for (size_t i = 0; i < v.size(); ++i) v[i] = res.at<uint16_t>(0, static_cast<int>(i));
  return v;
}
}  // namespace

TEST(DisparityToDepth, ExactValues)
{
  // f 400 px, B 0.06 m: Z_mm = 24000 / d
  auto v = convert({40 * 16, 20 * 16, 80 * 16, 24}, 400.0, 0.06, 0.2, 6.0);
  EXPECT_EQ(v[0], 600);   // 24000 / 40
  EXPECT_EQ(v[1], 1200);  // 24000 / 20
  EXPECT_EQ(v[2], 300);   // 24000 / 80
  EXPECT_EQ(v[3], 0);     // d = 1.5 px -> 16000 mm, beyond the 6 m limit
}

TEST(DisparityToDepth, FractionalDisparityRounds)
{
  // d = 33/16 px: Z = 24000 * 16 / 33 = 11636.36 -> 11636 (inside range 0.2 .. 20 m)
  auto v = convert({33}, 400.0, 0.06, 0.2, 20.0);
  EXPECT_EQ(v[0], 11636);
  // d = 35/16: 10971.43 -> 10971 ; d = 7/16 -> 54857.14 -> 54857
  auto w = convert({35, 7}, 400.0, 0.06, 0.2, 60.0);
  EXPECT_EQ(w[0], 10971);
  EXPECT_EQ(w[1], 54857);
  // rounding up: d = 31/16 -> 12387.10 ; d = 29/16 -> 13241.38 ; d = 23/16 -> 16695.65 -> 16696
  auto x = convert({23}, 400.0, 0.06, 0.2, 60.0);
  EXPECT_EQ(x[0], 16696);
}

TEST(DisparityToDepth, InvalidCasesGiveZero)
{
  auto v = convert({0, -1, -16, -32768, 1}, 400.0, 0.06, 0.2, 6.0);
  EXPECT_EQ(v[0], 0);  // d == 0
  EXPECT_EQ(v[1], 0);  // d < 0
  EXPECT_EQ(v[2], 0);  // matcher invalid marker
  EXPECT_EQ(v[3], 0);  // INT16_MIN
  EXPECT_EQ(v[4], 0);  // d = 1/16 px -> Z = 384 m, beyond max depth
}

TEST(DisparityToDepth, RangeIsInclusiveAndOutsideIsZero)
{
  // Z = 24000 / d. min 0.5 m -> d <= 48 ok ; max 2.0 m -> d >= 12 ok
  auto v = convert({49 * 16, 48 * 16, 12 * 16, 11 * 16}, 400.0, 0.06, 0.5, 2.0);
  EXPECT_EQ(v[0], 0);     // 489.8 mm < 500
  EXPECT_EQ(v[1], 500);   // on the near limit
  EXPECT_EQ(v[2], 2000);  // on the far limit
  EXPECT_EQ(v[3], 0);     // 2181.8 mm > 2000
}

TEST(DisparityToDepth, SaturationNeverWraps)
{
  // Z = 100000 mm at d = 1 px with f*B = 100 m px; a 200 m range would otherwise wrap past 65535.
  auto v = convert({16, 17, 32}, 1000.0, 0.1, 0.2, 200.0);
  EXPECT_EQ(v[0], 65535);  // 100000 saturates
  EXPECT_EQ(v[1], 65535);  // 94117 saturates
  EXPECT_EQ(v[2], 50000);
}

TEST(DisparityToDepth, WholeImageAndStreamAndOutputReuse)
{
  cv::Mat host(48, 64, CV_16SC1, cv::Scalar(40 * 16));
  cv::cuda::GpuMat dev, out;
  dev.upload(host);
  cv::cuda::Stream stream;
  disparity_to_depth_mm(dev, 400.0, 0.06, 0.2, 6.0, out, stream);
  stream.waitForCompletion();
  cv::Mat res;
  out.download(res);
  ASSERT_EQ(res.type(), CV_16UC1);
  ASSERT_EQ(res.size(), host.size());
  EXPECT_EQ(cv::countNonZero(res != 600), 0);
}

TEST(DisparityToDepth, RejectsBadArguments)
{
  cv::cuda::GpuMat f32(4, 4, CV_32FC1, cv::Scalar(1));
  cv::cuda::GpuMat d16(4, 4, CV_16SC1, cv::Scalar(16));
  EXPECT_THROW(disparity_to_depth_mm(f32, 400, 0.06, 0.2, 6.0), std::invalid_argument);
  EXPECT_THROW(disparity_to_depth_mm(d16, 0, 0.06, 0.2, 6.0), std::invalid_argument);
  EXPECT_THROW(disparity_to_depth_mm(d16, 400, -0.06, 0.2, 6.0), std::invalid_argument);
  EXPECT_THROW(disparity_to_depth_mm(d16, 400, 0.06, 6.0, 0.2), std::invalid_argument);
}

// Runs every frame: with the output reused, the conversion allocates no GPU memory (no temporaries).
TEST(DisparityToDepth, SteadyStateAllocatesNothing)
{
  cv::cuda::GpuMat disp(480, 640, CV_16SC1, cv::Scalar(40 * 16)), out;
  cv::cuda::Stream stream;
  disparity_to_depth_mm(disp, 400.0, 0.06, 0.2, 6.0, out, stream);  // first call creates `out`
  stream.waitForCompletion();
  GpuAllocCounter counter;
  disparity_to_depth_mm(disp, 400.0, 0.06, 0.2, 6.0, out, stream);
  stream.waitForCompletion();
  EXPECT_EQ(counter.count, 0);
  cv::Mat host;
  out.download(host);
  EXPECT_EQ(cv::countNonZero(host == 600), 640 * 480);
}
