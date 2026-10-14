#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <tuple>

#include "depthd/config.hpp"
#include "depthd/depth.hpp"
#include "depthd/matcher.hpp"

namespace
{
constexpr int kW = 640;
constexpr int kH = 480;
constexpr double kF = 400.0;
constexpr double kB = 0.06;

// Random texture; a left pixel at x matches right x - shift (disparity = shift).
void make_pair(int shift, cv::Mat & left, cv::Mat & right)
{
  cv::Mat wide(kH, kW + shift, CV_8UC1);
  cv::RNG rng(1234);
  rng.fill(wide, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(wide, wide, cv::Size(3, 3), 0.8);
  left = wide(cv::Rect(0, 0, kW, kH)).clone();
  right = wide(cv::Rect(shift, 0, kW, kH)).clone();  // right(x) = wide(x + shift) -> left(x) == right(x - shift)
}

depthd::Processing processing(int nd, const std::string & matcher = "cuda_sgm")
{
  depthd::Processing p;
  p.matcher = matcher;
  p.num_disparities = nd;
  return p;
}
const std::string kMatchers[] = {"cuda_sgm", "vpi", "libsgm"};
}  // namespace

TEST(MakeMatcher, RejectsUnknownName)
{
  depthd::Processing p = processing(128);
  p.matcher = "nonsense";
  EXPECT_THROW(depthd::make_matcher(p), std::invalid_argument);
  p.matcher = "";
  EXPECT_THROW(depthd::make_matcher(p), std::invalid_argument);
}

TEST(MakeMatcher, RejectsUnsupportedDisparityCount)
{
  for (const std::string & name : kMatchers) {
    EXPECT_THROW(depthd::make_matcher(processing(100, name)), std::invalid_argument) << name;
    EXPECT_THROW(depthd::make_matcher(processing(512, name)), std::invalid_argument) << name;
  }
}

TEST(MakeMatcher, Identity)
{
  for (const std::string & name : kMatchers)
    for (int nd : {64, 128, 256}) {
      auto m = depthd::make_matcher(processing(nd, name));
      ASSERT_TRUE(m) << name;
      EXPECT_EQ(m->name(), name);
      EXPECT_FALSE(m->supports_lr_check()) << name;  // none returns the right-referenced disparity
    }
  depthd::Config c;
  EXPECT_EQ(depthd::make_matcher(c)->name(), "cuda_sgm");
}

class MatcherInputs : public ::testing::TestWithParam<std::string>
{
};

TEST_P(MatcherInputs, RejectsBadInputs)
{
  auto m = depthd::make_matcher(processing(64, GetParam()));
  cv::cuda::GpuMat a(kH, kW, CV_8UC1, cv::Scalar(0)), b(kH, kW - 8, CV_8UC1, cv::Scalar(0));
  cv::cuda::GpuMat c(kH, kW, CV_8UC3, cv::Scalar(0)), empty;
  EXPECT_THROW(m->compute(a, b), std::invalid_argument);
  EXPECT_THROW(m->compute(a, c), std::invalid_argument);
  EXPECT_THROW(m->compute(a, empty), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(Matchers, MatcherInputs, ::testing::ValuesIn(kMatchers));

class MatcherDepth : public ::testing::TestWithParam<std::tuple<std::string, int>>
{
};

TEST_P(MatcherDepth, KnownShiftGivesDepthWithin2Percent)
{
  const std::string name = std::get<0>(GetParam());
  const int nd = std::get<1>(GetParam());
  const int shift = 40;  // Z = 400 * 0.06 / 40 = 0.6 m
  cv::Mat l, r;
  make_pair(shift, l, r);
  cv::cuda::GpuMat gl, gr;
  gl.upload(l);
  gr.upload(r);

  auto m = depthd::make_matcher(processing(nd, name));
  cv::cuda::Stream stream;
  cv::cuda::GpuMat disp = m->compute(gl, gr, stream);
  cv::cuda::GpuMat depth = depthd::disparity_to_depth_mm(disp, kF, kB, 0.2, 6.0, stream);
  stream.waitForCompletion();
  ASSERT_EQ(disp.type(), CV_16SC1);
  ASSERT_EQ(disp.size(), l.size());
  ASSERT_EQ(depth.type(), CV_16UC1);

  cv::Mat d;
  depth.download(d);
  // Textured interior: columns right of the shift plus the matcher window, away from the borders.
  const cv::Rect interior(nd, 16, kW - nd - 16, kH - 32);
  int good = 0, total = 0;
  for (int y = interior.y; y < interior.y + interior.height; ++y)
    for (int x = interior.x; x < interior.x + interior.width; ++x) {
      ++total;
      if (std::abs(static_cast<int>(d.at<uint16_t>(y, x)) - 600) <= 12) ++good;  // 2% of 600 mm
    }
  // All three matchers meet the same bar (vpi measured 100% here, so it needs no relaxed threshold).
  EXPECT_GE(good, static_cast<int>(0.9 * total)) << name << ": " << good << " of " << total << " at nd " << nd;
}

INSTANTIATE_TEST_SUITE_P(Disparities, MatcherDepth,
                         ::testing::Combine(::testing::ValuesIn(kMatchers), ::testing::Values(64, 128)));

// VPI's confidence map: an uncorrelated pair has no real match, so (almost) every pixel must come out invalid.
TEST(VpiConfidence, UncorrelatedPairIsInvalid)
{
  cv::Mat l(kH, kW, CV_8UC1), r(kH, kW, CV_8UC1);
  cv::RNG a(1), b(2);
  a.fill(l, cv::RNG::UNIFORM, 0, 256);
  b.fill(r, cv::RNG::UNIFORM, 0, 256);
  cv::cuda::GpuMat gl, gr;
  gl.upload(l);
  gr.upload(r);
  cv::Mat d;
  depthd::make_matcher(processing(64, "vpi"))->compute(gl, gr).download(d);
  int valid = 0;
  for (int y = 0; y < kH; ++y)
    for (int x = 0; x < kW; ++x)
      if (d.at<int16_t>(y, x) > 0) ++valid;
  EXPECT_LE(valid, kW * kH / 100) << valid << " valid pixels";
}
