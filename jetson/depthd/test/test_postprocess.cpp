#include <gtest/gtest.h>

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

#include "depthd/config.hpp"
#include "depthd/matcher.hpp"
#include "depthd/postprocess.hpp"

namespace
{
constexpr int kW = 640;
constexpr int kH = 480;
constexpr short k16 = 16;  // disparity * 16

depthd::Config all_off()
{
  depthd::Config c;
  c.postprocess.lr_check.enabled = false;
  c.postprocess.confidence.enabled = false;
  c.postprocess.speckle.enabled = false;
  c.postprocess.bilateral.enabled = false;
  c.postprocess.hole_fill.enabled = false;
  c.postprocess.temporal.enabled = false;
  c.postprocess.wls.enabled = false;
  return c;
}

cv::Mat constant_disp(int disparity_px)
{
  return cv::Mat(kH, kW, CV_16SC1, cv::Scalar(disparity_px * k16));
}

cv::Mat flat_grey(int v = 128) { return cv::Mat(kH, kW, CV_8UC1, cv::Scalar(v)); }

cv::Mat run_once(depthd::PostProcessor & pp, const cv::Mat & disp, const cv::Mat & left,
                 const cv::Mat * right_disp = nullptr)
{
  cv::cuda::GpuMat d(disp), l(left), r;
  if (right_disp) r.upload(*right_disp);
  pp.run(d, l, right_disp ? &r : nullptr);
  cv::Mat out;
  d.download(out);
  return out;
}

int count_equal(const cv::Mat & m, const cv::Rect & roi, short v)
{
  return cv::countNonZero(m(roi) == v);
}

const depthd::StageTiming * stage(const std::vector<depthd::StageTiming> & t, const std::string & name)
{
  for (const auto & s : t)
    if (s.name == name) return &s;
  return nullptr;
}
}  // namespace

TEST(PostProcess, LrCheckInvalidatesInjectedMismatch)
{
  depthd::Config c = all_off();
  c.postprocess.lr_check.enabled = true;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40), dr = constant_disp(40);
  // The right map disagrees by 10 px inside a block: left pixels that point into it must go invalid.
  dr(cv::Rect(200, 100, 10, 10)).setTo(10 * k16);
  cv::Mat out = run_once(pp, dl, flat_grey(), &dr);
  // left x = 240..249 maps to right x = 200..209
  EXPECT_EQ(count_equal(out, cv::Rect(240, 100, 10, 10), 0), 100);
  // everything away from the mismatch and from the left border survives untouched
  EXPECT_EQ(count_equal(out, cv::Rect(300, 0, 340, kH), 40 * k16), 340 * kH);
  EXPECT_EQ(count_equal(out, cv::Rect(40, 0, 190, kH), 40 * k16), 190 * kH);
  // pixels whose match falls outside the right image have no partner: invalid
  EXPECT_EQ(count_equal(out, cv::Rect(0, 0, 40, kH), 0), 40 * kH);
}

TEST(PostProcess, LrCheckToleratesMaxDiff)
{
  depthd::Config c = all_off();
  c.postprocess.lr_check.enabled = true;
  c.postprocess.lr_check.max_diff = 1;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40), dr = constant_disp(40);
  dr(cv::Rect(200, 100, 10, 10)).setTo(41 * k16);   // exactly 1 px off: kept
  dr(cv::Rect(200, 200, 10, 10)).setTo(42 * k16);   // 2 px off: invalid
  cv::Mat out = run_once(pp, dl, flat_grey(), &dr);
  EXPECT_EQ(count_equal(out, cv::Rect(240, 100, 10, 10), 40 * k16), 100);
  EXPECT_EQ(count_equal(out, cv::Rect(240, 200, 10, 10), 0), 100);
}

TEST(PostProcess, LrCheckNeedsRightDisparityUnlessMatcherIsNative)
{
  depthd::Config c = all_off();
  c.postprocess.lr_check.enabled = true;
  depthd::PostProcessor non_native(c, false);
  EXPECT_TRUE(non_native.needs_right_disparity());
  cv::cuda::GpuMat d(constant_disp(40)), l(flat_grey());
  EXPECT_THROW(non_native.run(d, l, nullptr), std::invalid_argument);

  depthd::PostProcessor native(c, true);
  EXPECT_FALSE(native.needs_right_disparity());
  cv::Mat before = constant_disp(40);
  cv::Mat out = run_once(native, before, flat_grey());
  EXPECT_EQ(cv::countNonZero(out != before), 0);  // the matcher already did the check: nothing runs
  const auto t = native.timings();
  ASSERT_NE(stage(t, "lr_check"), nullptr);
  EXPECT_FALSE(stage(t, "lr_check")->ran);
}

TEST(PostProcess, RightDisparityFromRealMatcherPassesLrCheck)
{
  depthd::Processing p;
  p.matcher = "cuda_sgm";
  p.num_disparities = 128;
  auto matcher = depthd::make_matcher(p);
  ASSERT_FALSE(matcher->supports_lr_check());
  cv::Mat wide(kH, kW + 40, CV_8UC1);
  cv::RNG rng(77);
  rng.fill(wide, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(wide, wide, cv::Size(3, 3), 0.8);
  cv::Mat lm = wide(cv::Rect(0, 0, kW, kH)).clone(), rm = wide(cv::Rect(40, 0, kW, kH)).clone();
  cv::cuda::GpuMat l(lm), r(rm);
  cv::cuda::GpuMat dl = matcher->compute(l, r);
  cv::cuda::GpuMat dr = depthd::compute_right_disparity(*matcher, l, r);
  cv::Mat dr_h;
  dr.download(dr_h);
  ASSERT_EQ(dr_h.type(), CV_16SC1);
  // right-referenced disparity is also 40 px in the interior
  cv::Mat inner = dr_h(cv::Rect(60, 40, kW - 160, kH - 80));
  EXPECT_GT(cv::countNonZero(cv::abs(inner - 40 * k16) <= k16), 0.9 * inner.total());

  depthd::Config c = all_off();
  c.postprocess.lr_check.enabled = true;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl_h;
  dl.download(dl_h);
  cv::Mat out = run_once(pp, dl_h, lm, &dr_h);
  cv::Mat inner_out = out(cv::Rect(60, 40, kW - 160, kH - 80));
  EXPECT_GT(cv::countNonZero(inner_out > 0), 0.85 * inner_out.total());
}

TEST(PostProcess, ConfidenceInvalidatesOutOfRangeAndOutsideTheRightImage)
{
  depthd::Config c = all_off();
  c.postprocess.confidence.enabled = true;  // processing.num_disparities = 128
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(300, 50, 20, 20)).setTo(0);                  // already invalid stays invalid
  dl(cv::Rect(300, 100, 20, 20)).setTo(128 * k16);         // at/above the search range: saturated
  dl(cv::Rect(300, 150, 20, 20)).setTo(k16 / 2);           // below 1 px: no depth
  dl(cv::Rect(300, 200, 20, 20)).setTo(-1);                // matcher invalid marker
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(300, 50, 20, 20), 0), 400);
  EXPECT_EQ(count_equal(out, cv::Rect(300, 100, 20, 20), 0), 400);
  EXPECT_EQ(count_equal(out, cv::Rect(300, 150, 20, 20), 0), 400);
  EXPECT_EQ(count_equal(out, cv::Rect(300, 200, 20, 20), 0), 400);
  EXPECT_EQ(count_equal(out, cv::Rect(100, 300, 400, 100), 40 * k16), 400 * 100);
  // x - d < 0: no partner pixel exists in the right image
  EXPECT_EQ(count_equal(out, cv::Rect(0, 0, 40, 40), 0), 1600);
}

TEST(PostProcess, SpeckleRemovesSmallBlobAndKeepsLargeRegion)
{
  depthd::Config c = all_off();
  c.postprocess.speckle.enabled = true;  // max_size 200, max_diff 1
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(300, 100, 5, 5)).setTo(60 * k16);      // 25 px blob
  dl(cv::Rect(400, 200, 100, 100)).setTo(20 * k16);  // 10000 px region, different disparity
  dl(cv::Rect(100, 300, 3, 3)).setTo(40 * k16 + 10);  // within 1 px of the surround: merges, kept
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(300, 100, 5, 5), 0), 25);
  EXPECT_EQ(count_equal(out, cv::Rect(400, 200, 100, 100), 20 * k16), 10000);
  EXPECT_EQ(count_equal(out, cv::Rect(100, 300, 3, 3), 40 * k16 + 10), 9);
  EXPECT_EQ(cv::countNonZero(out == 0), 25);
}

TEST(PostProcess, SpeckleHonoursMaxSize)
{
  depthd::Config c = all_off();
  c.postprocess.speckle.enabled = true;
  c.postprocess.speckle.max_size = 10;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(300, 100, 5, 5)).setTo(60 * k16);  // 25 px >= 10: kept
  dl(cv::Rect(300, 200, 3, 3)).setTo(60 * k16);  // 9 px < 10: removed
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(300, 100, 5, 5), 60 * k16), 25);
  EXPECT_EQ(count_equal(out, cv::Rect(300, 200, 3, 3), 0), 9);
}

TEST(PostProcess, HoleFillFillsSmallHoleAndLeavesLargeHoleAtZero)
{
  depthd::Config c = all_off();
  c.postprocess.hole_fill.enabled = true;  // max_hole_px 64
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(100, 100, 5, 5)).setTo(0);    // 25 px
  dl(cv::Rect(300, 200, 30, 30)).setTo(0);  // 900 px
  dl(cv::Rect(500, 400, 1, 1)).setTo(-1);   // 1 px, matcher marker
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(100, 100, 5, 5), 40 * k16), 25);
  EXPECT_EQ(count_equal(out, cv::Rect(300, 200, 30, 30), 0), 900);
  EXPECT_EQ(out.at<short>(400, 500), 40 * k16);
  EXPECT_EQ(cv::countNonZero(out == 0), 900);
}

TEST(PostProcess, HoleFillRespectsMaxHolePx)
{
  depthd::Config c = all_off();
  c.postprocess.hole_fill.enabled = true;
  c.postprocess.hole_fill.max_hole_px = 20;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(100, 100, 5, 5)).setTo(0);  // 25 > 20: stays
  dl(cv::Rect(200, 100, 4, 5)).setTo(0);  // 20: filled
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(100, 100, 5, 5), 0), 25);
  EXPECT_EQ(count_equal(out, cv::Rect(200, 100, 4, 5), 40 * k16), 20);
}

TEST(PostProcess, HoleFillIsEdgeAware)
{
  depthd::Config c = all_off();
  c.postprocess.hole_fill.enabled = true;
  depthd::PostProcessor pp(c, false);
  // Disparity step at column 320 that matches an intensity step in the left image; the hole straddles it.
  cv::Mat dl(kH, kW, CV_16SC1), left(kH, kW, CV_8UC1);
  dl(cv::Rect(0, 0, 320, kH)).setTo(30 * k16);
  dl(cv::Rect(320, 0, 320, kH)).setTo(20 * k16);
  left(cv::Rect(0, 0, 320, kH)).setTo(40);
  left(cv::Rect(320, 0, 320, kH)).setTo(210);
  dl(cv::Rect(317, 200, 6, 5)).setTo(0);
  cv::Mat out = run_once(pp, dl, left);
  EXPECT_EQ(count_equal(out, cv::Rect(317, 200, 3, 5), 30 * k16), 15);
  EXPECT_EQ(count_equal(out, cv::Rect(320, 200, 3, 5), 20 * k16), 15);
}

TEST(PostProcess, BilateralKeepsStepEdgeWithinOnePixel)
{
  depthd::Config c = all_off();
  c.postprocess.bilateral.enabled = true;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl(kH, kW, CV_16SC1), left(kH, kW, CV_8UC1);
  dl(cv::Rect(0, 0, 320, kH)).setTo(40 * k16);
  dl(cv::Rect(320, 0, 320, kH)).setTo(20 * k16);
  left(cv::Rect(0, 0, 320, kH)).setTo(40);
  left(cv::Rect(320, 0, 320, kH)).setTo(210);
  cv::RNG rng(5);
  cv::Mat noise(kH, kW, CV_16SC1);
  rng.fill(noise, cv::RNG::UNIFORM, -6, 7);
  cv::Mat noisy = dl + noise;
  cv::Mat out = run_once(pp, noisy, left);
  const short mid = 30 * k16;
  for (int y : {50, 240, 430}) {
    int edge = -1;
    for (int x = 1; x < kW; ++x)
      if (out.at<short>(y, x - 1) > mid && out.at<short>(y, x) <= mid) {
        edge = x;
        break;
      }
    EXPECT_NEAR(edge, 320, 1) << "row " << y;
  }
  // flat areas stay where they were
  EXPECT_NEAR(cv::mean(out(cv::Rect(50, 50, 200, 200)))[0], 40 * k16, 2.0);
  EXPECT_NEAR(cv::mean(out(cv::Rect(400, 50, 200, 200)))[0], 20 * k16, 2.0);
}

TEST(PostProcess, BilateralDoesNotValidateInvalidPixels)
{
  depthd::Config c = all_off();
  c.postprocess.bilateral.enabled = true;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(300, 200, 40, 40)).setTo(0);
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(300, 200, 40, 40), 0), 1600);
}

TEST(PostProcess, WlsRunsOnCpuAndKeepsInvalidInvalid)
{
  depthd::Config c = all_off();
  c.postprocess.wls.enabled = true;
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(300, 200, 40, 40)).setTo(0);
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(count_equal(out, cv::Rect(300, 200, 40, 40), 0), 1600);
  cv::Mat inner = out(cv::Rect(50, 50, 200, 200));
  EXPECT_EQ(cv::countNonZero(cv::abs(inner - 40 * k16) <= 2), 200 * 200);
  EXPECT_TRUE(stage(pp.timings(), "wls")->ran);
}

TEST(PostProcess, DisabledStagesDoNothingAndAreNotTimed)
{
  depthd::PostProcessor pp(all_off(), false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(10, 10, 5, 5)).setTo(0);
  dl(cv::Rect(300, 100, 5, 5)).setTo(60 * k16);
  cv::Mat out = run_once(pp, dl, flat_grey());
  EXPECT_EQ(cv::countNonZero(out != dl), 0);
  for (const auto & t : pp.timings()) {
    EXPECT_FALSE(t.ran) << t.name;
    EXPECT_EQ(t.ms, 0.f) << t.name;
    EXPECT_EQ(t.runs, 0u) << t.name;
  }
  // no kernel launch: a run with everything off is a few host instructions
  cv::cuda::GpuMat d(dl), l(flat_grey());
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000; ++i) pp.run(d, l, nullptr);
  const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / 1000;
  EXPECT_LT(us, 20.0);
}

TEST(PostProcess, EachToggleRunsOnlyItsStageAndTimesIt)
{
  const char * names[] = {"lr_check", "confidence", "speckle", "bilateral", "hole_fill", "wls", "temporal"};
  const cv::Mat dr = constant_disp(40);
  for (const char * enabled : names) {
    depthd::Config c = all_off();
    depthd::Postprocess & p = c.postprocess;
    const std::string n = enabled;
    p.lr_check.enabled = n == "lr_check";
    p.confidence.enabled = n == "confidence";
    p.speckle.enabled = n == "speckle";
    p.bilateral.enabled = n == "bilateral";
    p.hole_fill.enabled = n == "hole_fill";
    p.wls.enabled = n == "wls";
    p.temporal.enabled = n == "temporal";
    depthd::PostProcessor pp(c, false);
    run_once(pp, constant_disp(40), flat_grey(), &dr);
    const auto t = pp.timings();
    ASSERT_EQ(t.size(), 7u);
    for (size_t i = 0; i < t.size(); ++i) {
      EXPECT_EQ(t[i].name, names[i]);  // the documented stage order
      EXPECT_EQ(t[i].ran, t[i].name == n) << "enabled " << n << " stage " << t[i].name;
      if (t[i].ran) {
        EXPECT_GT(t[i].ms, 0.f) << t[i].name;
        EXPECT_EQ(t[i].runs, 1u);
      }
    }
  }
}

TEST(PostProcess, TogglesChangeAtRuntime)
{
  depthd::Config c = all_off();
  depthd::PostProcessor pp(c, false);
  cv::Mat dl = constant_disp(40);
  dl(cv::Rect(300, 100, 5, 5)).setTo(60 * k16);
  EXPECT_EQ(cv::countNonZero(run_once(pp, dl, flat_grey()) == 0), 0);
  depthd::Postprocess p = c.postprocess;
  p.speckle.enabled = true;
  pp.set_postprocess(p);
  EXPECT_EQ(cv::countNonZero(run_once(pp, dl, flat_grey()) == 0), 25);
  p.speckle.enabled = false;
  pp.set_postprocess(p);
  EXPECT_EQ(cv::countNonZero(run_once(pp, dl, flat_grey()) == 0), 0);
}

TEST(PostProcess, RejectsBadInputs)
{
  depthd::PostProcessor pp(all_off(), false);
  cv::cuda::GpuMat wrong_type(cv::Mat(kH, kW, CV_8UC1, cv::Scalar(1))), l(flat_grey());
  EXPECT_THROW(pp.run(wrong_type, l, nullptr), std::invalid_argument);
  cv::cuda::GpuMat d(constant_disp(40)), small(cv::Mat(10, 10, CV_8UC1, cv::Scalar(1)));
  EXPECT_THROW(pp.run(d, small, nullptr), std::invalid_argument);
}
