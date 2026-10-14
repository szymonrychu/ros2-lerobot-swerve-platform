// The monocular stage (T18): the robust scale + shift fit of mono inverse depth to stereo inverse depth, fill-only-
// invalid, the CUDA stream priorities, the MonoStage in a running pipeline with a fake model (depth unchanged
// with mono on or off, fused frames with the depth frame's header, the rate cap, a model that fails to load), and the
// real TensorRT engines built on this Jetson (DEPTHD_TEST_ENGINE_DIR, default /var/lib/depthd/models).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <opencv2/core/cuda.hpp>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "depthd/calibration.hpp"
#include "depthd/config.hpp"
#include "depthd/http_api.hpp"
#include "depthd/mono.hpp"
#include "depthd/pipeline.hpp"
#include "depthd/publisher.hpp"
#include "depthd/scale_align.hpp"
#include "fake_frame_source.hpp"

namespace
{
using depthd_test::FakeFrameSource;
using depthd_test::realtime_ns;
using depthd_test::Sub;
namespace wire = ros2_common_cpp::depth_wire;

const std::string kFixtures = std::string(DEPTHD_SOURCE_DIR) + "/test/fixtures/";
constexpr double kFB = 500.0 * 0.06;  // fixture f (px) * baseline (m)
constexpr double kD0 = 20.0;          // slanted plane: disparity at right column 0
constexpr double kSlope = 0.03;       // ... grows by this per right column

// ---- fit ----
struct Samples
{
  std::vector<float> mono;
  std::vector<float> target;
};

Samples line_with_outliers(int n, double scale, double shift, double outlier_share, unsigned seed = 1)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> m(0.1, 1.0);
  std::uniform_real_distribution<double> wild(0.17, 5.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, 0.004);
  Samples s;
  for (int i = 0; i < n; ++i) {
    const double mi = m(rng);
    double y = (scale * mi + shift) * (1.0 + noise(rng));
    if (u(rng) < outlier_share) y = wild(rng);
    s.mono.push_back(static_cast<float>(mi));
    s.target.push_back(static_cast<float>(y));
  }
  return s;
}

TEST(FitScaleShift, RecoversKnownScaleAndShiftWith30PercentOutliers)
{
  const Samples s = line_with_outliers(10000, 2.5, 0.3, 0.30);
  const depthd::ScaleShift f = depthd::fit_scale_shift(s.mono, s.target, depthd::AlignParams{});
  ASSERT_TRUE(f.ok) << f.reason;
  EXPECT_NEAR(f.scale, 2.5, 0.05);
  EXPECT_NEAR(f.shift, 0.3, 0.02);
  EXPECT_GE(f.inliers, 6500);
  EXPECT_LE(f.inliers, 7300);
  EXPECT_EQ(f.samples, 10000);
}

TEST(FitScaleShift, TooFewInliersIsNoFit)
{
  // 2000 samples, 85 % outliers: about 300 inliers (plus the few outliers that land on the line) < 500
  const Samples s = line_with_outliers(2000, 2.5, 0.3, 0.85);
  const depthd::ScaleShift f = depthd::fit_scale_shift(s.mono, s.target, depthd::AlignParams{});
  EXPECT_FALSE(f.ok);
  EXPECT_LT(f.inliers, 500);
  EXPECT_FALSE(f.reason.empty());
  // and fewer samples than min_inliers in the first place
  const Samples few = line_with_outliers(400, 2.5, 0.3, 0.0);
  EXPECT_FALSE(depthd::fit_scale_shift(few.mono, few.target, depthd::AlignParams{}).ok);
}

TEST(FitScaleShift, InvertedOrFlatRelationIsNoFit)
{
  const Samples inverted = line_with_outliers(5000, -2.0, 3.0, 0.0);
  EXPECT_FALSE(depthd::fit_scale_shift(inverted.mono, inverted.target, depthd::AlignParams{}).ok);
  // a scene at one depth gives nothing to scale against
  Samples flat = line_with_outliers(5000, 0.0, 0.8, 0.0);
  EXPECT_FALSE(depthd::fit_scale_shift(flat.mono, flat.target, depthd::AlignParams{}).ok);
}

TEST(FitScaleShift, NonFiniteSamplesAreIgnored)
{
  Samples s = line_with_outliers(3000, 1.5, 0.2, 0.0);
  for (int i = 0; i < 500; ++i) {
    s.mono.push_back(std::numeric_limits<float>::quiet_NaN());
    s.target.push_back(1.0F);
  }
  const depthd::ScaleShift f = depthd::fit_scale_shift(s.mono, s.target, depthd::AlignParams{});
  ASSERT_TRUE(f.ok) << f.reason;
  EXPECT_EQ(f.samples, 3000);
  EXPECT_NEAR(f.scale, 1.5, 0.03);
}

// ---- fill ----
// y (1/m) = 0.3 + 0.01 x over a 160x120 image; mono = (y - 0.2) / 2, so scale 2 and shift 0.2.
struct Scene
{
  cv::Mat mono = cv::Mat(120, 160, CV_32FC1);
  cv::Mat stereo = cv::Mat(120, 160, CV_16UC1);
  cv::Mat truth_mm = cv::Mat(120, 160, CV_32FC1);
};

Scene make_scene()
{
  Scene sc;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int y = 0; y < sc.mono.rows; ++y) {
    for (int x = 0; x < sc.mono.cols; ++x) {
      const double inv = 0.3 + 0.01 * x;
      sc.mono.at<float>(y, x) = static_cast<float>((inv - 0.2) / 2.0);
      sc.truth_mm.at<float>(y, x) = static_cast<float>(1000.0 / inv);
      uint16_t mm = static_cast<uint16_t>(std::lround(1000.0 / inv));
      const double r = u(rng);
      if (r < 0.2 || (x >= 40 && x < 80 && y >= 30 && y < 60)) mm = 0;  // holes
      else if (r < 0.3) mm = static_cast<uint16_t>(300 + 4000 * u(rng));  // stereo outliers stay as they are
      sc.stereo.at<uint16_t>(y, x) = mm;
    }
  }
  return sc;
}

TEST(AlignAndFill, ValidStereoNeverChangesAndHolesGetAlignedMonoDepth)
{
  Scene sc = make_scene();
  // a hole whose aligned depth is beyond max_depth_m (inverse 0.05 -> 20 m) and one with a non-finite mono value
  sc.stereo.at<uint16_t>(5, 7) = 0;
  sc.mono.at<float>(5, 7) = static_cast<float>((0.05 - 0.2) / 2.0);
  sc.stereo.at<uint16_t>(6, 7) = 0;
  sc.mono.at<float>(6, 7) = std::numeric_limits<float>::quiet_NaN();

  std::vector<uint16_t> fused;
  const depthd::FillResult r = depthd::align_and_fill(sc.mono, sc.stereo, depthd::AlignParams{}, fused);
  ASSERT_TRUE(r.fit.ok) << r.fit.reason;
  EXPECT_NEAR(r.fit.scale, 2.0, 0.05);
  EXPECT_NEAR(r.fit.shift, 0.2, 0.01);
  ASSERT_EQ(fused.size(), static_cast<std::size_t>(sc.stereo.total()));
  std::size_t holes = 0;
  for (int y = 0; y < sc.stereo.rows; ++y) {
    for (int x = 0; x < sc.stereo.cols; ++x) {
      const uint16_t s = sc.stereo.at<uint16_t>(y, x);
      const uint16_t f = fused[static_cast<std::size_t>(y) * sc.stereo.cols + x];
      if (s != 0) {
        ASSERT_EQ(f, s) << "valid stereo changed at " << x << "," << y;
        continue;
      }
      ++holes;
      if ((y == 5 || y == 6) && x == 7) {
        EXPECT_EQ(f, 0) << "out of range or non-finite must stay invalid";
        continue;
      }
      EXPECT_NEAR(f, sc.truth_mm.at<float>(y, x), 0.02 * sc.truth_mm.at<float>(y, x)) << x << "," << y;
    }
  }
  EXPECT_EQ(r.holes, holes);
  EXPECT_EQ(r.filled, holes - 2);
  EXPECT_NEAR(r.fill_ratio(), static_cast<double>(holes - 2) / holes, 1e-9);
}

TEST(AlignAndFill, NoFitLeavesTheOutputUntouched)
{
  Scene sc = make_scene();
  // keep only 300 valid stereo pixels
  int kept = 0;
  for (int y = 0; y < sc.stereo.rows; ++y) {
    for (int x = 0; x < sc.stereo.cols; ++x) {
      if (sc.stereo.at<uint16_t>(y, x) && kept < 300) {
        ++kept;
      } else {
        sc.stereo.at<uint16_t>(y, x) = 0;
      }
    }
  }
  std::vector<uint16_t> fused = {7, 7, 7};
  const depthd::FillResult r = depthd::align_and_fill(sc.mono, sc.stereo, depthd::AlignParams{}, fused);
  EXPECT_FALSE(r.fit.ok);
  EXPECT_EQ(fused, (std::vector<uint16_t>{7, 7, 7}));
  EXPECT_EQ(r.filled, 0U);
}

// ---- CUDA stream priorities ----
TEST(PriorityStream, StereoStreamOutranksTheMonoStream)
{
  depthd::PriorityStream high(depthd::PriorityStream::Priority::highest);
  depthd::PriorityStream low(depthd::PriorityStream::Priority::lowest);
  int least = 0;
  int greatest = 0;
  ASSERT_EQ(cudaDeviceGetStreamPriorityRange(&least, &greatest), cudaSuccess);
  ASSERT_LT(greatest, least) << "this GPU has no stream priorities";
  EXPECT_EQ(high.priority(), greatest);
  EXPECT_EQ(low.priority(), least);
  cv::cuda::GpuMat a(8, 8, CV_8UC1, cv::Scalar(3));
  cv::cuda::GpuMat b;
  a.copyTo(b, low.stream());
  low.stream().waitForCompletion();
  cv::Mat h;
  b.download(h);
  EXPECT_EQ(h.at<uint8_t>(2, 2), 3);
}

// ---- MonoStage in a pipeline ----
// Knows the slanted plane: relative inverse depth = column / width, which is linear in the true inverse depth.
class ColumnModel : public depthd::MonoModel
{
public:
  explicit ColumnModel(std::atomic<int> * calls = nullptr) : calls_(calls) {}
  void infer(const cv::cuda::GpuMat & grey, cv::Size out_size, cv::cuda::GpuMat & inv_out, cv::cuda::Stream & stream)
    override
  {
    EXPECT_EQ(grey.type(), CV_8UC1);
    cv::Mat host(out_size, CV_32FC1);
    for (int y = 0; y < host.rows; ++y) {
      for (int x = 0; x < host.cols; ++x) host.at<float>(y, x) = static_cast<float>(x) / host.cols;
    }
    inv_out.upload(host, stream);
    if (calls_) ++*calls_;
  }
  cv::Size input_size() const override { return {64, 64}; }

private:
  std::atomic<int> * calls_;
};

depthd::Config pipeline_config(bool mono, double rate_hz = 10.0)
{
  depthd::Config c;
  c.transport.bind_address = "127.0.0.1";
  c.transport.pub_port = 0;
  c.transport.send_hwm = 16;
  c.calibration.path = kFixtures + "stereo_valid.yaml";
  c.processing.matcher = "cuda_sgm";
  c.processing.num_disparities = 64;
  c.mono.enabled = mono;
  c.mono.rate_hz = rate_hz;
  return c;
}

depthd::StereoCalibration fixture_calibration(const depthd::Config & c)
{
  const depthd::CalibrationLoad load = depthd::load_calibration(
    c.calibration.path, cv::Size(c.processing.width, c.processing.height), c.calibration.max_rms_px);
  EXPECT_TRUE(load.ok()) << load.reason;
  return *load.calibration;
}

template <class Pred>
bool eventually(Pred pred, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

struct MonoRig
{
  depthd::Config config;
  depthd::Metrics metrics;
  FakeFrameSource source;
  std::unique_ptr<depthd::Publisher> publisher;
  std::unique_ptr<depthd::Pipeline> pipeline;
  std::unique_ptr<depthd::MonoStage> stage;

  MonoRig(const depthd::Config & c, depthd::MonoModelFactory factory) : config(c)
  {
    publisher = std::make_unique<depthd::Publisher>(config.transport);
    depthd::PipelineOptions opts;
    opts.kmsg_path = "";
    pipeline = std::make_unique<depthd::Pipeline>(config, source, metrics, opts);
    pipeline->set_calibration(fixture_calibration(config));
    if (factory) {
      stage = std::make_unique<depthd::MonoStage>(config, *pipeline, metrics, std::move(factory));
      stage->start();
    }
    std::string err;
    EXPECT_TRUE(pipeline->start(*publisher, &err)) << err;
    // The first match on a fresh process can take seconds (CUDA kernel load next to the running depthd): one
    // fronto-parallel warm-up pair (no depth spread, so never a fused frame) before the test's own pairs.
    const depthd::StereoCalibration cal = fixture_calibration(config);
    source.push_pair(depthd_test::upload_pair(depthd_test::make_raw_pair(cal, 25.0), realtime_ns(), 1));
    EXPECT_TRUE(eventually([&] { return pipeline->counters().depth_published >= 1; }, std::chrono::seconds(30)));
    std::this_thread::sleep_for(std::chrono::milliseconds(600));  // past the warm-up's mono rate period
  }
  ~MonoRig()
  {
    if (stage) stage->stop();
    pipeline->stop();
  }
};

std::vector<wire::Decoded> of_kind(const std::vector<wire::Decoded> & all, wire::Kind kind)
{
  std::vector<wire::Decoded> out;
  for (const auto & d : all) {
    if (d.ok() && d.header.kind == kind) out.push_back(d);
  }
  return out;
}

// One pair through a rig, stamped when it is pushed; returns every message published within 3 s.
std::vector<wire::Decoded> run_one_pair(MonoRig & rig, const depthd_test::RawPair & raw)
{
  Sub sub(rig.publisher->bound_port(), 200);
  rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), 77));
  return sub.drain(std::chrono::milliseconds(3000));
}

TEST(MonoStage, DepthIsUnchangedByMonoAndFusedFillsOnlyHoles)
{
  const depthd::Config off_cfg = pipeline_config(false);
  const depthd::StereoCalibration cal = fixture_calibration(off_cfg);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kD0, 7, kSlope);

  std::vector<wire::Decoded> off;
  {
    MonoRig rig(off_cfg, nullptr);
    off = run_one_pair(rig, raw);
  }
  std::vector<wire::Decoded> on;
  depthd::MonoStats stats;
  depthd::PipelineCounters counters;
  {
    MonoRig rig(pipeline_config(true), [](const depthd::Mono &) { return std::make_unique<ColumnModel>(); });
    ASSERT_TRUE(eventually([&] { return rig.pipeline->health_inputs().mono_state == "ok"; }))
      << rig.pipeline->health_inputs().mono_state;
    on = run_one_pair(rig, raw);
    stats = rig.stage->stats();
    counters = rig.pipeline->counters();
  }
  const auto depth_off = of_kind(off, wire::Kind::kDepth);
  const auto depth_on = of_kind(on, wire::Kind::kDepth);
  const auto fused_on = of_kind(on, wire::Kind::kFused);
  ASSERT_EQ(depth_off.size(), 1U);
  ASSERT_EQ(depth_on.size(), 1U);
  EXPECT_TRUE(of_kind(off, wire::Kind::kFused).empty());
  // Two mono-off runs of this pair already differ in 1-3 pixels (the stereo pipeline on the GPU is not bit-exact
  // between runs), so "unchanged" is within that noise; the fill itself (over 1000 pixels) never reaches depth.
  std::size_t differing = 0;
  for (std::size_t i = 0; i < depth_on.front().depth_mm.size(); ++i) {
    differing += depth_on.front().depth_mm[i] != depth_off.front().depth_mm[i] ? 1 : 0;
  }
  EXPECT_LE(differing, 10U) << "mono changed /stereo/depth/image_raw";

  ASSERT_EQ(fused_on.size(), 1U) << "runs " << stats.runs << " no_fit " << stats.no_fit << " rejected "
                                 << counters.fused_rejected;
  const wire::Decoded & d = depth_on.front();
  const wire::Decoded & f = fused_on.front();
  EXPECT_EQ(f.header.sequence, d.header.sequence);
  EXPECT_EQ(f.header.capture_realtime_ns, d.header.capture_realtime_ns);
  EXPECT_EQ(f.header.width, d.header.width);
  EXPECT_EQ(f.header.frame_id, d.header.frame_id);
  EXPECT_TRUE(f.header.calibrated);
  EXPECT_GT(f.header.stage_mono_ms, 0.0F);
  ASSERT_EQ(f.depth_mm.size(), d.depth_mm.size());
  const int w = static_cast<int>(d.header.width);
  std::size_t filled = 0;
  for (std::size_t i = 0; i < d.depth_mm.size(); ++i) {
    if (d.depth_mm[i] != 0) {
      ASSERT_EQ(f.depth_mm[i], d.depth_mm[i]) << "valid stereo changed at " << i;
    } else if (f.depth_mm[i] != 0) {
      ++filled;
      const double x = static_cast<double>(static_cast<int>(i) % w);
      const double disparity = kD0 + kSlope * (x - kD0) / (1.0 + kSlope);
      const double truth_mm = kFB / disparity * 1000.0;
      EXPECT_NEAR(f.depth_mm[i], truth_mm, 0.06 * truth_mm) << "x " << x;
    }
  }
  EXPECT_GT(filled, 1000U) << "the matcher's invalid left margin should be filled";
  EXPECT_EQ(stats.published, 1U);  // the warm-up plane has no depth spread: no fit, no fused
  EXPECT_GT(stats.last_inliers, 500);
  EXPECT_GT(stats.last_fill_ratio, 0.5);
}

TEST(MonoStage, RateCapLimitsInferences)
{
  std::atomic<int> calls{0};
  MonoRig rig(pipeline_config(true, 2.0), [&calls](const depthd::Mono &) { return std::make_unique<ColumnModel>(&calls); });
  ASSERT_TRUE(eventually([&] { return rig.pipeline->health_inputs().mono_state == "ok"; }));
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kD0, 7, kSlope);
  calls = 0;
  const auto start = std::chrono::steady_clock::now();
  uint64_t seq = 10;
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(3000)) {
    rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), seq++));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_GE(rig.pipeline->counters().depth_published, 11U);
  // 2 Hz on average over 3 s: 6, give or take the frame the window starts or ends on
  EXPECT_GE(calls.load(), 5);
  EXPECT_LE(calls.load(), 7);
}

TEST(MonoStage, DisabledLiveStopsFused)
{
  std::atomic<int> calls{0};
  MonoRig rig(pipeline_config(true, 20.0), [&calls](const depthd::Mono &) { return std::make_unique<ColumnModel>(&calls); });
  ASSERT_TRUE(eventually([&] { return rig.pipeline->health_inputs().mono_state == "ok"; }));
  depthd::Config off = rig.config;
  off.mono.enabled = false;
  rig.pipeline->apply_config(off);
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kD0, 7, kSlope);
  calls = 0;
  for (uint64_t seq = 10; seq <= 14; ++seq) {
    rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), seq));
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }
  ASSERT_TRUE(eventually([&] { return rig.pipeline->counters().depth_published >= 4; }));
  EXPECT_EQ(calls.load(), 0);
  EXPECT_EQ(rig.pipeline->counters().fused_published, 0U);
  EXPECT_EQ(rig.pipeline->health_inputs().mono_state, "disabled");
}

TEST(MonoStage, ModelThatFailsToLoadSetsErrorAndDepthStillFlows)
{
  MonoRig rig(pipeline_config(true), [](const depthd::Mono &) -> std::unique_ptr<depthd::MonoModel> {
    throw std::runtime_error("cannot open /nope.engine");
  });
  ASSERT_TRUE(eventually([&] { return rig.pipeline->health_inputs().mono_state.rfind("error", 0) == 0; }))
    << rig.pipeline->health_inputs().mono_state;
  EXPECT_NE(rig.pipeline->health_inputs().mono_state.find("/nope.engine"), std::string::npos);
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  const auto msgs = run_one_pair(rig, depthd_test::make_raw_pair(cal, kD0, 7, kSlope));
  EXPECT_EQ(of_kind(msgs, wire::Kind::kDepth).size(), 1U);
  EXPECT_TRUE(of_kind(msgs, wire::Kind::kFused).empty());
}

// ---- the engines built on this Jetson (jetson_depthd role engine step) ----
std::string engine_dir()
{
  const char * env = std::getenv("DEPTHD_TEST_ENGINE_DIR");
  return env && *env ? env : "/var/lib/depthd/models";
}

void check_engine(const std::string & name, int side)
{
  depthd::TrtMonoModel model(engine_dir() + "/" + name + ".engine");
  EXPECT_EQ(model.input_size(), cv::Size(side, side));
  // a textured synthetic scene: brighter towards the bottom, plus noise
  cv::Mat grey(480, 640, CV_8UC1);
  cv::RNG rng(11);
  rng.fill(grey, cv::RNG::UNIFORM, 0, 60);
  for (int y = 0; y < grey.rows; ++y) grey.row(y) += cv::Scalar(y * 190 / grey.rows);
  cv::cuda::GpuMat g;
  g.upload(grey);
  depthd::PriorityStream low(depthd::PriorityStream::Priority::lowest);
  cv::cuda::GpuMat inv;
  model.infer(g, cv::Size(640, 480), inv, low.stream());
  low.stream().waitForCompletion();
  ASSERT_EQ(inv.size(), cv::Size(640, 480));
  ASSERT_EQ(inv.type(), CV_32FC1);
  cv::Mat h;
  inv.download(h);
  EXPECT_TRUE(cv::checkRange(h)) << name << " output has non-finite values";
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(h, mean, stddev);
  EXPECT_GT(stddev[0], 1e-6) << name << " output is constant";
}

TEST(MonoEngine, DepthAnythingV2SmallLoadsAndInfers) { check_engine("dav2_small_266", 266); }

TEST(MonoEngine, MidasSmallLoadsAndInfers) { check_engine("midas_small_128", 128); }

TEST(MonoEngine, MissingEngineThrows)
{
  EXPECT_THROW(depthd::TrtMonoModel("/nonexistent/x.engine"), std::runtime_error);
}

}  // namespace
