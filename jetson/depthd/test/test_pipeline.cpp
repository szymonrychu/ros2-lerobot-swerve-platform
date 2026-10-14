// depthd::Pipeline end to end on the GPU: a test FrameSource feeds synthetic shifted pairs (a plane at a known
// disparity) and JPEGs, the valid fixture calibration rectifies them, and a ZeroMQ SUB on loopback decodes what the
// pipeline publishes. Covers the depth value, header stamps, intrinsics and frame ids, rgb with and without
// calibration, the no-depth/no-fused rule when uncalibrated, camera_error gating, the 100 ms stale-pair drop, the
// malformed-pair drop and the
// bounded two-slot hand-off that never makes capture wait.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "depthd/calibration.hpp"
#include "depthd/config.hpp"
#include "depthd/http_api.hpp"
#include "depthd/pipeline.hpp"
#include "depthd/publisher.hpp"
#include "fake_frame_source.hpp"

namespace
{
using depthd_test::FakeFrameSource;
using depthd_test::realtime_ns;
using depthd_test::Sub;
namespace wire = ros2_common_cpp::depth_wire;

const std::string kFixtures = std::string(DEPTHD_SOURCE_DIR) + "/test/fixtures/";
constexpr double kDisparity = 25.0;  // px; the fixture has f = 500 px, B = 0.06 m -> Z = 1.2 m
constexpr double kExpectedMm = 500.0 * 0.06 / kDisparity * 1000.0;

depthd::Config test_config()
{
  depthd::Config c;
  c.transport.bind_address = "127.0.0.1";
  c.transport.pub_port = 0;
  c.transport.send_hwm = 16;
  c.calibration.path = kFixtures + "stereo_valid.yaml";
  c.processing.matcher = "cuda_sgm";
  c.processing.num_disparities = 64;
  return c;
}

depthd::StereoCalibration fixture_calibration(const depthd::Config & c)
{
  const depthd::CalibrationLoad load = depthd::load_calibration(
    c.calibration.path, cv::Size(c.processing.width, c.processing.height), c.calibration.max_rms_px);
  EXPECT_TRUE(load.ok()) << load.reason;
  return *load.calibration;
}

struct Rig
{
  depthd::Config config = test_config();
  depthd::Metrics metrics;
  FakeFrameSource source;
  std::unique_ptr<depthd::Publisher> publisher;
  std::unique_ptr<depthd::Pipeline> pipeline;
  int64_t clock_offset_ns = 0;  // added to the real clock the pipeline reads

  explicit Rig(const depthd::Config & c = test_config()) : config(c)
  {
    publisher = std::make_unique<depthd::Publisher>(config.transport);
    depthd::PipelineOptions opts;
    opts.realtime_now = [this] { return realtime_ns() + clock_offset_ns; };
    opts.kmsg_path = "";  // no kernel log in unit tests
    pipeline = std::make_unique<depthd::Pipeline>(config, source, metrics, opts);
    std::string err;
    EXPECT_TRUE(pipeline->start(*publisher, &err)) << err;
  }
  ~Rig() { pipeline->stop(); }
};

std::vector<wire::Decoded> of_kind(const std::vector<wire::Decoded> & all, wire::Kind kind)
{
  std::vector<wire::Decoded> out;
  for (const auto & d : all) {
    if (d.ok() && d.header.kind == kind) out.push_back(d);
  }
  return out;
}

// Waits until `pred` holds or `timeout` passes.
template <class Pred>
bool eventually(Pred pred, std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

// Median and valid share of the central region of a depth image (left matcher margin excluded).
struct DepthStats
{
  double median_mm = 0.0;
  double valid_share = 0.0;
};
DepthStats central_stats(const std::vector<uint16_t> & depth, int width, int height)
{
  std::vector<uint16_t> valid;
  int total = 0;
  for (int y = height / 4; y < 3 * height / 4; ++y) {
    for (int x = width / 4; x < 3 * width / 4; ++x) {
      ++total;
      const uint16_t v = depth[static_cast<size_t>(y) * width + x];
      if (v > 0) valid.push_back(v);
    }
  }
  DepthStats s;
  if (valid.empty()) return s;
  std::nth_element(valid.begin(), valid.begin() + valid.size() / 2, valid.end());
  s.median_mm = valid[valid.size() / 2];
  s.valid_share = static_cast<double>(valid.size()) / total;
  return s;
}

}  // namespace

TEST(Pipeline, PublishesCorrectDepthWithStampsIntrinsicsAndFrameIds)
{
  Rig rig;
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  rig.pipeline->set_calibration(cal);
  ASSERT_TRUE(rig.pipeline->calibrated());
  Sub sub(rig.publisher->bound_port(), 5000);  // the first match also warms the matcher up

  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);
  const int64_t stamp = realtime_ns();
  rig.source.push_pair(depthd_test::upload_pair(raw, stamp, 41));

  wire::Decoded msg;
  ASSERT_TRUE(sub.recv(msg)) << "no depth message";
  ASSERT_TRUE(msg.ok()) << msg.error;
  EXPECT_EQ(msg.header.kind, wire::Kind::kDepth);
  EXPECT_EQ(msg.header.capture_realtime_ns, stamp);
  EXPECT_EQ(msg.header.right_minus_left_ns, 1500000);
  EXPECT_EQ(msg.header.sequence, 41U);
  EXPECT_EQ(msg.header.frame_id, rig.config.frame_ids.depth);
  EXPECT_EQ(msg.header.width, 640U);
  EXPECT_EQ(msg.header.height, 480U);
  EXPECT_TRUE(msg.header.calibrated);
  EXPECT_DOUBLE_EQ(msg.header.baseline_m, 0.06);
  // rectified P (P1) and the unrectified left K/D
  EXPECT_DOUBLE_EQ(msg.header.p[0], 500.0);
  EXPECT_DOUBLE_EQ(msg.header.p[2], 320.0);
  EXPECT_DOUBLE_EQ(msg.header.p[5], 500.0);
  EXPECT_DOUBLE_EQ(msg.header.p[6], 240.0);
  EXPECT_DOUBLE_EQ(msg.header.k[0], 510.0);
  EXPECT_DOUBLE_EQ(msg.header.k[2], 322.0);
  EXPECT_DOUBLE_EQ(msg.header.d[0], -0.12);
  // health fields travel with every frame
  EXPECT_GT(msg.header.stage_match_ms, 0.0F);
  EXPECT_GE(msg.header.stage_capture_ms, 0.0F);

  ASSERT_EQ(msg.depth_mm.size(), 640U * 480U);
  const DepthStats s = central_stats(msg.depth_mm, 640, 480);
  EXPECT_GT(s.valid_share, 0.8) << "too few valid pixels";
  EXPECT_NEAR(s.median_mm, kExpectedMm, kExpectedMm * 0.03);

  EXPECT_TRUE(eventually([&] { return rig.pipeline->counters().depth_published == 1U; }));
  EXPECT_EQ(rig.pipeline->health().state, "ok");
  EXPECT_NE(rig.metrics.render().find("depthd_frames_total{kind=\"depth\"} 1"), std::string::npos);
}

TEST(Pipeline, RgbCarriesLeftIntrinsicsScaledToTheRgbSizeOnlyWhenCalibrated)
{
  depthd::Config c = test_config();
  c.rgb.width = 320;
  c.rgb.height = 240;
  Rig rig(c);
  Sub sub(rig.publisher->bound_port(), 1000);

  const int64_t t0 = realtime_ns();
  rig.source.push_rgb(depthd_test::make_jpeg(320, 240, t0, 5));
  wire::Decoded before;
  ASSERT_TRUE(sub.recv(before));
  ASSERT_TRUE(before.ok()) << before.error;
  EXPECT_EQ(before.header.kind, wire::Kind::kRgb);
  EXPECT_EQ(before.header.frame_id, c.frame_ids.rgb);
  EXPECT_EQ(before.header.capture_realtime_ns, t0);
  EXPECT_EQ(before.header.sequence, 5U);
  EXPECT_EQ(before.header.width, 320U);
  EXPECT_EQ(before.header.height, 240U);
  EXPECT_FALSE(before.header.calibrated);
  EXPECT_DOUBLE_EQ(before.header.k[0], 0.0);
  EXPECT_FALSE(before.jpeg.empty());

  rig.pipeline->set_calibration(fixture_calibration(c));
  rig.source.push_rgb(depthd_test::make_jpeg(320, 240, realtime_ns(), 6));
  wire::Decoded after;
  ASSERT_TRUE(sub.recv(after));
  ASSERT_TRUE(after.ok()) << after.error;
  EXPECT_TRUE(after.header.calibrated);
  EXPECT_DOUBLE_EQ(after.header.k[0], 255.0);  // 510 * 320 / 640
  EXPECT_DOUBLE_EQ(after.header.k[2], 161.0);  // 322 * 320 / 640
  EXPECT_DOUBLE_EQ(after.header.k[4], 255.0);
  EXPECT_DOUBLE_EQ(after.header.k[8], 1.0);
  EXPECT_DOUBLE_EQ(after.header.d[0], -0.12);
}

TEST(Pipeline, WithoutCalibrationPublishesRgbButNeverDepthOrFused)
{
  depthd::Config c = test_config();
  c.calibration.path = kFixtures + "does_not_exist.yaml";
  Rig rig(c);
  Sub sub(rig.publisher->bound_port(), 100);
  ASSERT_FALSE(rig.pipeline->calibrated());

  const depthd::StereoCalibration cal = fixture_calibration(test_config());
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);
  for (int i = 0; i < 3; ++i) {
    rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), i));
    rig.source.push_rgb(depthd_test::make_jpeg(640, 480, realtime_ns(), i));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }
  wire::Header fused;
  fused.width = 2;
  fused.height = 1;
  EXPECT_FALSE(rig.pipeline->publish_fused(fused, {1000, 1000}));

  const auto all = sub.drain(std::chrono::milliseconds(800));
  EXPECT_EQ(of_kind(all, wire::Kind::kRgb).size(), 3U);
  EXPECT_TRUE(of_kind(all, wire::Kind::kDepth).empty());
  EXPECT_TRUE(of_kind(all, wire::Kind::kFused).empty());
  const depthd::PipelineCounters n = rig.pipeline->counters();
  EXPECT_EQ(n.depth_published, 0U);
  EXPECT_EQ(n.dropped_uncalibrated, 3U);
  EXPECT_EQ(n.fused_rejected, 1U);
  EXPECT_EQ(rig.pipeline->health().state, "uncalibrated");
}

TEST(Pipeline, FusedIsPublishedOnlyWhileCalibratedAndBothCamerasAreHealthy)
{
  Rig rig;
  rig.pipeline->set_calibration(fixture_calibration(rig.config));
  Sub sub(rig.publisher->bound_port(), 1000);
  wire::Header h;
  h.capture_realtime_ns = realtime_ns();
  h.width = 2;
  h.height = 1;
  ASSERT_TRUE(rig.pipeline->publish_fused(h, {1200, 0}));
  wire::Decoded msg;
  ASSERT_TRUE(sub.recv(msg));
  ASSERT_TRUE(msg.ok()) << msg.error;
  EXPECT_EQ(msg.header.kind, wire::Kind::kFused);
  EXPECT_EQ(msg.header.frame_id, rig.config.frame_ids.depth);
  EXPECT_TRUE(msg.header.calibrated);

  depthd::CaptureStatus broken;
  broken.state = depthd::CaptureState::camera_error;
  broken.reason = "right camera (sensor-id 1): delivered no frame for 2.1 s";
  broken.right_error = true;
  rig.source.set_status(broken);
  h.capture_realtime_ns = realtime_ns();
  EXPECT_FALSE(rig.pipeline->publish_fused(h, {1200, 0}));
  // a stale fused frame is refused too
  rig.source.set_status({});
  h.capture_realtime_ns = realtime_ns() - 500000000;
  EXPECT_FALSE(rig.pipeline->publish_fused(h, {1200, 0}));
  // one inference behind its depth frame (older than the 100 ms pair budget, within max_fused_age_ns) is published,
  // and its mono stage time is kept
  h.capture_realtime_ns = realtime_ns() - 300000000;
  h.stage_mono_ms = 187.5F;
  ASSERT_TRUE(rig.pipeline->publish_fused(h, {1200, 0}));
  ASSERT_TRUE(sub.recv(msg));
  ASSERT_TRUE(msg.ok()) << msg.error;
  EXPECT_EQ(msg.header.kind, wire::Kind::kFused);
  EXPECT_FLOAT_EQ(msg.header.stage_mono_ms, 187.5F);
}

TEST(Pipeline, CameraErrorSuppressesThatCamerasMessages)
{
  Rig rig;
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  rig.pipeline->set_calibration(cal);
  Sub sub(rig.publisher->bound_port(), 100);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);

  // The right camera fails: no depth (needs both), rgb (left camera) keeps flowing.
  depthd::CaptureStatus right_down;
  right_down.state = depthd::CaptureState::camera_error;
  right_down.reason = "right camera (sensor-id 1): stream ended";
  right_down.right_error = true;
  rig.source.set_status(right_down);
  rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), 1));
  rig.source.push_rgb(depthd_test::make_jpeg(640, 480, realtime_ns(), 1));
  auto all = sub.drain(std::chrono::milliseconds(500));
  EXPECT_TRUE(of_kind(all, wire::Kind::kDepth).empty());
  EXPECT_EQ(of_kind(all, wire::Kind::kRgb).size(), 1U);
  EXPECT_EQ(rig.pipeline->health().state, "camera_error");
  EXPECT_NE(rig.pipeline->health().reason.find("stream ended"), std::string::npos);

  // The left camera fails: neither depth nor rgb.
  depthd::CaptureStatus left_down;
  left_down.state = depthd::CaptureState::camera_error;
  left_down.reason = "left camera (sensor-id 0): stream ended";
  left_down.left_error = true;
  rig.source.set_status(left_down);
  rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), 2));
  rig.source.push_rgb(depthd_test::make_jpeg(640, 480, realtime_ns(), 2));
  all = sub.drain(std::chrono::milliseconds(500));
  EXPECT_TRUE(all.empty());
  const depthd::PipelineCounters n = rig.pipeline->counters();
  EXPECT_EQ(n.dropped_camera_error, 2U);
  EXPECT_EQ(n.rgb_dropped_camera_error, 1U);
}

TEST(Pipeline, PairOlderThan100msWhenMatchingStartsIsDroppedAndCounted)
{
  Rig rig;
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  rig.pipeline->set_calibration(cal);
  Sub sub(rig.publisher->bound_port(), 100);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);

  rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns() - 150000000, 1));  // 150 ms old
  rig.source.push_rgb(depthd_test::make_jpeg(640, 480, realtime_ns() - 150000000, 1));
  auto all = sub.drain(std::chrono::milliseconds(500));
  EXPECT_TRUE(all.empty()) << "a stale frame was published";
  depthd::PipelineCounters n = rig.pipeline->counters();
  EXPECT_EQ(n.dropped_stale, 1U);
  EXPECT_EQ(n.rgb_dropped_stale, 1U);
  EXPECT_EQ(rig.pipeline->health().pairs_dropped_total, 1U);
  EXPECT_NE(rig.metrics.render().find("depthd_drops_total{reason=\"stale\"} 1"), std::string::npos);

  // A capture time in the future (clock step) is not trusted either.
  rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns() + 500000000, 2));
  EXPECT_TRUE(eventually([&] { return rig.pipeline->counters().dropped_stale == 2U; }));

  // A fresh pair right after goes through.
  rig.source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), 3));
  all = sub.drain(std::chrono::milliseconds(1500));
  ASSERT_EQ(of_kind(all, wire::Kind::kDepth).size(), 1U);
  EXPECT_EQ(of_kind(all, wire::Kind::kDepth)[0].header.sequence, 3U);
}

TEST(Pipeline, MalformedPairIsDroppedAndCountedNeverMatched)
{
  Rig rig;
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  rig.pipeline->set_calibration(cal);
  Sub sub(rig.publisher->bound_port(), 100);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);

  // Wrong size: not the calibrated resolution.
  depthd_test::RawPair small;
  cv::resize(raw.left, small.left, cv::Size(), 0.5, 0.5);
  cv::resize(raw.right, small.right, cv::Size(), 0.5, 0.5);
  rig.source.push_pair(depthd_test::upload_pair(small, realtime_ns(), 1));
  EXPECT_TRUE(eventually([&] { return rig.pipeline->counters().dropped_bad_frame == 1U; }));

  // Wrong type: a colour frame where the matcher expects mono.
  depthd_test::RawPair colour;
  cv::cvtColor(raw.left, colour.left, cv::COLOR_GRAY2BGR);
  cv::cvtColor(raw.right, colour.right, cv::COLOR_GRAY2BGR);
  rig.source.push_pair(depthd_test::upload_pair(colour, realtime_ns(), 2));
  EXPECT_TRUE(eventually([&] { return rig.pipeline->counters().dropped_bad_frame == 2U; }));

  // An empty placeholder pair.
  rig.source.push_pair(depthd::StereoPair{});
  EXPECT_TRUE(eventually([&] { return rig.pipeline->counters().dropped_bad_frame == 3U; }));

  const auto all = sub.drain(std::chrono::milliseconds(300));
  EXPECT_TRUE(of_kind(all, wire::Kind::kDepth).empty()) << "a malformed pair was published";
  const depthd::PipelineCounters n = rig.pipeline->counters();
  EXPECT_EQ(n.dropped_error, 0U);
  EXPECT_EQ(rig.pipeline->health().pairs_dropped_total, 3U);
  EXPECT_NE(rig.metrics.render().find("depthd_drops_total{reason=\"bad_frame\"} 3"), std::string::npos);
}

TEST(Pipeline, CaptureNeverWaitsAndExcessPairsAreDroppedNotQueued)
{
  Rig rig;
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  rig.pipeline->set_calibration(cal);
  Sub sub(rig.publisher->bound_port(), 100);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);
  std::vector<depthd::StereoPair> pairs;
  for (int i = 0; i < 12; ++i) pairs.push_back(depthd_test::upload_pair(raw, 0, i));

  auto worst = std::chrono::nanoseconds(0);
  for (auto & p : pairs) {
    p.capture_realtime_ns = realtime_ns();
    const auto t0 = std::chrono::steady_clock::now();
    rig.source.push_pair(p);
    worst = std::max(worst, std::chrono::steady_clock::now() - t0);
  }
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(worst).count(), 5)
    << "the capture callback waited on the pipeline";
  ASSERT_TRUE(eventually([&] {
    const auto n = rig.pipeline->counters();
    return n.depth_published + n.dropped_busy + n.dropped_stale == 12U;
  }));
  const depthd::PipelineCounters n = rig.pipeline->counters();
  EXPECT_GE(n.dropped_busy, 1U) << "12 back-to-back pairs cannot all be matched";
  EXPECT_GE(n.depth_published, 1U);
  EXPECT_LE(rig.pipeline->in_flight_max(), static_cast<std::size_t>(depthd::Pipeline::kSlots));
}

TEST(Pipeline, LatestPairServesTheCalibrationCaptureEndpoint)
{
  Rig rig;
  EXPECT_FALSE(rig.pipeline->latest_pair().has_value());
  const depthd::StereoCalibration cal = fixture_calibration(rig.config);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);
  const int64_t t = realtime_ns();
  rig.source.push_pair(depthd_test::upload_pair(raw, t, 9));  // uncalibrated: still kept for the calibration tool
  const auto pair = rig.pipeline->latest_pair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_EQ(pair->left_capture_realtime_ns, t);
  EXPECT_EQ(pair->right_capture_realtime_ns, t + 1500000);
  EXPECT_EQ(cv::norm(pair->left, raw.left, cv::NORM_INF), 0.0);
  EXPECT_EQ(cv::norm(pair->right, raw.right, cv::NORM_INF), 0.0);
  rig.clock_offset_ns = 2000000000;  // two seconds later the pair is no longer fresh
  EXPECT_FALSE(rig.pipeline->latest_pair().has_value());
}

TEST(Pipeline, ApplyConfigChangesLiveKeys)
{
  Rig rig;
  depthd::Config live = rig.config;
  live.postprocess.temporal.enabled = false;
  live.rgb.jpeg_quality = 55;
  live.mono.enabled = false;
  EXPECT_EQ(rig.pipeline->apply_config(live), "");
  EXPECT_EQ(rig.source.jpeg_quality(), 55);
  EXPECT_EQ(rig.pipeline->health().mono_state, "disabled");
  EXPECT_EQ(rig.pipeline->health().matcher, "cuda_sgm");
}

TEST(Pipeline, StopsPromptlyWithWorkInFlight)
{
  auto rig = std::make_unique<Rig>();
  const depthd::StereoCalibration cal = fixture_calibration(rig->config);
  rig->pipeline->set_calibration(cal);
  const depthd_test::RawPair raw = depthd_test::make_raw_pair(cal, kDisparity);
  rig->source.push_pair(depthd_test::upload_pair(raw, realtime_ns(), 1));
  const auto t0 = std::chrono::steady_clock::now();
  rig->pipeline->stop();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(2));
}
