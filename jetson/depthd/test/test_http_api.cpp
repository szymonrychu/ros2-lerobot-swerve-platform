// HTTP API: every endpoint and status code, against a real server on loopback with a free port.
#include <gtest/gtest.h>

#include <atomic>
#include <fstream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "../third_party/httplib.h"
#include "depthd/config.hpp"
#include "depthd/http_api.hpp"

namespace
{
using nlohmann::json;

const std::string kFixtures = std::string(DEPTHD_SOURCE_DIR) + "/test/fixtures/";

depthd::Config test_config(const std::string & calibration = "stereo_valid.yaml")
{
  depthd::Config c;
  c.transport.bind_address = "127.0.0.1";
  c.calibration.path = kFixtures + calibration;
  return c;
}

struct Fixture
{
  depthd::Metrics metrics;
  depthd::HttpApiHooks hooks;
  std::unique_ptr<depthd::HttpApi> api;
  std::unique_ptr<httplib::Client> client;
  int applied = 0;
  depthd::Config last_applied;
  int calibrations_loaded = 0;

  explicit Fixture(depthd::Config config = test_config(), bool set_hooks = true)
  {
    if (set_hooks) {
      hooks.apply_config = [this](const depthd::Config & c) -> std::string {
        ++applied;
        last_applied = c;
        return reject_apply.empty() ? std::string() : reject_apply;
      };
      hooks.on_calibration = [this](const depthd::StereoCalibration &) { ++calibrations_loaded; };
    }
    api = std::make_unique<depthd::HttpApi>(config, metrics, hooks);
    EXPECT_TRUE(api->start(0)) << "http server failed to start";
    client = std::make_unique<httplib::Client>("127.0.0.1", api->bound_port());
    client->set_read_timeout(5);
  }
  ~Fixture() { api->stop(); }
  std::string reject_apply;
};

json body_of(const httplib::Result & r)
{
  EXPECT_TRUE(r);
  return json::parse(r->body);
}

httplib::Result put_config(Fixture & f, const json & body)
{
  return f.client->Put("/config", body.dump(), "application/json");
}
}  // namespace

// ---- /health ----
TEST(HttpApiHealth, StartingBeforeAnyHookIsSet)
{
  Fixture f(test_config(), false);
  const auto r = f.client->Get("/health");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  const json j = json::parse(r->body);
  EXPECT_EQ(j.at("state"), "starting");
  for (const char * key : {"state", "reason", "fps", "last_pair_age_ms", "pairs_dropped_total", "gpu_temp_c",
                           "nvgpu_errors_total", "matcher", "mono_state"}) {
    EXPECT_TRUE(j.contains(key)) << key;
  }
}

TEST(HttpApiHealth, ReportsEveryFieldFromTheHook)
{
  Fixture f;
  f.hooks.health = [] {
    depthd::Health h;
    h.state = "camera_error";
    h.reason = "right camera produced no frames";
    h.fps = 11.5;
    h.last_pair_age_ms = 812.0;
    h.pairs_dropped_total = 17;
    h.gpu_temp_c = 54.5;
    h.nvgpu_errors_total = 2;
    h.matcher = "cuda_sgm";
    h.mono_state = "running";
    return h;
  };
  f.api->stop();
  f.api = std::make_unique<depthd::HttpApi>(test_config(), f.metrics, f.hooks);
  ASSERT_TRUE(f.api->start(0));
  httplib::Client c("127.0.0.1", f.api->bound_port());
  const auto r = c.Get("/health");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  const json j = json::parse(r->body);
  EXPECT_EQ(j.at("state"), "camera_error");
  EXPECT_EQ(j.at("reason"), "right camera produced no frames");
  EXPECT_DOUBLE_EQ(j.at("fps"), 11.5);
  EXPECT_DOUBLE_EQ(j.at("last_pair_age_ms"), 812.0);
  EXPECT_EQ(j.at("pairs_dropped_total"), 17);
  EXPECT_DOUBLE_EQ(j.at("gpu_temp_c"), 54.5);
  EXPECT_EQ(j.at("nvgpu_errors_total"), 2);
  EXPECT_EQ(j.at("matcher"), "cuda_sgm");
  EXPECT_EQ(j.at("mono_state"), "running");
}

// ---- /config ----
TEST(HttpApiConfig, GetReturnsTheRunningConfig)
{
  Fixture f;
  const auto r = f.client->Get("/config");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  EXPECT_EQ(json::parse(r->body), depthd::to_json(test_config()));
}

TEST(HttpApiConfig, PutMutableKeyAppliesAndIsVisible)
{
  Fixture f;
  const auto r = put_config(f, {{"postprocess", {{"temporal", {{"alpha", 0.7}}}}}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200) << r->body;
  EXPECT_EQ(f.applied, 1);
  EXPECT_DOUBLE_EQ(f.last_applied.postprocess.temporal.alpha, 0.7);
  EXPECT_DOUBLE_EQ(body_of(r).at("postprocess").at("temporal").at("alpha"), 0.7);
  EXPECT_DOUBLE_EQ(body_of(f.client->Get("/config")).at("postprocess").at("temporal").at("alpha"), 0.7);
  EXPECT_DOUBLE_EQ(f.api->config().postprocess.temporal.alpha, 0.7);
}

TEST(HttpApiConfig, PutAcceptsDottedKeysAndSeveralMutableKeys)
{
  Fixture f;
  const auto r = put_config(f, {{"rgb.jpeg_quality", 60}, {"mono.rate_hz", 1.0}, {"postprocess.wls.enabled", true}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200) << r->body;
  EXPECT_EQ(f.api->config().rgb.jpeg_quality, 60);
  EXPECT_DOUBLE_EQ(f.api->config().mono.rate_hz, 1.0);
  EXPECT_TRUE(f.api->config().postprocess.wls.enabled);
}

TEST(HttpApiConfig, PutImmutableKeyIs409NamingTheKeyAndChangesNothing)
{
  Fixture f;
  const auto before = body_of(f.client->Get("/config"));
  const auto r = put_config(f, {{"processing", {{"width", 320}, {"height", 240}}}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 409);
  EXPECT_NE(r->body.find("processing.width"), std::string::npos) << r->body;
  EXPECT_NE(r->body.find("processing.height"), std::string::npos) << r->body;
  EXPECT_EQ(f.applied, 0);
  EXPECT_EQ(body_of(f.client->Get("/config")), before);
}

TEST(HttpApiConfig, MixedMutableAndImmutableChangesNothing)
{
  Fixture f;
  const auto before = body_of(f.client->Get("/config"));
  const auto r = put_config(f, {{"postprocess.temporal.alpha", 0.9}, {"transport.pub_port", 5551}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 409);
  EXPECT_NE(r->body.find("transport.pub_port"), std::string::npos) << r->body;
  EXPECT_EQ(f.applied, 0);
  EXPECT_EQ(body_of(f.client->Get("/config")), before);
}

TEST(HttpApiConfig, ImmutableKeyWithItsCurrentValueIsNotAChange)
{
  Fixture f;
  const auto r = put_config(f, {{"processing.width", 640}, {"postprocess.temporal.alpha", 0.5}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200) << r->body;
  EXPECT_DOUBLE_EQ(f.api->config().postprocess.temporal.alpha, 0.5);
}

TEST(HttpApiConfig, PutInvalidValueIs400WithTheValidationMessageAndChangesNothing)
{
  Fixture f;
  const auto before = body_of(f.client->Get("/config"));
  const auto r = put_config(f, {{"postprocess.temporal.alpha", 5.0}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 400);
  EXPECT_NE(r->body.find("postprocess.temporal.alpha"), std::string::npos) << r->body;
  EXPECT_EQ(f.applied, 0);
  EXPECT_EQ(body_of(f.client->Get("/config")), before);
}

TEST(HttpApiConfig, PutUnknownKeyWrongTypeAndBadJsonAre400)
{
  Fixture f;
  const auto before = body_of(f.client->Get("/config"));
  const auto unknown = put_config(f, {{"postprocess.no_such_key", 1}});
  ASSERT_TRUE(unknown);
  EXPECT_EQ(unknown->status, 400);
  const auto wrong_type = put_config(f, {{"rgb.jpeg_quality", "high"}});
  ASSERT_TRUE(wrong_type);
  EXPECT_EQ(wrong_type->status, 400);
  const auto bad_json = f.client->Put("/config", "{not json", "application/json");
  ASSERT_TRUE(bad_json);
  EXPECT_EQ(bad_json->status, 400);
  const auto not_object = f.client->Put("/config", "[1,2]", "application/json");
  ASSERT_TRUE(not_object);
  EXPECT_EQ(not_object->status, 400);
  EXPECT_EQ(f.applied, 0);
  EXPECT_EQ(body_of(f.client->Get("/config")), before);
}

TEST(HttpApiConfig, ApplyHookRejectionIs400AndChangesNothing)
{
  Fixture f;
  f.reject_apply = "mono model is loading";
  const auto before = body_of(f.client->Get("/config"));
  const auto r = put_config(f, {{"mono.enabled", false}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 400);
  EXPECT_NE(r->body.find("mono model is loading"), std::string::npos);
  EXPECT_EQ(body_of(f.client->Get("/config")), before);
}

// ---- /calibration ----
TEST(HttpApiCalibration, ValidFileReportsOkAndMetadata)
{
  Fixture f;
  const auto r = f.client->Get("/calibration");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  const json j = json::parse(r->body);
  EXPECT_EQ(j.at("status"), "ok");
  EXPECT_EQ(j.at("reason"), "");
  EXPECT_DOUBLE_EQ(j.at("rms"), 0.21);
  EXPECT_DOUBLE_EQ(j.at("baseline_m"), 0.06);
  ASSERT_TRUE(j.at("created_utc").is_string());
  EXPECT_FALSE(j.at("created_utc").get<std::string>().empty());
}

TEST(HttpApiCalibration, MissingFileIsUncalibratedWithReason)
{
  Fixture f(test_config("no_such_file.yaml"));
  const json j = body_of(f.client->Get("/calibration"));
  EXPECT_EQ(j.at("status"), "uncalibrated");
  EXPECT_FALSE(j.at("reason").get<std::string>().empty());
}

TEST(HttpApiCalibration, InvalidFileIsUncalibratedNamingTheReason)
{
  Fixture f(test_config("stereo_invalid_rms.yaml"));
  const json j = body_of(f.client->Get("/calibration"));
  EXPECT_EQ(j.at("status"), "uncalibrated");
  EXPECT_NE(j.at("reason").get<std::string>().find("rms"), std::string::npos) << j.dump();
}

TEST(HttpApiCalibration, ReloadPicksUpAFileThatAppearedAndNotifiesTheHook)
{
  const std::string path = ::testing::TempDir() + "depthd_http_reload.yaml";
  std::remove(path.c_str());
  depthd::Config c = test_config();
  c.calibration.path = path;
  Fixture f(c);
  EXPECT_EQ(body_of(f.client->Get("/calibration")).at("status"), "uncalibrated");

  std::ifstream src(kFixtures + "stereo_valid.yaml", std::ios::binary);
  std::ofstream dst(path, std::ios::binary);
  dst << src.rdbuf();
  dst.close();

  const auto r = f.client->Post("/calibration/reload", "", "application/json");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200) << r->body;
  EXPECT_EQ(json::parse(r->body).at("status"), "ok");
  EXPECT_EQ(f.calibrations_loaded, 1);
  EXPECT_EQ(body_of(f.client->Get("/calibration")).at("status"), "ok");
  std::remove(path.c_str());
}

TEST(HttpApiCalibration, ReloadOfInvalidFileIs422WithReasonAndKeepsTheActiveCalibration)
{
  const std::string path = ::testing::TempDir() + "depthd_http_reload_bad.yaml";
  {
    std::ifstream src(kFixtures + "stereo_valid.yaml", std::ios::binary);
    std::ofstream dst(path, std::ios::binary);
    dst << src.rdbuf();
  }
  depthd::Config c = test_config();
  c.calibration.path = path;
  Fixture f(c);
  ASSERT_EQ(body_of(f.client->Get("/calibration")).at("status"), "ok");
  {
    std::ifstream src(kFixtures + "stereo_invalid_rms.yaml", std::ios::binary);
    std::ofstream dst(path, std::ios::binary | std::ios::trunc);
    dst << src.rdbuf();
  }
  const auto r = f.client->Post("/calibration/reload", "", "application/json");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 422);
  EXPECT_NE(json::parse(r->body).at("reason").get<std::string>().find("rms"), std::string::npos) << r->body;
  EXPECT_EQ(f.calibrations_loaded, 1);  // only the startup load
  EXPECT_EQ(body_of(f.client->Get("/calibration")).at("status"), "ok");
  std::remove(path.c_str());
}

TEST(HttpApiCalibration, ReloadOfMissingFileIs422)
{
  Fixture f(test_config("no_such_file.yaml"));
  const auto r = f.client->Post("/calibration/reload", "", "application/json");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 422);
  EXPECT_FALSE(json::parse(r->body).at("reason").get<std::string>().empty());
}

// ---- /calibration/capture ----
namespace
{
struct Part
{
  std::string name;
  std::string content_type;
  std::string data;
};

std::vector<Part> parse_multipart(const httplib::Response & res)
{
  const std::string ct = res.get_header_value("Content-Type");
  EXPECT_EQ(ct.rfind("multipart/form-data", 0), 0U) << ct;
  const auto pos = ct.find("boundary=");
  EXPECT_NE(pos, std::string::npos);
  const std::string delim = "--" + ct.substr(pos + 9);
  std::vector<Part> parts;
  size_t at = res.body.find(delim);
  while (at != std::string::npos) {
    at += delim.size();
    if (res.body.compare(at, 2, "--") == 0) {
      break;
    }
    at += 2;  // CRLF
    const size_t head_end = res.body.find("\r\n\r\n", at);
    const std::string head = res.body.substr(at, head_end - at);
    const size_t data_start = head_end + 4;
    const size_t next = res.body.find("\r\n" + delim, data_start);
    Part p;
    p.data = res.body.substr(data_start, next - data_start);
    const auto n = head.find("name=\"");
    p.name = head.substr(n + 6, head.find('"', n + 6) - n - 6);
    const auto t = head.find("Content-Type: ");
    if (t != std::string::npos) {
      p.content_type = head.substr(t + 14, head.find("\r\n", t) - t - 14);
    }
    parts.push_back(p);
    at = next + 2;
  }
  return parts;
}
}  // namespace

TEST(HttpApiCapture, Is503WithoutAFreshPair)
{
  Fixture f;
  f.hooks.capture_pair = [] { return std::optional<depthd::CapturedPair>(); };
  f.api->stop();
  f.api = std::make_unique<depthd::HttpApi>(test_config(), f.metrics, f.hooks);
  ASSERT_TRUE(f.api->start(0));
  httplib::Client c("127.0.0.1", f.api->bound_port());
  const auto r = c.Get("/calibration/capture");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 503);
  EXPECT_TRUE(json::parse(r->body).contains("reason"));
}

TEST(HttpApiCapture, Is503WhenNoCaptureSourceIsWired)
{
  Fixture f(test_config(), false);
  const auto r = f.client->Get("/calibration/capture");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 503);
}

TEST(HttpApiCapture, ReturnsMultipartWithTwoGreyPngsAndTheirTimestamps)
{
  Fixture f;
  cv::Mat left(48, 64, CV_8UC1, cv::Scalar(40));
  cv::Mat right(48, 64, CV_8UC1, cv::Scalar(200));
  f.hooks.capture_pair = [&] {
    depthd::CapturedPair p;
    p.left = left;
    p.right = right;
    p.left_capture_realtime_ns = 1700000000111111111LL;
    p.right_capture_realtime_ns = 1700000000122222222LL;
    return std::optional<depthd::CapturedPair>(p);
  };
  f.api->stop();
  f.api = std::make_unique<depthd::HttpApi>(test_config(), f.metrics, f.hooks);
  ASSERT_TRUE(f.api->start(0));
  httplib::Client c("127.0.0.1", f.api->bound_port());
  const auto r = c.Get("/calibration/capture");
  ASSERT_TRUE(r);
  ASSERT_EQ(r->status, 200);
  const auto parts = parse_multipart(*r);
  std::map<std::string, Part> by_name;
  for (const auto & p : parts) {
    by_name[p.name] = p;
  }
  ASSERT_TRUE(by_name.count("left"));
  ASSERT_TRUE(by_name.count("right"));
  EXPECT_EQ(by_name["left"].content_type, "image/png");
  EXPECT_EQ(by_name["right"].content_type, "image/png");
  const auto decode = [](const Part & p) {
    return cv::imdecode(std::vector<uint8_t>(p.data.begin(), p.data.end()), cv::IMREAD_UNCHANGED);
  };
  const cv::Mat l = decode(by_name["left"]);
  const cv::Mat rr = decode(by_name["right"]);
  ASSERT_EQ(l.type(), CV_8UC1);
  ASSERT_EQ(rr.type(), CV_8UC1);
  EXPECT_EQ(l.size(), cv::Size(64, 48));
  EXPECT_EQ(l.at<uint8_t>(10, 10), 40);
  EXPECT_EQ(rr.at<uint8_t>(10, 10), 200);
  ASSERT_TRUE(by_name.count("left_capture_realtime_ns"));
  ASSERT_TRUE(by_name.count("right_capture_realtime_ns"));
  EXPECT_EQ(by_name["left_capture_realtime_ns"].data, "1700000000111111111");
  EXPECT_EQ(by_name["right_capture_realtime_ns"].data, "1700000000122222222");
}

// ---- /metrics ----
TEST(HttpApiMetrics, ServesPrometheusTextWithEveryDepthdMetric)
{
  Fixture f;
  f.metrics.inc_frames("depth");
  f.metrics.inc_frames("depth");
  f.metrics.inc_frames("rgb");
  f.metrics.inc_drop("encode");
  depthd::PairingStats ps;
  ps.pairs_emitted = 5;
  ps.dropped_skew = 3;
  ps.dropped_orphan = 4;
  ps.skew.record(0.5);
  ps.skew.record(3.0);
  ps.skew.record(40.0);
  ps.realignments_skew = 2;
  ps.realignments_stall = 1;
  ps.exposure.left = {true, 12000, 3.3125};
  ps.exposure.right = {true, 12001, 3.25};
  ps.exposure.saturation_valid = true;
  ps.exposure.saturated_fraction = 0.0125;
  ps.exposure.compensation = -0.5;
  ps.exposure.ae_adjustments = 3;
  ps.exposure.follow_pins = 7;
  ps.exposure.mismatch_drops = 11;
  f.metrics.set_pairing(ps);
  f.metrics.observe_stage_ms("match", 7.5);
  f.metrics.observe_stage_ms("match", 12.0);
  f.metrics.set_fps(12.25);
  f.metrics.set_gpu_temp_c(51.5);
  f.metrics.set_nvgpu_errors(3);
  f.metrics.set_zmq_send_drops(9);

  const auto r = f.client->Get("/metrics");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  EXPECT_NE(r->get_header_value("Content-Type").find("text/plain"), std::string::npos);
  const std::string & t = r->body;
  const auto has = [&](const std::string & s) { EXPECT_NE(t.find(s), std::string::npos) << "missing: " << s << "\n" << t; };
  has("# TYPE depthd_frames_total counter");
  has("depthd_frames_total{kind=\"depth\"} 2");
  has("depthd_frames_total{kind=\"rgb\"} 1");
  has("# TYPE depthd_drops_total counter");
  has("depthd_drops_total{reason=\"skew\"} 3");
  has("depthd_drops_total{reason=\"orphan\"} 4");
  has("depthd_drops_total{reason=\"encode\"} 1");
  has("# TYPE depthd_pair_skew_ms histogram");
  has("depthd_pair_skew_ms_bucket{le=\"1\"} 1");
  has("depthd_pair_skew_ms_bucket{le=\"4\"} 2");
  has("depthd_pair_skew_ms_bucket{le=\"+Inf\"} 3");
  has("depthd_pair_skew_ms_count 3");
  has("# TYPE depthd_camera_realignments_total counter");
  has("depthd_camera_realignments_total{reason=\"skew\"} 2");
  has("depthd_camera_realignments_total{reason=\"stall\"} 1");
  has("# TYPE depthd_camera_exposure_us gauge");
  has("depthd_camera_exposure_us{camera=\"left\"} 12000");
  has("depthd_camera_exposure_us{camera=\"right\"} 12001");
  has("# TYPE depthd_camera_gain gauge");
  has("depthd_camera_gain{camera=\"left\"} 3.3125");
  has("depthd_camera_gain{camera=\"right\"} 3.25");
  has("# TYPE depthd_camera_saturated_fraction gauge");
  has("depthd_camera_saturated_fraction 0.0125");
  has("# TYPE depthd_camera_exposure_compensation gauge");
  has("depthd_camera_exposure_compensation -0.5");
  has("# TYPE depthd_ae_adjustments_total counter");
  has("depthd_ae_adjustments_total 3");
  has("# TYPE depthd_exposure_follow_pins_total counter");
  has("depthd_exposure_follow_pins_total 7");
  has("depthd_drops_total{reason=\"exposure_mismatch\"} 11");
  has("# TYPE depthd_stage_time_ms histogram");
  has("depthd_stage_time_ms_bucket{stage=\"match\",le=\"10\"} 1");
  has("depthd_stage_time_ms_bucket{stage=\"match\",le=\"+Inf\"} 2");
  has("depthd_stage_time_ms_sum{stage=\"match\"} 19.5");
  has("depthd_stage_time_ms_count{stage=\"match\"} 2");
  has("# TYPE depthd_fps gauge");
  has("depthd_fps 12.25");
  has("depthd_gpu_temp_celsius 51.5");
  has("depthd_nvgpu_errors_total 3");
  has("depthd_zmq_send_drops_total 9");
}

TEST(HttpApiMetrics, EmptyMetricsStillRender)
{
  Fixture f;
  const auto r = f.client->Get("/metrics");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  EXPECT_NE(r->body.find("depthd_fps 0"), std::string::npos);
  EXPECT_NE(r->body.find("depthd_zmq_send_drops_total 0"), std::string::npos);
  EXPECT_NE(r->body.find("depthd_ae_adjustments_total 0"), std::string::npos);
  // No readback yet: no exposure, gain or saturation sample rather than a made-up 0.
  EXPECT_EQ(r->body.find("depthd_camera_exposure_us{"), std::string::npos);
  EXPECT_EQ(r->body.find("\ndepthd_camera_saturated_fraction "), std::string::npos);
}

TEST(HttpApi, UnknownPathIs404)
{
  Fixture f;
  const auto r = f.client->Get("/nope");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 404);
}
