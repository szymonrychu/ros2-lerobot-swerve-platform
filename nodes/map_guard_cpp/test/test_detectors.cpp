// Port of nodes/map_guard/tests/test_detectors.py (lifted / carried / rotated: sustained conditions trigger, spikes
// and commanded motion do not) plus the detectors.json golden fixture: the reasons reported at every 20 Hz step of
// every scenario equal the Python ones.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fixture_util.hpp"
#include "map_guard_cpp/config.hpp"
#include "map_guard_cpp/detectors.hpp"

namespace mg = map_guard_cpp;
namespace fx = map_guard_cpp::test;

namespace
{

constexpr double DT = 0.05;  // 20 Hz simulation step
const mg::Quaternion LEVEL = {0.0, 0.0, 0.0, 1.0};
const mg::Vector3 STILL = {0.0, 0.0, 0.0};

// Inputs of one step; unset streams are silent at that step (orientation, gyro, accel and wheel default as in Python).
struct Inputs
{
  std::optional<mg::Quaternion> orientation;
  std::optional<mg::Vector3> gyro;
  std::optional<mg::Vector3> accel;
  std::optional<double> lidar;
  std::optional<std::pair<double, double>> wheel;
  std::optional<std::pair<double, double>> cmd;
};

using InputFn = std::function<Inputs(double)>;
using Results = std::vector<std::pair<double, std::set<std::string>>>;

mg::Quaternion tilt_quaternion(double roll_deg)
{
  const double half = roll_deg * M_PI / 180.0 / 2;
  return {std::sin(half), 0.0, 0.0, std::cos(half)};
}

void step(mg::Disturbances & det, double t, const Inputs & v)
{
  det.on_imu(t, v.orientation.value_or(LEVEL), v.gyro.value_or(STILL), v.accel.value_or(STILL));
  if (v.lidar) {
    det.on_lidar_odom(t, *v.lidar);
  }
  const auto wheel = v.wheel.value_or(std::make_pair(0.0, 0.0));
  det.on_wheel_odom(t, wheel.first, wheel.second);
  if (v.cmd) {
    det.on_cmd(t, v.cmd->first, v.cmd->second);
  }
}

// Feed every input stream at 20 Hz in [start, end) and evaluate after each step.
Results run(mg::Disturbances & det, double start, double end, const InputFn & inputs)
{
  Results out;
  const int steps = static_cast<int>(std::round((end - start) / DT));
  for (int i = 0; i < steps; ++i) {
    const double t = start + i * DT;
    step(det, t, inputs(t));
    out.emplace_back(t, det.evaluate(t).reasons);
  }
  return out;
}

std::optional<double> first_fire(const Results & results, const std::string & reason)
{
  for (const auto & [t, reasons] : results) {
    if (reasons.count(reason)) {
      return t;
    }
  }
  return std::nullopt;
}

Inputs lidar(double speed)
{
  Inputs v;
  v.lidar = speed;
  return v;
}

double level_then(mg::Disturbances & det, double duration = 4.0)
{
  run(det, 0.0, duration, [](double) { return lidar(0.0); });
  return duration;
}

class Detectors : public ::testing::Test
{
protected:
  mg::Config config;
  mg::Disturbances det{config};
};

}  // namespace

TEST(DetectorHelpers, RollPitchDeg)
{
  const auto [roll, pitch] = mg::roll_pitch_deg(tilt_quaternion(20.0));
  EXPECT_NEAR(roll, 20.0, 1e-9);
  EXPECT_NEAR(pitch, 0.0, 1e-9);
}

TEST(DetectorHelpers, AngleDiffWraps)
{
  EXPECT_NEAR(mg::angle_diff_deg(179.0, -179.0), -2.0, 1e-12);
  EXPECT_NEAR(mg::angle_diff_deg(10.0, 0.0), 10.0, 1e-12);
}

TEST(DetectorHelpers, WorldVectorRotatesIntoTheWorldFrame)
{
  const auto v = mg::world_vector(tilt_quaternion(90.0), {0.0, 1.0, 0.0});
  EXPECT_NEAR(v[0], 0.0, 1e-9);
  EXPECT_NEAR(v[1], 0.0, 1e-9);
  EXPECT_NEAR(v[2], 1.0, 1e-9);
}

TEST(DetectorHelpers, OdomSpeedUsesTheLargerOfTwistAndPoseDelta)
{
  EXPECT_NEAR(mg::odom_speed(0.0, 0.0, mg::PlanarStamp{0.0, 0.0, 0.0}, {0.0, 0.05, 0.5}), 0.1, 1e-12);
  EXPECT_NEAR(mg::odom_speed(0.3, 0.0, mg::PlanarStamp{0.0, 0.0, 0.0}, {0.0, 0.05, 0.5}), 0.3, 1e-12);
  EXPECT_EQ(mg::odom_speed(0.0, 0.0, std::nullopt, {0.0, 0.05, 0.5}), 0.0);
  EXPECT_EQ(mg::odom_speed(0.0, 0.0, mg::PlanarStamp{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}), 0.0);
}

// --- LIFTED by tilt -------------------------------------------------------------------------------------------------

TEST_F(Detectors, SustainedTiltTriggersLiftedAfterLiftS)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 2.0, [](double) {
    Inputs v = lidar(0.0);
    v.orientation = tilt_quaternion(20.0);
    return v;
  });
  const auto fired = first_fire(results, mg::REASON_LIFTED);
  ASSERT_TRUE(fired);
  EXPECT_GE(*fired, start + 1.0 - 1e-6);
  EXPECT_LE(*fired, start + 1.0 + DT + 1e-6);
  EXPECT_NEAR(det.evaluate(start + 2.0).tilt_deg.value(), 20.0, 0.01);
}

TEST_F(Detectors, TiltSpikeOf02SNeverTriggers)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [start](double t) {
    Inputs v = lidar(0.0);
    v.orientation = tilt_quaternion(t < start + 0.2 ? 25.0 : 0.0);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

TEST_F(Detectors, NoTiltTriggerWithoutALevelBaseline)
{
  const auto results = run(det, 0.0, 0.5, [](double) {
    Inputs v;
    v.orientation = tilt_quaternion(30.0);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

TEST_F(Detectors, SmallTiltBelowThresholdNeverTriggers)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [](double) {
    Inputs v = lidar(0.0);
    v.orientation = tilt_quaternion(8.0);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

TEST_F(Detectors, BaselineFollowsALevelButInclinedFloor)
{
  run(det, 0.0, 4.0, [](double) {
    Inputs v = lidar(0.0);
    v.orientation = tilt_quaternion(5.0);
    return v;
  });
  const auto results = run(det, 4.0, 7.0, [](double) {
    Inputs v = lidar(0.0);
    v.orientation = tilt_quaternion(15.0);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

// --- LIFTED by vertical acceleration --------------------------------------------------------------------------------

TEST_F(Detectors, VerticalAccelerationThenTiltTriggersLifted)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 1.0, [start](double t) {
    Inputs v = lidar(0.0);
    if (t < start + 0.6) {
      v.accel = mg::Vector3{0.0, 0.0, 3.0};
    } else {
      v.orientation = tilt_quaternion(6.0);
    }
    return v;
  });
  const auto fired = first_fire(results, mg::REASON_LIFTED);
  ASSERT_TRUE(fired);
  EXPECT_LT(*fired, start + 1.0);
}

TEST_F(Detectors, VerticalAccelerationThenLidarMotionTriggersLifted)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 0.8, [start](double t) {
    if (t < start + 0.6) {
      Inputs v = lidar(0.0);
      v.accel = mg::Vector3{0.0, 0.0, -2.5};
      return v;
    }
    return lidar(0.15);
  });
  EXPECT_TRUE(first_fire(results, mg::REASON_LIFTED));
}

TEST_F(Detectors, VerticalAccelerationAloneNeverTriggers)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [](double) {
    Inputs v = lidar(0.0);
    v.accel = mg::Vector3{0.0, 0.0, 3.0};
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

TEST_F(Detectors, VerticalAccelerationSpikeThenTiltNeverTriggers)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 0.9, [start](double t) {
    Inputs v = lidar(0.0);
    if (t < start + 0.2) {
      v.accel = mg::Vector3{0.0, 0.0, 4.0};
    } else {
      v.orientation = tilt_quaternion(6.0);
    }
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

TEST_F(Detectors, GravityIncludedFallbackSamplesAreIgnored)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 1.5, [start](double t) {
    Inputs v = lidar(0.0);
    if (t < start + 1.0) {
      v.accel = mg::Vector3{0.0, 0.0, mg::GRAVITY};
    } else {
      v.orientation = tilt_quaternion(6.0);
    }
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_LIFTED));
}

// --- CARRIED / PUSHED -----------------------------------------------------------------------------------------------

TEST_F(Detectors, SustainedUnexplainedLidarMotionTriggersCarried)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 1.5, [](double) { return lidar(0.2); });
  const auto fired = first_fire(results, mg::REASON_CARRIED);
  ASSERT_TRUE(fired);
  EXPECT_GE(*fired, start + 1.0 - 1e-6);
  EXPECT_LE(*fired, start + 1.0 + DT + 1e-6);
  EXPECT_NEAR(det.evaluate(start + 1.5).unexplained_speed_mps.value(), 0.2, 1e-9);
}

TEST_F(Detectors, LidarMotionSpikeNeverTriggersCarried)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [start](double t) { return lidar(t < start + 0.2 ? 0.3 : 0.0); });
  EXPECT_FALSE(first_fire(results, mg::REASON_CARRIED));
}

TEST_F(Detectors, CommandedMotionSuppressesCarried)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [](double) {
    Inputs v = lidar(0.2);
    v.cmd = std::make_pair(0.2, 0.0);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_CARRIED));
  EXPECT_EQ(det.evaluate(start + 3.0).unexplained_speed_mps.value(), 0.0);
}

TEST_F(Detectors, CommandOlderThanCmdTimeoutDoesNotSuppress)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 2.0, [start](double t) {
    Inputs v = lidar(0.2);
    if (t < start + 0.1) {
      v.cmd = std::make_pair(0.2, 0.0);
    }
    return v;
  });
  EXPECT_TRUE(first_fire(results, mg::REASON_CARRIED));
}

TEST_F(Detectors, ZeroCommandDoesNotSuppress)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 2.0, [](double) {
    Inputs v = lidar(0.2);
    v.cmd = std::make_pair(0.0, 0.0);
    return v;
  });
  EXPECT_TRUE(first_fire(results, mg::REASON_CARRIED));
}

TEST_F(Detectors, MovingWheelsSuppressCarried)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [](double) {
    Inputs v = lidar(0.2);
    v.wheel = std::make_pair(0.18, 0.0);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_CARRIED));
}

TEST_F(Detectors, StaleLidarOdometryNeverTriggersCarried)
{
  const double start = level_then(det);
  det.on_lidar_odom(start, 0.3);
  const auto results = run(det, start, start + 3.0, [](double) { return Inputs{}; });
  EXPECT_FALSE(first_fire(results, mg::REASON_CARRIED));
  EXPECT_FALSE(det.evaluate(start + 3.0).unexplained_speed_mps);
}

// --- ROTATED --------------------------------------------------------------------------------------------------------

TEST_F(Detectors, SustainedUnexplainedYawRateTriggersRotated)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 1.5, [](double) {
    Inputs v = lidar(0.0);
    v.gyro = mg::Vector3{0.0, 0.0, 0.5};
    return v;
  });
  const auto fired = first_fire(results, mg::REASON_ROTATED);
  ASSERT_TRUE(fired);
  EXPECT_GE(*fired, start + 0.8 - 1e-6);
  EXPECT_LE(*fired, start + 0.8 + DT + 1e-6);
}

TEST_F(Detectors, YawRateSpikeNeverTriggersRotated)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [start](double t) {
    Inputs v;
    v.gyro = mg::Vector3{0.0, 0.0, t < start + 0.2 ? 1.0 : 0.0};
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_ROTATED));
}

TEST_F(Detectors, CommandedRotationSuppressesRotated)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [](double) {
    Inputs v;
    v.gyro = mg::Vector3{0.0, 0.0, 0.5};
    v.cmd = std::make_pair(0.0, 0.5);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_ROTATED));
}

TEST_F(Detectors, WheelYawRateSuppressesRotated)
{
  const double start = level_then(det);
  const auto results = run(det, start, start + 3.0, [](double) {
    Inputs v;
    v.gyro = mg::Vector3{0.0, 0.0, 0.5};
    v.wheel = std::make_pair(0.0, 0.45);
    return v;
  });
  EXPECT_FALSE(first_fire(results, mg::REASON_ROTATED));
}

TEST_F(Detectors, QuietRobotReportsNothing)
{
  const auto results = run(det, 0.0, 6.0, [](double) { return lidar(0.01); });
  for (const auto & [t, reasons] : results) {
    EXPECT_TRUE(reasons.empty()) << "t=" << t;
  }
}

// --- Golden fixture (Python outputs) --------------------------------------------------------------------------------

namespace
{

mg::Quaternion quat(const nlohmann::json & q)
{
  return {q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()};
}

mg::Vector3 vec(const nlohmann::json & v) { return {v[0].get<double>(), v[1].get<double>(), v[2].get<double>()}; }

Inputs segment_inputs(const nlohmann::json & segments, double t)
{
  for (const auto & segment : segments) {
    if (!segment["until"].is_null() && !(t < segment["until"].get<double>())) {
      continue;
    }
    const auto & in = segment["inputs"];
    Inputs v;
    if (in.contains("orientation")) {
      v.orientation = quat(in["orientation"]);
    }
    if (in.contains("gyro")) {
      v.gyro = vec(in["gyro"]);
    }
    if (in.contains("accel")) {
      v.accel = vec(in["accel"]);
    }
    if (in.contains("lidar")) {
      v.lidar = in["lidar"].get<double>();
    }
    if (in.contains("wheel")) {
      v.wheel = std::make_pair(in["wheel"][0].get<double>(), in["wheel"][1].get<double>());
    }
    if (in.contains("cmd")) {
      v.cmd = std::make_pair(in["cmd"][0].get<double>(), in["cmd"][1].get<double>());
    }
    return v;
  }
  return Inputs{};
}

std::string joined(const std::set<std::string> & reasons)
{
  std::string out;
  for (const auto & r : reasons) {
    out += (out.empty() ? "" : ",") + r;
  }
  return out;
}

void expect_optional_near(
  const nlohmann::json & expected, const std::optional<double> & actual, const std::string & what)
{
  if (expected.is_null()) {
    EXPECT_FALSE(actual) << what;
  } else {
    ASSERT_TRUE(actual) << what;
    EXPECT_NEAR(*actual, expected.get<double>(), 1e-9) << what;
  }
}

}  // namespace

TEST(DetectorsFixture, HelpersMatchPython)
{
  const auto f = fx::load_fixture("detectors.json");
  for (const auto & c : f["roll_pitch"]) {
    const auto [roll, pitch] = mg::roll_pitch_deg(quat(c["q"]));
    EXPECT_NEAR(roll, c["out"][0].get<double>(), 1e-12);
    EXPECT_NEAR(pitch, c["out"][1].get<double>(), 1e-12);
  }
  for (const auto & c : f["angle_diff"]) {
    EXPECT_DOUBLE_EQ(mg::angle_diff_deg(c["a"].get<double>(), c["b"].get<double>()), c["out"].get<double>());
  }
  for (const auto & c : f["world_vector"]) {
    const auto out = mg::world_vector(quat(c["q"]), vec(c["v"]));
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(out[i], c["out"][i].get<double>(), 1e-12);
    }
  }
  for (const auto & c : f["odom_speed"]) {
    std::optional<mg::PlanarStamp> previous;
    if (!c["previous"].is_null()) {
      previous = mg::PlanarStamp{c["previous"][0], c["previous"][1], c["previous"][2]};
    }
    const mg::PlanarStamp current{c["current"][0], c["current"][1], c["current"][2]};
    EXPECT_DOUBLE_EQ(
      mg::odom_speed(c["twist"][0].get<double>(), c["twist"][1].get<double>(), previous, current),
      c["out"].get<double>());
  }
}

TEST(DetectorsFixture, ReasonsAtEveryStepMatchPython)
{
  const auto f = fx::load_fixture("detectors.json");
  const double dt = f["dt"].get<double>();
  ASSERT_EQ(dt, DT);
  for (const auto & s : f["scenarios"]) {
    const std::string name = s["name"];
    mg::Config config;
    mg::Disturbances det(config);
    std::size_t index = 0;
    const auto & runs = s["runs"];
    for (std::size_t r = 0; r < runs.size(); ++r) {
      const double start = runs[r]["start"].get<double>();
      const double end = runs[r]["end"].get<double>();
      if (!s["lidar_at_start"].is_null() && r == s["lidar_at_run"].get<std::size_t>()) {
        det.on_lidar_odom(start, s["lidar_at_start"].get<double>());
      }
      const int steps = static_cast<int>(std::round((end - start) / dt));
      for (int i = 0; i < steps; ++i, ++index) {
        const double t = start + i * dt;
        step(det, t, segment_inputs(runs[r]["segments"], t));
        ASSERT_LT(index, s["steps"].size()) << name;
        ASSERT_DOUBLE_EQ(t, s["steps"][index][0].get<double>()) << name;
        EXPECT_EQ(joined(det.evaluate(t).reasons), s["steps"][index][1].get<std::string>()) << name << " at t=" << t;
      }
    }
    EXPECT_EQ(index, s["steps"].size()) << name;
    const auto final_report = det.evaluate(s["final_t"].get<double>());
    std::vector<std::string> reasons(final_report.reasons.begin(), final_report.reasons.end());
    EXPECT_EQ(reasons, s["final"]["reasons"].get<std::vector<std::string>>()) << name;
    expect_optional_near(s["final"]["tilt_deg"], final_report.tilt_deg, name + " tilt");
    expect_optional_near(s["final"]["unexplained_speed_mps"], final_report.unexplained_speed_mps, name + " speed");
    expect_optional_near(s["final"]["imu_yaw_rate"], final_report.imu_yaw_rate, name + " yaw rate");
    const auto & baseline = det.tilt_tracker().baseline();
    if (s["baseline"].is_null()) {
      EXPECT_FALSE(baseline) << name;
    } else {
      ASSERT_TRUE(baseline) << name;
      EXPECT_NEAR(baseline->first, s["baseline"][0].get<double>(), 1e-9) << name;
      EXPECT_NEAR(baseline->second, s["baseline"][1].get<double>(), 1e-9) << name;
    }
  }
}
