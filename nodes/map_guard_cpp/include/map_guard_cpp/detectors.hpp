// Disturbance detectors: the robot is LIFTED, CARRIED / PUSHED or ROTATED by hand.
// Port of nodes/map_guard/map_guard/detectors.py; the module docstring there describes every condition.
#ifndef MAP_GUARD_CPP__DETECTORS_HPP_
#define MAP_GUARD_CPP__DETECTORS_HPP_

#include <deque>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>

#include "map_guard_cpp/agreement.hpp"
#include "map_guard_cpp/config.hpp"
#include "map_guard_cpp/timing.hpp"

namespace map_guard_cpp
{

constexpr const char * REASON_MISMATCH = "mismatch";
constexpr const char * REASON_LIFTED = "lifted";
constexpr const char * REASON_CARRIED = "carried";
constexpr const char * REASON_ROTATED = "rotated";
constexpr double GRAVITY = 9.80665;

// (x, y, time_s) of an odometry pose.
struct PlanarStamp
{
  double x;
  double y;
  double t;
};

// What the detectors see at one instant.
struct DisturbanceReport
{
  std::set<std::string> reasons;   // detectors currently firing (REASON_* values)
  std::optional<double> tilt_deg;  // largest roll/pitch difference from the baseline; none without a baseline
  // Lidar minus wheel odometry speed when no drive command is active (0 when one is); none when odometry is stale.
  std::optional<double> unexplained_speed_mps;
  std::optional<double> imu_yaw_rate;  // world-frame IMU yaw rate; none when the IMU is stale
};

// (roll, pitch) of an orientation, deg.
std::pair<double, double> roll_pitch_deg(const Quaternion & q);

// a - b wrapped to [-180, 180), deg (Python's floored modulo).
double angle_diff_deg(double a, double b);

// Rotate a sensor-frame vector into the world frame.
Vector3 world_vector(const Quaternion & q, const Vector3 & v);

// Planar speed of an odometry message: the larger of its twist and its pose difference over time.
double odom_speed(
  double twist_x, double twist_y, const std::optional<PlanarStamp> & previous, const PlanarStamp & current);

// Level baseline and tilt from the baseline.
class TiltTracker
{
public:
  explicit TiltTracker(const Config & config);

  // Largest roll/pitch difference from the baseline, deg; none without a baseline.
  std::optional<double> tilt(double roll, double pitch) const;

  // Add an attitude sample and adopt a new baseline when the recent window is a level period.
  void add(double now, double roll, double pitch);

  const std::optional<std::pair<double, double>> & baseline() const { return baseline_; }

private:
  const Config & config_;
  std::deque<std::tuple<double, double, double>> window_;
  bool window_tilted_ = false;
  std::optional<std::pair<double, double>> baseline_;
};

// Feeds the IMU, odometry and command streams to the LIFTED / CARRIED / ROTATED detectors.
class Disturbances
{
public:
  // config must outlive the detectors.
  explicit Disturbances(const Config & config);

  void on_imu(
    double now, const Quaternion & orientation, const Vector3 & angular_velocity, const Vector3 & linear_acceleration);
  void on_lidar_odom(double now, double speed_mps);
  void on_wheel_odom(double now, double speed_mps, double yaw_rate);
  void on_cmd(double now, double linear_mps, double angular_rad_s);

  // Evaluate every detector at now.
  DisturbanceReport evaluate(double now);

  const TiltTracker & tilt_tracker() const { return tilt_tracker_; }

private:
  bool fresh(const std::optional<double> & sample_time, double now) const;

  const Config & config_;
  TiltTracker tilt_tracker_;
  HoldTimer tilt_hold_;
  HoldTimer accel_hold_;
  HoldTimer carried_hold_;
  HoldTimer rotated_hold_;
  std::optional<double> accel_evidence_until_;
  std::optional<double> tilt_deg_;
  std::optional<std::pair<double, double>> imu_;             // (time, world yaw rate)
  std::optional<std::pair<double, double>> lidar_;           // (time, speed)
  std::optional<std::tuple<double, double, double>> wheel_;  // (time, speed, yaw rate)
  std::optional<std::tuple<double, double, double>> cmd_;    // (time, |linear|, |angular|)
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__DETECTORS_HPP_
