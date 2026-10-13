#include "map_guard_cpp/detectors.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace map_guard_cpp
{

namespace
{

// Python's math.degrees.
double degrees(double radians) { return radians * (180.0 / M_PI); }

}  // namespace

std::pair<double, double> roll_pitch_deg(const Quaternion & q)
{
  const auto [x, y, z, w] = q;
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (w * y - z * x))));
  return {degrees(roll), degrees(pitch)};
}

double angle_diff_deg(double a, double b)
{
  // (a - b + 180) % 360 - 180 with Python's floored modulo (the result takes the sign of the divisor).
  const double x = a - b + 180.0;
  double mod = std::fmod(x, 360.0);
  if (mod != 0.0) {
    if (mod < 0.0) {
      mod += 360.0;
    }
  } else {
    mod = std::copysign(0.0, 360.0);
  }
  return mod - 180.0;
}

Vector3 world_vector(const Quaternion & q, const Vector3 & v)
{
  const Matrix3 m = quaternion_matrix(q);
  return {
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  };
}

double odom_speed(
  double twist_x, double twist_y, const std::optional<PlanarStamp> & previous, const PlanarStamp & current)
{
  double speed = std::hypot(twist_x, twist_y);
  if (previous) {
    const double dt = current.t - previous->t;
    if (dt > 0.0) {
      speed = std::max(speed, std::hypot(current.x - previous->x, current.y - previous->y) / dt);
    }
  }
  return speed;
}

TiltTracker::TiltTracker(const Config & config) : config_(config) {}

std::optional<double> TiltTracker::tilt(double roll, double pitch) const
{
  if (!baseline_) {
    return std::nullopt;
  }
  return std::max(std::abs(angle_diff_deg(roll, baseline_->first)), std::abs(angle_diff_deg(pitch, baseline_->second)));
}

void TiltTracker::add(double now, double roll, double pitch)
{
  const bool tilted = tilt(roll, pitch).value_or(0.0) > config_.lift_tilt_deg;
  const double span_s = tilted ? config_.level_rebaseline_s : config_.level_window_s;
  if (tilted != window_tilted_) {
    window_.clear();
    window_tilted_ = tilted;
  }
  window_.emplace_back(now, roll, pitch);
  while (!window_.empty() && now - std::get<0>(window_.front()) > span_s) {
    window_.pop_front();
  }
  if (now - std::get<0>(window_.front()) < span_s * 0.9) {
    return;
  }
  std::vector<double> rolls;
  std::vector<double> pitches;
  rolls.reserve(window_.size());
  pitches.reserve(window_.size());
  for (const auto & sample : window_) {
    rolls.push_back(std::get<1>(sample));
    pitches.push_back(std::get<2>(sample));
  }
  const double median_roll = median(rolls);
  const double median_pitch = median(pitches);
  double spread = 0.0;
  for (double r : rolls) {
    spread = std::max(spread, std::abs(angle_diff_deg(r, median_roll)));
  }
  for (double p : pitches) {
    spread = std::max(spread, std::abs(angle_diff_deg(p, median_pitch)));
  }
  if (spread <= config_.level_tolerance_deg) {
    baseline_ = std::make_pair(median_roll, median_pitch);
  }
}

Disturbances::Disturbances(const Config & config)
: config_(config),
  tilt_tracker_(config),
  tilt_hold_(config.lift_s),
  accel_hold_(config.lift_accel_s),
  carried_hold_(config.carried_s),
  rotated_hold_(config.rotate_s)
{
}

void Disturbances::on_imu(
  double now, const Quaternion & orientation, const Vector3 & angular_velocity, const Vector3 & linear_acceleration)
{
  const auto [roll, pitch] = roll_pitch_deg(orientation);
  tilt_deg_ = tilt_tracker_.tilt(roll, pitch);
  tilt_tracker_.add(now, roll, pitch);
  imu_ = std::make_pair(now, world_vector(orientation, angular_velocity)[2]);
  const double norm = std::sqrt(
    linear_acceleration[0] * linear_acceleration[0] + linear_acceleration[1] * linear_acceleration[1] +
    linear_acceleration[2] * linear_acceleration[2]);
  const bool gravity_included = std::abs(norm - GRAVITY) <= config_.gravity_band;
  const double a_z = world_vector(orientation, linear_acceleration)[2];
  if (accel_hold_.update(now, !gravity_included && std::abs(a_z) > config_.lift_accel_z)) {
    accel_evidence_until_ = now + config_.lift_confirm_s;
  }
}

void Disturbances::on_lidar_odom(double now, double speed_mps) { lidar_ = std::make_pair(now, speed_mps); }

void Disturbances::on_wheel_odom(double now, double speed_mps, double yaw_rate)
{
  wheel_ = std::make_tuple(now, speed_mps, yaw_rate);
}

void Disturbances::on_cmd(double now, double linear_mps, double angular_rad_s)
{
  cmd_ = std::make_tuple(now, std::abs(linear_mps), std::abs(angular_rad_s));
}

bool Disturbances::fresh(const std::optional<double> & sample_time, double now) const
{
  return sample_time && now - *sample_time <= config_.input_timeout_s;
}

DisturbanceReport Disturbances::evaluate(double now)
{
  const Config & cfg = config_;
  DisturbanceReport report;
  const bool imu_fresh = fresh(imu_ ? std::optional<double>(imu_->first) : std::nullopt, now);
  std::optional<double> lidar_speed;
  if (lidar_ && fresh(lidar_->first, now)) {
    lidar_speed = lidar_->second;
  }
  std::optional<std::tuple<double, double, double>> wheel;
  if (wheel_ && fresh(std::get<0>(*wheel_), now)) {
    wheel = wheel_;
  }
  const bool cmd_active = cmd_ && now - std::get<0>(*cmd_) <= cfg.cmd_timeout_s;
  const bool drive_cmd = cmd_active && std::max(std::get<1>(*cmd_), std::get<2>(*cmd_)) > cfg.cmd_eps;
  const bool angular_cmd = cmd_active && std::get<2>(*cmd_) > cfg.cmd_eps;

  const std::optional<double> tilt = imu_fresh ? tilt_deg_ : std::nullopt;
  if (tilt_hold_.update(now, tilt && *tilt > cfg.lift_tilt_deg)) {
    report.reasons.insert(REASON_LIFTED);
  }
  if (accel_evidence_until_ && now <= *accel_evidence_until_) {
    const bool moved = lidar_speed && *lidar_speed > cfg.carried_speed_mps;
    if ((tilt && *tilt > cfg.lift_confirm_tilt_deg) || moved) {
      report.reasons.insert(REASON_LIFTED);
    }
  }

  if (lidar_speed && wheel) {
    report.unexplained_speed_mps = drive_cmd ? 0.0 : std::max(0.0, *lidar_speed - std::get<1>(*wheel));
  }
  const bool carried = lidar_speed && wheel && !drive_cmd && *lidar_speed > cfg.carried_speed_mps &&
                       std::get<1>(*wheel) < cfg.wheel_still_mps;
  if (carried_hold_.update(now, carried)) {
    report.reasons.insert(REASON_CARRIED);
  }

  std::optional<double> yaw_rate;
  if (imu_ && imu_fresh) {
    yaw_rate = imu_->second;
  }
  const bool rotated = yaw_rate && wheel && !angular_cmd && std::abs(*yaw_rate) > cfg.rotate_rate_rad_s &&
                       std::abs(std::get<2>(*wheel)) < cfg.wheel_still_rad_s;
  if (rotated_hold_.update(now, rotated)) {
    report.reasons.insert(REASON_ROTATED);
  }
  report.tilt_deg = tilt;
  report.imu_yaw_rate = yaw_rate;
  return report;
}

}  // namespace map_guard_cpp
