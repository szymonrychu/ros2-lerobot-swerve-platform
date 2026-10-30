#include "ground_estimator_cpp/tracker.hpp"

#include <algorithm>
#include <cmath>

namespace ground_estimator_cpp
{

namespace
{
constexpr double DEG = M_PI / 180.0;
constexpr double HISTORY_MARGIN_S = 1.0;
constexpr double EPS_AXIS = 1e-12;  // rotation axis length below which two directions count as parallel

double angle_deg(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return std::atan2(a.cross(b).norm(), a.dot(b)) / DEG;
}

// Minimal rotation taking the unit vector a to +z. Eigen's FromTwoVectors runs a JacobiSVD whose uninitialised-read
// warning GCC 13 raises on amd64 (-Werror), so the axis-angle form is built here; it also stays defined when a is
// +z (identity) or -z (half turn about x).
Eigen::Quaterniond rotation_to_up(const Eigen::Vector3d & a)
{
  const Eigen::Vector3d axis = a.cross(Eigen::Vector3d::UnitZ());
  const double sin_angle = axis.norm();
  if (sin_angle < EPS_AXIS) {
    return a.z() > 0.0 ? Eigen::Quaterniond::Identity()
                       : Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(std::atan2(sin_angle, a.z()), axis / sin_angle));
}

Level level_of(double value_deg, double warn_deg, double error_deg)
{
  const double v = std::abs(value_deg);
  return v > error_deg ? Level::error : (v > warn_deg ? Level::warning : Level::ok);
}
}  // namespace

Eigen::Vector3d gravity_up_in_base(
  const Eigen::Quaterniond & imu_orientation, const Eigen::Quaterniond & imu_to_base, double level_roll_deg,
  double level_pitch_deg)
{
  const Eigen::Quaterniond reference = Eigen::AngleAxisd(level_pitch_deg * DEG, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(level_roll_deg * DEG, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d up_imu = imu_orientation.normalized().conjugate() * Eigen::Vector3d::UnitZ();
  return (imu_to_base.normalized() * (reference * up_imu)).normalized();
}

Prior choose_prior(const std::optional<ImuSample> & latest, double now_s, const ImuConfig & cfg)
{
  if (latest && now_s - latest->stamp_s < cfg.timeout_s) {
    return Prior{latest->up_base, true};
  }
  return Prior{Eigen::Vector3d::UnitZ(), false};
}

Slope slope_vs_gravity(const Plane & plane_base, const Eigen::Vector3d & gravity_up_base, const SlopeConfig & cfg)
{
  const Eigen::Vector3d n = plane_base.normal.normalized();
  // Base aligned to the plane would see gravity up as R * g, R taking the plane normal to +z.
  const Eigen::Vector3d g = rotation_to_up(n) * gravity_up_base.normalized();
  Slope s;
  s.pitch_deg = std::asin(std::clamp(-g.x(), -1.0, 1.0)) / DEG;
  s.roll_deg = std::atan2(g.y(), g.z()) / DEG;
  s.total_deg = angle_deg(n, gravity_up_base);
  const Level r = level_of(s.roll_deg, cfg.roll_warn_deg, cfg.roll_error_deg);
  const Level p = level_of(s.pitch_deg, cfg.pitch_warn_deg, cfg.pitch_error_deg);
  s.level = std::max(r, p);
  return s;
}

double plane_vs_base_deg(const Plane & plane_base) { return angle_deg(plane_base.normal, Eigen::Vector3d::UnitZ()); }

double base_tilt_deg(const Eigen::Vector3d & gravity_up_base)
{
  return angle_deg(gravity_up_base, Eigen::Vector3d::UnitZ());
}

PlaneTracker::PlaneTracker(const FitConfig & cfg) : cfg_(cfg) {}

void PlaneTracker::prune(double stamp_s)
{
  const double oldest = stamp_s - cfg_.hold_s - HISTORY_MARGIN_S;
  while (!fits_.empty() && fits_.front().stamp_s < oldest) {
    fits_.pop_front();
  }
  while (!failures_.empty() && failures_.front() < oldest) {
    failures_.pop_front();
  }
}

void PlaneTracker::update(const Plane & plane_base, const Eigen::Isometry3d & base_to_fixed, double stamp_s)
{
  Stored s;
  s.normal = base_to_fixed.linear() * plane_base.normal.normalized();
  s.d = plane_base.d - s.normal.dot(base_to_fixed.translation());
  s.stamp_s = stamp_s;
  if (!fits_.empty() && stamp_s >= fits_.back().stamp_s && stamp_s - fits_.back().stamp_s <= cfg_.hold_s) {
    const Stored & prev = fits_.back();
    const double a = cfg_.smoothing;
    s.normal = (a * s.normal + (1.0 - a) * prev.normal).normalized();
    s.d = a * s.d + (1.0 - a) * prev.d;
  }
  const auto pos =
    std::upper_bound(fits_.begin(), fits_.end(), stamp_s, [](double t, const Stored & f) { return t < f.stamp_s; });
  fits_.insert(pos, s);
  prune(std::max(stamp_s, fits_.back().stamp_s));
}

void PlaneTracker::mark_failed(double stamp_s)
{
  failures_.insert(std::upper_bound(failures_.begin(), failures_.end(), stamp_s), stamp_s);
  prune(std::max(stamp_s, fits_.empty() ? stamp_s : fits_.back().stamp_s));
}

std::optional<TrackedPlane> PlaneTracker::current(const Eigen::Isometry3d & base_to_fixed, double stamp_s) const
{
  const Stored * best = nullptr;
  for (const Stored & f : fits_) {
    if (f.stamp_s <= stamp_s) {
      best = &f;
    }
  }
  if (best == nullptr || stamp_s - best->stamp_s > cfg_.hold_s) {
    return std::nullopt;
  }
  TrackedPlane out;
  out.plane.normal = base_to_fixed.linear().transpose() * best->normal;
  out.plane.d = best->d + best->normal.dot(base_to_fixed.translation());
  out.age_s = stamp_s - best->stamp_s;
  out.fit_stamp_s = best->stamp_s;
  const bool failed_between =
    std::any_of(failures_.begin(), failures_.end(), [&](double f) { return f > best->stamp_s && f <= stamp_s; });
  out.source = failed_between ? PlaneSource::held : PlaneSource::fit;
  return out;
}

}  // namespace ground_estimator_cpp
