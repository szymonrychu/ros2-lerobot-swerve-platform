#include "head_rgbd_cpp/motion.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace head_rgbd_cpp
{

constexpr double NS_PER_S = 1e9;
constexpr double DEG_PER_RAD = 180.0 / EIGEN_PI;

Eigen::Isometry3d compensated_transform(
  const Eigen::Isometry3d & fixed_T_rgb_at_trgb, const Eigen::Isometry3d & fixed_T_tof_at_ttof)
{
  // p_fixed = fixed_T_tof * p_tof (where the ToF point was at t_tof), then into the RGB frame as it was at t_rgb.
  return fixed_T_rgb_at_trgb.inverse(Eigen::Isometry) * fixed_T_tof_at_ttof;
}

Eigen::Isometry3d registration_transform(
  const MotionConfig & config, const Eigen::Isometry3d & rgb_T_tof_static,
  const Eigen::Isometry3d & fixed_T_rgb_at_trgb, const Eigen::Isometry3d & fixed_T_tof_at_ttof)
{
  return config.compensate ? compensated_transform(fixed_T_rgb_at_trgb, fixed_T_tof_at_ttof) : rgb_T_tof_static;
}

MotionRate frame_rate(const Eigen::Isometry3d & fixed_T_a, const Eigen::Isometry3d & fixed_T_b, double dt_s)
{
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    throw std::invalid_argument("frame_rate: dt_s must be a positive finite number, got " + std::to_string(dt_s));
  }
  // The angle of the relative rotation from the quaternion, 2 * atan2(|v|, |w|): accurate near 0 and in [0, pi].
  const Eigen::Quaterniond q(fixed_T_a.linear().transpose() * fixed_T_b.linear());
  const double angle_rad = 2.0 * std::atan2(q.vec().norm(), std::abs(q.w()));
  const double distance_m = (fixed_T_b.translation() - fixed_T_a.translation()).norm();
  return MotionRate{angle_rad * DEG_PER_RAD / dt_s, distance_m / dt_s};
}

PairDecision evaluate_pair(
  int64_t t_rgb_ns, int64_t t_tof_ns, const MotionRate & tof_rate, const SyncConfig & sync, const MotionConfig & motion)
{
  const double pair_dt_s = std::abs(static_cast<double>(t_rgb_ns - t_tof_ns)) / NS_PER_S;
  if (pair_dt_s > sync.max_pair_dt_s) {
    return PairDecision::pair_dt;
  }
  // Written as "not within the limit" so that a NaN rate drops the pair.
  const bool angular_ok = tof_rate.angular_deg_s <= motion.max_angular_rate_deg_s;
  const bool linear_ok = tof_rate.linear_m_s <= motion.max_linear_rate_m_s;
  if (!angular_ok || !linear_ok) {
    return PairDecision::motion;
  }
  return PairDecision::accept;
}

const char * decision_label(PairDecision decision)
{
  switch (decision) {
    case PairDecision::pair_dt:
      return DROP_REASON_PAIR_DT;
    case PairDecision::motion:
      return DROP_REASON_MOTION;
    case PairDecision::accept:
      break;
  }
  return PAIR_ACCEPTED_LABEL;
}

}  // namespace head_rgbd_cpp
