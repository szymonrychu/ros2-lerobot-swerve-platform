#include "ground_estimator_cpp/motion.hpp"

#include <cmath>

namespace ground_estimator_cpp
{

namespace
{
constexpr double RAD_TO_DEG = 180.0 / M_PI;
}  // namespace

double rotation_rate_deg_s(const Eigen::Matrix3d & before, const Eigen::Matrix3d & after, double dt_s)
{
  const Eigen::AngleAxisd relative(before.transpose() * after);
  return std::abs(relative.angle()) * RAD_TO_DEG / dt_s;
}

bool is_moving(double rate_deg_s, const MotionConfig & cfg)
{
  return cfg.enabled && rate_deg_s > cfg.max_head_rate_deg_s;
}

}  // namespace ground_estimator_cpp
