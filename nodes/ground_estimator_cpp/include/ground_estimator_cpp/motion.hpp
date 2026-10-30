// Head-motion gate (task T2): the angular speed of the camera in base_link between two stamps, and the verdict of the
// gate. Pure core code, no ROS types.
#ifndef GROUND_ESTIMATOR_CPP__MOTION_HPP_
#define GROUND_ESTIMATOR_CPP__MOTION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ground_estimator_cpp/config.hpp"

namespace ground_estimator_cpp
{

// Angular speed of a frame whose orientation went from `before` to `after` in dt_s seconds: the angle of the relative
// rotation over the time.
// @param before orientation of the camera in base_link at the earlier stamp.
// @param after orientation of the camera in base_link at the later stamp.
// @param dt_s seconds between the two stamps (> 0).
// @return degrees per second, >= 0.
double rotation_rate_deg_s(const Eigen::Matrix3d & before, const Eigen::Matrix3d & after, double dt_s);

// The gate: a frame is moving when the gate is enabled and the camera turned faster than motion.max_head_rate_deg_s
// (strictly: exactly at the limit still votes).
// @param rate_deg_s the angular speed of the frame.
// @param cfg the gate settings.
// @return true when the frame must not vote.
bool is_moving(double rate_deg_s, const MotionConfig & cfg);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__MOTION_HPP_
