// Motion compensation and the motion gate of a ToF-RGB pair (design D6), Eigen only (the node converts its tf2
// lookups with tf2_eigen). The two cameras are exposed at different times t_tof and t_rgb while the head pans or the
// robot turns, so the registration transform is composed through a fixed frame (odom) from each camera's pose at its
// own stamp: tf2's time-travel lookupTransform(rgb, t_rgb, tof, t_tof, fixed). A ToF frame integrates four phase
// captures, so blur during fast motion cannot be compensated: pairs whose ToF frame moves faster than the limits, or
// whose stamps are too far apart, are dropped.
//
// Pose convention: fixed_T_x maps points of frame x into the fixed frame (p_fixed = fixed_T_x * p_x), as returned by
// lookupTransform(fixed, x, t).
#ifndef HEAD_RGBD_CPP__MOTION_HPP_
#define HEAD_RGBD_CPP__MOTION_HPP_

#include <Eigen/Geometry>
#include <cstdint>

#include "head_rgbd_cpp/config.hpp"

namespace head_rgbd_cpp
{

// Drop reason labels of head_rgbd_pairs_dropped_total{reason}.
constexpr const char * DROP_REASON_PAIR_DT = "pair_dt";
constexpr const char * DROP_REASON_MOTION = "motion";
// Label of an accepted pair (not a drop reason).
constexpr const char * PAIR_ACCEPTED_LABEL = "accept";

// The rate of a frame between two poses, in the fixed frame.
struct MotionRate
{
  double angular_deg_s = 0.0;  // relative rotation angle over the interval, deg/s, >= 0
  double linear_m_s = 0.0;     // distance travelled by the frame origin over the interval, m/s, >= 0
};

// The gate decision of one ToF-RGB pair.
enum class PairDecision
{
  accept,   // register the pair
  pair_dt,  // |t_rgb - t_tof| > sync.max_pair_dt_s
  motion,   // the ToF frame moves faster than motion.max_angular_rate_deg_s or motion.max_linear_rate_m_s
};

// The transform that maps ToF points captured at t_tof into the RGB optical frame at t_rgb:
// inverse(fixed_T_rgb_at_trgb) * fixed_T_tof_at_ttof, the result of tf2's
// lookupTransform(rgb, t_rgb, tof, t_tof, fixed).
// @param fixed_T_rgb_at_trgb the RGB optical frame pose in the fixed frame at the RGB stamp.
// @param fixed_T_tof_at_ttof the ToF optical frame pose in the fixed frame at the ToF stamp.
// @return rgb_T_tof, p_rgb(t_rgb) = rgb_T_tof * p_tof(t_tof), for register_depth.
Eigen::Isometry3d compensated_transform(
  const Eigen::Isometry3d & fixed_T_rgb_at_trgb, const Eigen::Isometry3d & fixed_T_tof_at_ttof);

// The registration transform the node uses: compensated_transform when config.compensate, else the static extrinsic
// (the gate applies either way).
// @param config the motion settings.
// @param rgb_T_tof_static the static ToF -> RGB transform (both poses at the same stamp).
// @param fixed_T_rgb_at_trgb the RGB optical frame pose in the fixed frame at the RGB stamp.
// @param fixed_T_tof_at_ttof the ToF optical frame pose in the fixed frame at the ToF stamp.
// @return rgb_T_tof for register_depth.
Eigen::Isometry3d registration_transform(
  const MotionConfig & config, const Eigen::Isometry3d & rgb_T_tof_static,
  const Eigen::Isometry3d & fixed_T_rgb_at_trgb, const Eigen::Isometry3d & fixed_T_tof_at_ttof);

// The angular and linear rate of a frame between two of its poses in the fixed frame.
// @param fixed_T_a the frame pose at the start of the interval.
// @param fixed_T_b the frame pose at the end of the interval.
// @param dt_s the interval, seconds (> 0, finite).
// @return the angle of the relative rotation inverse(R_a) * R_b over dt_s in deg/s and the distance between the two
//   origins over dt_s in m/s.
// @throws std::invalid_argument when dt_s is not a positive finite number.
MotionRate frame_rate(const Eigen::Isometry3d & fixed_T_a, const Eigen::Isometry3d & fixed_T_b, double dt_s);

// The gate of a pair: pair_dt when |t_rgb_ns - t_tof_ns| > sync.max_pair_dt_s, else motion when the ToF rate is
// above motion.max_angular_rate_deg_s or motion.max_linear_rate_m_s (or not finite), else accept. A value equal to
// a limit is accepted. motion.compensate does not change the gate.
// @param t_rgb_ns the RGB stamp, nanoseconds.
// @param t_tof_ns the ToF stamp, nanoseconds.
// @param tof_rate the ToF frame's rate over motion.rate_window_s before t_tof.
// @param sync the pairing settings.
// @param motion the motion settings.
// @return the decision.
PairDecision evaluate_pair(
  int64_t t_rgb_ns, int64_t t_tof_ns, const MotionRate & tof_rate, const SyncConfig & sync,
  const MotionConfig & motion);

// The metric label of a decision.
// @param decision the gate decision.
// @return DROP_REASON_PAIR_DT, DROP_REASON_MOTION or PAIR_ACCEPTED_LABEL.
const char * decision_label(PairDecision decision);

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__MOTION_HPP_
