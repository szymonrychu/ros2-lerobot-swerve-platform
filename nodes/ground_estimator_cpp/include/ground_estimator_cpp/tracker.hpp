// Floor plane tracking: IMU prior of the floor normal, a plane held in the fixed frame so it follows the robot, and the
// slope ahead against the tilt_guard limits.
#ifndef GROUND_ESTIMATOR_CPP__TRACKER_HPP_
#define GROUND_ESTIMATOR_CPP__TRACKER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <optional>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

// Latest IMU reading, already converted with gravity_up_in_base.
struct ImuSample
{
  Eigen::Vector3d up_base = Eigen::Vector3d::UnitZ();  // world up in base_link
  double stamp_s = 0.0;                                // seconds
};

// The floor normal prior of one fit.
struct Prior
{
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  bool from_imu = false;
};

// Worst-axis tilt level (tilt_guard semantics: strictly beyond the limit).
enum class Level
{
  ok,
  warning,
  error
};

// Floor tilt as the base roll/pitch would be on that plane (REP-103: roll > 0 left side up, pitch > 0 nose down).
struct Slope
{
  double total_deg = 0.0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  Level level = Level::ok;
};

// Whether a returned plane is the fit of its own cloud or held across failed fits.
enum class PlaneSource
{
  fit,
  held
};

// A tracked plane in base_link.
struct TrackedPlane
{
  Plane plane;
  double age_s = 0.0;        // query stamp minus fit stamp
  double fit_stamp_s = 0.0;  // stamp of the fit it comes from
  PlaneSource source = PlaneSource::fit;
};

// World up in base_link from the IMU orientation (REP-103), with the level reference removed so a level base gives +z.
// @param imu_orientation IMU frame orientation in the world.
// @param imu_to_base rotation of IMU-frame vectors into base_link.
// @param level_roll_deg IMU roll reading on level ground.
// @param level_pitch_deg IMU pitch reading on level ground.
// @return unit up vector in base_link.
Eigen::Vector3d gravity_up_in_base(
  const Eigen::Quaterniond & imu_orientation, const Eigen::Quaterniond & imu_to_base, double level_roll_deg,
  double level_pitch_deg);

// The IMU up when the sample is younger than imu.timeout_s, else base_link +z.
// @param latest the last IMU sample, if any.
// @param now_s current time in seconds.
// @param cfg IMU settings.
// @return the prior.
Prior choose_prior(const std::optional<ImuSample> & latest, double now_s, const ImuConfig & cfg);

// Slope of a floor plane against gravity.
// @param plane_base the plane in base_link.
// @param gravity_up_base world up in base_link.
// @param cfg tilt limits.
// @return roll, pitch, total and level.
Slope slope_vs_gravity(const Plane & plane_base, const Eigen::Vector3d & gravity_up_base, const SlopeConfig & cfg);

// Angle between the plane normal and base_link +z.
// @param plane_base the plane.
// @return degrees.
double plane_vs_base_deg(const Plane & plane_base);

// Angle between gravity up and base_link +z.
// @param gravity_up_base world up in base_link.
// @return degrees.
double base_tilt_deg(const Eigen::Vector3d & gravity_up_base);

// Keeps recent fits in the fixed frame and answers for any stamp within the hold age.
class PlaneTracker
{
public:
  explicit PlaneTracker(const FitConfig & cfg);

  // Record a successful fit (EMA smoothed against the previous fit when it is within hold_s).
  // @param plane_base the fitted plane in base_link.
  // @param base_to_fixed pose of base_link in the fixed frame at stamp_s.
  // @param stamp_s cloud stamp in seconds.
  void update(const Plane & plane_base, const Eigen::Isometry3d & base_to_fixed, double stamp_s);

  // Record a cloud that yielded no valid plane.
  // @param stamp_s cloud stamp in seconds.
  void mark_failed(double stamp_s);

  // The newest fit at or before stamp_s, in base_link at stamp_s; nullopt past the hold age or with no such fit.
  // @param base_to_fixed pose of base_link in the fixed frame at stamp_s.
  // @param stamp_s query stamp in seconds.
  // @return the plane, or nullopt.
  std::optional<TrackedPlane> current(const Eigen::Isometry3d & base_to_fixed, double stamp_s) const;

  // Number of stored fits.
  std::size_t history_size() const { return fits_.size(); }

private:
  struct Stored
  {
    Eigen::Vector3d normal;  // fixed frame
    double d;
    double stamp_s;
  };

  void prune(double stamp_s);

  FitConfig cfg_;
  std::deque<Stored> fits_;
  std::deque<double> failures_;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__TRACKER_HPP_
