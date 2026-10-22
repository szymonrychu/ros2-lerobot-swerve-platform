// Slope of an inclined surface from the raw (unfiltered) plane-relative heights of the last clouds around a cluster:
// a mixture fit of the floor, an inclined plane and outliers over the cluster's fixed-frame cells and their neighbours.
// The temporal cell median gives a cluster one sample per 10 cm cell; a small sparse ramp then has three or four
// samples whose cells straddle its foot, sides and top edge, and the fitted slope scatters by several degrees from
// cloud to cloud. The raw points of several clouds over the whole surface, its foot included, pin it down.
#ifndef GROUND_ESTIMATOR_CPP__SURFACE_SLOPE_HPP_
#define GROUND_ESTIMATOR_CPP__SURFACE_SLOPE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

// Temporal windows of raw clouds a cluster slope is fit over: two cell-median windows (16 clouds by default, at most
// temporal.max_age_s old). One window scatters a sparse 14 deg mat by ~0.75 deg, two by ~0.5 deg, so the 15 deg rule
// holds it a warning.
constexpr int SLOPE_HISTORY_WINDOWS = 2;

// Smallest cell-median slope (deg) of a cluster whose slope is refit from the raw clouds. A flat top (a box, a sill)
// keeps the cell-median fit: the ToF returns of its front face read every height up to the top and tilt a raw fit by a
// few degrees, which would turn a sill into a ramp.
constexpr double SLOPE_REFIT_MIN_DEG = 5.0;

// One raw point: fixed-frame xy, signed height above the floor plane of its cloud and its fixed-frame cell.
struct HeightSample
{
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  double height = 0.0;
  std::int64_t cell = 0;  // the samples of one cell share how much of it is surface and how much floor
};

// The history settings of the cluster slopes: the temporal settings with SLOPE_HISTORY_WINDOWS windows.
// @param temporal the temporal median settings (cell_size_m, window_frames, max_age_s).
// @return the settings for RawHeightHistory.
TemporalConfig slope_history_config(const TemporalConfig & temporal);

// The raw plane-relative heights of the last clouds, binned by fixed-frame cell (the CellMedianFilter grid).
class RawHeightHistory
{
public:
  // @param cfg cell_size_m (the cell edge), window_frames (clouds kept) and max_age_s (older clouds are dropped).
  explicit RawHeightHistory(const TemporalConfig & cfg);

  // Add one cloud; drops the clouds beyond window_frames or older than max_age_s. A stamp older than the newest one
  // (time jumped back) clears the history first.
  // @param points_base the raw cloud in base_link (non-finite points are skipped).
  // @param plane the floor plane of the cloud in base_link.
  // @param fixed_T_base pose of base_link in the fixed frame at the cloud stamp.
  // @param stamp_s cloud stamp in seconds.
  void add(
    const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base,
    double stamp_s);

  // The kept samples of the cells holding any of the points and of their 8 neighbours.
  // @param points_base points in base_link (a cluster).
  // @param fixed_T_base pose of base_link in the fixed frame.
  // @param clouds if not null, receives the number of kept clouds with samples in those cells.
  // @return the samples, fixed-frame xy.
  std::vector<HeightSample> around(
    const pcl::PointCloud<pcl::PointXYZ> & points_base, const Eigen::Isometry3d & fixed_T_base,
    std::size_t * clouds = nullptr) const;

  // Forget every cloud.
  void reset();

  // Clouds currently kept.
  // @return the count.
  std::size_t frames() const { return frames_.size(); }

private:
  struct Frame
  {
    double stamp_s = 0.0;
    std::unordered_map<std::int64_t, std::vector<HeightSample>> cells;
  };

  std::int64_t key_of(const Eigen::Vector2d & xy_fixed) const;

  TemporalConfig cfg_;
  std::deque<Frame> frames_;
};

// Slope of an inclined surface rising from (sign +1) or falling below (sign -1) the floor, from samples of both:
// expectation-maximisation of a mixture of the floor N(m_f, s_f), the surface N(a x + b y + c, s_s) and outliers
// uniform over the samples' height range (a step's front face, mixed pixels), started from a least-squares plane
// through the samples beyond surface_min_m. Each cell has its own share of surface and floor, so a plane cannot tilt
// into a wedge that takes the returns of floor cells, and the low returns of a ramp's foot stay on the ramp. Where the
// plane lies on the floor's far side (ahead of a ramp's foot) a sample is not on the surface.
// @param samples floor and surface samples (RawHeightHistory::around).
// @param sign +1 for a surface above the floor (a step cluster), -1 below it (a drop cluster).
// @param surface_min_m |height| beyond which a sample starts in the surface (fit.distance_m).
// @return atan(hypot(a, b)) in degrees; nullopt with fewer than three starting samples or no finite fit.
std::optional<double> surface_slope_deg(const std::vector<HeightSample> & samples, double sign, double surface_min_m);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__SURFACE_SLOPE_HPP_
