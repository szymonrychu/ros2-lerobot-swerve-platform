// The ground estimator node (design D1, D3, D4, D6, D7, D11, D14, D15): fits the floor plane from the ToF cloud with
// an IMU prior, publishes the lethal obstacle and drop clouds selected by the plane-relative bands and the obstacle
// versus warning rule (base_link coordinates, the cloud stamp), the warning areas in the map frame, the RGB-D cloud
// labelled against the tracked plane and the JSON state. Only real data with valid TF is published: no plane, no cloud.
//
// Topics (part ground-topics, names from Config): <ns>/obstacle_points, <ns>/drop_points (PointCloud2 XYZ, reliable
// depth 1), <ns>/warning_areas (std_msgs/String, reliable depth 1), <ns>/labelled_points (PointCloud2 XYZ rgb label,
// best_effort depth 1), <ns>/state (std_msgs/String, reliable transient_local depth 1).
// Metrics (part ground-metrics): see the constants below and GroundNode::GroundNode.
#ifndef GROUND_ESTIMATOR_CPP__GROUND_NODE_HPP_
#define GROUND_ESTIMATOR_CPP__GROUND_NODE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ground_estimator_cpp/cell_median.hpp"
#include "ground_estimator_cpp/colour.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/state.hpp"
#include "ground_estimator_cpp/tracker.hpp"
#include "ground_estimator_cpp/traversability.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace ground_estimator_cpp
{

constexpr const char * NODE_NAME = "ground_estimator";
// RANSAC seed of every fit: fixed per process so a cloud always gives the same plane.
constexpr unsigned FIT_SEED = 42;
// Throttle of the drop logs, milliseconds.
constexpr int64_t DROP_LOG_THROTTLE_MS = 5000;

// Metric names (part ground-metrics).
constexpr const char * METRIC_FITS = "ground_fits_total";
constexpr const char * METRIC_FIT_SECONDS = "ground_fit_seconds";
constexpr const char * METRIC_INLIER_RATIO = "ground_inlier_ratio";
constexpr const char * METRIC_SLOPE = "ground_slope_deg";
constexpr const char * METRIC_PLANE_VS_BASE = "ground_plane_vs_base_deg";
constexpr const char * METRIC_REGIONS = "ground_regions";
constexpr const char * METRIC_WARNING_AREAS = "ground_warning_areas";
constexpr const char * METRIC_OBSTACLE_POINTS = "ground_obstacle_points";
constexpr const char * METRIC_DROP_POINTS = "ground_drop_points";
constexpr const char * METRIC_TEMPORAL_CELLS = "ground_temporal_cells";
constexpr const char * METRIC_MISMATCH_RATIO = "ground_colour_mismatch_ratio";
constexpr const char * METRIC_DROPPED = "ground_clouds_dropped_total";
constexpr const char * METRIC_LABELLED = "ground_labelled_clouds_total";

// result label of ground_fits_total for a valid plane (rejects use to_string(FitReject)).
constexpr const char * FIT_RESULT_OK = "ok";

// reason labels of ground_clouds_dropped_total.
constexpr const char * DROP_REASON_TF = "tf";                        // a cloud without TF to base/fixed frame
constexpr const char * DROP_REASON_NO_PLANE_RGBD = "no_plane_rgbd";  // an RGB-D cloud without a plane at its stamp
constexpr const char * DROP_REASON_NO_MAP_TF = "no_map_tf";          // warning areas without TF to warnings.frame
constexpr const char * DROP_REASON_BAD_INPUT = "bad_input";          // a cloud without float32 x, y, z (or rgb)

// State reasons that are not a fit reject.
constexpr const char * REASON_NO_TF = "no_tf";
constexpr const char * REASON_NO_CLOUD = "no_cloud";

class GroundNode : public rclcpp::Node
{
public:
  // @param config the validated settings.
  // @param tf_buffer the TF buffer to look transforms up in; when null the node owns a buffer fed by a TF listener.
  // @param options node options.
  explicit GroundNode(
    const Config & config, std::shared_ptr<tf2_ros::Buffer> tf_buffer = nullptr,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // @return the node's metrics registry.
  ros2_common_cpp::Metrics & metrics() { return metrics_; }

private:
  // The latest colour warning areas, in base_link at their RGB-D stamp, with what is needed to move them.
  struct ColourFrame
  {
    std::vector<WarningArea> areas;  // all warnings
    Plane plane;                     // the floor plane in base_link at the stamp
    Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
    double stamp_s = 0.0;
  };

  // What the last ToF cloud produced, for the state.
  struct Cycle
  {
    std::vector<Region> regions;
    std::vector<WarningArea> warnings;  // base_link at the cloud stamp, cls "warning" only
    std::size_t obstacle_points = 0;
    std::size_t drop_points = 0;
  };

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void on_rgbd(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void on_state_timer();

  void process_plane(
    const sensor_msgs::msg::PointCloud2 & msg, const TrackedPlane & tracked, const Eigen::Isometry3d & fixed_T_base,
    double stamp_s, const std::chrono::steady_clock::time_point & started);
  void publish_warning_areas(const builtin_interfaces::msg::Time & stamp, const Plane & plane);
  void publish_state(int64_t stamp_ns, const std::optional<TrackedPlane> & plane, const char * reason_override);
  void publish_state_on_change(int64_t stamp_ns, const std::optional<TrackedPlane> & plane);
  void drop(const char * reason, const std::string & detail);
  bool lookup(
    const std::string & target, const std::string & source, const builtin_interfaces::msg::Time & stamp,
    Eigen::Isometry3d & out);
  void set_gauge(const char * name, double value);

  Config config_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  PlaneTracker tracker_;
  CellMedianFilter cell_filter_;
  FloorColourModel colour_model_;
  std::optional<ImuSample> latest_imu_;
  std::optional<Eigen::Quaterniond> imu_to_base_;  // static rotation of IMU-frame vectors into base_frame
  std::string imu_frame_;                          // the frame imu_to_base_ was looked up for
  Prior prior_;                                    // of the last ToF cloud
  std::size_t last_inliers_ = 0;
  double last_inlier_ratio_ = 0.0;
  std::optional<FitReject> last_reject_;
  const char * last_reason_ = nullptr;  // REASON_NO_TF after a TF drop
  bool have_cloud_ = false;
  std::optional<bool> last_state_valid_;
  Cycle cycle_;
  std::optional<ColourFrame> colour_;
  std::optional<ColourInputs> colour_inputs_;

  // Reused between clouds (the 10 Hz path keeps its capacity).
  pcl::PointCloud<pcl::PointXYZ> base_cloud_;
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud_;  // base_cloud_ after the temporal median
  pcl::PointCloud<pcl::PointXYZ> unclustered_above_, unclustered_below_;
  std::vector<float> unclustered_above_h_, unclustered_below_h_;
  std::vector<int> unclustered_index_;
  pcl::PointCloud<pcl::PointXYZRGB> rgbd_cloud_;
  std::vector<Label> labels_;
  pcl::PointCloud<pcl::PointXYZ> mismatch_cloud_;
  std::vector<float> mismatch_h_;
  std::vector<WarningArea> areas_in_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rgbd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr drop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr labelled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warnings_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  ros2_common_cpp::Metrics metrics_;
  prometheus::Histogram * fit_seconds_ = nullptr;
  prometheus::Counter * labelled_ = nullptr;
  // Gauge families registered without a series: a gauge appears with its first real value.
  std::unordered_map<std::string, prometheus::Family<prometheus::Gauge> *> gauges_;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__GROUND_NODE_HPP_
