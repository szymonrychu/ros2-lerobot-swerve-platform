// The ground estimator node (design D1, D3, D4, D6, D7, D11, D14, D15): fits the floor plane from the depth cloud with
// an IMU prior, publishes the lethal obstacle and drop clouds selected by the plane-relative bands and the obstacle
// versus warning rule (base_link coordinates, the cloud stamp), the warning areas in the map frame, the RGB-D cloud
// labelled against the tracked plane and the JSON state. Only real data with valid TF is published: no plane, no cloud.
//
// Stable classes (ground-detection-robustness D3, D5, D7, D8; part stable-warning-areas): every depth frame votes the
// rule's lethal points and warning areas (depth, colour) into a CellClassFilter on the temporal cell grid of the fixed
// frame. The lethal clouds carry only this frame's points whose cell is stable lethal (a lethal observation in a cell
// that is not yet stable is withheld, a held cell publishes nothing it did not see this frame), a region is an obstacle
// when a cell holding its cluster's points is stable lethal, and the warning areas are the stable warning cell groups
// with cell-aligned polygons. An area continuing a published one (same source, either polygon holding the other's
// vertex centroid) takes the kind of the cell classes as is (the kind hold is hysteresis.area_kind_frames alone) and
// keeps the published height and slope while the kind is unchanged and the new values stay within
// AREA_HEIGHT_DEADBAND_M and AREA_SLOPE_DEADBAND_DEG of them, so a static scene publishes identical areas (the values are earlier measurements
// of the same area; the polygon is always this frame's cells).
//
// Cluster slopes (temporal filter on): the slope of a cluster whose cell-median fit reads at least SLOPE_REFIT_MIN_DEG
// is refit from the raw heights of the last SLOPE_HISTORY_WINDOWS * temporal.window_frames clouds in its cells and
// their neighbours (surface_slope.hpp), so a small sparse ramp is judged on its whole surface instead of three or four
// cell medians. A new incline is fit over the clouds kept so far, so a steep one is lethal from its first frame; when
// the refit fails its slope is unknown.
//
// Guidance (part guided-node; design D3, D4, D5, D12), only while guidance.enabled: an ExactTime synchronizer over the
// registered depth, its camera info and the JPEG of head_rgbd_cpp (one header per pair) plus the latest depth
// camera info.
// A pair is decoded, moved to base_link and the fixed frame at its stamp, judged against the tracked plane at its
// stamp and sampled (guidance.hpp); its depth samples are kept for the edge points (edge_points.hpp) and its
// no-return samples voted into the NoReturnTracker (no_return.hpp) with the stable lethal and depth warning cells as
// blocked cells. Any failure drops the whole pair. On each depth cloud with a plane and guidance no older than
// guidance.max_age_s, the edge points of this frame's stable lethal marks are appended to the lethal clouds (never
// without a stable lethal point, never removing one) and the tracker's no_return areas join the warning areas; they
// are cell-aligned and stable on their own and skip the PublishedArea continuity. The executor is single-threaded:
// the guidance, cloud, IMU and RGB-D callbacks never run concurrently.
//
// Head-motion gate (task T2, motion.*): the camera angular speed in base_link between the stamp and the stamp minus
// motion.window_s decides per input. A depth cloud above motion.max_head_rate_deg_s still feeds the plane fit (a pan
// turns the cloud about the vertical axis, which leaves a level floor where it is) but does not vote: the temporal
// median, the class hysteresis, the lethal clouds and the warning areas keep their state and publish nothing for it.
// An RGB-D cloud or a guidance pair above the limit is dropped (counted in ground_head_moving_frames_total{input}).
// /ground/state reports head_moving. A rate that cannot be read (no TF history yet) counts as still, as before the gate.
//
// Topics (part ground-topics, names from Config): <ns>/obstacle_points, <ns>/drop_points (PointCloud2 XYZ, reliable
// depth 1), <ns>/warning_areas (std_msgs/String, reliable depth 1), <ns>/labelled_points (PointCloud2 XYZ rgb label,
// best_effort depth 1), <ns>/state (std_msgs/String, reliable transient_local depth 1).
// Metrics (part ground-metrics): see the constants below and GroundNode::GroundNode.
#ifndef GROUND_ESTIMATOR_CPP__GROUND_NODE_HPP_
#define GROUND_ESTIMATOR_CPP__GROUND_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ground_estimator_cpp/cell_class.hpp"
#include "ground_estimator_cpp/cell_median.hpp"
#include "ground_estimator_cpp/colour.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/edge_points.hpp"
#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/ground_classes.hpp"
#include "ground_estimator_cpp/guidance.hpp"
#include "ground_estimator_cpp/no_return.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/state.hpp"
#include "ground_estimator_cpp/surface_slope.hpp"
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
constexpr const char * METRIC_TEMPORAL_GATED = "ground_temporal_gated_cells";
constexpr const char * METRIC_STABLE_LETHAL = "ground_stable_lethal_cells";
constexpr const char * METRIC_STABLE_WARNING = "ground_stable_warning_cells";
constexpr const char * METRIC_OCCLUDED_POINTS = "ground_occluded_points";
constexpr const char * METRIC_MISMATCH_RATIO = "ground_colour_mismatch_ratio";
constexpr const char * METRIC_DROPPED = "ground_clouds_dropped_total";
constexpr const char * METRIC_LABELLED = "ground_labelled_clouds_total";
constexpr const char * METRIC_HEAD_MOVING = "ground_head_moving_frames_total";  // {input}: cloud, rgbd or guidance
constexpr const char * METRIC_HEAD_RATE = "ground_head_rate_deg_s";
constexpr const char * METRIC_HEAD_RATE_UNKNOWN = "ground_head_rate_unknown_total";
// Guidance metrics (part guided-node), registered only while guidance.enabled.
constexpr const char * METRIC_GUIDANCE_FRAMES = "ground_guidance_frames_total";  // {result}: ok or the drop reason
constexpr const char * METRIC_GUIDANCE_SECONDS = "ground_guidance_seconds";
constexpr const char * METRIC_EDGE_POINTS = "ground_edge_points";
constexpr const char * METRIC_NO_RETURN_AREAS = "ground_no_return_areas";
constexpr const char * METRIC_GUIDANCE_STALE = "ground_guidance_stale_frames_total";

// Queue of the guidance ExactTime synchronizer, pairs.
constexpr uint32_t GUIDANCE_QUEUE = 5;

// A lethal point votes its cell lethal only when the temporal cell is steady (CellMedianFilter::steady_at): a spread of
// at most LETHAL_SPREAD_FRACTION * temporal.max_spread_m and no spread gate of it or of its 8 neighbours in its last
// LETHAL_QUIET_WINDOWS * temporal.window_frames frames. Far-band cells of 30..42 mm temporal spread slip under the
// 25 mm gate for runs of frames by chance (an 8-frame estimate scatters ~25 %), and a cell mixing floor and far-band
// returns has a steady median between the two, which the N-of-M class hysteresis cannot absorb; a real obstacle's cells
// read a few millimetres (the 5 cm box top 3-6 mm) next to a steady floor and vote at once.
constexpr double LETHAL_SPREAD_FRACTION = 0.6;
constexpr int LETHAL_QUIET_WINDOWS = 2;

// Largest change of a stable area's height (m) and slope (deg) that keeps its published value.
constexpr double AREA_HEIGHT_DEADBAND_M = 0.01;
constexpr double AREA_SLOPE_DEADBAND_DEG = 1.0;

// result label of ground_fits_total for a valid plane (rejects use to_string(FitReject)).
constexpr const char * FIT_RESULT_OK = "ok";

// reason labels of ground_clouds_dropped_total.
constexpr const char * DROP_REASON_TF = "tf";                        // a cloud without TF to base/fixed frame
constexpr const char * DROP_REASON_NO_PLANE_RGBD = "no_plane_rgbd";  // an RGB-D cloud without a plane at its stamp
constexpr const char * DROP_REASON_NO_MAP_TF = "no_map_tf";          // warning areas without TF to warnings.frame
constexpr const char * DROP_REASON_BAD_INPUT = "bad_input";          // a cloud without float32 x, y, z (or rgb)
constexpr const char * DROP_REASON_GUIDANCE_BAD_INPUT = "guidance_bad_input";  // undecodable pair or no depth model
constexpr const char * DROP_REASON_GUIDANCE_TF = "guidance_tf";                // a pair without TF at its stamp
constexpr const char * DROP_REASON_GUIDANCE_NO_PLANE = "guidance_no_plane";    // a pair without a plane at its stamp

// result label of ground_guidance_frames_total for a pair that was used.
constexpr const char * GUIDANCE_RESULT_OK = "ok";
// result label of a pair not used because the head moved when it was captured (not a drop: nothing is wrong).
constexpr const char * GUIDANCE_RESULT_HEAD_MOVING = "head_moving";

// input labels of ground_head_moving_frames_total.
constexpr const char * INPUT_CLOUD = "cloud";
constexpr const char * INPUT_RGBD = "rgbd";
constexpr const char * INPUT_GUIDANCE = "guidance";

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

  // What the last depth cloud produced, for the state.
  struct Cycle
  {
    std::vector<Region> regions;        // cls from the stable cell classes
    std::vector<WarningArea> warnings;  // the stable warning areas in base_link at the cloud stamp
    std::size_t obstacle_points = 0;
    std::size_t drop_points = 0;
  };

  // The exact bits of a point's x, y, z: the rule's clouds copy the filtered points unchanged, so a lethal candidate
  // finds its own (raw) height of this frame by them.
  struct PointBits
  {
    std::uint32_t x = 0, y = 0, z = 0;
    bool operator==(const PointBits & o) const { return x == o.x && y == o.y && z == o.z; }
  };
  struct PointBitsHash
  {
    std::size_t operator()(const PointBits & b) const;
  };
  static PointBits bits_of(const pcl::PointXYZ & p);

  // A published stable warning area (fixed frame).
  struct PublishedArea
  {
    WarningArea area;
  };

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void on_rgbd(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void on_state_timer();
  // One synchronized guidance pair (part guided-node): counted in ground_guidance_frames_total, timed in
  // ground_guidance_seconds; a failure is a drop and leaves the kept guidance as it was.
  void on_guidance(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & rgb);
  void on_depth_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  // Whether the camera turned faster than motion.max_head_rate_deg_s at `stamp` (the gate is off, or the rate cannot be
  // read: false). Sets the ground_head_rate_deg_s gauge when `gauge` and counts an unreadable rate.
  // @param frame the camera frame the input is stamped in.
  // @param base_T_frame_now base_frame <- frame at the stamp.
  // @param stamp the input stamp.
  // @param gauge whether to publish the rate as the ground_head_rate_deg_s gauge (depth clouds only).
  bool head_moving_at(
    const std::string & frame, const Eigen::Isometry3d & base_T_frame_now, const builtin_interfaces::msg::Time & stamp,
    bool gauge);
  // Decode, move, judge and sample a pair, then keep its samples and vote its no-return cells.
  // @param detail set to the reason of a drop.
  // @return nullptr when the pair was used, else its drop reason (DROP_REASON_GUIDANCE_*).
  const char * use_guidance(
    const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & info,
    const sensor_msgs::msg::CompressedImage & rgb, std::string & detail);
  // @return true when guidance is on and a pair was used no more than guidance.max_age_s from stamp_ns.
  bool guidance_fresh(int64_t stamp_ns) const;
  // Append the edge points of this frame's stable lethal marks (stable_obstacle_, stable_drop_) to them when the kept
  // guidance points are fresh and guidance.edge.enabled; sets ground_edge_points while guidance is on.
  void append_edge_points(const Eigen::Isometry3d & fixed_T_base, int64_t stamp_ns);

  void process_plane(
    const sensor_msgs::msg::PointCloud2 & msg, const TrackedPlane & tracked, const Eigen::Isometry3d & fixed_T_base,
    const Eigen::Vector3d & sensor_base, double stamp_s, const std::chrono::steady_clock::time_point & started);
  // Removes the points in an occlusion shadow from filtered_cloud_ (kept in occluded_points_), sets floor_reference_ to
  // what is left (points and plane-relative heights) and the ground_occluded_points gauge.
  void drop_occluded(const Plane & plane, const Eigen::Vector3d & sensor_base);
  // Split base_cloud_ by raw plane-relative height: below_robot_ (at most traversability.robot_height_m, the input of
  // the temporal median and the rule) and overhang_points_ (above it: never an obstacle, task T3).
  void split_overhang(const Plane & plane);
  // Publish /ground/classified_points (only with a subscriber): this frame's measured points within the band range at
  // their own positions, in the fixed frame, each with its GroundClass (ground_classes.hpp).
  void publish_classified(
    const Plane & plane, const Eigen::Isometry3d & fixed_T_base, const builtin_interfaces::msg::Time & stamp);
  void refine_slopes(std::vector<Cluster> & clusters, double sign, const Eigen::Isometry3d & fixed_T_base) const;
  void observe_classes(const Traversability & rule, const Eigen::Isometry3d & fixed_T_base, double stamp_s);
  // The lethal marks of one side of the floor (lethal_marks): the points of `lethal` and `warning` (this frame's rule
  // output) in stable lethal cells that clear the side's step limit (limit_m) by their own height and the cell's
  // median, with company.
  void lethal_marks_of(
    const pcl::PointCloud<pcl::PointXYZ> & lethal, const pcl::PointCloud<pcl::PointXYZ> & warning, const Plane & plane,
    const Eigen::Isometry3d & fixed_T_base, double limit_m, pcl::PointCloud<pcl::PointXYZ> & out);
  void observe_steady(
    const pcl::PointCloud<pcl::PointXYZ> & points, CellClass cls, const Eigen::Isometry3d & fixed_T_base);
  void publish_warning_areas(const builtin_interfaces::msg::Time & stamp, const Plane & plane_fixed);
  std::vector<PublishedArea> stable_areas_fixed() const;
  void reset_cells();
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
  CellClassFilter cell_class_;                // on the cell_filter_ grid; reset with it
  RawHeightHistory raw_heights_;              // the raw clouds of the slope history, for the cluster slopes
  std::vector<PublishedArea> areas_fixed_;    // the published stable warning areas
  std::vector<WarningArea> no_return_fixed_;  // the published no_return areas (fixed frame)
  FloorColourModel colour_model_;
  std::optional<ImuSample> latest_imu_;
  std::optional<Eigen::Quaterniond> imu_to_base_;  // static rotation of IMU-frame vectors into base_frame
  std::string imu_frame_;                          // the frame imu_to_base_ was looked up for
  Prior prior_;                                    // of the last depth cloud
  std::size_t last_inliers_ = 0;
  double last_inlier_ratio_ = 0.0;
  std::optional<FitReject> last_reject_;
  const char * last_reason_ = nullptr;  // REASON_NO_TF after a TF drop
  bool have_cloud_ = false;
  std::optional<bool> last_state_valid_;
  bool head_moving_ = false;               // the last depth cloud was gated
  std::optional<bool> last_state_moving_;  // head_moving of the last state document
  Cycle cycle_;
  std::optional<ColourFrame> colour_;
  std::optional<ColourInputs> colour_inputs_;

  // Reused between clouds (the 10 Hz path keeps its capacity).
  pcl::PointCloud<pcl::PointXYZ> base_cloud_;
  pcl::PointCloud<pcl::PointXYZ> below_robot_;        // base_cloud_ at most the robot height above the plane
  pcl::PointCloud<pcl::PointXYZ> overhang_points_;    // base_cloud_ above the robot height (classified only)
  pcl::PointCloud<pcl::PointXYZ> occluded_points_;    // filtered points removed by the shadow gate (classified only)
  pcl::PointCloud<pcl::PointXYZ> classified_points_;  // scratch of publish_classified (fixed frame)
  std::vector<GroundClass> classes_;                  // scratch of publish_classified
  FloorReference floor_reference_;                    // filtered_cloud_ with plane-relative heights
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud_;     // base_cloud_ after the temporal median
  std::vector<float> filtered_raw_h_;                 // own height of each filtered_cloud_ point before the median
  std::unordered_map<PointBits, float, PointBitsHash> raw_height_of_;  // filtered point -> its own height
  pcl::PointCloud<pcl::PointXYZ> lethal_candidates_;                   // scratch of lethal_marks_of
  std::vector<float> candidate_filtered_h_, candidate_raw_h_;
  pcl::PointCloud<pcl::PointXYZ> unclustered_above_, unclustered_below_;
  std::vector<float> unclustered_above_h_, unclustered_below_h_;
  std::vector<int> unclustered_index_;
  pcl::PointCloud<pcl::PointXYZRGB> rgbd_cloud_;
  std::vector<Label> labels_;
  pcl::PointCloud<pcl::PointXYZ> mismatch_cloud_;
  std::vector<float> mismatch_h_;
  std::vector<WarningArea> areas_in_frame_;
  // This frame's lethal marks (lethal_marks_of), then plus the guidance edge points (append_edge_points).
  pcl::PointCloud<pcl::PointXYZ> stable_obstacle_, stable_drop_;
  pcl::PointCloud<pcl::PointXYZ> steady_lethal_;  // lethal points of steady cells (scratch)

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rgbd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr drop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr labelled_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr classified_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warnings_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // Guidance (only while guidance.enabled).
  using GuidancePolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CompressedImage>;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> rgb_sub_;
  std::unique_ptr<message_filters::Synchronizer<GuidancePolicy>> guidance_sync_;  // after its subscribers
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  std::optional<CameraModel> depth_model_;  // of the latest depth camera info with a usable calibration
  std::string depth_frame_;                 // its frame
  RayGrid guidance_grid_;
  GuidancePoints guidance_points_;      // the last good guidance per cell, expired at guidance.max_age_s
  NoReturnTracker no_return_;           // on the cell_filter_ grid; reset with it
  std::optional<int64_t> guidance_ns_;  // stamp of the latest pair used
  prometheus::Histogram * guidance_seconds_ = nullptr;

  ros2_common_cpp::Metrics metrics_;
  prometheus::Histogram * fit_seconds_ = nullptr;
  prometheus::Counter * labelled_ = nullptr;
  // Gauge families registered without a series: a gauge appears with its first real value.
  std::unordered_map<std::string, prometheus::Family<prometheus::Gauge> *> gauges_;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__GROUND_NODE_HPP_
