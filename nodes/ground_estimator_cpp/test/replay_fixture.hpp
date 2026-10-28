// Part replay-harness (ground-detection-robustness design D14): the reader of the replay binaries in
// test/data/replay (written by scripts/ground_replay_export.py) and the replay metrics over what GroundNode published
// per cloud.
//
// Binary layout, little-endian, versions 1 and 2 (scripts/ground_replay_export.py module docstring):
//   magic "GRNDRPLY" (8 bytes), uint32 version, uint32 frame_len + frame_len bytes of the cloud frame id,
//   uint32 cloud_count, per cloud: int64 stamp_ns, float64[7] base_T_cloud, float64[7] odom_T_base (x y z qx qy qz qw),
//   uint32 point_count, int16[point_count * 3] x y z mm in the cloud frame;
//   uint32 imu_count, per sample: int64 stamp_ns, float64[10] orientation x y z w, linear_acceleration x y z,
//   angular_velocity x y z (frame imu_link); float64[7] base_T_imu.
// Version 2 appends the guidance section: the RGB and D435 depth camera models (float64[9] K, float64[5] D,
// uint32 width, uint32 height, uint32 frame_len + frame id), uint32 pair_count, per pair int64 stamp_ns,
// float64[7] base_T_rgb, float64[7] odom_T_base, uint32 jpeg_len + JPEG bytes, uint32 png_len + PNG bytes (16-bit
// depth, mm).
//
// Metrics (D14, base_link, every one ignores points and areas with x < GRIPPER_X_MAX, the robot's parked gripper):
// the frames counted are those at least WARM_UP_S after the first frame that the node answered with a plane.
//   plane_share              answered frames / frames after the warm-up
//   box_lethal_share         share of answered frames with an obstacle point in BOX_LETHAL_ROI
//   box_warning_share        share of answered frames with a depth step warning area whose polygon centroid (vertex
//                            mean) lies in BOX_LETHAL_ROI (the 5 cm box under the temporary 60 mm lethal step)
//   board_lethal_frames      answered frames with an obstacle or drop point in BOARD_ROI
//   stray_lethal_share       share of answered frames with an obstacle or drop point in the stray zone: x >= 0.37,
//                            hypot(x, y) <= STRAY_RANGE_M, outside BOX_LETHAL_ROI
//   far_side_warning_frames  answered frames with a ramp or step warning area whose polygon centroid (vertex mean) lies
//                            in FAR_SIDE_ROI
//   ramp_lethal_frames       answered frames with an obstacle or drop point in RAMP_ROI
//   churn_share              share of consecutive answered frames whose warning set (kind plus polygon vertices
//                            rounded to 1 cm, scripts/ground_noise_report.py:warning_set) differs
// Shares are NaN when no frame is counted. ROI edges are inclusive.
//
// Truth based metrics (part guidance-replay-harness, design D9, D10; SceneTruth from scene-truth, the polygons of
// scripts/ground_scene_truth.py). The overloads of the run 4 names below take their ROI from the truth instead of the
// constants above; every other rule (frames counted, x < GRIPPER_X_MAX ignored, which points) is the same:
//   box_lethal_share(frames, truth)       obstacle point within BOX_GROW_M of the box footprint
//   board_lethal_frames(frames, truth)    obstacle or drop point inside the board polygon shrunk by BOARD_SHRINK_M
//   stray_lethal_share(frames, truth)     obstacle or drop point, hypot <= STRAY_RANGE_M, outside the grown box
//   ramp_lethal_frames(frames, truth)     obstacle or drop point inside the ramp footprint
//   empty_lethal_share                    share of frames with an obstacle point inside the empty rectangle
//   box_front_edge_error_m                p95, over the samples of all frames, of the distance along the front edge
//                                         normal to the nearest obstacle point near the box (EDGE_NEAR_M of the
//                                         footprint); samples every EDGE_SAMPLE_STEP_M over the middle EDGE_MIDDLE_SHARE
//                                         of the edge; a sample with no point within EDGE_LINE_TOL_M of its line along
//                                         the edge counts EDGE_NEAR_M
//   box_width_error_m                     p95 of |extent of those points along the edge - true width| (a frame without
//                                         such a point has extent 0)
//   outside_lethal_frames_share           frames with an obstacle point between OUTSIDE_MIN_M and OUTSIDE_MAX_M outside
//                                         the box footprint
//   no_return_cover_share                 frames in which the "no_return" areas cover >= COVER_MIN_SHARE of the ramp
//                                         footprint (0.01 m raster, RASTER_M)
//   no_return_stray_share                 frames with a "no_return" raster cell centre more than STRAY_AREA_M outside
//                                         the ramp footprint
#ifndef REPLAY_FIXTURE_HPP_
#define REPLAY_FIXTURE_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ground_replay
{

constexpr char REPLAY_MAGIC[] = "GRNDRPLY";
constexpr std::size_t REPLAY_MAGIC_SIZE = 8;
constexpr uint32_t REPLAY_VERSION = 1;        // clouds, IMU and base_T_imu
constexpr uint32_t REPLAY_VERSION_PAIRS = 2;  // plus the guidance section: camera models and RGB-D pairs
constexpr int CAMERA_K_FIELDS = 9;
constexpr int CAMERA_D_FIELDS = 5;
constexpr int POSE_FIELDS = 7;
constexpr int IMU_VALUES = 10;

// Axis-aligned rectangle in base_link metres, edges inclusive.
struct Roi
{
  double x_min, x_max, y_min, y_max;
  bool contains(double x, double y) const { return x >= x_min && x <= x_max && y >= y_min && y <= y_max; }
};

// D14 ROIs: the measured ground truth of the 2026-10-22 scene (brief ground-live.md).
constexpr Roi BOX_LETHAL_ROI{0.57, 0.81, 0.02, 0.36};  // box top x 0.60-0.78, y 0.05-0.33 dilated 3 cm
constexpr Roi BOARD_ROI{0.37, 0.57, -0.12, 0.14};      // the board, stopping 3 cm before the box front
constexpr Roi FAR_SIDE_ROI{0.78, 0.95, 0.05, 0.33};    // the occluded floor behind the box
constexpr Roi RAMP_ROI{0.75, 1.10, -0.45, -0.15};      // the black rubber ramp
constexpr double GRIPPER_X_MAX = 0.37;                 // x below this is the parked gripper: ignored everywhere
constexpr double STRAY_RANGE_M = 1.2;                  // stray zone radius around base_link
constexpr double WARM_UP_S = 2.0;                      // frames this soon after the first are not counted
constexpr double WARNING_ROUND_PER_M = 100.0;          // warning set vertices rounded to 1 cm
constexpr const char * FAR_SIDE_KINDS[] = {"ramp", "step"};
constexpr const char * NO_RETURN_KIND = "no_return";
constexpr double BOX_GROW_M = 0.03;      // truth box lethal ROI: the footprint grown by this
constexpr double BOARD_SHRINK_M = 0.02;  // truth board ROI: the polygon shrunk by this
constexpr double EDGE_NEAR_M = 0.10;     // points this near the box footprint belong to the box; also the miss value
constexpr double EDGE_SAMPLE_STEP_M = 0.02;  // spacing of the front edge samples
constexpr double EDGE_MIDDLE_SHARE = 0.80;   // the middle share of the edge that is sampled
constexpr double EDGE_LINE_TOL_M = 0.02;     // a sample's line takes points this near along the edge
constexpr double OUTSIDE_MIN_M = 0.03;       // outside lethal: farther than this from the box ...
constexpr double OUTSIDE_MAX_M = 0.15;       // ... and not farther than this
constexpr double COVER_MIN_SHARE = 0.70;     // no-return cover: share of the ramp footprint
constexpr double RASTER_M = 0.01;            // grid of the no-return rasterisation
constexpr double STRAY_AREA_M = 0.10;        // a no-return cell this far outside the ramp is stray
constexpr double EDGE_P95 = 0.95;
constexpr double EDGE_EPS = 1e-9;
constexpr int MIN_POLYGON_VERTICES = 3;
constexpr int EDGE_VERTICES = 2;

struct ReplayCloud
{
  int64_t stamp_ns = 0;
  Eigen::Isometry3d base_T_cloud = Eigen::Isometry3d::Identity();  // base_link <- cloud frame at the stamp
  Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();   // odom <- base_link at the stamp
  std::vector<std::array<int16_t, 3>> points_mm;                   // x y z in the cloud frame, mm
};

struct ReplayImu
{
  int64_t stamp_ns = 0;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

// A camera model of the guidance section.
struct CameraModel
{
  std::array<double, CAMERA_K_FIELDS> K{};
  std::array<double, CAMERA_D_FIELDS> D{};
  uint32_t width = 0;
  uint32_t height = 0;
  std::string frame;
};

// One RGB-D pair: the JPEG bytes of the RGB image and the upsampled depth (CV_16UC1, mm).
struct ReplayPair
{
  int64_t stamp_ns = 0;
  Eigen::Isometry3d base_T_rgb = Eigen::Isometry3d::Identity();  // base_link <- RGB frame
  Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
  std::vector<uint8_t> jpeg;
  cv::Mat depth_mm;
};

struct ReplayScene
{
  std::string frame_id;  // frame of the clouds
  std::vector<ReplayCloud> clouds;
  std::vector<ReplayImu> imu;
  Eigen::Isometry3d base_T_imu = Eigen::Isometry3d::Identity();  // base_link <- imu_link
  uint32_t version = REPLAY_VERSION;
  std::optional<CameraModel> rgb_model;    // empty for version 1
  std::optional<CameraModel> depth_model;  // empty for version 1
  std::vector<ReplayPair> pairs;           // empty for version 1
};
using Replay = ReplayScene;

// One area of the /ground/warning_areas document: polygon vertices in the document's frame; the heights are null for
// the areas that have none (no_return).
struct WarningArea
{
  std::string kind;
  std::vector<Eigen::Vector2d> polygon;
  std::optional<double> height_m;
  std::optional<double> slope_deg;
  std::string source;
};

// What GroundNode published for one cloud.
struct ReplayFrame
{
  int64_t stamp_ns = 0;
  std::vector<Eigen::Vector3f> obstacle;  // /ground/obstacle_points of the stamp, base_link
  std::vector<Eigen::Vector3f> drop;      // /ground/drop_points of the stamp, base_link
  nlohmann::json warnings;                // the /ground/warning_areas document of the stamp (null when none)
  std::vector<WarningArea> areas;         // the areas of that document, parsed (parse_areas)
  Eigen::Isometry3d base_T_warnings = Eigen::Isometry3d::Identity();  // base_link <- the document's frame
  bool has_plane = false;                                             // the node answered with a plane
};

// Sequential little-endian reader over a byte string; throws std::runtime_error past the end.
class ByteReader
{
public:
  explicit ByteReader(const std::string & bytes) : bytes_(bytes) {}

  // @param n bytes to take.
  // @return the next n bytes.
  std::string take(std::size_t n)
  {
    if (n > bytes_.size() - at_) {
      throw std::runtime_error(
        "truncated ground replay file: need " + std::to_string(n) + " bytes at " + std::to_string(at_) + ", have " +
        std::to_string(bytes_.size() - at_));
    }
    std::string out = bytes_.substr(at_, n);
    at_ += n;
    return out;
  }

  uint64_t unsigned_le(std::size_t n)
  {
    const std::string raw = take(n);
    uint64_t v = 0;
    for (std::size_t i = 0; i < n; ++i) {
      v |= static_cast<uint64_t>(static_cast<unsigned char>(raw[i])) << (8 * i);
    }
    return v;
  }

  uint32_t u32() { return static_cast<uint32_t>(unsigned_le(4)); }
  int64_t i64() { return static_cast<int64_t>(unsigned_le(8)); }
  int16_t i16() { return static_cast<int16_t>(static_cast<uint16_t>(unsigned_le(2))); }
  double f64()
  {
    const uint64_t bits = unsigned_le(8);
    double v = 0.0;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
  }

  // x y z qx qy qz qw as an isometry.
  Eigen::Isometry3d pose()
  {
    std::array<double, POSE_FIELDS> f{};
    for (double & v : f) {
      v = f64();
    }
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(f[0], f[1], f[2]);
    t.linear() = Eigen::Quaterniond(f[6], f[3], f[4], f[5]).normalized().toRotationMatrix();
    return t;
  }

  std::size_t remaining() const { return bytes_.size() - at_; }

private:
  const std::string & bytes_;
  std::size_t at_ = 0;
};

// @param in reader at a camera model of the guidance section.
// @return the model.
inline CameraModel read_camera_model(ByteReader & in)
{
  CameraModel m;
  for (double & v : m.K) {
    v = in.f64();
  }
  for (double & v : m.D) {
    v = in.f64();
  }
  m.width = in.u32();
  m.height = in.u32();
  m.frame = in.take(in.u32());
  return m;
}

// @param in reader at a pair of the guidance section.
// @param index the pair's index, for the error message.
// @return the pair, its depth decoded from the PNG.
// @throws std::runtime_error on a truncated pair or a PNG that does not decode to CV_16UC1.
inline ReplayPair read_pair(ByteReader & in, uint32_t index)
{
  ReplayPair pair;
  pair.stamp_ns = in.i64();
  pair.base_T_rgb = in.pose();
  pair.odom_T_base = in.pose();
  const std::string jpeg = in.take(in.u32());
  pair.jpeg.assign(jpeg.begin(), jpeg.end());
  const std::string png = in.take(in.u32());
  pair.depth_mm = cv::imdecode(std::vector<uint8_t>(png.begin(), png.end()), cv::IMREAD_UNCHANGED);
  if (pair.depth_mm.empty() || pair.depth_mm.type() != CV_16UC1) {
    throw std::runtime_error(
      "ground replay pair " + std::to_string(index) + ": depth PNG is not 16-bit single channel");
  }
  return pair;
}

// Decode a replay binary (version 1 or 2).
// @param bytes the file content.
// @return the scene; rgb_model, depth_model and pairs are empty for version 1.
// @throws std::runtime_error on a wrong magic or an unknown version, a truncated section, a PNG that does not decode
//         to 16-bit, or trailing bytes.
inline ReplayScene decode_replay(const std::string & bytes)
{
  ByteReader in(bytes);
  if (bytes.size() < REPLAY_MAGIC_SIZE || bytes.compare(0, REPLAY_MAGIC_SIZE, REPLAY_MAGIC) != 0) {
    throw std::runtime_error("not a ground replay file: wrong magic");
  }
  in.take(REPLAY_MAGIC_SIZE);
  const uint32_t version = in.u32();
  if (version != REPLAY_VERSION && version != REPLAY_VERSION_PAIRS) {
    throw std::runtime_error(
      "unsupported ground replay version " + std::to_string(version) + ", expected " + std::to_string(REPLAY_VERSION) +
      " or " + std::to_string(REPLAY_VERSION_PAIRS));
  }
  ReplayScene scene;
  scene.version = version;
  scene.frame_id = in.take(in.u32());
  const uint32_t clouds = in.u32();
  for (uint32_t c = 0; c < clouds; ++c) {
    ReplayCloud cloud;
    cloud.stamp_ns = in.i64();
    cloud.base_T_cloud = in.pose();
    cloud.odom_T_base = in.pose();
    const uint32_t count = in.u32();
    if (static_cast<uint64_t>(count) * 6 > in.remaining()) {
      throw std::runtime_error("truncated ground replay file: cloud " + std::to_string(c));
    }
    cloud.points_mm.resize(count);
    for (auto & p : cloud.points_mm) {
      p = {in.i16(), in.i16(), in.i16()};
    }
    scene.clouds.push_back(std::move(cloud));
  }
  const uint32_t samples = in.u32();
  for (uint32_t s = 0; s < samples; ++s) {
    ReplayImu imu;
    imu.stamp_ns = in.i64();
    std::array<double, IMU_VALUES> v{};
    for (double & x : v) {
      x = in.f64();
    }
    imu.orientation = Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
    imu.linear_acceleration = Eigen::Vector3d(v[4], v[5], v[6]);
    imu.angular_velocity = Eigen::Vector3d(v[7], v[8], v[9]);
    scene.imu.push_back(imu);
  }
  scene.base_T_imu = in.pose();
  if (version == REPLAY_VERSION_PAIRS) {
    scene.rgb_model = read_camera_model(in);
    scene.depth_model = read_camera_model(in);
    const uint32_t pairs = in.u32();
    for (uint32_t p = 0; p < pairs; ++p) {
      scene.pairs.push_back(read_pair(in, p));
    }
  }
  if (in.remaining() != 0) {
    throw std::runtime_error("ground replay file has " + std::to_string(in.remaining()) + " trailing bytes");
  }
  return scene;
}

// Read a replay binary.
// @param path the .bin file.
// @return the scene.
// @throws std::runtime_error when the file cannot be read or does not decode.
inline ReplayScene read_replay(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("cannot read ground replay file " + path);
  }
  return decode_replay(std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()));
}

// --- metrics -----------------------------------------------------------------------------------------------------

// The frames the metrics count: at least WARM_UP_S after the first frame and answered with a plane.
inline std::vector<const ReplayFrame *> counted_frames(const std::vector<ReplayFrame> & frames)
{
  std::vector<const ReplayFrame *> out;
  if (frames.empty()) {
    return out;
  }
  const int64_t first = frames.front().stamp_ns;
  for (const auto & f : frames) {
    if (f.has_plane && static_cast<double>(f.stamp_ns - first) * 1e-9 >= WARM_UP_S) {
      out.push_back(&f);
    }
  }
  return out;
}

// True when any point outside the gripper zone passes pred(x, y).
template <typename Pred>
bool any_point(const std::vector<Eigen::Vector3f> & points, Pred pred)
{
  for (const auto & p : points) {
    if (p.x() >= GRIPPER_X_MAX && pred(static_cast<double>(p.x()), static_cast<double>(p.y()))) {
      return true;
    }
  }
  return false;
}

template <typename Pred>
bool any_lethal(const ReplayFrame & f, Pred pred)
{
  return any_point(f.obstacle, pred) || any_point(f.drop, pred);
}

template <typename Pred>
int count_frames(const std::vector<ReplayFrame> & frames, Pred pred)
{
  int n = 0;
  for (const ReplayFrame * f : counted_frames(frames)) {
    n += pred(*f) ? 1 : 0;
  }
  return n;
}

template <typename Pred>
double share_of_frames(const std::vector<ReplayFrame> & frames, Pred pred)
{
  const auto counted = counted_frames(frames);
  return counted.empty() ? std::nan("") : static_cast<double>(count_frames(frames, pred)) / counted.size();
}

// The areas of a warning document (empty for a null document).
inline nlohmann::json areas_of(const ReplayFrame & f)
{
  return f.warnings.is_object() && f.warnings.contains("areas") ? f.warnings["areas"] : nlohmann::json::array();
}

// The vertex mean of an area's polygon in base_link.
inline Eigen::Vector2d centroid_in_base(const nlohmann::json & area, const Eigen::Isometry3d & base_T_warnings)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  const auto & polygon = area["polygon"];
  for (const auto & v : polygon) {
    sum += Eigen::Vector3d(v[0].get<double>(), v[1].get<double>(), 0.0);
  }
  const Eigen::Vector3d c = base_T_warnings * (polygon.empty() ? sum : sum / static_cast<double>(polygon.size()));
  return c.head<2>();
}

// One area of a warning set: kind plus the polygon vertices in cm.
using WarningKey = std::pair<std::string, std::vector<std::pair<long long, long long>>>;

// The warning set of a frame (scripts/ground_noise_report.py:warning_set), without areas centred in the gripper zone.
inline std::set<WarningKey> warning_set(const ReplayFrame & f)
{
  std::set<WarningKey> out;
  for (const auto & a : areas_of(f)) {
    if (centroid_in_base(a, f.base_T_warnings).x() < GRIPPER_X_MAX) {
      continue;
    }
    WarningKey key{a["kind"].get<std::string>(), {}};
    for (const auto & v : a["polygon"]) {
      key.second.emplace_back(
        std::llround(v[0].get<double>() * WARNING_ROUND_PER_M), std::llround(v[1].get<double>() * WARNING_ROUND_PER_M));
    }
    out.insert(std::move(key));
  }
  return out;
}

// @param frames every frame of a replay, in order.
// @return answered frames / frames after the warm-up (NaN without frames after it).
inline double plane_share(const std::vector<ReplayFrame> & frames)
{
  if (frames.empty()) {
    return std::nan("");
  }
  int after = 0;
  for (const auto & f : frames) {
    after += static_cast<double>(f.stamp_ns - frames.front().stamp_ns) * 1e-9 >= WARM_UP_S ? 1 : 0;
  }
  return after == 0 ? std::nan("") : static_cast<double>(counted_frames(frames).size()) / after;
}

// @return the share of counted frames with an obstacle point in BOX_LETHAL_ROI.
inline double box_lethal_share(const std::vector<ReplayFrame> & frames)
{
  return share_of_frames(frames, [](const ReplayFrame & f) {
    return any_point(f.obstacle, [](double x, double y) { return BOX_LETHAL_ROI.contains(x, y); });
  });
}

// @return the share of counted frames with a depth step warning area centred in BOX_LETHAL_ROI.
inline double box_warning_share(const std::vector<ReplayFrame> & frames)
{
  return share_of_frames(frames, [](const ReplayFrame & f) {
    for (const auto & a : areas_of(f)) {
      const Eigen::Vector2d c = centroid_in_base(a, f.base_T_warnings);
      const bool depth_step = a["kind"] == "step" && a["source"] == "depth";
      if (depth_step && c.x() >= GRIPPER_X_MAX && BOX_LETHAL_ROI.contains(c.x(), c.y())) {
        return true;
      }
    }
    return false;
  });
}

// @return the counted frames with an obstacle or drop point in BOARD_ROI.
inline int board_lethal_frames(const std::vector<ReplayFrame> & frames)
{
  return count_frames(frames, [](const ReplayFrame & f) {
    return any_lethal(f, [](double x, double y) { return BOARD_ROI.contains(x, y); });
  });
}

// @return the share of counted frames with an obstacle or drop point within STRAY_RANGE_M outside BOX_LETHAL_ROI.
inline double stray_lethal_share(const std::vector<ReplayFrame> & frames)
{
  return share_of_frames(frames, [](const ReplayFrame & f) {
    return any_lethal(
      f, [](double x, double y) { return std::hypot(x, y) <= STRAY_RANGE_M && !BOX_LETHAL_ROI.contains(x, y); });
  });
}

// @return the counted frames with a ramp or step warning area centred in FAR_SIDE_ROI.
inline int far_side_warning_frames(const std::vector<ReplayFrame> & frames)
{
  return count_frames(frames, [](const ReplayFrame & f) {
    for (const auto & a : areas_of(f)) {
      const std::string kind = a["kind"].get<std::string>();
      const bool far_kind = kind == FAR_SIDE_KINDS[0] || kind == FAR_SIDE_KINDS[1];
      const Eigen::Vector2d c = centroid_in_base(a, f.base_T_warnings);
      if (far_kind && c.x() >= GRIPPER_X_MAX && FAR_SIDE_ROI.contains(c.x(), c.y())) {
        return true;
      }
    }
    return false;
  });
}

// @return the counted frames with an obstacle or drop point in RAMP_ROI.
inline int ramp_lethal_frames(const std::vector<ReplayFrame> & frames)
{
  return count_frames(frames, [](const ReplayFrame & f) {
    return any_lethal(f, [](double x, double y) { return RAMP_ROI.contains(x, y); });
  });
}

// @return the share of consecutive counted frames whose warning sets differ (NaN under two counted frames).
inline double churn_share(const std::vector<ReplayFrame> & frames)
{
  const auto counted = counted_frames(frames);
  if (counted.size() < 2) {
    return std::nan("");
  }
  int changed = 0;
  for (std::size_t i = 1; i < counted.size(); ++i) {
    changed += warning_set(*counted[i]) != warning_set(*counted[i - 1]) ? 1 : 0;
  }
  return static_cast<double>(changed) / (counted.size() - 1);
}

// @param scene the scene name.
// @param frames every frame of the scene's replay.
// @return "replay <scene>: plane_share=... box_lethal_share=... board_lethal_frames=... stray_lethal_share=...
//         far_side_warning_frames=... ramp_lethal_frames=... churn_share=..." (shares with 3 decimals).
inline std::string metrics_line(const std::string & scene, const std::vector<ReplayFrame> & frames)
{
  char buf[512];
  std::snprintf(
    buf, sizeof(buf),
    "replay %s: plane_share=%.3f box_lethal_share=%.3f board_lethal_frames=%d stray_lethal_share=%.3f "
    "far_side_warning_frames=%d ramp_lethal_frames=%d churn_share=%.3f",
    scene.c_str(), plane_share(frames), box_lethal_share(frames), board_lethal_frames(frames),
    stray_lethal_share(frames), far_side_warning_frames(frames), ramp_lethal_frames(frames), churn_share(frames));
  return buf;
}

// --- parsed warning areas ----------------------------------------------------------------------------------------

// @param doc a /ground/warning_areas document (null or without "areas": none).
// @return its areas; a null or missing height_m / slope_deg stays empty, a missing source is "".
inline std::vector<WarningArea> parse_areas(const nlohmann::json & doc)
{
  std::vector<WarningArea> out;
  if (!doc.is_object() || !doc.contains("areas")) {
    return out;
  }
  const auto number = [](const nlohmann::json & a, const char * key) {
    return a.contains(key) && a[key].is_number() ? std::optional<double>(a[key].get<double>()) : std::nullopt;
  };
  for (const auto & a : doc["areas"]) {
    WarningArea w;
    w.kind = a["kind"].get<std::string>();
    for (const auto & v : a["polygon"]) {
      w.polygon.emplace_back(v[0].get<double>(), v[1].get<double>());
    }
    w.height_m = number(a, "height_m");
    w.slope_deg = number(a, "slope_deg");
    w.source = a.contains("source") && a["source"].is_string() ? a["source"].get<std::string>() : std::string();
    out.push_back(std::move(w));
  }
  return out;
}

// --- scene truth -------------------------------------------------------------------------------------------------

using Polygon = std::vector<Eigen::Vector2d>;

// The scene truth in base_link metres (scripts/ground_scene_truth.py): polygons as vertex lists, the box front edge
// as its two end points, the empty floor as a rectangle.
struct SceneTruth
{
  Polygon board, box, ramp;
  std::array<Eigen::Vector2d, EDGE_VERTICES> box_front_edge;
  Roi empty{0.0, 0.0, 0.0, 0.0};
};

// @param node a YAML node that should be a sequence of n [x, y] pairs (n >= min_count; exactly n when exact).
// @param key the key, for the error message.
inline Polygon read_points(const YAML::Node & node, const std::string & key, int min_count, bool exact)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("scene truth: key '" + key + "' is missing or not a list of [x, y] points");
  }
  Polygon out;
  for (const auto & v : node) {
    if (!v.IsSequence() || v.size() != 2) {
      throw std::runtime_error("scene truth: key '" + key + "' holds an entry that is not an [x, y] pair");
    }
    out.emplace_back(v[0].as<double>(), v[1].as<double>());
  }
  if (static_cast<int>(out.size()) < min_count || (exact && static_cast<int>(out.size()) != min_count)) {
    throw std::runtime_error(
      "scene truth: key '" + key + "' needs " + (exact ? "exactly " : "at least ") + std::to_string(min_count) +
      " points, has " + std::to_string(out.size()));
  }
  return out;
}

// @param path the truth YAML (scripts/ground_scene_truth.py output).
// @return board, box, ramp, box_front_edge and empty.
// @throws std::runtime_error when the file cannot be read or parsed, or any of the keys board, box, ramp,
//         box_front_edge, empty (x_min x_max y_min y_max) is missing or malformed.
inline SceneTruth load_truth(const std::string & path)
{
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot read scene truth " + path + ": " + e.what());
  }
  if (!doc.IsMap()) {
    throw std::runtime_error("scene truth " + path + " is not a mapping");
  }
  SceneTruth t;
  t.board = read_points(doc["board"], "board", MIN_POLYGON_VERTICES, false);
  t.box = read_points(doc["box"], "box", MIN_POLYGON_VERTICES, false);
  t.ramp = read_points(doc["ramp"], "ramp", MIN_POLYGON_VERTICES, false);
  const Polygon edge = read_points(doc["box_front_edge"], "box_front_edge", EDGE_VERTICES, true);
  t.box_front_edge = {edge[0], edge[1]};
  const YAML::Node empty = doc["empty"];
  if (!empty || !empty.IsMap()) {
    throw std::runtime_error("scene truth: key 'empty' is missing or not a mapping");
  }
  const auto bound = [&](const char * key) {
    if (!empty[key]) {
      throw std::runtime_error(std::string("scene truth: key 'empty.") + key + "' is missing");
    }
    return empty[key].as<double>();
  };
  t.empty = Roi{bound("x_min"), bound("x_max"), bound("y_min"), bound("y_max")};
  return t;
}

// --- polygon geometry --------------------------------------------------------------------------------------------

// @return the distance from p to the segment a-b.
inline double segment_distance(const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double len2 = ab.squaredNorm();
  const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
  return (p - (a + t * ab)).norm();
}

// @return true when p lies inside the polygon (even-odd rule).
inline bool inside_polygon(const Eigen::Vector2d & p, const Polygon & poly)
{
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const bool crosses = (poly[i].y() > p.y()) != (poly[j].y() > p.y());
    if (
      crosses &&
      p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()) {
      in = !in;
    }
  }
  return in;
}

// @return the distance from p to the polygon outline, negative when p is inside.
inline double signed_distance(const Eigen::Vector2d & p, const Polygon & poly)
{
  double d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    d = std::min(d, segment_distance(p, poly[j], poly[i]));
  }
  return inside_polygon(p, poly) ? -d : d;
}

// @return the polygon of an area in base_link.
inline Polygon area_in_base(const WarningArea & a, const Eigen::Isometry3d & base_T_warnings)
{
  Polygon out;
  for (const auto & v : a.polygon) {
    out.push_back((base_T_warnings * Eigen::Vector3d(v.x(), v.y(), 0.0)).head<2>());
  }
  return out;
}

// --- truth based metrics -----------------------------------------------------------------------------------------

// The obstacle points of a frame outside the gripper zone whose signed distance to the polygon passes pred.
template <typename Pred>
std::vector<Eigen::Vector2d> obstacle_xy_where(const ReplayFrame & f, const Polygon & poly, Pred pred)
{
  std::vector<Eigen::Vector2d> out;
  for (const auto & p : f.obstacle) {
    const Eigen::Vector2d xy(p.x(), p.y());
    if (p.x() >= GRIPPER_X_MAX && pred(signed_distance(xy, poly))) {
      out.push_back(xy);
    }
  }
  return out;
}

// @return the share of counted frames with an obstacle point within BOX_GROW_M of the truth box footprint.
inline double box_lethal_share(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return share_of_frames(frames, [&](const ReplayFrame & f) {
    return !obstacle_xy_where(f, truth.box, [](double d) { return d <= BOX_GROW_M; }).empty();
  });
}

// @return the counted frames with an obstacle or drop point inside the truth board shrunk by BOARD_SHRINK_M.
inline int board_lethal_frames(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return count_frames(frames, [&](const ReplayFrame & f) {
    return any_lethal(
      f, [&](double x, double y) { return signed_distance(Eigen::Vector2d(x, y), truth.board) <= -BOARD_SHRINK_M; });
  });
}

// @return the share of counted frames with an obstacle or drop point within STRAY_RANGE_M outside the grown box.
inline double stray_lethal_share(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return share_of_frames(frames, [&](const ReplayFrame & f) {
    return any_lethal(f, [&](double x, double y) {
      return std::hypot(x, y) <= STRAY_RANGE_M && signed_distance(Eigen::Vector2d(x, y), truth.box) > BOX_GROW_M;
    });
  });
}

// @return the counted frames with an obstacle or drop point inside the truth ramp footprint.
inline int ramp_lethal_frames(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return count_frames(frames, [&](const ReplayFrame & f) {
    return any_lethal(f, [&](double x, double y) { return inside_polygon(Eigen::Vector2d(x, y), truth.ramp); });
  });
}

// @return the share of counted frames with an obstacle point inside the truth empty rectangle.
inline double empty_lethal_share(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return share_of_frames(frames, [&](const ReplayFrame & f) {
    return any_point(f.obstacle, [&](double x, double y) { return truth.empty.contains(x, y); });
  });
}

// @return the share of counted frames with an obstacle point more than OUTSIDE_MIN_M and at most OUTSIDE_MAX_M outside
//         the truth box footprint.
inline double outside_lethal_frames_share(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return share_of_frames(frames, [&](const ReplayFrame & f) {
    return !obstacle_xy_where(f, truth.box, [](double d) { return d > OUTSIDE_MIN_M && d <= OUTSIDE_MAX_M; }).empty();
  });
}

// @param values samples (reordered).
// @return the nearest-rank 95th percentile, NaN without samples.
inline double percentile95(std::vector<double> values)
{
  if (values.empty()) {
    return std::nan("");
  }
  std::sort(values.begin(), values.end());
  const std::size_t rank = static_cast<std::size_t>(std::ceil(EDGE_P95 * static_cast<double>(values.size())));
  return values[std::max<std::size_t>(rank, 1) - 1];
}

// @return p95 over all samples of all counted frames of the distance along the box front edge normal to the nearest
//         nearby obstacle point; NaN without counted frames.
inline double box_front_edge_error_m(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  const Eigen::Vector2d a = truth.box_front_edge[0], b = truth.box_front_edge[1];
  const double length = (b - a).norm();
  const Eigen::Vector2d dir = (b - a) / length;
  const Eigen::Vector2d normal(-dir.y(), dir.x());
  std::vector<double> errors;
  for (const ReplayFrame * f : counted_frames(frames)) {
    const auto near_box = obstacle_xy_where(*f, truth.box, [](double d) { return d <= EDGE_NEAR_M; });
    const double first = 0.5 * (1.0 - EDGE_MIDDLE_SHARE) * length;
    const double last = first + EDGE_MIDDLE_SHARE * length;
    for (double t = first; t <= last + EDGE_EPS; t += EDGE_SAMPLE_STEP_M) {
      double best = EDGE_NEAR_M;
      for (const auto & p : near_box) {
        if (std::fabs((p - a).dot(dir) - t) <= EDGE_LINE_TOL_M) {
          best = std::min(best, std::fabs((p - a).dot(normal)));
        }
      }
      errors.push_back(best);
    }
  }
  return percentile95(errors);
}

// @return p95 over counted frames of |extent along the front edge of the nearby obstacle points - the true width|;
//         NaN without counted frames.
inline double box_width_error_m(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  const Eigen::Vector2d a = truth.box_front_edge[0], b = truth.box_front_edge[1];
  const double width = (b - a).norm();
  const Eigen::Vector2d dir = (b - a) / width;
  std::vector<double> errors;
  for (const ReplayFrame * f : counted_frames(frames)) {
    const auto near_box = obstacle_xy_where(*f, truth.box, [](double d) { return d <= EDGE_NEAR_M; });
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const auto & p : near_box) {
      lo = std::min(lo, (p - a).dot(dir));
      hi = std::max(hi, (p - a).dot(dir));
    }
    errors.push_back(std::fabs((near_box.empty() ? 0.0 : hi - lo) - width));
  }
  return percentile95(errors);
}

// The raster cell centres (RASTER_M grid) inside a polygon, gripper zone excluded.
inline std::vector<Eigen::Vector2d> raster_cells(const Polygon & poly)
{
  std::vector<Eigen::Vector2d> out;
  double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
  for (const auto & v : poly) {
    x0 = std::min(x0, v.x());
    x1 = std::max(x1, v.x());
    y0 = std::min(y0, v.y());
    y1 = std::max(y1, v.y());
  }
  for (long ix = std::lround(std::floor(x0 / RASTER_M)); ix <= std::lround(std::ceil(x1 / RASTER_M)); ++ix) {
    for (long iy = std::lround(std::floor(y0 / RASTER_M)); iy <= std::lround(std::ceil(y1 / RASTER_M)); ++iy) {
      const Eigen::Vector2d c((static_cast<double>(ix) + 0.5) * RASTER_M, (static_cast<double>(iy) + 0.5) * RASTER_M);
      if (c.x() >= GRIPPER_X_MAX && inside_polygon(c, poly)) {
        out.push_back(c);
      }
    }
  }
  return out;
}

// The no_return areas of a frame, in base_link.
inline std::vector<Polygon> no_return_polygons(const ReplayFrame & f)
{
  std::vector<Polygon> out;
  for (const auto & a : f.areas) {
    if (a.kind == NO_RETURN_KIND) {
      out.push_back(area_in_base(a, f.base_T_warnings));
    }
  }
  return out;
}

// @return the share of counted frames whose no_return areas cover at least COVER_MIN_SHARE of the truth ramp
//         footprint (raster cell centres outside the gripper zone); NaN without counted frames.
inline double no_return_cover_share(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  const auto ramp_cells = raster_cells(truth.ramp);
  return share_of_frames(frames, [&](const ReplayFrame & f) {
    if (ramp_cells.empty()) {
      return false;
    }
    const auto areas = no_return_polygons(f);
    std::size_t covered = 0;
    for (const auto & c : ramp_cells) {
      covered +=
        std::any_of(areas.begin(), areas.end(), [&](const Polygon & p) { return inside_polygon(c, p); }) ? 1 : 0;
    }
    return static_cast<double>(covered) >= COVER_MIN_SHARE * static_cast<double>(ramp_cells.size());
  });
}

// @return the share of counted frames with a no_return raster cell centre (outside the gripper zone) more than
//         STRAY_AREA_M outside the truth ramp footprint; NaN without counted frames.
inline double no_return_stray_share(const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  return share_of_frames(frames, [&](const ReplayFrame & f) {
    for (const auto & polygon : no_return_polygons(f)) {
      for (const auto & c : raster_cells(polygon)) {
        if (signed_distance(c, truth.ramp) > STRAY_AREA_M) {
          return true;
        }
      }
    }
    return false;
  });
}

// @param label the scene name.
// @param frames every frame of the scene's replay.
// @param truth the scene truth.
// @return one line "replay <label>: " followed by name=value of plane_share, the truth based box_lethal_share,
//         board_lethal_frames, stray_lethal_share, ramp_lethal_frames, churn_share and every metric of the truth
//         section above (floats with 3 decimals, metres with 4).
inline std::string truth_metrics_line(
  const std::string & label, const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  char buf[1024];
  std::snprintf(
    buf, sizeof(buf),
    "replay %s: plane_share=%.3f box_lethal_share=%.3f board_lethal_frames=%d stray_lethal_share=%.3f "
    "ramp_lethal_frames=%d churn_share=%.3f box_front_edge_error_m=%.4f box_width_error_m=%.4f "
    "outside_lethal_frames_share=%.3f no_return_cover_share=%.3f no_return_stray_share=%.3f empty_lethal_share=%.3f",
    label.c_str(), plane_share(frames), box_lethal_share(frames, truth), board_lethal_frames(frames, truth),
    stray_lethal_share(frames, truth), ramp_lethal_frames(frames, truth), churn_share(frames),
    box_front_edge_error_m(frames, truth), box_width_error_m(frames, truth), outside_lethal_frames_share(frames, truth),
    no_return_cover_share(frames, truth), no_return_stray_share(frames, truth), empty_lethal_share(frames, truth));
  return buf;
}

// Print truth_metrics_line to stdout.
inline void print_metrics(const std::string & label, const std::vector<ReplayFrame> & frames, const SceneTruth & truth)
{
  std::cout << truth_metrics_line(label, frames, truth) << std::endl;
}

}  // namespace ground_replay

#endif  // REPLAY_FIXTURE_HPP_
