// Part replay-harness (ground-detection-robustness design D14): the reader of the replay binaries in
// test/data/replay (written by scripts/ground_replay_export.py) and the replay metrics over what GroundNode published
// per cloud.
//
// Binary layout, little-endian, version 1 (scripts/ground_replay_export.py module docstring):
//   magic "GRNDRPLY" (8 bytes), uint32 version, uint32 frame_len + frame_len bytes of the cloud frame id,
//   uint32 cloud_count, per cloud: int64 stamp_ns, float64[7] base_T_cloud, float64[7] odom_T_base (x y z qx qy qz qw),
//   uint32 point_count, int16[point_count * 3] x y z mm in the cloud frame;
//   uint32 imu_count, per sample: int64 stamp_ns, float64[10] orientation x y z w, linear_acceleration x y z,
//   angular_velocity x y z (frame imu_link); float64[7] base_T_imu.
//
// Metrics (D14, base_link, every one ignores points and areas with x < GRIPPER_X_MAX, the robot's parked gripper):
// the frames counted are those at least WARM_UP_S after the first frame that the node answered with a plane.
//   plane_share              answered frames / frames after the warm-up
//   box_lethal_share         share of answered frames with an obstacle point in BOX_LETHAL_ROI
//   board_lethal_frames      answered frames with an obstacle or drop point in BOARD_ROI
//   stray_lethal_share       share of answered frames with an obstacle or drop point in the stray zone: x >= 0.37,
//                            hypot(x, y) <= STRAY_RANGE_M, outside BOX_LETHAL_ROI
//   far_side_warning_frames  answered frames with a ramp or step warning area whose polygon centroid (vertex mean) lies
//                            in FAR_SIDE_ROI
//   ramp_lethal_frames       answered frames with an obstacle or drop point in RAMP_ROI
//   churn_share              share of consecutive answered frames whose warning set (kind plus polygon vertices
//                            rounded to 1 cm, scripts/ground_noise_report.py:warning_set) differs
// Shares are NaN when no frame is counted. ROI edges are inclusive.
#ifndef REPLAY_FIXTURE_HPP_
#define REPLAY_FIXTURE_HPP_

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ground_replay
{

constexpr char REPLAY_MAGIC[] = "GRNDRPLY";
constexpr std::size_t REPLAY_MAGIC_SIZE = 8;
constexpr uint32_t REPLAY_VERSION = 1;
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

struct ReplayScene
{
  std::string frame_id;  // frame of the clouds
  std::vector<ReplayCloud> clouds;
  std::vector<ReplayImu> imu;
  Eigen::Isometry3d base_T_imu = Eigen::Isometry3d::Identity();  // base_link <- imu_link
};

// What GroundNode published for one cloud.
struct ReplayFrame
{
  int64_t stamp_ns = 0;
  std::vector<Eigen::Vector3f> obstacle;  // /ground/obstacle_points of the stamp, base_link
  std::vector<Eigen::Vector3f> drop;      // /ground/drop_points of the stamp, base_link
  nlohmann::json warnings;                // the /ground/warning_areas document of the stamp (null when none)
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

// Decode a replay binary.
// @param bytes the file content.
// @return the scene.
// @throws std::runtime_error on a wrong magic or version, a truncated file or trailing bytes.
inline ReplayScene decode_replay(const std::string & bytes)
{
  ByteReader in(bytes);
  if (bytes.size() < REPLAY_MAGIC_SIZE || bytes.compare(0, REPLAY_MAGIC_SIZE, REPLAY_MAGIC) != 0) {
    throw std::runtime_error("not a ground replay file: wrong magic");
  }
  in.take(REPLAY_MAGIC_SIZE);
  const uint32_t version = in.u32();
  if (version != REPLAY_VERSION) {
    throw std::runtime_error(
      "unsupported ground replay version " + std::to_string(version) + ", expected " + std::to_string(REPLAY_VERSION));
  }
  ReplayScene scene;
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

}  // namespace ground_replay

#endif  // REPLAY_FIXTURE_HPP_
