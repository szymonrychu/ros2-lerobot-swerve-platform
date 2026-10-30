// Synthetic head D435 for the ground estimator tests (task T2): the scene of tests/data/replay/tiny_truth.yaml (a flat
// board, a 5 cm box, a ~14 deg ramp on an empty floor, base_link, m) seen by the head D435 at a pan and tilt, rendered
// by ray casting with the factory depth intrinsics of the runbook d435-switchover step 2 (640x360), and turned into the
// cloud the node consumes the way the real pipeline does: edge mask of ros2_common_cpp::depth_mask, decimation,
// deprojection into the depth optical frame.
//
// The D435 noise model: disparity depth error sigma_z = z^2 * subpixel_px / (fx * baseline), i.e. 5 mm at 1 m and 2 cm at
// 2 m for the 50 mm baseline and 0.08 px subpixel accuracy (the plan figure behind the 8 cm at 4 m RMS), Gaussian,
// quantised to millimetres. Mixed pixels: where a nearer surface borders a farther one the first 1 to 3 pixels of the
// far side read a linear blend of the two depths (the flying pixels the edge mask removes). Holes: a small share of
// pixels returns nothing. Same arguments and seed give the same frame.
#ifndef GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_D435_HPP_
#define GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_D435_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>
#include <random>
#include <vector>

#include "ros2_common_cpp/depth_mask.hpp"

namespace d435_sim
{

constexpr int WIDTH = 640;
constexpr int HEIGHT = 360;
constexpr double DEG = M_PI / 180.0;
constexpr double MM_PER_M = 1000.0;
constexpr double MAX_RANGE_M = 10.0;  // farther surfaces return nothing
constexpr double FRAME_RATE_HZ = 15.0;
// Factory depth intrinsics, runbook d435-switchover step 2.
constexpr double FX = 318.9109, FY = 318.9109, CX = 319.8742, CY = 179.7617;
constexpr double BASELINE_M = 0.05;
constexpr double SUBPIXEL_PX = 0.08;
constexpr double HOLE_SHARE = 0.005;    // pixels that return nothing
constexpr double EDGE_JUMP_REL = 0.05;  // a neighbour this much farther than a pixel is behind its edge
constexpr int MAX_BLEND_PX = 3;
constexpr int DECIMATION = 4;  // point_cloud_xyz decimation
constexpr double CLOUD_MIN_M = 0.15, CLOUD_MAX_M = 3.0;

// head.urdf: head_pan_base in base_link, the tilt joint above it, the D435 in front of it (as in head_rgbd_cpp's
// synthetic_d435.hpp).
const Eigen::Vector3d HEAD_MOUNT{0.15, 0.10, 0.31};
const Eigen::Vector3d TILT_JOINT{0.0, 0.0, 0.04};
const Eigen::Vector3d D435_LINK{0.02, 0.0, 0.0};

enum Label : uint8_t
{
  NONE = 0,
  FLOOR = 1,
  BOARD = 2,
  BOX = 3,
  RAMP = 4,
  MIXED = 5
};

// Head orientation: pan about +z (CCW from above), tilt about +y (positive looks down).
struct HeadPose
{
  double pan_deg = 0.0;
  double tilt_deg = 30.0;  // the home pose
};

// Pose of the depth optical frame (x right, y down, z forward) in base_link.
inline Eigen::Isometry3d base_T_optical(const HeadPose & pose)
{
  const Eigen::Matrix3d pan = Eigen::AngleAxisd(pose.pan_deg * DEG, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d tilt = Eigen::AngleAxisd(pose.tilt_deg * DEG, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Matrix3d optical_in_head;  // columns: optical x, y, z in the head frame (x forward, y left, z up)
  optical_in_head.col(0) = -Eigen::Vector3d::UnitY();
  optical_in_head.col(1) = -Eigen::Vector3d::UnitZ();
  optical_in_head.col(2) = Eigen::Vector3d::UnitX();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = HEAD_MOUNT + pan * (TILT_JOINT + tilt * D435_LINK);
  t.linear() = pan * tilt * optical_in_head;
  return t;
}

// A convex solid as half-spaces n . p <= d (outward normals), so one ray-clipping routine serves boxes and the wedge.
struct Solid
{
  std::vector<std::pair<Eigen::Vector3d, double>> faces;
  Label label = BOX;
};

inline Solid box(const Eigen::Vector3d & lo, const Eigen::Vector3d & hi, Label label)
{
  Solid s;
  s.label = label;
  for (int a = 0; a < 3; ++a) {
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    n[a] = 1.0;
    s.faces.emplace_back(n, hi[a]);
    s.faces.emplace_back(-n, -lo[a]);
  }
  return s;
}

// Wedge standing on the floor: rises along +x from the floor at x0 to height_m at x0 + length_m over y0..y1, a vertical
// back face at x0 + length_m.
inline Solid wedge(double x0, double length_m, double height_m, double y0, double y1)
{
  Solid s;
  s.label = RAMP;
  const Eigen::Vector3d incline(-height_m, 0.0, length_m);  // outward normal of the sloped face (up and toward -x)
  s.faces.emplace_back(incline.normalized(), incline.normalized().dot(Eigen::Vector3d(x0, 0.0, 0.0)));
  s.faces.emplace_back(Eigen::Vector3d::UnitX(), x0 + length_m);
  s.faces.emplace_back(-Eigen::Vector3d::UnitZ(), 0.0);
  s.faces.emplace_back(Eigen::Vector3d::UnitY(), y1);
  s.faces.emplace_back(-Eigen::Vector3d::UnitY(), -y0);
  return s;
}

struct World
{
  double floor_z = 0.0;
  std::vector<Solid> solids;
};

// tiny_truth.yaml: board x 0.37-0.57, y -0.12-0.14 (a 5 mm plate); box x 0.60-0.78, y 0.05-0.33, 5 cm; ramp x 0.85-1.10,
// y -0.45--0.15, 25 cm long and 63 mm high (14.1 deg), low end near; the empty floor x 0.40-0.70, y -0.40--0.20.
constexpr double BOARD_HEIGHT_M = 0.005;
constexpr double BOX_HEIGHT_M = 0.05;
constexpr double RAMP_HEIGHT_M = 0.063, RAMP_LENGTH_M = 0.25;

inline World truth_world(bool board = true, bool box_on = true, bool ramp = true)
{
  World w;
  if (board) {
    w.solids.push_back(box({0.37, -0.12, 0.0}, {0.57, 0.14, BOARD_HEIGHT_M}, BOARD));
  }
  if (box_on) {
    w.solids.push_back(box({0.60, 0.05, 0.0}, {0.78, 0.33, BOX_HEIGHT_M}, BOX));
  }
  if (ramp) {
    w.solids.push_back(wedge(0.85, RAMP_LENGTH_M, RAMP_HEIGHT_M, -0.45, -0.15));
  }
  return w;
}

// Nearest hit of a ray (origin, direction with unit optical z) on a world: ray parameter and label; t = infinity for a
// miss.
inline bool cast(const World & w, const Eigen::Vector3d & o, const Eigen::Vector3d & d, double & t_hit, Label & label)
{
  t_hit = std::numeric_limits<double>::infinity();
  label = NONE;
  if (d.z() < -1e-12) {
    const double t = (w.floor_z - o.z()) / d.z();
    if (t > 0.0) {
      t_hit = t;
      label = FLOOR;
    }
  }
  for (const Solid & s : w.solids) {
    double t_in = 0.0;
    double t_out = std::numeric_limits<double>::infinity();
    bool ok = true;
    for (const auto & [n, offset] : s.faces) {
      const double denom = n.dot(d);
      const double dist = offset - n.dot(o);  // > 0 when the origin is inside this half-space
      if (std::fabs(denom) < 1e-12) {
        if (dist < 0.0) {
          ok = false;
          break;
        }
        continue;
      }
      const double t = dist / denom;
      if (denom < 0.0) {
        t_in = std::max(t_in, t);
      } else {
        t_out = std::min(t_out, t);
      }
      if (t_in > t_out) {
        ok = false;
        break;
      }
    }
    if (ok && t_in > 0.0 && t_in < t_hit) {
      t_hit = t_in;
      label = s.label;
    }
  }
  return label != NONE && t_hit <= MAX_RANGE_M;
}

struct Frame
{
  cv::Mat depth_mm;  // CV_16UC1, 0 = no depth
  cv::Mat label;     // CV_8UC1 of Label
};

// z^2 noise sigma of a depth, metres.
inline double sigma_m(double z_m) { return z_m * z_m * SUBPIXEL_PX / (FX * BASELINE_M); }

// Render the world seen from a head pose.
// @param world the surfaces.
// @param pose the head pose the frame was exposed at.
// @param seed noise seed.
// @param noise false for the noise-free geometry (no jitter, holes or mixed pixels).
// @return the depth and labels.
inline Frame render(const World & world, const HeadPose & pose, uint32_t seed, bool noise = true)
{
  const Eigen::Isometry3d t = base_T_optical(pose);
  Frame f{cv::Mat(HEIGHT, WIDTH, CV_16UC1, cv::Scalar(0)), cv::Mat(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(NONE))};
  cv::Mat true_m(HEIGHT, WIDTH, CV_32FC1, cv::Scalar(0.0));
  std::mt19937 rng(seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      const Eigen::Vector3d dir = t.linear() * Eigen::Vector3d((u - CX) / FX, (v - CY) / FY, 1.0);
      double z = 0.0;
      Label label = NONE;
      if (!cast(world, t.translation(), dir, z, label)) {
        continue;
      }
      true_m.at<float>(v, u) = static_cast<float>(z);
      f.label.at<uint8_t>(v, u) = label;
      if (noise && uniform(rng) < HOLE_SHARE) {
        f.label.at<uint8_t>(v, u) = NONE;
        continue;
      }
      const double measured = noise ? z + sigma_m(z) * unit(rng) : z;
      f.depth_mm.at<uint16_t>(v, u) = static_cast<uint16_t>(std::max(1L, std::lround(measured * MM_PER_M)));
    }
  }
  if (!noise) {
    return f;
  }
  // Mixed pixels: from each pixel along each image axis into a farther surface behind its edge.
  std::uniform_int_distribution<int> width_px(1, MAX_BLEND_PX);
  const std::array<cv::Point, 4> dirs = {cv::Point(1, 0), cv::Point(-1, 0), cv::Point(0, 1), cv::Point(0, -1)};
  const cv::Mat label_before = f.label.clone();
  const cv::Rect bounds(0, 0, WIDTH, HEIGHT);
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      const uint8_t l = label_before.at<uint8_t>(v, u);
      if (l == NONE || l == FLOOR) {
        continue;
      }
      const double z_fg = true_m.at<float>(v, u);
      for (const cv::Point & step : dirs) {
        const cv::Point q(u + step.x, v + step.y);
        if (
          !bounds.contains(q) || label_before.at<uint8_t>(q) != FLOOR ||
          true_m.at<float>(q) <= z_fg * (1.0 + EDGE_JUMP_REL)) {
          continue;
        }
        const int w = width_px(rng);
        for (int i = 0; i < w; ++i) {
          const cv::Point p(q.x + i * step.x, q.y + i * step.y);
          if (!bounds.contains(p) || f.label.at<uint8_t>(p) != FLOOR) {
            break;
          }
          const double z_bg = true_m.at<float>(p);
          const double mixed = z_fg + (z_bg - z_fg) * (i + 1) / (w + 1);
          f.depth_mm.at<uint16_t>(p) = static_cast<uint16_t>(std::lround(mixed * MM_PER_M));
          f.label.at<uint8_t>(p) = MIXED;
        }
      }
    }
  }
  return f;
}

// The cloud the obstacle pipeline hands the node: the shipped edge mask on the full-FOV depth (the filtered stream of
// head_rgbd_cpp), every DECIMATION-th pixel, deprojected into the depth optical frame, 0.15-3.0 m.
// @param depth_mm the rendered depth (copied; the mask runs on the copy).
// @param mask the edge mask settings (head_rgbd_cpp defaults), or nullptr for no mask.
// @return the points in the depth optical frame.
inline pcl::PointCloud<pcl::PointXYZ> optical_cloud(
  const cv::Mat & depth_mm, const ros2_common_cpp::DepthEdgeMaskParams * mask)
{
  cv::Mat depth = depth_mm.clone();
  if (mask != nullptr) {
    ros2_common_cpp::apply_depth_edge_mask(depth, *mask);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int v = DECIMATION / 2; v < HEIGHT; v += DECIMATION) {
    for (int u = DECIMATION / 2; u < WIDTH; u += DECIMATION) {
      const uint16_t mm = depth.at<uint16_t>(v, u);
      if (mm == 0) {
        continue;
      }
      const double z = mm / MM_PER_M;
      if (z < CLOUD_MIN_M || z > CLOUD_MAX_M) {
        continue;
      }
      cloud.push_back(pcl::PointXYZ(
        static_cast<float>((u - CX) / FX * z), static_cast<float>((v - CY) / FY * z), static_cast<float>(z)));
    }
  }
  return cloud;
}

}  // namespace d435_sim

#endif  // GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_D435_HPP_
