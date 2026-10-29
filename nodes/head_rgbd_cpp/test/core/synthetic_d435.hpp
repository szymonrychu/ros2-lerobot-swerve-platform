// Synthetic D435 depth of the ground scene (scripts/ground_record.yaml box scene, tiny_truth.yaml layout): the floor
// and the 5 cm box seen by the head D435 at a pan/tilt, rendered by ray casting with the factory intrinsics of the
// runbook d435-switchover step 2 (640x360), quantised to 16UC1 millimetres like the camera, with a small per-pixel
// jitter. Mixed pixels are injected where the box edge occludes the floor: linear blends between the box depth and the
// floor depth behind it, 1 to 3 px wide, on the floor side of the edge. Every pixel carries its true surface label so
// a test can count what a filter keeps and removes. OpenCV core only; same arguments give the same image.
#ifndef HEAD_RGBD_CPP__TEST__SYNTHETIC_D435_HPP_
#define HEAD_RGBD_CPP__TEST__SYNTHETIC_D435_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>
#include <random>

namespace d435_test
{

constexpr int WIDTH = 640;
constexpr int HEIGHT = 360;
constexpr double DEG = M_PI / 180.0;
constexpr double MM_PER_M = 1000.0;
// Farther surfaces give no depth (the D435 range at this profile is about 10 m).
constexpr double MAX_RANGE_M = 10.0;
// Per-pixel depth jitter, relative to the depth (2 mm at 1 m).
constexpr double JITTER_REL = 0.002;
// A neighbour this much farther than a box pixel is the floor behind the box edge (5 %, well above the floor's own
// depth gradient at 0.3-2 m from the head height).
constexpr double EDGE_JUMP_REL = 0.05;
constexpr int MAX_BLEND_PX = 3;

// head.urdf: head_pan_base in base_link, the tilt joint above it, d435_link and the colour frame in head_link.
const cv::Vec3d HEAD_MOUNT{0.15, 0.10, 0.31};
const cv::Vec3d TILT_JOINT{0.0, 0.0, 0.04};
const cv::Vec3d D435_LINK{0.02, 0.0, 0.0};
const cv::Vec3d COLOUR_IN_D435{0.0001761, 0.0150171, -0.0000698};

// The box of tiny_truth.yaml (base_link, m): x 0.60-0.78, y 0.05-0.33, 5 cm high.
const cv::Vec3d BOX_MIN{0.60, 0.05, 0.0};
const cv::Vec3d BOX_MAX{0.78, 0.33, 0.05};

struct Intrinsics
{
  double fx;
  double fy;
  double cx;
  double cy;
};

// Factory intrinsics, runbook d435-switchover step 2.
constexpr Intrinsics DEPTH_K{318.9109, 318.9109, 319.8742, 179.7617};
constexpr Intrinsics COLOUR_K{457.8466, 457.3613, 316.9875, 179.5461};

enum Label : uint8_t
{
  NONE = 0,
  FLOOR = 1,
  BOX_TOP = 2,
  BOX_SIDE = 3,
  MIXED = 4
};

struct Frame
{
  cv::Mat depth_mm;  // CV_16UC1
  cv::Mat label;     // CV_8UC1 of Label
  cv::Mat true_m;    // CV_64FC1 true depth before the jitter and the mixing, 0 for none
};

struct View
{
  double pan_deg = 0.0;
  double tilt_deg = 30.0;  // the home pose
  bool colour = false;     // the colour camera (the aligned depth) instead of the depth camera
};

inline cv::Matx33d rot_z(double a)
{
  return cv::Matx33d(std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1);
}

// Positive tilt looks down: +x of the head turns towards -z.
inline cv::Matx33d rot_y(double a)
{
  return cv::Matx33d(std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a));
}

// Ray parameter of the nearest hit of the box (slab test) and whether it enters through the top face.
inline bool hit_box(const cv::Vec3d & o, const cv::Vec3d & d, double & t_hit, bool & top)
{
  double t_near = -std::numeric_limits<double>::infinity();
  double t_far = std::numeric_limits<double>::infinity();
  int axis_near = -1;
  for (int a = 0; a < 3; ++a) {
    if (std::fabs(d[a]) < 1e-12) {
      if (o[a] < BOX_MIN[a] || o[a] > BOX_MAX[a]) {
        return false;
      }
      continue;
    }
    double t0 = (BOX_MIN[a] - o[a]) / d[a];
    double t1 = (BOX_MAX[a] - o[a]) / d[a];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    if (t0 > t_near) {
      t_near = t0;
      axis_near = a;
    }
    t_far = std::min(t_far, t1);
  }
  if (t_near > t_far || t_near <= 0.0) {
    return false;
  }
  t_hit = t_near;
  top = axis_near == 2;
  return true;
}

// Renders the scene for a view, then injects the mixed pixels.
// @param view the head pose and camera.
// @param seed seed of the jitter and of the blend widths.
// @return the depth, labels and true depth.
inline Frame render(const View & view, uint32_t seed = 7)
{
  const Intrinsics k = view.colour ? COLOUR_K : DEPTH_K;
  const cv::Matx33d r = rot_z(view.pan_deg * DEG) * rot_y(view.tilt_deg * DEG);
  const cv::Vec3d in_head = view.colour ? D435_LINK + COLOUR_IN_D435 : D435_LINK;
  const cv::Vec3d origin = HEAD_MOUNT + rot_z(view.pan_deg * DEG) * (TILT_JOINT + rot_y(view.tilt_deg * DEG) * in_head);

  Frame f{
    cv::Mat(HEIGHT, WIDTH, CV_16UC1, cv::Scalar(0)), cv::Mat(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(NONE)),
    cv::Mat(HEIGHT, WIDTH, CV_64FC1, cv::Scalar(0.0))};
  std::mt19937 rng(seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      // Optical (x right, y down, z forward) -> head link (x forward, y left, z up) -> base_link. z = 1 on the
      // optical axis, so the ray parameter is the depth.
      const cv::Vec3d optical((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      const cv::Vec3d dir = r * cv::Vec3d(optical[2], -optical[0], -optical[1]);
      double t = std::numeric_limits<double>::infinity();
      Label label = NONE;
      double t_box = 0.0;
      bool top = false;
      if (hit_box(origin, dir, t_box, top)) {
        t = t_box;
        label = top ? BOX_TOP : BOX_SIDE;
      } else if (dir[2] < 0.0) {
        t = -origin[2] / dir[2];
        label = FLOOR;
      }
      if (label == NONE || t > MAX_RANGE_M) {
        continue;
      }
      f.true_m.at<double>(v, u) = t;
      f.label.at<uint8_t>(v, u) = label;
      const double jittered = t * (1.0 + JITTER_REL * unit(rng));
      f.depth_mm.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(jittered * MM_PER_M));
    }
  }

  // Mixed pixels: from each box pixel, along each image axis, into the floor that jumps away behind the box edge.
  std::uniform_int_distribution<int> width_px(1, MAX_BLEND_PX);
  const std::array<cv::Point, 4> dirs = {cv::Point(1, 0), cv::Point(-1, 0), cv::Point(0, 1), cv::Point(0, -1)};
  const cv::Mat label_before = f.label.clone();
  const cv::Rect bounds(0, 0, WIDTH, HEIGHT);
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      const uint8_t l = label_before.at<uint8_t>(v, u);
      if (l != BOX_TOP && l != BOX_SIDE) {
        continue;
      }
      const double z_fg = f.true_m.at<double>(v, u);
      for (const cv::Point & step : dirs) {
        const cv::Point q(u + step.x, v + step.y);
        if (
          !bounds.contains(q) || label_before.at<uint8_t>(q) != FLOOR ||
          f.true_m.at<double>(q) <= z_fg * (1.0 + EDGE_JUMP_REL)) {
          continue;
        }
        const int w = width_px(rng);
        for (int i = 0; i < w; ++i) {
          const cv::Point p(q.x + i * step.x, q.y + i * step.y);
          if (!bounds.contains(p) || f.label.at<uint8_t>(p) != FLOOR) {
            break;
          }
          const double z_bg = f.true_m.at<double>(p);
          const double mixed = z_fg + (z_bg - z_fg) * (i + 1) / (w + 1);
          f.depth_mm.at<uint16_t>(p) = static_cast<uint16_t>(std::lround(mixed * MM_PER_M));
          f.label.at<uint8_t>(p) = MIXED;
        }
      }
    }
  }
  return f;
}

}  // namespace d435_test

#endif  // HEAD_RGBD_CPP__TEST__SYNTHETIC_D435_HPP_
