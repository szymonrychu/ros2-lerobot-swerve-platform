// ArmKinematics forward kinematics and IK in C++ (see kinematics.hpp); every step mirrors mcp_server/ik.py.
#if defined(__clang__)
// No fused multiply-add: the sums round like numpy's (GCC in ISO C++ mode does not contract either).
#pragma STDC FP_CONTRACT OFF
#endif

#include "ik_reach/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "ik_reach/trf.hpp"

namespace ik_reach {

namespace {

constexpr double TAU = 2.0 * PI;
constexpr double DEG_TO_RAD = PI / 180.0;

// The rotation of ikpy's geometry.axis_rotation_matrix about a unit axis.
std::array<std::array<double, 3>, 3> axis_rotation(const Vec3 &axis, double theta) {
  const double x = axis[0], y = axis[1], z = axis[2];
  const double c = std::cos(theta), s = std::sin(theta);
  return {{
      {x * x + (1 - x * x) * c, x * y * (1 - c) - z * s, x * z * (1 - c) + y * s},
      {x * y * (1 - c) + z * s, y * y + (1 - y * y) * c, y * z * (1 - c) - x * s},
      {x * z * (1 - c) - y * s, y * z * (1 - c) + x * s, z * z + (1 - z * z) * c},
  }};
}

Mat4 identity() {
  Mat4 m{};
  for (int i = 0; i < 4; ++i) m[i][i] = 1.0;
  return m;
}

Mat4 multiply(const Mat4 &a, const Mat4 &b) {
  Mat4 out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      double acc = 0.0;
      for (int k = 0; k < 4; ++k) acc += a[r][k] * b[k][c];
      out[r][c] = acc;
    }
  return out;
}

// One link's frame matrix: its fixed matrix times the joint rotation (ikpy URDFLink.get_link_frame_matrix).
Mat4 link_matrix(const Mat4 &fixed, const Vec3 &axis, bool moves, double theta) {
  if (!moves) return fixed;
  const auto rot = axis_rotation(axis, theta);
  Mat4 out = fixed;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 3; ++c) out[r][c] = fixed[r][0] * rot[0][c] + fixed[r][1] * rot[1][c] + fixed[r][2] * rot[2][c];
  return out;
}

// Python's math.dist / math.hypot of three components.
double length3(double a, double b, double c) { return std::sqrt(a * a + b * b + c * c); }

Vec3 rotate_offset(const Mat4 &frame, const Vec3 &offset) {
  Vec3 out{};
  for (int r = 0; r < 3; ++r) out[r] = frame[r][0] * offset[0] + frame[r][1] * offset[1] + frame[r][2] * offset[2];
  return out;
}

template <class T>
void append_bytes(std::string &key, const T &value) {
  key.append(reinterpret_cast<const char *>(&value), sizeof(T));
}

}  // namespace

double pitch_of(double ax, double ay, double az) { return std::atan2(-az, std::hypot(ax, ay)); }

ArmModel::ArmModel(ArmParams params) : params_(std::move(params)) {
  const std::size_t n = params_.link_fixed.size();
  if (n == 0 || n > MAX_LINKS || params_.link_axis.size() != n || params_.link_moves.size() != n)
    throw std::invalid_argument("ArmModel: link_fixed, link_axis and link_moves need the same 1..12 links");
  for (int j = 0; j < CHAIN_JOINTS; ++j) {
    const int link = params_.joint_link[j];
    if (link < 0 || link >= static_cast<int>(n) || !params_.link_moves[link])
      throw std::invalid_argument("ArmModel: every chain joint needs a revolute link");
  }
  for (int j = 0; j < ACTIVE_JOINTS; ++j) {
    lb_[j] = params_.lower[j] + params_.margin;
    ub_[j] = params_.upper[j] - params_.margin;
  }
}

LinkVector ArmModel::urdf_vector(const Joints &urdf) const {
  LinkVector q{};
  for (int j = 0; j < CHAIN_JOINTS; ++j) q[params_.joint_link[j]] = urdf[j];
  return q;
}

void ArmModel::forward_all(const LinkVector &q, Frames &out) const {
  Mat4 frame = identity();
  for (int i = 0; i < links(); ++i) {
    frame = multiply(frame, link_matrix(params_.link_fixed[i], params_.link_axis[i], params_.link_moves[i], q[i]));
    out[i] = frame;
  }
}

Mat4 ArmModel::forward_last(const LinkVector &q) const {
  Mat4 frame = identity();
  for (int i = 0; i < links(); ++i)
    frame = multiply(frame, link_matrix(params_.link_fixed[i], params_.link_axis[i], params_.link_moves[i], q[i]));
  return frame;
}

Joints ArmModel::to_urdf(const Joints &measured) const {
  Joints out{};
  for (int j = 0; j < CHAIN_JOINTS; ++j) out[j] = measured[j] + params_.offsets[j];
  return out;
}

Joints ArmModel::to_measured(const Joints &urdf) const {
  Joints out{};
  for (int j = 0; j < CHAIN_JOINTS; ++j) out[j] = urdf[j] - params_.offsets[j];
  return out;
}

Joints ArmModel::clamp_seed(const Joints &joints) const {
  Joints out{};
  for (int j = 0; j < CHAIN_JOINTS; ++j)
    out[j] = std::min(std::max(joints[j], params_.lower[j] + params_.margin), params_.upper[j] - params_.margin);
  return out;
}

bool ArmModel::within_limits(const Joints &urdf) const {
  const double eps = 1e-9;
  for (int j = 0; j < CHAIN_JOINTS; ++j) {
    if (!(params_.lower[j] + params_.margin - eps <= urdf[j] && urdf[j] <= params_.upper[j] - params_.margin + eps))
      return false;
  }
  return true;
}

Vec3 ArmModel::tool_point(const Joints &measured, const Vec3 &extra_offset, double *pitch) const {
  const Mat4 frame = forward_last(urdf_vector(to_urdf(measured)));
  Vec3 offset{};
  for (int i = 0; i < 3; ++i) offset[i] = params_.tool_offset[i] + extra_offset[i];
  const Vec3 turned = rotate_offset(frame, offset);
  if (pitch != nullptr) *pitch = pitch_of(frame[0][2], frame[1][2], frame[2][2]);
  return {frame[0][3] + turned[0], frame[1][3] + turned[1], frame[2][3] + turned[2]};
}

Joints ArmModel::solve(const Vec3 &target, const std::optional<Vec3> &orientation, const Joints &start, double roll) {
  const LinkVector initial = urdf_vector(clamp_seed(start));
  std::string key;
  key.reserve(8 * (3 + 3 + MAX_LINKS + 1) + 1);
  for (double v : target) append_bytes(key, v);
  if (orientation)
    for (double v : *orientation) append_bytes(key, v);
  for (int i = 0; i < links(); ++i) append_bytes(key, initial[i]);
  append_bytes(key, roll);
  key.push_back(orientation ? 'o' : 'p');
  auto hit = cache_.find(key);
  if (hit != cache_.end()) {
    order_.splice(order_.end(), order_, hit->second.second);
    return hit->second.first;
  }
  ++solve_count;
  trf::VecN<ACTIVE_JOINTS> x0{};
  for (int j = 0; j < ACTIVE_JOINTS; ++j) x0[j] = initial[params_.joint_link[j]];
  auto full = [&](const trf::VecN<ACTIVE_JOINTS> &x) {
    LinkVector q = initial;
    for (int j = 0; j < ACTIVE_JOINTS; ++j) q[params_.joint_link[j]] = x[j];
    return q;
  };
  trf::VecN<ACTIVE_JOINTS> x{};
  if (orientation) {
    const Vec3 o = *orientation;
    auto residuals = [&](const trf::VecN<ACTIVE_JOINTS> &v, trf::VecN<6> &f) {
      const Mat4 fk = forward_last(full(v));
      for (int r = 0; r < 3; ++r) {
        f[r] = fk[r][3] - target[r];
        f[3 + r] = 1.0 * (fk[r][2] - o[r]);
      }
    };
    x = trf::least_squares<6, ACTIVE_JOINTS>(residuals, x0, lb_, ub_);
  } else {
    auto residuals = [&](const trf::VecN<ACTIVE_JOINTS> &v, trf::VecN<3> &f) {
      const Mat4 fk = forward_last(full(v));
      for (int r = 0; r < 3; ++r) f[r] = fk[r][3] - target[r];
    };
    x = trf::least_squares<3, ACTIVE_JOINTS>(residuals, x0, lb_, ub_);
  }
  const LinkVector q = full(x);
  Joints sol{};
  for (int j = 0; j < CHAIN_JOINTS; ++j) sol[j] = q[params_.joint_link[j]];
  sol[WRIST_ROLL] = roll;
  order_.push_back(key);
  cache_.emplace(std::move(key), std::make_pair(sol, std::prev(order_.end())));
  if (cache_.size() > SOLUTION_CACHE_SIZE) {
    cache_.erase(order_.front());
    order_.pop_front();
  }
  return sol;
}

std::optional<Joints> ArmModel::inverse_flange(double x, double y, double z, std::optional<double> pitch,
                                               const Joints &seed_urdf, double heading_bias) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || (pitch && !std::isfinite(*pitch)))
    return std::nullopt;
  const double roll = std::min(std::max(seed_urdf[WRIST_ROLL], params_.lower[WRIST_ROLL] + params_.margin),
                               params_.upper[WRIST_ROLL] - params_.margin);
  const Vec3 target = {x, y, z};
  const double bearing = std::atan2(y - params_.pan_axis_y, x - params_.pan_axis_x);
  double heading = bearing + heading_bias;
  std::array<Joints, 1 + EXTRA_SEEDS.size()> seeds{};
  seeds[0] = seed_urdf;
  for (std::size_t k = 0; k < EXTRA_SEEDS.size(); ++k)
    seeds[k + 1] = {-heading, EXTRA_SEEDS[k][0], EXTRA_SEEDS[k][1], EXTRA_SEEDS[k][2], 0.0};
  const double pitch_tolerance = PITCH_TOLERANCE_DEG * DEG_TO_RAD;
  for (Joints candidate : seeds) {
    candidate[WRIST_ROLL] = roll;
    Joints sol = clamp_seed(candidate);
    bool has_previous = false;
    double previous_heading = 0.0, previous_gap = 0.0;
    const int refinements = pitch ? HEADING_REFINEMENTS : 1;
    for (int it = 0; it < refinements; ++it) {
      std::optional<Vec3> orientation;
      if (pitch) {
        const double cp = std::cos(*pitch);
        orientation = Vec3{cp * std::cos(heading), cp * std::sin(heading), -std::sin(*pitch)};
      }
      sol = solve(target, orientation, sol, roll);
      const Mat4 frame = forward_last(urdf_vector(sol));
      const double ax = frame[0][2], ay = frame[1][2], az = frame[2][2];
      const double error = length3(frame[0][3] - x, frame[1][3] - y, frame[2][3] - z);
      const double reached_pitch = pitch_of(ax, ay, az);
      const bool pitch_ok = !pitch || std::abs(reached_pitch - *pitch) <= pitch_tolerance;
      if (error <= POSITION_TOLERANCE_M && pitch_ok && within_limits(sol)) {
        if (pitch) last_heading_bias = std::remainder(heading - bearing, TAU);
        return to_measured(sol);
      }
      if (std::hypot(ax, ay) > 1e-6) {
        const double reached_heading = std::atan2(ay, ax);
        const double gap = std::remainder(reached_heading - heading, TAU);
        double next_heading = reached_heading;
        const double step = has_previous ? std::remainder(heading - previous_heading, TAU) : 0.0;
        // A repeated heading (step 0) has no secant: fall back to the reached heading.
        if (has_previous && std::abs(gap - previous_gap) > 1e-12 && step != 0.0) {
          const double slope = (gap - previous_gap) / step;
          next_heading = std::abs(slope) > 1e-9 ? heading - gap / slope : reached_heading;
        }
        has_previous = true;
        previous_heading = heading;
        previous_gap = gap;
        heading = next_heading;
      }
    }
  }
  return std::nullopt;
}

std::optional<Joints> ArmModel::inverse(double x, double y, double z, std::optional<double> pitch,
                                        const Joints &seed_urdf, const Vec3 &extra_offset, double heading_bias) {
  Vec3 offset{};
  bool any_offset = false;
  bool any_extra = false;
  for (int i = 0; i < 3; ++i) {
    offset[i] = params_.tool_offset[i] + extra_offset[i];
    any_offset = any_offset || params_.tool_offset[i] != 0.0;
    any_extra = any_extra || extra_offset[i] != 0.0;
  }
  const double tool_reach = length3(offset[0], offset[1], offset[2]);
  if (std::sqrt(x * x + y * y + z * z) > params_.max_reach_m + tool_reach + POSITION_TOLERANCE_M) return std::nullopt;
  if (!(any_offset || any_extra)) return inverse_flange(x, y, z, pitch, seed_urdf, heading_bias);
  Vec3 flange = {x, y, z};
  std::optional<Joints> sol = inverse_flange(x, y, z, pitch, seed_urdf, heading_bias);
  if (!sol) return std::nullopt;
  for (int it = 0; it < TOOL_ITERATIONS; ++it) {
    const Vec3 reached = tool_point(*sol, extra_offset, nullptr);
    const Vec3 residual = {x - reached[0], y - reached[1], z - reached[2]};
    if (length3(residual[0], residual[1], residual[2]) < TOOL_CONVERGENCE_M) break;
    for (int i = 0; i < 3; ++i) flange[i] = flange[i] + residual[i];
    sol = inverse_flange(flange[0], flange[1], flange[2], pitch, to_urdf(*sol), last_heading_bias);
    if (!sol) return std::nullopt;
  }
  const Vec3 reached = tool_point(*sol, extra_offset, nullptr);
  if (length3(reached[0] - x, reached[1] - y, reached[2] - z) > POSITION_TOLERANCE_M) return std::nullopt;
  return sol;
}

}  // namespace ik_reach
