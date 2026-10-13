#include "arm_ik.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <nanobind/stl/string.h>

#include "py_format.hpp"
#include "trf.hpp"

namespace nb = nanobind;

namespace grasp {
namespace {

constexpr double TAU = 6.283185307179586;
constexpr double LIMIT_EPS = 1e-9;          // ArmKinematics.within_limits slack
constexpr double HEADING_MIN_NORM = 1e-6;   // horizontal approach norm below which the heading is undefined
constexpr double GAP_MIN_CHANGE = 1e-12;    // secant step needs a change of the heading gap
constexpr double SLOPE_MIN = 1e-9;          // secant slope below which the plain fixed-point step is taken
constexpr int POSE_RESIDUALS = 6;  // position, then the tool z axis (ikpy orientation_mode "Z")

Mat4 identity() {
  Mat4 m{};
  m[0] = m[5] = m[10] = m[15] = 1.0;
  return m;
}

Mat4 multiply(const Mat4 &a, const Mat4 &b) {
  Mat4 c{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) s += a[i * 4 + k] * b[k * 4 + j];
      c[i * 4 + j] = s;
    }
  return c;
}

// ikpy geometry.axis_rotation_matrix as a homogeneous transform.
Mat4 axis_rotation(const Vec3 &axis, double theta) {
  const double x = axis[0], y = axis[1], z = axis[2];
  const double c = std::cos(theta), s = std::sin(theta);
  Mat4 r = identity();
  r[0] = x * x + (1 - x * x) * c;
  r[1] = x * y * (1 - c) - z * s;
  r[2] = x * z * (1 - c) + y * s;
  r[4] = x * y * (1 - c) + z * s;
  r[5] = y * y + (1 - y * y) * c;
  r[6] = y * z * (1 - c) - x * s;
  r[8] = x * z * (1 - c) - y * s;
  r[9] = y * z * (1 - c) + x * s;
  r[10] = z * z + (1 - z * z) * c;
  return r;
}

double dist3(const Vec3 &a, const Vec3 &b) { return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]); }

double norm3(const Vec3 &a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

int position_of(const std::vector<std::string> &names, const char *name) {
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) throw std::invalid_argument(std::string("geometry: joints lack ") + name);
  return static_cast<int>(it - names.begin());
}

template <typename T>
std::vector<T> cast_list(const nb::handle &h) {
  std::vector<T> out;
  for (nb::handle item : h) out.push_back(nb::cast<T>(item));
  return out;
}

Vec3 cast_vec3(const nb::handle &h) {
  const auto v = cast_list<double>(h);
  if (v.size() != 3) throw std::invalid_argument("geometry: expected 3 values");
  return {v[0], v[1], v[2]};
}

void append_bytes(std::string &key, const double *values, std::size_t count) {
  key.append(reinterpret_cast<const char *>(values), count * sizeof(double));
}

}  // namespace

GeometryModel GeometryModel::parse(const nb::dict &g) {
  GeometryModel m;
  m.origin.push_back(identity());
  m.axis.push_back({0.0, 0.0, 0.0});
  m.revolute.push_back(false);
  std::vector<std::string> link_names{"Base link"};
  std::vector<std::pair<double, double>> bounds{{0.0, 0.0}};
  for (nb::handle link : nb::cast<nb::list>(g["links"])) {
    nb::dict d = nb::cast<nb::dict>(link);
    Mat4 origin{};
    int r = 0;
    for (nb::handle row : nb::cast<nb::list>(d["origin"])) {
      int c = 0;
      for (nb::handle v : row) origin[r * 4 + c++] = nb::cast<double>(v);
      ++r;
    }
    m.origin.push_back(origin);
    const bool revolute = nb::cast<std::string>(d["joint_type"]) == "revolute";
    m.revolute.push_back(revolute);
    m.axis.push_back(revolute ? cast_vec3(d["axis"]) : Vec3{0.0, 0.0, 0.0});
    bounds.push_back({0.0, 0.0});
    if (revolute) {
      const auto b = cast_list<double>(d["bounds"]);
      bounds.back() = {b.at(0), b.at(1)};
    }
    link_names.push_back(nb::cast<std::string>(d["name"]));
  }
  m.links = static_cast<int>(m.origin.size());
  m.joints = cast_list<std::string>(g["joints"]);
  const auto active = cast_list<std::string>(g["active"]);
  for (int l = 0; l < m.links; ++l) {
    if (std::find(active.begin(), active.end(), link_names[static_cast<std::size_t>(l)]) != active.end()) {
      m.active_links.push_back(l);
      m.lower.push_back(bounds[static_cast<std::size_t>(l)].first);
      m.upper.push_back(bounds[static_cast<std::size_t>(l)].second);
    }
  }
  nb::dict index = nb::cast<nb::dict>(g["index"]);
  nb::dict offsets = nb::cast<nb::dict>(g["offsets"]);
  nb::dict limits = nb::cast<nb::dict>(g["limits"]);
  for (const std::string &j : m.joints) {
    m.index.push_back(nb::cast<int>(index[j.c_str()]));
    m.offsets.push_back(nb::cast<double>(offsets[j.c_str()]));
    const auto lim = cast_list<double>(limits[j.c_str()]);
    m.limit_lo.push_back(lim.at(0));
    m.limit_hi.push_back(lim.at(1));
  }
  m.roll = position_of(m.joints, "wrist_roll");
  m.pan = position_of(m.joints, "shoulder_pan");
  m.lift = position_of(m.joints, "shoulder_lift");
  m.elbow = position_of(m.joints, "elbow_flex");
  m.wrist = position_of(m.joints, "wrist_flex");
  m.margin = nb::cast<double>(g["margin"]);
  m.tool_offset = cast_vec3(g["tool_offset"]);
  m.max_reach_m = nb::cast<double>(g["max_reach_m"]);
  const auto pan_axis = cast_list<double>(g["pan_axis_xy"]);
  m.pan_x = pan_axis.at(0);
  m.pan_y = pan_axis.at(1);
  m.position_tolerance_m = nb::cast<double>(g["position_tolerance_m"]);
  m.pitch_tolerance_rad = nb::cast<double>(g["pitch_tolerance_rad"]);
  m.heading_refinements = nb::cast<int>(g["heading_refinements"]);
  m.tool_iterations = nb::cast<int>(g["tool_iterations"]);
  m.tool_convergence_m = nb::cast<double>(g["tool_convergence_m"]);
  for (nb::handle s : nb::cast<nb::list>(g["extra_seeds"])) m.extra_seeds.push_back(cast_vec3(s));
  m.cache_size = nb::cast<std::size_t>(g["cache_size"]);
  if (static_cast<int>(m.active_links.size()) > TRF_MAX_N) throw std::invalid_argument("geometry: too many active joints");
  return m;
}

ArmIk::ArmIk(const GeometryModel &model, nb::object cache) : model_(model), cache_(std::move(cache)) {
  for (const std::string &j : model_.joints) names_.emplace_back(j.c_str());
}

Mat4 ArmIk::forward_kinematics(const std::vector<double> &q) const {
  Mat4 frame = identity();
  for (int l = 0; l < model_.links; ++l) {
    const std::size_t i = static_cast<std::size_t>(l);
    const Mat4 link = model_.revolute[i] ? multiply(model_.origin[i], axis_rotation(model_.axis[i], q[i])) : model_.origin[i];
    frame = multiply(frame, link);
  }
  return frame;
}

std::vector<double> ArmIk::urdf_vector(const Joints &urdf) const {
  std::vector<double> q(static_cast<std::size_t>(model_.links), 0.0);
  for (std::size_t j = 0; j < urdf.size(); ++j) q[static_cast<std::size_t>(model_.index[j])] = urdf[j];
  return q;
}

Joints ArmIk::clamp_seed(const Joints &urdf) const {
  Joints out(urdf.size());
  for (std::size_t j = 0; j < urdf.size(); ++j)
    out[j] = std::min(std::max(urdf[j], model_.limit_lo[j] + model_.margin), model_.limit_hi[j] - model_.margin);
  return out;
}

bool ArmIk::within_limits(const Joints &urdf) const {
  for (std::size_t j = 0; j < urdf.size(); ++j)
    if (!(model_.limit_lo[j] + model_.margin - LIMIT_EPS <= urdf[j] &&
          urdf[j] <= model_.limit_hi[j] - model_.margin + LIMIT_EPS))
      return false;
  return true;
}

Vec3 ArmIk::tool_point(const Joints &measured, const std::optional<Vec3> &extra) const {
  Joints urdf(measured.size());
  for (std::size_t j = 0; j < measured.size(); ++j) urdf[j] = measured[j] + model_.offsets[j];
  const Mat4 f = forward_kinematics(urdf_vector(urdf));
  Vec3 offset = model_.tool_offset;
  if (extra)
    for (int k = 0; k < 3; ++k) offset[static_cast<std::size_t>(k)] = model_.tool_offset[static_cast<std::size_t>(k)] + (*extra)[static_cast<std::size_t>(k)];
  Vec3 p{};
  for (int r = 0; r < 3; ++r) {
    double rot = 0.0;
    for (int k = 0; k < 3; ++k) rot += f[r * 4 + k] * offset[static_cast<std::size_t>(k)];
    p[static_cast<std::size_t>(r)] = f[r * 4 + 3] + rot;
  }
  return p;
}

Joints ArmIk::solve(const Vec3 &target, const Vec3 &orientation, const Joints &start, double roll) {
  const std::vector<double> initial = urdf_vector(clamp_seed(start));
  std::string key;
  append_bytes(key, target.data(), 3);
  append_bytes(key, orientation.data(), 3);
  append_bytes(key, initial.data(), initial.size());
  append_bytes(key, &roll, 1);
  key += 'o';
  nb::bytes cache_key(key.data(), key.size());
  nb::object cached = cache_.attr("get")(cache_key);
  Joints sol(model_.joints.size());
  if (!cached.is_none()) {
    cache_.attr("move_to_end")(cache_key);
    nb::dict values = nb::cast<nb::dict>(cached);
    for (std::size_t j = 0; j < sol.size(); ++j) sol[j] = nb::cast<double>(values[names_[j]]);
    return sol;
  }
  ++solves;
  const int n = static_cast<int>(model_.active_links.size());
  const int m = POSE_RESIDUALS;
  TrfVecN x0{};
  for (int i = 0; i < n; ++i) x0[static_cast<std::size_t>(i)] = initial[static_cast<std::size_t>(model_.active_links[static_cast<std::size_t>(i)])];
  std::vector<double> q = initial;
  const TrfResidual residual = [&](const double *x, double *f) {
    for (int i = 0; i < n; ++i) q[static_cast<std::size_t>(model_.active_links[static_cast<std::size_t>(i)])] = x[i];
    const Mat4 fk = forward_kinematics(q);
    f[0] = fk[3] - target[0];
    f[1] = fk[7] - target[1];
    f[2] = fk[11] - target[2];
    f[3] = fk[2] - orientation[0];
    f[4] = fk[6] - orientation[1];
    f[5] = fk[10] - orientation[2];
  };
  const TrfResult result = least_squares_trf(residual, n, m, x0.data(), model_.lower.data(), model_.upper.data());
  q = initial;
  for (int i = 0; i < n; ++i) q[static_cast<std::size_t>(model_.active_links[static_cast<std::size_t>(i)])] = result.x[static_cast<std::size_t>(i)];
  for (std::size_t j = 0; j < sol.size(); ++j) sol[j] = q[static_cast<std::size_t>(model_.index[j])];
  sol[static_cast<std::size_t>(model_.roll)] = roll;
  nb::dict stored;
  for (std::size_t j = 0; j < sol.size(); ++j) stored[names_[j]] = sol[j];
  cache_[cache_key] = stored;
  if (nb::len(cache_) > model_.cache_size) cache_.attr("popitem")(nb::arg("last") = false);
  return sol;
}

bool ArmIk::inverse_flange(const Vec3 &target, double pitch, const Joints &seed_measured,
                           double heading_bias, Joints &out, std::string &error) {
  const double x = target[0], y = target[1], z = target[2];
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(pitch)) {
    error = "target must be finite";
    return false;
  }
  const std::size_t nj = model_.joints.size();
  const std::size_t roll_j = static_cast<std::size_t>(model_.roll);
  Joints seed(nj);
  for (std::size_t j = 0; j < nj; ++j) seed[j] = seed_measured[j] + model_.offsets[j];
  const double roll = std::min(std::max(seed[roll_j], model_.limit_lo[roll_j] + model_.margin),
                               model_.limit_hi[roll_j] - model_.margin);
  const double bearing = std::atan2(y - model_.pan_y, x - model_.pan_x);
  double heading = bearing + heading_bias;
  std::vector<Joints> seeds{seed};
  for (const Vec3 &extra : model_.extra_seeds) {
    Joints s(nj, 0.0);
    s[static_cast<std::size_t>(model_.pan)] = -heading;
    s[static_cast<std::size_t>(model_.lift)] = extra[0];
    s[static_cast<std::size_t>(model_.elbow)] = extra[1];
    s[static_cast<std::size_t>(model_.wrist)] = extra[2];
    seeds.push_back(s);
  }
  double best_error = std::numeric_limits<double>::infinity();
  for (Joints candidate : seeds) {
    candidate[roll_j] = roll;
    Joints sol = clamp_seed(candidate);
    bool has_previous = false;
    double prev_heading = 0.0, prev_gap = 0.0;
    for (int it = 0; it < model_.heading_refinements; ++it) {
      const Vec3 orientation{std::cos(pitch) * std::cos(heading), std::cos(pitch) * std::sin(heading), -std::sin(pitch)};
      sol = solve(target, orientation, sol, roll);
      const Mat4 frame = forward_kinematics(urdf_vector(sol));
      const Vec3 approach{frame[2], frame[6], frame[10]};
      const double err = dist3({frame[3], frame[7], frame[11]}, target);
      const double reached_pitch = std::atan2(-approach[2], std::hypot(approach[0], approach[1]));
      best_error = std::min(best_error, err);
      const bool pitch_ok = std::abs(reached_pitch - pitch) <= model_.pitch_tolerance_rad;
      if (err <= model_.position_tolerance_m && pitch_ok && within_limits(sol)) {
        last_heading_bias = std::remainder(heading - bearing, TAU);
        out.resize(nj);
        for (std::size_t j = 0; j < nj; ++j) out[j] = sol[j] - model_.offsets[j];
        return true;
      }
      if (std::hypot(approach[0], approach[1]) > HEADING_MIN_NORM) {
        const double reached_heading = std::atan2(approach[1], approach[0]);
        const double gap = std::remainder(reached_heading - heading, TAU);
        double next_heading = reached_heading;
        if (has_previous && std::abs(gap - prev_gap) > GAP_MIN_CHANGE) {
          const double slope = (gap - prev_gap) / std::remainder(heading - prev_heading, TAU);
          next_heading = std::abs(slope) > SLOPE_MIN ? heading - gap / slope : reached_heading;
        }
        has_previous = true;
        prev_heading = heading;
        prev_gap = gap;
        heading = next_heading;
      }
    }
  }
  error = format("target (%.3f, %.3f, %.3f) pitch %.2f rad is unreachable within joint limits (closest %.0f mm)", x, y, z,
                 pitch, best_error * 1000);
  return false;
}

bool ArmIk::inverse(const Vec3 &target, double pitch, const Joints &seed,
                    const std::optional<Vec3> &extra, double heading_bias, Joints &out, std::string &error) {
  const double x = target[0], y = target[1], z = target[2];
  Vec3 tool = model_.tool_offset;
  if (extra)
    for (std::size_t k = 0; k < 3; ++k) tool[k] = model_.tool_offset[k] + (*extra)[k];
  const double tool_reach = norm3(tool);
  if (std::sqrt(x * x + y * y + z * z) > model_.max_reach_m + tool_reach + model_.position_tolerance_m) {
    error = format("unreachable: target (%.3f, %.3f, %.3f) is beyond the arm's reach of %.3f m", x, y, z,
                   model_.max_reach_m + tool_reach);
    return false;
  }
  const bool tool_set = model_.tool_offset[0] != 0.0 || model_.tool_offset[1] != 0.0 || model_.tool_offset[2] != 0.0;
  const bool extra_set = extra && ((*extra)[0] != 0.0 || (*extra)[1] != 0.0 || (*extra)[2] != 0.0);
  if (!(tool_set || extra_set)) return inverse_flange(target, pitch, seed, heading_bias, out, error);
  Vec3 flange = target;
  Joints sol;
  if (!inverse_flange(flange, pitch, seed, heading_bias, sol, error)) return false;
  for (int it = 0; it < model_.tool_iterations; ++it) {
    const Vec3 reached = tool_point(sol, extra);
    const Vec3 residual{target[0] - reached[0], target[1] - reached[1], target[2] - reached[2]};
    if (norm3(residual) < model_.tool_convergence_m) break;
    for (std::size_t k = 0; k < 3; ++k) flange[k] = flange[k] + residual[k];
    Joints next;
    if (!inverse_flange(flange, pitch, sol, last_heading_bias, next, error)) return false;
    sol = next;
  }
  const Vec3 reached = tool_point(sol, extra);
  if (dist3(reached, target) > model_.position_tolerance_m) {
    error = format("target (%.3f, %.3f, %.3f) tool point did not converge within joint limits", x, y, z);
    return false;
  }
  out = sol;
  return true;
}

}  // namespace grasp
