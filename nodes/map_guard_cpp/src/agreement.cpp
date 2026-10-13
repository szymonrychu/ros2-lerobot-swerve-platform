#include "map_guard_cpp/agreement.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace map_guard_cpp
{

namespace
{

template <typename T>
Points endpoints(
  const std::vector<T> & ranges, double angle_min, double angle_increment, double range_min, double range_max,
  double max_range_m)
{
  const double upper = std::min(range_max, max_range_m);
  Points out;
  out.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r) || r < range_min || r > upper) {
      continue;
    }
    const double a = angle_min + angle_increment * static_cast<double>(i);
    out.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return out;
}

}  // namespace

Mask dilate(const Mask & mask, int width, int height, int radius)
{
  if (radius <= 0) {
    return mask;
  }
  // Separable: a square (Chebyshev) dilation is a row pass then a column pass of the same radius.
  Mask rows(mask.size(), 0);
  for (int y = 0; y < height; ++y) {
    const std::uint8_t * in = &mask[static_cast<std::size_t>(y) * width];
    std::uint8_t * out = &rows[static_cast<std::size_t>(y) * width];
    for (int x = 0; x < width; ++x) {
      if (in[x]) {
        const int lo = std::max(0, x - radius);
        const int hi = std::min(width - 1, x + radius);
        std::fill(out + lo, out + hi + 1, 1);
      }
    }
  }
  Mask out(mask.size(), 0);
  for (int y = 0; y < height; ++y) {
    const int lo = std::max(0, y - radius);
    const int hi = std::min(height - 1, y + radius);
    std::uint8_t * dst = &out[static_cast<std::size_t>(y) * width];
    for (int yy = lo; yy <= hi; ++yy) {
      const std::uint8_t * src = &rows[static_cast<std::size_t>(yy) * width];
      for (int x = 0; x < width; ++x) {
        dst[x] |= src[x];
      }
    }
  }
  return out;
}

PreparedMap prepare_map(GridMap grid, int occupied_threshold, int hit_radius_cells)
{
  const std::size_t n = grid.cells.size();
  Mask known(n, 0);
  Mask occupied(n, 0);
  long known_cells = 0;  // NOLINT(runtime/int)
  for (std::size_t i = 0; i < n; ++i) {
    const int value = grid.cells[i];
    if (value != UNKNOWN) {
      known[i] = 1;
      ++known_cells;
      occupied[i] = value >= occupied_threshold ? 1 : 0;
    }
  }
  PreparedMap prepared;
  prepared.hit_mask = dilate(occupied, grid.width, grid.height, hit_radius_cells);
  prepared.known_mask = dilate(known, grid.width, grid.height, hit_radius_cells);
  prepared.known_cells = known_cells;
  prepared.grid = std::move(grid);
  return prepared;
}

Points scan_endpoints(
  const std::vector<float> & ranges, double angle_min, double angle_increment, double range_min, double range_max,
  double max_range_m)
{
  return endpoints(ranges, angle_min, angle_increment, range_min, range_max, max_range_m);
}

Points scan_endpoints(
  const std::vector<double> & ranges, double angle_min, double angle_increment, double range_min, double range_max,
  double max_range_m)
{
  return endpoints(ranges, angle_min, angle_increment, range_min, range_max, max_range_m);
}

Quaternion yaw_quaternion(double yaw) { return {0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)}; }

double quaternion_yaw(const Quaternion & q)
{
  const auto [x, y, z, w] = q;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Matrix3 quaternion_matrix(const Quaternion & q)
{
  const auto [x, y, z, w] = q;
  return {{
    {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
    {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
    {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)},
  }};
}

Points transform_points(const Points & points, const Vector3 & translation, const Quaternion & rotation)
{
  const Matrix3 rot = quaternion_matrix(rotation);
  Points out;
  out.reserve(points.size());
  for (const auto & p : points) {
    out.push_back(
      {p.x * rot[0][0] + p.y * rot[0][1] + translation[0], p.x * rot[1][0] + p.y * rot[1][1] + translation[1]});
  }
  return out;
}

std::optional<AgreementResult> agreement(const Points & points_map, const PreparedMap * prepared, int min_points)
{
  if (prepared == nullptr || prepared->known_cells == 0 || points_map.empty()) {
    return std::nullopt;
  }
  const GridMap & grid = prepared->grid;
  const double cos_yaw = std::cos(grid.origin_yaw);
  const double sin_yaw = std::sin(grid.origin_yaw);
  long evaluated = 0;  // NOLINT(runtime/int)
  long agreeing = 0;   // NOLINT(runtime/int)
  for (const auto & p : points_map) {
    const double dx = p.x - grid.origin_x;
    const double dy = p.y - grid.origin_y;
    const double col_f = std::floor((cos_yaw * dx + sin_yaw * dy) / grid.resolution);
    const double row_f = std::floor((-sin_yaw * dx + cos_yaw * dy) / grid.resolution);
    if (!(col_f >= 0 && col_f < grid.width && row_f >= 0 && row_f < grid.height)) {
      continue;
    }
    const std::size_t index = static_cast<std::size_t>(row_f) * grid.width + static_cast<std::size_t>(col_f);
    if (prepared->known_mask[index]) {
      ++evaluated;
      agreeing += prepared->hit_mask[index];
    }
  }
  if (evaluated < min_points) {
    return std::nullopt;
  }
  return AgreementResult{static_cast<double>(agreeing) / static_cast<double>(evaluated), evaluated, agreeing};
}

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  if (n % 2 == 1) {
    return values[n / 2];
  }
  return (values[n / 2 - 1] + values[n / 2]) / 2;
}

MismatchMonitor::MismatchMonitor(
  double mismatch_ratio, double healthy_ratio, double mismatch_s,
  long min_map_cells_known,  // NOLINT(runtime/int)
  int window)
: mismatch_ratio_(mismatch_ratio),
  healthy_ratio_(healthy_ratio),
  min_map_cells_known_(min_map_cells_known),
  window_(static_cast<std::size_t>(window)),
  hold_(mismatch_s)
{
}

std::optional<double> MismatchMonitor::median() const
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  return map_guard_cpp::median(std::vector<double>(samples_.begin(), samples_.end()));
}

bool MismatchMonitor::update(double now, double ratio, long known_cells)  // NOLINT(runtime/int)
{
  samples_.push_back(ratio);
  while (samples_.size() > window_) {
    samples_.pop_front();
  }
  const double med = *median();
  const bool big_enough = known_cells >= min_map_cells_known_;
  if (big_enough && med >= healthy_ratio_) {
    armed_ = true;
  }
  return hold_.update(now, armed_ && big_enough && med < mismatch_ratio_);
}

void MismatchMonitor::reset_hold() { hold_.reset(); }

void MismatchMonitor::reset()
{
  samples_.clear();
  hold_.reset();
  armed_ = false;
}

}  // namespace map_guard_cpp
