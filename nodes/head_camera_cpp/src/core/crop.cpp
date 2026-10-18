#include "head_camera_cpp/crop.hpp"

#include <algorithm>
#include <cmath>

namespace head_camera_cpp
{

namespace
{

constexpr int EVEN = 2;

int floor_even(int value) { return value - (value % EVEN); }

int round_even(double value) { return EVEN * static_cast<int>(std::lround(value / EVEN)); }

bool in_unit(double value) { return value >= 0.0 && value <= 1.0; }

// Top-left coordinate of a span of `length` centred on `centre`, shifted inside [0, total] and rounded down to even.
int place(double centre, int length, int total)
{
  const int start = static_cast<int>(std::lround(centre - length / 2.0));
  const int max_start = floor_even(total - length);
  return floor_even(std::clamp(start, 0, std::max(max_start, 0)));
}

}  // namespace

Rect zoom_region(double u, double v, int zoom, Size full)
{
  if (!in_unit(u) || !in_unit(v)) {
    throw std::invalid_argument("zoom point must be within [0, 1]");
  }
  if (zoom < 1) {
    throw std::invalid_argument("zoom factor must be >= 1");
  }
  if (full.width < EVEN || full.height < EVEN) {
    throw std::invalid_argument("full frame size too small");
  }
  const int width = std::max(floor_even(full.width / zoom), EVEN);
  const int height = std::max(floor_even(full.height / zoom), EVEN);
  return Rect{place(u * full.width, width, full.width), place(v * full.height, height, full.height), width, height};
}

Size output_size(Rect region, int max_px)
{
  if (max_px < 1) {
    throw std::invalid_argument("max_px must be >= 1");
  }
  if (region.width < 1 || region.height < 1) {
    throw std::invalid_argument("region must not be empty");
  }
  const int longest = std::max(region.width, region.height);
  const double scale = std::min(1.0, static_cast<double>(max_px) / longest);
  const int cap = std::max(floor_even(max_px), EVEN);
  const int max_w = std::min(cap, std::max(floor_even(region.width), EVEN));
  const int max_h = std::min(cap, std::max(floor_even(region.height), EVEN));
  const int width = std::clamp(round_even(region.width * scale), EVEN, max_w);
  const int height = std::clamp(round_even(region.height * scale), EVEN, max_h);
  return Size{width, height};
}

Rect af_window(double u, double v, double fraction, Rect scaler_crop_max)
{
  if (!in_unit(u) || !in_unit(v)) {
    throw std::invalid_argument("af point must be within [0, 1]");
  }
  if (!(fraction > 0.0 && fraction <= 1.0)) {
    throw std::invalid_argument("af fraction must be within (0, 1]");
  }
  if (scaler_crop_max.width < 1 || scaler_crop_max.height < 1) {
    throw std::invalid_argument("scaler crop must not be empty");
  }
  const int width = std::max(1, static_cast<int>(std::lround(fraction * scaler_crop_max.width)));
  const int height = std::max(1, static_cast<int>(std::lround(fraction * scaler_crop_max.height)));
  const int x = scaler_crop_max.x + static_cast<int>(std::lround(u * scaler_crop_max.width - width / 2.0));
  const int y = scaler_crop_max.y + static_cast<int>(std::lround(v * scaler_crop_max.height - height / 2.0));
  return Rect{
    std::clamp(x, scaler_crop_max.x, scaler_crop_max.x + scaler_crop_max.width - width),
    std::clamp(y, scaler_crop_max.y, scaler_crop_max.y + scaler_crop_max.height - height), width, height};
}

}  // namespace head_camera_cpp
