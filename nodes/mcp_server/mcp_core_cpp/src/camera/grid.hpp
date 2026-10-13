// Occupancy grid kernels of perception.py (map_stats, render_map_crop): cell counts and the grey crop around the robot.
// The grid arrives as Python passes it: a buffer (array.array('b') from rclpy, numpy) or a sequence of ints (list).
#pragma once

#include <cstdint>
#include <vector>

#include <nanobind/nanobind.h>

namespace mcp_core::camera
{

// Thresholds and grey levels of perception.py (OCCUPIED_THRESHOLD, FREE_THRESHOLD, *_GREY): perception.py is their only
// definition, its callers pass them in, so this code holds no copy that could drift.
struct Occupancy
{
  int16_t occupied_threshold;  // cells >= this are occupied
  int16_t free_threshold;      // cells in 0..this are free
  uint8_t unknown_grey;
  uint8_t free_grey;
  uint8_t occupied_grey;
};

struct MapCounts
{
  int64_t known = 0;     // cells >= 0
  int64_t occupied = 0;  // cells >= OCCUPIED_THRESHOLD
  int64_t free = 0;      // cells in 0..FREE_THRESHOLD
};

// Known, occupied and free cells of a grid (every cell read as numpy's int16 conversion would).
// Throws nanobind::type_error for a non-integer item and raises OverflowError for an item outside int16.
MapCounts map_counts(nanobind::handle data, int16_t occupied_threshold, int16_t free_threshold);

// Grey crop of size x size cells whose bottom-left cell is (x0, y0), flipped so the first row is the top (north up).
// Cells outside the grid are occupancy.unknown_grey. Returns size * size bytes, row-major.
// Throws nanobind::value_error when the grid does not hold width x height cells (numpy's reshape error).
std::vector<uint8_t> map_crop(
  nanobind::handle data, int64_t width, int64_t height, int64_t x0, int64_t y0, int64_t size,
  const Occupancy & occupancy);

}  // namespace mcp_core::camera
