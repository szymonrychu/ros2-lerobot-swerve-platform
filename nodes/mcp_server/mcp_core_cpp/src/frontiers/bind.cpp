// Python bindings of the frontiers kernel family (submodule mcp_core_cpp.frontiers): traversability, frontier_field,
// geodesic_from and tracker_update, called by mcp_server/frontiers.py. Arrays are row-major numpy arrays; results are
// new numpy arrays owned by Python. The GIL is released while a kernel runs.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

#include "families.hpp"
#include "frontiers/grid_kernels.hpp"

namespace nb = nanobind;
using namespace nb::literals;

namespace {

using Map = nb::ndarray<const int16_t, nb::ndim<2>, nb::c_contig, nb::device::cpu>;
using Mask = nb::ndarray<const bool, nb::ndim<2>, nb::c_contig, nb::device::cpu>;
using Cells = nb::ndarray<const int64_t, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;
using Clock = std::chrono::steady_clock;

// A numpy array of the given shape taking ownership of data.
template <typename T>
nb::object to_numpy(std::unique_ptr<T[]> data, std::size_t rows, std::size_t cols) {
  T *raw = data.release();
  nb::capsule owner(raw, [](void *p) noexcept { delete[] static_cast<T *>(p); });
  return nb::cast(nb::ndarray<nb::numpy, T, nb::ndim<2>>(raw, {rows, cols}, owner));
}

// A numpy array copied from a vector (rows x cols).
template <typename T>
nb::object vector_to_numpy(const std::vector<T> &values, std::size_t rows, std::size_t cols) {
  std::unique_ptr<T[]> data(new T[values.size()]);
  std::copy(values.begin(), values.end(), data.get());
  return to_numpy(std::move(data), rows, cols);
}

std::vector<frontiers::Cluster> clusters_of(const std::vector<Cells> &cells, const std::vector<int32_t> &labels) {
  if (cells.size() != labels.size()) throw nb::value_error("cells and labels must have the same length");
  std::vector<frontiers::Cluster> clusters;
  clusters.reserve(cells.size());
  for (std::size_t k = 0; k < cells.size(); ++k) clusters.push_back({cells[k].data(), cells[k].shape(0), labels[k]});
  return clusters;
}

// Reject a cell outside the map before any kernel indexes with it.
void check_cells(const std::vector<frontiers::Cluster> &clusters, int h, int w) {
  for (const frontiers::Cluster &cluster : clusters) {
    for (std::size_t k = 0; k < cluster.n; ++k) {
      const int64_t r = cluster.cells[2 * k];
      const int64_t c = cluster.cells[2 * k + 1];
      if (r < 0 || r >= h || c < 0 || c >= w) throw nb::value_error("frontier cell outside the map");
    }
  }
}

nb::dict field_dict(const frontiers::Field &field) {
  nb::dict out;
  out["row0"] = field.row0;
  out["col0"] = field.col0;
  out["distance_m"] = vector_to_numpy(field.distance_m, field.h, field.w);
  out["label"] = vector_to_numpy(field.label, field.h, field.w);
  out["nearest_row"] = vector_to_numpy(field.nearest_row, field.h, field.w);
  out["nearest_col"] = vector_to_numpy(field.nearest_col, field.h, field.w);
  return out;
}

double elapsed_ms(Clock::time_point from, Clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

nb::tuple traversability(const Map &data, double resolution, double pass_clearance_m, int free_threshold,
                         int occupied_threshold) {
  const int h = static_cast<int>(data.shape(0));
  const int w = static_cast<int>(data.shape(1));
  const std::size_t size = static_cast<std::size_t>(h) * w;
  std::unique_ptr<bool[]> trav(new bool[size]);
  std::unique_ptr<float[]> clearance(new float[size]);
  {
    nb::gil_scoped_release release;
    frontiers::traversability(data.data(), h, w, resolution, pass_clearance_m, free_threshold, occupied_threshold,
                              trav.get(), clearance.get());
  }
  return nb::make_tuple(to_numpy(std::move(trav), h, w), to_numpy(std::move(clearance), h, w));
}

nb::dict frontier_field(const Map &data, const std::vector<Cells> &cells, const std::vector<int32_t> &labels,
                        double resolution) {
  const int h = static_cast<int>(data.shape(0));
  const int w = static_cast<int>(data.shape(1));
  const std::vector<frontiers::Cluster> clusters = clusters_of(cells, labels);
  check_cells(clusters, h, w);
  frontiers::Field field;
  {
    nb::gil_scoped_release release;
    field = frontiers::frontier_field(data.data(), h, w, clusters, resolution);
  }
  return field_dict(field);
}

nb::object geodesic_from(const Mask &trav, int snapped_row, int snapped_col, double snap_m, int downsample,
                         double resolution) {
  const int h = static_cast<int>(trav.shape(0));
  const int w = static_cast<int>(trav.shape(1));
  if (downsample < 1) throw nb::value_error("downsample must be >= 1");
  if (snapped_row < 0 || snapped_row >= h || snapped_col < 0 || snapped_col >= w ||
      !trav.data()[static_cast<std::size_t>(snapped_row) * w + snapped_col]) {
    throw nb::value_error("the snapped start must be a traversable cell");
  }
  std::unique_ptr<double[]> out(new double[static_cast<std::size_t>(h) * w]);
  {
    nb::gil_scoped_release release;
    frontiers::geodesic_from(trav.data(), h, w, snapped_row, snapped_col, snap_m, downsample, resolution, out.get());
  }
  return to_numpy(std::move(out), h, w);
}

nb::dict tracker_update(const Map &data, const std::vector<Cells> &cells, const std::vector<int32_t> &labels,
                        double resolution, double pass_clearance_m, int free_threshold, int occupied_threshold) {
  const int h = static_cast<int>(data.shape(0));
  const int w = static_cast<int>(data.shape(1));
  const std::size_t size = static_cast<std::size_t>(h) * w;
  const std::vector<frontiers::Cluster> clusters = clusters_of(cells, labels);
  check_cells(clusters, h, w);
  std::unique_ptr<bool[]> trav(new bool[size]);
  std::unique_ptr<float[]> clearance(new float[size]);
  frontiers::Field field;
  double trav_ms = 0.0, field_ms = 0.0;
  {
    nb::gil_scoped_release release;
    const Clock::time_point t0 = Clock::now();
    frontiers::traversability(data.data(), h, w, resolution, pass_clearance_m, free_threshold, occupied_threshold,
                              trav.get(), clearance.get());
    const Clock::time_point t1 = Clock::now();
    field = frontiers::frontier_field(data.data(), h, w, clusters, resolution);
    const Clock::time_point t2 = Clock::now();
    trav_ms = elapsed_ms(t0, t1);
    field_ms = elapsed_ms(t1, t2);
  }
  nb::dict out;
  out["trav"] = to_numpy(std::move(trav), h, w);
  out["clearance_m"] = to_numpy(std::move(clearance), h, w);
  out["field"] = field_dict(field);
  out["traversability_ms"] = trav_ms;
  out["field_ms"] = field_ms;
  return out;
}

}  // namespace

void register_frontiers(nb::module_ &m) {
  m.def("traversability", &traversability, "data"_a, "resolution"_a, "pass_clearance_m"_a, "free_threshold"_a,
        "occupied_threshold"_a,
        "(trav bool H x W, clearance_m float32 H x W) of an int16 occupancy grid (frontiers.traversability).");
  m.def("frontier_field", &frontier_field, "data"_a, "cells"_a, "labels"_a, "resolution"_a,
        "Distance field to the nearest frontier cell over the known box: dict row0, col0, distance_m, label, "
        "nearest_row, nearest_col (frontiers.frontier_field).");
  m.def("geodesic_from", &geodesic_from, "trav"_a, "snapped_row"_a, "snapped_col"_a, "snap_m"_a, "downsample"_a,
        "resolution"_a,
        "float64 H x W geodesic distance (m) from a traversable start cell, inf where unreachable "
        "(frontiers.geodesic_from after the snap).");
  m.def("tracker_update", &tracker_update, "data"_a, "cells"_a, "labels"_a, "resolution"_a, "pass_clearance_m"_a,
        "free_threshold"_a, "occupied_threshold"_a,
        "The traversability and frontier-field stages of FrontierTracker.update in one call: dict trav, "
        "clearance_m, field (as frontier_field), traversability_ms, field_ms.");
}
