// Python bindings of the camera kernel family (submodule mcp_core_cpp.camera): the kernels bench/report.json ranks
// above the threshold (camera_overlay.draw_lidar, perception.map_stats, perception.render_map_crop). The entry points in
// mcp_server call these; their Python originals are mcp_server/reference/camera.py.
#include <cstdint>
#include <initializer_list>
#include <utility>
#include <vector>

#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

#include "camera/grid.hpp"
#include "camera/lidar.hpp"
#include "families.hpp"

namespace nb = nanobind;
using namespace nb::literals;

namespace
{

using Points = nb::ndarray<const double, nb::shape<-1, 4>, nb::c_contig, nb::device::cpu>;
using Mat4 = nb::ndarray<const double, nb::shape<4, 4>, nb::c_contig, nb::device::cpu>;
using Mat3 = nb::ndarray<const double, nb::shape<3, 3>, nb::c_contig, nb::device::cpu>;

// A numpy array owning the vector's data.
template <typename T>
nb::ndarray<nb::numpy, T> to_numpy(std::vector<T> && values, std::initializer_list<size_t> shape)
{
  auto * owned = new std::vector<T>(std::move(values));
  nb::capsule owner(owned, [](void * p) noexcept {delete static_cast<std::vector<T> *>(p);});
  return nb::ndarray<nb::numpy, T>(owned->data(), shape, owner);
}

nb::tuple lidar_dots(
  const Points & points, const Mat4 & t_base_ref, const Mat4 & t_ref_optical, const Mat3 & camera_matrix,
  const std::vector<double> & distortion, int64_t width, int64_t height, double max_range_m)
{
  mcp_core::camera::LidarDots dots;
  {
    nb::gil_scoped_release release;
    dots = mcp_core::camera::lidar_dots(
      points.data(), points.shape(0), t_base_ref.data(), t_ref_optical.data(), camera_matrix.data(), distortion, width,
      height, max_range_m);
  }
  const size_t m = dots.values.size();
  return nb::make_tuple(to_numpy(std::move(dots.centres), {m, 2}), to_numpy(std::move(dots.values), {m}));
}

nb::dict map_counts(nb::handle data, int16_t occupied_threshold, int16_t free_threshold)
{
  const mcp_core::camera::MapCounts counts = mcp_core::camera::map_counts(data, occupied_threshold, free_threshold);
  nb::dict out;
  out["known_cells"] = counts.known;
  out["occupied_cells"] = counts.occupied;
  out["free_cells"] = counts.free;
  return out;
}

nb::ndarray<nb::numpy, uint8_t> map_crop(
  nb::handle data, int64_t width, int64_t height, int64_t x0, int64_t y0, int64_t size, int16_t occupied_threshold,
  int16_t free_threshold, uint8_t unknown_grey, uint8_t free_grey, uint8_t occupied_grey)
{
  const mcp_core::camera::Occupancy occupancy{occupied_threshold, free_threshold, unknown_grey, free_grey, occupied_grey};
  std::vector<uint8_t> crop = mcp_core::camera::map_crop(data, width, height, x0, y0, size, occupancy);
  const auto side = static_cast<size_t>(size);
  return to_numpy(std::move(crop), {side, side});
}

}  // namespace

void register_camera(nb::module_ & m)
{
  m.def(
    "lidar_dots", &lidar_dots, "points"_a, "t_base_ref"_a, "t_ref_optical"_a, "camera_matrix"_a, "distortion"_a,
    "width"_a, "height"_a, "max_range_m"_a,
    "Visible lidar dots of camera_overlay.draw_lidar.\n\n"
    "points: (N, 4) float64 rows (x, y, z, range) in base_link; t_base_ref, t_ref_optical: 4x4 float64; camera_matrix:"
    " 3x3 float64; distortion: 0, 4, 5, 8, 12 or 14 OpenCV coefficients; width, height: image size; max_range_m: range"
    " at the far end of the colormap.\n"
    "Returns (centres (M, 2) int32 rounded pixels, values (M,) uint8 JET colormap indices) of the points in front of"
    " the camera and inside the image, in input order.");
  m.def(
    "map_counts", &map_counts, "data"_a, "occupied_threshold"_a, "free_threshold"_a,
    "Known, occupied and free cells of an occupancy grid (perception.map_stats).\n\n"
    "data: row-major cells as a buffer (array.array, numpy) or a sequence of ints; occupied_threshold, free_threshold:"
    " perception.OCCUPIED_THRESHOLD / FREE_THRESHOLD (occupied: cell >= occupied_threshold; free: 0 <= cell <="
    " free_threshold).\n"
    "Returns {'known_cells', 'occupied_cells', 'free_cells'}.");
  m.def(
    "map_crop", &map_crop, "data"_a, "width"_a, "height"_a, "x0"_a, "y0"_a, "size"_a,
    "occupied_threshold"_a, "free_threshold"_a, "unknown_grey"_a, "free_grey"_a, "occupied_grey"_a,
    "Grey crop of an occupancy grid (perception.render_map_crop).\n\n"
    "data: row-major width x height cells (buffer or sequence of ints); (x0, y0): bottom-left cell of the crop; size:"
    " crop side in cells; occupied_threshold, free_threshold, unknown_grey, free_grey, occupied_grey: the"
    " perception.py constants of the same names.\n"
    "Returns a (size, size) uint8 array, north up (first row = largest y): free_grey free, occupied_grey occupied,"
    " unknown_grey unknown or off the map.");
}
