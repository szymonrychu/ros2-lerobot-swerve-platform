// Frontier grid kernels (mcp_server/frontiers.py): traversability, the frontier distance field, geodesic distance and
// the fused per-map-version stages of FrontierTracker.update. Plain C++ on row-major arrays, no Python types; the
// bindings live in bind.cpp. Each kernel reproduces its Python reference (mcp_server/reference/frontiers.py) exactly:
// integer squared distances for the EDTs, scipy's Voronoi feature transform tie-breaking for the nearest frontier cell.
#pragma once

#include <cstdint>
#include <vector>

namespace frontiers {

// Bounding box of the known cells (value >= 0) grown by margin and clipped to the grid; ends exclusive.
struct Box {
  bool valid = false;
  int r0 = 0;
  int c0 = 0;
  int r1 = 0;
  int c1 = 0;
};

// One frontier cluster: n (row, col) cells (row-major int64 n x 2) and its label value.
struct Cluster {
  const int64_t *cells = nullptr;
  std::size_t n = 0;
  int32_t label = 0;
};

// Frontier distance field over the known box (FrontierField); empty (h = w = 0) without frontiers or known cells.
struct Field {
  int row0 = 0;
  int col0 = 0;
  int h = 0;
  int w = 0;
  std::vector<double> distance_m;
  std::vector<int32_t> label;
  std::vector<int32_t> nearest_row;
  std::vector<int32_t> nearest_col;
};

// Known-cell bounding box of data (h x w) grown by margin cells on every side.
Box known_bbox(const int16_t *data, int h, int w, int margin);

// Traversable cells and obstacle clearance (frontiers.traversability): clearance[i] = max(edt - 0.5, 0) * resolution
// (float32) over the known box + 1 with unknown, occupied (>= occupied_threshold) and off-map cells as obstacles;
// trav[i] = 0 <= data[i] <= free_threshold and clearance[i] >= (float)pass_clearance_m. trav and clearance hold h x w.
void traversability(const int16_t *data, int h, int w, double resolution, double pass_clearance_m,
                    int free_threshold, int occupied_threshold, bool *trav, float *clearance);

// Distance from every known-box cell to the nearest frontier cell (frontiers.frontier_field), with that cell and its
// label; ties go to the cell scipy.ndimage.distance_transform_edt(return_indices=True) picks.
Field frontier_field(const int16_t *data, int h, int w, const std::vector<Cluster> &clusters, double resolution);

// Geodesic distance (m) from the snapped start cell (sr, sc) through traversable cells (frontiers.geodesic_from after
// the snap): 8-neighbour Dijkstra on a downsample x coarser grid over the traversable bounding box, plus snap_m;
// out (h x w) is inf where unreachable or not traversable.
void geodesic_from(const bool *trav, int h, int w, int sr, int sc, double snap_m, int downsample, double resolution,
                   double *out);

}  // namespace frontiers
