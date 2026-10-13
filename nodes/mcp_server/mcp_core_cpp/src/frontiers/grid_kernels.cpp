// Frontier grid kernels; see grid_kernels.hpp.
#include "frontiers/grid_kernels.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace frontiers {

namespace {

constexpr int64_t NO_SITE = -1;
constexpr double INF = std::numeric_limits<double>::infinity();
const double SQRT2 = std::sqrt(2.0);

// Lower envelope of parabolas (Felzenszwalb & Huttenlocher): d[q] = min_p (q - p)^2 + f[p] for q in [0, n).
// Every f is finite (squared distances of a column scan that always reaches a background cell).
void squared_dt_1d(const std::vector<int64_t> &f, std::vector<int64_t> &d, std::vector<int> &v, std::vector<double> &z) {
  const int n = static_cast<int>(f.size());
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;
  for (int q = 1; q < n; ++q) {
    double s;
    while (true) {
      const int p = v[k];
      s = (static_cast<double>(f[q] + static_cast<int64_t>(q) * q) - static_cast<double>(f[p] + static_cast<int64_t>(p) * p)) /
          (2.0 * q - 2.0 * p);
      if (s > z[k]) break;
      --k;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INF;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    const int64_t dq = q - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

// Squared Euclidean distance (cells) from every open cell of an h x w mask to the nearest closed cell, where every cell
// outside the mask counts as closed (the np.pad(..., constant_values=False) of the reference).
std::vector<int64_t> squared_edt_closed_border(const std::vector<uint8_t> &open, int h, int w) {
  // Columns, scanned row by row (cache friendly): distance to the closed cell above (row -1 is closed), then the
  // smaller of that and the distance to the closed cell below (row h is closed).
  std::vector<int64_t> g(static_cast<std::size_t>(h) * w);
  std::vector<int64_t> run(w, 0);
  for (int r = 0; r < h; ++r) {
    const std::size_t base = static_cast<std::size_t>(r) * w;
    for (int c = 0; c < w; ++c) {
      run[c] = open[base + c] ? run[c] + 1 : 0;
      g[base + c] = run[c];
    }
  }
  std::fill(run.begin(), run.end(), 0);
  for (int r = h - 1; r >= 0; --r) {
    const std::size_t base = static_cast<std::size_t>(r) * w;
    for (int c = 0; c < w; ++c) {
      run[c] = open[base + c] ? run[c] + 1 : 0;
      g[base + c] = std::min(g[base + c], run[c]);
    }
  }
  // Rows: columns -1 and w are closed sites at distance 0, so the 1D transform runs over w + 2 positions.
  const int n = w + 2;
  std::vector<int64_t> f(n), d(n);
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  std::vector<int64_t> out(static_cast<std::size_t>(h) * w);
  for (int r = 0; r < h; ++r) {
    f[0] = 0;
    f[n - 1] = 0;
    for (int c = 0; c < w; ++c) {
      const int64_t gi = g[static_cast<std::size_t>(r) * w + c];
      f[c + 1] = gi * gi;
    }
    squared_dt_1d(f, d, v, z);
    for (int c = 0; c < w; ++c) out[static_cast<std::size_t>(r) * w + c] = d[c + 1];
  }
  return out;
}

// One line of scipy's _VoronoiFT (ni_morphology.c) along axis 1 of a 2D feature transform, for row `row`: sites are
// the entries with fr[i] >= 0 (feature row fr[i], feature column fc[i]); every entry gets the feature of its nearest
// site, ties to the earlier site, exactly as scipy decides them (integer arithmetic, exact here).
void voronoi_row(int64_t row, int n, int64_t *fr, int64_t *fc, std::vector<int64_t> &sr, std::vector<int64_t> &sc,
                 std::vector<int> &g) {
  for (int i = 0; i < n; ++i) {
    sr[i] = fr[i];
    sc[i] = fc[i];
  }
  int l = -1;
  for (int i = 0; i < n; ++i) {
    if (sr[i] < 0) continue;
    const int64_t fd = sc[i];
    const int64_t tw = sr[i] - row;
    const int64_t wr = tw * tw;
    while (l >= 1) {
      const int idx1 = g[l];
      const int idx2 = g[l - 1];
      const int64_t f1 = sc[idx1];
      const int64_t a = f1 - sc[idx2];
      const int64_t b = fd - f1;
      const int64_t c = a + b;
      const int64_t tu = sr[idx2] - row;
      const int64_t tv = sr[idx1] - row;
      const int64_t ur = tu * tu;
      const int64_t vr = tv * tv;
      if (c * vr - b * ur - a * wr - a * b * c <= 0) break;
      --l;
    }
    ++l;
    g[l] = i;
  }
  const int maxl = l;
  if (maxl < 0) return;
  l = 0;
  for (int i = 0; i < n; ++i) {
    auto delta = [&](int site) {
      const int64_t dr = sr[site] - row;
      const int64_t dc = sc[site] - i;
      return dr * dr + dc * dc;
    };
    int64_t delta1 = delta(g[l]);
    while (l < maxl) {
      const int64_t delta2 = delta(g[l + 1]);
      if (delta1 <= delta2) break;
      delta1 = delta2;
      ++l;
    }
    fr[i] = sr[g[l]];
    fc[i] = sc[g[l]];
  }
}

}  // namespace

Box known_bbox(const int16_t *data, int h, int w, int margin) {
  Box box;
  int rmin = h, rmax = -1, cmin = w, cmax = -1;
  for (int r = 0; r < h; ++r) {
    const int16_t *row = data + static_cast<std::size_t>(r) * w;
    int first = -1, last = -1;
    for (int c = 0; c < w; ++c) {
      if (row[c] >= 0) {
        if (first < 0) first = c;
        last = c;
      }
    }
    if (first < 0) continue;
    rmin = std::min(rmin, r);
    rmax = r;
    cmin = std::min(cmin, first);
    cmax = std::max(cmax, last);
  }
  if (rmax < 0) return box;
  box.valid = true;
  box.r0 = std::max(rmin - margin, 0);
  box.c0 = std::max(cmin - margin, 0);
  box.r1 = std::min(rmax + margin + 1, h);
  box.c1 = std::min(cmax + margin + 1, w);
  return box;
}

void traversability(const int16_t *data, int h, int w, double resolution, double pass_clearance_m,
                    int free_threshold, int occupied_threshold, bool *trav, float *clearance) {
  const std::size_t size = static_cast<std::size_t>(h) * w;
  std::fill(clearance, clearance + size, 0.0f);
  const Box box = known_bbox(data, h, w, 1);
  if (box.valid) {
    const int bh = box.r1 - box.r0;
    const int bw = box.c1 - box.c0;
    std::vector<uint8_t> open(static_cast<std::size_t>(bh) * bw);
    for (int r = 0; r < bh; ++r) {
      const int16_t *row = data + static_cast<std::size_t>(r + box.r0) * w + box.c0;
      for (int c = 0; c < bw; ++c) open[static_cast<std::size_t>(r) * bw + c] = row[c] >= 0 && row[c] < occupied_threshold;
    }
    const std::vector<int64_t> d2 = squared_edt_closed_border(open, bh, bw);
    for (int r = 0; r < bh; ++r) {
      float *out = clearance + static_cast<std::size_t>(r + box.r0) * w + box.c0;
      for (int c = 0; c < bw; ++c) {
        const double cells = std::sqrt(static_cast<double>(d2[static_cast<std::size_t>(r) * bw + c]));
        out[c] = static_cast<float>(std::max(cells - 0.5, 0.0) * resolution);
      }
    }
  }
  // The reference compares the float32 clearance with the Python float as float32 (NumPy 2 weak scalar promotion).
  const float pass = static_cast<float>(pass_clearance_m);
  for (std::size_t i = 0; i < size; ++i) trav[i] = data[i] >= 0 && data[i] <= free_threshold && clearance[i] >= pass;
}

Field frontier_field(const int16_t *data, int h, int w, const std::vector<Cluster> &clusters, double resolution) {
  Field field;
  const Box box = known_bbox(data, h, w, 1);
  if (!box.valid || clusters.empty()) return field;
  const int fh = box.r1 - box.r0;
  const int fw = box.c1 - box.c0;
  const std::size_t size = static_cast<std::size_t>(fh) * fw;
  field.row0 = box.r0;
  field.col0 = box.c0;
  field.h = fh;
  field.w = fw;
  std::vector<int32_t> label(size, 0);
  for (const Cluster &cluster : clusters) {
    for (std::size_t k = 0; k < cluster.n; ++k) {
      const int64_t r = cluster.cells[2 * k] - box.r0;
      const int64_t c = cluster.cells[2 * k + 1] - box.c0;
      label[static_cast<std::size_t>(r) * fw + c] = cluster.label;
    }
  }
  // Feature transform (row, col of the nearest frontier cell), scipy's two passes: along each column (axis 0), then
  // along each row (axis 1). In the column pass every site has the same column, so the nearest site wins, ties to
  // the site above (the earlier one), which two linear scans give directly.
  // Both scans run row by row over all columns at once (cache friendly): fr first holds the site above, then the
  // nearer of it and the site below.
  std::vector<int64_t> fr(size, NO_SITE), fc(size, NO_SITE);
  std::vector<int64_t> last(fw, NO_SITE);
  for (int r = 0; r < fh; ++r) {
    const std::size_t base = static_cast<std::size_t>(r) * fw;
    for (int c = 0; c < fw; ++c) {
      if (label[base + c] != 0) last[c] = r;
      fr[base + c] = last[c];
    }
  }
  std::fill(last.begin(), last.end(), NO_SITE);
  for (int r = fh - 1; r >= 0; --r) {
    const std::size_t base = static_cast<std::size_t>(r) * fw;
    for (int c = 0; c < fw; ++c) {
      if (label[base + c] != 0) last[c] = r;
      const int64_t above = fr[base + c];
      const int64_t below = last[c];
      const int64_t site = (above == NO_SITE || (below != NO_SITE && below - r < r - above)) ? below : above;
      fr[base + c] = site;
      fc[base + c] = site == NO_SITE ? NO_SITE : c;
    }
  }
  std::vector<int64_t> sr(fw), sc(fw);
  std::vector<int> g(fw);
  for (int r = 0; r < fh; ++r) {
    voronoi_row(r, fw, fr.data() + static_cast<std::size_t>(r) * fw, fc.data() + static_cast<std::size_t>(r) * fw, sr, sc, g);
  }
  field.distance_m.resize(size);
  field.label.resize(size);
  field.nearest_row.resize(size);
  field.nearest_col.resize(size);
  for (int r = 0; r < fh; ++r) {
    for (int c = 0; c < fw; ++c) {
      const std::size_t i = static_cast<std::size_t>(r) * fw + c;
      const int64_t dr = fr[i] - r;
      const int64_t dc = fc[i] - c;
      field.distance_m[i] = std::sqrt(static_cast<double>(dr * dr + dc * dc)) * resolution;
      field.label[i] = label[static_cast<std::size_t>(fr[i]) * fw + fc[i]];
      field.nearest_row[i] = static_cast<int32_t>(fr[i] + box.r0);
      field.nearest_col[i] = static_cast<int32_t>(fc[i] + box.c0);
    }
  }
  return field;
}

void geodesic_from(const bool *trav, int h, int w, int sr, int sc, double snap_m, int downsample, double resolution,
                   double *out) {
  const std::size_t size = static_cast<std::size_t>(h) * w;
  std::fill(out, out + size, INF);
  int r0 = h, r1 = 0, c0 = w, c1 = 0;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (!trav[static_cast<std::size_t>(r) * w + c]) continue;
      r0 = std::min(r0, r);
      r1 = r + 1;
      c0 = std::min(c0, c);
      c1 = std::max(c1, c + 1);
    }
  }
  if (r1 <= r0) return;
  const int ds = downsample;
  const int ch = (r1 - r0 + ds - 1) / ds;
  const int cw = (c1 - c0 + ds - 1) / ds;
  // Coarse grid with a one-cell blocked border, so neighbours need no bounds checks.
  const int pw = cw + 2;
  const std::size_t nodes = static_cast<std::size_t>(ch + 2) * pw;
  std::vector<uint8_t> coarse(nodes, 0);
  for (int r = r0; r < r1; ++r) {
    const bool *row = trav + static_cast<std::size_t>(r) * w;
    uint8_t *crow = coarse.data() + static_cast<std::size_t>((r - r0) / ds + 1) * pw + 1;
    for (int c = c0; c < c1; ++c) crow[(c - c0) / ds] |= static_cast<uint8_t>(row[c]);
  }
  // Dijkstra with a bucket queue of width 1 (Dial): every edge costs at least 1, so no node of bucket floor(d) can
  // lower another node of the same bucket, and a bucket's nodes are final in any order. A node may sit in several
  // buckets after a decrease; only the entry of its current bucket is expanded.
  std::vector<double> dist(nodes, INF);
  std::vector<uint8_t> done(nodes, 0);
  std::vector<std::vector<int>> buckets(1);
  const int offsets[8] = {-pw - 1, -pw, -pw + 1, -1, 1, pw - 1, pw, pw + 1};
  const double costs[8] = {SQRT2, 1.0, SQRT2, 1.0, 1.0, SQRT2, 1.0, SQRT2};
  const int start = ((sr - r0) / ds + 1) * pw + (sc - c0) / ds + 1;
  dist[start] = 0.0;
  buckets[0].push_back(start);
  for (std::size_t b = 0; b < buckets.size(); ++b) {
    for (std::size_t q = 0; q < buckets[b].size(); ++q) {
      const int u = buckets[b][q];
      if (done[u] || static_cast<std::size_t>(dist[u]) != b) continue;
      done[u] = 1;
      const double du = dist[u];
      for (int k = 0; k < 8; ++k) {
        const int v = u + offsets[k];
        if (!coarse[v] || done[v]) continue;
        const double nd = du + costs[k];
        if (nd < dist[v]) {
          dist[v] = nd;
          const std::size_t nb = static_cast<std::size_t>(nd);
          if (nb >= buckets.size()) buckets.resize(nb + 1);
          buckets[nb].push_back(v);
        }
      }
    }
    std::vector<int>().swap(buckets[b]);
  }
  for (int r = r0; r < r1; ++r) {
    for (int c = c0; c < c1; ++c) {
      const std::size_t i = static_cast<std::size_t>(r) * w + c;
      if (!trav[i]) continue;
      const double d = dist[static_cast<std::size_t>((r - r0) / ds + 1) * pw + (c - c0) / ds + 1];
      out[i] = d * ds * resolution + snap_m;
    }
  }
}

}  // namespace frontiers
