#include "ground_estimator_cpp/cell_class.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>

namespace ground_estimator_cpp
{

namespace
{

constexpr const char * KIND_STEP = "step";
constexpr const char * KIND_DROP = "drop";
constexpr const char * SOURCE_DEPTH = "depth";
constexpr const char * SOURCE_COLOUR = "colour";
constexpr double CENTROID_MIN_AREA = 1e-12;

bool is_lethal(CellClass c) { return c == CellClass::obstacle || c == CellClass::drop; }

int rank(CellClass c)
{
  switch (c) {
    case CellClass::none:
      return 0;
    case CellClass::warning:
      return 1;
    default:
      return 2;
  }
}

// Median of a non-empty vector (mean of the two middle values for an even count); reorders the vector.
double median(std::vector<double> & v)
{
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  const double upper = v[mid];
  if (v.size() % 2 == 1) {
    return upper;
  }
  const double lower = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid));
  return 0.5 * (lower + upper);
}

double cross(const Eigen::Vector2d & o, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

// Convex hull, counter-clockwise from the lowest-x (then lowest-y) point, without collinear vertices (monotone chain).
std::vector<Eigen::Vector2d> convex_hull(std::vector<Eigen::Vector2d> pts)
{
  std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  if (pts.size() < 3) {
    return pts;
  }
  std::vector<Eigen::Vector2d> hull(2 * pts.size());
  std::size_t k = 0;
  for (const auto & p : pts) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {
      --k;
    }
    hull[k++] = p;
  }
  const std::size_t lower = k + 1;
  for (auto it = pts.rbegin() + 1; it != pts.rend(); ++it) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], *it) <= 0.0) {
      --k;
    }
    hull[k++] = *it;
  }
  hull.resize(k - 1);
  return hull;
}

// Even-odd point-in-polygon test; a point exactly on an edge may fall either way.
bool inside(const std::vector<Eigen::Vector2d> & polygon, const Eigen::Vector2d & p)
{
  bool in = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Eigen::Vector2d & a = polygon[i];
    const Eigen::Vector2d & b = polygon[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) && p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()) {
      in = !in;
    }
  }
  return in;
}

// Area centroid of a polygon; the vertex mean for a degenerate (zero-area) polygon.
Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> & polygon)
{
  double twice_area = 0.0;
  Eigen::Vector2d c = Eigen::Vector2d::Zero();
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double w = polygon[j].x() * polygon[i].y() - polygon[i].x() * polygon[j].y();
    twice_area += w;
    c += w * (polygon[j] + polygon[i]);
  }
  if (std::fabs(twice_area) > CENTROID_MIN_AREA) {
    return c / (3.0 * twice_area);
  }
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto & v : polygon) {
    mean += v;
  }
  return mean / static_cast<double>(polygon.size());
}

}  // namespace

CellKey cell_key_of(const Eigen::Vector2d & xy_fixed, double cell_size_m)
{
  return {
    static_cast<std::int32_t>(std::floor(xy_fixed.x() / cell_size_m)),
    static_cast<std::int32_t>(std::floor(xy_fixed.y() / cell_size_m))};
}

std::vector<std::set<CellKey>> cell_components(const std::set<CellKey> & cells)
{
  std::vector<std::set<CellKey>> out;
  std::set<CellKey> visited;
  for (const CellKey & start : cells) {
    if (!visited.insert(start).second) {
      continue;
    }
    std::set<CellKey> component;
    std::vector<CellKey> stack{start};
    while (!stack.empty()) {
      const CellKey key = stack.back();
      stack.pop_back();
      component.insert(key);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const CellKey next{key.first + dx, key.second + dy};
          if (cells.count(next) > 0 && visited.insert(next).second) {
            stack.push_back(next);
          }
        }
      }
    }
    out.push_back(std::move(component));
  }
  return out;
}

std::vector<Eigen::Vector2d> cell_outline(const std::set<CellKey> & cells, double cell_size_m)
{
  std::vector<Eigen::Vector2d> corners;
  corners.reserve(4 * cells.size());
  for (const CellKey & key : cells) {
    for (const int cx : {0, 1}) {
      for (const int cy : {0, 1}) {
        corners.emplace_back((key.first + cx) * cell_size_m, (key.second + cy) * cell_size_m);
      }
    }
  }
  return convex_hull(std::move(corners));
}

CellClassFilter::CellClassFilter(const HysteresisConfig & cfg, double cell_size_m)
: cfg_(cfg), cell_size_m_(cell_size_m)
{
  if (!cfg_.enabled) {
    // Pass-through: the stable class is this frame's observed label.
    cfg_.window_frames = 1;
    cfg_.enter_frames = 1;
    cfg_.exit_frames = 1;
    cfg_.area_enter_frames = 1;
    cfg_.area_min_cells = 1;
    cfg_.area_join_frames = 1;
    cfg_.area_kind_frames = 1;
    cfg_.area_history_frames = 1;
    cfg_.area_enter_density = 0.0;
    cfg_.area_exit_density = 0.0;
  }
  cfg_.window_frames = std::max(cfg_.window_frames, 1);
}

CellClassFilter::Key CellClassFilter::key_of(const Eigen::Vector2d & xy_fixed) const
{
  return cell_key_of(xy_fixed, cell_size_m_);
}

CellClassFilter::FrameObservation & CellClassFilter::observe(const Key & key, CellClass cls)
{
  FrameObservation & obs = frame_[key];
  if (rank(cls) > rank(obs.label) || (is_lethal(cls) && is_lethal(obs.label))) {
    obs.label = cls;
  }
  return obs;
}

void CellClassFilter::begin_frame() { frame_.clear(); }

void CellClassFilter::observe_points(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, CellClass cls, const Eigen::Isometry3d & fixed_T_base)
{
  if (cls == CellClass::none) {
    return;
  }
  for (const auto & p : points_base) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const Eigen::Vector3d pf = fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z);
    FrameObservation & obs = observe(key_of(pf.head<2>()), cls);
    auto & z = cls == CellClass::obstacle ? obs.obstacle_z : (cls == CellClass::drop ? obs.drop_z : obs.warning_z);
    z.push_back(static_cast<double>(p.z));
  }
}

void CellClassFilter::observe_area(const WarningArea & area_base, const Eigen::Isometry3d & fixed_T_base)
{
  if (area_base.polygon.empty()) {
    return;
  }
  std::vector<Eigen::Vector2d> polygon;
  polygon.reserve(area_base.polygon.size());
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d hi = Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto & v : area_base.polygon) {
    const Eigen::Vector2d vf = (fixed_T_base * Eigen::Vector3d(v.x(), v.y(), 0.0)).head<2>();
    polygon.push_back(vf);
    lo = lo.cwiseMin(vf);
    hi = hi.cwiseMax(vf);
  }
  Record rec;
  rec.kind = area_base.kind;
  rec.height_m = area_base.height_m;
  rec.slope_deg = area_base.slope_deg;
  rec.source = area_base.source;

  std::set<Key> marked{key_of(centroid(polygon))};
  const Key lo_key = key_of(lo);
  const Key hi_key = key_of(hi);
  for (std::int32_t ix = lo_key.first; ix <= hi_key.first; ++ix) {
    for (std::int32_t iy = lo_key.second; iy <= hi_key.second; ++iy) {
      const Eigen::Vector2d centre((ix + 0.5) * cell_size_m_, (iy + 0.5) * cell_size_m_);
      if (inside(polygon, centre)) {
        marked.insert({ix, iy});
      }
    }
  }
  for (const Key & key : marked) {
    observe(key, CellClass::warning).area = rec;
  }
}

void CellClassFilter::end_frame()
{
  // Commit this frame's observations: labels become votes below, records replace the cells' latest records.
  for (auto & [key, obs] : frame_) {
    Cell & cell = cells_[key];
    if (obs.label == CellClass::none) {
      continue;
    }
    if (is_lethal(obs.label)) {
      const bool drop = obs.label == CellClass::drop;
      Record rec;
      rec.kind = drop ? KIND_DROP : KIND_STEP;
      rec.height_m = median(drop ? obs.drop_z : obs.obstacle_z);
      rec.source = SOURCE_DEPTH;
      cell.record = rec;
    } else if (obs.area) {
      cell.record = obs.area;
    } else if (!obs.warning_z.empty()) {
      Record rec;
      rec.kind = KIND_STEP;
      rec.height_m = median(obs.warning_z);
      rec.source = SOURCE_DEPTH;
      cell.record = rec;
    }
  }

  const auto window = static_cast<std::size_t>(cfg_.window_frames);
  for (auto it = cells_.begin(); it != cells_.end();) {
    Cell & cell = it->second;
    const auto obs = frame_.find(it->first);
    const CellClass vote = obs == frame_.end() ? CellClass::none : obs->second.label;
    cell.votes.push_back(vote);
    while (cell.votes.size() > window) {
      cell.votes.pop_front();
    }
    if (std::all_of(cell.votes.begin(), cell.votes.end(), [](CellClass v) { return v == CellClass::none; })) {
      it = cells_.erase(it);
      continue;
    }
    int lethal = 0;
    int warning = 0;
    for (const CellClass v : cell.votes) {
      if (is_lethal(v)) {
        ++lethal;
        cell.last_lethal = v;
      } else if (v == CellClass::warning) {
        ++warning;
      }
    }
    const bool was_lethal = is_lethal(cell.stable);
    const bool was_warning = cell.stable == CellClass::warning;
    if (lethal >= cfg_.enter_frames || (was_lethal && lethal >= cfg_.exit_frames)) {
      cell.stable = cell.last_lethal;
    } else if (
      warning + lethal >= cfg_.enter_frames || ((was_warning || was_lethal) && warning + lethal >= cfg_.exit_frames)) {
      cell.stable = CellClass::warning;
    } else {
      cell.stable = CellClass::none;
    }
    // A cell is active while it is voted or has a stable class, so a cell decaying out of its class keeps its run.
    cell.active_run = (vote != CellClass::none || cell.stable != CellClass::none) ? cell.active_run + 1 : 0;
    ++it;
  }
  // Area history: one bit per frame (the cell was voted) for the last area_history_frames frames, kept until all clear.
  for (const auto & [key, obs] : frame_) {
    if (obs.label != CellClass::none) {
      history_[key];
    }
  }
  const auto history_window = static_cast<std::size_t>(cfg_.area_history_frames);
  for (auto it = history_.begin(); it != history_.end();) {
    const auto obs = frame_.find(it->first);
    it->second.push_back(obs != frame_.end() && obs->second.label != CellClass::none);
    while (it->second.size() > history_window) {
      it->second.pop_front();
    }
    const bool voted = std::any_of(it->second.begin(), it->second.end(), [](bool v) { return v; });
    it = voted ? std::next(it) : history_.erase(it);
  }
  update_areas();
}

CellClass CellClassFilter::stable_at(const Eigen::Vector2d & xy_fixed) const
{
  const auto it = cells_.find(key_of(xy_fixed));
  return it == cells_.end() ? CellClass::none : it->second.stable;
}

std::size_t CellClassFilter::count(CellClass cls) const
{
  return static_cast<std::size_t>(
    std::count_if(cells_.begin(), cells_.end(), [cls](const auto & kv) { return kv.second.stable == cls; }));
}

std::vector<CellKey> CellClassFilter::blocking_cells() const
{
  std::vector<CellKey> out;
  for (const auto & [key, cell] : cells_) {
    const bool depth_warning = cell.stable == CellClass::warning && cell.record && cell.record->source == SOURCE_DEPTH;
    if (is_lethal(cell.stable) || depth_warning) {
      out.push_back(key);
    }
  }
  return out;
}

std::vector<StableArea> CellClassFilter::warning_areas() const { return areas_; }

void CellClassFilter::update_areas()
{
  // The previous frame's footprint: the cells of every published area.
  std::set<Key> published;
  for (const Track & track : tracks_) {
    published.insert(track.cells.begin(), track.cells.end());
  }
  const auto run_of = [this](const Key & key) {
    const auto it = cells_.find(key);
    return it != cells_.end() && it->second.stable == CellClass::warning ? it->second.active_run : 0;
  };

  // Candidate cells: stable warning for area_enter_frames active frames.
  // Share of the cell's history frames that voted; a cell must reach area_enter_density to join an area and stay at or
  // above area_exit_density once in one.
  const auto density = [this](const Key & key) {
    const auto it = history_.find(key);
    if (it == history_.end()) {
      return 0.0;
    }
    return static_cast<double>(std::count(it->second.begin(), it->second.end(), true)) /
           static_cast<double>(it->second.size());
  };
  std::set<Key> candidates;
  for (const auto & entry : cells_) {
    const double needed = published.count(entry.first) > 0 ? cfg_.area_exit_density : cfg_.area_enter_density;
    if (run_of(entry.first) >= cfg_.area_enter_frames && density(entry.first) >= needed) {
      candidates.insert(entry.first);
    }
  }
  // A component with no published cell is new and takes all its cells; one touching a published area gets only the
  // published cells that are still candidates and the cells active for area_join_frames.
  std::vector<std::set<Key>> footprints;
  for (std::set<Key> & component : cell_components(candidates)) {
    const bool touches =
      std::any_of(component.begin(), component.end(), [&](const Key & k) { return published.count(k) > 0; });
    if (!touches) {
      footprints.push_back(std::move(component));
      continue;
    }
    std::set<Key> admitted;
    for (const Key & key : component) {
      if (published.count(key) > 0 || run_of(key) >= cfg_.area_join_frames) {
        admitted.insert(key);
      }
    }
    for (std::set<Key> & part : cell_components(admitted)) {
      footprints.push_back(std::move(part));
    }
  }
  std::sort(footprints.begin(), footprints.end(), [](const std::set<Key> & a, const std::set<Key> & b) {
    return *a.begin() < *b.begin();
  });

  std::vector<StableArea> areas;
  std::vector<Track> tracks;
  for (std::set<Key> & footprint : footprints) {
    if (static_cast<int>(footprint.size()) < cfg_.area_min_cells) {
      continue;
    }
    StableArea area;
    std::map<std::string, int> kinds;
    std::vector<double> heights;
    std::vector<double> slopes;
    bool depth = false;
    for (const Key & key : footprint) {
      const std::optional<Record> & rec = cells_.at(key).record;
      if (!rec) {
        continue;
      }
      ++kinds[rec->kind];
      if (rec->height_m) {
        heights.push_back(*rec->height_m);
      }
      if (rec->slope_deg) {
        slopes.push_back(*rec->slope_deg);
      }
      depth = depth || rec->source == SOURCE_DEPTH;
    }
    area.polygon = cell_outline(footprint, cell_size_m_);
    int best = 0;
    area.kind = KIND_STEP;
    for (const auto & [kind, n] : kinds) {
      if (n > best) {
        best = n;
        area.kind = kind;
      }
    }
    area.height_m = heights.empty() ? std::nullopt : std::optional<double>(median(heights));
    area.slope_deg = slopes.empty() ? std::nullopt : std::optional<double>(median(slopes));
    area.source = depth ? SOURCE_DEPTH : SOURCE_COLOUR;

    // Kind persistence: the previous area sharing most cells keeps its published kind until another kind has most of
    // the cells for area_kind_frames consecutive frames.
    Track track;
    track.cells = std::move(footprint);
    track.kind = area.kind;
    const Track * prev = nullptr;
    std::size_t prev_overlap = 0;
    for (const Track & candidate : tracks_) {
      const auto overlap = static_cast<std::size_t>(std::count_if(
        track.cells.begin(), track.cells.end(), [&](const Key & k) { return candidate.cells.count(k) > 0; }));
      if (overlap > prev_overlap) {
        prev = &candidate;
        prev_overlap = overlap;
      }
    }
    if (prev != nullptr) {
      track.kind = prev->kind;
      if (area.kind != prev->kind && kinds[area.kind] > kinds[prev->kind]) {
        track.pending_kind = area.kind;
        track.pending_frames = prev->pending_kind == area.kind ? prev->pending_frames + 1 : 1;
        if (track.pending_frames >= cfg_.area_kind_frames) {
          track.kind = area.kind;
          track.pending_kind.clear();
          track.pending_frames = 0;
        }
      }
    }
    area.kind = track.kind;
    tracks.push_back(std::move(track));
    areas.push_back(std::move(area));
  }
  areas_ = std::move(areas);
  tracks_ = std::move(tracks);
}

void CellClassFilter::reset()
{
  frame_.clear();
  cells_.clear();
  areas_.clear();
  tracks_.clear();
  history_.clear();
}

}  // namespace ground_estimator_cpp
