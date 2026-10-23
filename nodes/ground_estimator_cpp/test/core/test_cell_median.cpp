#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <random>

#include "ground_estimator_cpp/cell_median.hpp"
#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/segmentation.hpp"
#include "ground_estimator_cpp/traversability.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::CellMedianFilter;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::TemporalConfig;
using ground_test::add_z_noise;
using ground_test::DEG;
using ground_test::grid_cloud;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

constexpr double SIGMA_M = 0.026;
constexpr double OUTLIER_FRACTION = 0.2;
constexpr double OUTLIER_RANGE_M = 0.06;
constexpr double STEP_RULE_M = 0.04;
constexpr double LEGACY_STEP_FRACTION = 0.3333;
constexpr double FRAME_DT_S = 0.1;
constexpr double GRID_STEP_M = 0.04;
constexpr int FRAMES = 20;

double max_abs_z(const Cloud & c)
{
  double m = 0.0;
  for (const auto & p : c) {
    m = std::max(m, static_cast<double>(std::fabs(p.z)));
  }
  return m;
}

constexpr double CELL_M = 0.10;
constexpr double CELL_POINT_OFFSET_M = 0.02;

// One cloud with four points in every 0.1 m cell of [x0, x1) x [-0.3, 0.3); cell (ix, iy) sits at height value(ix, iy).
Cloud cell_cloud(double x0, double x1, const std::function<double(int, int)> & value)
{
  Cloud c;
  for (int ix = static_cast<int>(std::lround(x0 / CELL_M)); ix < static_cast<int>(std::lround(x1 / CELL_M)); ++ix) {
    for (int iy = -3; iy < 3; ++iy) {
      const double z = value(ix, iy);  // one value per cell and frame
      for (const double dx : {-CELL_POINT_OFFSET_M, CELL_POINT_OFFSET_M}) {
        for (const double dy : {-CELL_POINT_OFFSET_M, CELL_POINT_OFFSET_M}) {
          c.push_back(pcl::PointXYZ(
            static_cast<float>((ix + 0.5) * CELL_M + dx), static_cast<float>((iy + 0.5) * CELL_M + dy),
            static_cast<float>(z)));
        }
      }
    }
  }
  return c;
}

}  // namespace

TEST(CellMedian, NoisyFlatFloorHasNoStepOrDrop)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Plane plane;
  const Cloud truth = grid_cloud(-1.0, 1.0, -1.0, 1.0, GRID_STEP_M);
  double raw_max = 0.0;
  double filtered_max = 0.0;
  std::size_t filtered_points = 0;
  for (int i = 0; i < FRAMES; ++i) {
    const Cloud noisy = add_z_noise(truth, SIGMA_M, OUTLIER_FRACTION, OUTLIER_RANGE_M, 100 + i);
    raw_max = std::max(raw_max, max_abs_z(noisy));
    const Cloud out = filter.apply(noisy, plane, Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= cfg.min_frames) {
      filtered_max = std::max(filtered_max, max_abs_z(out));
      filtered_points += out.size();
    }
  }
  EXPECT_GT(raw_max, STEP_RULE_M);
  EXPECT_GT(filtered_points, 0u);
  EXPECT_LE(filtered_max, STEP_RULE_M);
}

TEST(CellMedian, PlateAndDropKeepTheirHeights)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Plane plane;
  Cloud truth = grid_cloud(-1.0, 1.0, -1.0, 1.0, GRID_STEP_M);
  for (auto & p : truth) {
    if (p.x >= 0.5 && p.x < 1.0 && p.y >= 0.0 && p.y < 0.3) {
      p.z = 0.08f;
    } else if (p.x < -0.5 && p.y >= -1.0 && p.y < 0.0) {
      p.z = -0.15f;
    }
  }
  Cloud out;
  for (int i = 0; i < FRAMES; ++i) {
    out = filter.apply(
      add_z_noise(truth, SIGMA_M, OUTLIER_FRACTION, OUTLIER_RANGE_M, 7 + i), plane, Eigen::Isometry3d::Identity(),
      i * FRAME_DT_S);
  }
  std::size_t plate = 0;
  std::size_t drop = 0;
  for (const auto & p : out) {
    if (p.x >= 0.5 && p.x < 1.0 && p.y >= 0.0 && p.y < 0.3) {
      EXPECT_NEAR(p.z, 0.08, 0.01);
      ++plate;
    } else if (p.x < -0.5 && p.y >= -1.0 && p.y < 0.0) {
      EXPECT_NEAR(p.z, -0.15, 0.01);
      ++drop;
    }
  }
  EXPECT_GT(plate, 0u);
  EXPECT_GT(drop, 0u);
}

TEST(CellMedian, YoungCellsAreNotOutput)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Cloud cloud = grid_cloud(0.0, 0.1, 0.0, 0.1, 0.02);
  for (int i = 0; i < cfg.min_frames - 1; ++i) {
    EXPECT_TRUE(filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S).empty());
  }
  EXPECT_EQ(filter.cell_count(), 1u);
  EXPECT_EQ(
    filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), cfg.min_frames * FRAME_DT_S).size(), cloud.size());
}

TEST(CellMedian, CellsStayInTheFixedFrame)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Cloud fixed_points = grid_cloud(0.52, 0.58, 0.02, 0.08, 0.02, 0.1);
  for (int i = 0; i < cfg.min_frames; ++i) {
    Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
    fixed_T_base.translation() = Eigen::Vector3d(0.05 * i, 0.0, 0.0);
    Cloud base_points;
    for (const auto & p : fixed_points) {
      base_points.push_back(pcl::PointXYZ(p.x - 0.05f * static_cast<float>(i), p.y, p.z));
    }
    const Cloud out = filter.apply(base_points, Plane(), fixed_T_base, i * FRAME_DT_S);
    if (i < cfg.min_frames - 1) {
      EXPECT_TRUE(out.empty());
    } else {
      ASSERT_EQ(out.size(), base_points.size());
      EXPECT_NEAR(out[0].z, 0.1, 1e-6);
    }
  }
  EXPECT_EQ(filter.cell_count(), 1u);
}

TEST(CellMedian, OldValuesExpire)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Cloud cloud = grid_cloud(0.0, 0.1, 0.0, 0.1, 0.02);
  const Cloud far_cloud = grid_cloud(1.0, 1.1, 1.0, 1.1, 0.02);
  for (int i = 0; i < cfg.min_frames; ++i) {
    filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
  }
  EXPECT_EQ(filter.cell_count(), 1u);
  // 1.75 s: the values at 0.0, 0.1 and 0.2 s are older than 1.5 s, one value left plus the new frame: 2 < min_frames.
  EXPECT_TRUE(filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), 1.75).empty());
  // A frame elsewhere much later erases the old cell.
  filter.apply(far_cloud, Plane(), Eigen::Isometry3d::Identity(), 5.0);
  EXPECT_EQ(filter.cell_count(), 1u);
}

TEST(CellMedian, DisabledPassesThrough)
{
  TemporalConfig cfg;
  cfg.enabled = false;
  CellMedianFilter filter(cfg);
  const Cloud cloud = add_z_noise(grid_cloud(0.0, 0.5, 0.0, 0.5, 0.05), 0.02, 0.0, 0.0, 3);
  const Cloud out = filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), 0.0);
  ASSERT_EQ(out.size(), cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_EQ(out[i].z, cloud[i].z);
  }
  EXPECT_EQ(filter.cell_count(), 0u);
}

// The raw plane-relative height of every output point comes out parallel to the output: a point moved to its cell's
// median keeps its own height of this frame for the lethal marks.
TEST(CellMedian, RawHeightsComeOutParallelToTheOutput)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  Plane plane;
  plane.d = -0.01;  // floor at z = 0.01
  const auto heights_of = [](int frame) {
    return [frame](int ix, int) { return 0.01 + 0.05 * (ix % 2) + 0.002 * frame; };
  };
  std::vector<float> raw;
  for (int i = 0; i < cfg.min_frames + 2; ++i) {
    const Cloud in = cell_cloud(0.6, 0.9, heights_of(i));
    const Cloud out = filter.apply(in, plane, Eigen::Isometry3d::Identity(), i * FRAME_DT_S, &raw);
    ASSERT_EQ(raw.size(), out.size()) << "frame " << i;
    if (i < cfg.min_frames - 1) {
      continue;
    }
    ASSERT_EQ(out.size(), in.size());
    for (std::size_t k = 0; k < in.size(); ++k) {
      EXPECT_NEAR(raw[k], in[k].z - 0.01, 1e-6) << "frame " << i << " point " << k;
    }
  }
  // Disabled: the input heights, the input unchanged.
  TemporalConfig off = cfg;
  off.enabled = false;
  CellMedianFilter disabled(off);
  const Cloud in = cell_cloud(0.6, 0.9, heights_of(0));
  const Cloud out = disabled.apply(in, plane, Eigen::Isometry3d::Identity(), 0.0, &raw);
  ASSERT_EQ(raw.size(), in.size());
  for (std::size_t k = 0; k < in.size(); ++k) {
    EXPECT_NEAR(raw[k], in[k].z - 0.01, 1e-6);
  }
}

TEST(CellMedian, StampGoingBackResets)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Cloud cloud = grid_cloud(0.0, 0.1, 0.0, 0.1, 0.02);
  for (int i = 0; i < cfg.min_frames; ++i) {
    filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), 10.0 + i * FRAME_DT_S);
  }
  EXPECT_TRUE(filter.apply(cloud, Plane(), Eigen::Isometry3d::Identity(), 1.0).empty());
  EXPECT_EQ(filter.cell_count(), 1u);
}

namespace
{

constexpr int SEED_COUNT = 24;
constexpr int SIGMA_STEPS = 13;  // temporal stds spread evenly over a band, one per cell column/row combination

struct Visibility
{
  std::size_t cell_frames = 0;
  std::size_t visible_cell_frames = 0;
  std::size_t gated_seen = 0;
  double fraction() const { return static_cast<double>(visible_cell_frames) / static_cast<double>(cell_frames); }
};

// Runs SEED_COUNT fixed seeds (40 frames each) over cells at x in [x0, x1) whose per-frame value has a temporal std
// spread evenly over [sigma_lo, sigma_hi] metres (one std per cell). Uniform noise (the far-band ToF shape) when
// uniform is true, Gaussian otherwise. Counts cell-frames after window_frames.
Visibility measure_visibility(double x0, double x1, double sigma_lo, double sigma_hi, bool uniform)
{
  const TemporalConfig cfg;
  Visibility result;
  for (int seed = 1; seed <= SEED_COUNT; ++seed) {
    CellMedianFilter filter(cfg);
    std::mt19937 rng(static_cast<unsigned>(seed));
    for (int i = 0; i < 40; ++i) {
      const Cloud in = cell_cloud(x0, x1, [&](int ix, int iy) {
        const double sigma =
          sigma_lo + (sigma_hi - sigma_lo) * (((ix * 7 + iy + 30) % SIGMA_STEPS) / (SIGMA_STEPS - 1.0));
        if (uniform) {
          return 0.04 + std::uniform_real_distribution<double>(-std::sqrt(3.0) * sigma, std::sqrt(3.0) * sigma)(rng);
        }
        return std::normal_distribution<double>(0.0, sigma)(rng);
      });
      const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
      if (i >= cfg.window_frames) {
        result.cell_frames += in.size() / 4;
        result.visible_cell_frames += out.size() / 4;
        result.gated_seen += filter.gated_cell_count();
      }
    }
  }
  return result;
}

}  // namespace

// The measured far-band noise (floor + U(+0.04, +0.19), temporal std 43..90 mm) is gated. A window of 8 values gives a
// spread estimate with ~25% relative scatter, so a cell is not gated with certainty: the leak is bounded, not zero.
TEST(CellMedian, FarBandNoiseCellsAreGatedAndRarelyOutput)
{
  const Visibility v = measure_visibility(1.15, 1.45, 0.043, 0.090, true);
  EXPECT_GT(v.gated_seen, 0u);
  EXPECT_LE(v.fraction(), 0.05);
}

// 30..42 mm is only 1.2..1.7 times the 25 mm limit; the downstream class hysteresis and lethal point gates absorb the rest.
TEST(CellMedian, ThirtyToFortyTwoMillimetreFarBandCellsLeakAtMostTenPercent)
{
  const Visibility v = measure_visibility(1.15, 1.45, 0.030, 0.042, true);
  EXPECT_GT(v.gated_seen, 0u);
  EXPECT_LE(v.fraction(), 0.10);
}

TEST(CellMedian, MeasuredFloorNoiseStaysVisible)
{
  const Visibility v = measure_visibility(0.35, 1.05, 0.004, 0.020, false);
  EXPECT_GE(v.fraction(), 0.95);
}

// A gated cell is known again as soon as its spread is under the limit (no stricter release threshold): after the
// noisy values leave the window, +-16 mm alternation (estimated spread ~23 mm) is visible in every frame.
TEST(CellMedian, GatedCellIsReleasedAsSoonAsSpreadIsUnderTheLimit)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  for (int i = 0; i < 40; ++i) {
    const double noisy = (i % 2 == 0 ? 1.0 : -1.0) * 0.1;
    const double calm = (i % 2 == 0 ? 1.0 : -1.0) * 0.016;
    const double value = i < 12 ? noisy : calm;
    const Cloud in = cell_cloud(0.6, 0.7, [&](int, int) { return value; });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= 12 + cfg.window_frames) {
      EXPECT_EQ(out.size(), in.size()) << "frame " << i;
    }
  }
}

TEST(CellMedian, StaticBoxTopIsOutputEveryFrame)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  std::mt19937 rng(21);
  std::normal_distribution<double> noise(0.0, 0.0045);
  for (int i = 0; i < 25; ++i) {
    const Cloud in = cell_cloud(0.6, 0.9, [&](int, int) { return 0.05 + noise(rng); });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= cfg.min_frames) {
      ASSERT_EQ(out.size(), in.size()) << "frame " << i;
      for (const auto & p : out) {
        EXPECT_NEAR(p.z, 0.05, 0.01);
      }
    }
  }
}

TEST(CellMedian, SingleOutlierFrameDoesNotGateASteadyCell)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  std::mt19937 rng(31);
  std::normal_distribution<double> noise(0.0, 0.003);
  for (int i = 0; i < 20; ++i) {
    const double bump = i == 10 ? 0.15 : 0.0;
    const Cloud in = cell_cloud(0.6, 0.7, [&](int, int) { return bump + noise(rng); });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= cfg.min_frames) {
      EXPECT_EQ(out.size(), in.size()) << "frame " << i;
      EXPECT_EQ(filter.gated_cell_count(), 0u) << "frame " << i;
    }
  }
}

TEST(CellMedian, GatedCellRecoversWithinTheWindow)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  std::mt19937 rng(41);
  std::normal_distribution<double> quiet(0.0, 0.003);
  std::uniform_real_distribution<double> wild(0.0, 0.3);
  const int noisy_frames = 12;
  for (int i = 0; i < noisy_frames; ++i) {
    filter.apply(
      cell_cloud(0.6, 0.7, [&](int, int) { return wild(rng); }), Plane(), Eigen::Isometry3d::Identity(),
      i * FRAME_DT_S);
  }
  EXPECT_GT(filter.gated_cell_count(), 0u);
  std::size_t frames_to_recover = 0;
  bool recovered = false;
  for (int i = 0; i < 3 * cfg.window_frames && !recovered; ++i) {
    const Cloud in = cell_cloud(0.6, 0.7, [&](int, int) { return quiet(rng); });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), (noisy_frames + i) * FRAME_DT_S);
    ++frames_to_recover;
    recovered = out.size() == in.size() && filter.gated_cell_count() == 0u;
  }
  EXPECT_TRUE(recovered);
  EXPECT_LE(frames_to_recover, static_cast<std::size_t>(cfg.window_frames));
}

TEST(CellMedian, DisabledReportsNoGatedCells)
{
  TemporalConfig cfg;
  cfg.enabled = false;
  CellMedianFilter filter(cfg);
  filter.apply(grid_cloud(0.0, 0.5, 0.0, 0.5, 0.05), Plane(), Eigen::Isometry3d::Identity(), 0.0);
  EXPECT_EQ(filter.gated_cell_count(), 0u);
}

// steady_at (the node's lethal vote gate): a steady box top is steady from its first output on.
TEST(CellMedian, SteadyBoxTopIsSteadyFromItsFirstOutput)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  std::mt19937 rng(51);
  std::normal_distribution<double> noise(0.0, 0.004);
  const Eigen::Vector2d centre(0.65, 0.05);
  for (int i = 0; i < 12; ++i) {
    const Cloud in = cell_cloud(0.6, 0.7, [&](int, int) { return 0.05 + noise(rng); });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    EXPECT_EQ(filter.steady_at(centre, 0.6 * cfg.max_spread_m, 2 * cfg.window_frames), !out.empty()) << "frame " << i;
    if (i >= cfg.min_frames - 1) {
      EXPECT_FALSE(out.empty()) << "frame " << i;
    }
  }
}

// A cell gated by its spread is not steady until it has been ungated for quiet_frames frames, and a cell whose spread
// is under the gate but over the stricter max_spread_m is not steady either.
TEST(CellMedian, RecentlyGatedOrMildlyNoisyCellIsNotSteady)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Eigen::Vector2d centre(0.65, 0.05);
  const int quiet_frames = 2 * cfg.window_frames;
  int frame = 0;
  for (; frame < 12; ++frame) {
    const double value = (frame % 2 == 0 ? 1.0 : -1.0) * 0.1;
    filter.apply(
      cell_cloud(0.6, 0.7, [&](int, int) { return value; }), Plane(), Eigen::Isometry3d::Identity(),
      frame * FRAME_DT_S);
  }
  EXPECT_GT(filter.gated_cell_count(), 0u);
  EXPECT_FALSE(filter.steady_at(centre, 0.6 * cfg.max_spread_m, quiet_frames));
  int first_steady = -1;
  for (int i = 0; i < 4 * cfg.window_frames; ++i, ++frame) {
    const Cloud out = filter.apply(
      cell_cloud(0.6, 0.7, [&](int, int) { return 0.05; }), Plane(), Eigen::Isometry3d::Identity(), frame * FRAME_DT_S);
    if (first_steady < 0 && filter.steady_at(centre, 0.6 * cfg.max_spread_m, quiet_frames)) {
      first_steady = i;
      EXPECT_FALSE(out.empty());
    }
  }
  // Visible again within the window, steady only after quiet_frames ungated frames.
  EXPECT_GE(first_steady, quiet_frames - 1);
  EXPECT_LE(first_steady, quiet_frames + cfg.window_frames);

  // +-12 mm alternation: a spread of ~17 mm is under the 25 mm gate (visible) but over 15 mm (not steady).
  CellMedianFilter mild(cfg);
  for (int i = 0; i < 20; ++i) {
    const double value = (i % 2 == 0 ? 1.0 : -1.0) * 0.012;
    const Cloud out = mild.apply(
      cell_cloud(0.6, 0.7, [&](int, int) { return value; }), Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= cfg.min_frames) {
      EXPECT_FALSE(out.empty()) << "frame " << i;
      EXPECT_FALSE(mild.steady_at(centre, 0.6 * cfg.max_spread_m, quiet_frames)) << "frame " << i;
    }
  }
}

// A steady cell next to a cell gated in its last quiet_frames frames is not steady: at the edge of a noisy band the
// median of a cell mixing floor and band returns sits steadily between the two, so a noisy neighbour withholds it.
// A gated cell two cells away does not.
TEST(CellMedian, SteadyCellNextToARecentlyGatedCellIsNotSteady)
{
  const TemporalConfig cfg;
  const int quiet_frames = 2 * cfg.window_frames;
  const Eigen::Vector2d steady(0.65, 0.05);
  const auto run = [&](CellMedianFilter & filter, double noisy_x0, int frames, int first_frame, bool noisy) {
    for (int i = 0; i < frames; ++i) {
      const double value = noisy ? ((first_frame + i) % 2 == 0 ? 1.0 : -1.0) * 0.1 : 0.0;
      Cloud in = cell_cloud(0.6, 0.7, [](int, int) { return 0.05; });
      const Cloud band = cell_cloud(noisy_x0, noisy_x0 + CELL_M, [&](int, int) { return value; });
      in.insert(in.end(), band.begin(), band.end());
      filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), (first_frame + i) * FRAME_DT_S);
    }
  };
  CellMedianFilter next_to(cfg);
  run(next_to, 0.7, 12, 0, true);
  EXPECT_GT(next_to.gated_cell_count(), 0u);
  EXPECT_FALSE(next_to.steady_at(steady, 0.6 * cfg.max_spread_m, quiet_frames));
  // The neighbour turns quiet: steady again once it has been ungated for quiet_frames frames.
  run(next_to, 0.7, quiet_frames + cfg.window_frames, 12, false);
  EXPECT_TRUE(next_to.steady_at(steady, 0.6 * cfg.max_spread_m, quiet_frames));

  CellMedianFilter two_away(cfg);
  run(two_away, 0.8, 12, 0, true);
  EXPECT_GT(two_away.gated_cell_count(), 0u);
  EXPECT_TRUE(two_away.steady_at(steady, 0.6 * cfg.max_spread_m, quiet_frames));
}

// An object placed on a steady floor cell gates the cell while its history turns over from the floor to the object
// (old and new values mix). That gate is the new surface arriving, not noise: once the history holds the new level the
// cell is steady from its first output, so a new real obstacle is lethal without waiting quiet_frames (A13).
TEST(CellMedian, ObjectPlacedOnASteadyCellIsSteadyOnceItsHistoryTurnsOver)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Eigen::Vector2d centre(0.65, 0.05);
  const int quiet_frames = 2 * cfg.window_frames;
  int frame = 0;
  for (; frame < 12; ++frame) {
    filter.apply(
      cell_cloud(0.6, 0.7, [](int, int) { return 0.0; }), Plane(), Eigen::Isometry3d::Identity(), frame * FRAME_DT_S);
  }
  ASSERT_TRUE(filter.steady_at(centre, 0.6 * cfg.max_spread_m, quiet_frames));
  bool gated = false;
  int first_box = -1;
  for (int i = 0; i < 2 * cfg.window_frames && first_box < 0; ++i, ++frame) {
    const Cloud out = filter.apply(
      cell_cloud(0.6, 0.7, [](int, int) { return 0.05; }), Plane(), Eigen::Isometry3d::Identity(), frame * FRAME_DT_S);
    gated = gated || filter.gated_cell_count() > 0;
    if (!out.empty() && std::fabs(out.front().z - 0.05) < 1e-3) {
      first_box = i;
      EXPECT_TRUE(filter.steady_at(centre, 0.6 * cfg.max_spread_m, quiet_frames)) << "frame " << i;
    }
  }
  EXPECT_TRUE(gated) << "the turnover is expected to gate the cell";
  ASSERT_GE(first_box, 0);
  EXPECT_LT(first_box, cfg.window_frames);
}

// A noisy run between two steady levels is noise, not a turnover: the cell still needs quiet_frames ungated frames.
TEST(CellMedian, NoisyRunBetweenTwoSteadyLevelsStillNeedsQuietFrames)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  const Eigen::Vector2d centre(0.65, 0.05);
  const int quiet_frames = 2 * cfg.window_frames;
  int frame = 0;
  for (; frame < 12; ++frame) {
    filter.apply(
      cell_cloud(0.6, 0.7, [](int, int) { return 0.0; }), Plane(), Eigen::Isometry3d::Identity(), frame * FRAME_DT_S);
  }
  for (int i = 0; i < 4; ++i, ++frame) {
    const double value = (i % 2 == 0 ? 1.0 : -1.0) * 0.1;
    filter.apply(
      cell_cloud(0.6, 0.7, [&](int, int) { return value; }), Plane(), Eigen::Isometry3d::Identity(),
      frame * FRAME_DT_S);
  }
  int first_output = -1;
  for (int i = 0; i < quiet_frames - 1; ++i, ++frame) {
    const Cloud out = filter.apply(
      cell_cloud(0.6, 0.7, [](int, int) { return 0.05; }), Plane(), Eigen::Isometry3d::Identity(), frame * FRAME_DT_S);
    if (!out.empty() && first_output < 0) {
      first_output = i;
    }
    EXPECT_FALSE(filter.steady_at(centre, 0.6 * cfg.max_spread_m, quiet_frames)) << "frame " << i;
  }
  EXPECT_GE(first_output, 0);
}

TEST(CellMedian, DisabledIsAlwaysSteadyAndUnknownCellsAreNot)
{
  TemporalConfig cfg;
  CellMedianFilter enabled(cfg);
  EXPECT_FALSE(enabled.steady_at(Eigen::Vector2d(0.65, 0.05), cfg.max_spread_m, 0));
  cfg.enabled = false;
  CellMedianFilter disabled(cfg);
  EXPECT_TRUE(disabled.steady_at(Eigen::Vector2d(0.65, 0.05), cfg.max_spread_m, 100));
}

namespace
{

// The robot scene: the user's 25 cm, 63 mm (14.1 deg) ramp surrounded by floor, 0.02 m floor band, per-point noise
// 0.02 m, 60% of the 2 cm grid per frame, cells at a random offset and yaw per seed.
constexpr double RAMP_X0_M = 1.0;
constexpr double RAMP_LENGTH_M = 0.25;
constexpr double RAMP_HEIGHT_M = 0.063;
constexpr double RAMP_Y0_M = -0.45;
constexpr double RAMP_Y1_M = -0.20;
constexpr double RAMP_GRID_M = 0.02;
constexpr double RAMP_KEEP_FRACTION = 0.6;
constexpr double RAMP_SIGMA_M = 0.02;
constexpr double ROBOT_FLOOR_BAND_M = 0.02;
constexpr double EDGE_BAND_M = 0.05;
constexpr int RAMP_FRAMES = 10;
constexpr unsigned RAMP_SEEDS = 20;
constexpr double RAMP_TOL_DEG = 3.0;
constexpr double MAX_ALIGNMENT_OFFSET_M = 0.1;
constexpr double MAX_YAW_DEG = 45.0;

}  // namespace

TEST(CellMedian, RobotRampSlopeSurvivesTheCells)
{
  ground_test::Scene scene;
  ground_test::add_ramp(scene, RAMP_X0_M, RAMP_LENGTH_M, RAMP_HEIGHT_M, RAMP_Y0_M, RAMP_Y1_M);
  const Cloud dense = ground_test::sample_grid(scene, 0.8, 1.45, -0.65, 0.0, RAMP_GRID_M);
  const double truth_deg = std::atan(RAMP_HEIGHT_M / RAMP_LENGTH_M) / DEG;
  std::vector<double> slopes;
  for (unsigned seed = 1; seed <= RAMP_SEEDS; ++seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> offset(0.0, MAX_ALIGNMENT_OFFSET_M);
    std::uniform_real_distribution<double> yaw(0.0, MAX_YAW_DEG);
    Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
    fixed_T_base.translate(Eigen::Vector3d(offset(rng), offset(rng), 0.0));
    fixed_T_base.rotate(Eigen::AngleAxisd(yaw(rng) * DEG, Eigen::Vector3d::UnitZ()));
    CellMedianFilter filter{TemporalConfig{}};
    Cloud filtered;
    for (int frame = 0; frame < RAMP_FRAMES; ++frame) {
      const unsigned frame_seed = seed * 1000U + static_cast<unsigned>(frame);
      const Cloud noisy = add_z_noise(
        ground_test::keep_fraction(dense, RAMP_KEEP_FRACTION, frame_seed), RAMP_SIGMA_M, 0.0, 0.0, frame_seed + 500U);
      filtered = filter.apply(noisy, Plane(), fixed_T_base, frame * FRAME_DT_S);
    }
    const auto seg =
      ground_estimator_cpp::segment(filtered, Plane(), ground_estimator_cpp::BandsConfig{}, ROBOT_FLOOR_BAND_M);
    const auto clusters = ground_estimator_cpp::extract_clusters(
      seg.above, seg.above_height, "step", Plane(), ground_estimator_cpp::EdgesConfig{}, EDGE_BAND_M);
    const ground_estimator_cpp::Cluster * best = nullptr;
    for (const auto & c : clusters) {
      if (best == nullptr || c.points.size() > best->points.size()) {
        best = &c;
      }
    }
    if (best != nullptr && best->slope_known) {
      slopes.push_back(best->slope_deg);
    }
  }
  ASSERT_GE(slopes.size(), RAMP_SEEDS * 3 / 4);
  std::sort(slopes.begin(), slopes.end());
  EXPECT_NEAR(slopes[slopes.size() / 2], truth_deg, RAMP_TOL_DEG);
}

TEST(CellMedian, SlopeInsideACellIsKept)
{
  // A noise-free 14 deg incline over many 10 cm cells: collapsing each cell to one height would put the points up to
  // half a cell times the slope (12 mm) off; with the in-cell gradient they stay within a few mm.
  constexpr double SLOPE = 0.25;
  constexpr double MAX_ERR_M = 0.004;
  constexpr double SENSOR_SIGMA_M = 0.002;
  constexpr double INTERIOR_X0_M = 0.1;
  constexpr double INTERIOR_X1_M = 0.7;
  constexpr double INTERIOR_Y_M = 0.1;
  Cloud truth;
  for (double x = 0.0; x < 0.8; x += 0.02) {
    for (double y = -0.2; y < 0.2; y += 0.02) {
      truth.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(SLOPE * x)));
    }
  }
  CellMedianFilter filter{TemporalConfig{}};
  Cloud out;
  Cloud in;
  for (int i = 0; i < 10; ++i) {
    in = add_z_noise(truth, SENSOR_SIGMA_M, 0.0, 0.0, 700 + i);
    out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
  }
  ASSERT_GT(out.size(), truth.size() / 2);
  double worst = 0.0;
  for (const auto & p : out) {
    if (p.x < INTERIOR_X0_M || p.x > INTERIOR_X1_M || std::fabs(p.y) > INTERIOR_Y_M) {
      continue;  // border cells hold few points and stay flat
    }
    worst = std::max(worst, static_cast<double>(std::fabs(p.z - SLOPE * p.x)));
  }
  EXPECT_LT(worst, MAX_ERR_M);
}

namespace
{

// A 3 cm door sill (x 1.0 .. 1.15, |y| <= 0.4, 2 cm grid, 3 mm noise) and the user's 5 cm box with its noisy edge band
// (x 0.62 .. 0.93, y 0.10 .. 0.35; the robot-facing 5 cm reads uniform 3.5 .. 6.5 cm) on a 2 cm grid floor.
constexpr double SILL_X0_M = 1.0;
constexpr double SILL_X1_M = 1.15;
constexpr double SILL_HALF_WIDTH_M = 0.4;
constexpr double SILL_HEIGHT_M = 0.03;
constexpr double SILL_NOISE_M = 0.003;
constexpr double BOX_X0_M = 0.62, BOX_X1_M = 0.93, BOX_Y0_M = 0.10, BOX_Y1_M = 0.35;
constexpr double BOX_HEIGHT_M = 0.05;
constexpr double BOX_EDGE_BAND_M = 0.05;
constexpr double BOX_EDGE_LO_M = 0.035, BOX_EDGE_HI_M = 0.065;
constexpr double BOX_TOP_NOISE_M = 0.004;
constexpr double BOX_FLOOR_NOISE_M = 0.008;
constexpr double SCENE_GRID_M = 0.02;
constexpr int STEP_FRAMES = 12;
constexpr unsigned STEP_SEEDS = 8;

// Largest cluster above the floor of a filtered cloud, or nullopt.
std::optional<ground_estimator_cpp::Cluster> largest_step_cluster(const Cloud & filtered)
{
  const auto seg =
    ground_estimator_cpp::segment(filtered, Plane(), ground_estimator_cpp::BandsConfig{}, ROBOT_FLOOR_BAND_M);
  const auto clusters = ground_estimator_cpp::extract_clusters(
    seg.above, seg.above_height, "step", Plane(), ground_estimator_cpp::EdgesConfig{}, EDGE_BAND_M);
  std::optional<ground_estimator_cpp::Cluster> best;
  for (const auto & c : clusters) {
    if (!best || c.points.size() > best->points.size()) {
      best = c;
    }
  }
  return best;
}

// A pose of base_link in the fixed frame that puts the cell boundaries at a random place on the scene.
Eigen::Isometry3d random_cell_alignment(unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> offset(0.0, MAX_ALIGNMENT_OFFSET_M);
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.translate(Eigen::Vector3d(offset(rng), offset(rng), 0.0));
  return fixed_T_base;
}

}  // namespace

// A cell straddling a step edge fits an in-cell gradient as steady as a ramp's, but its points lie on two levels: it
// keeps its median height, so the sill stays a flat step (under the 2 deg ramp limit), not a 3 deg ramp.
TEST(CellMedian, SillEdgeCellsKeepTheirMedianHeight)
{
  for (unsigned seed = 1; seed <= STEP_SEEDS; ++seed) {
    const Eigen::Isometry3d fixed_T_base = random_cell_alignment(seed);
    CellMedianFilter filter{TemporalConfig{}};
    Cloud filtered;
    for (int frame = 0; frame < STEP_FRAMES; ++frame) {
      std::mt19937 rng(seed * 1000U + static_cast<unsigned>(frame));
      std::normal_distribution<double> noise(0.0, SILL_NOISE_M);
      Cloud cloud = grid_cloud(0.3, 1.4, -0.5, 0.5, SCENE_GRID_M);
      for (auto & p : cloud) {
        const bool on_sill = p.x >= SILL_X0_M && p.x <= SILL_X1_M && std::fabs(p.y) <= SILL_HALF_WIDTH_M;
        p.z = static_cast<float>((on_sill ? SILL_HEIGHT_M : 0.0) + noise(rng));
      }
      filtered = filter.apply(cloud, Plane(), fixed_T_base, frame * FRAME_DT_S);
    }
    const auto sill = largest_step_cluster(filtered);
    ASSERT_TRUE(sill.has_value()) << "seed " << seed;
    if (sill->slope_known) {
      EXPECT_LT(sill->slope_deg, ground_estimator_cpp::RAMP_MIN_SLOPE_DEG) << "seed " << seed;
    }
  }
}

// The box's edge cells mix floor, the noisy edge band and the top: they keep their median height, so the box keeps
// an edge step over the step rule in every frame instead of a smeared plane at its foot.
TEST(CellMedian, BoxEdgeCellsKeepTheStep)
{
  ground_estimator_cpp::TraversabilityConfig rule;
  rule.max_step_fraction = LEGACY_STEP_FRACTION;  // the 5 cm box is judged against the 40 mm step
  const double limit = ground_estimator_cpp::max_step_m(rule);
  for (unsigned seed = 1; seed <= STEP_SEEDS; ++seed) {
    const Eigen::Isometry3d fixed_T_base = random_cell_alignment(seed);
    CellMedianFilter filter{TemporalConfig{}};
    const TemporalConfig cfg;
    for (int frame = 0; frame < STEP_FRAMES; ++frame) {
      std::mt19937 rng(seed * 1000U + static_cast<unsigned>(frame));
      std::normal_distribution<double> floor_noise(0.0, BOX_FLOOR_NOISE_M);
      std::normal_distribution<double> top_noise(0.0, BOX_TOP_NOISE_M);
      std::uniform_real_distribution<double> edge(BOX_EDGE_LO_M, BOX_EDGE_HI_M);
      Cloud cloud = grid_cloud(0.3, 1.4, -0.4, 0.6, SCENE_GRID_M);
      for (auto & p : cloud) {
        const bool on_box = p.x >= BOX_X0_M && p.x <= BOX_X1_M && p.y >= BOX_Y0_M && p.y <= BOX_Y1_M;
        if (!on_box) {
          p.z = static_cast<float>(floor_noise(rng));
        } else if (p.x < BOX_X0_M + BOX_EDGE_BAND_M) {
          p.z = static_cast<float>(edge(rng));
        } else {
          p.z = static_cast<float>(BOX_HEIGHT_M + top_noise(rng));
        }
      }
      const Cloud filtered = filter.apply(cloud, Plane(), fixed_T_base, frame * FRAME_DT_S);
      if (frame < cfg.window_frames) {
        continue;
      }
      const auto box = largest_step_cluster(filtered);
      ASSERT_TRUE(box.has_value()) << "seed " << seed << " frame " << frame;
      EXPECT_GT(box->edge_step_m, limit) << "seed " << seed << " frame " << frame;
    }
  }
}
