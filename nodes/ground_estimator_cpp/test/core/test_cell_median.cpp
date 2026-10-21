#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>

#include "ground_estimator_cpp/cell_median.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::CellMedianFilter;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::TemporalConfig;
using ground_test::add_z_noise;
using ground_test::grid_cloud;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

constexpr double SIGMA_M = 0.026;
constexpr double OUTLIER_FRACTION = 0.2;
constexpr double OUTLIER_RANGE_M = 0.06;
constexpr double STEP_RULE_M = 0.04;
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

TEST(CellMedian, FarBandNoiseCellsAreGatedAndNeverOutput)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  std::mt19937 rng(11);
  std::size_t gated_seen = 0;
  std::size_t cell_frames = 0;
  std::size_t leaked_cell_frames = 0;
  for (int i = 0; i < 40; ++i) {
    const Cloud in = cell_cloud(1.2, 1.5, [&](int ix, int iy) {
      // Floor + U(+0.04, +0.04 + width): width 0.15..0.31 m gives a temporal std of 43..90 mm.
      const double width = 0.15 + 0.032 * ((ix * 3 + iy + 30) % 6);
      return 0.04 + std::uniform_real_distribution<double>(0.0, width)(rng);
    });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= cfg.window_frames) {
      cell_frames += in.size() / 4;
      leaked_cell_frames += out.size() / 4;
      gated_seen += filter.gated_cell_count();
    }
  }
  EXPECT_GT(gated_seen, 0u);
  // The MAD of a handful of samples is itself noisy, so a rare cell slips under the limit for a frame.
  EXPECT_LE(static_cast<double>(leaked_cell_frames) / static_cast<double>(cell_frames), 0.05);
}

TEST(CellMedian, MeasuredFloorNoiseStaysVisible)
{
  const TemporalConfig cfg;
  CellMedianFilter filter(cfg);
  std::mt19937 rng(5);
  std::size_t frames_with_output = 0;
  std::size_t cell_frames = 0;
  std::size_t expected_cells = 0;
  for (int i = 0; i < 40; ++i) {
    const Cloud in = cell_cloud(0.4, 1.0, [&](int ix, int iy) {
      const double sigma = 0.004 + 0.016 * (((ix * 7 + iy + 30) % 11) / 10.0);  // 4..20 mm
      return std::normal_distribution<double>(0.0, sigma)(rng);
    });
    const Cloud out = filter.apply(in, Plane(), Eigen::Isometry3d::Identity(), i * FRAME_DT_S);
    if (i >= cfg.min_frames) {
      expected_cells += in.size() / 4;
      cell_frames += out.size() / 4;
      ++frames_with_output;
    }
  }
  ASSERT_GT(frames_with_output, 0u);
  EXPECT_GE(static_cast<double>(cell_frames) / static_cast<double>(expected_cells), 0.95);
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
