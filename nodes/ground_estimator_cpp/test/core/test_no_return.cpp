#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include "ground_estimator_cpp/no_return.hpp"

namespace
{

using ground_estimator_cpp::CellKey;
using ground_estimator_cpp::ColourConfig;
using ground_estimator_cpp::FloorColourModel;
using ground_estimator_cpp::GuidanceSamples;
using ground_estimator_cpp::NoReturnConfig;
using ground_estimator_cpp::NoReturnTracker;
using ground_estimator_cpp::WarningArea;
using Polygon = std::vector<Eigen::Vector2d>;

constexpr double CELL_M = 0.1;
constexpr double FLOOR_BAND_M = 0.02;
constexpr double STEP_M = 0.02;  // sample spacing; samples sit at odd multiples of 1 cm, never on a cell border
constexpr double FLOOR_X0 = 0.3, FLOOR_X1 = 1.5, FLOOR_Y0 = -0.5, FLOOR_Y1 = 0.5;
// The black ramp: 25 x 30 cm. It covers cells ix 8..9, iy 0..2 fully and half of ix 10.
constexpr double RAMP_X0 = 0.80, RAMP_X1 = 1.05, RAMP_Y0 = 0.0, RAMP_Y1 = 0.30;
constexpr int FRAMES = 6;
constexpr unsigned SEED = 11;
const cv::Vec3b FLOOR_RGB(150, 140, 120);
const cv::Vec3b BLACK_RGB(20, 20, 24);
const cv::Vec3b WHITE_RGB(235, 235, 235);

// Where a sample of the floor grid goes: a depth return, a no-return sample, or nothing.
enum class Sample
{
  depth,
  no_return,
  none
};

using Painter = std::function<Sample(double x, double y)>;

bool in_rect(double x, double y, double x0, double y0, double x1, double y1)
{
  return x >= x0 && x < x1 && y >= y0 && y < y1;
}

// One guidance frame over the floor grid: depth returns at height 0 and no-return floor hits (z = 0), each with the
// colour the paint function gives, in base_link.
GuidanceSamples frame_of(const Painter & paint, const std::function<cv::Vec3b(double, double)> & colour)
{
  GuidanceSamples s;
  for (double x = FLOOR_X0 + 0.5 * STEP_M; x < FLOOR_X1; x += STEP_M) {
    for (double y = FLOOR_Y0 + 0.5 * STEP_M; y < FLOOR_Y1; y += STEP_M) {
      const pcl::PointXYZ p(static_cast<float>(x), static_cast<float>(y), 0.0F);
      switch (paint(x, y)) {
        case Sample::depth:
          s.points.push_back(p);
          s.heights.push_back(0.0F);
          s.colours.push_back(colour(x, y));
          break;
        case Sample::no_return:
          s.no_return.push_back(p);
          s.no_return_colours.push_back(colour(x, y));
          break;
        case Sample::none:
          break;
      }
    }
  }
  return s;
}

// The floor with returns everywhere.
GuidanceSamples floor_frame()
{
  return frame_of([](double, double) { return Sample::depth; }, [](double, double) { return FLOOR_RGB; });
}

// The floor with a no-return patch of colour patch_rgb.
GuidanceSamples patch_frame(double x0, double y0, double x1, double y1, const cv::Vec3b & patch_rgb = BLACK_RGB)
{
  return frame_of(
    [=](double x, double y) { return in_rect(x, y, x0, y0, x1, y1) ? Sample::no_return : Sample::depth; },
    [=](double x, double y) { return in_rect(x, y, x0, y0, x1, y1) ? patch_rgb : FLOOR_RGB; });
}

GuidanceSamples ramp_frame() { return patch_frame(RAMP_X0, RAMP_Y0, RAMP_X1, RAMP_Y1); }

ColourConfig colour_config()
{
  ColourConfig c;
  c.min_floor_samples = 50;
  return c;
}

// A floor colour model trained on noisy floor colours; ready when the training reaches min_floor_samples.
FloorColourModel floor_model(int samples = 400)
{
  FloorColourModel model(colour_config());
  std::mt19937 rng(SEED);
  std::uniform_int_distribution<int> noise(-6, 6);
  std::vector<cv::Vec3b> batch;
  for (int i = 0; i < samples; ++i) {
    batch.emplace_back(
      static_cast<uchar>(FLOOR_RGB[0] + noise(rng)), static_cast<uchar>(FLOOR_RGB[1] + noise(rng)),
      static_cast<uchar>(FLOOR_RGB[2] + noise(rng)));
  }
  model.update(batch);
  return model;
}

NoReturnConfig config()
{
  NoReturnConfig c;  // cell_min_samples 6, cell_min_share 0.8, window 4, enter 3, min_cells 2, clearance 1
  return c;
}

// Feed n frames of the same samples.
void feed(
  NoReturnTracker & t, const GuidanceSamples & s, int n, const FloorColourModel & model,
  const Eigen::Isometry3d & fixed_T_base = Eigen::Isometry3d::Identity(), const std::vector<CellKey> & blocked = {})
{
  for (int i = 0; i < n; ++i) {
    t.observe(s, model, colour_config(), fixed_T_base, blocked);
  }
}

bool inside(const Polygon & polygon, const Eigen::Vector2d & p)
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

void expect_no_return_area(const WarningArea & a)
{
  EXPECT_EQ(a.kind, "no_return");
  EXPECT_EQ(a.source, "rgb");
  EXPECT_FALSE(a.height_m.has_value());
  EXPECT_FALSE(a.slope_deg.has_value());
}

// Every polygon vertex lies within [x0, x1] x [y0, y1] (1 um slack).
void expect_within(const Polygon & polygon, double x0, double y0, double x1, double y1)
{
  for (const auto & v : polygon) {
    EXPECT_GE(v.x(), x0 - 1e-6);
    EXPECT_LE(v.x(), x1 + 1e-6);
    EXPECT_GE(v.y(), y0 - 1e-6);
    EXPECT_LE(v.y(), y1 + 1e-6);
  }
}

}  // namespace

TEST(NoReturn, BlackRampBecomesOneArea)
{
  const NoReturnConfig cfg = config();
  NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
  const FloorColourModel model = floor_model();
  ASSERT_TRUE(model.ready());
  // The robot stands 1 m forward and 2 m left of the fixed origin, turned by a quarter turn: the cells stay aligned.
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.translate(Eigen::Vector3d(1.0, 2.0, 0.0));
  fixed_T_base.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Isometry3d base_T_fixed = fixed_T_base.inverse();

  feed(t, ramp_frame(), cfg.enter_frames - 1, model, fixed_T_base);
  EXPECT_TRUE(t.areas(base_T_fixed).empty());
  feed(t, ramp_frame(), 1, model, fixed_T_base);
  const std::vector<WarningArea> areas = t.areas(base_T_fixed);
  ASSERT_EQ(areas.size(), 1u);
  expect_no_return_area(areas[0]);
  // The polygon (in base_link) covers every cell the ramp fully covers and stays inside the cells the ramp touches.
  for (int ix = 8; ix <= 9; ++ix) {
    for (int iy = 0; iy <= 2; ++iy) {
      EXPECT_TRUE(inside(areas[0].polygon, Eigen::Vector2d((ix + 0.5) * CELL_M, (iy + 0.5) * CELL_M)))
        << "cell " << ix << ", " << iy;
    }
  }
  expect_within(areas[0].polygon, 0.8, 0.0, 1.1, 0.3);
}

TEST(NoReturn, FloorColouredHoleIsNoArea)
{
  NoReturnTracker t(config(), CELL_M, FLOOR_BAND_M);
  // Glossy floor: no returns over the patch, but the RGB sees the floor colour.
  feed(t, patch_frame(RAMP_X0, RAMP_Y0, RAMP_X1, RAMP_Y1, FLOOR_RGB), FRAMES, floor_model());
  EXPECT_TRUE(t.areas(Eigen::Isometry3d::Identity()).empty());
  EXPECT_TRUE(t.stable_cells().empty());
}

TEST(NoReturn, ScatteredDarkSquaresAreNoArea)
{
  NoReturnTracker t(config(), CELL_M, FLOOR_BAND_M);
  // The board: 5 cm squares, the black ones without returns, the white ones with floor returns (about 50 % share).
  const auto black = [](double x, double y) {
    const bool on_board = in_rect(x, y, 0.8, 0.0, 1.2, 0.4);
    const int i = static_cast<int>(std::floor(x / 0.05));
    const int j = static_cast<int>(std::floor(y / 0.05));
    return on_board && (i + j) % 2 == 0;
  };
  const GuidanceSamples board = frame_of(
    [&](double x, double y) { return black(x, y) ? Sample::no_return : Sample::depth; },
    [&](double x, double y) {
      if (black(x, y)) {
        return BLACK_RGB;
      }
      return in_rect(x, y, 0.8, 0.0, 1.2, 0.4) ? WHITE_RGB : FLOOR_RGB;
    });
  ASSERT_GT(board.no_return.size(), 0u);
  feed(t, board, FRAMES, floor_model());
  EXPECT_TRUE(t.areas(Eigen::Isometry3d::Identity()).empty());
  EXPECT_TRUE(t.stable_cells().empty());
}

TEST(NoReturn, CellsNextToBlockedCellsAreExcluded)
{
  const FloorColourModel model = floor_model();
  // A box seen by depth just in front of the ramp (cells ix 7): the ramp cells next to it (ix 8) are its shadow.
  {
    NoReturnTracker t(config(), CELL_M, FLOOR_BAND_M);
    feed(t, ramp_frame(), FRAMES, model, Eigen::Isometry3d::Identity(), {{7, 0}, {7, 1}, {7, 2}});
    const std::vector<WarningArea> areas = t.areas(Eigen::Isometry3d::Identity());
    ASSERT_EQ(areas.size(), 1u);
    expect_no_return_area(areas[0]);
    expect_within(areas[0].polygon, 0.9, 0.0, 1.1, 0.3);
    const std::vector<CellKey> expected{{9, 0}, {9, 1}, {9, 2}};
    EXPECT_EQ(t.stable_cells(), expected);
  }
  // Blocked cells on the ramp's first column: every ramp cell is within one cell of one.
  {
    NoReturnTracker t(config(), CELL_M, FLOOR_BAND_M);
    feed(t, ramp_frame(), FRAMES, model, Eigen::Isometry3d::Identity(), {{8, 0}, {8, 1}, {8, 2}});
    EXPECT_TRUE(t.areas(Eigen::Isometry3d::Identity()).empty());
    EXPECT_TRUE(t.stable_cells().empty());
  }
  // A blocked cell two cells away does not exclude anything with clearance_cells 1.
  {
    NoReturnTracker t(config(), CELL_M, FLOOR_BAND_M);
    feed(t, ramp_frame(), FRAMES, model, Eigen::Isometry3d::Identity(), {{6, 1}});
    EXPECT_EQ(t.stable_cells().size(), 6u);
  }
}

TEST(NoReturn, ColourModelNotReadyGivesNothing)
{
  NoReturnTracker t(config(), CELL_M, FLOOR_BAND_M);
  const FloorColourModel model = floor_model(colour_config().min_floor_samples - 1);
  ASSERT_FALSE(model.ready());
  feed(t, ramp_frame(), FRAMES, model);
  EXPECT_TRUE(t.areas(Eigen::Isometry3d::Identity()).empty());
  EXPECT_TRUE(t.stable_cells().empty());
}

TEST(NoReturn, OneFrameBlipDoesNotEnter)
{
  const NoReturnConfig cfg = config();
  const FloorColourModel model = floor_model();
  const Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  {
    NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
    feed(t, ramp_frame(), 1, model);
    for (int i = 0; i < FRAMES; ++i) {
      EXPECT_TRUE(t.areas(id).empty()) << "frame " << i;
      feed(t, floor_frame(), 1, model);
    }
    EXPECT_TRUE(t.areas(id).empty());
  }
  // An area that loses its samples leaves once fewer than enter_frames of the window are no-return frames.
  {
    NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
    feed(t, ramp_frame(), cfg.enter_frames, model);
    ASSERT_EQ(t.areas(id).size(), 1u);
    // window 4, enter 3: one floor frame keeps 3 of the last 4, the second drops to 2.
    feed(t, floor_frame(), cfg.window_frames - cfg.enter_frames, model);
    EXPECT_EQ(t.areas(id).size(), 1u);
    feed(t, floor_frame(), 1, model);
    EXPECT_TRUE(t.areas(id).empty());
    EXPECT_TRUE(t.stable_cells().empty());
  }
  // Frames without any sample count as frames where the cell is no no-return cell.
  {
    NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
    feed(t, ramp_frame(), cfg.enter_frames, model);
    ASSERT_EQ(t.areas(id).size(), 1u);
    feed(t, GuidanceSamples{}, cfg.window_frames - cfg.enter_frames + 1, model);
    EXPECT_TRUE(t.areas(id).empty());
  }
}

TEST(NoReturn, SmallerThanMinCellsIsNoArea)
{
  const FloorColourModel model = floor_model();
  const Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  const GuidanceSamples one_cell = patch_frame(0.8, 0.0, 0.9, 0.1);
  NoReturnConfig cfg = config();
  ASSERT_EQ(cfg.min_cells, 2);
  {
    NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
    feed(t, one_cell, FRAMES, model);
    EXPECT_EQ(t.stable_cells().size(), 1u);
    EXPECT_TRUE(t.areas(id).empty());
  }
  cfg.min_cells = 1;
  {
    NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
    feed(t, one_cell, FRAMES, model);
    const std::vector<WarningArea> areas = t.areas(id);
    ASSERT_EQ(areas.size(), 1u);
    expect_no_return_area(areas[0]);
    expect_within(areas[0].polygon, 0.8, 0.0, 0.9, 0.1);
    EXPECT_EQ(areas[0].polygon.size(), 4u);
  }
}

TEST(NoReturn, ResetClears)
{
  const NoReturnConfig cfg = config();
  const FloorColourModel model = floor_model();
  const Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  NoReturnTracker t(cfg, CELL_M, FLOOR_BAND_M);
  feed(t, ramp_frame(), cfg.enter_frames, model);
  ASSERT_EQ(t.areas(id).size(), 1u);
  t.reset();
  EXPECT_TRUE(t.areas(id).empty());
  EXPECT_TRUE(t.stable_cells().empty());
  // The history is gone: enter_frames - 1 more frames do not bring the area back.
  feed(t, ramp_frame(), cfg.enter_frames - 1, model);
  EXPECT_TRUE(t.areas(id).empty());
}
