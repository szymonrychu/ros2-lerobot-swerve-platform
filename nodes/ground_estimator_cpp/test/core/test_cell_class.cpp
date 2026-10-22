#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "ground_estimator_cpp/cell_class.hpp"

namespace
{

using ground_estimator_cpp::CellClass;
using ground_estimator_cpp::CellClassFilter;
using ground_estimator_cpp::HysteresisConfig;
using ground_estimator_cpp::StableArea;
using ground_estimator_cpp::WarningArea;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<Eigen::Vector2d>;

constexpr double CELL_M = 0.1;
constexpr double JITTER_M = 0.02;
constexpr int STABLE_FRAMES = 20;
constexpr unsigned SEED = 7;
const Eigen::Vector2d BOX_XY(0.65, 0.15);  // centre of cell (6, 1)
const Eigen::Isometry3d IDENTITY = Eigen::Isometry3d::Identity();

// A cloud of a few points around xy at height z.
Cloud points_at(const Eigen::Vector2d & xy, float z)
{
  Cloud c;
  for (const double dx : {-0.02, 0.0, 0.02}) {
    c.push_back(pcl::PointXYZ(static_cast<float>(xy.x() + dx), static_cast<float>(xy.y()), z));
  }
  return c;
}

// An axis-aligned rectangle polygon, counter-clockwise.
Polygon rect(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

WarningArea area_of(const Polygon & polygon, const std::string & kind = "step", const std::string & source = "tof")
{
  WarningArea a;
  a.kind = kind;
  a.polygon = polygon;
  a.height_m = 0.03;
  a.slope_deg = 4.0;
  a.source = source;
  return a;
}

// One frame of the filter: a single class observation of the points (none observes nothing).
void frame(CellClassFilter & f, CellClass cls, const Eigen::Vector2d & xy = BOX_XY)
{
  f.begin_frame();
  if (cls != CellClass::none) {
    const float z = cls == CellClass::drop ? -0.08F : 0.05F;
    f.observe_points(points_at(xy, z), cls, IDENTITY);
  }
  f.end_frame();
}

void area_frame(CellClassFilter & f, const std::vector<WarningArea> & areas)
{
  f.begin_frame();
  for (const auto & a : areas) {
    f.observe_area(a, IDENTITY);
  }
  f.end_frame();
}

HysteresisConfig disabled()
{
  HysteresisConfig cfg;
  cfg.enabled = false;
  return cfg;
}

void expect_polygon_eq(const Polygon & actual, const Polygon & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i].x(), expected[i].x(), 1e-9) << "vertex " << i;
    EXPECT_NEAR(actual[i].y(), expected[i].y(), 1e-9) << "vertex " << i;
  }
}

}  // namespace

TEST(CellClass, OneLethalVoteNeverMakesACellLethal)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  for (int i = 0; i < HysteresisConfig{}.window_frames; ++i) {
    frame(f, CellClass::none);
    EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  }
  EXPECT_EQ(f.count(CellClass::obstacle), 0u);
}

TEST(CellClass, ThreeConsecutiveLethalVotesMakeItLethalOnTheThirdFrame)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle);
  EXPECT_EQ(f.count(CellClass::obstacle), 1u);
  // The neighbouring cell is untouched.
  EXPECT_EQ(f.stable_at(BOX_XY + Eigen::Vector2d(CELL_M, 0.0)), CellClass::none);
}

TEST(CellClass, BoxCellAlternatingObstacleAndWarningStaysObstacleOnceEntered)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  bool entered = false;
  for (int i = 0; i < STABLE_FRAMES; ++i) {
    frame(f, i % 2 == 0 ? CellClass::obstacle : CellClass::warning);
    if (entered) {
      EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle) << "frame " << i;
    }
    entered = entered || f.stable_at(BOX_XY) == CellClass::obstacle;
  }
  EXPECT_TRUE(entered);
}

TEST(CellClass, LethalCellLeavesLethalAfterWindowFramesWithoutLethalVotes)
{
  const HysteresisConfig cfg;  // window 5, enter 3, exit 1
  CellClassFilter f(cfg, CELL_M);
  for (int i = 0; i < cfg.enter_frames; ++i) {
    frame(f, CellClass::obstacle);
  }
  ASSERT_EQ(f.stable_at(BOX_XY), CellClass::obstacle);
  // The last lethal vote stays in the window for window_frames - 1 more frames.
  for (int i = 0; i < cfg.window_frames - 1; ++i) {
    frame(f, CellClass::none);
    EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle) << "frame " << i;
  }
  frame(f, CellClass::none);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  EXPECT_EQ(f.count(CellClass::obstacle), 0u);
  EXPECT_EQ(f.count(CellClass::none), 0u);  // a window of only none is forgotten
}

TEST(CellClass, CellAlternatingWarningAndNoneStaysWarningOnceEntered)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  bool entered = false;
  for (int i = 0; i < STABLE_FRAMES; ++i) {
    frame(f, i % 2 == 0 ? CellClass::warning : CellClass::none);
    if (entered) {
      EXPECT_EQ(f.stable_at(BOX_XY), CellClass::warning) << "frame " << i;
    }
    entered = entered || f.stable_at(BOX_XY) == CellClass::warning;
  }
  EXPECT_TRUE(entered);
}

TEST(CellClass, WarningBlockGivesOneAreaWithAStablePolygonUnderJitter)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  std::mt19937 rng(SEED);
  std::uniform_real_distribution<double> jitter(-JITTER_M, JITTER_M);
  // 3 x 2 cells: x 0.3..0.6, y 0.0..0.2.
  const Polygon expected = rect(0.3, 0.0, 0.6, 0.2);
  Polygon first;
  int frames_with_area = 0;
  for (int i = 0; i < STABLE_FRAMES + HysteresisConfig{}.enter_frames; ++i) {
    Polygon observed = expected;
    for (auto & v : observed) {
      v += Eigen::Vector2d(jitter(rng), jitter(rng));
    }
    area_frame(f, {area_of(observed)});
    const std::vector<StableArea> areas = f.warning_areas();
    if (i + 1 < HysteresisConfig{}.enter_frames) {
      EXPECT_TRUE(areas.empty());
      continue;
    }
    ASSERT_EQ(areas.size(), 1u) << "frame " << i;
    expect_polygon_eq(areas[0].polygon, expected);
    if (first.empty()) {
      first = areas[0].polygon;
    }
    ASSERT_EQ(areas[0].polygon.size(), first.size());
    for (std::size_t k = 0; k < first.size(); ++k) {
      EXPECT_EQ(areas[0].polygon[k], first[k]) << "frame " << i << " vertex " << k;  // bit-identical
    }
    ++frames_with_area;
  }
  EXPECT_EQ(frames_with_area, STABLE_FRAMES + 1);
  EXPECT_EQ(f.count(CellClass::warning), 6u);
}

TEST(CellClass, TwoSeparateBlocksGiveTwoAreasInKeyOrder)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  const Polygon far = rect(1.0, 0.0, 1.2, 0.2);
  const Polygon near = rect(0.3, -0.3, 0.5, -0.1);
  for (int i = 0; i < HysteresisConfig{}.enter_frames; ++i) {
    area_frame(f, {area_of(far, "ramp"), area_of(near, "colour", "colour")});
  }
  const std::vector<StableArea> areas = f.warning_areas();
  ASSERT_EQ(areas.size(), 2u);
  expect_polygon_eq(areas[0].polygon, near);  // smallest cell key (3, -3) first
  EXPECT_EQ(areas[0].kind, "colour");
  EXPECT_EQ(areas[0].source, "colour");
  expect_polygon_eq(areas[1].polygon, far);
  EXPECT_EQ(areas[1].kind, "ramp");
  EXPECT_EQ(areas[1].source, "tof");
}

TEST(CellClass, DiagonalNeighboursAreOneArea)
{
  CellClassFilter f(disabled(), CELL_M);
  // Cells (0, 0) and (1, 1): 8-connected, one hull over both squares.
  area_frame(f, {area_of(rect(0.0, 0.0, 0.1, 0.1)), area_of(rect(0.1, 0.1, 0.2, 0.2))});
  const std::vector<StableArea> areas = f.warning_areas();
  ASSERT_EQ(areas.size(), 1u);
  expect_polygon_eq(areas[0].polygon, {{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.1}, {0.2, 0.2}, {0.1, 0.2}, {0.0, 0.1}});
}

TEST(CellClass, AreaAttributesAreTheCellMajorityAndMedians)
{
  CellClassFilter f(disabled(), CELL_M);
  WarningArea a = area_of(rect(0.0, 0.0, 0.2, 0.1), "step", "colour");  // cells (0, 0), (1, 0)
  a.height_m = 0.02;
  a.slope_deg = 2.0;
  WarningArea b = area_of(rect(0.2, 0.0, 0.3, 0.1), "ramp", "tof");  // cell (2, 0)
  b.height_m = 0.05;
  b.slope_deg = 11.0;
  area_frame(f, {a, b});
  const std::vector<StableArea> areas = f.warning_areas();
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_EQ(areas[0].kind, "step");
  ASSERT_TRUE(areas[0].height_m.has_value());
  EXPECT_DOUBLE_EQ(*areas[0].height_m, 0.02);
  ASSERT_TRUE(areas[0].slope_deg.has_value());
  EXPECT_DOUBLE_EQ(*areas[0].slope_deg, 2.0);
  EXPECT_EQ(areas[0].source, "tof");

  // A 1:1 tie of kinds picks the lexicographically smallest; even counts take the mean of the middle values.
  CellClassFilter g(disabled(), CELL_M);
  WarningArea c = area_of(rect(0.0, 0.0, 0.1, 0.1), "step", "colour");
  c.height_m = 0.02;
  WarningArea d = area_of(rect(0.1, 0.0, 0.2, 0.1), "drop", "colour");
  d.height_m = -0.03;
  area_frame(g, {c, d});
  const std::vector<StableArea> tied = g.warning_areas();
  ASSERT_EQ(tied.size(), 1u);
  EXPECT_EQ(tied[0].kind, "drop");
  ASSERT_TRUE(tied[0].height_m.has_value());
  EXPECT_NEAR(*tied[0].height_m, -0.005, 1e-12);
  EXPECT_EQ(tied[0].source, "colour");
}

TEST(CellClassFilter, AreaWithoutRecordedSlopeHasNoSlope)
{
  CellClassFilter f(disabled(), CELL_M);
  WarningArea a = area_of(rect(0.0, 0.0, 0.2, 0.1), "step", "colour");
  a.height_m = std::nullopt;
  a.slope_deg = std::nullopt;
  area_frame(f, {a});
  const std::vector<StableArea> areas = f.warning_areas();
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_FALSE(areas[0].height_m.has_value());
  EXPECT_FALSE(areas[0].slope_deg.has_value());
}

TEST(CellClass, DropOrObstacleByTheLatestLethalVote)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  for (int i = 0; i < 3; ++i) {
    frame(f, CellClass::obstacle);
  }
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle);
  frame(f, CellClass::drop);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::drop);
  EXPECT_EQ(f.count(CellClass::drop), 1u);
  EXPECT_EQ(f.count(CellClass::obstacle), 0u);
  frame(f, CellClass::warning);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::drop);  // a warning vote does not change the lethal kind
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle);
}

TEST(CellClass, LethalBeatsWarningWithinAFrame)
{
  CellClassFilter f(disabled(), CELL_M);
  f.begin_frame();
  f.observe_points(points_at(BOX_XY, 0.05F), CellClass::obstacle, IDENTITY);
  f.observe_points(points_at(BOX_XY, 0.02F), CellClass::warning, IDENTITY);
  f.observe_area(area_of(rect(0.6, 0.1, 0.7, 0.2)), IDENTITY);
  f.end_frame();
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle);
  EXPECT_TRUE(f.warning_areas().empty());
}

TEST(CellClass, ObserveAreaMarksCentreInsideCellsOnlyPlusTheCentroidCell)
{
  CellClassFilter f(disabled(), CELL_M);
  // x 0.04..0.26 covers the centres 0.05, 0.15, 0.25 of cells 0..2 but not -0.05 or 0.35; y 0.0..0.1 is row 0 only.
  area_frame(f, {area_of(rect(0.04, 0.0, 0.26, 0.1))});
  EXPECT_EQ(f.stable_at({0.05, 0.05}), CellClass::warning);
  EXPECT_EQ(f.stable_at({0.15, 0.05}), CellClass::warning);
  EXPECT_EQ(f.stable_at({0.25, 0.05}), CellClass::warning);
  EXPECT_EQ(f.stable_at({-0.05, 0.05}), CellClass::none);
  EXPECT_EQ(f.stable_at({0.35, 0.05}), CellClass::none);
  EXPECT_EQ(f.count(CellClass::warning), 3u);

  // A small area covering no cell centre marks the cell of its centroid.
  area_frame(f, {area_of(rect(0.61, 0.11, 0.64, 0.14))});
  EXPECT_EQ(f.stable_at({0.65, 0.15}), CellClass::warning);
  EXPECT_EQ(f.count(CellClass::warning), 1u);

  // An area straddling four cells without covering any centre marks only its centroid cell (0.495 -> (4, 4)).
  area_frame(f, {area_of(rect(0.47, 0.47, 0.52, 0.52))});
  EXPECT_EQ(f.count(CellClass::warning), 1u);
  EXPECT_EQ(f.stable_at({0.45, 0.45}), CellClass::warning);
}

TEST(CellClass, CellsAreInTheFixedFrame)
{
  CellClassFilter f(disabled(), CELL_M);
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
  fixed_T_base.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  f.begin_frame();
  f.observe_points(points_at(BOX_XY, 0.05F), CellClass::obstacle, fixed_T_base);
  f.observe_area(area_of(rect(0.3, 0.0, 0.4, 0.1)), fixed_T_base);
  f.end_frame();
  // base (0.65, 0.15) -> fixed (1 - 0.15, 2 + 0.65); base cell (3, 0) centre (0.35, 0.05) -> (0.95, 2.35).
  EXPECT_EQ(f.stable_at({0.85, 2.65}), CellClass::obstacle);
  EXPECT_EQ(f.stable_at({0.95, 2.35}), CellClass::warning);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
}

TEST(CellClass, ResetForgetsEverything)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  for (int i = 0; i < 3; ++i) {
    f.begin_frame();
    f.observe_points(points_at(BOX_XY, 0.05F), CellClass::obstacle, IDENTITY);
    f.observe_area(area_of(rect(0.0, 0.0, 0.2, 0.2)), IDENTITY);
    f.end_frame();
  }
  ASSERT_EQ(f.count(CellClass::obstacle), 1u);
  ASSERT_EQ(f.warning_areas().size(), 1u);
  f.reset();
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  EXPECT_EQ(f.count(CellClass::obstacle), 0u);
  EXPECT_EQ(f.count(CellClass::warning), 0u);
  EXPECT_EQ(f.count(CellClass::none), 0u);
  EXPECT_TRUE(f.warning_areas().empty());
  // The history is gone: one more lethal vote does not reach enter_frames.
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
}

// --- area stability (hysteresis.area_*) ---------------------------------------------------------------------------

// A 3 x 2 core block plus a border cell (3, 0) next to it.
const Polygon CORE = rect(0.0, 0.0, 0.3, 0.2);
const Polygon BORDER_CELL = rect(0.3, 0.0, 0.4, 0.1);
const Polygon CORE_WITH_BORDER = {{0.0, 0.0}, {0.4, 0.0}, {0.4, 0.1}, {0.3, 0.2}, {0.0, 0.2}};

// Class votes that flip a cell's stable class every frame (2 of 3 to enter, 2 to stay), no area gating.
HysteresisConfig ungated_flickering()
{
  HysteresisConfig cfg;
  cfg.window_frames = 3;
  cfg.enter_frames = 2;
  cfg.exit_frames = 2;
  cfg.area_enter_frames = 1;
  cfg.area_join_frames = 1;
  cfg.area_min_cells = 1;
  cfg.area_enter_density = 0.0;
  cfg.area_exit_density = 0.0;
  return cfg;
}

// The same class votes with the shipped area settings.
HysteresisConfig gated_flickering()
{
  HysteresisConfig cfg;
  cfg.window_frames = 3;
  cfg.enter_frames = 2;
  cfg.exit_frames = 2;
  return cfg;
}

// Frames from the third on whose single area is exactly the core, with the border cell seen every other frame.
int frames_with_core_area(const HysteresisConfig & cfg, int frames)
{
  CellClassFilter f(cfg, CELL_M);
  int identical = 0;
  for (int i = 0; i < frames; ++i) {
    std::vector<WarningArea> seen{area_of(CORE)};
    if (i % 2 == 0) {
      seen.push_back(area_of(BORDER_CELL));
    }
    area_frame(f, seen);
    const std::vector<StableArea> areas = f.warning_areas();
    if (
      i >= 2 && areas.size() == 1 && areas[0].polygon.size() == CORE.size() &&
      areas[0].polygon[2].isApprox(Eigen::Vector2d(0.3, 0.2))) {
      ++identical;
    }
  }
  return identical;
}

TEST(CellClass, BorderCellFlickeringEveryOtherFrameLeavesOneIdenticalAreaEveryFrame)
{
  constexpr int FRAMES = 20;
  // Ungated, the border cell reshapes the area every time it is stable.
  EXPECT_LT(frames_with_core_area(ungated_flickering(), FRAMES), FRAMES - 2 - 3);
  // Gated, the core is the area on every frame from the third (the core's second vote makes it stable on the second
  // frame and its third active frame publishes it).
  EXPECT_EQ(frames_with_core_area(gated_flickering(), FRAMES), FRAMES - 2);
}

TEST(CellClass, BorderCellJoinsAPublishedAreaOnItsAreaJoinFramesThActiveFrame)
{
  const HysteresisConfig cfg;  // area_join_frames 8
  CellClassFilter f(cfg, CELL_M);
  for (int i = 0; i < 3; ++i) {
    area_frame(f, {area_of(CORE)});
  }
  ASSERT_EQ(f.warning_areas().size(), 1u);
  expect_polygon_eq(f.warning_areas()[0].polygon, CORE);
  for (int k = 1; k <= cfg.area_join_frames; ++k) {
    area_frame(f, {area_of(CORE), area_of(BORDER_CELL)});
    ASSERT_EQ(f.warning_areas().size(), 1u) << "border frame " << k;
    if (k < cfg.area_join_frames) {
      expect_polygon_eq(f.warning_areas()[0].polygon, CORE);
    }
  }
  expect_polygon_eq(f.warning_areas()[0].polygon, CORE_WITH_BORDER);
}

TEST(CellClass, ACellVotingTwoFramesOfThreeIsStableButNeverJoinsAnArea)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  for (int i = 0; i < 30; ++i) {
    std::vector<WarningArea> seen{area_of(CORE)};
    if (i % 3 != 2) {
      seen.push_back(area_of(BORDER_CELL));
    }
    area_frame(f, seen);
    if (i >= 2) {
      ASSERT_EQ(f.warning_areas().size(), 1u) << "frame " << i;
      expect_polygon_eq(f.warning_areas()[0].polygon, CORE);
    }
  }
  EXPECT_EQ(f.stable_at(Eigen::Vector2d(0.35, 0.05)), CellClass::warning);
  EXPECT_EQ(f.count(CellClass::warning), 7u);
}

TEST(CellClass, AJoinedCellVotingOnceInFiveFramesStaysStableButLeavesTheArea)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  for (int i = 0; i < 20; ++i) {
    area_frame(f, {area_of(CORE), area_of(BORDER_CELL)});
  }
  ASSERT_EQ(f.warning_areas().size(), 1u);
  expect_polygon_eq(f.warning_areas()[0].polygon, CORE_WITH_BORDER);
  // One vote in five frames keeps the cell stable (exit_frames 1) while its density falls below 0.25 within 20 frames.
  for (int i = 0; i < 40; ++i) {
    std::vector<WarningArea> seen{area_of(CORE)};
    if (i % 5 == 0) {
      seen.push_back(area_of(BORDER_CELL));
    }
    area_frame(f, seen);
  }
  EXPECT_EQ(f.stable_at(Eigen::Vector2d(0.35, 0.05)), CellClass::warning);
  ASSERT_EQ(f.warning_areas().size(), 1u);
  expect_polygon_eq(f.warning_areas()[0].polygon, CORE);
}

TEST(CellClass, AreaIsGoneAsSoonAsItsCellsAreNoLongerStableWarningCells)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  for (int i = 0; i < 6; ++i) {
    area_frame(f, {area_of(CORE)});
  }
  ASSERT_EQ(f.warning_areas().size(), 1u);
  // exit_frames 1: the cells stay stable until window_frames - 1 empty frames have passed, then nothing is kept.
  for (int i = 0; i < HysteresisConfig{}.window_frames - 1; ++i) {
    area_frame(f, {});
    EXPECT_EQ(f.warning_areas().size(), 1u) << "empty frame " << i;
  }
  area_frame(f, {});
  EXPECT_TRUE(f.warning_areas().empty());
  EXPECT_EQ(f.count(CellClass::warning), 0u);
}

TEST(CellClass, ABlipSmallerThanAreaMinCellsIsNotPublishedButItsCellsKeepTheirClass)
{
  const HysteresisConfig cfg;  // area_min_cells 3
  CellClassFilter f(cfg, CELL_M);
  for (int i = 0; i < 6; ++i) {
    area_frame(f, {area_of(rect(0.0, 0.0, 0.2, 0.1))});  // two cells
  }
  EXPECT_TRUE(f.warning_areas().empty());
  EXPECT_EQ(f.count(CellClass::warning), 2u);
  EXPECT_EQ(f.stable_at(Eigen::Vector2d(0.05, 0.05)), CellClass::warning);
  // A third cell makes a component of area_min_cells: published.
  for (int i = 0; i < cfg.area_join_frames; ++i) {
    area_frame(f, {area_of(rect(0.0, 0.0, 0.3, 0.1))});
  }
  ASSERT_EQ(f.warning_areas().size(), 1u);
  expect_polygon_eq(f.warning_areas()[0].polygon, rect(0.0, 0.0, 0.3, 0.1));
}

TEST(CellClass, ABlockFlipFloppingBetweenStepAndRampKeepsItsPublishedKind)
{
  const HysteresisConfig cfg;  // area_kind_frames 10
  CellClassFilter f(cfg, CELL_M);
  for (int i = 0; i < 3; ++i) {
    area_frame(f, {area_of(CORE, "step")});
  }
  ASSERT_EQ(f.warning_areas().size(), 1u);
  EXPECT_EQ(f.warning_areas()[0].kind, "step");
  for (int i = 0; i < 40; ++i) {
    area_frame(f, {area_of(CORE, i % 2 == 0 ? "ramp" : "step")});
    ASSERT_EQ(f.warning_areas().size(), 1u);
    EXPECT_EQ(f.warning_areas()[0].kind, "step") << "frame " << i;
  }
  // Another kind with most of the cells for area_kind_frames frames in a row takes over on the last of them.
  for (int i = 1; i <= cfg.area_kind_frames; ++i) {
    area_frame(f, {area_of(CORE, "ramp")});
    EXPECT_EQ(f.warning_areas()[0].kind, i < cfg.area_kind_frames ? "step" : "ramp") << "ramp frame " << i;
  }
}

TEST(CellClass, ANewKindNeedsMostOfTheAreasCellsNotJustSome)
{
  CellClassFilter f(HysteresisConfig{}, CELL_M);
  for (int i = 0; i < 3; ++i) {
    area_frame(f, {area_of(CORE, "step")});
  }
  // One of six cells reads ramp every frame: never most of the cells.
  for (int i = 0; i < 30; ++i) {
    area_frame(
      f, {area_of(rect(0.0, 0.0, 0.2, 0.2), "step"), area_of(rect(0.2, 0.0, 0.3, 0.1), "ramp"),
          area_of(rect(0.2, 0.1, 0.3, 0.2), "step")});
    ASSERT_EQ(f.warning_areas().size(), 1u);
    EXPECT_EQ(f.warning_areas()[0].kind, "step") << "frame " << i;
  }
}

TEST(CellClass, DisabledPublishesEveryComponentAtOnce)
{
  CellClassFilter f(disabled(), CELL_M);
  area_frame(f, {area_of(rect(0.0, 0.0, 0.1, 0.1), "step")});
  ASSERT_EQ(f.warning_areas().size(), 1u);
  area_frame(f, {area_of(rect(0.0, 0.0, 0.1, 0.1), "ramp")});
  ASSERT_EQ(f.warning_areas().size(), 1u);
  EXPECT_EQ(f.warning_areas()[0].kind, "ramp");
}

TEST(CellClass, DisabledIsPassThrough)
{
  CellClassFilter f(disabled(), CELL_M);
  frame(f, CellClass::obstacle);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::obstacle);
  frame(f, CellClass::warning);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::warning);
  frame(f, CellClass::drop);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::drop);
  frame(f, CellClass::none);
  EXPECT_EQ(f.stable_at(BOX_XY), CellClass::none);
  EXPECT_EQ(f.count(CellClass::none), 0u);
}
