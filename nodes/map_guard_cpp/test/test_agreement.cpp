// Port of nodes/map_guard/tests/test_agreement.py (synthetic square room) plus the agreement.json golden fixture:
// the same endpoints, map-frame points, known cells and agreement ratio / counts as the Python functions.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "fixture_util.hpp"
#include "map_guard_cpp/agreement.hpp"

namespace mg = map_guard_cpp;
namespace fx = map_guard_cpp::test;

namespace
{

constexpr double RESOLUTION = 0.05;
constexpr double HALF_ROOM_M = 3.0;
constexpr int SIZE = 200;  // 10 m x 10 m, origin at (-5, -5)
constexpr double ROBOT_X = 0.5;
constexpr double ROBOT_Y = 0.2;
constexpr double ROBOT_YAW = 0.3;
constexpr int N_BEAMS = 360;
constexpr double INF = std::numeric_limits<double>::infinity();

// Square room: walls at |x| = 3 or |y| = 3 (occupied), free inside, unknown outside.
mg::GridMap room_grid()
{
  mg::GridMap grid;
  grid.resolution = RESOLUTION;
  grid.origin_x = -5.0;
  grid.origin_y = -5.0;
  grid.width = SIZE;
  grid.height = SIZE;
  grid.cells.assign(SIZE * SIZE, mg::UNKNOWN);
  for (int row = 0; row < SIZE; ++row) {
    for (int col = 0; col < SIZE; ++col) {
      const double x = -5.0 + (col + 0.5) * RESOLUTION;
      const double y = -5.0 + (row + 0.5) * RESOLUTION;
      std::int8_t value = mg::UNKNOWN;
      if (std::abs(x) < HALF_ROOM_M && std::abs(y) < HALF_ROOM_M) {
        value = 0;
      }
      const bool wall =
        (std::abs(std::abs(x) - HALF_ROOM_M) < RESOLUTION / 2 + 1e-9 && std::abs(y) <= HALF_ROOM_M + RESOLUTION) ||
        (std::abs(std::abs(y) - HALF_ROOM_M) < RESOLUTION / 2 + 1e-9 && std::abs(x) <= HALF_ROOM_M + RESOLUTION);
      if (wall) {
        value = 100;
      }
      grid.cells[row * SIZE + col] = value;
    }
  }
  return grid;
}

// Ray-cast the room from a pose; ranges with angle_min -pi and increment 2 pi / n.
std::vector<double> room_scan(double x, double y, double yaw, int n = N_BEAMS)
{
  std::vector<double> ranges(n);
  const double increment = 2 * M_PI / n;
  for (int i = 0; i < n; ++i) {
    const double a = yaw - M_PI + i * increment;
    const double dx = std::cos(a);
    const double dy = std::sin(a);
    double best = INF;
    for (double t :
         {std::abs(dx) > 1e-9 ? (HALF_ROOM_M - x) / dx : -1.0, std::abs(dx) > 1e-9 ? (-HALF_ROOM_M - x) / dx : -1.0,
          std::abs(dy) > 1e-9 ? (HALF_ROOM_M - y) / dy : -1.0, std::abs(dy) > 1e-9 ? (-HALF_ROOM_M - y) / dy : -1.0}) {
      if (t > 0 && t < best) {
        best = t;
      }
    }
    ranges[i] = best;
  }
  return ranges;
}

std::optional<mg::AgreementResult> score(
  double tx, double ty, double tyaw, std::optional<std::vector<double>> ranges = std::nullopt,
  std::optional<mg::GridMap> grid = std::nullopt)
{
  const std::vector<double> r = ranges ? *ranges : room_scan(ROBOT_X, ROBOT_Y, ROBOT_YAW);
  const mg::Points points = mg::scan_endpoints(r, -M_PI, 2 * M_PI / r.size(), 0.15, 12.0, 8.0);
  const mg::Points in_map = mg::transform_points(points, {tx, ty, 0.0}, mg::yaw_quaternion(tyaw));
  const mg::PreparedMap prepared = mg::prepare_map(grid ? *grid : room_grid(), 65, 1);
  return mg::agreement(in_map, &prepared, 40);
}

long count(const mg::Mask & mask)  // NOLINT(runtime/int)
{
  long n = 0;  // NOLINT(runtime/int)
  for (auto v : mask) {
    n += v;
  }
  return n;
}

mg::MismatchMonitor monitor() { return mg::MismatchMonitor(0.3, 0.6, 5.0, 1000, 5); }

// Feed samples at 2 Hz in [start, end); returns the last result.
bool feed(mg::MismatchMonitor & mon, double start, double end, double ratio, long known = 5000)  // NOLINT
{
  bool fired = false;
  for (double t = start; t < end - 1e-9; t += 0.5) {
    fired = mon.update(t, ratio, known);
  }
  return fired;
}

}  // namespace

TEST(Agreement, ScanEndpointsFilterRanges)
{
  const std::vector<double> ranges = {1.0, INF, std::nan(""), 0.05, 9.0, 2.0};
  const auto points = mg::scan_endpoints(ranges, 0.0, M_PI / 2, 0.15, 12.0, 8.0);
  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].x, 1.0, 1e-9);
  EXPECT_NEAR(points[0].y, 0.0, 1e-9);
  EXPECT_NEAR(points[1].x, 0.0, 1e-9);
  EXPECT_NEAR(points[1].y, 2.0, 1e-9);
}

TEST(Agreement, ScanEndpointsRespectRangeMaxBelowMaxRange)
{
  EXPECT_EQ(mg::scan_endpoints(std::vector<double>{5.0, 3.0}, 0.0, 0.1, 0.15, 4.0, 8.0).size(), 1u);
}

TEST(Agreement, ScanEndpointsOfFloatRanges)
{
  // LaserScan ranges are float32: the same filter applies.
  const auto points = mg::scan_endpoints(std::vector<float>{1.0f, INFINITY, 0.05f}, 0.0, 0.1, 0.15, 12.0, 8.0);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, 1.0, 1e-9);
}

TEST(Agreement, TransformPointsRotatesThenTranslates)
{
  const auto out = mg::transform_points({{1.0, 0.0}}, {2.0, 3.0, 0.0}, mg::yaw_quaternion(M_PI / 2));
  EXPECT_NEAR(out[0].x, 2.0, 1e-9);
  EXPECT_NEAR(out[0].y, 4.0, 1e-9);
}

TEST(Agreement, TransformPointsHandlesAnUpsideDownMount)
{
  const auto out = mg::transform_points({{1.0, 1.0}}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(out[0].x, 1.0, 1e-9);
  EXPECT_NEAR(out[0].y, -1.0, 1e-9);
}

TEST(Agreement, QuaternionYawRoundTrips)
{
  for (double yaw : {0.0, 0.3, M_PI / 2, -2.5}) {
    EXPECT_NEAR(mg::quaternion_yaw(mg::yaw_quaternion(yaw)), yaw, 1e-12);
  }
}

TEST(Agreement, DilateSquareRadius)
{
  mg::Mask mask(25, 0);
  mask[2 * 5 + 2] = 1;
  EXPECT_EQ(count(mg::dilate(mask, 5, 5, 1)), 9);
  EXPECT_EQ(count(mg::dilate(mask, 5, 5, 0)), 1);
  EXPECT_EQ(count(mg::dilate(mask, 5, 5, 2)), 25);
}

TEST(Agreement, PerfectAlignmentScoresHigh)
{
  const auto result = score(ROBOT_X, ROBOT_Y, ROBOT_YAW);
  ASSERT_TRUE(result);
  EXPECT_GT(result->ratio, 0.95);
  EXPECT_GE(result->evaluated, 300);
}

TEST(Agreement, ShiftedTransformScoresLow)
{
  const auto result = score(ROBOT_X + 0.4, ROBOT_Y + 0.4, ROBOT_YAW);
  ASSERT_TRUE(result);
  EXPECT_LT(result->ratio, 0.3);
}

TEST(Agreement, RotatedTransformScoresLow)
{
  const auto result = score(ROBOT_X, ROBOT_Y, ROBOT_YAW + 0.35);
  ASSERT_TRUE(result);
  EXPECT_LT(result->ratio, 0.3);
}

TEST(Agreement, HitRadiusAcceptsOneCellOffOnly)
{
  const auto one_cell = score(ROBOT_X + RESOLUTION * 0.9, ROBOT_Y, ROBOT_YAW);
  ASSERT_TRUE(one_cell);
  EXPECT_GT(one_cell->ratio, 0.9);
  const auto three_cells = score(ROBOT_X + RESOLUTION * 3, ROBOT_Y + RESOLUTION * 3, ROBOT_YAW);
  ASSERT_TRUE(three_cells);
  EXPECT_LT(three_cells->ratio, 0.3);
}

TEST(Agreement, EmptyMapIsSkipped)
{
  mg::GridMap empty = room_grid();
  std::fill(empty.cells.begin(), empty.cells.end(), mg::UNKNOWN);
  EXPECT_FALSE(score(ROBOT_X, ROBOT_Y, ROBOT_YAW, std::nullopt, empty));
  EXPECT_EQ(mg::prepare_map(empty, 65, 1).known_cells, 0);
}

TEST(Agreement, MissingMapIsSkipped) { EXPECT_FALSE(mg::agreement(mg::Points(100, {0.0, 0.0}), nullptr, 40)); }

TEST(Agreement, FewPointsAreSkipped)
{
  auto ranges = room_scan(ROBOT_X, ROBOT_Y, ROBOT_YAW);
  std::fill(ranges.begin() + 30, ranges.end(), INF);  // 30 finite beams < min_points 40
  EXPECT_FALSE(score(ROBOT_X, ROBOT_Y, ROBOT_YAW, ranges));
}

TEST(Agreement, PointsOutsideTheMapOrInUnknownSpaceAreNotEvaluated)
{
  const auto prepared = mg::prepare_map(room_grid(), 65, 1);
  mg::Points far;
  for (int i = 0; i < 30; ++i) {
    far.push_back({100.0, 100.0});
    far.push_back({4.5, 4.5});
  }
  EXPECT_FALSE(mg::agreement(far, &prepared, 40));
}

TEST(Agreement, RotatedMapOrigin)
{
  mg::GridMap rotated = room_grid();
  rotated.origin_x = 5.0;
  rotated.origin_y = -5.0;
  rotated.origin_yaw = M_PI / 2;
  const auto prepared = mg::prepare_map(rotated, 65, 1);
  const auto ranges = room_scan(ROBOT_X, ROBOT_Y, ROBOT_YAW);
  const auto points = mg::transform_points(
    mg::scan_endpoints(ranges, -M_PI, 2 * M_PI / N_BEAMS, 0.15, 12.0, 8.0), {ROBOT_X, ROBOT_Y, 0.0},
    mg::yaw_quaternion(ROBOT_YAW));
  const auto result = mg::agreement(points, &prepared, 40);
  ASSERT_TRUE(result);
  EXPECT_GT(result->ratio, 0.9);
}

TEST(Agreement, KnownCellsCount)
{
  const auto grid = room_grid();
  long known = 0;  // NOLINT(runtime/int)
  for (auto c : grid.cells) {
    known += c != mg::UNKNOWN;
  }
  EXPECT_EQ(mg::prepare_map(grid, 65, 1).known_cells, known);
}

TEST(Agreement, MedianOfOddAndEvenSamples)
{
  EXPECT_DOUBLE_EQ(mg::median({3.0, 1.0, 2.0}), 2.0);
  EXPECT_DOUBLE_EQ(mg::median({0.2, 0.9}), (0.2 + 0.9) / 2);
}

// --- MismatchMonitor ------------------------------------------------------------------------------------------------

TEST(MismatchMonitor, FreshMapNeverTriggers)
{
  auto mon = monitor();
  EXPECT_FALSE(feed(mon, 0.0, 30.0, 0.1));
}

TEST(MismatchMonitor, SmallMapNeverTriggers)
{
  auto mon = monitor();
  feed(mon, 0.0, 5.0, 0.9, 500);
  EXPECT_FALSE(feed(mon, 5.0, 30.0, 0.1, 500));
}

TEST(MismatchMonitor, MismatchAfterHealthyTriggersAfterMismatchS)
{
  auto mon = monitor();
  feed(mon, 0.0, 5.0, 0.9);
  EXPECT_TRUE(mon.armed());
  EXPECT_FALSE(feed(mon, 5.0, 10.5, 0.1));
  EXPECT_TRUE(feed(mon, 10.5, 11.5, 0.1));
}

TEST(MismatchMonitor, SingleLowSamplesDoNotTrigger)
{
  auto mon = monitor();
  feed(mon, 0.0, 5.0, 0.9);
  for (int i = 0; i < 40; ++i) {
    EXPECT_FALSE(mon.update(5.0 + i * 0.5, i % 3 == 0 ? 0.1 : 0.8, 5000));
  }
}

TEST(MismatchMonitor, ResetDisarms)
{
  auto mon = monitor();
  feed(mon, 0.0, 5.0, 0.9);
  mon.reset();
  EXPECT_FALSE(mon.armed());
  EXPECT_FALSE(mon.median());
  EXPECT_FALSE(feed(mon, 5.0, 30.0, 0.1));
}

TEST(MismatchMonitor, MedianTracksWindow)
{
  for (double ratio : {0.9, 0.45}) {
    auto mon = monitor();
    feed(mon, 0.0, 5.0, ratio);
    ASSERT_TRUE(mon.median());
    EXPECT_NEAR(*mon.median(), ratio, 1e-12);
  }
}

// --- Golden fixture (Python outputs) --------------------------------------------------------------------------------

TEST(AgreementFixture, RoomGridMatchesPython)
{
  const auto f = fx::load_fixture("agreement.json");
  const auto python_room = fx::grid(f["room"]);
  EXPECT_EQ(room_grid().cells, python_room.cells);
  EXPECT_EQ(mg::prepare_map(python_room, 65, 1).known_cells, f["room_known_cells"].get<long>());  // NOLINT
}

TEST(AgreementFixture, DilateMatchesPython)
{
  const auto f = fx::load_fixture("agreement.json");
  for (const auto & c : f["dilate"]) {
    int width = 0;
    int height = 0;
    const auto in = fx::mask(c["mask"], width, height);
    const auto expected = fx::mask(c["out"], width, height);
    EXPECT_EQ(mg::dilate(in, width, height, c["radius"].get<int>()), expected) << c["name"];
  }
}

TEST(AgreementFixture, ScanEndpointsMatchPython)
{
  const auto f = fx::load_fixture("agreement.json");
  for (const auto & c : f["scan_endpoints"]) {
    const auto out = mg::scan_endpoints(
      fx::nums(c["ranges"]), c["angle_min"].get<double>(), c["angle_increment"].get<double>(),
      c["range_min"].get<double>(), c["range_max"].get<double>(), c["max_range_m"].get<double>());
    const auto expected = fx::points(c["points"]);
    ASSERT_EQ(out.size(), expected.size()) << c["name"];
    for (std::size_t i = 0; i < out.size(); ++i) {
      EXPECT_NEAR(out[i].x, expected[i].x, 1e-12) << c["name"];
      EXPECT_NEAR(out[i].y, expected[i].y, 1e-12) << c["name"];
    }
  }
}

TEST(AgreementFixture, TransformAndYawQuaternionMatchPython)
{
  const auto f = fx::load_fixture("agreement.json");
  for (const auto & c : f["transform"]) {
    const auto t = c["translation"];
    const auto r = c["rotation"];
    const auto out = mg::transform_points(
      fx::points(c["points"]), {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()},
      {r[0].get<double>(), r[1].get<double>(), r[2].get<double>(), r[3].get<double>()});
    const auto expected = fx::points(c["out"]);
    ASSERT_EQ(out.size(), expected.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
      EXPECT_NEAR(out[i].x, expected[i].x, 1e-12) << c["name"];
      EXPECT_NEAR(out[i].y, expected[i].y, 1e-12) << c["name"];
    }
  }
  for (const auto & c : f["yaw_quaternion"]) {
    const auto q = mg::yaw_quaternion(c["yaw"].get<double>());
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(q[i], c["q"][i].get<double>(), 1e-15);
    }
  }
}

TEST(AgreementFixture, AgreementRatiosMatchPython)
{
  const auto f = fx::load_fixture("agreement.json");
  for (const auto & c : f["scores"]) {
    const std::string name = c["name"];
    const auto & s = c["scan"];
    const auto endpoints = mg::scan_endpoints(
      fx::nums(s["ranges"]), s["angle_min"].get<double>(), s["angle_increment"].get<double>(),
      s["range_min"].get<double>(), s["range_max"].get<double>(), s["max_range_m"].get<double>());
    const auto t = c["translation"];
    const auto r = c["rotation"];
    const auto in_map = mg::transform_points(
      endpoints, {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()},
      {r[0].get<double>(), r[1].get<double>(), r[2].get<double>(), r[3].get<double>()});
    const auto python_points = fx::points(c["points_map"]);
    ASSERT_EQ(in_map.size(), python_points.size()) << name;
    for (std::size_t i = 0; i < in_map.size(); ++i) {
      ASSERT_NEAR(in_map[i].x, python_points[i].x, 1e-12) << name;
      ASSERT_NEAR(in_map[i].y, python_points[i].y, 1e-12) << name;
    }
    const auto prepared =
      mg::prepare_map(fx::grid(c["grid"]), c["occupied_threshold"].get<int>(), c["hit_radius_cells"].get<int>());
    EXPECT_EQ(prepared.known_cells, c["known_cells"].get<long>()) << name;  // NOLINT(runtime/int)
    const int min_points = c["min_points"].get<int>();
    // Scored on the Python points the counts are exact; on our own points they agree too.
    for (const auto * pts : {&python_points, &in_map}) {
      const auto result = mg::agreement(*pts, &prepared, min_points);
      if (c["result"].is_null()) {
        EXPECT_FALSE(result) << name;
        continue;
      }
      ASSERT_TRUE(result) << name;
      EXPECT_EQ(result->evaluated, c["result"]["evaluated"].get<long>()) << name;  // NOLINT(runtime/int)
      EXPECT_EQ(result->agreeing, c["result"]["agreeing"].get<long>()) << name;    // NOLINT(runtime/int)
      EXPECT_DOUBLE_EQ(result->ratio, c["result"]["ratio"].get<double>()) << name;
    }
  }
  const auto prepared_room = mg::prepare_map(fx::grid(f["room"]), 65, 1);
  EXPECT_TRUE(f["outside_points"]["result"].is_null());
  EXPECT_FALSE(mg::agreement(fx::points(f["outside_points"]["points_map"]), &prepared_room, 40));
}

TEST(AgreementFixture, MismatchMonitorMatchesPython)
{
  const auto f = fx::load_fixture("agreement.json");
  for (const auto & c : f["monitor"]) {
    auto mon = monitor();
    const auto & updates = c["updates"];
    for (std::size_t i = 0; i < updates.size(); ++i) {
      if (updates[i].is_string()) {
        mon.reset();
        continue;
      }
      EXPECT_EQ(
        mon.update(updates[i][0].get<double>(), updates[i][1].get<double>(), updates[i][2].get<long>()),  // NOLINT
        c["fired"][i].get<bool>())
        << c["name"] << " update " << i;
    }
    EXPECT_EQ(mon.armed(), c["armed"].get<bool>()) << c["name"];
    if (c["median"].is_null()) {
      EXPECT_FALSE(mon.median()) << c["name"];
    } else {
      ASSERT_TRUE(mon.median()) << c["name"];
      EXPECT_DOUBLE_EQ(*mon.median(), c["median"].get<double>()) << c["name"];
    }
  }
}
