// Port of nodes/map_guard/tests/test_backup.py (backup naming and pruning) plus the backup.json golden fixture.
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "fixture_util.hpp"
#include "map_guard_cpp/backup.hpp"

namespace mg = map_guard_cpp;
namespace fx = map_guard_cpp::test;
namespace fs = std::filesystem;
using std::chrono::system_clock;

namespace
{

const std::string PREFIX = "map_guard_backup";
// 2026-01-01T00:00:00Z
const auto BASE = system_clock::time_point(std::chrono::seconds(1767225600));

class Backup : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char tmpl[] = "/tmp/map_guard_cpp_backup_XXXXXX";
    dir = mkdtemp(tmpl);
  }
  void TearDown() override { fs::remove_all(dir); }

  void touch(const std::string & name) { std::ofstream(dir / name) << "x"; }

  // Create count serialized maps (.posegraph + .data), oldest first; returns their stems.
  std::vector<std::string> make_backups(int count)
  {
    std::vector<std::string> stems;
    for (int i = 0; i < count; ++i) {
      const std::string stem = mg::backup_name(PREFIX, BASE + std::chrono::minutes(i));
      touch(stem + ".posegraph");
      touch(stem + ".data");
      stems.push_back(stem);
    }
    return stems;
  }

  std::vector<std::string> listing() const
  {
    std::vector<std::string> out;
    for (const auto & entry : fs::directory_iterator(dir)) {
      out.push_back(entry.path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
  }

  fs::path dir;
};

std::vector<std::string> names(const std::vector<fs::path> & paths)
{
  std::vector<std::string> out;
  for (const auto & p : paths) {
    out.push_back(p.filename().string());
  }
  std::sort(out.begin(), out.end());
  return out;
}

system_clock::time_point from_epoch(double seconds)
{
  return system_clock::time_point(
    std::chrono::duration_cast<system_clock::duration>(std::chrono::duration<double>(seconds)));
}

}  // namespace

TEST(BackupName, IsASortableUtcTimestamp)
{
  // 2026-10-10T07:05:03Z
  EXPECT_EQ(mg::backup_name(PREFIX, from_epoch(1791615903.0)), "map_guard_backup_20261010T070503Z");
}

TEST(BackupName, ConvertsToUtc)
{
  // 2026-10-10T09:00:00+02:00 is 07:00:00Z whatever the local time zone is.
  setenv("TZ", "Europe/Warsaw", 1);
  tzset();
  EXPECT_EQ(mg::backup_name(PREFIX, from_epoch(1791615600.0)), "map_guard_backup_20261010T070000Z");
  unsetenv("TZ");
  tzset();
}

TEST_F(Backup, PruneKeepsTheNewest)
{
  const auto stems = make_backups(8);
  touch("slam_map.posegraph");  // the main map is never touched
  touch("slam_map.data");
  const auto deleted = names(mg::prune_backups(dir, PREFIX, 5));
  std::vector<std::string> expected;
  for (int i = 0; i < 3; ++i) {
    expected.push_back(stems[i] + ".posegraph");
    expected.push_back(stems[i] + ".data");
  }
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(deleted, expected);
  std::vector<std::string> remaining = {"slam_map.data", "slam_map.posegraph"};
  for (int i = 3; i < 8; ++i) {
    remaining.push_back(stems[i] + ".posegraph");
    remaining.push_back(stems[i] + ".data");
  }
  std::sort(remaining.begin(), remaining.end());
  EXPECT_EQ(listing(), remaining);
}

TEST_F(Backup, PruneWithFewerBackupsThanKeep)
{
  make_backups(2);
  EXPECT_TRUE(mg::prune_backups(dir, PREFIX, 5).empty());
  EXPECT_EQ(listing().size(), 4u);
}

TEST_F(Backup, PruneMissingDirectory) { EXPECT_TRUE(mg::prune_backups(dir / "nope", PREFIX, 5).empty()); }

TEST_F(Backup, PruneIgnoresOtherPrefixes)
{
  make_backups(6);
  touch("map_guard_backup_old_manual.txt");  // not a timestamped backup
  EXPECT_EQ(mg::prune_backups(dir, PREFIX, 5).size(), 2u);
  EXPECT_TRUE(fs::exists(dir / "map_guard_backup_old_manual.txt"));
}

TEST_F(Backup, PruneReturnsFilesSortedByStemThenName)
{
  make_backups(3);
  const auto deleted = mg::prune_backups(dir, PREFIX, 1);
  ASSERT_EQ(deleted.size(), 4u);
  EXPECT_TRUE(std::is_sorted(deleted.begin(), deleted.end()));
}

TEST_F(Backup, FixtureMatchesPython)
{
  const auto f = fx::load_fixture("backup.json");
  for (const auto & c : f["names"]) {
    EXPECT_EQ(mg::backup_name(c["prefix"], from_epoch(c["epoch_s"].get<double>())), c["name"].get<std::string>());
  }
  for (const auto & c : f["prune"]) {
    fs::remove_all(dir);
    fs::create_directories(dir);
    for (const auto & file : c["files"]) {
      touch(file.get<std::string>());
    }
    EXPECT_EQ(
      names(mg::prune_backups(dir, PREFIX, c["keep"].get<long>())),  // NOLINT(runtime/int)
      c["deleted"].get<std::vector<std::string>>())
      << c["name"];
    EXPECT_EQ(listing(), c["remaining"].get<std::vector<std::string>>()) << c["name"];
  }
}
