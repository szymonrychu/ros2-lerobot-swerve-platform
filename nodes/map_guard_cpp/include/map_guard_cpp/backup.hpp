// Names and pruning of the serialized map backups (<prefix>_<UTC timestamp>.posegraph / .data).
// Port of nodes/map_guard/map_guard/backup.py.
#ifndef MAP_GUARD_CPP__BACKUP_HPP_
#define MAP_GUARD_CPP__BACKUP_HPP_

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

namespace map_guard_cpp
{

// File stem of a backup taken at when: "<prefix>_<YYYYMMDDTHHMMSSZ>" in UTC, so names sort by time
// (slam_toolbox appends .posegraph and .data).
std::string backup_name(const std::string & prefix, std::chrono::system_clock::time_point when);

// Delete all but the newest keep backups (every file of an older stem); returns the deleted files, sorted by stem and
// then by name. A missing directory deletes nothing.
std::vector<std::filesystem::path> prune_backups(
  const std::filesystem::path & directory, const std::string & prefix, long keep);  // NOLINT(runtime/int)

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__BACKUP_HPP_
