#include "map_guard_cpp/backup.hpp"

#include <algorithm>
#include <ctime>
#include <map>
#include <regex>
#include <string>
#include <system_error>
#include <vector>

namespace map_guard_cpp
{

namespace
{

constexpr const char * TIMESTAMP_FORMAT = "%Y%m%dT%H%M%SZ";
constexpr const char * TIMESTAMP_PATTERN = R"(\d{8}T\d{6}Z)";

std::string regex_escape(const std::string & text)
{
  static const std::regex special(R"([.^$|()\[\]{}*+?\\])");
  return std::regex_replace(text, special, R"(\$&)");
}

}  // namespace

std::string backup_name(const std::string & prefix, std::chrono::system_clock::time_point when)
{
  const std::time_t seconds =
    static_cast<std::time_t>(std::chrono::floor<std::chrono::seconds>(when.time_since_epoch()).count());
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), TIMESTAMP_FORMAT, &utc);
  return prefix + "_" + stamp;
}

std::vector<std::filesystem::path> prune_backups(
  const std::filesystem::path & directory, const std::string & prefix, long keep)  // NOLINT(runtime/int)
{
  std::error_code ec;
  if (!std::filesystem::is_directory(directory, ec)) {
    return {};
  }
  const std::regex pattern("^" + regex_escape(prefix) + "_" + TIMESTAMP_PATTERN + "$");
  std::map<std::string, std::vector<std::filesystem::path>> by_stem;
  for (const auto & entry : std::filesystem::directory_iterator(directory, ec)) {
    const std::string name = entry.path().filename().string();
    const std::string stem = name.substr(0, name.find('.'));
    if (entry.is_regular_file(ec) && std::regex_match(stem, pattern)) {
      by_stem[stem].push_back(entry.path());
    }
  }
  std::vector<std::filesystem::path> deleted;
  if (static_cast<long>(by_stem.size()) <= keep) {  // NOLINT(runtime/int)
    return deleted;
  }
  std::size_t to_delete = by_stem.size() - static_cast<std::size_t>(keep);
  for (auto & [stem, paths] : by_stem) {
    if (to_delete-- == 0) {
      break;
    }
    std::sort(paths.begin(), paths.end());
    for (const auto & path : paths) {
      std::filesystem::remove(path, ec);
      deleted.push_back(path);
    }
  }
  return deleted;
}

}  // namespace map_guard_cpp
