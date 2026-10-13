// Golden fixture access for the map_guard_cpp gtests: test/fixtures/<name>.json written by test/gen_fixtures.py from
// the Python map_guard functions. MAP_GUARD_FIXTURE_DIR is set by CMakeLists.txt.
#ifndef MAP_GUARD_CPP__TEST__FIXTURE_UTIL_HPP_
#define MAP_GUARD_CPP__TEST__FIXTURE_UTIL_HPP_

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "map_guard_cpp/agreement.hpp"

namespace map_guard_cpp::test
{

// Parsed test/fixtures/<name>.
inline nlohmann::json load_fixture(const std::string & name)
{
  const std::string path = std::string(MAP_GUARD_FIXTURE_DIR) + "/" + name;
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("missing fixture " + path + " (run test/gen_fixtures.py)");
  }
  return nlohmann::json::parse(in);
}

// A fixture number: JSON number, or "inf" / "-inf" / "nan".
inline double num(const nlohmann::json & value)
{
  if (value.is_string()) {
    const std::string s = value.get<std::string>();
    if (s == "inf") {
      return std::numeric_limits<double>::infinity();
    }
    if (s == "-inf") {
      return -std::numeric_limits<double>::infinity();
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
  return value.get<double>();
}

inline std::vector<double> nums(const nlohmann::json & values)
{
  std::vector<double> out;
  for (const auto & v : values) {
    out.push_back(num(v));
  }
  return out;
}

inline Points points(const nlohmann::json & values)
{
  Points out;
  for (const auto & p : values) {
    out.push_back({p[0].get<double>(), p[1].get<double>()});
  }
  return out;
}

// A grid_json() object of gen_fixtures.py.
inline GridMap grid(const nlohmann::json & g)
{
  GridMap out;
  out.resolution = g["resolution"].get<double>();
  out.origin_x = g["origin_x"].get<double>();
  out.origin_y = g["origin_y"].get<double>();
  out.origin_yaw = g["origin_yaw"].get<double>();
  out.width = g["width"].get<int>();
  out.height = g["height"].get<int>();
  for (const auto & run : g["cells_rle"]) {
    out.cells.insert(out.cells.end(), run[1].get<std::size_t>(), static_cast<std::int8_t>(run[0].get<int>()));
  }
  return out;
}

// Rows of "0"/"1" as a row-major mask.
inline Mask mask(const nlohmann::json & rows, int & width, int & height)
{
  Mask out;
  height = static_cast<int>(rows.size());
  width = 0;
  for (const auto & row : rows) {
    const std::string s = row.get<std::string>();
    width = static_cast<int>(s.size());
    for (char c : s) {
      out.push_back(c == '1' ? 1 : 0);
    }
  }
  return out;
}

}  // namespace map_guard_cpp::test

#endif  // MAP_GUARD_CPP__TEST__FIXTURE_UTIL_HPP_
