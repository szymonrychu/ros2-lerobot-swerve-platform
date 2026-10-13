#include "ros2_common_cpp/config.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ros2_common_cpp
{

void forbid_unknown(
  const YAML::Node & node, const std::vector<std::string> & allowed, Errors & errors, const std::string & where)
{
  const std::string prefix = where.empty() ? "" : where + ".";
  if (!node.IsMap()) {
    errors.push_back((where.empty() ? std::string("config root") : where) + ": expected mapping");
    return;
  }
  for (const auto & entry : node) {
    const std::string key = entry.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      errors.push_back(prefix + key + ": extra field not permitted");
    }
  }
}

std::string config_path_from_env(const std::string & env_var, const std::string & default_path)
{
  const char * value = std::getenv(env_var.c_str());
  return (value != nullptr && value[0] != '\0') ? std::string(value) : default_path;
}

LoadResult load_config(const std::string & path, const Validator & validate)
{
  LoadResult result{path, YAML::Node(YAML::NodeType::Map), {}};
  if (!std::ifstream(path).good()) {
    result.errors.push_back("cannot read config file " + path);
    return result;
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    result.errors.push_back(std::string("invalid YAML: ") + e.what());
    return result;
  }
  if (root.IsNull()) {
    root = YAML::Node(YAML::NodeType::Map);
  }
  if (!root.IsMap()) {
    result.errors.push_back("config root must be a mapping");
    return result;
  }
  result.node = root;
  result.errors = validate(root);
  return result;
}

YAML::Node load_config_or_exit(
  const std::string & env_var, const std::string & default_path, const Validator & validate)
{
  const LoadResult result = load_config(config_path_from_env(env_var, default_path), validate);
  if (result.errors.empty()) {
    return result.node;
  }
  for (const auto & error : result.errors) {
    std::cerr << "config error: " << result.path << ": " << error << "\n";
  }
  std::cerr.flush();
  std::exit(EXIT_CONFIG);
}

}  // namespace ros2_common_cpp
