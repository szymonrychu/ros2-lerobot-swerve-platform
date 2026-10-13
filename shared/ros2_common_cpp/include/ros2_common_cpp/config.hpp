// YAML config validation for the robot's C++ nodes: the C++ counterpart of the pydantic models of the Python nodes
// (extra="forbid", ge/gt/le, required fields), with the same exit code 78 (EX_CONFIG) on any error.
//
// A node writes one validate function that walks its YAML::Node and returns every error it finds:
//
//   ros2_common_cpp::Errors validate(const YAML::Node & cfg)
//   {
//     ros2_common_cpp::Errors errors;
//     ros2_common_cpp::forbid_unknown(cfg, {"rate_hz", "topic"}, errors);
//     if (auto rate = ros2_common_cpp::require<double>(cfg, "rate_hz", errors)) {
//       ros2_common_cpp::gt("rate_hz", *rate, 0.0, errors);
//     }
//     ros2_common_cpp::get_or<std::string>(cfg, "topic", "/scan", errors);
//     return errors;
//   }
//
// and loads it with load_config_or_exit("MY_NODE_CONFIG", "/etc/ros2/my_node/config.yaml", validate).
#ifndef ROS2_COMMON_CPP__CONFIG_HPP_
#define ROS2_COMMON_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace ros2_common_cpp
{

// Exit code of a process whose config is invalid (sysexits EX_CONFIG); systemd units carry
// RestartPreventExitStatus=78, so a bad config stops the unit instead of restart-looping it.
constexpr int EXIT_CONFIG = 78;

// Every validation error, one human-readable line each ("<key>: <problem>").
using Errors = std::vector<std::string>;

// A node's validator: the whole config in, every error out (empty = valid).
using Validator = std::function<Errors(const YAML::Node &)>;

// Result of load_config: the path read, the parsed root (an empty map for an empty file) and every error.
struct LoadResult
{
  std::string path;
  YAML::Node node;
  Errors errors;
};

// Name of T in error messages ("int", "float", "bool", "string", "list of float", ...).
template <typename T>
std::string type_name()
{
  return "value";
}
template <>
inline std::string type_name<int>()
{
  return "int";
}
template <>
inline std::string type_name<long>()  // NOLINT(runtime/int)
{
  return "int";
}
template <>
inline std::string type_name<unsigned int>()
{
  return "non-negative int";
}
template <>
inline std::string type_name<double>()
{
  return "float";
}
template <>
inline std::string type_name<float>()
{
  return "float";
}
template <>
inline std::string type_name<bool>()
{
  return "bool";
}
template <>
inline std::string type_name<std::string>()
{
  return "string";
}
template <>
inline std::string type_name<std::vector<double>>()
{
  return "list of float";
}
template <>
inline std::string type_name<std::vector<int>>()
{
  return "list of int";
}
template <>
inline std::string type_name<std::vector<std::string>>()
{
  return "list of string";
}

// Convert node[key] to T. Records "<key>: expected <type>" in errors and returns nullopt when it does not convert
// (a null value never converts, a string never converts to bool or a number).
template <typename T>
std::optional<T> convert(const YAML::Node & value, const std::string & key, Errors & errors)
{
  bool ok = !value.IsNull();
  if constexpr (std::is_same_v<T, std::string>) {
    ok = ok && value.IsScalar();
  }
  if (ok) {
    try {
      return value.as<T>();
    } catch (const YAML::Exception &) {
    }
  }
  errors.push_back(key + ": expected " + type_name<T>());
  return std::nullopt;
}

// Required field: "<key>: field required" when missing, "<key>: expected <type>" when it does not convert.
template <typename T>
std::optional<T> require(const YAML::Node & node, const std::string & key, Errors & errors)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    errors.push_back(key + ": field required");
    return std::nullopt;
  }
  return convert<T>(value, key, errors);
}

// Optional field: fallback when missing; "<key>: expected <type>" (and the fallback) when it does not convert.
template <typename T>
T get_or(const YAML::Node & node, const std::string & key, const T & fallback, Errors & errors)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    return fallback;
  }
  return convert<T>(value, key, errors).value_or(fallback);
}

// Reject keys outside allowed (pydantic extra="forbid"): "<key>: extra field not permitted" per unknown key,
// "<where>: expected mapping" when node is not a mapping. where prefixes nested keys ("camera.width").
void forbid_unknown(
  const YAML::Node & node, const std::vector<std::string> & allowed, Errors & errors, const std::string & where = "");

// Range checks, the pydantic ge / gt / le: "<key>: must be >= <bound> (got <value>)".
template <typename T>
void ge(const std::string & key, T value, T bound, Errors & errors)
{
  if (!(value >= bound)) {
    std::ostringstream msg;
    msg << key << ": must be >= " << bound << " (got " << value << ")";
    errors.push_back(msg.str());
  }
}

template <typename T>
void gt(const std::string & key, T value, T bound, Errors & errors)
{
  if (!(value > bound)) {
    std::ostringstream msg;
    msg << key << ": must be > " << bound << " (got " << value << ")";
    errors.push_back(msg.str());
  }
}

template <typename T>
void le(const std::string & key, T value, T bound, Errors & errors)
{
  if (!(value <= bound)) {
    std::ostringstream msg;
    msg << key << ": must be <= " << bound << " (got " << value << ")";
    errors.push_back(msg.str());
  }
}

// The value of env_var when set and non-empty, else default_path.
std::string config_path_from_env(const std::string & env_var, const std::string & default_path);

// Read and validate one YAML file. A missing or unparsable file, a root that is not a mapping, and every validator
// error all land in errors; it never exits.
LoadResult load_config(const std::string & path, const Validator & validate);

// The node entry point: path from env_var (default default_path), load_config, then on any error log every error to
// stderr ("config error: <path>: <error>") and exit EXIT_CONFIG. Returns the validated root on success.
YAML::Node load_config_or_exit(
  const std::string & env_var, const std::string & default_path, const Validator & validate);

}  // namespace ros2_common_cpp

#endif  // ROS2_COMMON_CPP__CONFIG_HPP_
