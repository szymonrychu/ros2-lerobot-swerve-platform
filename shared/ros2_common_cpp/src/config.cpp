#include "ros2_common_cpp/config.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace ros2_common_cpp
{

namespace
{

// One YAML value as Python sees it after yaml.safe_load (YAML 1.1 resolver of PyYAML).
struct Value
{
  enum class Kind
  {
    NONE,
    BOOL,
    INT,
    FLOAT,
    STR,
    LIST,
    MAP
  };
  Kind kind = Kind::NONE;
  bool b = false;
  long i = 0;  // NOLINT(runtime/int)
  double f = 0.0;
  std::string s;
};

// PyYAML's implicit resolvers (yaml/resolver.py) for plain scalars.
const std::regex BOOL_RE("^(?:yes|Yes|YES|no|No|NO|true|True|TRUE|false|False|FALSE|on|On|ON|off|Off|OFF)$");
const std::regex INT_RE(
  "^(?:[-+]?0b[0-1_]+|[-+]?0[0-7_]+|[-+]?(?:0|[1-9][0-9_]*)|[-+]?0x[0-9a-fA-F_]+|[-+]?[1-9][0-9_]*(?::[0-5]?[0-9])+)$");
const std::regex FLOAT_RE(
  "^(?:[-+]?(?:[0-9][0-9_]*)\\.[0-9_]*(?:[eE][-+][0-9]+)?|\\.[0-9][0-9_]*(?:[eE][-+][0-9]+)?"
  "|[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\\.[0-9_]*|[-+]?\\.(?:inf|Inf|INF)|\\.(?:nan|NaN|NAN))$");
const std::regex NULL_RE("^(?:~|null|Null|NULL|)$");

std::string strip_underscores(const std::string & text)
{
  std::string out;
  std::copy_if(text.begin(), text.end(), std::back_inserter(out), [](char c) { return c != '_'; });
  return out;
}

// Value of a PyYAML int literal (binary, octal, decimal, hex, sexagesimal).
long yaml_int(const std::string & raw)  // NOLINT(runtime/int)
{
  std::string text = strip_underscores(raw);
  long sign = 1;  // NOLINT(runtime/int)
  if (!text.empty() && (text[0] == '-' || text[0] == '+')) {
    sign = text[0] == '-' ? -1 : 1;
    text = text.substr(1);
  }
  if (text == "0") {
    return 0;
  }
  if (text.rfind("0b", 0) == 0) {
    return sign * std::strtol(text.c_str() + 2, nullptr, 2);
  }
  if (text.rfind("0x", 0) == 0) {
    return sign * std::strtol(text.c_str() + 2, nullptr, 16);
  }
  if (text[0] == '0') {
    return sign * std::strtol(text.c_str(), nullptr, 8);
  }
  if (text.find(':') != std::string::npos) {
    long value = 0;  // NOLINT(runtime/int)
    std::stringstream parts(text);
    std::string part;
    while (std::getline(parts, part, ':')) {
      value = value * 60 + std::strtol(part.c_str(), nullptr, 10);
    }
    return sign * value;
  }
  return sign * std::strtol(text.c_str(), nullptr, 10);
}

// Value of a PyYAML float literal.
double yaml_float(const std::string & raw)
{
  std::string text = strip_underscores(raw);
  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  if (lower == ".nan") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (lower == ".inf" || lower == "+.inf") {
    return std::numeric_limits<double>::infinity();
  }
  if (lower == "-.inf") {
    return -std::numeric_limits<double>::infinity();
  }
  if (text.find(':') != std::string::npos) {
    double sign = 1.0;
    if (text[0] == '-' || text[0] == '+') {
      sign = text[0] == '-' ? -1.0 : 1.0;
      text = text.substr(1);
    }
    double value = 0.0;
    std::stringstream parts(text);
    std::string part;
    while (std::getline(parts, part, ':')) {
      value = value * 60 + std::strtod(part.c_str(), nullptr);
    }
    return sign * value;
  }
  return std::strtod(text.c_str(), nullptr);
}

Value classify(const YAML::Node & node)
{
  Value v;
  if (node.IsSequence()) {
    v.kind = Value::Kind::LIST;
    return v;
  }
  if (node.IsMap()) {
    v.kind = Value::Kind::MAP;
    return v;
  }
  if (node.IsNull()) {
    return v;
  }
  const std::string text = node.Scalar();
  const std::string tag = node.Tag();
  v.s = text;
  if (tag == "!" || tag == "tag:yaml.org,2002:str") {  // quoted, or explicitly a string
    v.kind = Value::Kind::STR;
  } else if (std::regex_match(text, BOOL_RE)) {
    v.kind = Value::Kind::BOOL;
    const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(text[0])));
    v.b = c == 'y' || c == 't' || (c == 'o' && text.size() == 2);
  } else if (std::regex_match(text, INT_RE)) {
    v.kind = Value::Kind::INT;
    v.i = yaml_int(text);
  } else if (std::regex_match(text, FLOAT_RE)) {
    v.kind = Value::Kind::FLOAT;
    v.f = yaml_float(text);
  } else if (std::regex_match(text, NULL_RE)) {
    v.kind = Value::Kind::NONE;
  } else {
    v.kind = Value::Kind::STR;
  }
  return v;
}

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n\f\v");
  if (first == std::string::npos) {
    return "";
  }
  return text.substr(first, text.find_last_not_of(" \t\r\n\f\v") - first + 1);
}

// pydantic's float parsing of a str: surrounding whitespace and digit-separating underscores allowed, inf/nan words.
std::optional<double> parse_float_str(const std::string & raw)
{
  const std::string text = strip_underscores(trim(raw));
  if (text.empty()) {
    return std::nullopt;
  }
  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  if (lower.find("0x") != std::string::npos) {
    return std::nullopt;
  }
  char * end = nullptr;
  errno = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0') {
    return std::nullopt;
  }
  return value;
}

// pydantic lax mode: a float field takes bool, int, float and numeric strings.
std::optional<double> as_float(const Value & v)
{
  switch (v.kind) {
    case Value::Kind::BOOL:
      return v.b ? 1.0 : 0.0;
    case Value::Kind::INT:
      return static_cast<double>(v.i);
    case Value::Kind::FLOAT:
      return v.f;
    case Value::Kind::STR:
      return parse_float_str(v.s);
    default:
      return std::nullopt;
  }
}

// pydantic lax mode: an int field takes bool, int, a finite integral float and integer (or integral float) strings.
std::optional<long> as_int(const Value & v)  // NOLINT(runtime/int)
{
  auto from_float = [](double f) -> std::optional<long> {  // NOLINT(runtime/int)
    if (!std::isfinite(f) || f != std::floor(f)) {
      return std::nullopt;
    }
    return static_cast<long>(f);  // NOLINT(runtime/int)
  };
  switch (v.kind) {
    case Value::Kind::BOOL:
      return v.b ? 1 : 0;
    case Value::Kind::INT:
      return v.i;
    case Value::Kind::FLOAT:
      return from_float(v.f);
    case Value::Kind::STR: {
      const std::string text = strip_underscores(trim(v.s));
      static const std::regex integer("^[-+]?[0-9]+$");
      if (std::regex_match(text, integer)) {
        return std::strtol(text.c_str(), nullptr, 10);
      }
      const auto f = parse_float_str(v.s);
      return f ? from_float(*f) : std::nullopt;
    }
    default:
      return std::nullopt;
  }
}

// pydantic lax mode: a bool field takes bool, 0/1 numbers and the words of pydantic's bool parser.
std::optional<bool> as_bool(const Value & v)
{
  switch (v.kind) {
    case Value::Kind::BOOL:
      return v.b;
    case Value::Kind::INT:
      if (v.i == 0 || v.i == 1) {
        return v.i == 1;
      }
      return std::nullopt;
    case Value::Kind::FLOAT:
      if (v.f == 0.0 || v.f == 1.0) {
        return v.f == 1.0;
      }
      return std::nullopt;
    case Value::Kind::STR: {
      std::string lower = v.s;
      std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
      for (const char * word : {"1", "on", "t", "true", "y", "yes"}) {
        if (lower == word) {
          return true;
        }
      }
      for (const char * word : {"0", "off", "f", "false", "n", "no"}) {
        if (lower == word) {
          return false;
        }
      }
      return std::nullopt;
    }
    default:
      return std::nullopt;
  }
}

}  // namespace

bool is_yaml_null(const YAML::Node & node) { return classify(node).kind == Value::Kind::NONE; }

std::optional<double> lax_float(const YAML::Node & node) { return as_float(classify(node)); }

std::optional<long> lax_int(const YAML::Node & node)  // NOLINT(runtime/int)
{
  return as_int(classify(node));
}

std::optional<bool> lax_bool(const YAML::Node & node) { return as_bool(classify(node)); }

std::optional<std::string> lax_string(const YAML::Node & node)
{
  const Value v = classify(node);
  if (v.kind != Value::Kind::STR) {
    return std::nullopt;
  }
  return v.s;
}

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
