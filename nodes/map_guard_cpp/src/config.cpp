#include "map_guard_cpp/config.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

namespace map_guard_cpp
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

std::string number_text(double value)
{
  std::ostringstream out;
  out << value;
  return out.str();
}

// Reads the fields of one mapping, recording every error.
class Reader
{
public:
  Reader(const YAML::Node & node, ros2_common_cpp::Errors & errors) : node_(node), errors_(errors) {}

  void boolean(const std::string & key, bool & field)
  {
    if (const auto v = value(key)) {
      if (const auto b = as_bool(*v)) {
        field = *b;
      } else {
        errors_.push_back(key + ": expected bool");
      }
    }
  }

  void string(const std::string & key, std::string & field)
  {
    if (const auto v = value(key)) {
      if (v->kind == Value::Kind::STR) {
        field = v->s;
      } else {
        errors_.push_back(key + ": expected string");
      }
    }
  }

  // A float field; gt / ge / le bounds as in the pydantic model (NaN fails every bound).
  void number(
    const std::string & key, double & field, std::optional<double> gt = std::nullopt,
    std::optional<double> ge = std::nullopt, std::optional<double> le = std::nullopt)
  {
    const auto v = value(key);
    if (!v) {
      return;
    }
    const auto f = as_float(*v);
    if (!f) {
      errors_.push_back(key + ": expected float");
      return;
    }
    if ((gt && !(*f > *gt)) || (ge && !(*f >= *ge)) || (le && !(*f <= *le))) {
      errors_.push_back(key + ": " + bound_text(gt, ge, le) + " (got " + number_text(*f) + ")");
      return;
    }
    field = *f;
  }

  void positive(const std::string & key, double & field) { number(key, field, 0.0); }

  void integer(
    const std::string & key, long & field,                                         // NOLINT(runtime/int)
    std::optional<long> ge = std::nullopt, std::optional<long> le = std::nullopt)  // NOLINT(runtime/int)
  {
    const auto v = value(key);
    if (!v) {
      return;
    }
    const auto i = as_int(*v);
    if (!i) {
      errors_.push_back(key + ": expected int");
      return;
    }
    if ((ge && *i < *ge) || (le && *i > *le)) {
      errors_.push_back(
        key + ": " +
        bound_text(
          std::nullopt, ge ? std::optional<double>(static_cast<double>(*ge)) : std::nullopt,
          le ? std::optional<double>(static_cast<double>(*le)) : std::nullopt) +
        " (got " + std::to_string(*i) + ")");
      return;
    }
    field = *i;
  }

  void optional_integer(const std::string & key, std::optional<long> & field)  // NOLINT(runtime/int)
  {
    const auto v = value(key);
    if (!v) {
      return;
    }
    if (v->kind == Value::Kind::NONE) {
      field.reset();
      return;
    }
    const auto i = as_int(*v);
    if (!i) {
      errors_.push_back(key + ": expected int or null");
      return;
    }
    field = *i;
  }

  void literals(const std::string & key, std::vector<std::string> & field, const std::vector<std::string> & allowed)
  {
    const YAML::Node node = node_[key];
    if (!node.IsDefined()) {
      return;
    }
    if (!node.IsSequence()) {
      errors_.push_back(key + ": expected list");
      return;
    }
    std::vector<std::string> out;
    bool ok = true;
    for (const auto & item : node) {
      const Value v = classify(item);
      if (v.kind != Value::Kind::STR || std::find(allowed.begin(), allowed.end(), v.s) == allowed.end()) {
        std::string choices;
        for (const auto & a : allowed) {
          choices += (choices.empty() ? "" : ", ") + a;
        }
        errors_.push_back(
          key + ": " + (item.IsScalar() ? "'" + item.Scalar() + "'" : "item") + " is not one of " + choices);
        ok = false;
        continue;
      }
      out.push_back(v.s);
    }
    if (ok) {
      field = out;
    }
  }

private:
  std::optional<Value> value(const std::string & key)
  {
    const YAML::Node node = node_[key];
    if (!node.IsDefined()) {
      return std::nullopt;
    }
    return classify(node);
  }

  static std::string bound_text(std::optional<double> gt, std::optional<double> ge, std::optional<double> le)
  {
    std::string out;
    if (gt) {
      out += "must be > " + number_text(*gt);
    }
    if (ge) {
      out += std::string(out.empty() ? "" : " and ") + "must be >= " + number_text(*ge);
    }
    if (le) {
      out += std::string(out.empty() ? "" : " and ") + "must be <= " + number_text(*le);
    }
    return out;
  }

  const YAML::Node & node_;
  ros2_common_cpp::Errors & errors_;
};

}  // namespace

const std::vector<std::string> & all_reasons()
{
  static const std::vector<std::string> reasons = {"mismatch", "lifted", "carried", "rotated"};
  return reasons;
}

const std::vector<std::string> & config_keys()
{
  static const std::vector<std::string> keys = {
    "dry_run",
    "reasons",
    "scan_topic",
    "map_topic",
    "imu_topic",
    "wheel_odom_topic",
    "lidar_odom_topic",
    "cmd_vel_topic",
    "map_frame",
    "reset_topic",
    "events_topic",
    "pause_service",
    "reset_service",
    "serialize_service",
    "parameters_service",
    "service_timeout_s",
    "agent_state_url",
    "agent_poll_s",
    "agent_timeout_s",
    "metrics_port",
    "tick_hz",
    "max_range_m",
    "hit_radius_cells",
    "occupied_threshold",
    "min_points",
    "agreement_hz",
    "median_window",
    "mismatch_ratio",
    "mismatch_s",
    "healthy_ratio",
    "min_map_cells_known",
    "lift_tilt_deg",
    "lift_s",
    "level_window_s",
    "level_tolerance_deg",
    "level_rebaseline_s",
    "lift_accel_z",
    "lift_accel_s",
    "lift_confirm_s",
    "lift_confirm_tilt_deg",
    "gravity_band",
    "carried_speed_mps",
    "wheel_still_mps",
    "cmd_timeout_s",
    "cmd_eps",
    "carried_s",
    "input_timeout_s",
    "rotate_rate_rad_s",
    "wheel_still_rad_s",
    "rotate_s",
    "settle_s",
    "evaluate_s",
    "cooldown_s",
    "backup_dir",
    "backup_prefix",
    "keep_backups"};
  return keys;
}

Config parse_config(const YAML::Node & node, ros2_common_cpp::Errors & errors)
{
  Config c;
  ros2_common_cpp::forbid_unknown(node, config_keys(), errors);
  if (!node.IsMap()) {
    return c;
  }
  Reader r(node, errors);
  r.boolean("dry_run", c.dry_run);
  r.literals("reasons", c.reasons, all_reasons());

  for (auto [key, field] : std::initializer_list<std::pair<const char *, std::string *>>{
         {"scan_topic", &c.scan_topic},
         {"map_topic", &c.map_topic},
         {"imu_topic", &c.imu_topic},
         {"wheel_odom_topic", &c.wheel_odom_topic},
         {"lidar_odom_topic", &c.lidar_odom_topic},
         {"cmd_vel_topic", &c.cmd_vel_topic},
         {"map_frame", &c.map_frame},
         {"reset_topic", &c.reset_topic},
         {"events_topic", &c.events_topic},
         {"pause_service", &c.pause_service},
         {"reset_service", &c.reset_service},
         {"serialize_service", &c.serialize_service},
         {"parameters_service", &c.parameters_service},
         {"agent_state_url", &c.agent_state_url},
         {"backup_dir", &c.backup_dir},
         {"backup_prefix", &c.backup_prefix}}) {
    r.string(key, *field);
  }

  for (auto [key, field] : std::initializer_list<std::pair<const char *, double *>>{
         {"service_timeout_s", &c.service_timeout_s},
         {"agent_poll_s", &c.agent_poll_s},
         {"agent_timeout_s", &c.agent_timeout_s},
         {"tick_hz", &c.tick_hz},
         {"max_range_m", &c.max_range_m},
         {"agreement_hz", &c.agreement_hz},
         {"mismatch_s", &c.mismatch_s},
         {"lift_tilt_deg", &c.lift_tilt_deg},
         {"lift_s", &c.lift_s},
         {"level_window_s", &c.level_window_s},
         {"level_tolerance_deg", &c.level_tolerance_deg},
         {"level_rebaseline_s", &c.level_rebaseline_s},
         {"lift_accel_z", &c.lift_accel_z},
         {"lift_accel_s", &c.lift_accel_s},
         {"lift_confirm_s", &c.lift_confirm_s},
         {"lift_confirm_tilt_deg", &c.lift_confirm_tilt_deg},
         {"gravity_band", &c.gravity_band},
         {"carried_speed_mps", &c.carried_speed_mps},
         {"wheel_still_mps", &c.wheel_still_mps},
         {"cmd_timeout_s", &c.cmd_timeout_s},
         {"cmd_eps", &c.cmd_eps},
         {"carried_s", &c.carried_s},
         {"input_timeout_s", &c.input_timeout_s},
         {"rotate_rate_rad_s", &c.rotate_rate_rad_s},
         {"wheel_still_rad_s", &c.wheel_still_rad_s},
         {"rotate_s", &c.rotate_s},
         {"settle_s", &c.settle_s},
         {"evaluate_s", &c.evaluate_s},
         {"cooldown_s", &c.cooldown_s}}) {
    r.positive(key, *field);
  }
  r.number("mismatch_ratio", c.mismatch_ratio, std::nullopt, 0.0, 1.0);
  r.number("healthy_ratio", c.healthy_ratio, std::nullopt, 0.0, 1.0);

  r.optional_integer("metrics_port", c.metrics_port);
  r.integer("hit_radius_cells", c.hit_radius_cells, 0);
  r.integer("occupied_threshold", c.occupied_threshold, 1, 100);
  r.integer("min_points", c.min_points, 1);
  r.integer("median_window", c.median_window, 1);
  r.integer("min_map_cells_known", c.min_map_cells_known, 0);
  r.integer("keep_backups", c.keep_backups, 1);

  // The model validator runs only when every field is valid, as in pydantic.
  if (errors.empty() && c.healthy_ratio <= c.mismatch_ratio) {
    errors.push_back(
      "healthy_ratio: (" + number_text(c.healthy_ratio) + ") must be above mismatch_ratio (" +
      number_text(c.mismatch_ratio) + ")");
  }
  return c;
}

ros2_common_cpp::Errors validate_config(const YAML::Node & node)
{
  ros2_common_cpp::Errors errors;
  parse_config(node, errors);
  return errors;
}

Config load_config_or_exit()
{
  const YAML::Node node =
    ros2_common_cpp::load_config_or_exit(ENV_CONFIG_PATH_KEY, DEFAULT_CONFIG_PATH, validate_config);
  ros2_common_cpp::Errors errors;
  return parse_config(node, errors);
}

}  // namespace map_guard_cpp
