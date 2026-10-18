#include "map_guard_cpp/agent.hpp"

#include <httplib.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace map_guard_cpp
{

namespace
{

// Disturbances that mean the robot is in someone's hands: the agent must stop driving.
bool stops_driving(const std::string & reason) { return reason == "lifted" || reason == "carried"; }

// Python's "{:.2f}".
std::string two_decimals(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.2f", value);
  return buf;
}

// Python's round(value, 3): correctly rounded through the decimal text.
double round3(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.3f", value);
  return std::strtod(buf, nullptr);
}

std::string message(const std::string & kind, const std::string & reason, const std::string & ratio)
{
  if (kind == EVENT_DISTURBED) {
    return "SLAM map guard: robot " + reason + " (agreement " + ratio + "); SLAM paused until it settles";
  }
  if (kind == EVENT_RESUMED) {
    return "SLAM map guard: scan agrees with the map again (agreement " + ratio + ") after " + reason +
           "; SLAM resumed";
  }
  return "SLAM map guard: map no longer matches after " + reason + " (agreement " + ratio +
         "); map backed up and reset, relocalising against it failed, so the old poses and POIs were discarded "
         "(data.discarded_pois)";
}

double wall_clock_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace

std::optional<std::string> http_get(const std::string & url, double timeout_s)
{
  const auto scheme_end = url.find("://");
  if (scheme_end == std::string::npos) {
    return std::nullopt;
  }
  const auto path_start = url.find('/', scheme_end + 3);
  const std::string base = url.substr(0, path_start);
  const std::string path = path_start == std::string::npos ? "/" : url.substr(path_start);
  try {
    httplib::Client client(base);
    const auto seconds = static_cast<time_t>(timeout_s);
    const auto micros = static_cast<time_t>(std::llround((timeout_s - static_cast<double>(seconds)) * 1e6));
    client.set_connection_timeout(seconds, micros);
    client.set_read_timeout(seconds, micros);
    client.set_write_timeout(seconds, micros);
    client.set_follow_location(true);
    const auto response = client.Get(path);
    if (!response || response->status < 200 || response->status >= 300) {
      return std::nullopt;
    }
    return response->body;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool parse_busy(const std::string & body)
{
  const auto payload = nlohmann::json::parse(body, nullptr, false);
  if (payload.is_discarded() || !payload.is_object()) {
    return false;
  }
  const auto busy = payload.find("busy");
  return busy != payload.end() && busy->is_boolean() && busy->get<bool>();
}

bool fetch_busy(const std::string & url, double timeout_s, const HttpGet & get)
{
  const auto body = get(url, timeout_s);
  return body && parse_busy(*body);
}

AgentState::AgentState(std::string url, double poll_s, double timeout_s, HttpGet get)
: url_(std::move(url)), poll_s_(poll_s), timeout_s_(timeout_s), get_(std::move(get))
{
}

bool AgentState::busy(double now)
{
  if (!last_poll_ || now - *last_poll_ >= poll_s_) {
    last_busy_ = fetch_busy(url_, timeout_s_, get_);
    last_poll_ = now;
  }
  return last_busy_;
}

std::string event_severity(const std::string & kind, const std::string & reason, bool dry_run)
{
  if (kind == EVENT_DISTURBED && stops_driving(reason)) {
    return SEVERITY_CRITICAL;
  }
  if (kind == EVENT_RESET && !dry_run) {
    return SEVERITY_CRITICAL;
  }
  return SEVERITY_WARNING;
}

nlohmann::ordered_json build_event(
  long seq, double ts, const std::string & kind, const std::string & reason,  // NOLINT(runtime/int)
  const std::optional<double> & ratio, bool dry_run, const EventData & data)
{
  const std::string text = message(kind, reason, ratio ? two_decimals(*ratio) : "unknown");
  nlohmann::ordered_json event_data = {
    {"reason", reason},
    {"ratio", ratio ? nlohmann::ordered_json(round3(*ratio)) : nlohmann::ordered_json(nullptr)},
    {"dry_run", dry_run},
  };
  for (const auto & [key, value] : data) {
    event_data[key] = value;
  }
  return {
    {"seq", seq},
    {"ts", ts},
    {"type", kind},
    {"severity", event_severity(kind, reason, dry_run)},
    {"source", EVENT_SOURCE},
    {"message", dry_run ? DRY_RUN_PREFIX + text : text},
    {"data", event_data},
  };
}

EventNotifier::EventNotifier(
  AgentState agent, std::function<void(const std::string &)> publish, bool dry_run, std::function<double()> epoch_now)
: agent_(std::move(agent)),
  publish_(std::move(publish)),
  dry_run_(dry_run),
  epoch_now_(epoch_now ? std::move(epoch_now) : wall_clock_s)
{
}

std::optional<nlohmann::ordered_json> EventNotifier::send(
  double now, const std::string & kind, const std::string & reason, const std::optional<double> & ratio,
  const EventData & data)
{
  if (!agent_.busy(now)) {
    return std::nullopt;
  }
  ++seq_;
  auto event = build_event(seq_, epoch_now_(), kind, reason, ratio, dry_run_, data);
  publish_(event.dump());
  return event;
}

void EventNotifier::notify(
  double now, const std::string & kind, const std::string & reason, const std::optional<double> & ratio,
  const EventData & data)
{
  send(now, kind, reason, ratio, data);
}

}  // namespace map_guard_cpp
