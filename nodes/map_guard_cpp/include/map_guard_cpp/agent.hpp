// Agent notification: /robot_events JSON, sent only while claude_agent is processing an instruction (busy).
// Port of nodes/map_guard/map_guard/agent.py. Contract (nodes/claude_agent/claude_agent/robot_events.py):
// {seq, ts, type, severity: info|warning|critical, source, message, data}; claude_agent interrupts its turn only on
// critical events.
#ifndef MAP_GUARD_CPP__AGENT_HPP_
#define MAP_GUARD_CPP__AGENT_HPP_

#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace map_guard_cpp
{

constexpr const char * EVENT_SOURCE = "map_guard";
constexpr const char * EVENT_DISTURBED = "map_guard_disturbed";
constexpr const char * EVENT_RESET = "map_guard_reset";
constexpr const char * EVENT_RESUMED = "map_guard_resumed";
constexpr const char * SEVERITY_WARNING = "warning";
constexpr const char * SEVERITY_CRITICAL = "critical";
constexpr const char * DRY_RUN_PREFIX = "[dry run] ";

// Extra fields of an event's data (e.g. {"backup": "<path>", "discarded_pois": 3}).
using EventData = std::map<std::string, nlohmann::json>;

// GET url with a timeout (s): the body of a 2xx response, nullopt on any failure (unreachable, timeout, HTTP error).
using HttpGet = std::function<std::optional<std::string>(const std::string & url, double timeout_s)>;

// HttpGet over cpp-httplib (http:// URLs).
std::optional<std::string> http_get(const std::string & url, double timeout_s);

// The "busy" field of a /api/state body when it is the JSON literal true; false for anything else (malformed JSON,
// not an object, missing, not a bool).
bool parse_busy(const std::string & body);

// Ask claude_agent whether it is processing an instruction; false when unreachable, timed out or malformed.
bool fetch_busy(const std::string & url, double timeout_s, const HttpGet & get = http_get);

// Cached agent busy flag, re-fetched at most every poll_s.
class AgentState
{
public:
  AgentState(std::string url, double poll_s, double timeout_s, HttpGet get = http_get);

  // Busy flag at now (monotonic s), fetched when the cached one is older than poll_s.
  bool busy(double now);

private:
  std::string url_;
  double poll_s_;
  double timeout_s_;
  HttpGet get_;
  std::optional<double> last_poll_;
  bool last_busy_ = false;
};

// "critical" for a lifted/carried disturbance (also in dry run: the agent must stop driving) and for a real reset,
// else "warning".
std::string event_severity(const std::string & kind, const std::string & reason, bool dry_run);

// One /robot_events payload (keys in the Python order). ratio is the latest agreement median (none: unknown).
nlohmann::ordered_json build_event(
  long seq, double ts, const std::string & kind, const std::string & reason,  // NOLINT(runtime/int)
  const std::optional<double> & ratio, bool dry_run, const EventData & data = {});

// Agent event sink of the guard.
class Notifier
{
public:
  virtual ~Notifier() = default;
  // Send an event if the agent is busy.
  virtual void notify(
    double now, const std::string & kind, const std::string & reason, const std::optional<double> & ratio,
    const EventData & data) = 0;
};

// Publishes map guard events (JSON text) only while the agent is busy.
class EventNotifier : public Notifier
{
public:
  // publish receives the JSON text; epoch_now gives the event ts (wall clock, s).
  EventNotifier(
    AgentState agent, std::function<void(const std::string &)> publish, bool dry_run,
    std::function<double()> epoch_now = nullptr);

  // The published event, or nullopt when the agent is idle or unreachable.
  std::optional<nlohmann::ordered_json> send(
    double now, const std::string & kind, const std::string & reason, const std::optional<double> & ratio,
    const EventData & data = {});

  void notify(
    double now, const std::string & kind, const std::string & reason, const std::optional<double> & ratio,
    const EventData & data) override;

private:
  AgentState agent_;
  std::function<void(const std::string &)> publish_;
  bool dry_run_;
  std::function<double()> epoch_now_;
  long seq_ = 0;  // NOLINT(runtime/int)
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__AGENT_HPP_
