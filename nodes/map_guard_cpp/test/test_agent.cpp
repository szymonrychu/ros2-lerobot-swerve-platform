// Port of nodes/map_guard/tests/test_agent.py (agent-busy gating and the /robot_events JSON contract) plus the
// agent.json golden fixture, and the cpp-httplib GET against a local HTTP server.
#include <gtest/gtest.h>
#include <httplib.h>

#include <chrono>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fixture_util.hpp"
#include "map_guard_cpp/agent.hpp"

namespace mg = map_guard_cpp;
namespace fx = map_guard_cpp::test;
using nlohmann::json;

namespace
{

const std::string URL = "http://127.0.0.1:18300/api/state";
const std::vector<std::string> CONTRACT_KEYS = {"seq", "ts", "type", "severity", "source", "message", "data"};

mg::HttpGet returning(
  const std::optional<std::string> & body, std::vector<std::pair<std::string, double>> * calls = nullptr)
{
  return [body, calls](const std::string & url, double timeout_s) {
    if (calls != nullptr) {
      calls->emplace_back(url, timeout_s);
    }
    return body;
  };
}

struct Notifier
{
  explicit Notifier(const std::optional<std::string> & busy_body, bool dry_run = false)
  : notifier(
      mg::AgentState(URL, 2.0, 0.3, returning(busy_body)),
      [this](const std::string & text) { published.push_back(text); }, dry_run, []() { return 1700000000.25; })
  {
  }
  std::vector<std::string> published;
  mg::EventNotifier notifier;
};

const std::string BUSY = R"({"busy": true})";
const std::string IDLE = R"({"busy": false})";

}  // namespace

TEST(Agent, FetchBusyTrueAndFalse)
{
  EXPECT_TRUE(mg::fetch_busy(URL, 0.5, returning(R"({"busy": true, "session": "x"})")));
  EXPECT_FALSE(mg::fetch_busy(URL, 0.5, returning(IDLE)));
}

TEST(Agent, FetchBusyMalformedIsNotBusy)
{
  for (const std::string body : {R"({"state": "idle"})", R"({"busy": "yes"})", "[1, 2]", "not json"}) {
    EXPECT_FALSE(mg::fetch_busy(URL, 0.5, returning(body))) << body;
  }
}

TEST(Agent, FetchBusyUnreachableOrTimedOutIsNotBusy)
{
  EXPECT_FALSE(mg::fetch_busy(URL, 0.5, returning(std::nullopt)));
}

TEST(Agent, AgentStatePollsAtMostEveryPollS)
{
  std::vector<std::pair<std::string, double>> calls;
  mg::AgentState state(URL, 2.0, 0.3, returning(BUSY, &calls));
  EXPECT_TRUE(state.busy(0.0));
  EXPECT_TRUE(state.busy(1.0));
  EXPECT_TRUE(state.busy(1.9));
  ASSERT_EQ(calls.size(), 1u);
  EXPECT_EQ(calls[0], std::make_pair(URL, 0.3));
  state.busy(2.1);
  EXPECT_EQ(calls.size(), 2u);
}

TEST(Agent, EventsPublishedOnlyWhenAgentBusy)
{
  Notifier n(BUSY);
  const auto event = n.notifier.send(0.0, mg::EVENT_DISTURBED, "rotated", 0.42);
  ASSERT_TRUE(event);
  ASSERT_EQ(n.published.size(), 1u);
  EXPECT_EQ(json::parse(n.published[0]), json::parse(event->dump()));
}

TEST(Agent, IdleAgentGetsNoEvent)
{
  Notifier n(IDLE);
  EXPECT_FALSE(n.notifier.send(0.0, mg::EVENT_RESET, "mismatch", 0.1));
  EXPECT_TRUE(n.published.empty());
}

TEST(Agent, UnreachableAgentGetsNoEvent)
{
  Notifier n(std::nullopt);
  EXPECT_FALSE(n.notifier.send(0.0, mg::EVENT_RESET, "mismatch", 0.1));
  EXPECT_TRUE(n.published.empty());
}

TEST(Agent, SeverityByKindAndReason)
{
  const std::vector<std::tuple<std::string, std::string, std::string>> cases = {
    {mg::EVENT_DISTURBED, "lifted", "critical"}, {mg::EVENT_DISTURBED, "carried", "critical"},
    {mg::EVENT_DISTURBED, "rotated", "warning"}, {mg::EVENT_DISTURBED, "mismatch", "warning"},
    {mg::EVENT_RESUMED, "lifted", "warning"},    {mg::EVENT_RESET, "mismatch", "critical"}};
  for (const auto & [kind, reason, severity] : cases) {
    Notifier n(BUSY);
    const auto event = n.notifier.send(0.0, kind, reason, 0.5);
    ASSERT_TRUE(event);
    EXPECT_EQ((*event)["severity"], severity) << kind << " " << reason;
  }
}

TEST(Agent, DryRunSeverityKeepsLiftCritical)
{
  const std::vector<std::tuple<std::string, std::string, std::string>> cases = {
    {mg::EVENT_DISTURBED, "lifted", "critical"}, {mg::EVENT_DISTURBED, "carried", "critical"},
    {mg::EVENT_DISTURBED, "rotated", "warning"}, {mg::EVENT_DISTURBED, "mismatch", "warning"},
    {mg::EVENT_RESUMED, "lifted", "warning"},    {mg::EVENT_RESET, "mismatch", "warning"},
    {mg::EVENT_RESET, "lifted", "warning"}};
  for (const auto & [kind, reason, severity] : cases) {
    Notifier n(BUSY, true);
    const auto event = n.notifier.send(0.0, kind, reason, 0.2);
    ASSERT_TRUE(event);
    EXPECT_EQ((*event)["severity"], severity) << kind << " " << reason;
    EXPECT_EQ((*event)["message"].get<std::string>().rfind("[dry run] ", 0), 0u);
    EXPECT_EQ((*event)["data"]["dry_run"], true);
    EXPECT_EQ(n.published.size(), 1u);
  }
}

TEST(Agent, EventMatchesRobotEventsContract)
{
  const auto event = mg::build_event(7, 1700000000.5, mg::EVENT_RESET, "carried", 0.123456, false, {{"backup", "/x"}});
  std::vector<std::string> keys;
  for (const auto & [key, value] : event.items()) {
    keys.push_back(key);
  }
  EXPECT_EQ(keys, CONTRACT_KEYS);  // the Python key order
  EXPECT_TRUE(event["seq"].is_number_integer());
  EXPECT_EQ(event["seq"], 7);
  EXPECT_TRUE(event["ts"].is_number_float());
  EXPECT_EQ(event["type"], "map_guard_reset");
  EXPECT_EQ(event["source"], "map_guard");
  const std::string message = event["message"];
  EXPECT_GT(message.size(), 0u);
  EXPECT_LE(message.size(), 500u);
  EXPECT_EQ(event["data"]["reason"], "carried");
  EXPECT_NEAR(event["data"]["ratio"].get<double>(), 0.123, 1e-3);
  EXPECT_EQ(event["data"]["backup"], "/x");
  EXPECT_LE(event["data"].dump().size(), 2000u);
}

TEST(Agent, EventRatioMayBeUnknown)
{
  EXPECT_TRUE(mg::build_event(1, 1.0, mg::EVENT_DISTURBED, "lifted", std::nullopt, false)["data"]["ratio"].is_null());
}

TEST(Agent, EventTypes)
{
  EXPECT_STREQ(mg::EVENT_DISTURBED, "map_guard_disturbed");
  EXPECT_STREQ(mg::EVENT_RESET, "map_guard_reset");
  EXPECT_STREQ(mg::EVENT_RESUMED, "map_guard_resumed");
}

TEST(Agent, SequenceNumbersIncrease)
{
  Notifier n(BUSY);
  n.notifier.send(0.0, mg::EVENT_DISTURBED, "lifted", std::nullopt);
  n.notifier.send(0.1, mg::EVENT_RESUMED, "lifted", 0.9);
  ASSERT_EQ(n.published.size(), 2u);
  EXPECT_EQ(json::parse(n.published[0])["seq"], 1);
  EXPECT_EQ(json::parse(n.published[1])["seq"], 2);
  EXPECT_EQ(json::parse(n.published[0])["ts"], 1700000000.25);
}

// --- cpp-httplib GET ------------------------------------------------------------------------------------------------

TEST(AgentHttp, HttpGetReadsALocalServer)
{
  httplib::Server server;
  server.Get("/api/state", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"busy": true})", "application/json");
  });
  server.Get("/slow", [](const httplib::Request &, httplib::Response & res) {
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    res.set_content(R"({"busy": true})", "application/json");
  });
  const int port = server.bind_to_any_port("127.0.0.1");
  std::thread thread([&server]() { server.listen_after_bind(); });
  server.wait_until_ready();
  const std::string base = "http://127.0.0.1:" + std::to_string(port);
  EXPECT_EQ(mg::http_get(base + "/api/state", 0.5), std::optional<std::string>(R"({"busy": true})"));
  EXPECT_TRUE(mg::fetch_busy(base + "/api/state", 0.5));
  EXPECT_FALSE(mg::http_get(base + "/missing", 0.5));  // 404
  EXPECT_FALSE(mg::fetch_busy(base + "/slow", 0.2));   // timed out
  EXPECT_FALSE(mg::http_get("not a url", 0.5));
  server.stop();
  thread.join();
  EXPECT_FALSE(mg::fetch_busy(base + "/api/state", 0.2));  // unreachable
}

// --- Golden fixture (Python outputs) --------------------------------------------------------------------------------

TEST(AgentFixture, MatchesPython)
{
  const auto f = fx::load_fixture("agent.json");
  EXPECT_EQ(
    f["event_types"].get<std::vector<std::string>>(),
    (std::vector<std::string>{mg::EVENT_DISTURBED, mg::EVENT_RESET, mg::EVENT_RESUMED}));
  for (const auto & c : f["busy"]) {
    EXPECT_EQ(mg::parse_busy(c["body"]), c["busy"].get<bool>()) << c["body"];
  }
  for (const auto & c : f["severity"]) {
    EXPECT_EQ(mg::event_severity(c["kind"], c["reason"], c["dry_run"]), c["severity"].get<std::string>());
  }
  for (const auto & c : f["events"]) {
    std::optional<double> ratio;
    if (!c["ratio"].is_null()) {
      ratio = c["ratio"].get<double>();
    }
    mg::EventData data;
    if (!c["data"].is_null()) {
      data = c["data"].get<mg::EventData>();
    }
    const auto event = mg::build_event(
      c["seq"].get<long>(), c["ts"].get<double>(), c["kind"], c["reason"], ratio, c["dry_run"], data);  // NOLINT
    EXPECT_EQ(json::parse(event.dump()), c["event"]) << c["event"].dump() << "\n" << event.dump();
  }
}
