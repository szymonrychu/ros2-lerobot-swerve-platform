// The ROS wiring of Map3dManagerNode against in-process fake RTAB-Map Empty services and a temporary database
// directory: save, reset, pause/resume, busy, timeout, invalid, map_reset and the status line.
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "map3d_manager_cpp/map3d_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/map_graph.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace m3 = map3d_manager_cpp;
using namespace std::chrono_literals;
using nlohmann::json;

namespace
{

constexpr const char * NS = "/rtabmap_test";
constexpr const char * DB_CONTENT = "rtabmap database bytes";

// Stand-in for rtabmap_ros: the four Empty services, each counting its calls; `missing` leaves one unserved.
class FakeRtabmap : public rclcpp::Node
{
public:
  explicit FakeRtabmap(const std::vector<std::string> & missing = {})
  : Node("fake_rtabmap_" + std::to_string(next_id()))
  {
    for (const std::string name : {"backup", "reset", "pause", "resume"}) {
      if (std::find(missing.begin(), missing.end(), name) != missing.end()) {
        continue;
      }
      services_.push_back(create_service<std_srvs::srv::Empty>(
        std::string(NS) + "/" + name,
        [this, name](const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
          std::this_thread::sleep_for(delay);
          ++calls[name];
        }));
    }
  }
  std::chrono::milliseconds delay{0};
  std::map<std::string, std::atomic<int>> calls;

private:
  static int next_id()
  {
    static int id = 0;
    return id++;
  }
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};

// Harness: a manager over a temp dir, a client node publishing commands and collecting results and statuses.
class Harness
{
public:
  explicit Harness(const std::vector<std::string> & missing = {}, double service_timeout_s = 2.0, double period_s = 0.2)
  : dir(
      std::filesystem::temp_directory_path() /
      ("map3d_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter++)))
  {
    std::filesystem::create_directories(dir);
    std::ofstream(dir / m3::LIVE_DB_NAME) << DB_CONTENT;
    m3::Config config;
    config.rtabmap_namespace = NS;
    config.database_dir = dir.string();
    config.service_timeout_s = service_timeout_s;
    config.status_period_s = period_s;
    rtabmap = std::make_shared<FakeRtabmap>(missing);
    manager = std::make_shared<m3::Map3dManagerNode>(config);
    client = std::make_shared<rclcpp::Node>("map3d_test_client");
    command_pub = client->create_publisher<std_msgs::msg::String>(config.command_topic, rclcpp::QoS(10).reliable());
    reset_pub = client->create_publisher<std_msgs::msg::Empty>(config.map_reset_topic, rclcpp::QoS(10).reliable());
    rgb_pub = client->create_publisher<sensor_msgs::msg::CameraInfo>(config.rgb_info_topic, rclcpp::SensorDataQoS());
    depth_pub =
      client->create_publisher<sensor_msgs::msg::CameraInfo>(config.depth_info_topic, rclcpp::SensorDataQoS());
    info_pub = client->create_publisher<rtabmap_msgs::msg::Info>(std::string(NS) + "/info", rclcpp::SensorDataQoS());
    graph_pub =
      client->create_publisher<rtabmap_msgs::msg::MapGraph>(std::string(NS) + "/mapGraph", rclcpp::SensorDataQoS());
    result_sub = client->create_subscription<std_msgs::msg::String>(
      config.result_topic, rclcpp::QoS(10).reliable(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        results.push_back(json::parse(msg->data));
      });
    status_sub = client->create_subscription<std_msgs::msg::String>(
      config.status_topic, rclcpp::QoS(10).reliable(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        statuses.push_back(json::parse(msg->data));
      });
    executor.add_node(manager);
    executor.add_node(rtabmap);
    executor.add_node(client);
    spinner = std::thread([this]() { executor.spin(); });
  }
  ~Harness()
  {
    executor.cancel();
    spinner.join();
    executor.remove_node(manager);
    manager.reset();
    std::filesystem::remove_all(dir);
  }

  void send(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    wait_for([this]() { return command_pub->get_subscription_count() > 0; });
    command_pub->publish(msg);
  }
  void send(const std::string & id, const std::string & action, const std::string & name = "")
  {
    json j = {{"request_id", id}, {"action", action}};
    if (!name.empty()) {
      j["name"] = name;
    }
    send(j.dump());
  }

  bool wait_for(const std::function<bool()> & pred, std::chrono::milliseconds limit = 5000ms)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }
  // The result with this request id (null json when none within the limit).
  json result_of(const std::string & id, std::chrono::milliseconds limit = 5000ms)
  {
    json found;
    wait_for(
      [&]() {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto & r : results) {
          if (r.value("request_id", "") == id) {
            found = r;
            return true;
          }
        }
        return false;
      },
      limit);
    return found;
  }
  std::size_t result_count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return results.size();
  }
  json last_status()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return statuses.empty() ? json() : statuses.back();
  }
  bool wait_status(const std::function<bool(const json &)> & pred)
  {
    return wait_for([&]() {
      const json s = last_status();
      return !s.is_null() && pred(s);
    });
  }
  void publish_infos()
  {
    sensor_msgs::msg::CameraInfo info;
    rgb_pub->publish(info);
    depth_pub->publish(info);
    info_pub->publish(rtabmap_msgs::msg::Info());
  }
  std::string read(const std::filesystem::path & path)
  {
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in), {});
  }

  static inline std::atomic<int> counter{0};
  std::filesystem::path dir;
  std::shared_ptr<FakeRtabmap> rtabmap;
  std::shared_ptr<m3::Map3dManagerNode> manager;
  std::shared_ptr<rclcpp::Node> client;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_pub;
  rclcpp::Publisher<rtabmap_msgs::msg::Info>::SharedPtr info_pub;
  rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr graph_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 4};
  std::thread spinner;
  std::mutex mutex;
  std::vector<json> results;
  std::vector<json> statuses;
};

}  // namespace

TEST(Map3dManagerNode, SaveCopiesLiveDbToSavedAndNamedAtomically)
{
  Harness h;
  h.send("s1", "save", "office");
  const json r = h.result_of("s1");
  ASSERT_FALSE(r.is_null());
  EXPECT_EQ(r["status"], "ok");
  EXPECT_EQ(r["action"], "save");
  EXPECT_EQ(h.rtabmap->calls["backup"], 1);
  EXPECT_EQ(h.read(h.dir / "saved.db"), DB_CONTENT);
  EXPECT_EQ(h.read(h.dir / "office.db"), DB_CONTENT);
  EXPECT_EQ(r["db_path"], (h.dir / "saved.db").string());
  EXPECT_GT(r["db_size_mb"].get<double>(), 0.0);
  for (const auto & entry : std::filesystem::directory_iterator(h.dir)) {
    EXPECT_NE(entry.path().extension(), ".tmp") << entry.path();
  }
}

TEST(Map3dManagerNode, SaveWithoutNameUsesDefaultName)
{
  Harness h;
  h.send("s2", "save");
  EXPECT_EQ(h.result_of("s2")["status"], "ok");
  EXPECT_TRUE(std::filesystem::exists(h.dir / (std::string(m3::DEFAULT_MAP_NAME) + ".db")));
}

TEST(Map3dManagerNode, SaveWithMissingLiveDbIsAnErrorWithErrnoText)
{
  Harness h;
  std::filesystem::remove(h.dir / m3::LIVE_DB_NAME);
  h.send("s3", "save", "x");
  const json r = h.result_of("s3");
  EXPECT_EQ(r["status"], "error");
  EXPECT_FALSE(r["message"].get<std::string>().empty());
  EXPECT_FALSE(std::filesystem::exists(h.dir / "saved.db"));
}

TEST(Map3dManagerNode, SaveNamedLiveIsInvalidAndLeavesTheRunningDatabase)
{
  Harness h;
  h.send("lv", "save", "live");
  EXPECT_EQ(h.result_of("lv")["status"], "invalid");
  EXPECT_EQ(h.rtabmap->calls["backup"], 0);
}

TEST(Map3dManagerNode, ResetCallsTheResetService)
{
  Harness h;
  h.send("r1", "reset");
  EXPECT_EQ(h.result_of("r1")["status"], "ok");
  EXPECT_EQ(h.rtabmap->calls["reset"], 1);
}

TEST(Map3dManagerNode, PauseAndResumeCallServicesAndTrackThePausedFlag)
{
  Harness h;
  h.publish_infos();
  h.send("p1", "pause");
  EXPECT_EQ(h.result_of("p1")["status"], "ok");
  EXPECT_EQ(h.rtabmap->calls["pause"], 1);
  EXPECT_TRUE(h.wait_status([](const json & s) { return s["state"] == "paused"; }));
  h.send("p2", "resume");
  EXPECT_EQ(h.result_of("p2")["status"], "ok");
  EXPECT_EQ(h.rtabmap->calls["resume"], 1);
  h.publish_infos();
  EXPECT_TRUE(h.wait_status([](const json & s) { return s["state"] != "paused"; }));
}

TEST(Map3dManagerNode, SecondCommandWhileOneRunsIsBusy)
{
  Harness h;
  h.rtabmap->delay = 600ms;
  h.send("b1", "reset");
  ASSERT_TRUE(h.wait_for([&]() { return h.rtabmap->calls["reset"] == 0 && h.manager->busy(); }));
  h.send("b2", "reset");
  const json busy = h.result_of("b2");
  EXPECT_EQ(busy["status"], "busy");
  EXPECT_EQ(h.result_of("b1")["status"], "ok");
  EXPECT_EQ(h.rtabmap->calls["reset"], 1);
}

TEST(Map3dManagerNode, MissingServiceIsATimeout)
{
  Harness h({"reset"}, 0.3);
  h.send("t1", "reset");
  const json r = h.result_of("t1");
  EXPECT_EQ(r["status"], "timeout");
  h.send("t2", "save", "z");
  EXPECT_EQ(h.result_of("t2")["status"], "ok");  // backup is served
}

TEST(Map3dManagerNode, InvalidCommandKeepsTheRequestId)
{
  Harness h;
  h.send("i1", "explode");
  const json r = h.result_of("i1");
  EXPECT_EQ(r["status"], "invalid");
  EXPECT_EQ(r["request_id"], "i1");
  h.send("i2", "save", "bad name!");
  EXPECT_EQ(h.result_of("i2")["status"], "invalid");
}

TEST(Map3dManagerNode, UnusableCommandIsDroppedWithoutResult)
{
  Harness h;
  h.send("not json");
  h.send("{\"action\": \"save\"}");
  h.send("sync", "reset");
  EXPECT_EQ(h.result_of("sync")["status"], "ok");
  EXPECT_EQ(h.result_count(), 1u);
}

TEST(Map3dManagerNode, MapResetMessageTriggersResetWithoutAResult)
{
  Harness h;
  ASSERT_TRUE(h.wait_for([&]() { return h.reset_pub->get_subscription_count() > 0; }));
  h.reset_pub->publish(std_msgs::msg::Empty());
  ASSERT_TRUE(h.wait_for([&]() { return h.rtabmap->calls["reset"] == 1; }));
  EXPECT_EQ(h.result_count(), 0u);
}

TEST(Map3dManagerNode, MapResetWhileAnActionRunsIsAppliedAfterItFinishes)
{
  Harness h;
  h.rtabmap->delay = 400ms;
  ASSERT_TRUE(h.wait_for([&]() { return h.reset_pub->get_subscription_count() > 0; }));
  h.send("s1", "pause");
  ASSERT_TRUE(h.wait_for([&]() { return h.manager->busy(); }));
  h.reset_pub->publish(std_msgs::msg::Empty());
  EXPECT_EQ(h.result_of("s1")["status"], "ok");
  ASSERT_TRUE(h.wait_for([&]() { return h.rtabmap->calls["reset"] == 1; }));
  EXPECT_EQ(h.result_count(), 1u);
}

TEST(Map3dManagerNode, StatusWaitsForInputsThenMapsAndTakesTheGraphCorrection)
{
  Harness h;
  ASSERT_TRUE(h.wait_status([](const json & s) { return s["state"] == "waiting_for_inputs"; }));
  const json first = h.last_status();
  EXPECT_TRUE(first["nodes"].is_null());
  EXPECT_TRUE(first["graph_correction_m"].is_null());
  EXPECT_EQ(first["missing_inputs"].size(), 3u);

  rtabmap_msgs::msg::MapGraph graph;
  graph.header.stamp = h.client->now();
  graph.map_to_odom.translation.x = 0.3;
  graph.map_to_odom.translation.y = 0.4;
  graph.map_to_odom.rotation.w = 1.0;
  graph.poses.resize(7);
  graph.poses_id.resize(7);
  ASSERT_TRUE(h.wait_for([&]() { return h.graph_pub->get_subscription_count() > 0; }));
  ASSERT_TRUE(h.wait_for([&]() {
    h.publish_infos();
    h.graph_pub->publish(graph);
    const json s = h.last_status();
    return !s.is_null() && s["state"] == "mapping" && s["nodes"] == 7;
  }));
  const json s = h.last_status();
  EXPECT_NEAR(s["graph_correction_m"].get<double>(), 0.5, 1e-6);
  EXPECT_NEAR(s["graph_correction_deg"].get<double>(), 0.0, 1e-6);
  EXPECT_EQ(s["correction_ok"], false);
  EXPECT_TRUE(s["last_save"].is_null());
  EXPECT_NEAR(s["db_size_mb"].get<double>(), std::strlen(DB_CONTENT) / (1024.0 * 1024.0), 1e-9);
}

TEST(Map3dManagerNode, StatusReportsLastSave)
{
  Harness h;
  h.send("ls", "save", "kitchen");
  ASSERT_EQ(h.result_of("ls")["status"], "ok");
  ASSERT_TRUE(h.wait_status([](const json & s) { return !s["last_save"].is_null(); }));
  EXPECT_EQ(h.last_status()["last_save"]["name"], "kitchen");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
