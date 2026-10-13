// The process_* series of prometheus_client's ProcessCollector, which the Python map_guard exports and the C++ port
// must keep (nodes/map_guard has no test for them: they come with prometheus_client).
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <string>

#include "map_guard_cpp/process_metrics.hpp"

using map_guard_cpp::ProcessMetrics;

namespace
{

// Value of the unlabelled series name, or -1 when absent.
double value(ros2_common_cpp::Metrics & metrics, const std::string & name)
{
  for (const auto & family : metrics.registry()->Collect()) {
    if (family.name != name || family.metric.empty()) {
      continue;
    }
    const auto & metric = family.metric.front();
    EXPECT_TRUE(metric.label.empty()) << name;
    return family.type == prometheus::MetricType::Counter ? metric.counter.value : metric.gauge.value;
  }
  return -1.0;
}

void burn_cpu(double seconds)
{
  const clock_t start = clock();
  volatile double x = 0.0;
  while (static_cast<double>(clock() - start) / CLOCKS_PER_SEC < seconds) {
    x = x + 1.0;
  }
}

}  // namespace

TEST(ProcessMetrics, NoValueBeforeTheFirstUpdate)
{
  ros2_common_cpp::Metrics metrics("map_guard_cpp");
  ProcessMetrics process(metrics);
  EXPECT_EQ(metrics.render().find("process_resident_memory_bytes "), std::string::npos);
}

TEST(ProcessMetrics, UpdateExportsEverySeriesOfThePythonCollector)
{
  ros2_common_cpp::Metrics metrics("map_guard_cpp");
  ProcessMetrics process(metrics);
  ASSERT_TRUE(process.update());
  const double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  EXPECT_GT(value(metrics, "process_resident_memory_bytes"), 1e5);
  EXPECT_GE(value(metrics, "process_virtual_memory_bytes"), value(metrics, "process_resident_memory_bytes"));
  EXPECT_GE(value(metrics, "process_cpu_seconds_total"), 0.0);
  const double start = value(metrics, "process_start_time_seconds");
  EXPECT_GT(start, now - 3600.0);
  EXPECT_LE(start, now + 1.0);
  EXPECT_GE(value(metrics, "process_open_fds"), 3.0);
  EXPECT_GE(value(metrics, "process_max_fds"), value(metrics, "process_open_fds"));
  const std::string text = metrics.render();
  EXPECT_NE(text.find("# TYPE process_cpu_seconds_total counter"), std::string::npos);
  EXPECT_NE(text.find("# TYPE process_resident_memory_bytes gauge"), std::string::npos);
}

TEST(ProcessMetrics, CpuSecondsFollowTheProcess)
{
  ros2_common_cpp::Metrics metrics("map_guard_cpp");
  ProcessMetrics process(metrics);
  ASSERT_TRUE(process.update());
  const double before = value(metrics, "process_cpu_seconds_total");
  burn_cpu(0.3);
  ASSERT_TRUE(process.update());
  const double after = value(metrics, "process_cpu_seconds_total");
  EXPECT_GE(after - before, 0.2);
  EXPECT_LE(after - before, 1.0);
}

TEST(ProcessMetrics, UpdateIfDueIsRateLimited)
{
  ros2_common_cpp::Metrics metrics("map_guard_cpp");
  ProcessMetrics process(metrics);
  EXPECT_TRUE(process.update_if_due(10.0));
  EXPECT_FALSE(process.update_if_due(10.5));
  EXPECT_TRUE(process.update_if_due(11.0));
}
