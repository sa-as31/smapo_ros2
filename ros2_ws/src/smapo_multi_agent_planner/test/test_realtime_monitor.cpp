#include <gtest/gtest.h>

#include <chrono>

#include "smapo_multi_agent_planner/realtime_monitor.hpp"

namespace smapo_multi_agent_planner
{

TEST(RealtimeMonitorTest, RecordsSuccessfulPlanLatency)
{
  RealtimeMonitor monitor(std::chrono::milliseconds(30));

  monitor.recordCycle(3);
  monitor.recordPlan(std::chrono::microseconds(2500), true, false);

  const auto metrics = monitor.metrics();
  EXPECT_EQ(metrics.cycles, 1U);
  EXPECT_EQ(metrics.task_queue_size, 3U);
  EXPECT_EQ(metrics.successful_plans, 1U);
  EXPECT_EQ(metrics.failed_plans, 0U);
  EXPECT_DOUBLE_EQ(metrics.last_latency_ms, 2.5);
  EXPECT_TRUE(metrics.healthy);
}

TEST(RealtimeMonitorTest, CountsTimeouts)
{
  RealtimeMonitor monitor(std::chrono::milliseconds(10));

  monitor.recordPlan(std::chrono::microseconds(15000), false, true);

  const auto metrics = monitor.metrics();
  EXPECT_EQ(metrics.failed_plans, 1U);
  EXPECT_EQ(metrics.timeout_plans, 1U);
  EXPECT_FALSE(metrics.healthy);
}

TEST(RealtimeMonitorTest, ResetClearsMetrics)
{
  RealtimeMonitor monitor(std::chrono::milliseconds(10));
  monitor.recordCycle(1);
  monitor.recordPlan(std::chrono::microseconds(15000), false, true);

  monitor.reset();

  const auto metrics = monitor.metrics();
  EXPECT_EQ(metrics.cycles, 0U);
  EXPECT_EQ(metrics.failed_plans, 0U);
  EXPECT_EQ(metrics.timeout_plans, 0U);
  EXPECT_TRUE(metrics.healthy);
}

}  // namespace smapo_multi_agent_planner

