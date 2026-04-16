#include "smapo_multi_agent_planner/realtime_monitor.hpp"

#include <algorithm>

namespace smapo_multi_agent_planner
{

RealtimeMonitor::RealtimeMonitor(std::chrono::milliseconds timeout_budget)
: timeout_budget_(timeout_budget)
{
}

void RealtimeMonitor::recordPlan(std::chrono::microseconds elapsed, bool success, bool timeout)
{
  const double latency_ms = static_cast<double>(elapsed.count()) / 1000.0;
  metrics_.last_latency_ms = latency_ms;
  metrics_.max_latency_ms = std::max(metrics_.max_latency_ms, latency_ms);
  total_latency_ms_ += latency_ms;

  const auto completed = metrics_.successful_plans + metrics_.failed_plans + 1U;
  metrics_.average_latency_ms = total_latency_ms_ / static_cast<double>(completed);

  if (success) {
    ++metrics_.successful_plans;
  } else {
    ++metrics_.failed_plans;
  }

  if (timeout || elapsed > timeout_budget_) {
    ++metrics_.timeout_plans;
  }

  metrics_.healthy = metrics_.timeout_plans == 0U && metrics_.failed_plans <= metrics_.successful_plans + 3U;
}

void RealtimeMonitor::recordCycle(std::size_t task_queue_size)
{
  ++metrics_.cycles;
  metrics_.task_queue_size = task_queue_size;
}

void RealtimeMonitor::reset()
{
  metrics_ = RealtimeMetrics{};
  total_latency_ms_ = 0.0;
}

RealtimeMetrics RealtimeMonitor::metrics() const noexcept
{
  return metrics_;
}

}  // namespace smapo_multi_agent_planner

