#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace smapo_multi_agent_planner
{

struct RealtimeMetrics
{
  std::uint64_t cycles{0};
  std::uint64_t successful_plans{0};
  std::uint64_t failed_plans{0};
  std::uint64_t timeout_plans{0};
  double last_latency_ms{0.0};
  double average_latency_ms{0.0};
  double max_latency_ms{0.0};
  std::size_t task_queue_size{0};
  bool healthy{true};
};

class RealtimeMonitor
{
public:
  explicit RealtimeMonitor(std::chrono::milliseconds timeout_budget = std::chrono::milliseconds(30));

  void recordPlan(std::chrono::microseconds elapsed, bool success, bool timeout);
  void recordCycle(std::size_t task_queue_size);
  void reset();

  RealtimeMetrics metrics() const noexcept;

private:
  std::chrono::milliseconds timeout_budget_;
  RealtimeMetrics metrics_;
  double total_latency_ms_{0.0};
};

}  // namespace smapo_multi_agent_planner
