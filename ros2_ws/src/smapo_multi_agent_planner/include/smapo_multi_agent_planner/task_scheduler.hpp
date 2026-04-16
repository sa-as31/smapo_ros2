#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "smapo_multi_agent_planner/planner_core.hpp"

namespace smapo_multi_agent_planner
{

struct AgentState
{
  std::string agent_id;
  Position position;
  bool busy{false};
  std::int64_t timestamp_ms{0};
};

struct TaskRequest
{
  std::string task_id;
  Position goal;
  int priority{0};
};

struct TaskAssignment
{
  std::string agent_id;
  std::string task_id;
  double score{0.0};
};

struct SchedulerConfig
{
  bool skip_busy_agents{true};
  double priority_weight{10.0};
};

class TaskScheduler
{
public:
  explicit TaskScheduler(SchedulerConfig config = SchedulerConfig{});

  std::vector<TaskAssignment> assign(
    const std::vector<AgentState> & agents,
    const std::vector<TaskRequest> & tasks) const;

private:
  double score(const AgentState & agent, const TaskRequest & task) const noexcept;

  SchedulerConfig config_;
};

}  // namespace smapo_multi_agent_planner

