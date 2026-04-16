#include "smapo_multi_agent_planner/task_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace smapo_multi_agent_planner
{

TaskScheduler::TaskScheduler(SchedulerConfig config)
: config_(config)
{
}

std::vector<TaskAssignment> TaskScheduler::assign(
  const std::vector<AgentState> & agents,
  const std::vector<TaskRequest> & tasks) const
{
  std::vector<TaskRequest> sorted_tasks = tasks;
  std::sort(sorted_tasks.begin(), sorted_tasks.end(), [](const TaskRequest & lhs, const TaskRequest & rhs) {
    if (lhs.priority != rhs.priority) {
      return lhs.priority > rhs.priority;
    }
    return lhs.task_id < rhs.task_id;
  });

  std::vector<TaskAssignment> assignments;
  std::unordered_set<std::string> assigned_agents;

  for (const auto & task : sorted_tasks) {
    const AgentState * best_agent = nullptr;
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto & agent : agents) {
      if (config_.skip_busy_agents && agent.busy) {
        continue;
      }
      if (assigned_agents.find(agent.agent_id) != assigned_agents.end()) {
        continue;
      }

      const double candidate_score = score(agent, task);
      if (candidate_score < best_score) {
        best_score = candidate_score;
        best_agent = &agent;
      }
    }

    if (best_agent == nullptr) {
      continue;
    }

    assigned_agents.insert(best_agent->agent_id);
    assignments.push_back(TaskAssignment{best_agent->agent_id, task.task_id, best_score});
  }

  return assignments;
}

double TaskScheduler::score(const AgentState & agent, const TaskRequest & task) const noexcept
{
  const auto distance = std::abs(agent.position.x - task.goal.x) +
    std::abs(agent.position.y - task.goal.y);
  return static_cast<double>(distance) - config_.priority_weight * static_cast<double>(task.priority);
}

}  // namespace smapo_multi_agent_planner

