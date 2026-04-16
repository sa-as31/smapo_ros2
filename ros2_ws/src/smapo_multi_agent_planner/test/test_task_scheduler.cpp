#include <gtest/gtest.h>

#include <vector>

#include "smapo_multi_agent_planner/task_scheduler.hpp"

namespace smapo_multi_agent_planner
{

TEST(TaskSchedulerTest, AssignsNearestIdleAgent)
{
  const std::vector<AgentState> agents = {
    AgentState{"a_far", Position{10, 10}, false, 1},
    AgentState{"a_near", Position{1, 1}, false, 1},
  };
  const std::vector<TaskRequest> tasks = {
    TaskRequest{"task", Position{2, 1}, 0},
  };

  const auto assignments = TaskScheduler().assign(agents, tasks);

  ASSERT_EQ(assignments.size(), 1U);
  EXPECT_EQ(assignments.front().agent_id, "a_near");
  EXPECT_EQ(assignments.front().task_id, "task");
}

TEST(TaskSchedulerTest, SkipsBusyAgents)
{
  const std::vector<AgentState> agents = {
    AgentState{"busy_near", Position{1, 1}, true, 1},
    AgentState{"idle_far", Position{5, 5}, false, 1},
  };
  const std::vector<TaskRequest> tasks = {
    TaskRequest{"task", Position{2, 1}, 0},
  };

  const auto assignments = TaskScheduler().assign(agents, tasks);

  ASSERT_EQ(assignments.size(), 1U);
  EXPECT_EQ(assignments.front().agent_id, "idle_far");
}

TEST(TaskSchedulerTest, EmptyTasksReturnNoAssignments)
{
  const std::vector<AgentState> agents = {
    AgentState{"agent", Position{0, 0}, false, 1},
  };

  const auto assignments = TaskScheduler().assign(agents, {});

  EXPECT_TRUE(assignments.empty());
}

TEST(TaskSchedulerTest, HighPriorityTaskIsAssignedFirst)
{
  const std::vector<AgentState> agents = {
    AgentState{"agent_a", Position{0, 0}, false, 1},
    AgentState{"agent_b", Position{9, 9}, false, 1},
  };
  const std::vector<TaskRequest> tasks = {
    TaskRequest{"low", Position{0, 1}, 0},
    TaskRequest{"high", Position{8, 9}, 5},
  };

  const auto assignments = TaskScheduler().assign(agents, tasks);

  ASSERT_EQ(assignments.size(), 2U);
  EXPECT_EQ(assignments.front().task_id, "high");
}

}  // namespace smapo_multi_agent_planner

