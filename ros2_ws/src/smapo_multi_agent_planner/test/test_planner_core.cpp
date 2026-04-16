#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "smapo_multi_agent_planner/planner_core.hpp"

namespace smapo_multi_agent_planner
{
namespace
{

GridMap emptyMap(int width, int height)
{
  return GridMap(width, height, std::vector<std::int8_t>(
    static_cast<std::size_t>(width * height), 0));
}

}  // namespace

TEST(PlannerCoreTest, FindsStraightPathOnFreeMap)
{
  PlannerCore planner;
  const auto map = emptyMap(5, 5);

  PlanRequest request;
  request.start = Position{0, 0};
  request.goal = Position{4, 0};

  const auto result = planner.plan(map, request);

  ASSERT_TRUE(result.success) << result.status;
  ASSERT_EQ(result.path.front(), request.start);
  ASSERT_EQ(result.path.back(), request.goal);
  EXPECT_EQ(result.path.size(), 5U);
}

TEST(PlannerCoreTest, RoutesAroundStaticObstacle)
{
  auto map = emptyMap(5, 3);
  map.setCell(Position{2, 0}, 100);
  map.setCell(Position{2, 1}, 100);

  PlanRequest request;
  request.start = Position{0, 0};
  request.goal = Position{4, 0};

  const auto result = PlannerCore().plan(map, request);

  ASSERT_TRUE(result.success) << result.status;
  ASSERT_EQ(result.path.back(), request.goal);
  EXPECT_GT(result.path.size(), 5U);
}

TEST(PlannerCoreTest, ReportsUnreachableGoal)
{
  auto map = emptyMap(3, 1);
  map.setCell(Position{1, 0}, 100);

  PlanRequest request;
  request.start = Position{0, 0};
  request.goal = Position{2, 0};

  const auto result = PlannerCore().plan(map, request);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.empty());
}

TEST(PlannerCoreTest, DynamicOccupancyCanChangeSelectedCorridor)
{
  PlannerConfig config;
  config.dynamic_occupancy_weight = 20.0;
  PlannerCore planner(config);
  const auto map = emptyMap(5, 3);

  PlanRequest request;
  request.start = Position{0, 1};
  request.goal = Position{4, 1};
  request.dynamic_occupancy[Position{1, 1}] = 1.0;
  request.dynamic_occupancy[Position{2, 1}] = 1.0;
  request.dynamic_occupancy[Position{3, 1}] = 1.0;

  const auto result = planner.plan(map, request);

  ASSERT_TRUE(result.success) << result.status;
  bool used_detour = false;
  for (const auto & cell : result.path) {
    if (cell.y != 1) {
      used_detour = true;
    }
  }
  EXPECT_TRUE(used_detour);
}

TEST(PlannerCoreTest, RespectsExpansionLimit)
{
  PlannerConfig config;
  config.max_expansions = 1;
  config.planning_timeout = std::chrono::milliseconds(100);
  PlannerCore planner(config);

  PlanRequest request;
  request.start = Position{0, 0};
  request.goal = Position{9, 9};

  const auto result = planner.plan(emptyMap(10, 10), request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, "max_expansions_exceeded");
}

}  // namespace smapo_multi_agent_planner

