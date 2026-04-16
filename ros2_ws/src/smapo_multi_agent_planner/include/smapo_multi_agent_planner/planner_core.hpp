#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace smapo_multi_agent_planner
{

struct Position
{
  int x{0};
  int y{0};

  bool operator==(const Position & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct PositionHash
{
  std::size_t operator()(const Position & position) const noexcept;
};

struct CellTime
{
  int x{0};
  int y{0};
  int t{0};

  bool operator==(const CellTime & other) const
  {
    return x == other.x && y == other.y && t == other.t;
  }
};

struct CellTimeHash
{
  std::size_t operator()(const CellTime & cell_time) const noexcept;
};

class GridMap
{
public:
  GridMap() = default;
  GridMap(int width, int height, std::vector<std::int8_t> cells);

  int width() const noexcept;
  int height() const noexcept;
  bool empty() const noexcept;
  bool inBounds(const Position & position) const noexcept;
  bool isFree(const Position & position) const noexcept;
  std::int8_t cellValue(const Position & position) const;
  void setCell(const Position & position, std::int8_t value);

private:
  int width_{0};
  int height_{0};
  std::vector<std::int8_t> cells_;
};

struct PlannerConfig
{
  std::size_t max_expansions{20000};
  double dynamic_occupancy_weight{2.0};
  double planned_conflict_weight{4.0};
  int conflict_time_window{2};
  bool allow_wait_action{true};
  std::chrono::milliseconds planning_timeout{30};
};

struct PlanRequest
{
  Position start;
  Position goal;
  std::unordered_map<Position, double, PositionHash> dynamic_occupancy;
  std::unordered_map<CellTime, double, CellTimeHash> planned_occupancy;
};

struct PlanResult
{
  bool success{false};
  std::vector<Position> path;
  double cost{0.0};
  std::size_t expansions{0};
  std::chrono::microseconds elapsed{0};
  std::string status{"not_started"};
};

class PlannerCore
{
public:
  explicit PlannerCore(PlannerConfig config = PlannerConfig{});

  const PlannerConfig & config() const noexcept;
  void setConfig(const PlannerConfig & config);
  PlanResult plan(const GridMap & map, const PlanRequest & request) const;

private:
  double heuristic(const Position & a, const Position & b) const noexcept;
  double transitionCost(const PlanRequest & request, const Position & to, int arrival_time) const;

  PlannerConfig config_;
};

}  // namespace smapo_multi_agent_planner
