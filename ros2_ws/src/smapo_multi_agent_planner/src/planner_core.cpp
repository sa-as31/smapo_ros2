#include "smapo_multi_agent_planner/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

namespace smapo_multi_agent_planner
{
namespace
{

struct SearchState
{
  Position position;
  int time{0};
  double g{0.0};
  double h{0.0};

  double f() const noexcept
  {
    return g + h;
  }
};

struct SearchStateGreater
{
  bool operator()(const SearchState & lhs, const SearchState & rhs) const noexcept
  {
    if (std::abs(lhs.f() - rhs.f()) > 1e-9) {
      return lhs.f() > rhs.f();
    }
    return lhs.h > rhs.h;
  }
};

CellTime toCellTime(const SearchState & state)
{
  return CellTime{state.position.x, state.position.y, state.time};
}

std::vector<Position> reconstructPath(
  CellTime goal_state,
  const std::unordered_map<CellTime, CellTime, CellTimeHash> & parent)
{
  std::vector<Position> reversed;
  CellTime current = goal_state;
  reversed.push_back(Position{current.x, current.y});

  while (true) {
    const auto parent_it = parent.find(current);
    if (parent_it == parent.end()) {
      break;
    }
    current = parent_it->second;
    reversed.push_back(Position{current.x, current.y});
  }

  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

}  // namespace

std::size_t PositionHash::operator()(const Position & position) const noexcept
{
  const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(position.x));
  const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(position.y));
  return static_cast<std::size_t>((x << 32U) ^ y);
}

std::size_t CellTimeHash::operator()(const CellTime & cell_time) const noexcept
{
  const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_time.x));
  const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_time.y));
  const auto t = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_time.t));
  return static_cast<std::size_t>((x << 42U) ^ (y << 21U) ^ t);
}

GridMap::GridMap(int width, int height, std::vector<std::int8_t> cells)
: width_(width), height_(height), cells_(std::move(cells))
{
  if (width_ < 0 || height_ < 0) {
    throw std::invalid_argument("GridMap dimensions must be non-negative");
  }
  if (cells_.size() != static_cast<std::size_t>(width_ * height_)) {
    throw std::invalid_argument("GridMap cell count does not match dimensions");
  }
}

int GridMap::width() const noexcept
{
  return width_;
}

int GridMap::height() const noexcept
{
  return height_;
}

bool GridMap::empty() const noexcept
{
  return width_ <= 0 || height_ <= 0 || cells_.empty();
}

bool GridMap::inBounds(const Position & position) const noexcept
{
  return position.x >= 0 && position.y >= 0 && position.x < width_ && position.y < height_;
}

bool GridMap::isFree(const Position & position) const noexcept
{
  if (!inBounds(position)) {
    return false;
  }
  const auto value = cells_[static_cast<std::size_t>(position.y * width_ + position.x)];
  return value >= 0 && value <= 50;
}

std::int8_t GridMap::cellValue(const Position & position) const
{
  if (!inBounds(position)) {
    throw std::out_of_range("GridMap cell is out of bounds");
  }
  return cells_[static_cast<std::size_t>(position.y * width_ + position.x)];
}

void GridMap::setCell(const Position & position, std::int8_t value)
{
  if (!inBounds(position)) {
    throw std::out_of_range("GridMap cell is out of bounds");
  }
  cells_[static_cast<std::size_t>(position.y * width_ + position.x)] = value;
}

PlannerCore::PlannerCore(PlannerConfig config)
: config_(config)
{
}

const PlannerConfig & PlannerCore::config() const noexcept
{
  return config_;
}

void PlannerCore::setConfig(const PlannerConfig & config)
{
  config_ = config;
}

PlanResult PlannerCore::plan(const GridMap & map, const PlanRequest & request) const
{
  const auto started_at = std::chrono::steady_clock::now();
  PlanResult result;

  if (map.empty()) {
    result.status = "empty_map";
    return result;
  }
  if (!map.isFree(request.start) || !map.isFree(request.goal)) {
    result.status = "invalid_start_or_goal";
    return result;
  }
  if (request.start == request.goal) {
    result.success = true;
    result.path = {request.start};
    result.status = "already_at_goal";
    result.elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started_at);
    return result;
  }

  std::priority_queue<SearchState, std::vector<SearchState>, SearchStateGreater> open;
  std::unordered_map<CellTime, double, CellTimeHash> best_cost;
  std::unordered_map<CellTime, CellTime, CellTimeHash> parent;
  const SearchState start_state{request.start, 0, 0.0, heuristic(request.start, request.goal)};
  open.push(start_state);
  best_cost[toCellTime(start_state)] = 0.0;

  const std::vector<Position> moves = config_.allow_wait_action
    ? std::vector<Position>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {0, 0}}
    : std::vector<Position>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    if (result.expansions >= config_.max_expansions) {
      result.status = "max_expansions_exceeded";
      break;
    }
    if ((result.expansions & 0xFFU) == 0U) {
      const auto elapsed = std::chrono::steady_clock::now() - started_at;
      if (elapsed > config_.planning_timeout) {
        result.status = "planning_timeout";
        break;
      }
    }

    const SearchState current = open.top();
    open.pop();

    const auto current_key = toCellTime(current);
    const auto best_it = best_cost.find(current_key);
    if (best_it != best_cost.end() && current.g > best_it->second + 1e-9) {
      continue;
    }

    ++result.expansions;
    if (current.position == request.goal) {
      result.success = true;
      result.cost = current.g;
      result.path = reconstructPath(current_key, parent);
      result.status = "success";
      break;
    }

    for (const auto & move : moves) {
      const Position next{current.position.x + move.x, current.position.y + move.y};
      if (!map.isFree(next)) {
        continue;
      }

      const int next_time = current.time + 1;
      const double next_g = current.g + transitionCost(request, next, next_time);
      const CellTime next_key{next.x, next.y, next_time};
      const auto next_best_it = best_cost.find(next_key);
      if (next_best_it != best_cost.end() && next_g >= next_best_it->second - 1e-9) {
        continue;
      }

      best_cost[next_key] = next_g;
      parent[next_key] = current_key;
      open.push(SearchState{next, next_time, next_g, heuristic(next, request.goal)});
    }
  }

  if (result.status == "not_started") {
    result.status = "unreachable";
  }
  result.elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - started_at);
  return result;
}

double PlannerCore::heuristic(const Position & a, const Position & b) const noexcept
{
  return static_cast<double>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
}

double PlannerCore::transitionCost(
  const PlanRequest & request,
  const Position & to,
  int arrival_time) const
{
  double cost = 1.0;

  const auto dynamic_it = request.dynamic_occupancy.find(to);
  if (dynamic_it != request.dynamic_occupancy.end()) {
    cost += config_.dynamic_occupancy_weight * dynamic_it->second;
  }

  for (int offset = -config_.conflict_time_window; offset <= config_.conflict_time_window; ++offset) {
    const int time = arrival_time + offset;
    if (time < 0) {
      continue;
    }

    const auto planned_it = request.planned_occupancy.find(CellTime{to.x, to.y, time});
    if (planned_it == request.planned_occupancy.end()) {
      continue;
    }

    const double occupancy = planned_it->second;
    if (offset == 0) {
      cost += config_.planned_conflict_weight * occupancy * occupancy;
    } else {
      cost += 0.5 * config_.planned_conflict_weight * occupancy *
        std::pow(0.8, static_cast<double>(std::abs(offset)));
    }
  }

  return cost;
}

}  // namespace smapo_multi_agent_planner

