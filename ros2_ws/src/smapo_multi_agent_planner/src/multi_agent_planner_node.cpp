#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "smapo_multi_agent_planner/planner_core.hpp"
#include "smapo_multi_agent_planner/realtime_monitor.hpp"
#include "smapo_multi_agent_planner/task_scheduler.hpp"

namespace smapo_multi_agent_planner
{
namespace
{

std::vector<std::string> splitJsonObjects(const std::string & payload)
{
  std::vector<std::string> objects;
  std::size_t begin = std::string::npos;
  int depth = 0;

  for (std::size_t i = 0; i < payload.size(); ++i) {
    if (payload[i] == '{') {
      if (depth == 0) {
        begin = i;
      }
      ++depth;
    } else if (payload[i] == '}') {
      --depth;
      if (depth == 0 && begin != std::string::npos) {
        objects.push_back(payload.substr(begin, i - begin + 1U));
        begin = std::string::npos;
      }
    }
  }

  return objects;
}

std::optional<std::string> stringField(const std::string & object, const std::string & key)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  if (!std::regex_search(object, match, pattern)) {
    return std::nullopt;
  }
  return match[1].str();
}

std::optional<int> intField(const std::string & object, const std::string & key)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (!std::regex_search(object, match, pattern)) {
    return std::nullopt;
  }
  return std::stoi(match[1].str());
}

std::optional<std::int64_t> int64Field(const std::string & object, const std::string & key)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (!std::regex_search(object, match, pattern)) {
    return std::nullopt;
  }
  return std::stoll(match[1].str());
}

bool boolField(const std::string & object, const std::string & key, bool default_value)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false|0|1)");
  std::smatch match;
  if (!std::regex_search(object, match, pattern)) {
    return default_value;
  }
  const auto value = match[1].str();
  return value == "true" || value == "1";
}

std::string escapeJson(const std::string & value)
{
  std::ostringstream out;
  for (const char ch : value) {
    if (ch == '"' || ch == '\\') {
      out << '\\';
    }
    out << ch;
  }
  return out.str();
}

std::string toFixed(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

std::vector<AgentState> parseAgents(const std::string & payload)
{
  std::vector<AgentState> agents;
  for (const auto & object : splitJsonObjects(payload)) {
    const auto agent_id = stringField(object, "agent_id");
    const auto x = intField(object, "x");
    const auto y = intField(object, "y");
    if (!agent_id || !x || !y) {
      continue;
    }

    AgentState agent;
    agent.agent_id = *agent_id;
    agent.position = Position{*x, *y};
    agent.busy = boolField(object, "busy", false);
    agent.timestamp_ms = int64Field(object, "timestamp_ms").value_or(0);
    agents.push_back(agent);
  }
  return agents;
}

std::vector<TaskRequest> parseTasks(const std::string & payload)
{
  std::vector<TaskRequest> tasks;
  for (const auto & object : splitJsonObjects(payload)) {
    const auto task_id = stringField(object, "task_id");
    const auto goal_x = intField(object, "goal_x");
    const auto goal_y = intField(object, "goal_y");
    if (!task_id || !goal_x || !goal_y) {
      continue;
    }

    TaskRequest task;
    task.task_id = *task_id;
    task.goal = Position{*goal_x, *goal_y};
    task.priority = intField(object, "priority").value_or(0);
    tasks.push_back(task);
  }
  return tasks;
}

const AgentState * findAgent(const std::vector<AgentState> & agents, const std::string & agent_id)
{
  const auto it = std::find_if(agents.begin(), agents.end(), [&](const AgentState & agent) {
    return agent.agent_id == agent_id;
  });
  return it == agents.end() ? nullptr : &(*it);
}

const TaskRequest * findTask(const std::vector<TaskRequest> & tasks, const std::string & task_id)
{
  const auto it = std::find_if(tasks.begin(), tasks.end(), [&](const TaskRequest & task) {
    return task.task_id == task_id;
  });
  return it == tasks.end() ? nullptr : &(*it);
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

class MultiAgentPlannerNode final : public rclcpp::Node
{
public:
  MultiAgentPlannerNode()
  : Node("multi_agent_planner_node")
  {
    PlannerConfig planner_config;
    planner_config.max_expansions = static_cast<std::size_t>(
      declare_parameter<int>("max_expansions", 20000));
    planner_config.dynamic_occupancy_weight =
      declare_parameter<double>("dynamic_occupancy_weight", 2.0);
    planner_config.conflict_time_window = declare_parameter<int>("conflict_time_window", 2);
    planner_config.planning_timeout = std::chrono::milliseconds(
      declare_parameter<int>("planning_timeout_ms", 30));
    planner_.setConfig(planner_config);
    monitor_ = RealtimeMonitor(planner_config.planning_timeout);

    const auto planning_period = std::chrono::milliseconds(
      declare_parameter<int>("planning_period_ms", 100));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/smapo/map", rclcpp::QoS(1).transient_local(),
      std::bind(&MultiAgentPlannerNode::onMap, this, std::placeholders::_1));
    agents_sub_ = create_subscription<std_msgs::msg::String>(
      "/smapo/agent_states", 10,
      std::bind(&MultiAgentPlannerNode::onAgents, this, std::placeholders::_1));
    tasks_sub_ = create_subscription<std_msgs::msg::String>(
      "/smapo/task_requests", 10,
      std::bind(&MultiAgentPlannerNode::onTasks, this, std::placeholders::_1));

    paths_pub_ = create_publisher<std_msgs::msg::String>("/smapo/planned_paths", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/smapo/planner_diagnostics", 10);

    timer_ = create_wall_timer(planning_period, std::bind(&MultiAgentPlannerNode::onTimer, this));
  }

private:
  void onMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::vector<std::int8_t> cells;
    cells.reserve(msg->data.size());
    for (const auto value : msg->data) {
      cells.push_back(static_cast<std::int8_t>(value));
    }
    map_ = GridMap(
      static_cast<int>(msg->info.width),
      static_cast<int>(msg->info.height),
      std::move(cells));
    have_map_ = true;
  }

  void onAgents(std_msgs::msg::String::SharedPtr msg)
  {
    agents_ = parseAgents(msg->data);
  }

  void onTasks(std_msgs::msg::String::SharedPtr msg)
  {
    tasks_ = parseTasks(msg->data);
  }

  void onTimer()
  {
    monitor_.recordCycle(tasks_.size());

    if (!have_map_ || agents_.empty() || tasks_.empty()) {
      publishDiagnostics();
      return;
    }

    const auto assignments = scheduler_.assign(agents_, tasks_);
    std::unordered_map<CellTime, double, CellTimeHash> planned_occupancy;
    std::ostringstream payload;
    payload << "[";

    bool first_result = true;
    for (const auto & assignment : assignments) {
      const auto * agent = findAgent(agents_, assignment.agent_id);
      const auto * task = findTask(tasks_, assignment.task_id);
      if (agent == nullptr || task == nullptr) {
        continue;
      }

      PlanRequest request;
      request.start = agent->position;
      request.goal = task->goal;
      request.planned_occupancy = planned_occupancy;
      for (const auto & other_agent : agents_) {
        if (other_agent.agent_id != agent->agent_id) {
          request.dynamic_occupancy[other_agent.position] += 1.0;
        }
      }

      const auto result = planner_.plan(map_, request);
      monitor_.recordPlan(
        result.elapsed,
        result.success,
        result.status == "planning_timeout");

      if (result.success) {
        for (std::size_t t = 0; t < result.path.size(); ++t) {
          const auto & cell = result.path[t];
          planned_occupancy[CellTime{cell.x, cell.y, static_cast<int>(t)}] += 1.0;
        }
      }

      if (!first_result) {
        payload << ",";
      }
      first_result = false;
      appendPlanJson(payload, *agent, *task, result);
    }

    payload << "]";
    std_msgs::msg::String paths_msg;
    paths_msg.data = payload.str();
    paths_pub_->publish(paths_msg);
    publishDiagnostics();
  }

  void appendPlanJson(
    std::ostringstream & out,
    const AgentState & agent,
    const TaskRequest & task,
    const PlanResult & result) const
  {
    out << "{";
    out << "\"agent_id\":\"" << escapeJson(agent.agent_id) << "\",";
    out << "\"task_id\":\"" << escapeJson(task.task_id) << "\",";
    out << "\"success\":" << (result.success ? "true" : "false") << ",";
    out << "\"status\":\"" << escapeJson(result.status) << "\",";
    out << "\"cost\":" << toFixed(result.cost) << ",";
    out << "\"planning_time_ms\":" << toFixed(static_cast<double>(result.elapsed.count()) / 1000.0) << ",";
    out << "\"expansions\":" << result.expansions << ",";
    out << "\"path\":[";
    for (std::size_t i = 0; i < result.path.size(); ++i) {
      if (i > 0U) {
        out << ",";
      }
      out << "{\"x\":" << result.path[i].x << ",\"y\":" << result.path[i].y << "}";
    }
    out << "]}";
  }

  void publishDiagnostics()
  {
    const auto metrics = monitor_.metrics();
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "smapo_multi_agent_planner";
    status.hardware_id = "software_planner";
    status.level = metrics.healthy ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = metrics.healthy ? "planner healthy" : "planner degraded";
    status.values.push_back(keyValue("cycles", std::to_string(metrics.cycles)));
    status.values.push_back(keyValue("successful_plans", std::to_string(metrics.successful_plans)));
    status.values.push_back(keyValue("failed_plans", std::to_string(metrics.failed_plans)));
    status.values.push_back(keyValue("timeout_plans", std::to_string(metrics.timeout_plans)));
    status.values.push_back(keyValue("last_latency_ms", toFixed(metrics.last_latency_ms)));
    status.values.push_back(keyValue("average_latency_ms", toFixed(metrics.average_latency_ms)));
    status.values.push_back(keyValue("max_latency_ms", toFixed(metrics.max_latency_ms)));
    status.values.push_back(keyValue("task_queue_size", std::to_string(metrics.task_queue_size)));

    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }

  PlannerCore planner_;
  TaskScheduler scheduler_;
  RealtimeMonitor monitor_;
  GridMap map_;
  bool have_map_{false};
  std::vector<AgentState> agents_;
  std::vector<TaskRequest> tasks_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agents_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tasks_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr paths_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace smapo_multi_agent_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smapo_multi_agent_planner::MultiAgentPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
