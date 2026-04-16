# SMAPO

本项目为多智能体路径规划、训练与 ROS2 通信集成原型代码，主体基于 Sample Factory + Pogema，并补充了 C++ 路径规划模块和 ROS2/C++17 多智能体通信示例。

当前仓库包含：

- Python 多智能体仿真与训练入口：`main.py`、`main_gpu.py`
- C++/pybind11 路径规划扩展：`planner/Static_A`、`planner/LB_A`
- Web 演示服务和前端：`web_demo/`、`web_frontend*/`
- ROS2/C++17 多智能体路径规划原型包：`ros2_ws/src/smapo_multi_agent_planner`

英文文档见：[README_EN.md](README_EN.md)

## 1. 环境要求

- Python: `3.9`
- 操作系统: Linux/macOS（推荐在 Docker 中运行）
- 可选 GPU: NVIDIA + nvidia-container-toolkit（仅 Docker GPU 运行需要）
- 可选 ROS2: Humble/Iron/Jazzy 等支持 `ament_cmake` 的 ROS2 发行版

## 2. Linux + NVIDIA 一键运行（推荐）

仅适用于 Linux + NVIDIA 驱动已安装的机器。

```bash
bash scripts/run_gpu_one_click.sh
```

自定义训练参数示例：

```bash
bash scripts/run_gpu_one_click.sh \
  --train_for_seconds=600 \
  --num_workers=4 \
  --num_envs_per_worker=2 \
  --worker_num_splits=1 \
  --target_num_agents=256 \
  --num_agents=128 \
  --use_wandb=False
```

该脚本会自动执行：

1. 检查主机 NVIDIA 驱动可用性（`nvidia-smi`）
2. 构建 GPU 镜像（`Dockerfile.gpu`）
3. 在容器内检查 CUDA 与 PyTorch（`scripts/verify_gpu.py`）
4. 启动 `main_gpu.py` 训练

## 3. 本地运行（不使用 Docker）

### 3.1 安装依赖

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
pip install pybind11 cppimport
```

### 3.2 编译规划器扩展

```bash
python -c "import cppimport; cppimport.imp('planner.LB_A.planner')"
python -c "import cppimport; cppimport.imp('planner.Static_A.planner')"
```

### 3.3 CPU 训练（推荐先跑通）

```bash
python main.py \
  --train_for_seconds=30 \
  --num_workers=2 \
  --num_envs_per_worker=2 \
  --worker_num_splits=1 \
  --target_num_agents=256 \
  --num_agents=128 \
  --use_wandb=False
```

### 3.4 GPU 训练入口

```bash
python main_gpu.py \
  --train_for_seconds=30 \
  --num_workers=2 \
  --num_envs_per_worker=2 \
  --worker_num_splits=1 \
  --target_num_agents=256 \
  --num_agents=128 \
  --use_wandb=False
```

说明：`main_gpu.py` 默认将 `global_settings.device` 设为 `gpu`。若机器无 CUDA，会在日志中提示并自动切换 CPU。

## 4. Docker 指南

### 4.1 CPU 镜像构建

```bash
docker build -t smapo:local .
```

### 4.2 在 Docker 中运行 CPU 训练

```bash
docker run --rm smapo:local sh -lc "python main.py \
  --train_for_seconds=30 \
  --num_workers=2 \
  --num_envs_per_worker=2 \
  --worker_num_splits=1 \
  --target_num_agents=256 \
  --num_agents=128 \
  --use_wandb=False"
```

### 4.3 在 Docker 中验证 PyOctoMap 原生 3D 地图后端

```bash
docker build -t smapo:pyoctomap-test .
docker run --rm smapo:pyoctomap-test python scripts/smoke_pyoctomap_env.py
```

说明：

- 该 smoke 测试会启用 `height_levels=4`、`native_3d_obstacles=True`、`obstacle_backend=pyoctomap`
- 生成的全局障碍是原生 `4 x 16 x 16` 三维体素障碍图，不再是二维障碍按层复制

### 4.4 手动构建 GPU 镜像

```bash
docker build -f Dockerfile.gpu -t smapo:gpu .
```

### 4.5 在 Docker 中运行 GPU 训练

```bash
docker run --rm --gpus all --ipc=host smapo:gpu bash -lc "python scripts/verify_gpu.py && python main_gpu.py \
  --train_for_seconds=300 \
  --num_workers=2 \
  --num_envs_per_worker=2 \
  --worker_num_splits=1 \
  --target_num_agents=256 \
  --num_agents=128 \
  --use_wandb=False"
```

### 4.6 使用 docker compose（GPU）

```bash
docker compose -f docker-compose.gpu.yml up --build
```

### 4.7 使用 docker compose（CPU）

```bash
docker compose up --build
```

## 5. ROS2/C++17 多智能体规划原型

ROS2 原型包位于：

```text
ros2_ws/src/smapo_multi_agent_planner
```

该包采用标准 `ament_cmake` C++ 包结构，并将核心算法与 ROS2 通信层分离：

- `PlannerCore`：基于 C++17 的栅格 A* 规划器，支持静态障碍、动态占用代价、时空冲突惩罚、扩展数限制和规划超时检查
- `TaskScheduler`：贪心式多智能体任务分配模块，跳过 busy 状态智能体，并在距离代价和任务优先级之间做权衡
- `RealtimeMonitor`：轻量级指标收集器，用于记录规划延迟、失败次数、超时次数和健康状态
- `multi_agent_planner_node`：`rclcpp::Node`，订阅地图、智能体状态和任务请求 topic，发布规划路径和诊断信息

ROS2 节点使用标准消息类型，不额外定义自定义 message 包。

### 5.1 ROS2 topic 接口

订阅：

- `/smapo/map` (`nav_msgs/msg/OccupancyGrid`)：二维占用栅格地图。大于 `50` 的值和未知栅格会被视为不可通行
- `/smapo/agent_states` (`std_msgs/msg/String`)：JSON 数组，包含智能体状态对象
- `/smapo/task_requests` (`std_msgs/msg/String`)：JSON 数组，包含任务请求对象

发布：

- `/smapo/planned_paths` (`std_msgs/msg/String`)：JSON 数组，包含已分配的路径结果
- `/smapo/planner_diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)：规划循环延迟和稳定性指标

智能体状态 payload 示例：

```json
[
  {"agent_id":"uav_01","x":2,"y":4,"busy":false,"timestamp_ms":1710000000000},
  {"agent_id":"uav_02","x":8,"y":3,"busy":true,"timestamp_ms":1710000000000}
]
```

任务请求 payload 示例：

```json
[
  {"task_id":"inspect_a","goal_x":12,"goal_y":8,"priority":3},
  {"task_id":"inspect_b","goal_x":4,"goal_y":14,"priority":1}
]
```

路径结果 payload 示例：

```json
[
  {
    "agent_id":"uav_01",
    "task_id":"inspect_a",
    "success":true,
    "cost":18.5,
    "planning_time_ms":2.4,
    "path":[{"x":2,"y":4},{"x":3,"y":4},{"x":4,"y":4}]
  }
]
```

### 5.2 ROS2 参数

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `planning_period_ms` | `100` | 调度和重规划定时器周期 |
| `planning_timeout_ms` | `30` | 单个智能体规划的超时预算 |
| `max_expansions` | `20000` | 单次请求允许的最大 A* 状态扩展数 |
| `dynamic_occupancy_weight` | `2.0` | 当前占用栅格的代价权重 |
| `conflict_time_window` | `2` | 规划路径冲突惩罚的时间窗口 |

### 5.3 ROS2 构建和测试

从仓库根目录执行：

```bash
cd ros2_ws
colcon build --packages-select smapo_multi_agent_planner
colcon test --packages-select smapo_multi_agent_planner
source install/setup.bash
ros2 run smapo_multi_agent_planner multi_agent_planner_node
```

说明：ROS2 包是一个独立原型层，可以脱离现有 Python 训练流程进行检查，不需要修改当前仿真入口。

## 6. 仓库结构

```text
planner/
  Static_A/                 # 已有 pybind11 C++ 栅格规划器
  LB_A/                     # 已有带时序拥塞代价的 pybind11 规划器
planning/
  replan_algo.py            # 推理流程使用的 Python 随机化 A* 重规划器
pogema/, sample_factory/    # 多智能体仿真和训练栈
web_demo/, web_frontend*/   # 演示服务和前端
ros2_ws/src/smapo_multi_agent_planner/
  include/                  # 不依赖 ROS 的 C++ 规划、调度、监控接口
  src/                      # 核心实现和 ROS2 节点
  test/                     # 基于 gtest 的单元测试场景
```

## 7. 常见问题

1. 日志出现 Gym 弃用警告

这是上游依赖提示，不影响本项目当前训练流程。

2. 报错 `Target num agents must be divisible by num agents`

需要保证 `target_num_agents % num_agents == 0`，例如：`256 % 128 == 0`。

3. 训练结果在哪里

默认输出目录在 `results/train_dir`。

4. 在 macOS 上能否测试 CUDA？

不能。macOS（包括 Apple Silicon）没有 NVIDIA CUDA 运行时。请在 Linux + NVIDIA 环境使用上面的 GPU 一键脚本。

5. 当前环境没有 ROS2 怎么办？

不影响 Python 仿真和训练流程。`ros2_ws/src/smapo_multi_agent_planner` 是独立 ROS2/C++17 原型包，需要在安装 ROS2 和 `colcon` 的环境中构建。

