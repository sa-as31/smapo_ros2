# SMAPO

This project contains multi-agent path planning, training, and ROS2 communication integration prototype code. The main stack is based on Sample Factory + Pogema, with additional C++ planning modules and a ROS2/C++17 multi-agent communication example.

The repository includes:

- Python multi-agent simulation and training entry points: `main.py`, `main_gpu.py`
- C++/pybind11 planning extensions: `planner/Static_A`, `planner/LB_A`
- Web demo services and frontends: `web_demo/`, `web_frontend*/`
- ROS2/C++17 multi-agent planning prototype package: `ros2_ws/src/smapo_multi_agent_planner`

Chinese documentation: [README.md](README.md)

## 1. Requirements

- Python: `3.9`
- OS: Linux/macOS. Docker is recommended.
- Optional GPU: NVIDIA + nvidia-container-toolkit, required only for Docker GPU runs.
- Optional ROS2: Humble/Iron/Jazzy or another ROS2 distribution with `ament_cmake` support.

## 2. Linux + NVIDIA One-Command Run

This option only applies to Linux machines with an installed NVIDIA driver.

```bash
bash scripts/run_gpu_one_click.sh
```

Example with custom training arguments:

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

The script automatically:

1. Checks NVIDIA driver availability with `nvidia-smi`
2. Builds the GPU image from `Dockerfile.gpu`
3. Verifies CUDA and PyTorch inside the container with `scripts/verify_gpu.py`
4. Starts training through `main_gpu.py`

## 3. Local Run Without Docker

### 3.1 Install Dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
pip install pybind11 cppimport
```

### 3.2 Build Planner Extensions

```bash
python -c "import cppimport; cppimport.imp('planner.LB_A.planner')"
python -c "import cppimport; cppimport.imp('planner.Static_A.planner')"
```

### 3.3 CPU Training

Run this first to validate the environment:

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

### 3.4 GPU Training Entry Point

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

`main_gpu.py` sets `global_settings.device` to `gpu` by default. If CUDA is unavailable, it logs the condition and falls back to CPU.

## 4. Docker Guide

### 4.1 Build the CPU Image

```bash
docker build -t smapo:local .
```

### 4.2 Run CPU Training in Docker

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

### 4.3 Verify the Native PyOctoMap 3D Map Backend

```bash
docker build -t smapo:pyoctomap-test .
docker run --rm smapo:pyoctomap-test python scripts/smoke_pyoctomap_env.py
```

Notes:

- The smoke test enables `height_levels=4`, `native_3d_obstacles=True`, and `obstacle_backend=pyoctomap`
- The generated global obstacle map is a native `4 x 16 x 16` 3D voxel grid, not a 2D obstacle map copied across layers

### 4.4 Build the GPU Image Manually

```bash
docker build -f Dockerfile.gpu -t smapo:gpu .
```

### 4.5 Run GPU Training in Docker

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

### 4.6 Run with docker compose GPU

```bash
docker compose -f docker-compose.gpu.yml up --build
```

### 4.7 Run with docker compose CPU

```bash
docker compose up --build
```

## 5. ROS2/C++17 Multi-Agent Planning Prototype

The ROS2 prototype package is located at:

```text
ros2_ws/src/smapo_multi_agent_planner
```

The package is structured as a standard `ament_cmake` C++ package and separates core algorithms from ROS2 communication:

- `PlannerCore`: C++17 A* planner over occupancy grids with static obstacles, dynamic occupancy costs, temporal conflict penalties, expansion limits, and planning timeout checks
- `TaskScheduler`: Greedy multi-agent task assignment that skips busy agents and balances distance against task priority
- `RealtimeMonitor`: Lightweight metrics collector for planning latency, failure count, timeout count, and health status
- `multi_agent_planner_node`: `rclcpp::Node` that subscribes to map, agent state, and task request topics, then publishes planned paths and diagnostics

The ROS2 node uses standard message types and does not require a separate custom message package.

### 5.1 ROS2 Topics

Subscriptions:

- `/smapo/map` (`nav_msgs/msg/OccupancyGrid`): 2D occupancy grid. Values above `50` and unknown cells are treated as blocked
- `/smapo/agent_states` (`std_msgs/msg/String`): JSON array with agent state objects
- `/smapo/task_requests` (`std_msgs/msg/String`): JSON array with task request objects

Publications:

- `/smapo/planned_paths` (`std_msgs/msg/String`): JSON array of assigned paths
- `/smapo/planner_diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Planning loop latency and stability metrics

Agent state payload example:

```json
[
  {"agent_id":"uav_01","x":2,"y":4,"busy":false,"timestamp_ms":1710000000000},
  {"agent_id":"uav_02","x":8,"y":3,"busy":true,"timestamp_ms":1710000000000}
]
```

Task request payload example:

```json
[
  {"task_id":"inspect_a","goal_x":12,"goal_y":8,"priority":3},
  {"task_id":"inspect_b","goal_x":4,"goal_y":14,"priority":1}
]
```

Planned path payload example:

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

### 5.2 ROS2 Parameters

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `planning_period_ms` | `100` | Timer period for dispatch and replanning |
| `planning_timeout_ms` | `30` | Per-agent planning timeout budget |
| `max_expansions` | `20000` | Maximum A* state expansions per request |
| `dynamic_occupancy_weight` | `2.0` | Cost multiplier for currently occupied cells |
| `conflict_time_window` | `2` | Temporal window for planned path conflict penalties |

### 5.3 ROS2 Build and Test

From the repository root:

```bash
cd ros2_ws
colcon build --packages-select smapo_multi_agent_planner
colcon test --packages-select smapo_multi_agent_planner
source install/setup.bash
ros2 run smapo_multi_agent_planner multi_agent_planner_node
```

The ROS2 package is an independent prototype layer. It can be inspected separately from the Python training workflow and does not require changing the current simulation entry points.

## 6. Repository Layout

```text
planner/
  Static_A/                 # Existing pybind11 C++ grid planner
  LB_A/                     # Existing pybind11 planner with temporal congestion cost
planning/
  replan_algo.py            # Python randomized A* replanner used by inference flows
pogema/, sample_factory/    # Multi-agent simulation and training stack
web_demo/, web_frontend*/   # Demo services and frontends
ros2_ws/src/smapo_multi_agent_planner/
  include/                  # ROS-independent C++ planning, scheduling, monitoring APIs
  src/                      # Core implementations and ROS2 node
  test/                     # gtest-based unit test scenarios
```

## 7. FAQ

1. Gym deprecation warnings appear in logs

This is an upstream dependency warning and does not affect the current training workflow.

2. Error: `Target num agents must be divisible by num agents`

Make sure `target_num_agents % num_agents == 0`, for example `256 % 128 == 0`.

3. Where are training outputs stored?

The default output directory is `results/train_dir`.

4. Can CUDA be tested on macOS?

No. macOS, including Apple Silicon, does not provide the NVIDIA CUDA runtime. Use the Linux + NVIDIA one-command script for GPU runs.

5. What if ROS2 is not installed in the current environment?

The Python simulation and training workflows are unaffected. `ros2_ws/src/smapo_multi_agent_planner` is an independent ROS2/C++17 prototype package and should be built in an environment with ROS2 and `colcon`.

