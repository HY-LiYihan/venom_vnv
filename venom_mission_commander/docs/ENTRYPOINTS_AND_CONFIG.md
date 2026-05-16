# Mission Commander Entrypoints And Config

这份文档集中记录 `venom_mission_commander` 当前可用的运行入口、参数、mission YAML 结构，以及本机、Docker、仿真和真机之间的边界。它只描述现状和推荐使用方式；长期结构拆分建议见 `INTEGRATION_ROADMAP.md`。

## 入口总览

`venom_mission_commander` 当前只有一个稳定可执行入口：

```bash
ros2 run venom_mission_commander mission_commander
```

它对应 `setup.py` 中的 console script：

```text
mission_commander = venom_mission_commander.mission_commander:main
```

所有 launch 文件最终也只是启动这个同一个 ROS 2 node：`MissionCommander`，节点名为 `mission_commander`。

| 入口 | 位置 | 用途 | 是否启动 Gazebo/Nav2/RViz |
| --- | --- | --- | --- |
| `mission_commander` | `setup.py` console script | 最底层可执行入口 | 否 |
| `mission_commander.launch.py` | `launch/mission_commander.launch.py` | 通用 node wrapper，可 mock，可 Nav2 | 否 |
| `mission_commander_nav2_sim.launch.py` | `launch/mission_commander_nav2_sim.launch.py` | Nav2 仿真快捷 wrapper，默认 RMUL mission | 否 |
| `bringup_sim.launch.py` | `simulation/venom_nav_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` | 启动仿真、定位、Nav2、RViz | 是 |
| `bringup_real.launch.py` | `simulation/venom_nav_simulation/src/rm_nav_bringup/launch/bringup_real.launch.py` | 启动真机传感器、定位、Nav2 | 是 |
| `run_humble_sim.sh` | `simulation/venom_nav_simulation/docker/run_humble_sim.sh` | 本机进入 Humble Docker 测试环境 | 否，只进入容器并做 bootstrap |

关键边界：

- `mission_commander` 负责读 mission、调用导航适配层、执行 task plugins、记录状态。
- `mission_commander*.launch.py` 只负责给 commander node 传参。
- `bringup_sim.launch.py` / `bringup_real.launch.py` 负责把机器人、地图、定位、Nav2、RViz 等系统先启动起来。
- Docker 脚本只负责提供 Humble 仿真测试环境，不是 mission commander 的正式运行入口。

## Commander 运行参数

这些参数由 `MissionCommander.__init__()` 声明，并由 `mission_commander.launch.py` 透传。

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `mission_config` | string | 包内 `config/simple_mission.yaml` | mission YAML 的绝对路径或可解析路径 |
| `use_nav` | bool | `false` | `false` 使用 mock 导航；`true` 使用 Nav2 |
| `mock_nav_delay_sec` | float | `0.5` | mock 导航每个 waypoint 的模拟等待时间 |
| `nav2_wait_mode` | string | `bt_navigator` | Nav2 ready 等待方式，支持 `bt_navigator` 或 `full` |
| `navigator_ready_timeout_sec` | float | `30.0` | 启动检查等待 mock/Nav2 navigator ready 的最长时间 |
| `nav_feedback_log_interval_sec` | float | `5.0` | Nav2 周期反馈日志间隔；设为 `0` 可关闭 `[NAV2] Still navigating` |
| `log_state_transitions` | bool | `false` | 是否打印 `MissionManager` 底层状态迁移日志 |
| `use_sim_time` | bool | `false` | 仿真时设为 `true`，真机时设为 `false` |

参数使用建议：

- 常规比赛/调试看 `[STARTUP]`、`[STATUS]`、`[NAV2]` 和 `[SUMMARY]` 即可。
- 如果 `[NAV2] Still navigating` 太频繁，调大 `nav_feedback_log_interval_sec`；如果不想要周期反馈，设为 `0`。
- 如果要排查状态机内部迁移，再临时打开 `log_state_transitions:=true`，平时保持默认关闭。

`mission_commander_nav2_sim.launch.py` 是一个固定参数的快捷 wrapper：

- 默认 `mission_config` 为包内 `config/rmul_sim_mission.yaml`。
- 固定 `use_nav: true`。
- 固定 `mock_nav_delay_sec: 0.0`。
- 默认 `nav2_wait_mode:=bt_navigator`。
- 默认 `navigator_ready_timeout_sec:=30.0`。
- 默认 `nav_feedback_log_interval_sec:=5.0`。
- 默认 `log_state_transitions:=false`。
- 默认 `use_sim_time:=true`。

## 运行日志结构

Commander 日志按来源分成四类，默认优先保证比赛现场和调试终端可读：

| 前缀 | 来源 | 什么时候出现 | 主要用途 |
| --- | --- | --- | --- |
| `[STARTUP]` | `StartupChecker` / `MissionStatusReporter` | mission 进入 `RUNNING` 前 | 看配置、插件和 navigator ready 检查是否通过 |
| `[STATUS]` | `MissionStatusReporter` | mission / waypoint / navigation / task 关键事件 | 看当前阶段、当前 waypoint、当前 task、导航尝试和最近任务结果 |
| `[NAV2]` | `Nav2WaypointNavigator` | Nav2 目标下发、周期反馈、超时、取消、恢复 | 看导航层是否还在走、是否超时、是否触发恢复 |
| `[SUMMARY]` | `MissionStatusReporter` | 程序退出前 | 看最终状态、完成进度、失败原因和最后一次关键结果 |

`[STATUS]` 不是周期性状态流，而是关键事件触发的完整快照。典型输出如下：

```text
[STARTUP] PASS mission_config
	message: mission config is valid

[STATUS] task_completed
	state: executing_tasks | phase: executing_task | startup_checks: passed
	waypoint: 2/8:task_point_1_pick(operation_stop)
	task: 2:grasp_item_at_point_1
	nav: attempt=1/2 timeout=35.0s last_success=True
	last_task: grasp_item_at_point_1:True:grasp succeeded

[NAV2] Still navigating: elapsed=7.0s, nav_time=2.2s, eta=2.3s

[SUMMARY] mission
	id: competition_10x6_nav2_mission_commander
	state: completed
	phase: completed
	progress: 8/8 waypoint(s)
	startup_checks: passed
	last_waypoint: return_start_area
	last_task: finish_delay:True:waited 0.50s
	navigation: waypoint=return_start_area | attempt=1 | success=True
```

字段含义：

- `state`：mission 顶层状态，例如 `running`、`navigating`、`executing_tasks`、`completed`、`failed`。
- `phase`：当前细分阶段，例如 `startup_checks`、`navigating`、`navigation_skipped`、`executing_task`、`waypoint_done`。
- `waypoint`：格式为 `当前序号/总数:名称(kind)`，其中 `kind` 来自 YAML 的 waypoint 语义标签。
- `task`：当前正在执行的 task；`-` 表示当前没有 task。
- `nav`：当前导航尝试次数、该 waypoint 的导航超时和最近导航结果。
- `last_task`：最近一次完成或失败的 task 结果，不一定等于当前正在执行的 task。

默认不会打印 `MissionManager` 的底层状态迁移日志，但状态历史仍记录在内存中。需要看到 `Mission state: old -> new` 时，启动时追加：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  log_state_transitions:=true
```

## 常用启动矩阵

### 1. Mock 模式

不依赖 Gazebo、Nav2、地图、视觉、语音或机械臂节点，适合验证任务编排流程。

```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash

ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false
```

等价的直接 run 方式：

```bash
ros2 run venom_mission_commander mission_commander \
  --ros-args \
  -p mission_config:=/path/to/simple_mission.yaml \
  -p use_nav:=false
```

### 2. 本机 RMUL 仿真 Nav2

先启动仿真导航栈，再启动 commander。`mission_commander_nav2_sim.launch.py` 不会替你启动 Gazebo、Nav2 或 RViz。

终端 1：

```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL \
  mode:=nav \
  lio:=pointlio \
  localization:=slam_toolbox \
  lio_rviz:=False \
  nav_rviz:=True
```

终端 2：

```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash

ros2 launch venom_mission_commander mission_commander_nav2_sim.launch.py
```

等价显式写法：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=true \
  use_sim_time:=true \
  nav2_wait_mode:=bt_navigator \
  mission_config:=/path/to/rmul_sim_mission.yaml
```

### 3. Docker Humble mock 模式

Docker 工作流位于 `simulation/venom_nav_simulation`，用于在 Ubuntu 24.04 / Jazzy 主机上测试 Ubuntu 22.04 / Humble 仿真栈。

宿主机：

```bash
cd /path/to/venom_vnv/simulation/venom_nav_simulation
./docker/run_humble_sim.sh
```

进入容器后：

```bash
source /opt/ros/humble/setup.bash
source /opt/venom_nav_ws/install/setup.bash

ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false
```

当前 `docker/bootstrap_humble_sim.sh` 已自动把 `venom_mission_commander` symlink 到 `/opt/venom_nav_ws/src` 并参与构建；不需要手动执行 `ln -sfn`。

### 4. Docker RMUL 仿真 Nav2

终端 1：启动 Gazebo、定位、Nav2 和 RViz。

```bash
source /opt/ros/humble/setup.bash
source /opt/venom_nav_ws/install/setup.bash

ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL \
  mode:=nav \
  lio:=pointlio \
  localization:=slam_toolbox \
  lio_rviz:=False \
  nav_rviz:=True
```

终端 2：启动 commander。

```bash
source /opt/ros/humble/setup.bash
source /opt/venom_nav_ws/install/setup.bash

ros2 launch venom_mission_commander mission_commander_nav2_sim.launch.py
```

### 5. Docker competition_10x6 仿真 Nav2

终端 1：启动比赛地图仿真导航栈。

```bash
source /opt/ros/humble/setup.bash
source /opt/venom_nav_ws/install/setup.bash

ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=competition_10x6 \
  mode:=nav \
  localization:=amcl \
  lio:=none \
  lio_rviz:=False \
  nav_rviz:=True
```

终端 2：启动 commander，并显式指定比赛地图 mission。

```bash
source /opt/ros/humble/setup.bash
source /opt/venom_nav_ws/install/setup.bash

ros2 launch venom_mission_commander mission_commander_nav2_sim.launch.py \
  mission_config:=/workspaces/venom_vnv/venom_mission_commander/config/competition_10x6_mission.yaml \
  nav2_wait_mode:=bt_navigator \
  use_sim_time:=true
```

### 6. 真机 Nav2

真机流程应先由 bringup 启动传感器、TF、定位和 Nav2，再用 commander 接入任务编排。

终端 1：

```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_real.launch.py \
  use_sim_time:=false \
  mode:=nav
```

终端 2：

```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash

ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=true \
  use_sim_time:=false \
  nav2_wait_mode:=bt_navigator \
  mission_config:=/path/to/real_robot_mission.yaml
```

真实机器人专用 mission 建议放在 bringup 侧，例如 `venom_bringup/config/<robot>/missions/`，不要长期放在 `venom_mission_commander/config` 中。

## Mission YAML 结构

YAML 顶层当前支持这些区域：

```yaml
map:
  name: RMUL
  frame_id: map
  source: rm_nav_bringup/map/RMUL.yaml

mission:
  id: example_mission
  loop: false
  stop_on_task_failure: true
  nav_timeout_sec: 35.0
  retry_count: 1

waypoints:
  - name: task_point_1
    frame_id: map
    x: 1.0
    y: 2.0
    yaw: 0.0
    kind: operation_stop
    skip_navigation: false
    nav_timeout_sec: 20.0
    retry_count: 0
    description: optional human note
    tasks:
      - name: detect_item_at_point_1
        type: detect_item
        target: cube
```

### `map`

`map` 当前只是说明性 metadata，loader 不解析，也不会影响运行逻辑。推荐记录地图名、frame、地图文件来源、坐标标定说明等信息。

### `mission`

| 字段 | 必填 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `id` | 否 | `venom_mission_commander` | mission 运行 ID |
| `loop` | 否 | `false` | 是否循环执行 waypoint 列表 |
| `stop_on_task_failure` | 否 | `true` | task 失败时是否立即终止 mission |
| `nav_timeout_sec` | 否 | `null` | 默认导航超时；设置时必须为正数 |
| `retry_count` | 否 | `0` | 默认导航重试次数；必须为非负整数 |

### `waypoints[]`

| 字段 | 必填 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `name` | 是 | 无 | waypoint 名称，建议稳定且可读 |
| `x` | 是 | 无 | `frame_id` 坐标系下的目标 x |
| `y` | 是 | 无 | `frame_id` 坐标系下的目标 y |
| `frame_id` | 否 | `map` | Nav2 目标 pose 的 frame |
| `yaw` | 否 | `0.0` | 目标朝向，单位 rad |
| `kind` | 否 | `operation_stop` | `pass_through`、`operation_stop` 或 `return_park` |
| `tasks` | 否 | `[]` | 到点后按顺序执行的任务列表 |
| `skip_navigation` | 否 | `false` | 是否跳过导航，常用于起点或纯 mock waypoint |
| `description` | 否 | `""` | 人类可读说明 |
| `nav_timeout_sec` | 否 | 继承 mission 默认值 | 覆盖该 waypoint 的导航超时 |
| `retry_count` | 否 | 继承 mission 默认值 | 覆盖该 waypoint 的导航重试次数 |

### `tasks[]`

| 字段 | 必填 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `type` | 是 | 无 | 用于在 `TaskPluginRegistry` 中查找插件 |
| `name` | 否 | 等于 `type` | task 实例名，用于日志和状态记录 |
| 其它字段 | 否 | 无 | 全部进入 `TaskSpec.params`，由对应插件解释 |

默认 mock 插件类型包括：

- `detect_item`
- `grasp_item`
- `read_meter`
- `voice_report`
- `detect_flame`
- `track_flame`
- `classify_place`
- `wait`

真实视觉、语音、机械臂任务接入规范见 `TASK_PLUGIN_INTEGRATION_GUIDE.md`。

## 现有配置文件

| 文件 | 用途 | 建议定位 |
| --- | --- | --- |
| `config/simple_mission.yaml` | mock-first 最小验证路线 | 核心包保留 |
| `config/rmul_sim_mission.yaml` | RMUL Gazebo/Nav2 仿真路线 | 后续可迁到仿真包 |
| `config/competition_mission_template.yaml` | 比赛/真机 mission 模板 | 可保留为模板或迁到 bringup 示例 |
| `config/competition_10x6_mission.yaml` | 10m x 6m 比赛仿真地图近似路线 | 建议视为仿真/实验资产 |

长期建议：核心包只保留通用示例和 schema 说明；地图、机器人、比赛场地绑定的 mission 放到对应仿真包或 bringup 包。

## Docker 注意事项

Docker 现状是本机测试链路，目的是让 Ubuntu 24.04 / Jazzy 主机能运行 Ubuntu 22.04 / Humble 仿真栈。

权威 Docker 文件在 `simulation/venom_nav_simulation`：

- `Dockerfile.humble`
- `docker-compose.humble.yml`
- `docker/run_humble_sim.sh`
- `docker/bootstrap_humble_sim.sh`

当前行为：

- 仓库 bind mount 到容器内 `/workspaces/venom_vnv`。
- 持久化 ROS workspace 在 `/opt/venom_nav_ws`。
- `run_humble_sim.sh` 会构建镜像、启动 `humble-sim`、首次执行 bootstrap，然后进入容器。
- bootstrap 已把 `venom_mission_commander` 加入自动 symlink 和构建列表。
- 如果修改 `docker/bootstrap_humble_sim.sh` 并继续依赖 `run_humble_sim.sh`，需要重建 Docker 镜像，因为脚本执行的是镜像内复制到 `/opt/venom_scripts/bootstrap_humble_sim.sh` 的版本。

Docker 的重建策略和故障排查以 `simulation/venom_nav_simulation/README.md` 与 `simulation/venom_nav_simulation/AI_DOCKER_WORKFLOW.md` 为准。

## 推荐文件归属

| 内容 | 推荐位置 | 原因 |
| --- | --- | --- |
| commander 核心代码 | `venom_mission_commander/venom_mission_commander` | 可复用任务编排引擎 |
| 通用 node wrapper | `venom_mission_commander/launch/mission_commander.launch.py` | 不绑定仿真或真机环境 |
| mock 示例 mission | `venom_mission_commander/config/simple_mission.yaml` | 最小可运行示例 |
| RMUL / competition 仿真 mission | `simulation/venom_nav_simulation/config/missions` | 依赖仿真地图和 world |
| robot-specific 真机 mission | `venom_bringup/config/<robot>/missions` | 依赖机器人和现场地图 |
| robot-specific 真机 wrapper launch | `venom_bringup/launch/<robot>` | 只做真机参数包装 |
| Ubuntu 24.04 host Docker 安装脚本 | 本地或仿真包私有说明 | 主机强相关，不适合作为核心包默认入口 |

核心原则：换地图、换机器人、换宿主机就会变的内容，不要长期放在 `venom_mission_commander` 核心包里。
