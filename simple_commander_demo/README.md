# simple_commander_demo

`simple_commander_demo` 是一个独立 ROS 2 Python demo 包，用来验证“按路点顺序导航，到点后执行任务，再去下一个点”的任务编排流程。

默认运行模式是 mock：不需要真实地图、Nav2 action server、视觉节点、语音节点或机械臂节点，只打印流程并在内存 `blackboard` 中传递模拟结果。

## Demo 任务路线

```text
起停区
→ 任务点一：识别物品 → 机械臂夹取
→ 任务点二：识别电表图像 → 语音播报
→ 任务点三：识别火焰图片 → 追踪
→ 任务点四：物品分类 → 放置
→ 回到起停区
```

默认 mock 坐标写在 `config/simple_mission.yaml` 中。接真实仿真导航时优先使用 `config/rmul_sim_mission.yaml`；后续接比赛地图时复制 `config/competition_mission_template.yaml` 后替换 `x/y/yaw`。

从 demo 演进到正式工程集成的阶段路线见 `docs/INTEGRATION_ROADMAP.md`。

如果未来想把 mission 输入进一步泛化为“语义地图 + 任务目标 + 约束”，并动态生成 waypoint/task 绑定，可参考 `docs/FUTURE_DYNAMIC_MISSION_BRANCH.md`。

## 主要接口

- `SimpleCommander.configure()`：读取 YAML、注册任务插件、创建导航器、初始化任务状态。
- `SimpleCommander.run()`：执行完整 mission，可根据 YAML 的 `loop` 决定是否循环。
- `SimpleCommander.run_waypoint()`：处理单个路点，先导航，再执行该路点任务列表。
- `MissionLoader.load(config_path)`：把 YAML 转成 `MissionConfig` / `WaypointSpec` / `TaskSpec`。
- `MissionManager.transition_to()`：记录任务状态切换。
- `MissionManager.save_state()`：保存当前路点、当前任务、最近任务结果等运行状态。
- `MockWaypointNavigator`：默认导航器，不依赖 Nav2。
- `Nav2WaypointNavigator`：真实 Nav2 导航器，使用 `nav2_simple_commander.BasicNavigator.goToPose()`。
- `WaypointTaskRunner.run_tasks()`：按顺序执行当前路点的 tasks。
- `TaskPluginRegistry.get(task_type)`：根据 YAML 的 `type` 找到任务插件。
- `BaseTaskPlugin.execute(context, spec)`：任务插件统一入口。

## 当前 mock 任务插件

- `detect_item`：模拟识别物品，写入 `blackboard["detected_item"]`。
- `grasp_item`：模拟机械臂夹取，读取 `detected_item`，写入 `blackboard["grasped_object"]`。
- `read_meter`：模拟电表图像识别，写入 `blackboard["meter_reading"]`。
- `voice_report`：模拟语音播报，默认读取 `meter_reading`。
- `detect_flame`：模拟火焰图片识别，写入 `blackboard["flame_detection"]`。
- `track_flame`：模拟火焰追踪，读取 `flame_detection`。
- `classify_place`：模拟分类放置，读取 `grasped_object`。
- `wait`：模拟等待。

## 本机工作区运行

```bash
cd /home/alex/venom_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select simple_commander_demo
source install/setup.bash
ros2 launch simple_commander_demo simple_commander_demo.launch.py use_nav:=false
```

也可以直接指定配置：

```bash
ros2 run simple_commander_demo simple_commander_demo \
  --ros-args \
  -p mission_config:=/home/alex/venom_ws/src/venom_vnv/simple_commander_demo/config/simple_mission.yaml \
  -p use_nav:=false
```

## Docker 里运行 mock demo

当前 Humble Docker 的仓库挂载点是 `/workspaces/venom_vnv`，仿真工作区是 `/opt/venom_nav_ws`。这个 demo 包没有修改 bootstrap 脚本，所以首次在容器内运行时手动把包 symlink 进仿真工作区即可。

```bash
cd /workspaces/venom_vnv/simulation/venom_nav_simulation
./docker/run_humble_sim.sh
```

进入容器后：

```bash
source /opt/ros/humble/setup.bash
mkdir -p /opt/venom_nav_ws/src
ln -sfn /workspaces/venom_vnv/simple_commander_demo /opt/venom_nav_ws/src/simple_commander_demo
cd /opt/venom_nav_ws
colcon build --symlink-install --packages-select simple_commander_demo
source install/setup.bash
ros2 launch simple_commander_demo simple_commander_demo.launch.py use_nav:=false
```

## Docker 里接 Gazebo / Nav2 / RViz

第一步先启动现有 Humble 仿真导航栈。这个 launch 负责 Gazebo、定位、Nav2 和 RViz；`simple_commander_demo` 只作为 Nav2 action client，不重复启动导航栈。

终端 1：进入容器并启动仿真导航。

```bash
cd /workspaces/venom_vnv/simulation/venom_nav_simulation
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

第二步先在 RViz 里手动用 `2D Goal Pose` 验证机器人能导航。如果手动目标都失败，先排查地图、定位、costmap、TF 和 Nav2 lifecycle，不要先查 commander。

终端 2：启动真实 Nav2 模式的 commander。

```bash
source /opt/ros/humble/setup.bash
source /opt/venom_nav_ws/install/setup.bash
ros2 launch simple_commander_demo simple_commander_nav2_sim.launch.py
```

等价的显式命令是：

```bash
ros2 launch simple_commander_demo simple_commander_demo.launch.py \
  use_nav:=true \
  use_sim_time:=true \
  nav2_wait_mode:=bt_navigator \
  mission_config:=/opt/venom_nav_ws/src/simple_commander_demo/config/rmul_sim_mission.yaml
```

`simple_commander_nav2_sim.launch.py` 默认使用 `config/rmul_sim_mission.yaml`、`use_nav:=true`、`use_sim_time:=true`，适合在 Gazebo/RViz/Nav2 已经启动后直接验证整条链路。

## 接真实 Nav2

启动仿真导航栈后，把 `use_nav` 改成 `true`：

```bash
ros2 launch simple_commander_demo simple_commander_demo.launch.py \
  use_nav:=true \
  use_sim_time:=true \
  nav2_wait_mode:=bt_navigator
```

如果后续改成 AMCL 等完整 Nav2 lifecycle 流程，可以尝试：

```bash
ros2 launch simple_commander_demo simple_commander_demo.launch.py use_nav:=true nav2_wait_mode:=full
```

## 后续接比赛地图

建议保持“任务插件不变，只换地图坐标”的接入方式：

1. 复制模板：

   ```bash
   cp /opt/venom_nav_ws/src/simple_commander_demo/config/competition_mission_template.yaml \
      /opt/venom_nav_ws/src/simple_commander_demo/config/my_competition_mission.yaml
   ```

2. 启动目标地图对应的 Gazebo/Nav2/RViz。
3. 在 RViz 里逐个验证可达点，记录 `map` frame 下的 `x/y/yaw`。
4. 只替换 `my_competition_mission.yaml` 中的坐标和地图说明，尽量保持路点名与任务名稳定。
5. 用同一个 launch 跑新地图任务：

   ```bash
   ros2 launch simple_commander_demo simple_commander_demo.launch.py \
     use_nav:=true \
     use_sim_time:=true \
     nav2_wait_mode:=bt_navigator \
     mission_config:=/opt/venom_nav_ws/src/simple_commander_demo/config/my_competition_mission.yaml
   ```

比赛地图接入时优先检查这些条件：

- 所有 waypoint 都使用 `frame_id: map`，并且地图 origin 与 RViz 显示一致。
- 每个任务点先用 RViz `2D Goal Pose` 单独验证可达，再跑完整 mission。
- 起停区如果需要真实返航，`return_start_area` 必须填真实起点坐标；不要只依赖 `start_area.skip_navigation`。
- 如果切换 AMCL 或完整 lifecycle 流程，再尝试 `nav2_wait_mode:=full`。

## 后续接真实任务

真实视觉、语音和机械臂接口建议优先替换这些类的内部 mock 方法，而不是改 `SimpleCommander` 主流程：

完整任务插件接入规范见 `docs/TASK_PLUGIN_INTEGRATION_GUIDE.md`，里面约定了 YAML task 格式、`BaseTaskPlugin` 接口、`TaskContext` / `TaskExecutionResult` 数据结构、`blackboard` key 和 ROS service/action 接入方式。

如果要规划什么时候接仿真、什么时候接真实任务模块、什么时候纳入 `venom_bringup` 和 Docker 默认构建，见 `docs/INTEGRATION_ROADMAP.md`。

- `DetectItemTaskPlugin`
- `GraspItemTaskPlugin`
- `ReadMeterTaskPlugin`
- `VoiceReportTaskPlugin`
- `DetectFlameTaskPlugin`
- `TrackFlameTaskPlugin`
- `ClassifyPlaceTaskPlugin`

这样主流程仍保持：`导航 → 到点 → 执行任务列表 → 保存状态 → 下一个点`。
