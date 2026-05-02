# venom_bringup

系统启动配置包

## 功能
- 建图模式启动
- 重定位模式启动
- 集成多传感器驱动
- PX4 DDS 探测示例启动
- 支持将路网 YAML 转换为 Nav2 航点 YAML 和比赛 `waypoint.txt`

## 路网转航点

先用转换命令把路网文件导出为现有航点格式：

```bash
ros2 run venom_bringup road_network_to_waypoints \
  --road-network-file /path/to/road_network.yaml \
  --route-name demo_patrol \
  --output-file /path/to/waypoints.yaml \
  --competition-output-file /path/to/waypoint.txt
```

也可以直接在 `craic_mission_main`、`multi_waypoint_commander` 或 `bringup_all.launch.py` 里传 `road_network_file`、`start_node_id`、`goal_node_id` 或 `blocked_edges` 参数。

## 启动文件
- `scout_mini_mapping.launch.py` - Scout Mini 3D+2D 建图
- `sentry_mapping.launch.py` - Sentry 3D+2D 建图
- `relocalization_bringup.launch.py` - 重定位
- `px4_agent_probe.launch.py` - PX4 uXRCE-DDS 连通性探测与桥接状态检查
- `px4_vps_bridge.launch.py` - PX4 外部位姿桥接
