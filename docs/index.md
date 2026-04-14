---
title: Venom VNV
desc: 基于 ROS2 Humble 的 RoboMaster 机器人系统，集成定位建图、重定位、导航与自瞄能力。
breadcrumb: 首页
layout: default
---

## 快速导航

<div class="card-grid">
  <a href="{{ '/quick_start' | relative_url }}" class="card" style="text-decoration:none">
    <h3>⚙️ 快速开始</h3>
    <p>按标准流程拉取仓库、安装依赖并完成首次编译。</p>
  </a>
  <a href="{{ '/environment' | relative_url }}" class="card" style="text-decoration:none">
    <h3>🧰 环境配置</h3>
    <p>准备 Ubuntu、SSH、ROS、rosdep、VS Code、Clash 与 NoMachine。</p>
  </a>
  <a href="{{ '/lidar_setup' | relative_url }}" class="card" style="text-decoration:none">
    <h3>📡 配置雷达</h3>
    <p>安装 Livox-SDK2、配置 MID360 网络参数并验证雷达链路。</p>
  </a>
</div>

## 子模块一览

| 分类 | 子模块 | 说明 |
|------|--------|------|
| 🖥️ 硬件驱动 | `livox_ros_driver2` | Livox Mid360 激光雷达驱动 |
| 🖥️ 硬件驱动 | `ros2_hik_camera` | 海康 USB3.0 工业相机驱动 |
| 🖥️ 硬件驱动 | `scout_ros2` + `ugv_sdk` | Scout Mini 底盘驱动 |
| 🖥️ 硬件驱动 | `venom_serial_driver` | NUC ↔ DJI C 板串口通信 |
| 📍 定位建图 | `Point-LIO` | 高带宽激光惯性里程计 |
| 📍 定位建图 | `rf2o_laser_odometry` | 2D 激光扫描里程计 |
| 📍 定位建图 | `small_gicp_relocalization` | 点云重定位 |
| 🎯 自瞄算法 | `rm_auto_aim` | 检测 + 跟踪 + 解算 + 消息定义 |
| 🔧 系统集成 | `venom_bringup` | 启动配置 + Mission Controller |
| 🔧 系统集成 | `venom_robot_description` | TF 树发布 |

## 快速开始

```bash
# 克隆主仓库及子模块
git clone --recurse-submodules git@github.com:Venom-Algorithm/Venom_VNV.git
cd Venom_VNV

# 安装依赖
rosdep install --from-paths . --ignore-src -r -y

# 编译
colcon build --symlink-install -DCMAKE_BUILD_TYPE=Release
```

> 详细的装配步骤见 [装配配置]({{ '/setup' | relative_url }})
