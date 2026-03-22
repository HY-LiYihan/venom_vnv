# Venom VNV 机器人系统

基于 ROS2 Humble 的多功能机器人系统，集成了激光雷达建图定位、视觉识别、自动瞄准和导航功能。

## 📋 系统架构

### 核心模块

#### 1. 定位与建图 (LIO)
- **Point-LIO**: 激光雷达惯性里程计，提供高精度 3D 建图和实时定位
  - 支持 Livox MID360 激光雷达
  - 基于 IEKF 的紧耦合融合算法
  - 输出点云地图和里程计信息

#### 2. 重定位 (Relocalization)
- **small_gicp_relocalization**: 基于点云配准的重定位模块
  - 使用 small_gicp 算法进行点云对齐
  - 计算 map 到 odom 的坐标变换
  - 支持先验地图加载

#### 3. 视觉系统 (Vision)
- **rm_auto_aim**: RoboMaster 装甲板自动瞄准
  - **armor_detector**: 装甲板识别与 3D 位置解算
  - **armor_tracker**: 目标跟踪与状态估计
  - **auto_aim_interfaces**: 接口定义
- **rm_vision_bringup**: 视觉系统启动配置

#### 4. 硬件驱动 (Drivers)
- **livox_ros_driver2**: Livox 激光雷达驱动
- **ros2_hik_camera**: 海康工业相机驱动
- **scout_ros2**: 松灵 Scout 底盘驱动
- **ugv_sdk**: 通用无人车 SDK

#### 5. 机器人描述
- **rm_gimbal_description**: 云台 URDF 模型

#### 6. 系统集成 (Bringup)
- **venom_bringup**: 系统启动配置
  - `mapping_bringup.launch.py`: 建图模式
    - Livox 雷达驱动
    - Point-LIO 3D 建图
    - slam_toolbox 2D 建图
    - 点云转激光扫描
  - `relocalization_bringup.launch.py`: 重定位模式
    - Livox 雷达驱动
    - Point-LIO 定位
    - 先验地图发布
    - 重定位节点

## 🗂️ 目录结构

```
venom_vnv/
├── driver/                    # 硬件驱动
│   ├── livox_ros_driver2/    # Livox 雷达驱动
│   ├── ros2_hik_camera/      # 海康相机驱动
│   ├── scout_ros2/           # Scout 底盘驱动
│   └── ugv_sdk/              # UGV SDK
├── lio/                       # 激光雷达里程计
│   └── Point-LIO/            # Point-LIO 算法
├── relocalization/            # 重定位模块
│   └── small_gicp_relocalization/
├── rm_auto_aim/              # 自动瞄准
│   ├── armor_detector/       # 装甲板检测
│   ├── armor_tracker/        # 目标跟踪
│   └── auto_aim_interfaces/  # 接口定义
├── rm_gimbal_description/    # 云台描述
├── venom_vision/             # 视觉系统
│   └── rm_vision_bringup/    # 视觉启动
├── venom_bringup/            # 系统启动
│   ├── launch/               # 启动文件
│   ├── config/               # 配置文件
│   └── rviz_cfg/             # RViz 配置
└── docs/                      # 文档
```

## 🚀 快速开始

### 建图模式
```bash
ros2 launch venom_bringup mapping_bringup.launch.py
```

### 重定位模式
```bash
ros2 launch venom_bringup relocalization_bringup.launch.py pcd_file:=/path/to/map.pcd
```

## 📚 详细文档

- [安装配置教程](./docs/setup.md)
- [Point-LIO 说明](./docs/point_lio.md)
- [串口驱动说明](./docs/venom_serial_driver.md)

## 📦 子项目文档

### 硬件驱动
- [venom_serial_driver](./docs/venom_serial_driver.md) - DJI C板串口通信驱动
- [livox_ros_driver2](./driver/livox_ros_driver2/README.md) - Livox 激光雷达驱动
- [ros2_hik_camera](./driver/ros2_hik_camera/README.md) - 海康工业相机驱动
- [scout_ros2](./driver/scout_ros2/README.md) - 松灵 Scout 底盘驱动

### 定位与建图
- [Point-LIO](./docs/point_lio.md) - 激光雷达惯性里程计
- [small_gicp_relocalization](./docs/small_gicp_relocalization.md) - 点云重定位

### 视觉系统
- [rm_auto_aim](./docs/rm_auto_aim.md) - 自动瞄准系统
- [armor_detector](./docs/armor_detector.md) - 装甲板检测
- [armor_tracker](./docs/armor_tracker.md) - 目标跟踪
- [rm_vision](./venom_vision/README.md) - 视觉框架

### 系统集成
- [venom_bringup](./venom_bringup/README.md) - 系统启动配置
- [rm_gimbal_description](./rm_gimbal_description/README.md) - 云台描述

### 环境配置
- [安装配置](./docs/setup.md) - 系统安装与配置指南
- [Point-LIO](./docs/point_lio.md) - 激光雷达惯性里程计

### 系统配置
- [环境配置](./docs/setup.md) - 系统安装与配置指南

## 🔧 依赖项

- ROS2 Humble
- Livox-SDK2
- PCL (Point Cloud Library)
- Eigen3
- OpenCV
- Nav2

## 📝 License

各子模块遵循各自的开源协议，详见各模块 README。