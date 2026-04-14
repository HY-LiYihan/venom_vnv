---
title: 快速开始
desc: 面向第一次使用者的最短上手路径。
breadcrumb: 部署与使用
layout: default
---

## 开始之前

请先确认以下工具已经可用：

- ROS 2 Humble
- `rosdep`
- `colcon`
- Livox-SDK2

可以先用下面的方式确认 Livox-SDK2 已安装：

```bash
ldconfig -p | grep LivoxSdkCore
```

如果还没有安装，请先参考 [装配配置](setup.md) 完成 Livox-SDK2 安装。

如果 `rosdep install` 过程中报错，建议先尝试：

```bash
sudo rosdep init
rosdep update
```

## 推荐安装方式

如果你之前有旧工作区，建议先清理后重新拉取：

```bash
rm -rf ~/venom_ws
mkdir -p ~/venom_ws/src
git clone --recurse-submodules https://github.com/Venom-Algorithm/Venom_VNV ~/venom_ws/src/venom_vnv
cp ~/venom_ws/src/venom_vnv/driver/livox_ros_driver2/package_ROS2.xml \
   ~/venom_ws/src/venom_vnv/driver/livox_ros_driver2/package.xml

cd ~/venom_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
```

编译完成后：

```bash
source ~/venom_ws/install/setup.bash
```

## 推荐阅读顺序

1. [环境准备]({{ '/environment' | relative_url }})
2. [硬件部署]({{ '/hardware_setup' | relative_url }})
3. [软件部署]({{ '/software_setup' | relative_url }})
4. [启动使用]({{ '/launch_usage' | relative_url }})

## 首次验证

- 雷达链路：参考 [Livox 雷达驱动](livox_ros_driver2.md)
- 相机链路：参考 [海康相机驱动](ros2_hik_camera.md)
- 串口链路：参考 [串口通信驱动](venom_serial_driver.md)
- 系统级数据流：参考 [话题与 TF 总览]({{ '/system_overview' | relative_url }})
