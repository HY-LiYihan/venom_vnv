---
title: 配置雷达
desc: Livox Mid360 与 livox_ros_driver2 的安装、网络配置和基础验证。
breadcrumb: 部署与使用
layout: default
---

## 安装 Livox-SDK2

```bash
cd ~
sudo apt update
sudo apt install -y cmake git
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

## 检查是否安装成功

```bash
ldconfig -p | grep LivoxSdkCore
```

## 卸载旧版 Livox-SDK2

如果需要清理旧版本，可执行：

```bash
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*
```

## 准备 livox_ros_driver2

仓库已经通过 submodule 提供 `driver/livox_ros_driver2`，构建前需要先执行：

```bash
cp ~/venom_ws/src/venom_vnv/driver/livox_ros_driver2/package_ROS2.xml \
   ~/venom_ws/src/venom_vnv/driver/livox_ros_driver2/package.xml
```

## 配置网卡静态 IP

建议把连接 Mid360 的有线网卡配置为静态 IP。

推荐配置：

- 本机有线网卡 IP：`192.168.1.50`
- 子网掩码：`255.255.255.0`
- 网关：`192.168.1.1`

Mid360 默认地址通常为：

- `192.168.1.1xx`

其中 `xx` 是雷达序列号后两位。比如序列号末两位是 `33`，则雷达 IP 可对应理解为 `192.168.1.133`。

配置完成后，WiFi 可以继续保持联网，用于 SSH、NoMachine 或其他网络访问。

## 修改 MID360 配置文件

打开：

```text
~/venom_ws/src/venom_vnv/driver/livox_ros_driver2/config/MID360_config.json
```

重点确认以下字段：

```json
"cmd_data_ip": "192.168.1.50",
"push_msg_ip": "192.168.1.50",
"lidar_ip": "192.168.1.133"
```

其中：

- `cmd_data_ip` 和 `push_msg_ip` 应与本机有线网卡静态 IP 一致
- `lidar_ip` 应改成你自己 Mid360 的实际 IP

如果已经完成编译，也建议同步检查安装目录下的配置文件：

```text
~/venom_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

## 雷达验证

先测试网络是否连通：

```bash
ping 192.168.1.133
```

如果你的雷达不是这个地址，请把上面的 IP 替换成实际值。

然后启动驱动验证：

```bash
cd ~/venom_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

再开一个终端检查话题是否正常发布：

```bash
source ~/venom_ws/install/setup.bash
ros2 topic list | grep livox
```

如果能看到 `/livox/lidar` 等相关话题，说明基础链路已经打通。

## 相关文档

- [装配配置](setup.md)
- [Livox 雷达驱动](livox_ros_driver2.md)
- [快速开始](quick_start.md)
