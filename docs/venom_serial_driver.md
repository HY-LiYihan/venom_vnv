# venom_serial_driver

DJI C板串口通信驱动，用于 RoboMaster 机器人视觉系统与电控板之间的数据交互。

## 功能特性

- 双向通信协议实现
- CRC16 数据校验
- 滑动窗口帧解析
- 超时保护机制
- ROS2 话题接口

## 安装

```bash
cd ~/venom_ws
colcon build --packages-select venom_serial_driver
source install/setup.bash
```

## 使用方法

### 启动驱动

```bash
ros2 launch venom_serial_driver serial_driver.launch.py
```

### 自定义串口参数

```bash
ros2 launch venom_serial_driver serial_driver.launch.py \
  port_name:=/dev/ttyUSB0 \
  baud_rate:=921600
```

## ROS2 话题

### 发布话题

- `/robot_state` (std_msgs/String) - 机器人状态数据（JSON格式）
  ```json
  {
    "timestamp_us": 1234567,
    "pitch": 0.1,
    "yaw": 0.2,
    "pitch_speed": 0.01,
    "yaw_speed": 0.02,
    "current_HP": 600,
    "game_progress": 4
  }
  ```

### 订阅话题

- `/vision_ctrl` (std_msgs/String) - 视觉控制指令（JSON格式）
  ```json
  {
    "tracking_state": 1,
    "target_pitch": 0.15,
    "target_yaw": 0.25,
    "target_pitch_v": 0.01,
    "target_yaw_v": 0.02
  }
  ```

## 参数配置

编辑 `config/serial_params.yaml`：

```yaml
serial_node:
  ros__parameters:
    port_name: "/dev/ttyUSB0"  # 串口设备
    baud_rate: 921600           # 波特率
    timeout: 0.1                # 读取超时(秒)
    loop_rate: 50               # 循环频率(Hz)
```

## 通信协议

详细协议说明请参考 [protocol.md](protocol.md)

### 帧格式

**NUC → C板 (控制指令)**
```
[0xA5][len(2)][0x02][data][CRC16(2)]
```

**C板 → NUC (状态数据)**
```
[0x5A][len(2)][0x01][data][CRC16(2)]
```

## 测试

### 查看状态数据

```bash
ros2 topic echo /robot_state
```

### 发送控制指令

```bash
ros2 topic pub /vision_ctrl std_msgs/String \
  'data: "{\"tracking_state\":1,\"target_pitch\":0.1,\"target_yaw\":0.2}"'
```

## 故障排查

1. **串口权限问题**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **检查串口连接**
   ```bash
   ls /dev/ttyUSB*
   ```

3. **查看日志**
   ```bash
   ros2 run venom_serial_driver serial_node --ros-args --log-level debug
   ```

