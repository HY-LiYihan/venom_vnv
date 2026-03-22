# venom_serial_driver

DJI C板串口通信驱动，用于 NUC 与电控板之间的双向数据传输。

## 协议参数

- **波特率**: 921600 bps
- **C板→NUC**: 72字节状态帧
- **NUC→C板**: 33字节控制帧
- **校验**: CRC16

## 项目结构

```
venom_serial_driver/
├── venom_serial_driver/          # Python 模块
│   ├── serial_interface.py       # 串口通信层
│   ├── crc_utils.py              # CRC 校验
│   ├── serial_protocol.py        # 通信协议
│   └── serial_node.py            # ROS2 节点
├── msg/
│   ├── RobotStatus.msg           # 机器人状态消息
│   └── GameStatus.msg            # 比赛状态消息
├── config/
│   └── serial_params.yaml        # 参数配置
├── launch/
│   └── serial_driver.launch.py  # 启动文件
├── test/
│   ├── test_loopback.py          # 自回环测试
│   ├── test_monitor.py           # 持续监听测试
│   └── test_hardware.py          # 真机测试
└── docs/
    └── protocol.md               # 协议文档
```

## 核心模块

### serial_interface.py
串口通信层，封装 pyserial 提供底层读写接口。
- 921600 波特率
- 线程安全
- 超时保护

### crc_utils.py
CRC 校验工具，实现 DJI 标准的 CRC8/CRC16 算法。

### serial_protocol.py
通信协议实现：
- `RobotCtrlData`: NUC → C板控制指令（33字节）
  - flags, 7个float (lx,ly,lz,ax,ay,az,dist), frame_x, frame_y
- `RobotStateData`: C板 → NUC状态数据（72字节）
  - 9个float (速度、角度、角速度等) + 裁判系统数据
- 帧格式：[SOF][len][cmd_id][data][CRC16]

### serial_node.py
ROS2 节点，提供话题接口：
- 发布：`/robot_status` (RobotStatus) - 机器人运动状态
- 发布：`/game_status` (GameStatus) - 比赛信息
- 订阅：`/cmd_vel` (Twist) - 底盘和云台控制

## 使用

### ROS2 节点
```bash
# 编译
cd ~/venom_ws
colcon build --packages-select venom_serial_driver

# 启动
source install/setup.bash
ros2 launch venom_serial_driver serial_driver.launch.py
```

### 测试程序

所有测试程序都是独立脚本，无需编译：

```bash
cd ~/venom_ws/src/venom_vnv/driver/venom_serial_driver

# 自回环测试（需要 TX-RX 短接）
python3 test/test_loopback.py /dev/ttyUSB0

# 持续监听测试（持续运行，Ctrl+C 退出）
python3 test/test_monitor.py --port /dev/ttyUSB0

# Yaw 旋转测试（10秒）
python3 test/test_hardware.py --port /dev/ttyUSB0 --test yaw --duration 10

# 底盘运动测试（前后左右各2秒）
python3 test/test_hardware.py --port /dev/ttyUSB0 --test chassis

# 全部测试
python3 test/test_hardware.py --port /dev/ttyUSB0 --test all
```

## 更新日志

### v0.2.0 (2026-03-22)
- 协议迁移：72字节状态帧 + 33字节控制帧
- 重构话题结构：`/robot_status`, `/game_status`, `/cmd_vel`
- 新增自定义消息：RobotStatus, GameStatus
- 新增测试程序：test_monitor.py, test_hardware.py
- 重命名：test_serial.py → test_loopback.py

详细文档：[venom_serial_driver.md](../../docs/venom_serial_driver.md)
