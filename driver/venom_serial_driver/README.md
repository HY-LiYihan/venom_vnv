# venom_serial_driver

DJI C板串口通信驱动，用于 NUC 与电控板之间的双向数据传输。

## 项目结构

```
venom_serial_driver/
├── venom_serial_driver/          # Python 模块
│   ├── serial_interface.py       # 串口通信层
│   ├── crc_utils.py              # CRC 校验
│   ├── serial_protocol.py        # 通信协议
│   └── serial_node.py            # ROS2 节点
├── config/
│   └── serial_params.yaml        # 参数配置
├── launch/
│   └── serial_driver.launch.py  # 启动文件
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
- `VisionCtrlData`: NUC → C板控制指令
- `RobotStateData`: C板 → NUC状态数据
- 帧格式：[SOF][len][cmd_id][data][CRC16]

### serial_node.py
ROS2 节点，提供话题接口：
- 发布：`/robot_state` - 机器人状态
- 订阅：`/vision_ctrl` - 视觉控制

## 使用
```bash
ros2 launch venom_serial_driver serial_driver.launch.py
```

详细文档：[venom_serial_driver.md](../../docs/venom_serial_driver.md)
