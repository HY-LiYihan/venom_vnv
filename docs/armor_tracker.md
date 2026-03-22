# armor_tracker

目标跟踪模块，使用扩展卡尔曼滤波跟踪装甲板目标并预测运动。

## 功能

接收装甲板 3D 位置，进行滤波跟踪，输出目标状态和预测位置。

## 快速开始

```bash
ros2 run armor_tracker armor_tracker_node
```

## 主要特性

- 扩展卡尔曼滤波（EKF）
- 多目标跟踪管理
- 运动预测与补偿
- 目标丢失处理

## 详细文档

详见：[armor_tracker README](../rm_auto_aim/armor_tracker/README.md)
