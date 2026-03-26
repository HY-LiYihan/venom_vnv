# venom_robot_description

静态 TF 树发布包，用于定义 Venom 机器人的固定坐标变换关系。

## 功能

- 从 YAML 配置文件读取静态 TF 定义
- 自动为每个变换启动 `static_transform_publisher` 节点
- 支持任意数量的固定坐标变换，无需修改代码

## 使用

### 编译
```bash
cd /home/venom/venom_ws
colcon build --packages-select venom_robot_description
source install/setup.bash
```

### 启动
```bash
ros2 launch venom_robot_description scout_mini_description.launch.py
```

### 验证
```bash
# 查看 TF 变换
ros2 run tf2_ros tf2_echo base_link laser_link

# 生成 TF 树图
ros2 run tf2_tools view_frames
```

## 配置

编辑 `config/static_tf.yaml` 添加或修改静态变换：

```yaml
transforms:
  - parent_frame: base_link
    child_frame: laser_link
    translation: [0.0, 0.0, 0.2]   # x, y, z (米)
    rotation: [0.0, 0.0, 0.0]      # roll, pitch, yaw (弧度)
```

修改后重启 launch 文件即可生效。

## TF 树结构

参见 [docs/tf_tree.md](../docs/tf_tree.md)
