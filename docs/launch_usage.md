---
title: 启动使用
desc: 建图、重定位、自瞄和整机模式的常用启动方式。
breadcrumb: 部署与使用
layout: default
---

## 常用启动命令

```bash
source install/setup.bash

# 自瞄测试
ros2 launch venom_bringup autoaim_test_bringup.launch.py

# 导航 + 自瞄
ros2 launch venom_bringup autoaim_nav_bringup.launch.py

# 建图
ros2 launch venom_bringup mapping_bringup.launch.py

# 重定位
ros2 launch venom_bringup relocalization_bringup.launch.py
```

## 阅读建议

- 启动入口设计：参考 [系统集成]({{ '/integration_overview' | relative_url }})
- 不同模式的区别：参考 [运行模式]({{ '/run_modes' | relative_url }})
