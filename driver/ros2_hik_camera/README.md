
<div align="center">

# ROS2 Hikvision Camera Driver

<p align="center">
  <a href="#introduction">Introduction</a> •
  <a href="#features">Features</a> •
  <a href="#installation">Installation</a> •
  <a href="#usage">Usage</a> •
  <a href="#configuration">Configuration</a> •
  <a href="#acknowledgement">Acknowledgement</a>
</p>

[![ROS2](https://img.shields.io/badge/ROS2-Humble%7CIron-blue)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-x86__64%20|%20ARM64-orange)](https://ubuntu.com/)
[![Build](https://img.shields.io/badge/Build-Colcon-brightgreen)](https://colcon.readthedocs.io/)

</div>

---

## 📝 Introduction

**ros2_hik_camera** is a high-performance ROS2 driver designed for Hikvision Industrial Cameras (USB3.0).

This project aims to provide a robust and easy-to-use interface for integrating Hikvision cameras into the ROS2 ecosystem. It leverages the official Hikvision MVS SDK and supports image streaming, parameter configuration, and hardware triggering.

This repository is developed based on the work of [chenjunnn/ros2_hik_camera](https://github.com/chenjunnn/ros2_hik_camera), with improvements in documentation, structure, and compatibility.

## ✨ Features

- **ROS2 Native**: Fully compatible with ROS2 communication mechanisms.
- **Embedded SDK**: The Hikvision MVS SDK is integrated directly into the package (`hikSDK`), eliminating the need for system-wide driver installation.
- **Multi-Platform Support**:
    - **amd64 (x86_64)**: Standard Desktop/Laptop (Intel/AMD).
    - **arm64 (aarch64)**: Embedded systems like NVIDIA Jetson Orin/Xavier and Raspberry Pi.
- **Configurable**: Full control over exposure, gain, and camera intrinsics via YAML configuration files.

## 📂 File Structure

```text
ros2_hik_camera
├── config/                 # Parameter configuration files
│   ├── camera_info.yaml    # Camera calibration data (intrinsics)
│   └── camera_params.yaml  # Camera control params (exposure, gain, etc.)
├── hikSDK/                 # Integrated Hikvision MVS SDK
│   ├── include/            # SDK Headers
│   └── lib/                # Pre-compiled shared libraries (amd64/arm64)
├── launch/                 # ROS2 Launch files
├── src/                    # Driver source code
└── CMakeLists.txt          # Build configuration
````

## 🛠️ Installation

### 1\. Prerequisites

  - **Ubuntu** 20.04 / 22.04 / 24.04
  - **ROS2** (Galactic, Humble, Iron, or Jazzy)

### 2\. Clone and Build

Clone this repository into your ROS2 workspace `src` folder:

```bash
cd ~/venom_ws/src
git clone [https://github.com/HY-LiYihan/ros2_hik_camera.git](https://github.com/HY-LiYihan/ros2_hik_camera.git)
cd ..
colcon build --symlink-install --packages-select hik_camera
```

## 🚀 Usage

Source the setup script and launch the camera node:

```bash
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```

You can view the image stream using `rqt` or `rviz2`:

  - **Topic**: `/image_raw` (or as defined in your launch file)

## ⚙️ Configuration

The camera parameters can be modified in `config/camera_params.yaml`.

| Parameter | Type | Description | Default |
| :--- | :--- | :--- | :--- |
| `exposure_time` | float | Exposure time in microseconds. | `5000.0` |
| `gain` | float | Analog gain of the camera. | `16.0` |


*Note: The actual parameters depend on the specific implementation in `hik_camera_node.cpp`.*

## 🤝 Acknowledgement

We would like to express our gratitude to the original author for their open-source contribution:

  - **Base Project**: [chenjunnn/ros2\_hik\_camera](https://www.google.com/url?sa=E&source=gmail&q=https://github.com/chenjunnn/ros2_hik_camera)

This project builds upon their work to further enhance usability and integration for RoboMaster and other robotics applications.

## 📄 License

The source code is released under the **Apache License 2.0**.
