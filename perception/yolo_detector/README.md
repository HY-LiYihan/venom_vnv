# yolo_detector

`yolo_detector` is a ROS 2 package set for YOLO-based 2D object detection.

It contains:

- `yolo_interfaces/`: YOLO-specific message definitions
- `yolo_detector/`: Python node that runs YOLO and publishes detections
- `launch/`: launch entry for the detector node
- `config/`: default runtime parameters

## Message Format

The current output is intentionally minimal and only keeps fields needed for pure YOLO 2D detection.

### `yolo_interfaces/msg/YoloBox.msg`

```msg
float32 center_x
float32 center_y
float32 size_x
float32 size_y
```

- All values are pixel-space `xywh`
- `center_x`, `center_y` are the box center in the image
- `size_x`, `size_y` are width and height

### `yolo_interfaces/msg/YoloHypothesis.msg`

```msg
int32 class_id
string class_name
float32 score
```

- `class_id`: model output class id
- `class_name`: resolved class label
- `score`: confidence score

### `yolo_interfaces/msg/YoloDetection.msg`

```msg
YoloHypothesis hypothesis
YoloBox bbox
```

- One detected object
- Contains only category and bounding box

### `yolo_interfaces/msg/YoloDetections.msg`

```msg
std_msgs/Header header
YoloDetection[] detections
```

- One frame of detector output
- `header.stamp`: detection timestamp
- `header.frame_id`: usually the source image frame

## Topics

Default topics:

- input image: `/image_raw`
- detections: `/perception/detections`
- debug image: `/perception/debug/yolo_result`

## Deployment

### 1. System prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- OpenCV / `cv_bridge`

### 2. Install workspace dependencies

Use `rosdep` first:

```bash
cd ~/venom_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

`yolo_detector/package.xml` declares:

- `cv_bridge`
- `sensor_msgs`
- `rclpy`
- `yolo_interfaces`
- `python3-ultralytics-pip`

### 3. About `ultralytics`

`colcon build` does not install Python dependencies by itself. It only builds the ROS workspace.

`rosdep` is the step that installs external dependencies declared in `package.xml`.

For this package, `ultralytics` is declared via the rosdep key:

- `python3-ultralytics-pip`

According to the ROS dependency index, this key resolves to a `pip` install of `ultralytics` on Ubuntu/Debian.

That means:

- If `rosdep` is installed and configured correctly, `rosdep install ...` should install `ultralytics`
- If `rosdep` is not available in the environment, you need to install it manually

Manual fallback:

```bash
python3 -m pip install -U ultralytics
```

### 4. Build

```bash
cd ~/venom_ws
colcon build --packages-select yolo_interfaces yolo_detector
source install/setup.bash
```

### 5. Run

```bash
ros2 launch yolo_detector yolo_detector.launch.py model_path:=/path/to/model.pt
```

Or with defaults:

```bash
ros2 run yolo_detector yolo_node
```

## Notes

- The current format is only for YOLO-style 2D bounding-box output
- It does not include depth, pose, or tracker state
- If you later add depth or tracking, it is better to extend with a new message instead of polluting this minimal format
