from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value="yolov8n.pt"),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("output_topic", default_value="/perception/detections"),
            Node(
                package="yolo_detector",
                executable="yolo_node",
                name="yolo_detector",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    }
                ],
            ),
        ]
    )
