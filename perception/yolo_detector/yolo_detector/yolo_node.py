from typing import List, Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_interfaces.msg import YoloBox, YoloDetection, YoloDetections, YoloHypothesis


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.bridge = CvBridge()

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("detector_name", "yolo_detector")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("output_topic", "/perception/detections")
        self.declare_parameter("annotated_image_topic", "/perception/debug/yolo_result")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "")
        self.declare_parameter("class_ids", "")

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.detector_name = self.get_parameter("detector_name").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.annotated_image_topic = (
            self.get_parameter("annotated_image_topic").get_parameter_value().string_value
        )
        self.publish_annotated_image = (
            self.get_parameter("publish_annotated_image").get_parameter_value().bool_value
        )
        self.confidence_threshold = (
            self.get_parameter("confidence_threshold").get_parameter_value().double_value
        )
        self.iou_threshold = self.get_parameter("iou_threshold").get_parameter_value().double_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.class_ids = self._parse_class_ids(
            self.get_parameter("class_ids").get_parameter_value().string_value
        )

        self.model = self._load_model(self.model_path)

        self.detections_pub = self.create_publisher(YoloDetections, self.output_topic, 10)
        self.annotated_image_pub = None
        if self.publish_annotated_image:
            self.annotated_image_pub = self.create_publisher(
                Image, self.annotated_image_topic, 10
            )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f"YOLO detector ready. model={self.model_path}, input={self.image_topic}, "
            f"output={self.output_topic}"
        )

    def _load_model(self, model_path: str):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required at runtime for yolo_detector. "
                "Install it in your ROS environment before launching this node."
            ) from exc

        return YOLO(model_path)

    @staticmethod
    def _parse_class_ids(class_ids: str) -> Optional[List[int]]:
        if not class_ids.strip():
            return None
        return [int(token.strip()) for token in class_ids.split(",") if token.strip()]

    def image_callback(self, msg: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(
            source=image,
            verbose=False,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device or None,
            classes=self.class_ids,
        )

        if not results:
            self.detections_pub.publish(self._build_detections_msg(msg, []))
            return

        result = results[0]
        detections = self._build_detection_list(result, image.shape[1], image.shape[0])
        self.detections_pub.publish(self._build_detections_msg(msg, detections))

        if self.annotated_image_pub is not None:
            annotated = result.plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

    def _build_detections_msg(
        self, image_msg: Image, detections: List[YoloDetection]
    ) -> YoloDetections:
        msg = YoloDetections()
        msg.header = image_msg.header
        msg.detections = detections
        return msg

    def _build_detection_list(
        self, result, image_width: int, image_height: int
    ) -> List[YoloDetection]:
        detections: List[YoloDetection] = []
        names = getattr(result, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return detections

        image_center_x = image_width / 2.0
        image_center_y = image_height / 2.0

        xywh = boxes.xywh.cpu().tolist()
        scores = boxes.conf.cpu().tolist()
        class_ids = boxes.cls.cpu().tolist()

        for bbox_xywh, score, class_id_raw in zip(xywh, scores, class_ids):
            center_x, center_y, size_x, size_y = bbox_xywh
            class_id = int(class_id_raw)

            detection = YoloDetection()
            detection.hypothesis = YoloHypothesis()
            detection.hypothesis.class_id = class_id
            detection.hypothesis.class_name = str(names.get(class_id, class_id))
            detection.hypothesis.score = float(score)

            detection.bbox = YoloBox()
            detection.bbox.center_x = float(center_x)
            detection.bbox.center_y = float(center_y)
            detection.bbox.size_x = float(size_x)
            detection.bbox.size_y = float(size_y)

            detections.append(detection)

        return detections


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
