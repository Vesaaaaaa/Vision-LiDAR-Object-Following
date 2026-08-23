import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetectorNode(Node):
    """Runs a YOLO model on an incoming camera stream and publishes
    candidate object detections as a Detection2DArray."""

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('camera_topic', '/following_bot/camera/rgb')
        self.declare_parameter('detections_topic', '/bot_vision/detections')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('detected_classes_topic', '/bot_vision/detected_classes')

        self.camera_topic = self.get_parameter('camera_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detected_classes_topic = self.get_parameter('detected_classes_topic').value

        self.bridge = CvBridge()
        self.model = None
        if YOLO is not None: # Debugging purposes
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLO model "{self.model_path}" loaded successfully from venv site-packages')
            except Exception as exc:
                self.get_logger().error(f'Failed to load YOLO model: {exc}')
        else:
            self.get_logger().warn('ultralytics/yolo_v8 not available; detector will publish empty results')

        
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self._image_callback, sensor_qos)
        self.detections_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10)
        self.classes_pub = self.create_publisher(
            String, self.detected_classes_topic, 10)

        self.get_logger().info(f'yolo_detector subscribed to {self.camera_topic}, publishing to {self.detections_topic}')

    def _image_callback(self, msg: Image):
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        
        classes_msg = String()
        classes_msg.data = "no_objects"

        if self.model is None:
            self.detections_pub.publish(detections_msg)
            self.classes_pub.publish(classes_msg)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            self.detections_pub.publish(detections_msg)
            self.classes_pub.publish(classes_msg)
            return

        results = self.model.predict(cv_image, conf=self.confidence_threshold, verbose=False)
        if not results:
            self.detections_pub.publish(detections_msg)
            self.classes_pub.publish(classes_msg)
            return

        result = results[0]
        names = result.names if hasattr(result, 'names') else {}

        detection_names = set()
        max_score = 0.0
        best_class = "none"
        for box in getattr(result, 'boxes', []):
            cls_id = int(box.cls[0]) if hasattr(box, 'cls') else -1
            score = float(box.conf[0]) if hasattr(box, 'conf') else 0.0
            if score >= self.confidence_threshold:
                class_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
                detection_names.add(class_name)
                if score > max_score:
                    max_score = score
                    best_class = class_name

        if detection_names:
            classes_msg.data = ', '.join(sorted(detection_names))
        else:
            classes_msg.data = "no_objects"
        
        self.get_logger().info(
            f'Detected: {classes_msg.data} | Best: {best_class} ({max_score:.2f})',
            throttle_duration_sec=1.0
        )
            
        for box in getattr(result, 'boxes', []):
            xyxy = box.xyxy[0].tolist()
            x1, y1, x2, y2 = xyxy
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1

            det = Detection2D()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            cls_id = int(box.cls[0]) if hasattr(box, 'cls') else -1
            score = float(box.conf[0]) if hasattr(box, 'conf') else 0.0
            class_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = score
            det.results.append(hypothesis)

            detections_msg.detections.append(det)

        self.detections_pub.publish(detections_msg)
        self.classes_pub.publish(classes_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
