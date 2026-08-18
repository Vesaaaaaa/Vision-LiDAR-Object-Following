import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
import cv2


class TrackerNode(Node):
    """Lightweight single-target tracker.

    Consumes the camera image plus the selected target's detection box and
    maintains a KCF tracker across frames. Publishes the target's bearing
    (rad, relative to camera boresight) and estimated distance (m, from
    known object width) as a Vector3Stamped (x=bearing, y=distance).
    """

    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameter('camera_topic', '/following_bot/camera/rgb')
        self.declare_parameter('selected_detection_topic', '/bot_vision/selected_detection')
        self.declare_parameter('target_id_topic', 'bot_vision/target_id')
        self.declare_parameter('bearing_distance_topic', 'bot_vision/target_bearing_distance')
        self.declare_parameter('target_detected_topic', '/odom/target_detected')
        self.declare_parameter('camera_hfov_rad', 1.047)
        self.declare_parameter('known_object_width_m', 0.15)
        self.declare_parameter('reacquire_iou_threshold', 0.3)
        self.declare_parameter('tracker_type', 'KCF')
        self.declare_parameter('control_rate_hz', 30.0)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.selected_detection_topic = self.get_parameter('selected_detection_topic').value
        self.target_id_topic = self.get_parameter('target_id_topic').value
        self.bearing_distance_topic = self.get_parameter('bearing_distance_topic').value
        self.target_detected_topic = self.get_parameter('target_detected_topic').value
        self.hfov = float(self.get_parameter('camera_hfov_rad').value)
        self.known_width = float(self.get_parameter('known_object_width_m').value)
        self.iou_thresh = float(self.get_parameter('reacquire_iou_threshold').value)
        self.tracker_type = self.get_parameter('tracker_type').value
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.bridge = CvBridge()
        self.cv_tracker = None
        self.last_frame = None
        self.frame_width = None
        self.target_id = None
        self.pending_bbox = None
        self.have_lock = False

        self.bearing_pub = self.create_publisher(Vector3Stamped, self.bearing_distance_topic, 10)
        self.detected_pub = self.create_publisher_bool = self.create_publisher(
            __import__('std_msgs.msg', fromlist=['Bool']).Bool, self.target_detected_topic, 10)
        
        
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.image_sub = self.create_subscription(Image, self.camera_topic, self._image_cb, sensor_qos)
        self.det_sub = self.create_subscription(Detection2D, self.selected_detection_topic, self._detections_cb, 10)
        self.target_id_sub = self.create_subscription(Int32, self.target_id_topic, self._target_id_cb, 10)

        self.create_timer(1.0 / self.rate_hz, self._timer_cb)

        self.get_logger().info('tracker_node started')

    def _make_tracker(self):
        if hasattr(cv2, 'TrackerKCF_create'):
            return cv2.TrackerKCF_create()
        return cv2.legacy.TrackerKCF_create()

    def _target_id_cb(self, msg):
        if msg.data != self.target_id:
            self.target_id = msg.data
            self.cv_tracker = None
            self.have_lock = False

    def _detections_cb(self, det):
        if self.target_id is None:
            return
        if not det.results:
            return
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y
        self.pending_bbox = (cx - w / 2.0, cy - h / 2.0, w, h)

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return
        self.last_frame = frame
        self.frame_width = frame.shape[1]

        if self.pending_bbox is not None and not self.have_lock:
            self.cv_tracker = self._make_tracker()
            bbox_init = tuple(int(round(v)) for v in self.pending_bbox) # KCF tracker expects an integer Rect
            self.cv_tracker.init(frame, bbox_init)
            self.have_lock = True
            self.pending_bbox = None
            return

        if self.have_lock and self.cv_tracker is not None:
            ok, bbox = self.cv_tracker.update(frame)
            if not ok:
                self.have_lock = False
                self.cv_tracker = None
            else:
                self._publish_bearing_distance(bbox)

    def _publish_bearing_distance(self, bbox):
        x, y, w, h = bbox
        if self.frame_width is None or w <= 0:
            return
        center_x = x + w / 2.0
        norm_offset = (center_x - self.frame_width / 2.0) / (self.frame_width / 2.0)
        bearing = norm_offset * (self.hfov / 2.0)
        focal_px = (self.frame_width / 2.0) / math.tan(self.hfov / 2.0)
        distance = (self.known_width * focal_px) / w if w > 0 else float('inf')

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.vector.x = bearing
        msg.vector.y = distance
        msg.vector.z = 0.0
        self.bearing_pub.publish(msg)

    def _timer_cb(self):
        from std_msgs.msg import Bool
        detected = Bool()
        detected.data = bool(self.have_lock)
        self.detected_pub.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
