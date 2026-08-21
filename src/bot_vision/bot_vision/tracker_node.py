import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from std_msgs.msg import Int32
from std_msgs.msg import Bool
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
        self.declare_parameter('lost_timeout_sec', 1.0)
        self.declare_parameter('no_detection_timeout_sec', 1.0)

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
        self.lost_timeout_sec = float(self.get_parameter('lost_timeout_sec').value)
        self.no_detection_timeout_sec = float(self.get_parameter('no_detection_timeout_sec').value)

        self.bridge = CvBridge()
        self.cv_tracker = None
        self.last_frame = None
        self.frame_width = None
        self.target_id = None
        self.pending_bbox = None
        self.have_lock = False
        self.last_seen_time = None
        self.last_detection_time = None
        self.last_tracked_bbox = None

        self.bearing_pub = self.create_publisher(Vector3Stamped, self.bearing_distance_topic, 10)
        self.detected_pub = self.create_publisher(Bool, self.target_detected_topic, 10)
        
        
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
        
        
    def _iou(self, bbox1, bbox2):
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)
        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        bbox1_area = w1 * h1
        bbox2_area = w2 * h2
        union_area = bbox1_area + bbox2_area - inter_area
        if union_area == 0:
            return 0.0
        return inter_area / union_area    

    def _make_tracker(self):
        # cv2 5.0.0's non-legacy TrackerKCF_create().init() returns None
        # instead of True/False (verified empirically), which Python
        # evaluates as falsy and makes every init() call look like a
        # failure regardless of bbox quality. The legacy API returns a
        # real bool, so prefer it whenever it's available.
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            return cv2.legacy.TrackerKCF_create()
        return cv2.TrackerKCF_create()

    def _clamp_bbox(self, bbox, frame_shape):
        height, width = frame_shape[0], frame_shape[1]
        x, y, w, h = bbox
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(1, min(w, width - x))
        h = max(1, min(h, height - y))
        return (x, y, w, h)

    def _target_id_cb(self, msg):
        if msg.data != self.target_id:
            self.target_id = msg.data
            self.cv_tracker = None
            self.have_lock = False
            self.last_tracked_bbox = None
            self.last_detection_time = None

    def _detections_cb(self, det):
        if self.target_id is None or not det.results:
            return
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y
        new_bbox = (cx - w / 2.0, cy - h / 2.0, w, h)
        if self.have_lock and self.last_tracked_bbox is not None:
            iou = self._iou(self.last_tracked_bbox, new_bbox)
            self.get_logger().debug(f'IoU = {iou:.2f}', throttle_duration_sec=0.5)
            if iou < self.iou_thresh:
                self.get_logger().info(f'Re-initialising tracker (IoU={iou:.2f} < {self.iou_thresh})')
                self.cv_tracker = self._make_tracker()
                clamped_bbox = self._clamp_bbox(new_bbox, self.last_frame.shape)
                bbox_init = tuple(int(round(v)) for v in clamped_bbox)
                if self.cv_tracker.init(self.last_frame, bbox_init):
                    self.have_lock = True
                    self.last_seen_time = self.get_clock().now()
                    self.last_detection_time = self.get_clock().now()
                    self.last_tracked_bbox = new_bbox
                    self.pending_bbox = None
                else:
                    self.have_lock = False
                    self.cv_tracker = None
                return
            else:
                self.last_detection_time = self.get_clock().now()
        else:
            self.pending_bbox = new_bbox
            self.last_seen_time = self.get_clock().now()

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
            clamped_bbox = self._clamp_bbox(self.pending_bbox, frame.shape)
            bbox_init = tuple(int(round(v)) for v in clamped_bbox) # KCF tracker expects an integer Rect
            if self.cv_tracker.init(frame, bbox_init): 
                self.have_lock = True
                self.last_seen_time = self.get_clock().now()
                self.last_detection_time = self.get_clock().now()
                self.last_tracked_bbox = self.pending_bbox
                self.get_logger().info("Tracker initialised")
            else:
                self.cv_tracker = None
                self.have_lock = False
                self.get_logger().warn("Tracker initialisation failed")
            self.pending_bbox = None
            return

        if self.have_lock and self.cv_tracker is not None:
            ok, bbox = self.cv_tracker.update(frame)
            if not ok:
                self.have_lock = False
                self.cv_tracker = None
                self.last_tracked_bbox = None
                self.get_logger().warn("Tracker update failed, lock lost")
            else:
                self.last_seen_time = self.get_clock().now()
                self.last_tracked_bbox = bbox
                self.get_logger().info(
                    f'Tracker OK: bbox=({bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}, {bbox[3]:.1f})',
                    throttle_duration_sec=0.5
                )
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
        now = self.get_clock().now()
        if self.have_lock:
            if self.last_seen_time is not None:
                elapsed = (now - self.last_seen_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'Timer: have_lock={self.have_lock}, elapsed={elapsed:.2f}s (timeout={self.lost_timeout_sec}s)',
                    throttle_duration_sec=2.0
                )
                if elapsed > self.lost_timeout_sec:
                    self.have_lock = False
                    self.cv_tracker = None
                    self.last_tracked_bbox = None
                    self.get_logger().warn(f"Lock timed out (tracker update) after {elapsed:.2f}s")
            if self.have_lock and self.last_detection_time is not None:
                det_elapsed = (now - self.last_detection_time).nanoseconds / 1e9
                if det_elapsed > self.no_detection_timeout_sec:
                    self.have_lock = False
                    self.cv_tracker = None
                    self.last_tracked_bbox = None
                    self.get_logger().warn(f'Lock timed out (no overlapping detection) after {det_elapsed:.2f}s')

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
