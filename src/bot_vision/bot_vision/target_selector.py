import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D
from std_msgs.msg import Int32


class TargetSelector(Node):
    """Chooses the primary target from candidate detections.

    Selection strategy is configurable: by class name, by proximity
    (largest bounding box area == closest), or a fixed user-specified
    track id override. Publishes the selected detection's bounding box
    plus a stable numeric target id so the tracker can lock onto it.
    """

    def __init__(self):
        super().__init__('target_selector')

        self.declare_parameter('detections_topic', '/bot_vision/detections')
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('selection_mode', 'class_and_proximity')  # class_only | proximity_only | class_and_proximity
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('selected_detection_topic', '/bot_vision/selected_detection')
        self.declare_parameter('selected_target_id_topic', '/bot_vision/target_id')

        self.detections_topic = self.get_parameter('detections_topic').value
        self.target_class = self.get_parameter('target_class').value
        self.selection_mode = self.get_parameter('selection_mode').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.selected_detection_topic = self.get_parameter('selected_detection_topic').value
        self.selected_target_id_topic = self.get_parameter('selected_target_id_topic').value

        self._next_id = 0
        self._current_id = None

        self.detection_pub = self.create_publisher(Detection2D, self.selected_detection_topic, 10)
        self.target_id_pub = self.create_publisher(Int32, self.selected_target_id_topic, 10)

        self.detections_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self._detections_callback, 10)

        self.get_logger().info(
            f"target_selector listening on '{self.detections_topic}', "
            f"target_class='{self.target_class}', mode='{self.selection_mode}'")

    def _bbox_area(self, detection: Detection2D) -> float:
        size = detection.bbox.size_x * detection.bbox.size_y
        return float(size)

    def _matches_class(self, detection: Detection2D, target_class: str) -> bool:
        if not detection.results:
            return False
        best = max(detection.results, key=lambda r: r.hypothesis.score)
        if best.hypothesis.score < self.min_confidence:
            return False
        return best.hypothesis.class_id == target_class

    def _detections_callback(self, msg: Detection2DArray):
        candidates = list(msg.detections)
        current_target_class = self.get_parameter('target_class').value

        if self.selection_mode in ('class_only', 'class_and_proximity'):
            candidates = [d for d in candidates if self._matches_class(d, current_target_class)]

        if not candidates:
            return

        if self.selection_mode in ('proximity_only', 'class_and_proximity'):
            selected = max(candidates, key=self._bbox_area)
        else:
            selected = candidates[0]

        if self._current_id is None:
            self._current_id = self._next_id
            self._next_id += 1

        self.detection_pub.publish(selected)

        id_msg = Int32()
        id_msg.data = self._current_id
        self.target_id_pub.publish(id_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
