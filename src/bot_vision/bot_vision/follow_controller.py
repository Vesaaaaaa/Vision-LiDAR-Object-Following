import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from std_msgs.msg import Bool


class FollowController(Node):
    """Proportional controller that turns bearing/distance into a desired
    velocity command for the obstacle_avoidance_node to arbitrate."""

    def __init__(self):
        super().__init__('follow_controller')

        self.declare_parameter('bearing_distance_topic', 'bot_vision/target_bearing_distance')
        self.declare_parameter('target_detected_topic', '/odom/target_detected')
        self.declare_parameter('output_cmd_vel_topic', '/odom/desired_cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('linear_gain', 0.6)
        self.declare_parameter('angular_gain', 1.8)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('distance_deadband', 0.1)
        self.declare_parameter('bearing_deadband', 0.02)
        self.declare_parameter('target_timeout_sec', 0.5)

        bearing_distance_topic = self.get_parameter('bearing_distance_topic').value
        target_detected_topic = self.get_parameter('target_detected_topic').value
        output_topic = self.get_parameter('output_cmd_vel_topic').value
        control_rate = self.get_parameter('control_rate_hz').value

        self._bearing = 0.0
        self._distance = 0.0
        self._target_detected = False
        self._last_target_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(TwistStamped, output_topic, 10)

        self.create_subscription(
            Vector3Stamped, bearing_distance_topic, self._bearing_distance_cb, 10)
        self.create_subscription(Bool, target_detected_topic, self._target_detected_cb, 10)

        period = 1.0 / float(control_rate)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'follow_controller started: bearing/distance<-{bearing_distance_topic}, '
            f'detected<-{target_detected_topic}, '
            f'cmd_vel->{output_topic} @ {control_rate} Hz'
        )

    def _bearing_distance_cb(self, msg):
        self._bearing = msg.vector.x
        self._distance = msg.vector.y
        self._last_target_time = self.get_clock().now()

    def _target_detected_cb(self, msg):
        self._target_detected = msg.data
        if msg.data:
            self._last_target_time = self.get_clock().now()

    def _control_loop(self):
        timeout = self.get_parameter('target_timeout_sec').value
        now = self.get_clock().now()
        elapsed = (now - self._last_target_time).nanoseconds / 1e9

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        if not self._target_detected or elapsed > timeout:
            # No valid target: publish zero velocity, let obstacle_avoidance_node
            # fall back to SEARCH/PATROL based on /odom/target_detected.
            self.cmd_pub.publish(msg)
            return

        desired_distance = self.get_parameter('desired_distance').value
        linear_gain = self.get_parameter('linear_gain').value
        angular_gain = self.get_parameter('angular_gain').value
        max_linear = self.get_parameter('max_linear_speed').value
        max_angular = self.get_parameter('max_angular_speed').value
        dist_deadband = self.get_parameter('distance_deadband').value
        bearing_deadband = self.get_parameter('bearing_deadband').value

        distance_error = self._distance - desired_distance
        if abs(distance_error) < dist_deadband:
            distance_error = 0.0

        bearing = self._bearing
        if abs(bearing) < bearing_deadband:
            bearing = 0.0

        linear = linear_gain * distance_error
        angular = angular_gain * bearing

        linear = max(-max_linear, min(max_linear, linear))
        angular = max(-max_angular, min(max_angular, angular))

        # Only drive forward toward a target in front; don't reverse for now.
        if linear < 0.0:
            linear = 0.0

        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
