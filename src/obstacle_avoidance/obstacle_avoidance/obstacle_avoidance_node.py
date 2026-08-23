#!/usr/bin/env python3
import math
import random
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


class SafetyState(Enum):
    PATROL = 0
    TRACK = 1
    AVOID = 2
    SEARCH = 3


class ObstacleAvoidanceNode(Node):
    """Reactive LiDAR safety layer with a PATROL/TRACK/AVOID/SEARCH state
    machine. PATROL is the default when no vision target is known; TRACK
    forwards the upstream vision-follower command; SEARCH rotates in
    place looking for a lost target; AVOID overrides PATROL or TRACK when
    an obstacle is detected and returns control to whichever behavior it
    interrupted once the path is clear.

    Command topics are strictly separated: this node SUBSCRIBES to the
    upstream desired command on input_cmd_vel_topic and PUBLISHES the
    final drive command on output_cmd_vel_topic. Those topics must be
    different so this node never subscribes and publishes on the same
    velocity topic."""

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # --- Parameters ---
        self.declare_parameter('scan_topic', '/scan')
        # Upstream desired command from the (future) vision-follower. NOT
        # the final drive command. Must be different from the output topic.
        self.declare_parameter('input_cmd_vel_topic', '/odom/desired_cmd_vel')
        # Final command sent toward the diff_drive_controller (see project
        # remap convention: /diff_drive_controller/cmd_vel).
        self.declare_parameter('output_cmd_vel_topic', '/diff_drive_controller/cmd_vel')
        # The vision system publishes std_msgs/Bool where True means a
        # valid target is currently detected and False (message absence
        # beyond target_detected_timeout) means no valid target.
        self.declare_parameter('vision_target_detected_topic', '/odom/target_detected')
        self.declare_parameter('target_detected_timeout', 0.5)

        self.declare_parameter('obstacle_distance', 0.6)
        self.declare_parameter('clear_distance', 0.6)
        self.declare_parameter('emergency_stop_distance', 0.2)

        self.declare_parameter('front_half_width_deg', 20.0)
        self.declare_parameter('front_side_half_width_deg', 25.0)

        self.declare_parameter('avoid_linear_speed', 0.05)
        self.declare_parameter('avoid_angular_speed', 1.0)
        self.declare_parameter('avoid_clear_ticks', 15)
        self.declare_parameter('search_angular_speed', 1.0)

        self.declare_parameter('patrol_linear_speed', 0.7)
        self.declare_parameter('patrol_angular_speed', 1.0)
        self.declare_parameter('patrol_turn_interval_sec', 10.0)
        self.declare_parameter('patrol_turn_duration_sec', 1.0)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        p = self.get_parameter
        self.scan_topic = p('scan_topic').value
        self.input_topic = p('input_cmd_vel_topic').value
        self.output_topic = p('output_cmd_vel_topic').value
        self.vision_target_detected_topic = p('vision_target_detected_topic').value
        self.target_detected_timeout = float(p('target_detected_timeout').value)

        self.obstacle_distance = float(p('obstacle_distance').value)
        self.clear_distance = float(p('clear_distance').value)
        self.emergency_stop_distance = float(p('emergency_stop_distance').value)

        self.front_half_width = math.radians(float(p('front_half_width_deg').value))
        self.front_side_half_width = math.radians(float(p('front_side_half_width_deg').value))

        self.avoid_linear_speed = float(p('avoid_linear_speed').value)
        self.avoid_angular_speed = float(p('avoid_angular_speed').value)
        self.avoid_clear_ticks = int(p('avoid_clear_ticks').value)
        self.search_angular_speed = float(p('search_angular_speed').value)

        self.patrol_linear_speed = float(p('patrol_linear_speed').value)
        self.patrol_angular_speed = float(p('patrol_angular_speed').value)
        self.patrol_turn_interval_sec = float(p('patrol_turn_interval_sec').value)
        self.patrol_turn_duration_sec = float(p('patrol_turn_duration_sec').value)

        self.cmd_vel_timeout = float(p('cmd_vel_timeout').value)
        
        self._search_spin_duration = (4.0 * math.pi / self.search_angular_speed
                              if self.search_angular_speed > 0 else float('inf'))
        self._search_start_time = None

        # --- State ---
        # PATROL is the initial state. The robot patrols autonomously
        # until the (future) vision system reports a valid target.
        self.state = SafetyState.PATROL
        # Which non-AVOID behavior AVOID interrupted, so AVOID can return
        # control to it once the path is clear.
        self.previous_behavior_state = SafetyState.PATROL

        self.sectors = {'left': math.inf, 'front_left': math.inf,
                        'front': math.inf, 'front_right': math.inf,
                        'right': math.inf}
        self.turn_direction = 1.0
        self.avoid_clear_tick_count = 0

        self.last_input_cmd = TwistStamped()
        self.last_input_cmd_time = self.get_clock().now()
        self.have_input_cmd = False

        # Defaults to False until a message is received from the
        # future vision node. 
        self.target_detected = False
        self.last_target_detected_time = self.get_clock().now()
        self.have_target_msg = False

        # Patrol sweep timing state.  
        self.patrol_mode = 'FORWARD'
        self.patrol_mode_start_time = self.get_clock().now()
        
        # --- Pub/Sub ---
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = QoSHistoryPolicy.KEEP_LAST

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, sensor_qos)

        self.input_cmd_sub = self.create_subscription(
            TwistStamped, self.input_topic, self.input_cmd_callback, 10)

        self.target_detected_sub = self.create_subscription(
            Bool, self.vision_target_detected_topic,
            self.target_detected_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, self.output_topic, 10)

        period = 1.0 / float(p('control_rate_hz').value)
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"obstacle_avoidance_node started: scan='{self.scan_topic}' "
            f"input='{self.input_topic}' output='{self.output_topic}' "
            f"vision='{self.vision_target_detected_topic}' state=PATROL")

    # ---------------------------------------------------------------
    def input_cmd_callback(self, msg: TwistStamped):
        self.last_input_cmd = msg
        self.last_input_cmd_time = self.get_clock().now()
        self.have_input_cmd = True

    def target_detected_callback(self, msg: Bool):
        self.target_detected = bool(msg.data)
        self.last_target_detected_time = self.get_clock().now()
        self.have_target_msg = True

    def is_target_detected(self) -> bool:
        """Fail-safe: treat detection as False if no fresh vision message
        has arrived within target_detected_timeout. This means TRACK/SEARCH
        will never drive without an actively-published True from vision."""
        if not self.have_target_msg:
            return False
        age = (self.get_clock().now() - self.last_target_detected_time).nanoseconds / 1e9
        if age > self.target_detected_timeout:
            return False
        return self.target_detected

    def is_input_cmd_fresh(self) -> bool:
        if not self.have_input_cmd:
            return False
        age = (self.get_clock().now() - self.last_input_cmd_time).nanoseconds / 1e9
        return age <= self.cmd_vel_timeout

    # ---------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """Bucket range readings into 5 sectors and record min range per sector."""
        mins = {k: math.inf for k in self.sectors}
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                sector = self._sector_for_angle(angle)
                if r < mins[sector]:
                    mins[sector] = r
            angle += msg.angle_increment
        self.sectors = mins

    def _sector_for_angle(self, angle: float) -> str:
        # Normalize to [-pi, pi]
        a = math.atan2(math.sin(angle), math.cos(angle))
        fw = self.front_half_width
        sw = self.front_side_half_width
        if -fw <= a <= fw:
            return 'front'
        if fw < a <= fw + sw:
            return 'front_left'
        if -(fw + sw) <= a < -fw:
            return 'front_right'
        if a > 0:
            return 'left'
        return 'right'

    # ---------------------------------------------------------------
    def control_loop(self):
        front = self.sectors['front']
        front_left = self.sectors['front_left']
        front_right = self.sectors['front_right']
        left = self.sectors['left']
        right = self.sectors['right']
        closest_front = min(front, front_left, front_right)
        
        self.get_logger().info(f"State: {self.state.name}, front: {front:.2f}, left: {left:.2f}, right: {right:.2f}")

        # Emergency stop always wins, regardless of state.
        if closest_front <= self.emergency_stop_distance:
            self.publish_cmd(0.0, 0.0)
            return

        # LiDAR safety override: PATROL and TRACK can be pre-empted by
        # AVOID whenever an obstacle appears. AVOID records which
        # behavior it interrupted so it can hand control back later.
        if self.state in (SafetyState.PATROL, SafetyState.TRACK, SafetyState.SEARCH):
            if front <= self.obstacle_distance:
                self.restore = self.state
                if self.restore not in (SafetyState.PATROL, SafetyState.TRACK):
                    self.restore = self.SafetyState.PATROL
                self.previous_behavior_state = self.restore
                self.state = SafetyState.AVOID
                self.turn_direction = 1.0 if left >= right else -1.0
                self.avoid_clear_tick_count = 0
                self.get_logger().info(
                    f'Obstacle detected ({closest_front:.2f} m) while in '
                    f'{self.previous_behavior_state.name} -> AVOID, turning '
                    f"{'left' if self.turn_direction > 0 else 'right'}")

        if self.state == SafetyState.PATROL:
            # PATROL does NOT depend on the vision system for its motion,
            # only for the transition to TRACK.
            if self.is_target_detected():
                self.state = SafetyState.TRACK
                self.get_logger().info('Target detected while patrolling -> TRACK')
                self.publish_cmd(0.0, 0.0)
                return
            self.patrol_behavior()
            return

        if self.state == SafetyState.TRACK:
            if not self.is_target_detected():
                self.state = SafetyState.SEARCH
                self._search_start_time = self.get_clock().now()
                self.get_logger().info('Target lost -> SEARCH')
                # Fall through to SEARCH handling below.
            else:
                self.track_behavior()
                return

        if self.state == SafetyState.SEARCH:
            if self.is_target_detected():
                self.state = SafetyState.TRACK
                self._search_start_time = None
                self.get_logger().info('Target reacquired -> TRACK')
                self.track_behavior()
                return
            if self._search_start_time is not None:
                elapsed = (self.get_clock().now() - self._search_start_time).nanoseconds / 1e9
                if elapsed >= self._search_spin_duration:
                    self.state = SafetyState.PATROL
                    self._search_start_time = None
                    self.get_logger().info('Search timeout -> PATROL')
                    self.publish_cmd(0.0, 0.0)
                    return
            # Rotate in place looking for the target. Never drive forward
            # in SEARCH -- no valid target is currently detected.
            self.publish_cmd(0.0, self.search_angular_speed)
            return

        if self.state == SafetyState.AVOID:
            if closest_front < self.obstacle_distance * 0.7:
                linear = 0.0
            else:
                linear = self.avoid_linear_speed
                
            self.publish_cmd(
                linear,
                self.turn_direction * self.avoid_angular_speed)

            if front >= self.clear_distance:
                self.avoid_clear_tick_count += 1
            else:
                self.avoid_clear_tick_count = 0

            if self.avoid_clear_tick_count >= self.avoid_clear_ticks:
                # Return control to whichever behavior AVOID interrupted.
                self.state = self.previous_behavior_state
                self.avoid_clear_tick_count = 0
                self.get_logger().info(
                    f'Path clear -> resuming {self.state.name}')
            return

    # ---------------------------------------------------------------
    def track_behavior(self):
        """TRACK: forward the upstream vision-follower command, but ONLY
        while a valid target is currently detected AND the upstream
        command is fresh. Otherwise publish a zero command -- never move
        forward in TRACK without an actual detected target."""
        if not self.is_target_detected():
            self.publish_cmd(0.0, 0.0)
            return
        if not self.is_input_cmd_fresh():
            # Vision says target present but no fresh command yet.
            self.publish_cmd(0.0, 0.0)
            return
        self.publish_cmd(
            self.last_input_cmd.twist.linear.x,
            self.last_input_cmd.twist.angular.z)

    def patrol_behavior(self):
        """Drive straight for a while, then turn left or right randomly."""
        now = self.get_clock().now()
        elapsed = (now - self.patrol_mode_start_time).nanoseconds/1e9
        
        if self.patrol_mode == 'FORWARD':
            if elapsed >= self.patrol_turn_interval_sec:
                if random.choice([True, False]):
                    self.patrol_mode = 'TURN_LEFT'
                    self.get_logger().info("Patrol: turning left")
                else:
                    self.patrol_mode = 'TURN_RIGHT'
                    self.get_logger().info("Patrol: turning right")
                self.patrol_mode_start_time = now
            else:
                self.publish_cmd(self.patrol_linear_speed, 0.0)
                
        elif self.patrol_mode == 'TURN_LEFT':
            if elapsed >= self.patrol_turn_duration_sec:
                self.patrol_mode = 'FORWARD'
                self.patrol_mode_start_time = now
                self.publish_cmd(self.patrol_linear_speed, 0.0)
            else:
                self.publish_cmd(0.0, self.patrol_angular_speed)
                
        elif self.patrol_mode == 'TURN_RIGHT':
            if elapsed >= self.patrol_turn_duration_sec:
                self.patrol_mode = 'FORWARD'
                self.patrol_mode_start_time = now
                self.publish_cmd(self.patrol_linear_speed, 0.0)
            else:
                self.publish_cmd(0.0, -self.patrol_angular_speed)
                
    # ---------------------------------------------------------------
    def publish_cmd(self, linear_x: float, angular_z: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
