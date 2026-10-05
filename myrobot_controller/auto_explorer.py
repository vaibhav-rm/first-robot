#!/usr/bin/env python3
"""Auto Explorer: wall-following coverage for autonomous mapping.

Reactive controller that sweeps the world without teleoperation:
- Left-hand-rule wall following so the whole reachable area gets scanned
- Sector analysis (front / front-left / front-right / left / right / rear)
- Never reverses: blocked poses are escaped by rotating in place, which
  avoids backing into whatever sits behind the robot
- Geofence keeps the robot inside the mapped arena
- Stuck detection with a real-time (not tick-counted) timeout
- Runs for a configurable duration, then stops
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # Parameters
        self.declare_parameter('max_duration_sec', 300)
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('turn_speed', 0.7)
        self.declare_parameter('obstacle_threshold', 0.75)
        self.declare_parameter('geofence_radius', 4.5)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('stuck_timeout', 4.0)
        self.declare_parameter('stuck_distance_threshold', 0.08)

        self.max_duration = float(self.get_parameter('max_duration_sec').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold').value)
        self.geofence_radius = float(self.get_parameter('geofence_radius').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.stuck_distance_threshold = float(self.get_parameter('stuck_distance_threshold').value)

        # Clearance floor used for reverse-permission checks
        self.declare_parameter('rear_clear_threshold', 0.45)
        self.rear_clear_threshold = float(self.get_parameter('rear_clear_threshold').value)

        # State
        self.start_time = self.get_clock().now()
        self.last_loop_time = self.get_clock().now()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Sector clearances, in metres (inf when sector is empty)
        self.front = math.inf
        self.front_left = math.inf
        self.front_right = math.inf
        self.left = math.inf
        self.right = math.inf
        self.rear = math.inf

        # Wall following
        self.wall_side = 1.0          # +1 hug left wall, -1 hug right wall
        self.wall_lost_ticks = 0.0    # time since we last saw the wall
        self.wall_lost_timeout = 1.5

        # Stuck detection / escape
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0
        self.stuck_timer = 0.0
        self.escape_mode = False
        self.escape_stage = 0
        self.escape_timer = 0.0
        self.escape_turn_sign = 1.0
        self.escape_reversing = False

        # Escape stage durations (seconds)
        self.escape_rotate_time = 1.5
        self.escape_reverse_time = 2.5
        self.escape_final_rotate_time = 1.5

        # Post-escape commitment: don't immediately re-approach the trap.
        self.commit_timer = 0.0
        self.commit_speed_scale = 1.0

        # Publishers / subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos)
        self.odom_sub = self.create_subscription(Odometry, '/wheel/odom', self.odom_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Auto Explorer started: {self.max_duration}s max, '
            f'speed={self.forward_speed}, turn={self.turn_speed}, '
            f'obstacle_thresh={self.obstacle_threshold}m, '
            f'geofence={self.geofence_radius}m')

    # ------------------------------------------------------------------ #
    # Sensing
    # ------------------------------------------------------------------ #
    def scan_cb(self, msg):
        ranges = list(msg.ranges)
        if not ranges:
            return

        n = len(ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        lo = msg.range_min
        hi = msg.range_max

        def sector(start_deg, end_deg):
            """Minimum valid range between two bearings (deg, CCW, 0 = forward)."""
            best = math.inf
            for i, r in enumerate(ranges):
                if not (lo < r < hi):
                    continue
                bearing = math.degrees(angle_min + (angle_max - angle_min) * i / n)
                # Normalise into [-180, 180]
                bearing = (bearing + 180.0) % 360.0 - 180.0
                if start_deg <= bearing < end_deg and r < best:
                    best = r
            return best

        # Front spans -25..+25 deg, diagonal quadrants 25..70 deg.
        self.front = sector(-25.0, 25.0)
        self.front_left = sector(25.0, 70.0)
        self.front_right = sector(-70.0, -25.0)
        self.left = sector(70.0, 135.0)
        self.right = sector(-135.0, -70.0)
        # Rear is the narrow 140..180 / -180..-140 wedge directly behind us.
        # Kept narrow on purpose: a wide rear wedge catches the diagonal
        # obstacles we are free to reverse past, which would block the
        # escape manoeuvre unnecessarily.
        self.rear = min(sector(140.0, 180.0), sector(-180.0, -140.0))

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def nearest(self, a, b):
        finite = [v for v in (a, b) if math.isfinite(v)]
        return min(finite) if finite else math.inf

    def is_blocked(self, clearance, margin=0.0):
        return clearance < self.obstacle_threshold + margin

    def can_reverse(self):
        """Only reverse when the rear sector is verified clear."""
        return self.rear > self.rear_clear_threshold

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------ #
    # Stuck / escape
    # ------------------------------------------------------------------ #
    def update_stuck(self, dt):
        moved = math.hypot(self.robot_x - self.last_progress_x,
                           self.robot_y - self.last_progress_y)
        if moved < self.stuck_distance_threshold:
            self.stuck_timer += dt
        else:
            self.stuck_timer = 0.0
            self.last_progress_x = self.robot_x
            self.last_progress_y = self.robot_y
        return self.stuck_timer >= self.stuck_timeout

    def start_escape(self):
        """Multi-stage escape: rotate to the open side, back out, realign.

        Rotating in place alone cannot escape a wedge (the odom position never
        changes, so stuck detection re-fires immediately). Backing out is what
        actually creates displacement, so stage 2 reverses - but only when the
        rear LiDAR sector is verified clear, which is what stops us from
        reversing into the cylinder behind us.
        """
        self.escape_mode = True
        self.escape_stage = 0
        self.escape_timer = self.escape_rotate_time

        left_open = self._openness(self.front_left, self.left)
        right_open = self._openness(self.front_right, self.right)

        if abs(left_open - right_open) > 0.3:
            # Turn toward the roomier side (positive z turns left)
            self.escape_turn_sign = 1.0 if left_open > right_open else -1.0
        else:
            # Ambiguous: use the side we already lean toward
            self.escape_turn_sign = self.wall_side

        # Only plan a reversing phase if the rear is genuinely clear.
        self.escape_reversing = self.can_reverse()

        self.get_logger().warn(
            f'Stuck at ({self.robot_x:.2f}, {self.robot_y:.2f}) - escaping: '
            f'turn={self.escape_turn_sign:+.0f}, rear={self.rear:.2f}m, '
            f'reversing={self.escape_reversing}')

    def _openness(self, diagonal, lateral):
        """Combine diagonal and lateral clearance into a comparable score."""
        diag = diagonal if math.isfinite(diagonal) else 5.0
        lat = lateral if math.isfinite(lateral) else 5.0
        return min(diag, lat)

    # ------------------------------------------------------------------ #
    # Control
    # ------------------------------------------------------------------ #
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now
        # Guard against the sim clock stalling or jumping
        dt = max(0.0, min(dt, 0.5))

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed >= self.max_duration:
            self.stop_robot()
            self.timer.cancel()
            self.get_logger().info(f'Exploration complete after {elapsed:.1f}s')
            return

        cmd = Twist()

        # 1. Escape mode takes priority over all other behaviour.
        if self.escape_mode:
            self.escape_timer -= dt

            if self.escape_stage == 0:
                # Stage 1: rotate toward the open side, in place.
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_turn_sign * self.turn_speed
                if self.escape_timer <= 0:
                    self.escape_stage = 1
                    if self.escape_reversing:
                        self.escape_timer = self.escape_reverse_time
                    else:
                        # No room behind - go straight to realignment.
                        self.escape_timer = self.escape_final_rotate_time

            elif self.escape_stage == 1:
                # Stage 2: back out while continuing to turn. This is the only
                # stage that produces translation, so it is what actually
                # breaks a wedge. Re-check the rear sector continuously: if
                # something closes in behind, stop reversing immediately.
                if self.can_reverse():
                    cmd.linear.x = -0.15
                    cmd.angular.z = self.escape_turn_sign * self.turn_speed * 0.7
                else:
                    # Rear became blocked - halt and just keep turning.
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.escape_turn_sign * self.turn_speed

                if self.escape_timer <= 0:
                    self.escape_stage = 2
                    self.escape_timer = self.escape_final_rotate_time

            else:
                # Stage 3: realign perpendicular to the obstacle and commit to
                # driving away so we do not immediately re-wedge.
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_turn_sign * self.turn_speed
                if self.escape_timer <= 0:
                    self.escape_mode = False
                    self.stuck_timer = 0.0
                    self.last_progress_x = self.robot_x
                    self.last_progress_y = self.robot_y
                    # Hug whichever side just opened up.
                    self.wall_side = 1.0 if self._openness(self.front_left, self.left) > \
                        self._openness(self.front_right, self.right) else -1.0
                    # Commit to driving forward for a moment without
                    # re-triggering stuck detection.
                    self.commit_timer = 3.0
                    self.commit_speed_scale = 1.0

            self.cmd_pub.publish(cmd)
            return

        # 2. Geofence: turn back toward the arena centre.
        dist_from_origin = math.hypot(self.robot_x, self.robot_y)
        if dist_from_origin > self.geofence_radius:
            angle_to_origin = math.atan2(-self.robot_y, -self.robot_x)
            angle_diff = self.normalize_angle(angle_to_origin - self.robot_yaw)
            if abs(angle_diff) > 0.3:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if angle_diff > 0 else -self.turn_speed
                self.cmd_pub.publish(cmd)
                return

        # 3. Stuck detection.
        if self.update_stuck(dt):
            self.start_escape()
            cmd.angular.z = self.escape_turn_sign * self.turn_speed
            self.cmd_pub.publish(cmd)
            return

        # 4. Wall-following coverage.
        wall_side_clear = self.left if self.wall_side > 0 else self.right
        wall_side_diag = self.front_left if self.wall_side > 0 else self.front_right
        other_side_diag = self.front_right if self.wall_side > 0 else self.front_left

        if self.is_blocked(self.front):
            # Front blocked. Rotate toward the open side. If we are wedged
            # (front and both diagonals all closed) reversing is the only way
            # out, so hand over to the escape sequencer.
            turn = self._choose_escape_turn()
            if self.is_blocked(self.front, margin=0.15) and \
               self.is_blocked(self.front_left) and self.is_blocked(self.front_right):
                self.escape_turn_sign = turn
                self.start_escape()
                cmd.angular.z = turn * self.turn_speed
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = turn * self.turn_speed

        elif self.is_blocked(wall_side_diag) and self.is_blocked(wall_side_clear):
            # Wall side closed off - corner. Peel away from the wall.
            cmd.linear.x = self.forward_speed * 0.4
            cmd.angular.z = -self.wall_side * self.turn_speed

        elif math.isfinite(wall_side_clear) and wall_side_clear < 1.4:
            # Normal wall following: hug the wall, drifting toward it.
            if math.isfinite(wall_side_diag):
                cmd.linear.x = self.forward_speed * 0.8
                cmd.angular.z = self.wall_side * self.turn_speed * 0.55
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = self.wall_side * self.turn_speed * 0.3
            # Wall is close: keep tracking it
            self.wall_lost_ticks = 0.0

        else:
            # Wall lost. Search for it in the hugging direction instead of
            # driving straight - this is what sweeps out new area.
            self.wall_lost_ticks += dt
            if self.wall_lost_ticks > self.wall_lost_timeout:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = self.wall_side * self.turn_speed * 0.6
                if self.wall_lost_ticks > self.wall_lost_timeout + 3.0:
                    # Nothing found at all; flip to the other side
                    self.wall_side *= -1
                    self.wall_lost_ticks = 0.0
            else:
                cmd.linear.x = self.forward_speed
                if math.isfinite(other_side_diag) and self.is_blocked(other_side_diag):
                    cmd.angular.z = -self.wall_side * self.turn_speed * 0.5
                else:
                    cmd.angular.z = 0.1 * math.sin(elapsed * 0.7)

        # Last-resort collision guard: never command forward into a closed sector.
        if cmd.linear.x > 0 and self.is_blocked(self.front, margin=0.1):
            cmd.linear.x = 0.0

        # Honour the post-escape commitment window.
        if self.commit_timer > 0:
            self.commit_timer -= dt
            if self.is_blocked(self.front, margin=0.05):
                self.commit_timer = 0.0
            else:
                cmd.linear.x = self.forward_speed * self.commit_speed_scale

        self.cmd_pub.publish(cmd)

    def _choose_escape_turn(self):
        """Pick the turn sign that heads toward more open space."""
        left = self._openness(self.front_left, self.left)
        right = self._openness(self.front_right, self.right)

        # If one side is clearly roomier, commit to it.
        if left > right + 0.4:
            return 1.0
        if right > left + 0.4:
            return -1.0
        # Ambiguous: keep turning the way we already lean so we break out
        # rather than dithering between the two options.
        return self.wall_side


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()