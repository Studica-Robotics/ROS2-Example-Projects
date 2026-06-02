#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf2_ros


class OdometryNode(Node):
    """Fuses mecanum wheel encoder data with IMU heading to publish odometry.

    Position (x, y) is integrated from encoder distance deltas via mecanum
    inverse kinematics.

    Velocity is split into independent sources for the EKF:
      - twist.linear.x / .y — from RPM via mecanum inverse kinematics (no dt noise)
      - twist.angular.z     — from all-wheel RPM differential via track_width and
                              wheel_base (encoder-derived, independent of IMU)

    Publishing encoder angular velocity alongside the IMU source lets the EKF
    cross-check them — divergence reveals wheel slip or IMU drift.

    Mecanum inverse kinematics (positive = wheel moves forward in body frame):
      dx_body = (fl + fr + rl + rr) / 4
      dy_body = (-fl + fr + rl - rr) / 4
      ω       = (-fl + fr - rl + rr) / (2 * (track_width + wheel_base))

    Requires:
      - Titan motors in Quadrature encoder mode, dist_per_tick set in
        studica_control so encoder distance topics publish metres.
      - invert_encoder: true for front_right and rear_right in studica_control.
      - studica_control IMU component enabled.
    """

    def __init__(self):
        super().__init__('odometry')

        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            self.declare_parameter(f'{pos}.titan', 'drive')
            self.declare_parameter(f'{pos}.port',  0)

        # wheel_radius and gear_ratio convert RPM → m/s.
        self.declare_parameter('wheel_radius', 0.076)   # metres — measure your wheel
        self.declare_parameter('gear_ratio',   1.0)     # motor_turns / wheel_turns
        self.declare_parameter('track_width',  0.3)     # metres — left-to-right wheel centre distance
        self.declare_parameter('wheel_base',   0.3)     # metres — front-to-rear wheel centre distance
        self.declare_parameter('imu_topic',    'imu')
        self.declare_parameter('odom_topic',   'odom')
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('publish_tf',   False)

        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._gear_ratio   = self.get_parameter('gear_ratio').value
        self._track_width  = self.get_parameter('track_width').value
        self._wheel_base   = self.get_parameter('wheel_base').value
        # RPM (rev/min) → m/s: multiply by (2π × r) / 60
        self._rpm_to_ms    = (2.0 * math.pi * self._wheel_radius) / (60.0 * self._gear_ratio)
        # Denominator for encoder-derived angular velocity (metres)
        self._ang_denom    = 2.0 * (self._track_width + self._wheel_base)
        self._odom_frame   = self.get_parameter('odom_frame').value
        self._base_frame   = self.get_parameter('base_frame').value
        self._publish_tf   = self.get_parameter('publish_tf').value

        # encoder distance (for position integration)
        self._enc_dist = {pos: None for pos in ('front_left', 'front_right', 'rear_left', 'rear_right')}
        self._enc_prev = {pos: None for pos in ('front_left', 'front_right', 'rear_left', 'rear_right')}

        # RPM (for velocity output)
        self._rpm = {pos: None for pos in ('front_left', 'front_right', 'rear_left', 'rear_right')}

        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            titan = self.get_parameter(f'{pos}.titan').value
            port  = self.get_parameter(f'{pos}.port').value
            enc_topic = f'/{titan}/m_{port}/encoder'
            rpm_topic = f'/{titan}/m_{port}/rpm'
            self.create_subscription(Float64, enc_topic,
                lambda msg, p=pos: self._encoder_cb(msg, p), 10)
            self.create_subscription(Float64, rpm_topic,
                lambda msg, p=pos: self._rpm_cb(msg, p), 10)
            self.get_logger().info(f'  {pos:12s} -> {enc_topic}  {rpm_topic}')

        self._imu_yaw        = None
        self._imu_yaw_offset = None

        imu_topic = self.get_parameter('imu_topic').value
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.get_logger().info(f'  imu          -> /{imu_topic}')

        self._x = 0.0
        self._y = 0.0

        odom_topic = self.get_parameter('odom_topic').value
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(0.05, self._odom_update)

        self.get_logger().info(
            f'odometry ready  wheel_radius={self._wheel_radius:.4f}m  '
            f'track_width={self._track_width:.3f}m  wheel_base={self._wheel_base:.3f}m')

    # callbacks

    def _encoder_cb(self, msg: Float64, pos: str):
        self._enc_dist[pos] = msg.data

    def _rpm_cb(self, msg: Float64, pos: str):
        self._rpm[pos] = msg.data

    def _imu_cb(self, msg: Imu):
        yaw = _quaternion_to_yaw(msg.orientation)
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = yaw
        self._imu_yaw = yaw - self._imu_yaw_offset

    # odometry timer

    def _odom_update(self):
        if self._imu_yaw is None:
            return

        now = self.get_clock().now()

        # Seed prev on first encoder reading, skip this tick.
        seeding = False
        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            if self._enc_dist[pos] is not None and self._enc_prev[pos] is None:
                self._enc_prev[pos] = self._enc_dist[pos]
                seeding = True
        if seeding:
            return

        deltas = {}
        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            curr = self._enc_dist[pos]
            prev = self._enc_prev[pos]
            deltas[pos] = (curr - prev) if (curr is not None and prev is not None) else None

        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            if self._enc_dist[pos] is not None:
                self._enc_prev[pos] = self._enc_dist[pos]

        if any(v is None for v in deltas.values()):
            return

        # Position: mecanum inverse kinematics on encoder distance deltas.
        fl, fr = deltas['front_left'], deltas['front_right']
        rl, rr = deltas['rear_left'],  deltas['rear_right']

        dx_body = (fl + fr + rl + rr) / 4.0
        dy_body = (-fl + fr + rl - rr) / 4.0

        cos_y = math.cos(self._imu_yaw)
        sin_y = math.sin(self._imu_yaw)
        self._x += dx_body * cos_y - dy_body * sin_y
        self._y += dx_body * sin_y + dy_body * cos_y

        # Velocity: mecanum inverse kinematics on RPM — no dt division.
        rpm = self._rpm
        rpm_ready = all(rpm[p] is not None for p in ('front_left', 'front_right', 'rear_left', 'rear_right'))
        if rpm_ready:
            vfl = rpm['front_left']  * self._rpm_to_ms
            vfr = rpm['front_right'] * self._rpm_to_ms
            vrl = rpm['rear_left']   * self._rpm_to_ms
            vrr = rpm['rear_right']  * self._rpm_to_ms
            vx = (vfl + vfr + vrl + vrr) / 4.0
            vy = (-vfl + vfr + vrl - vrr) / 4.0
            # Encoder-derived angular velocity — independent of IMU for EKF cross-check.
            vz = (-vfl + vfr - vrl + vrr) / self._ang_denom
        else:
            vx, vy, vz = 0.0, 0.0, 0.0

        q     = _yaw_to_quaternion(self._imu_yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp          = stamp
        odom.header.frame_id       = self._odom_frame
        odom.child_frame_id        = self._base_frame
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = vz

        # Pose covariance — diagonal: x, y, z, roll, pitch, yaw
        odom.pose.covariance[0]  = 0.01   # x  (σ = 10 cm)
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw (σ ≈ 13°)

        # Twist covariance — diagonal: vx, vy, vz, vroll, vpitch, vyaw
        odom.twist.covariance[0]  = 0.001  # vx
        odom.twist.covariance[7]  = 0.001  # vy (lateral — meaningful on mecanum)
        odom.twist.covariance[35] = 0.001  # vyaw (wheel differential)

        self._odom_pub.publish(odom)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp            = stamp
            t.header.frame_id         = self._odom_frame
            t.child_frame_id          = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation      = q
            self._tf_broadcaster.sendTransform(t)


# helpers

def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
