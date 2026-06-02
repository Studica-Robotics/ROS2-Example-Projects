#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DiffDriveNode(Node):
    """Maps /cmd_vel to per-motor duty cycle commands on the configured Titan topics.

    Each wheel position independently specifies its Titan instance and port so motors
    can be spread across multiple controllers without any code changes.

    cmd_vel callback only stores the latest values.  A fixed-rate timer drains
    them to the Titan so the CAN bus is never flooded faster than it can process
    commands (50 Hz max).  Jerk limiting ramps the current duty cycle toward the
    target at most accel_rate per tick.
    """

    def __init__(self):
        super().__init__('diff_drive')

        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            self.declare_parameter(f'{pos}.titan', 'drive')
            self.declare_parameter(f'{pos}.port',  0)

        self.declare_parameter('max_linear',    1.0)
        self.declare_parameter('max_angular',   2.0)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.declare_parameter('publish_rate', 50) # 50 hz

        # Jerk limiting — max duty cycle change per timer tick.
        # At 50 Hz: 0.05 → full speed in 400 ms.  Set to 1.0 to disable.
        self.declare_parameter('accel_rate', 0.05)

        self._max_linear  = self.get_parameter('max_linear').value
        self._max_angular = self.get_parameter('max_angular').value
        self._accel_rate  = self.get_parameter('accel_rate').value
        publish_rate      = self.get_parameter('publish_rate').value

        self._motor_pubs = {}
        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            titan = self.get_parameter(f'{pos}.titan').value
            port  = self.get_parameter(f'{pos}.port').value
            topic = f'/{titan}/m_{port}/cmd'
            self._motor_pubs[pos] = self.create_publisher(Float64, topic, 1)
            self.get_logger().info(f'  {pos:12s} -> {topic}')

        # Latest cmd_vel (stored by callback, consumed by timer)
        self._latest_linear  = 0.0
        self._latest_angular = 0.0

        # Ramped duty cycles currently being sent to motors
        self._current_left  = 0.0
        self._current_right = 0.0

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(Twist, cmd_topic, self._cmd_vel_cb, 1)

        period = 1.0 / publish_rate
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'diff_drive ready  publish_rate={publish_rate} Hz  accel_rate={self._accel_rate:.2f}')

    def _cmd_vel_cb(self, msg: Twist):
        # Store latest velocities — the timer will send them at the fixed rate.
        self._latest_linear  = msg.linear.x
        self._latest_angular = msg.angular.z

    def _on_timer(self):
        # Ramp current duty toward target — limits jerk.
        target_left  = (self._latest_linear / self._max_linear) - (-self._latest_angular / self._max_angular)
        target_right = (self._latest_linear / self._max_linear) + (-self._latest_angular / self._max_angular)

        # Ratio-preserving clamp on target before ramping.
        peak = max(abs(target_left), abs(target_right), 1.0)
        target_left  /= peak
        target_right /= peak

        # Step current duty toward target by at most accel_rate per tick.
        self._current_left  = self._ramp(self._current_left,  target_left)
        self._current_right = self._ramp(self._current_right, target_right)

        self._publish(self._motor_pubs['front_left'],  self._current_left)
        self._publish(self._motor_pubs['rear_left'],   self._current_left)
        self._publish(self._motor_pubs['front_right'], self._current_right)
        self._publish(self._motor_pubs['rear_right'],  self._current_right)

    def _ramp(self, current: float, target: float) -> float:
        diff = target - current
        step = max(-self._accel_rate, min(self._accel_rate, diff))
        return current + step

    def _publish(self, pub, value: float):
        msg = Float64()
        msg.data = max(-1.0, min(1.0, float(value)))
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
