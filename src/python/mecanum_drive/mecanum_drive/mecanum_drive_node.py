#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class MecanumDriveNode(Node):
    """Maps /cmd_vel to per-motor duty cycle commands for a 4-wheel mecanum drive.

    Holonomic kinematics allow simultaneous forward, strafe, and rotation.
    Each wheel independently specifies its Titan instance and port so motors
    can be spread across multiple controllers without any code changes.

    cmd_vel callback only stores the latest values.  A fixed-rate timer drains
    them to the Titan so the CAN bus is never flooded faster than it can process
    commands (50 Hz max).  Jerk limiting ramps the normalised inputs (vx, vy, wz)
    before computing per-wheel kinematics so holonomic ratios are preserved
    throughout the acceleration phase — the robot always moves in the correct
    direction during a ramp.

    Wheel equations (unit duty cycle):
      front_left  = vx - vy - wz
      front_right = vx + vy + wz
      rear_left   = vx + vy - wz
      rear_right  = vx - vy + wz

    All four outputs are scaled together so the dominant axis is never clipped.
    """

    def __init__(self):
        super().__init__('mecanum_drive')

        for pos in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            self.declare_parameter(f'{pos}.titan', 'drive')
            self.declare_parameter(f'{pos}.port',  0)

        self.declare_parameter('max_linear',    1.0)
        self.declare_parameter('max_angular',   2.0)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.declare_parameter('publish_rate', 50) # 50 Hz

        # Jerk limiting — max normalised input change per timer tick (0.0–1.0).
        # Applied to vx/vy/wz inputs before kinematics so holonomic ratios are preserved.
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

        # Latest cmd_vel input (stored by callback, consumed by timer)
        self._latest_vx = 0.0
        self._latest_vy = 0.0
        self._latest_wz = 0.0

        # Ramped normalised values currently being sent to motors
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_wz = 0.0

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(Twist, cmd_topic, self._cmd_vel_cb, 1)

        period = 1.0 / publish_rate
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'mecanum_drive ready  publish_rate={publish_rate} Hz  accel_rate={self._accel_rate:.2f}')

    def _cmd_vel_cb(self, msg: Twist):
        # Store latest velocities — the timer will send them at the fixed rate.
        self._latest_vx = msg.linear.x
        self._latest_vy = msg.linear.y
        self._latest_wz = msg.angular.z

    def _on_timer(self):
        # Ramp inputs toward target — preserves holonomic kinematics during acceleration.
        self._current_vx = self._ramp(self._current_vx, self._latest_vx / self._max_linear)
        self._current_vy = self._ramp(self._current_vy, self._latest_vy / self._max_linear)
        self._current_wz = self._ramp(self._current_wz, self._latest_wz / self._max_angular)

        # Mecanum holonomic kinematics
        fl = self._current_vx - self._current_vy - self._current_wz
        fr = self._current_vx + self._current_vy + self._current_wz
        rl = self._current_vx + self._current_vy - self._current_wz
        rr = self._current_vx - self._current_vy + self._current_wz

        # Ratio-preserving clamp: scale all four together so no axis is lost.
        scale = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        fl /= scale
        fr /= scale
        rl /= scale
        rr /= scale

        self._publish(self._motor_pubs['front_left'],  fl)
        self._publish(self._motor_pubs['front_right'], fr)
        self._publish(self._motor_pubs['rear_left'],   rl)
        self._publish(self._motor_pubs['rear_right'],  rr)

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
    node = MecanumDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
