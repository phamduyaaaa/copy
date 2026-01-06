# copy
```bash
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
import rcl_interfaces.msg


class TeleopPS3(Node):

    def __init__(self):
        super().__init__('teleop_ps3')

        # Parameters
        read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)

        self.stamped = self.declare_parameter(
            'stamped', True, read_only).value
        self.frame_id = self.declare_parameter(
            'frame_id', '', read_only).value
        self.speed = self.declare_parameter(
            'speed', 0.5, read_only).value
        self.turn = self.declare_parameter(
            'turn', 1.0, read_only).value

        if not self.stamped and self.frame_id:
            raise RuntimeError(
                "'frame_id' can only be set when 'stamped' is True")

        # Message type
        if self.stamped:
            self.TwistMsg = TwistStamped
        else:
            self.TwistMsg = Twist

        self.pub = self.create_publisher(self.TwistMsg, 'cmd_vel', 10)
        self.sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("Teleop PS3 started")

    def joy_callback(self, joy: Joy):
        msg = self.TwistMsg()

        if self.stamped:
            twist = msg.twist
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
        else:
            twist = msg

        # === AXIS MAPPING (PS3) ===
        # Left stick: linear x/y
        linear_x = joy.axes[1]        # forward/back
        linear_y = joy.axes[0]        # left/right

        # Right stick: angular z
        angular_z = joy.axes[3]       # rotate

        twist.linear.x = linear_x * self.speed
        twist.linear.y = linear_y * self.speed
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z * self.turn

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopPS3()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
