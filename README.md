# copy
```bash
#!/usr/bin/env python3
"""
Copyright (c) 2026 Duy Pham
All rights reserved.

Author: Duy Pham
Contact: duypham.robotics@gmail.com
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TurnController(Node):
    def __init__(self):
        super().__init__("turn_controller")

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            "cmd_vel",
            10
        )

        self.frame_id = "base_link"

        self.get_logger().info("TurnController node started")

    def turn_left(self, linear_speed: float = 0.0, angular_speed: float = 1.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.twist.linear.x = linear_speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular_speed  # +z = rẽ trái (CCW)

        self.cmd_vel_pub.publish(msg)

    def turn_right(self, linear_speed: float = 0.0, angular_speed: float = 1.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.twist.linear.x = linear_speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = -angular_speed  # -z = rẽ phải (CW)

        self.cmd_vel_pub.publish(msg)

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurnController()

    try:
        # Demo: rẽ trái 3 giây → dừng → rẽ phải 3 giây
        node.get_logger().info("Turning left...")
        start = node.get_clock().now().seconds_nanoseconds()[0]
        while rclpy.ok():
            now = node.get_clock().now().seconds_nanoseconds()[0]
            if now - start > 3:
                break
            node.turn_left(0.0, 1.0)
            rclpy.spin_once(node, timeout_sec=0.1)

        node.stop()

        node.get_logger().info("Turning right...")
        start = node.get_clock().now().seconds_nanoseconds()[0]
        while rclpy.ok():
            now = node.get_clock().now().seconds_nanoseconds()[0]
            if now - start > 3:
                break
            node.turn_right(0.0, 1.0)
            rclpy.spin_once(node, timeout_sec=0.1)

        node.stop()

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


    return msg


```
