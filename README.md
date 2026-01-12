# copy
```bash
"""
Copyright (c) 2026 Duy Pham
All rights reserved.

Author: Duy Pham
Contact: duypham.robotics@gmail.com
"""

from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock


def turn_left(
    linear_speed: float = 0.0,
    angular_speed: float = 1.0,
    frame_id: str = "base_link"
) -> TwistStamped:
    """
    Robot rẽ trái (CCW)
    """
    msg = TwistStamped()
    msg.header.stamp = Clock().now().to_msg()
    msg.header.frame_id = frame_id

    msg.twist.linear.x = linear_speed
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0

    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = angular_speed  # +z = trái

    return msg


def turn_right(
    linear_speed: float = 0.0,
    angular_speed: float = 1.0,
    frame_id: str = "base_link"
) -> TwistStamped:
    """
    Robot rẽ phải (CW)
    """
    msg = TwistStamped()
    msg.header.stamp = Clock().now().to_msg()
    msg.header.frame_id = frame_id

    msg.twist.linear.x = linear_speed
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0

    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = -angular_speed  # -z = phải

    return msg


```
