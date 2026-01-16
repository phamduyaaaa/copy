# copy
```bash
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np
import math


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== CONTROL PARAM =====
        self.lookahead_ratio = 0.6   # bỏ vùng mù
        self.kp = 0.004
        self.kd = 0.001
        self.prev_error = 0.0

        self.v = 0.15
        self.max_w = 0.8

        self.get_logger().info("Line follower with visualization started")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        h, w, _ = frame.shape

        # ================= ROI =================
        roi = frame[int(h*0.5):h, :]

        # ================= BEV =================
        src = np.float32([
            [w*0.3, 0],
            [w*0.7, 0],
            [w, h*0.5],
            [0, h*0.5]
        ])

        dst = np.float32([
            [0, 0],
            [400, 0],
            [400, 200],
            [0, 200]
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(roi, M, (400, 200))

        # ================= THRESHOLD =================
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # ================= LOOKAHEAD =================
        look_y = int(binary.shape[0] * self.lookahead_ratio)
        row = binary[look_y-5:look_y+5, :]
        xs = np.where(row > 0)[1]

        twist = Twist()

        if len(xs) > 0:
            center_x = int(xs.mean())
            error = center_x - (binary.shape[1] // 2)

            d_error = error - self.prev_error
            w_cmd = self.kp * error + self.kd * d_error
            w_cmd = np.clip(w_cmd, -self.max_w, self.max_w)

            twist.linear.x = self.v
            twist.angular.z = -w_cmd

            self.prev_error = error

            # Vẽ debug
            cv2.circle(bev, (center_x, look_y), 6, (0, 0, 255), -1)
            cv2.line(bev,
                     (binary.shape[1]//2, binary.shape[0]),
                     (center_x, look_y),
                     (255, 0, 0), 2)

        else:
            # mất line → quay chậm tìm
            twist.linear.x = 0.05
            twist.angular.z = 0.4

        self.pub.publish(twist)

        # ================= SHOW =================
        cv2.imshow("raw", frame)
        cv2.imshow("bev", bev)
        cv2.imshow("binary", binary)

        cv2.waitKey(1)


def main():
    rclpy.init()
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
