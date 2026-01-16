# copy
```bash
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_slow_precise')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ================= PHYSICAL PARAM =================
        self.LOOKAHEAD_CM = 45.0        # > 34 cm vùng mù
        self.CM_TO_PIXEL = 2.0          # 100 cm → 200 px BEV
        self.lookahead_px = int(self.LOOKAHEAD_CM * self.CM_TO_PIXEL)

        # ================= CONTROL =================
        self.v = 0.10                   # CHẠY CHẬM
        self.kp = 0.003
        self.kd = 0.001
        self.prev_error = 0.0
        self.max_w = 0.5

        self.get_logger().info("Line follower (slow & precise) started")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape

        # ================= ROI =================
        roi = frame[int(h * 0.5):h, :]

        # ================= BEV =================
        # ⚠️ BẠN CHỈ CẦN CHỈNH 4 ĐIỂM NÀY
        src = np.float32([
            [w * 0.30, 0],
            [w * 0.70, 0],
            [w * 0.95, h * 0.5],
            [w * 0.05, h * 0.5]
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

        # ================= LOOK-AHEAD =================
        look_y = binary.shape[0] - self.lookahead_px
        look_y = np.clip(look_y, 20, binary.shape[0] - 20)

        row = binary[look_y-4:look_y+4, :]
        xs = np.where(row > 0)[1]

        twist = Twist()

        if len(xs) > 0:
            center_x = int(xs.mean())
            error = center_x - (binary.shape[1] // 2)

            d_error = error - self.prev_error
            w = self.kp * error + self.kd * d_error
            w = np.clip(w, -self.max_w, self.max_w)

            twist.linear.x = self.v
            twist.angular.z = -w
            self.prev_error = error

            # ===== DEBUG DRAW =====
            cv2.circle(bev, (center_x, look_y), 6, (0, 0, 255), -1)
            cv2.line(
                bev,
                (binary.shape[1] // 2, binary.shape[0]),
                (center_x, look_y),
                (255, 0, 0),
                2
            )

        else:
            # Mất line → quay tìm rất chậm
            twist.linear.x = 0.04
            twist.angular.z = 0.3

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
