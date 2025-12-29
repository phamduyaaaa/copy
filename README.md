# copy
```bash
#!/usr/bin/env python3
"""
NHẬN DỮ LIỆU STM32 (15 BYTE) + TÍNH ODOM MECANUM
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import math
import struct
import threading
import time

# ================== UART PROTOCOL ==================
START_BYTE = 0xFF
STOP_BYTE  = 0xFE
FRAME_LEN  = 15

# ================== CHECKSUM ==================
def calc_checksum_xor(buf):
    cs = buf[1]
    for i in range(2, 13):
        cs ^= buf[i]
    return cs


class STM32OdomReceiver(Node):
    def __init__(self):
        super().__init__('stm32_odom_receiver')

        # ================== PARAMETERS ==================
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('wheel_radius', 0.048)
        self.declare_parameter('lx', 0.235)
        self.declare_parameter('ly', 0.20)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.r  = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.geo_factor = self.lx + self.ly

        # ================== SERIAL ==================
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"UART connected: {port}")
        except serial.SerialException as e:
            self.get_logger().fatal(str(e))
            exit(1)

        # ================== ROS PUBLISHER ==================
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        # ================== ODOM STATE ==================
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # ================== UART BUFFER ==================
        self.rx_buffer = bytearray()

        # ================== TIMER ==================
        self.create_timer(0.02, self.read_uart)  # 50Hz

    # ================== UART RX ==================
    def read_uart(self):
        try:
            data = self.ser.read(64)
            if not data:
                return

            self.rx_buffer.extend(data)

            while len(self.rx_buffer) >= FRAME_LEN:
                if self.rx_buffer[0] != START_BYTE:
                    self.rx_buffer.pop(0)
                    continue

                frame = self.rx_buffer[:FRAME_LEN]

                if frame[14] != STOP_BYTE:
                    self.rx_buffer.pop(0)
                    continue

                if calc_checksum_xor(frame) != frame[13]:
                    self.get_logger().warn("Checksum error")
                    del self.rx_buffer[0]
                    continue

                # ===== PARSE FRAME =====
                data = self.parse_frame(frame)

                self.update_odom(
                    data["FR"],
                    data["FL"],
                    data["RR"],
                    data["RL"],
                )

                del self.rx_buffer[:FRAME_LEN]

        except Exception as e:
            self.get_logger().error(f"UART error: {e}")

    # ================== PARSE ==================
    def parse_frame(self, buf):
        FR = struct.unpack(">h", bytes(buf[1:3]))[0]
        FL = struct.unpack(">h", bytes(buf[3:5]))[0]
        RR = struct.unpack(">h", bytes(buf[5:7]))[0]
        RL = struct.unpack(">h", bytes(buf[7:9]))[0]
        GOC = struct.unpack(">i", bytes(buf[9:13]))[0]

        return {
            "FR": FR,
            "FL": FL,
            "RR": RR,
            "RL": RL,
            "GOC": GOC
        }

    # ================== ODOM ==================
    def update_odom(self, n_fr, n_fl, n_rr, n_rl):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        scale = (2 * math.pi * self.r) / 60.0
        v_fr = n_fr * scale
        v_fl = n_fl * scale
        v_rr = n_rr * scale
        v_rl = n_rl * scale

        vx = (v_fr + v_fl + v_rr + v_rl) / 4.0
        vy = (v_fr - v_fl + v_rr - v_rl) / 4.0
        wz = (v_fr - v_fl - v_rr + v_rl) / (4.0 * self.geo_factor)

        dx = vx * dt
        dy = vy * dt
        dth = wz * dt

        self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
        self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
        self.th += dth

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = self.euler_to_quaternion(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.03

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
        qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
        qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = STM32OdomReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```
