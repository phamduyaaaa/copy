# copy
```bash
#!/usr/bin/env python3
"""
STM32 FULL BRIDGE FOR MECANUM ROBOT
Final Version: Integrated Hard Constraint Filter & Twist Covariance
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Quaternion
import serial
import math
import struct
import time

# ================== UART PROTOCOL ==================
START_BYTE = 0xFF
STOP_BYTE  = 0xFE
FRAME_LEN  = 15
SCALE_CMD  = 100.0 

def calc_checksum_xor(buf):
    cs = buf[1]
    for i in range(2, 13):
        cs ^= buf[i]
    return cs

class STM32BridgeFull(Node):
    def __init__(self):
        super().__init__('stm32_bridge_full')

        # --- PARAMETERS ---
        self.declare_parameter('port', '/dev/ttyUSB0')
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

        # --- SERIAL CONNECTION ---
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info(f"Connected to STM32 at {port}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial Error: {e}")
            exit(1)

        # --- ROS COMM ---
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub  = self.create_publisher(Imu, '/imu/data', 10)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- STATE ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.rx_buffer = bytearray()
        
        # Timers
        self.create_timer(0.02, self.read_uart) # 50Hz reading
        self.last_cmd_time = time.time()
        self.create_timer(0.1, self.watchdog_callback)

    # --- TX ---
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        az = msg.twist.angular.z

        vx_i = int(vx * SCALE_CMD)
        vy_i = int(vy * SCALE_CMD)
        az_i = int(az * SCALE_CMD)

        vx_i = max(-32768, min(32767, vx_i))
        vy_i = max(-32768, min(32767, vy_i))
        az_i = max(-32768, min(32767, az_i))

        payload = bytearray()
        for val in (vx_i, vy_i, az_i):
            payload.append((val >> 8) & 0xFF)
            payload.append(val & 0xFF)

        checksum = 0
        for b in payload:
            checksum ^= b

        frame = bytearray()
        frame.append(START_BYTE)
        frame.extend(payload)
        frame.append(checksum)
        frame.append(STOP_BYTE)
        self.ser.write(frame)

    def watchdog_callback(self):
        if time.time() - self.last_cmd_time > 0.5:
            # Send stop command if no cmd_vel received
            payload = bytearray([0,0, 0,0, 0,0])
            frame = bytearray([START_BYTE]) + payload + bytearray([0, STOP_BYTE])
            self.ser.write(frame)

    # --- RX ---
    def read_uart(self):
        try:
            if self.ser.in_waiting > 0:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

            while len(self.rx_buffer) >= FRAME_LEN:
                if self.rx_buffer[0] != START_BYTE:
                    self.rx_buffer.pop(0)
                    continue

                frame = self.rx_buffer[:FRAME_LEN]
                if frame[14] != STOP_BYTE:
                    self.rx_buffer.pop(0)
                    continue

                if calc_checksum_xor(frame) != frame[13]:
                    del self.rx_buffer[0]
                    continue

                parsed = self.parse_frame(frame)
                self.update_odom(parsed)
                self.update_imu(parsed["GOC"])
                del self.rx_buffer[:FRAME_LEN]

        except Exception as e:
            self.get_logger().error(f"UART RX Error: {e}")
            self.ser.reset_input_buffer()

    def parse_frame(self, buf):
        FR = struct.unpack(">h", bytes(buf[1:3]))[0]
        FL = struct.unpack(">h", bytes(buf[3:5]))[0]
        RR = struct.unpack(">h", bytes(buf[5:7]))[0]
        RL = struct.unpack(">h", bytes(buf[7:9]))[0]
        GOC = struct.unpack(">i", bytes(buf[9:13]))[0]
        return {"FR": FR, "FL": FL, "RR": RR, "RL": RL, "GOC": GOC}

    def update_odom(self, data):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. Tính toán vận tốc bánh xe (m/s)
        scale = (2 * math.pi * self.r) / 60.0
        v_fr = data["FR"] * scale
        v_fl = data["FL"] * scale
        v_rr = data["RR"] * scale
        v_rl = data["RL"] * scale

        # 2. Tính toán động học (Kinematics)
        vx = (v_fr + v_fl + v_rr + v_rl) / 4.0
        vy = (v_fr - v_fl + v_rr - v_rl) / 4.0
        wz = (v_fr - v_fl - v_rr + v_rl) / (4.0 * self.geo_factor)

        # ==========================================================
        # 3. BỘ LỌC CƯỠNG BỨC (HARD CONSTRAINT FILTER)
        # ==========================================================
        # Mục tiêu: Loại bỏ vận tốc tịnh tiến "ảo" khi robot xoay tại chỗ
        
        # Nếu robot đang xoay (ngưỡng 0.08 rad/s ~ 4.5 độ/s)
        if abs(wz) > 0.08:
            # Mà vận tốc thẳng tính ra quá nhỏ (< 8cm/s)
            # -> Coi như là nhiễu do trượt bánh -> Gán về 0
            if abs(vx) < 0.08: vx = 0.0
            if abs(vy) < 0.08: vy = 0.0

        # Lọc nhiễu khi đứng yên (Deadzone)
        if abs(vx) < 0.005 and abs(vy) < 0.005 and abs(wz) < 0.01:
            vx = 0.0
            vy = 0.0
            wz = 0.0
        # ==========================================================

        # 4. Tích phân (chỉ để debug, EKF không dùng x,y này)
        dx = vx * dt
        dy = vy * dt
        dth = wz * dt
        self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
        self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
        self.th += dth

        # 5. Đóng gói tin nhắn ROS
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Position (EKF sẽ bỏ qua, nhưng Rviz có thể xem)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.th)

        # Velocity (EKF dùng cái này - Đã được lọc ở bước 3)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # === COVARIANCE (Độ tin cậy) ===
        # Pose (X, Y, Yaw)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.1 # encoder yaw kém tin cậy

        # Twist (Vx, Vy, Vz) - Bắt buộc cho Mecanum
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.02
        
        self.odom_pub.publish(odom)

    def update_imu(self, goc_raw):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Chuyển đổi đơn vị
        yaw_deg = - goc_raw / 10.0 
        yaw_rad = yaw_deg * (math.pi / 180.0)

        imu_msg.orientation = self.euler_to_quaternion(0, 0, yaw_rad)
        
        # Covariance IMU: Tin tuyệt đối vào Yaw (9999 cho các trục khác)
        imu_msg.orientation_covariance = [9999.0, 0.0, 0.0, 
                                          0.0, 9999.0, 0.0, 
                                          0.0, 0.0, 0.001]
        
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        
        self.imu_pub.publish(imu_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeFull()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
