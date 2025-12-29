# copy
```bash
def update_imu(self, goc_raw):
        # 1. Khởi tạo message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link" # Vì em không có TF imu_link riêng

        # ------------------------------------------------------------------
        # 2. XỬ LÝ GÓC QUAY (ORIENTATION)
        # ------------------------------------------------------------------
        # Đổi từ (Độ * 10) sang Radian
        # Ví dụ: 905 -> 90.5 độ -> 1.57 rad
        yaw_deg = goc_raw / 10.0
        yaw_rad = yaw_deg * (math.pi / 180.0)

        # [CẢNH BÁO CHIỀU QUAY]:
        # ROS quy định: Quay trái (ngược chiều kim đồng hồ) là DƯƠNG (+).
        # Nếu STM32 của em quay trái mà giá trị GIẢM, em phải thêm dấu trừ:
        # yaw_rad = -1.0 * yaw_deg * (math.pi / 180.0)

        # Đổi sang Quaternion (Giả sử Roll=0, Pitch=0 vì xe đi trên sàn phẳng)
        q = self.euler_to_quaternion(0, 0, yaw_rad)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # COVARIANCE CHO ORIENTATION (Quan trọng!)
        # Ma trận 3x3 trải phẳng [VarRoll, 0, 0, 0, VarPitch, 0, 0, 0, VarYaw]
        # - Roll, Pitch: Ta giả định bằng 0 nhưng không chắc, set số LỚN (9999) để EKF ít tin.
        # - Yaw: Ta có dữ liệu đo, set số NHỎ (0.01) để EKF tin tưởng tuyệt đối.
        imu_msg.orientation_covariance = [
            9999.0, 0.0, 0.0,
            0.0, 9999.0, 0.0,
            0.0, 0.0, 0.01 
        ]

        # ------------------------------------------------------------------
        # 3. XỬ LÝ VẬN TỐC GÓC & GIA TỐC (Dữ liệu thiếu)
        # ------------------------------------------------------------------
        # Quy tắc ROS: Nếu không có dữ liệu, set phần tử đầu tiên của Covariance là -1
        
        # Không có vận tốc góc (Gyro)
        imu_msg.angular_velocity_covariance[0] = -1.0

        # Không có gia tốc (Accel)
        imu_msg.linear_acceleration_covariance[0] = -1.0

        # Gửi đi
        self.imu_pub.publish(imu_msg)
```
