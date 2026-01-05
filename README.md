# copy
```bash
    def update_odom(self, n_fr, n_fl, n_rr, n_rl):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. Đổi RPM sang m/s (v = omega * r)
        # 1 vòng/phút = 2*pi/60 rad/s
        scale = (2 * math.pi * self.r) / 60.0
        v_fr = n_fr * scale
        v_fl = n_fl * scale
        v_rr = n_rr * scale
        v_rl = n_rl * scale

        # 2. Động học thuận Mecanum (Forward Kinematics)
        # Công thức chuẩn (O-rectangle layout)
        vx = (v_fr + v_fl + v_rr + v_rl) / 4.0
        vy = (v_fr - v_fl + v_rr - v_rl) / 4.0
        wz = (v_fr - v_fl - v_rr + v_rl) / (4.0 * self.geo_factor)

        # 3. Tích phân vị trí (Dead Reckoning)
        # Tính quãng đường di chuyển trong hệ toạ độ robot
        dx_robot = vx * dt
        dy_robot = vy * dt
        dth = wz * dt

        # Chiếu sang hệ toạ độ bản đồ (Map/Odom frame)
        # Phải xoay theo hướng robot đang đứng (self.th)
        delta_x = dx_robot * math.cos(self.th) - dy_robot * math.sin(self.th)
        delta_y = dx_robot * math.sin(self.th) + dy_robot * math.cos(self.th)
        
        # Cộng dồn vị trí
        self.x += delta_x
        self.y += delta_y
        self.th += dth

        # 4. Đóng gói tin nhắn ROS
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Vị trí
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = self.euler_to_quaternion(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Vận tốc
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        # Thiết lập độ tin cậy (Covariance) - Rất quan trọng cho EKF
        # [0]=x, [7]=y, [35]=yaw
        odom_msg.pose.covariance[0] = 0.01  
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[35] = 0.03 

        self.odom_pub.publish(odom_msg)
        #print(f"self.x = {self.x} | self.y = {self.y} | q = {q}")
```
