# copy
```bash
void publish_odom(rclcpp::Time now, double vx, double vy, double wz) {
        // --- 1. TẮT GỬI TF (Để tránh xung đột với EKF) ---
        // (Đoạn này đã xóa hoặc comment biến 't' đi rồi)
        
        // --- 2. CHUẨN BỊ ODOM MESSAGE ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // TÍNH TOÁN QUATERNION TRỰC TIẾP (Thay vì lấy từ t)
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);

        // Gán vị trí
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Gán hướng (Sửa lỗi ở đây: Gán trực tiếp từ q)
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Gán vận tốc
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        // --- 3. DYNAMIC COVARIANCE (CHỐNG TRÔI) ---
        if (std::abs(vx) < 0.001 && std::abs(vy) < 0.001 && std::abs(wz) < 0.001) {
            // Đứng yên: Tin tưởng tuyệt đối (Variance cực nhỏ)
            odom.twist.covariance[0] = 1e-9;
            odom.twist.covariance[7] = 1e-9;
            odom.twist.covariance[35] = 1e-9;
        } else {
            // Di chuyển: Variance bình thường
            odom.twist.covariance[0] = 0.01; 
            odom.twist.covariance[7] = 0.01;
            odom.twist.covariance[35] = 0.01;
        }

        odom_pub_->publish(odom);
    }
```
