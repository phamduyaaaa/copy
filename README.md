# copy
```bash
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <libserial/SerialPort.h>

using namespace std::chrono_literals;
using namespace LibSerial;

// --- PROTOCOL CONSTANTS ---
const uint8_t START_BYTE = 0xFF;
const uint8_t STOP_BYTE  = 0xFE;
const int FRAME_LEN      = 15;
const double SCALE_CMD   = 100.0;

class STM32Bridge : public rclcpp::Node {
public:
    STM32Bridge() : Node("stm32_bridge_cpp") {
        // 1. Parameters (Khai báo tham số để chỉnh được trong launch file)
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("wheel_radius", 0.048);
        this->declare_parameter("lx", 0.235);
        this->declare_parameter("ly", 0.20);

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baudrate").as_int();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        double lx = this->get_parameter("lx").as_double();
        double ly = this->get_parameter("ly").as_double();
        geo_factor_ = lx + ly;

        // 2. Serial Connection
        init_serial(port, baud);

        // 3. ROS Publishers & Subscribers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, std::bind(&STM32Bridge::cmd_vel_callback, this, std::placeholders::_1));

        // 4. Timers (Loop)
        // 50Hz Read Loop (Giống Python)
        timer_read_ = this->create_wall_timer(
            20ms, std::bind(&STM32Bridge::read_uart_loop, this));
        
        // Watchdog Timer (Check nếu mất kết nối cmd_vel)
        timer_watchdog_ = this->create_wall_timer(
            100ms, std::bind(&STM32Bridge::watchdog_callback, this));

        last_cmd_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "STM32 Bridge C++ Started!");
    }

private:
    // --- VARIABLES ---
    SerialPort serial_port_;
    double wheel_radius_;
    double geo_factor_;
    
    // Robot State
    double x_ = 0.0;
    double y_ = 0.0;
    double th_ = 0.0;
    rclcpp::Time last_time_;
    bool first_run_ = true;

    // Buffers
    std::vector<uint8_t> rx_buffer_;
    rclcpp::Time last_cmd_time_;

    // ROS Handles
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_read_;
    rclcpp::TimerBase::SharedPtr timer_watchdog_;

    // --- HELPER FUNCTIONS ---

    void init_serial(std::string port, int baud) {
        try {
            serial_port_.Open(port);
            // Chuyển int baud sang Enum của LibSerial (Hơi bất tiện tí)
            if (baud == 115200) serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            else serial_port_.SetBaudRate(BaudRate::BAUD_115200); // Mặc định
            
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
        } catch (const OpenFailed&) {
            RCLCPP_FATAL(this->get_logger(), "Cannot open serial port: %s", port.c_str());
            exit(1);
        }
    }

    // --- TX LOGIC (GỬI LỆNH) ---
    void send_packet(int16_t vx, int16_t vy, int16_t wz) {
        std::vector<uint8_t> frame;
        frame.push_back(START_BYTE);

        // Payload: Vx(H,L) Vy(H,L) Wz(H,L) - Big Endian
        uint8_t payload[6];
        payload[0] = (vx >> 8) & 0xFF; payload[1] = vx & 0xFF;
        payload[2] = (vy >> 8) & 0xFF; payload[3] = vy & 0xFF;
        payload[4] = (wz >> 8) & 0xFF; payload[5] = wz & 0xFF;

        uint8_t checksum = 0;
        for(int i=0; i<6; i++) {
            frame.push_back(payload[i]);
            checksum ^= payload[i]; // XOR checksum
        }

        frame.push_back(checksum);
        frame.push_back(STOP_BYTE);

        // Gửi qua serial
        serial_port_.Write(frame);
    }

    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_cmd_time_ = this->now();

        // Convert float -> int16 (Scale 100)
        int16_t vx_i = static_cast<int16_t>(std::clamp(msg->twist.linear.x * SCALE_CMD, -32768.0, 32767.0));
        int16_t vy_i = static_cast<int16_t>(std::clamp(msg->twist.linear.y * SCALE_CMD, -32768.0, 32767.0));
        int16_t az_i = static_cast<int16_t>(std::clamp(msg->twist.angular.z * SCALE_CMD, -32768.0, 32767.0));

        send_packet(vx_i, vy_i, az_i);
    }

    void watchdog_callback() {
        // Nếu quá 0.5s không có lệnh -> Gửi lệnh dừng an toàn
        auto now = this->now();
        if ((now - last_cmd_time_).seconds() > 0.5) {
            send_packet(0, 0, 0);
        }
    }

    // --- RX LOGIC (NHẬN DỮ LIỆU) ---
    uint8_t calc_checksum_xor(const std::vector<uint8_t>& frame) {
        // Frame: [START, D1..D12, CS, STOP]
        // Checksum tính từ D1 đến D12 (Index 1 đến 12)
        uint8_t cs = frame[1]; // Bắt đầu từ byte đầu tiên của payload
        for (int i = 2; i <= 12; i++) {
            cs ^= frame[i];
        }
        return cs;
    }

    void read_uart_loop() {
        if (!serial_port_.IsOpen()) return;

        // 1. Đọc dữ liệu mới vào buffer
        try {
            while (serial_port_.IsDataAvailable()) {
                uint8_t byte;
                serial_port_.ReadByte(byte);
                rx_buffer_.push_back(byte);
            }
        } catch (...) {}

        // 2. Parse (Sliding Window)
        while (rx_buffer_.size() >= FRAME_LEN) {
            // Check Start Byte
            if (rx_buffer_[0] != START_BYTE) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // Check Stop Byte
            if (rx_buffer_[14] != STOP_BYTE) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // Copy frame ra để xử lý
            std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + FRAME_LEN);

            // Check Checksum
            if (calc_checksum_xor(frame) != frame[13]) {
                // Checksum sai -> Xóa byte đầu để trượt tiếp
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // --- DECODE DATA (Big Endian) ---
            // Python: struct.unpack(">h") -> C++: (H << 8) | L
            int16_t fr = (int16_t)((frame[1] << 8) | frame[2]);
            int16_t fl = (int16_t)((frame[3] << 8) | frame[4]);
            int16_t rr = (int16_t)((frame[5] << 8) | frame[6]);
            int16_t rl = (int16_t)((frame[7] << 8) | frame[8]);
            
            // Góc (int32)
            int32_t goc = (int32_t)((frame[9] << 24) | (frame[10] << 16) | (frame[11] << 8) | frame[12]);

            // Cập nhật Odom
            process_kinematics(fr, fl, rr, rl, goc);

            // Xóa frame đã xử lý khỏi buffer
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + FRAME_LEN);
        }
    }

    void process_kinematics(int16_t fr, int16_t fl, int16_t rr, int16_t rl, int32_t goc_raw) {
        rclcpp::Time current_time = this->now();
        
        if (first_run_) {
            last_time_ = current_time;
            first_run_ = false;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 1. Chuyển đổi RPM/Tick sang m/s (Giống Python của bạn)
        // scale = (2 * pi * r) / 60
        double scale = (2 * M_PI * wheel_radius_) / 60.0;
        
        double v_fr = fr * scale;
        double v_fl = fl * scale;
        double v_rr = rr * scale;
        double v_rl = rl * scale;

        // 2. Mecanum Kinematics
        double vx = (v_fr + v_fl + v_rr + v_rl) / 4.0;
        double vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
        double wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * geo_factor_);

        // 3. Tích phân (Integrate)
        // Lưu ý: wz ở trên là từ Encoder bánh xe.
        // Bạn có thể chọn dùng wz từ bánh xe hoặc từ IMU (goc_raw) để tính dth.
        // Ở đây tôi giữ nguyên logic Python (dùng encoder cho odom).
        
        double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
        double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
        double delta_th = wz * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        // 4. Publish TF & Odom
        publish_odom(current_time, vx, vy, wz);
        publish_imu(current_time, goc_raw);
    }

    void publish_odom(rclcpp::Time now, double vx, double vy, double wz) {
        // TF
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // Odom Message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = t.transform.rotation;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        odom_pub_->publish(odom);
    }

    void publish_imu(rclcpp::Time now, int32_t goc_raw) {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = now;
        imu.header.frame_id = "imu_link";

        // Convert raw yaw to radian (Logic Python: -raw / 10.0 => deg => rad)
        double yaw_deg = -goc_raw / 10.0;
        double yaw_rad = yaw_deg * (M_PI / 180.0);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);

        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();

        // Covariance (Fake như Python)
        imu.orientation_covariance[0] = 9999.0;
        imu.orientation_covariance[4] = 9999.0;
        imu.orientation_covariance[8] = 0.001;

        imu_pub_->publish(imu);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<STM32Bridge>());
    rclcpp::shutdown();
    return 0;
}
```
