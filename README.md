# copy
```bash
### ekf_node config for Medical Robot (Mecanum) - OPTIMIZED ###
ekf_filter_node:
  ros__parameters:
    # 1. Tan so & Che do
    frequency: 30.0      # Tăng lên 30Hz cho mượt (vì Timer Python em chạy 50Hz)
    two_d_mode: true
    publish_tf: true     # EKF sẽ chịu trách nhiệm publish odom -> base_footprint

    # 2. Khung toa do
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    # =======================================================
    # 3. Input 1: ENCODER (Odometry từ Python)
    # CHIẾN THUẬT: Chỉ tin Vận Tốc, không tin Vị Trí tích lũy
    # =======================================================
    odom0: /wheel/odom
    
    # Config: [x, y, z, 
    #          roll, pitch, yaw, 
    #          vx, vy, vz, 
    #          vroll, vpitch, vyaw, 
    #          ax, ay, az]
    
    odom0_config: [false, false, false,  # KHÔNG lấy vị trí x, y (vì Mecanum trượt)
                   false, false, false,  # KHÔNG lấy góc (để IMU lo)
                   true,  true,  false,  # LẤY vx, vy (QUAN TRỌNG NHẤT)
                   false, false, true,   # LẤY vyaw (vận tốc xoay từ bánh xe)
                   false, false, false]

    odom0_differential: false # False vì ta lấy vận tốc trực tiếp (vx), không phải đạo hàm từ vị trí
    odom0_queue_size: 10
    
    # =======================================================
    # 4. Input 2: IMU (BNO055 từ Python)
    # CHIẾN THUẬT: Tin tuyệt đối vào góc Yaw
    # =======================================================
    imu0: /imu/data
    
    imu0_config: [false, false, false,
                  false, false, true,   # LẤY GÓC YAW (Tuyệt đối tin tưởng)
                  false, false, false,
                  false, false, false,  # KHÔNG lấy v_yaw (vì code Python em để -1.0)
                  false, false, false]
                  
    imu0_differential: false
    imu0_queue_size: 10

    # =======================================================
    # 5. MA TRẬN NHIỄU HỆ THỐNG (Process Noise - Q)
    # Đây là "tính cách" của robot Mecanum
    # =======================================================
    process_noise_covariance: [
      0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # x
      0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # y
      0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # z
      0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # roll
      0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # pitch
      0.0, 0.0, 0.0, 0.0, 0.0, 0.1,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # yaw (Tăng lên 0.1 vì Mecanum lắc)
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # vx (Tăng lên 0.25 vì hay trượt)
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # vy (Tăng lên 0.25 vì trượt ngang nhiều)
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # vz
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, # vroll
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, # vpitch
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,  0.0, 0.0, 0.0, # vyaw (Tăng lên 0.1)
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, # ax
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, # ay
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015 # az
    ]

    initial_estimate_covariance: [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]
```
