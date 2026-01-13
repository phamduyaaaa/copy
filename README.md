# copy
```bash
amcl:
  ros__parameters:
    # ---------------------------------------------------------
    # 1. MOTION MODEL (ODOMETRY) - QUAN TRỌNG NHẤT
    # Giảm các chỉ số này để bảo AMCL: "Tin vào bánh xe hơn!"
    # Giá trị cũ (0.2 - 0.3) là quá lớn (coi như bánh xe trượt nhiều).
    # Giảm xuống 0.05 - 0.1 giúp hạt ít bị tán xạ khi di chuyển.
    # ---------------------------------------------------------
    robot_model_type: "nav2_amcl::OmniMotionModel"
    alpha1: 0.05  # (rot -> rot) Giảm nhiễu xoay do xoay
    alpha2: 0.05  # (trans -> rot) Giảm nhiễu xoay do tịnh tiến
    alpha3: 0.05  # (trans -> trans) Giảm nhiễu tịnh tiến do tịnh tiến
    alpha4: 0.05  # (rot -> trans) Giảm nhiễu tịnh tiến do xoay
    alpha5: 0.05  # (strafe) Giảm nhiễu khi đi ngang (cho Mecanum)

    # ---------------------------------------------------------
    # 2. PARTICLES - TĂNG SỐ LƯỢNG MẪU
    # Tăng max_particles để bộ lọc có nhiều giả thuyết hơn để chọn lọc
    # Tăng min_particles để luôn duy trì độ mật độ hạt nhất định
    # ---------------------------------------------------------
    min_particles: 1000  # Cũ: 500
    max_particles: 4000  # Cũ: 1000. Tăng lên giúp robot "bắt" vị trí nhạy hơn
    pf_err: 0.01         # Cũ: 0.03. Giảm sai số cho phép -> Ép hạt hội tụ chặt hơn
    pf_z: 0.99           # Cũ: 0.95. Tăng độ tin cậy thống kê
    resample_interval: 1

    # ---------------------------------------------------------
    # 3. SENSOR MODEL (LASER) - "SIẾT" ĐỘ KHỚP
    # ---------------------------------------------------------
    scan_topic: "scan"
    max_beams: 180         # Cũ: 100. Tăng số tia mẫu để tính toán chính xác hơn (tốn CPU hơn xíu)
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.05
    
    # Quan trọng:
    z_hit: 0.95            # Cũ: 0.9. Tăng trọng số cho các điểm laser trúng tường
    z_rand: 0.05           # Nhiễu ngẫu nhiên
    sigma_hit: 0.1         # Cũ: 0.2. GIẢM CÁI NÀY! 
                           # Ý nghĩa: Chỉ chấp nhận các điểm laser khớp KHÍT với bản đồ. 
                           # Nếu để 0.2, laser lệch 20cm nó vẫn coi là đúng -> hạt bị bè ra.
                           # Để 0.1 bắt buộc robot phải khớp rất sát mới tính điểm cao.

    # ---------------------------------------------------------
    # 4. UPDATE FREQUENCY - CẬP NHẬT LIÊN TỤC
    # ---------------------------------------------------------
    update_min_d: 0.1      # Cũ: 0.2. Đi 10cm là cập nhật lại vị trí ngay.
    update_min_a: 0.1      # Cũ: 0.2. Xoay nhẹ là cập nhật ngay.
                           # Việc update thường xuyên giúp triệt tiêu sai số tích lũy nhanh hơn.

    # Các thông số khác giữ nguyên
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"
    lambda_short: 0.1
    z_short: 0.05
    z_max: 0.05
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_threshold: 0.3
    beam_skip_error_threshold: 0.9
    publish_particle_cloud: true  # Bật lên để debug xem hạt đã tụ chưa
    save_pose_rate: 0.5
    tf_broadcast: true
    transform_tolerance: 1.0 
    set_initial_pose: true
    always_reset_initial_pose: false
    first_map_only: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```
