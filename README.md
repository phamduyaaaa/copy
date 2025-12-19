# copy
```bash
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 0.5
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan_filter
    map_topic: map
    set_initial_pose: true
    always_reset_initial_pose: false
    first_map_only: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.5
    filter_duration: 0.3
    default_nav_to_pose_bt_xml: "$(find-pkg-share omni_base_nav)/param/navigate_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "$(find-pkg-share omni_base_nav)/param/navigate_w_replanning_and_round_robin_recovery.xml"
    always_reload_bt_xml: false
    goal_blackboard_id: goal
    goals_blackboard_id: goals
    path_blackboard_id: path
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    error_code_name_prefixes:
      - assisted_teleop
      - backup
      - compute_path
      - dock_robot
      - drive_on_heading
      - follow_path
      - nav_thru_poses
      - nav_to_pose
      - spin
      - route
      - undock_robot
      - wait

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    costmap_update_timeout: 0.30
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    use_realtime_priority: false

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.1
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 6.28

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.1
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.4
      vx_min: -0.1
      vy_max: 0.4
      wz_max: 1.9
      ax_max: 3.0
      ax_min: -3.0
      ay_min: -3.0
      ay_max: 3.0
      az_max: 3.5
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      regenerate_noises: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      TrajectoryValidator:
        plugin: "mppi::DefaultOptimalTrajectoryValidator"
        collision_lookahead_time: 2.0
        consider_footprint: false
      AckermannConstraints:
        min_turning_r: 0.2
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0

local_costmap:
  local_costmap:
    ros__parameters:  
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05
      robot_radius: 0.3
      footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
      plugins: ["stvl_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.2
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 15.0
        decay_model: 0
        voxel_size: 0.05
        track_unknown_space: true
        mark_threshold: 0
        update_footprint_enabled: true
        combination_method: 1
        origin_z: 0.0
        publish_voxel_map: false
        transform_tolerance: 0.2
        mapping_mode: false
        map_save_duration: 60.0
        observation_sources: ["rgbd1_mark", "rgbd1_clear"]
        rgbd1_mark:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: false
          obstacle_range: 3.0
          min_obstacle_height: 0.02
          max_obstacle_height: 2.0
          expected_update_rate: 0.0
          observation_persistence: 0.0
          inf_is_valid: false
          filter: "passthrough"
          voxel_min_points: 0
          clear_after_reading: true
        rgbd1_clear:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: false
          clearing: true
          max_z: 7.0
          min_z: 0.02
          vertical_fov_angle: 0.8745
          horizontal_fov_angle: 1.048
          decay_acceleration: 15.0
          model_type: 0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "stvl_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.2
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 15.0
        decay_model: 0
        voxel_size: 0.05
        track_unknown_space: true
        mark_threshold: 0
        update_footprint_enabled: true
        combination_method: 1
        origin_z: 0.0
        publish_voxel_map: false
        transform_tolerance: 0.2
        mapping_mode: false
        map_save_duration: 60.0
        observation_sources: ["rgbd1_mark", "rgbd1_clear"]
        rgbd1_mark:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: false
          obstacle_range: 3.0
          min_obstacle_height: 0.02
          max_obstacle_height: 2.0
          expected_update_rate: 0.0
          observation_persistence: 0.0
          inf_is_valid: false
          filter: "passthrough"
          voxel_min_points: 0
          clear_after_reading: true
        rgbd1_clear:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: false
          clearing: true
          max_z: 7.0
          min_z: 0.02
          vertical_fov_angle: 0.8745
          horizontal_fov_angle: 1.048
          decay_acceleration: 15.0
          model_type: 0

# map_saver, planner_server, smoother_server, behavior_server, waypoint_follower,
# velocity_smoother, collision_monitor, docking_server, loopback_simulator
# giữ nguyên thông số như file gốc bạn gửi ban đầu, chỉ cập nhật STVL cho local/global costmap
```
