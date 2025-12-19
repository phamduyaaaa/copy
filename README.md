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
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_round_robin_recovery.xml"
    always_reload_bt_xml: false
    goal_blackboard_id: goal
    goals_blackboard_id: goals
    path_blackboard_id: path
    navigators: ['navigate_to_pose', 'navigate_through_poses']

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

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
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]

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
          filter: "passthrough"
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

behavior_server:
  ros__parameters:
    plugin_names: ["back_up", "spin", "wait"]
    plugin_types: ["nav2_behaviors::BackUp", "nav2_behaviors::Spin", "nav2_behaviors::Wait"]

waypoint_follower:
  ros__parameters:
    plugin: "nav2_waypoint_follower::WaypointFollower"
    follow_cycle_frequency: 10.0

velocity_smoother:
  ros__parameters:
    plugin: "nav2_velocity_smoother::VelocitySmoother"
    max_accel_x: 0.5
    max_accel_y: 0.5
    max_accel_theta: 0.3

collision_monitor:
  ros__parameters:
    plugin: "nav2_collision_monitor::CollisionMonitor"
    monitor_frequency: 5.0
    collision_radius: 0.3

docking_server:
  ros__parameters:
    plugin: "nav2_docking::Docking"

loopback_simulator:
  ros__parameters:
    plugin: "nav2_loopback_simulator::LoopbackSimulator"

```
