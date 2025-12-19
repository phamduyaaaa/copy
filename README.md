# copy
```bash
# =========================
# NAV2 CONFIG – DIFF-DRIVE + STVL
# =========================

amcl:
  ros__parameters:
    use_sim_time: false
    base_frame_id: base_footprint
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: scan_filter
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.25
    update_min_a: 0.2
    set_initial_pose: true

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    default_nav_to_pose_bt_xml: navigate_w_replanning_and_recovery.xml
    default_nav_through_poses_bt_xml: navigate_w_replanning_and_round_robin_recovery.xml

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: nav2_mppi_controller::MPPIController
      motion_model: DiffDrive
      time_steps: 56
      model_dt: 0.1
      batch_size: 2000
      iteration_count: 1
      vx_max: 0.4
      vx_min: 0.0
      wz_max: 1.8
      ax_max: 3.0
      temperature: 0.3
      gamma: 0.015
      transform_tolerance: 0.1

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4.0
      height: 4.0
      resolution: 0.05
      plugins: ["stvl_layer", "inflation_layer"]

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.3
        cost_scaling_factor: 3.0

      stvl_layer:
        plugin: spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer
        enabled: true
        voxel_size: 0.05
        voxel_decay: 15.0
        decay_model: 0
        track_unknown_space: true
        combination_method: 1
        origin_z: 0.0
        publish_voxel_map: true
        observation_sources: depth
        depth:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: true
          obstacle_range: 3.0
          min_obstacle_height: 0.05
          max_obstacle_height: 2.0

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "stvl_layer", "inflation_layer"]

      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        observation_sources: scan
        scan:
          topic: scan_filter
          data_type: LaserScan
          marking: true
          clearing: true
          obstacle_max_range: 2.5

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.4

      stvl_layer:
        plugin: spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer
        enabled: true
        voxel_size: 0.05
        voxel_decay: 15.0
        decay_model: 0
        track_unknown_space: true
        observation_sources: depth
        depth:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: true

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 0.5

behavior_server:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    max_velocity: [0.4, 0.0, 2.5]
    min_velocity: [-0.4, 0.0, -2.5]
    max_accel: [1.0, 0.0, 1.2]
    max_decel: [-1.0, 0.0, -1.2]
    odom_topic: odom
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

docking_server:
  ros__parameters:
    controller_frequency: 50.0
    base_frame: base_link
    fixed_frame: odom
    dock_plugins: ['simple_charging_dock']
    simple_charging_dock:
      plugin: 'opennav_docking::SimpleChargingDock'
      docking_threshold: 0.05
      staging_x_offset: -0.7
      use_external_detection_pose: true
      use_battery_status: false
      controller:
        k_phi: 3.0
        k_delta: 2.0
        v_linear_min: 0.15
        v_linear_max: 0.15
        use_collision_detection: true
        costmap_topic: local_costmap/costmap_raw
        footprint_topic: local_costmap/published_footprint

```
