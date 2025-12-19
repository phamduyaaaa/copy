# copy
```bash
####################################
# AMCL
####################################
amcl:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: scan_filter
    map_topic: map
    tf_broadcast: true
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.25
    update_min_a: 0.2
    transform_tolerance: 0.5

####################################
# BT Navigator
####################################
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.5
    navigators: ["navigate_to_pose"]
    navigate_to_pose:
      plugin: nav2_bt_navigator::NavigateToPoseNavigator

####################################
# Controller (MPPI)
####################################
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: nav2_mppi_controller::MPPIController
      motion_model: DiffDrive
      time_steps: 56
      model_dt: 0.1
      batch_size: 2000
      vx_max: 0.4
      vx_min: 0.0
      wz_max: 1.8
      ax_max: 3.0
      temperature: 0.3
      gamma: 0.015

####################################
# LOCAL COSTMAP (STVL + RGBD)
####################################
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4.0
      height: 4.0
      resolution: 0.05
      robot_radius: 0.3

      update_frequency: 5.0
      publish_frequency: 2.0

      track_unknown_space: false
      always_send_full_costmap: true

      plugins: ["rgbd_voxel_layer", "inflation_layer"]

      rgbd_voxel_layer:
        plugin: spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer
        enabled: true
        combination_method: 1
        voxel_size: 0.05
        voxel_decay: 15.0
        decay_model: 0

        origin_z: -0.5
        min_obstacle_height: -0.3
        max_obstacle_height: 2.0

        track_unknown_space: false
        update_footprint_enabled: true
        transform_tolerance: 0.2

        observation_sources: rgbd_mark rgbd_clear

        rgbd_mark:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: false
          obstacle_range: 3.0

        rgbd_clear:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: false
          clearing: true
          min_z: -0.5
          max_z: 7.0

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.3
        cost_scaling_factor: 3.0

####################################
# GLOBAL COSTMAP (STVL + MAP)
####################################
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      robot_radius: 0.3

      update_frequency: 1.0
      publish_frequency: 1.0

      track_unknown_space: false
      always_send_full_costmap: true

      plugins: ["static_layer", "rgbd_voxel_layer", "inflation_layer"]

      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true

      rgbd_voxel_layer:
        plugin: spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer
        enabled: true
        combination_method: 1
        voxel_size: 0.05
        voxel_decay: 15.0
        decay_model: 0

        origin_z: -0.5
        min_obstacle_height: -0.3
        max_obstacle_height: 2.0

        track_unknown_space: false
        update_footprint_enabled: true
        transform_tolerance: 0.2

        observation_sources: rgbd_mark rgbd_clear

        rgbd_mark:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: false
          obstacle_range: 3.5

        rgbd_clear:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: false
          clearing: true
          min_z: -0.5
          max_z: 7.0

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.4
        cost_scaling_factor: 5.0

####################################
# Planner
####################################
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 0.5
      allow_unknown: false

####################################
# Velocity smoother
####################################
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    max_velocity: [0.4, 0.0, 1.8]
    max_accel: [1.0, 0.0, 1.0]
    odom_topic: odom

```
