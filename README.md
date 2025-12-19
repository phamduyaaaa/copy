# copy
```bash
amcl:
  ros__parameters:
    # ... (giữ nguyên thông số alpha/particles)
    robot_model_type: "nav2_amcl::DifferentialMotionModel" # Quay lại dùng ::
    scan_topic: scan_filter
    map_topic: map
    set_initial_pose: true
    always_reset_initial_pose: false
    first_map_only: false
    initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.5
    default_nav_to_pose_bt_xml: "$(find-pkg-share omni_base_nav)/param/navigate_w_replanning_and_recovery.xml"
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator" # Dùng ::
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator" # Dùng ::

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker" # Dùng :: theo log báo lỗi

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker" # Dùng ::

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController" # Dùng ::
      motion_model: "DiffDrive"
      # ... (giữ nguyên các thông số MPPI của bạn)

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
      footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
      plugins: ["rgbd_obstacle_layer", "inflation_layer"]
      
      rgbd_obstacle_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer" # GIỮ NGUYÊN / (STVL đã chạy với dấu này)
        enabled: true
        voxel_decay: 2.0
        decay_model: 0
        voxel_size: 0.05
        track_unknown_space: true
        observation_sources: rgbd1_mark
        rgbd1_mark:
          data_type: PointCloud2
          topic: /input_pointcloud
          marking: true
          clearing: true
          obstacle_max_range: 3.0
          min_obstacle_height: 0.05
          max_obstacle_height: 2.0
          voxel_filter: true
          clear_after_reading: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer" # Đổi lại thành :: theo log báo lỗi

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer" # Dùng ::
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer" # Dùng ::
        observation_sources: scan
        scan:
          topic: /scan_filter
          data_type: "LaserScan"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer" # Dùng ::

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner" # Dùng ::

behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin: {plugin: "nav2_behaviors::Spin"}
    backup: {plugin: "nav2_behaviors::BackUp"}
    drive_on_heading: {plugin: "nav2_behaviors::DriveOnHeading"}
    wait: {plugin: "nav2_behaviors::Wait"}
    assisted_teleop: {plugin: "nav2_behaviors::AssistedTeleop"}

docking_server:
  ros__parameters:
    controller_frequency: 50.0
    base_frame: "base_link"
    fixed_frame: "odom"
    dock_plugins: ['simple_charging_dock']
    simple_charging_dock:
      plugin: 'opennav_docking::SimpleChargingDock' # Dùng ::
    docks: ['home_dock']
    home_dock:
      type: 'simple_charging_dock'
      frame: map
      pose: [0.0, 0.0, 0.0]
```
