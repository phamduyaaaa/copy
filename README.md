# copy
```bash
####################################
# AMCL - LOCALIZATION (ƯU TIÊN ỔN ĐỊNH)
####################################
amcl:
  ros__parameters:
    use_sim_time: false

    # Frames
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"

    # Motion model (CỐ TÌNH dùng differential để tránh drift)
    robot_model_type: "nav2_amcl::DifferentialMotionModel"

    # Odometry noise (THỰC TẾ, không lạc quan)
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.3
    alpha4: 0.3
    alpha5: 0.2

    # Laser model
    scan_topic: "scan"
    max_beams: 60
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    laser_max_range: 3.5
    laser_min_range: 0.1
    sigma_hit: 0.2

    z_hit: 0.5
    z_short: 0.05
    z_max: 0.05
    z_rand: 0.4
    lambda_short: 0.1

    # Beam skip (TẮT cho đơn giản)
    do_beamskip: false

    # Particles
    min_particles: 500
    max_particles: 1500
    pf_err: 0.05
    pf_z: 0.99
    resample_interval: 1

    # Update conditions
    update_min_d: 0.2
    update_min_a: 0.2

    # TF
    tf_broadcast: true
    transform_tolerance: 0.3

    # Initial pose
    set_initial_pose: false
    always_reset_initial_pose: false


####################################
# CONTROLLER SERVER (MECANUM)
####################################
controller_server:
  ros__parameters:
    controller_frequency: 15.0

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.1
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.25

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false

      # Velocities
      min_vel_x: 0.0
      min_vel_y: -0.2
      max_vel_x: 0.5
      max_vel_y: 0.2
      max_vel_theta: 1.0

      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0

      # Acceleration
      acc_lim_x: 1.0
      acc_lim_y: 1.0
      acc_lim_theta: 1.5

      decel_lim_x: -1.0
      decel_lim_y: -1.0
      decel_lim_theta: -1.5

      # Sampling
      vx_samples: 20
      vy_samples: 20
      vtheta_samples: 40

      sim_time: 1.5
      transform_tolerance: 0.3

      critics:
        - BaseObstacle
        - PathAlign
        - GoalAlign
        - PathDist
        - GoalDist
        - RotateToGoal
        - Oscillation

      BaseObstacle.scale: 0.1
      PathAlign.scale: 24.0
      GoalAlign.scale: 20.0
      PathDist.scale: 24.0
      GoalDist.scale: 20.0
      RotateToGoal.scale: 20.0


####################################
# LOCAL COSTMAP (KHÔNG CHE LỖI)
####################################
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0

      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true

      width: 3.0
      height: 3.0
      resolution: 0.05

      footprint: "[[0.19, 0.14], [0.19, -0.14], [-0.19, -0.14], [-0.19, 0.14]]"

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: LaserScan
          clearing: true
          marking: true
          max_obstacle_height: 2.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.35
        cost_scaling_factor: 8.0

      always_send_full_costmap: true


####################################
# GLOBAL COSTMAP
####################################
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0

      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05

      footprint: "[[0.19, 0.14], [0.19, -0.14], [-0.19, -0.14], [-0.19, 0.14]]"

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: LaserScan
          clearing: true
          marking: true
          obstacle_max_range: 2.5
          raytrace_max_range: 3.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5
        cost_scaling_factor: 3.0

      always_send_full_costmap: true


####################################
# PLANNER
####################################
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true


####################################
# BT NAVIGATOR
####################################
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.3
    default_nav_to_pose_bt_xml: "$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"


####################################
# VELOCITY SMOOTHER
####################################
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    max_velocity: [0.6, 0.2, 1.0]
    min_velocity: [-0.6, -0.2, -1.0]
    max_accel: [0.8, 0.8, 1.5]
    max_decel: [-0.8, -0.8, -1.5]
    odom_topic: "odom"
    velocity_timeout: 1.0

```
