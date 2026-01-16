# copy
```bash
# ============================================================
# AMCL – OPTIMIZED FOR RPLIDAR C1 + LOW SPEED (<0.5 m/s)
# ============================================================
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::OmniMotionModel"

    alpha1: 0.05
    alpha2: 0.05
    alpha3: 0.05
    alpha4: 0.05
    alpha5: 0.05

    min_particles: 400
    max_particles: 1200
    pf_err: 0.02
    pf_z: 0.98
    resample_interval: 1

    scan_topic: "scan"
    max_beams: 90
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.05

    z_hit: 0.75
    z_rand: 0.1
    z_short: 0.1
    z_max: 0.05
    sigma_hit: 0.2

    update_min_d: 0.15
    update_min_a: 0.15

    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"

    do_beamskip: false
    publish_particle_cloud: false
    save_pose_rate: 0.5
    tf_broadcast: true
    transform_tolerance: 1.0

    set_initial_pose: true
    always_reset_initial_pose: false

# ============================================================
# BT NAVIGATOR
# ============================================================
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 1.0
    default_nav_to_pose_bt_xml: "$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    navigators: ["navigate_to_pose", "navigate_through_poses"]

# ============================================================
# CONTROLLER SERVER – DWB (LOW SPEED)
# ============================================================
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      min_vel_x: 0.0
      min_vel_y: -0.2
      max_vel_x: 0.5
      max_vel_y: 0.2
      max_vel_theta: 1.0

      min_speed_xy: 0.0
      max_speed_xy: 0.4

      acc_lim_x: 0.5
      acc_lim_y: 0.5
      acc_lim_theta: 1.5

      decel_lim_x: -0.5
      decel_lim_y: -0.5
      decel_lim_theta: -1.0

      vx_samples: 12
      vy_samples: 8
      vtheta_samples: 12

      sim_time: 1.0
      linear_granularity: 0.07
      angular_granularity: 0.035

      transform_tolerance: 1.0

      critics: ["RotateToGoal", "BaseObstacle", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 4.0
      PathAlign.scale: 12.0
      PathDist.scale: 24.0
      GoalDist.scale: 20.0
      RotateToGoal.scale: 24.0

# ============================================================
# LOCAL COSTMAP – LIGHTWEIGHT
# ============================================================
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0

      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true

      width: 3.0
      height: 3.0
      resolution: 0.07

      footprint: "[ [0.18, 0.12], [0.18, -0.12], [-0.18, -0.12], [-0.18, 0.12] ]"

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          marking: true
          clearing: true
          obstacle_max_range: 5.5
          raytrace_max_range: 10.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.25
        cost_scaling_factor: 6.0

# ============================================================
# GLOBAL COSTMAP
# ============================================================
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5
      publish_frequency: 1.0

      global_frame: map
      robot_base_frame: base_link
      resolution: 0.07

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      inflation_layer:
        inflation_radius: 0.25
        cost_scaling_factor: 4.0

# ============================================================
# PLANNER
# ============================================================
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      allow_unknown: true

# ============================================================
# VELOCITY SMOOTHER
# ============================================================
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0]
    min_velocity: [-0.5, -0.2, -1.0]
    max_accel: [0.8, 0.8, 1.5]
    max_decel: [-0.8, -0.8, -1.5]
    odom_topic: "odom"

```
