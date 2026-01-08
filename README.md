# copy
```bash
amcl:
  ros__parameters:
    alpha1: 0.05
    alpha2: 0.05
    alpha3: 0.1
    alpha4: 0.1
    alpha5: 0.05
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"

    # Laser / Sensor
    scan_topic: "scan"
    max_beams: 60
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    laser_max_range: 3.5
    laser_min_range: 0.1
    lambda_short: 0.1
    z_hit: 0.5
    z_short: 0.05
    z_max: 0.05
    z_rand: 0.5
    sigma_hit: 0.2

    # Beam skipping
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_threshold: 0.3
    beam_skip_error_threshold: 0.9

    # Particles
    min_particles: 500
    max_particles: 1000
    pf_err: 0.03
    pf_z: 0.95
    resample_interval: 1

    # Robot motion model
    robot_model_type: "nav2_amcl::OmniMotionModel" # đúng cho Mecanum

    # Pose publishing
    publish_particle_cloud: false
    save_pose_rate: 0.5
    tf_broadcast: true
    transform_tolerance: 2.5 # tăng lên để tránh Extrapolation Error

    # Minimum updates
    update_min_d: 0.25
    update_min_a: 0.2

    # Initial pose
    set_initial_pose: false
    always_reset_initial_pose: false
    first_map_only: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

```
