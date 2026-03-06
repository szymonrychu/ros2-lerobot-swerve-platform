# Nav2 bringup

Runs the Nav2 2D navigation stack in a container. Configured to read `/odom` (or `/odometry/filtered`), `/scan`, and publish `/cmd_vel`. Launched with `use_localization:=false` so no map/AMCL is required (odom-only navigation). Use the `navigate_to_pose` action to send goals. Tune max velocities in `config/nav2_params.yaml` to respect swerve steering rate (ST3215 ~4.71 rad/s).
