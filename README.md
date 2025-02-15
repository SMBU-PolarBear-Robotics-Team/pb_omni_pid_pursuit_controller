# PolarBear Omni PID Pursuit Controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/pb_omni_pid_pursuit_controller/actions/workflows/ci.yml/badge.svg)](https://github.com/LihanChen2004/pb_omni_pid_pursuit_controller/actions/workflows/ci.yml)

## Configuration

| Parameter | Description |
|-----|----|
| `translation_kp` | The proportional gain for the translation PID controller. Controls how strongly the robot reacts to translation errors. |
| `translation_ki` | The integral gain for the translation PID controller. Helps eliminate steady-state errors over time. |
| `translation_kd` | The derivative gain for the translation PID controller. Damps oscillations by reacting to the rate of change of translation errors. |
| `enable_rotation` | Whether to enable the rotation PID controller. If disabled, the robot will not rotate to face the path's direction. `twist.angular.z` always remains zero. |
| `rotation_kp` | The proportional gain for the rotation PID controller. Controls how strongly the robot reacts to rotational errors. |
| `rotation_ki` | The integral gain for the rotation PID controller. Helps eliminate steady-state rotational errors. |
| `rotation_kd` | The derivative gain for the rotation PID controller. Damps oscillations by reacting to the rate of change of rotational errors. |
| `transform_tolerance` | The tolerance for transforming between frames. A higher value may allow for more flexibility in handling small delays in the transformation. |
| `min_max_sum_error` | The minimum threshold for the maximum sum of errors used to limit the accumulated error in the PID controller to prevent windup. |
| `lookahead_dist` | The fixed lookahead distance used to find the lookahead point for path following. |
| `use_velocity_scaled_lookahead_dist` | Whether to scale the lookahead distance based on the current velocity, instead of using a constant distance. |
| `min_lookahead_dist` | The minimum allowable lookahead distance when using velocity scaling for the lookahead point. |
| `max_lookahead_dist` | The maximum allowable lookahead distance when using velocity scaling for the lookahead point. |
| `lookahead_time` | The time used to project the robot's velocity to calculate the velocity-scaled lookahead distance. |
| `use_interpolation` | Enables interpolation between poses along the path when selecting the lookahead point, improving smoothness but potentially increasing computational cost. |
| `use_rotate_to_heading` | Whether to rotate the robot to face the path's direction before moving forward, useful in holonomic robots. |
| `use_rotate_to_heading_treshold` | The angular threshold at which the robot should rotate in place to align with the desired heading. |
| `min_approach_linear_velocity` | The minimum linear velocity when approaching the goal to ensure the robot moves slowly when close to its target. |
| `approach_velocity_scaling_dist` | The distance from the goal where velocity scaling starts when approaching, slowing the robot down as it nears the target. |
| `min_translation_speed` | The minimum translation speed the robot can command, allowing for reverse or slow-forward movement. |
| `max_translation_speed` | The maximum translation speed the robot can command, setting the upper limit for forward movement. |
| `min_rotation_speed` | The minimum rotation speed the robot can command, allowing for counter-clockwise rotation. |
| `max_rotation_speed` | The maximum rotation speed the robot can command, setting the upper limit for clockwise rotation. |
| `max_robot_pose_search_dist` | The maximum distance along the path to search for the robot's closest pose, used to keep the robot on the planned path. |

Example fully-described XML with default parameter values:

```yaml
controller_server:
  ros__parameters:
    odom_topic: odometry
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
      translation_kp: 3.0
      translation_ki: 0.1
      translation_kd: 0.3
      enable_rotation: false
      rotation_kp: 3.0
      rotation_ki: 0.1
      rotation_kd: 0.3
      transform_tolerance: 0.1
      min_max_sum_error: 1.0
      lookahead_dist: 2.0
      use_velocity_scaled_lookahead_dist: true
      lookahead_time: 1.0
      min_lookahead_dist: 0.5
      max_lookahead_dist: 1.0
      use_interpolation: false
      use_rotate_to_heading: false
      use_rotate_to_heading_treshold: 0.1
      min_approach_linear_velocity: 0.5
      approach_velocity_scaling_dist: 1.5
      min_translation_speed: -2.5
      max_translation_speed: 2.5
      min_rotation_speed: -3.0
      max_rotation_speed: 3.0
```

## TODO

- [ ] Adjust speed based on curvature
