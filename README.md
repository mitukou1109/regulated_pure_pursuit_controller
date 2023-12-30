# regulated_pure_pursuit_controller

This is an implementation of Pure Pursuit Control algorithm [1] for ROS Noetic.
Most of the parameters correspond to those of [nav2 implementation](https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller).

# Topics

## Published Topics

| Name         | Type            | Description                                   |
| :----------- | :-------------- | :-------------------------------------------- |
| `local_plan` | `nav_msgs/Path` | Trajectory that the robot will ideally follow |

## Subscribed Topics

| Name   | Type                | Description           |
| :----- | :------------------ | :-------------------- |
| `odom` | `nav_msgs/Odometry` | Odometry of the robot |

# Parameters

## Robot

| Name                            | Type     | Description                                                                                                                                 | Default value |
| :------------------------------ | :------- | :------------------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| `acc_lim_linear`                | `double` | The linear acceleration limit of the robot in m/s^2                                                                                         | 2.5           |
| `min_vel_linear`                | `double` | The minimum linear velocity for the robot in m/s                                                                                            | 0.1           |
| `max_vel_linear`                | `double` | The maximum linear velocity for the robot in m/s                                                                                            | 0.5           |
| `acc_lim_angular`               | `double` | The angular acceleration limit of the robot in rad/s^2                                                                                      | 3.2           |
| `min_vel_angular`               | `double` | The minimum angular velocity for the robot in rad/s                                                                                         | 0.2           |
| `max_vel_angular`               | `double` | The maximum angular velocity for the robot in rad/s                                                                                         | 1.0           |
| `is_omnidirectional`            | `bool`   | Whether the robot is omnidirectional or not                                                                                                 | `true`        |
| `follow_viapoint_orientation`   | `bool`   | Whether to follow orientation set to the viapoints or a linear blend of current and goal orientation when is_omnidirectional is true        | `true`        |
| `use_rotate_to_heading`         | `bool`   | Whether to enable rotating at start and goal when is_omnidirectional is false (Recommended on for all robot types that can rotate in place) | `true`        |
| `rotate_to_heading_angular_vel` | `double` | If use_rotate_to_heading is true, this is the angular velocity to use                                                                       | 0.5           |

## Goal tolerance

| Name                 | Type     | Description                                                                            | Default value |
| :------------------- | :------- | :------------------------------------------------------------------------------------- | :------------ |
| `xy_goal_tolerance`  | `double` | The tolerance in meters for the controller in the x & y distance when achieving a goal | 0.25          |
| `yaw_goal_tolerance` | `double` | The tolerance in radians for the controller in yaw/rotation when achieving its goal    | 0.25          |

## Regulated Pure Pursuit

| Name                                 | Type     | Description                                                                                                                   | Default value |
| :----------------------------------- | :------- | :---------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `use_velocity_scaled_lookahead_dist` | `bool`   | Whether to use the velocity scaled lookahead distances or constant lookahead_distance                                         | `true`        |
| `lookahead_dist`                     | `double` | The lookahead distance (m) to use to find the lookahead point when use_velocity_scaled_lookahead_dist is false                | 0.6           |
| `min_lookahead_dist`                 | `double` | The minimum lookahead distance (m) threshold when use_velocity_scaled_lookahead_dist is true                                  | 0.3           |
| `max_lookahead_dist`                 | `double` | The maximum lookahead distance (m) threshold when use_velocity_scaled_lookahead_dist is true                                  | 0.9           |
| `lookahead_time`                     | `double` | The time (s) to project the velocity by when use_velocity_scaled_lookahead_dist is true. Also known as the lookahead gain     | 1.5           |
| `use_curvature_heuristic`            | `bool`   | Whether to use the regulated features for path curvature (e.g. slow on high curvature paths)                                  | `true`        |
| `curvature_gain`                     | `double` | The gain to use for the curvature heuristic when use_curvature_heuristic is true                                              | 1.0           |
| `curvature_threshold`                | `double` | The curvature (1/m) for which the regulation features are triggered when use_curvature_heuristic is true                      | 1.1           |
| `use_proximity_heuristic`            | `bool`   | Whether to use the regulated features for proximity to obstacles (e.g. in close proximity to obstacles)                       | `true`        |
| `proximity_gain`                     | `double` | The gain to use for the proximity heuristic when use_proximity_heuristic is true                                              | 0.5           |
| `proximity_threshold`                | `double` | The distance (m) to the closest obstacle for which the regulation features are triggered when use_proximity_heuristic is true | 0.5           |
| `transform_tolerance`                | `double` | The TF transform tolerance (s)                                                                                                | 0.1           |

# References

[1] S. Macenski, S. Singh, F. Martin, J. Gines, [Regulated Pure Pursuit for Robot Path Tracking](https://arxiv.org/abs/2305.20026). Autonomous Robots, 2023.
