# Recorded ROS Configuration

The latest engineering handoff records these active parameters. The full vendor files are omitted from this local public snapshot until their source and redistribution license are verified.

## AMCL

| Parameter | Value |
|---|---:|
| `odom_model_type` | `omni` |
| `update_min_d` | `0.02` |
| `update_min_a` | `0.05` |

## DWA local planner

| Parameter | Value |
|---|---:|
| `max_vel_x` | `0.08` |
| `max_vel_y` | `0.05` |
| `max_vel_theta` | `0.0` |
| `xy_goal_tolerance` | `0.03` |
| `yaw_goal_tolerance` | `0.2` |

## move_base

| Parameter | Value |
|---|---:|
| `oscillation_timeout` | `5.0` |
| `oscillation_distance` | `0.01` |
| recovery rotation | disabled |

These are evidence records, not universal defaults. Do not apply them to another robot without reviewing footprint, controller, localization, and emergency-stop behavior.

