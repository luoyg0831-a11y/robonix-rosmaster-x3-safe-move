# Recorded ROS Configuration

Values below were read from the Jetson workspace files on 2026-08-04. They are
disk configuration, not proof of live parameter loading. Vendor files are not
redistributed here.

| Area | Parameter | Disk value |
|---|---|---:|
| AMCL | `odom_model_type` | `omni` (launch default) |
| AMCL | `update_min_d` | `0.20` |
| AMCL | `update_min_a` | `0.20` |
| DWA | `max_vel_x` | `0.08` |
| DWA | `max_vel_y` | `0.05` |
| DWA | `max_vel_theta` | `0.0` |
| DWA | `xy_goal_tolerance` | `0.03` |
| DWA | `yaw_goal_tolerance` | `0.2` |
| move_base | `oscillation_timeout` | `5.0` |
| move_base | `oscillation_distance` | `0.01` |
| move_base | recovery/clearing rotation | disabled |

The handoff's older AMCL values `0.02/0.05` do not match the inspected disk
file and must not be reported as current.

Relevant sanitized SHA256 values:

| Source | SHA256 |
|---|---|
| AMCL config | `365bedbec8b24b6576c0c770fc1b1126ddd905d36a9532724b095a1476cd96ea` |
| navigation launch | `3046349a275167891384ad6fdfdb8c956c335f4be7c3f43d6432d82a74fc626a` |
| move_base launch | `ce76265ae9057a3252f1c86d7c6cd5803ccca2bfdbcd4eed61f342e97254d7cf` |
| DWA X3 config | `24355a12368297eca8d74c5bedbefbaaeaa00c4372e5363647dc7b9fe7686431` |
| move_base parameters | `470c614d563a47e631aaeb3e653e6f18f890f218c7ca65cb78529f2b7444475f` |

Do not copy these settings to another robot without reviewing footprint,
controller, localization, wiring, and emergency-stop behavior.
