# Recorded ROS Configuration

Values below were read from the active X3 source workspace on 2026-08-25.
They describe disk configuration, not proof that every value was live-loaded.
Vendor source and site maps are not redistributed because their license or
field-data status is unresolved.

| Area | Parameter | Recorded value |
|---|---|---:|
| AMCL | `update_min_d` | `0.02` |
| AMCL | `update_min_a` | `0.05` |
| DWA | `max_vel_x` | `0.06` |
| DWA | `max_vel_y` | `0.03` |
| DWA | `max_vel_theta` | `0.0` |
| DWA | `acc_lim_x/y/theta` | `0.8` |
| DWA | `xy_goal_tolerance` | `0.025` |
| DWA | `yaw_goal_tolerance` | `3.2` |
| DWA | `sim_time` | `1.0` |
| DWA | controller frequency | `10.0 Hz` |
| move_base | `oscillation_timeout` | `10.0` |
| move_base | `oscillation_distance` | `0.01` |
| robot_localization | filter frequency | `20 Hz` |
| robot_localization | inputs | wheel odometry + IMU |

The navigation launch now accepts `initial_pose_x`, `initial_pose_y`, and
`initial_pose_a`. The bringup launch disables the base driver's TF publisher
and routes odometry plus IMU data through `robot_localization`. The laser launch
uses the recorded scan filter. These robot-specific changes require a fresh
review before use on different hardware.

## Sanitized SHA256 inventory

| Source role | SHA256 |
|---|---|
| RPLIDAR scan filter | `3736382a54d2b4f984e4ecd97e899d27bed7528733fd6ff04958bdb572d72915` |
| X3 base driver | `d9760c42a7f1456f9c36687fe66d8af04c318dc97d8d2a2e045571eea176f801` |
| robot_localization config | `c83024a2daa32483ce5254c1afebd89635f094cd5eec93a1a9c7bbbc7fd132da` |
| X3 bringup launch | `e5acb2dffd308a4baf5d4d3de618e599404d0a27ea8d3ac2ea6c71cb3931ba58` |
| navigation launch | `52efbb09dedb08e1db7a2417b5932fc33728e2390d6f6a624c83a04169b50824` |
| laser launch | `021f9498be40b6ec8bf3c5166fa3e75158cc43e23144c81af5c297ec590e17ca` |
| AMCL launch | `519fef72866e4d414b477a3801753a5798aac550bf7b4c84fb8d8448bfd42671` |
| DWA X3 config | `a8312e09dc8c36edce26b8991facd5ebe206827123878a58634f0a391864d573` |
| move_base parameters | `d8b8ed921cad00f7603182bce39a5b8f34708b266765a719d3b22d62a474bdea` |
| Current site map PGM | `7476bbf1e57afff0e761102db52e036702eed6583d13ee4817f350bbcc3bf126` |
| Current site map YAML | `ac2bf4b3a0e0c736fb61fd6ee04b106859b4f3f82999a8b8fceaca9d2600600e` |

Do not copy these settings to another robot without reviewing footprint,
controller behavior, localization, wiring, emergency stop, and map provenance.
