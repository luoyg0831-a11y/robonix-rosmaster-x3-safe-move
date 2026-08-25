---
description: Guarded ROS1 navigation for ROSMASTER X3 with preview, confirmation, execution monitoring, and cancellation.
---

# ROSMASTER X3 Safe Move

The normal call sequence is:

1. `preview_move_options`
2. `prepare_selected_move`
3. operator confirmation in the CLI
4. `execute_prepared_navigation`

The execution call accepts a one-time token from `prepare_selected_move`; it
does not accept coordinates. Before publishing a single `move_base` goal, the
provider checks the selected target for a third time. The bridge then tracks the
goal, velocity, AMCL, and odometry results and can cancel the associated goal.

Candidate search is limited to 0.80 m and the forward ±30° sector. Project code
does not publish `/cmd_vel`. With the 1.00 m odometry/AMCL limit, cancellation
starts at 0.90 m.

Preview and preparation do not move the robot. Live acceptance of the current
0.80 m configuration has not been completed.
