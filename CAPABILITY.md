---
description: Preview, prepare, confirm, execute, observe, and cancel a guarded short ROS1 navigation goal.
---

# ROSMASTER X3 Safe Move

Call `preview_move_options`, then `prepare_selected_move`. The operator-facing
CLI displays the prepared target and requires the exact phrase `确认执行` before
it opens the process-local execution gate. Cancellation or any other input
leaves the gate closed.

`execute_prepared_navigation` consumes a single-use token, performs an
independent third validation, publishes at most one ROS navigation goal, and
returns correlated status, velocity, AMCL, and odometry evidence. It never
publishes `/cmd_vel` directly. Candidate radius is capped at 0.08 m and both
displacement watchdog inputs are capped at 0.10 m.

The preview, readiness, status, and preparation calls are motion-free. No test
or package metadata in this repository authorizes real robot motion.
