---
description: Preview, prepare, confirm, execute, observe, and cancel a guarded ROS1 navigation goal up to 0.80 m.
---

# ROSMASTER X3 Safe Move

Call `preview_move_options`, then `prepare_selected_move`. The operator-facing
CLI displays the prepared target and requires the exact phrase `确认执行` before
it opens the process-local execution gate. Cancellation or any other input
leaves the gate closed.

`execute_prepared_navigation` consumes a single-use token, performs an
independent third validation, publishes at most one ROS navigation goal, and
returns correlated status, velocity, AMCL, and odometry evidence. It never
publishes `/cmd_vel` directly. Candidate search is capped at 0.80 m in the
forward ±30° sector. Both displacement watchdog inputs are capped at 1.00 m;
the bridge cancels at 0.90 m to preserve a 0.10 m stopping margin.

The preview, readiness, status, and preparation calls are motion-free. The
0.80 m candidate has not completed long-distance hardware acceptance; no test,
video, or package metadata in this repository authorizes real robot motion.
