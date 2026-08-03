# Security Policy

This project controls physical robot motion. Treat every change to goal generation, validation, token handling, timeouts, ROS publishers, or cancellation as safety-critical.

Do not report credentials, private maps, live robot addresses, or unredacted field logs in public issues. Use the repository owner's private GitHub contact channel for sensitive reports.

The following changes require real-ROS no-motion regression before any hardware run:

- exposing a new capability;
- changing distance, cost, timeout, or localization thresholds;
- changing the navigation publisher lifecycle or goal correlation;
- changing token claim, replay, cancellation, or watchdog behavior;
- changing any `/cmd_vel` publisher topology.

No test in this repository authorizes robot motion.

