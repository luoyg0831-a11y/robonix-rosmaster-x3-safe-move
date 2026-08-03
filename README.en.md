# Robonix x ROSMASTER X3 Safe Short Move

This repository summarizes a ROS1 safety layer for short-distance navigation on a Yahboom ROSMASTER X3. From the Jetson `robonix` environment, the `move_base` command discovers up to five server-generated safe poses, accepts a numbered selection, revalidates it, publishes one ROS navigation goal, and returns correlated motion evidence.

> Status: `0.1.0-local-preview`. This is a curated Windows snapshot, not the Jetson production baseline. The 0.08 m candidate passed local syntax checks and offline unit tests, but has not completed Jetson deployment, real-ROS no-motion regression, or final hardware acceptance.

## Safety properties

- Never publishes raw `/cmd_vel`.
- Does not accept arbitrary goal coordinates from the caller.
- Revalidates the selected pose before preparation and again before publishing.
- Uses single-use tokens, atomic claiming, replay prevention, and bounded execution.
- Does not retry an indeterminate publish with the same token.
- Correlates move_base state, velocity, base feedback, AMCL, odometry, and final idle state.
- Keeps legacy `send_nav_goal` and `go_to_waypoint` contracts out of the public manifest.

## Offline checks

```bash
python -m py_compile jetson/main.py jetson/x3_bridge.py jetson/scripts/move_base_cli.py
python jetson/stage1_unit_test.py
python jetson/stage2_unit_test.py
python jetson/x3_bridge_navigation_unit_test.py
python jetson/tests/move_base_cli_unit_test.py
```

These checks use fake ROS/Robonix modules and do not constitute hardware acceptance.

## Upstream compatibility

This snapshot targets an older Python Robonix package model with Ubuntu 18.04, ROS1 Melodic, rosbridge, and ROS `move_base`. The current [syswonder/robonix](https://github.com/syswonder/robonix) main branch uses Rust, ROS2, and `rbnx_manifest.yaml`. See [upstream alignment](docs/upstream-alignment.md) for the proposed provider/skill migration.

## License

The historical package manifest declares the original project code as Apache-2.0. Robonix, ROS, Yahboom sources, generated code, and other third-party components remain under their respective licenses.

