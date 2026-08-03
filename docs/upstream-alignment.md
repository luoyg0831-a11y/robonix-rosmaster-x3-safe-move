# Alignment with Current Robonix

## Version boundary

This project was developed against an older Python Robonix package model:

- `package_manifest.yaml`
- generated MCP/service bindings
- ROS1 Melodic and rosbridge
- ROS `move_base`

The current `syswonder/robonix` main branch uses:

- a Rust task manager and CLI;
- ROS2 interfaces and ROS2 Humble as the recommended environment;
- `rbnx_manifest.yaml` provider packages;
- standardized primitives and services plus flexible basic/RTDL skills.

The local snapshot is therefore not a drop-in package for current main.

## Proposed upstream mapping

| Existing component | Current Robonix role |
|---|---|
| AMCL pose reader | `prm::base.pose.cov` provider |
| Correlated navigation goal execution | `prm::base.navigate` provider |
| Candidate generation and three-stage safety workflow | `skl::safe_short_move` basic skill |
| Numbered `move_base` command | package-specific demo/operations CLI |
| Cost/path checks | internal skill module until a standard service exists |

Safety gates must remain next to actuation and cannot exist only in the interactive CLI.

## Recommended contribution sequence

1. Freeze and validate this ROS1 companion repository.
2. Open an upstream design issue describing the ROS1/ROS2 compatibility boundary.
3. Implement a separate ROS2-facing provider package under `rust/provider/rosmaster_x3_safe_navigation`.
4. Use explicit ROS1/ROS2 gateway interfaces; do not silently mix manifests.
5. Submit documentation and provider code as reviewable commits or separate pull requests.

No upstream pull request should claim hardware completion until the current revision is tested on the Jetson and robot.

