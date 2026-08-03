# Alignment with Current Robonix

## Version boundary

This project was developed with an earlier Robonix Python API and targets:

- `package_manifest.yaml`;
- generated MCP/service bindings;
- ROS1 Melodic and rosbridge;
- ROS `move_base`.

The current `syswonder/robonix` `dev` branch also uses `package_manifest.yaml`, but its capability-provider lifecycle, Python API, generated bindings, and contract details have evolved. ROS2 is the documented native ROS transport. The local snapshot is therefore not a drop-in package for current `dev`.

## Proposed upstream mapping

The existing global navigation contracts are the natural public boundary:

| Existing component | Current Robonix role |
|---|---|
| ROS1 AMCL, costmap, `make_plan`, and `move_base` adapter | Internal implementation of a navigation service package |
| Goal execution and correlated result tracking | `robonix/service/navigation/navigate`, `navigate/status`, and `navigate/cancel` |
| Package lifecycle | `robonix/service/navigation/driver` |
| Candidate generation, numbered selection, confirmation, and one-time token | Safety policy inside the navigation service or a separately reviewed skill |
| Numbered `move_base` command | Deployment-specific operations CLI |

Safety gates must remain next to goal publication and cannot exist only in the interactive CLI. The package must not expose legacy direct-goal tools or raw `/cmd_vel` control.

## Open integration decision

The current upstream tree has a Webots `simple_nav` example but no ROS1 ROSMASTER X3 implementation. Maintainer guidance is needed before choosing among:

1. an external deployment package implementing the four existing navigation contracts;
2. a new hardware example under the upstream repository;
3. a ROS1 bridge package plus a separate Robonix navigation service.

No new global contract is proposed yet. The existing navigation contracts should be tried first, with any extra safe-short-move contract discussed separately.

## Recommended contribution sequence

1. Freeze and validate this ROS1 companion repository.
2. Open an upstream integration issue with the public repository and prerelease.
3. Confirm package placement and ROS1 bridge expectations with maintainers.
4. Rebase the package on current `dev` APIs and the four navigation contracts.
5. Submit code only after local current-API tests and Jetson hardware validation.

No pull request should claim hardware completion until the exact submitted revision is tested on the Jetson and robot.
