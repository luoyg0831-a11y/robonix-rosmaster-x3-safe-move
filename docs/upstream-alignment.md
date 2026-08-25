# Compatibility with Current Robonix

This comparison was refreshed on 2026-08-25 against the Robonix `dev` branch at
`f718d8d1f2e2d7c150020ad6e184d7754abde0a7`.

The X3 package uses an earlier Robonix Python API, generated service bindings,
ROS1 Melodic, rosbridge, and ROS `move_base`. The current upstream tree still
contains the navigation service contracts and the Webots `simple_nav` example,
but its provider lifecycle, Python API, bindings, and contract details have
changed. This repository cannot be copied into current `dev` unchanged.

## Likely mapping

| X3 component | Current Robonix counterpart |
|---|---|
| AMCL, costmap, `make_plan`, and `move_base` adapter | Internal navigation-service implementation |
| Goal execution and result tracking | `navigation/navigate`, `navigate/status`, and `navigate/cancel` |
| Package startup and shutdown | `navigation/driver` |
| Candidate search, numbered selection, confirmation, and token handling | X3-specific safety policy |
| `move_base` terminal command | Deployment CLI |

Goal checks and one-time-token handling need to stay beside goal publication;
placing them only in the CLI would allow another caller to bypass them. A port
must also avoid exposing direct coordinates or raw `/cmd_vel` control.

## Next upstream step

[Robonix issue #212](https://github.com/syswonder/robonix/issues/212) remains
open. Before a code contribution, maintainers need to choose whether the ROS1
support belongs in an external package, an upstream hardware example, or a ROS1
bridge paired with the current navigation service. After that decision, the
package can be rebased on the current API and tested again locally and on the
robot.
