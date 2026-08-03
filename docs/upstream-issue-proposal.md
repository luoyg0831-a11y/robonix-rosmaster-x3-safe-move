## Feature description

Add a documented integration path for Yahboom ROSMASTER X3 robots running ROS1 Melodic and ROS `move_base`, based on a public safe short-distance navigation prototype. The prototype discovers up to five server-generated safe poses, accepts a numbered choice, revalidates it, requires explicit confirmation, consumes a one-time token, performs a final safety check, publishes one navigation goal, and reports correlated execution evidence.

## Affected component

**Service**

- [x] navigation

**Skill / Primitive**

- [ ] skill
- [ ] primitive

**Tooling / cross-cutting**

- [x] capabilities & contracts / docs

## Problem statement

Robonix currently has standard navigation contracts and a Webots `simple_nav` example, but there is no documented integration for a physical ROSMASTER X3 that remains on Ubuntu 18.04, ROS1 Melodic, and `move_base`. A direct port of the prototype would also be unsafe: its earlier Robonix Python API and generated bindings no longer match current `dev`, and its local 0.08 m revision has not yet completed Jetson deployment and hardware acceptance.

## Proposed solution

Use the existing global contracts as the public boundary:

- `robonix/service/navigation/driver`
- `robonix/service/navigation/navigate`
- `robonix/service/navigation/navigate/status`
- `robonix/service/navigation/navigate/cancel`

Keep ROS1 AMCL, costmap, `/move_base/make_plan`, and `move_base` behind a navigation service package. Keep candidate generation, three-stage validation, execution gating, and one-time-token handling next to goal publication. Do not expose arbitrary goal coordinates, legacy direct-goal tools, or raw `/cmd_vel` control to the agent.

Before preparing a code contribution, I would like maintainer guidance on the preferred placement:

1. an external deployment package implementing the existing contracts;
2. an upstream hardware example similar to `examples/webots/services/simple_nav`;
3. a ROS1 bridge package plus a separate current-API navigation service.

No new global contract is requested at this stage. If numbered candidate discovery should become an agent-facing reusable skill, I propose discussing that contract separately after the navigation service is current-API compliant.

## Alternatives considered

- Submitting the historical provider directly: rejected because its API and generated bindings do not match current `dev`.
- Publishing raw chassis velocity: rejected because it bypasses `move_base` planning and the existing safety model.
- Claiming the local snapshot as hardware-complete: rejected because the exact 0.08 m revision has only passed local syntax and offline unit tests.

## Use case

From a Robonix terminal, a user requests a short move, receives at most five safe numbered poses, selects one, reviews the target and path information, explicitly confirms execution, and receives an arrival or clear failure result. The intended deployment is a Yahboom ROSMASTER X3 with Jetson TX2 NX, RPLidar, ROS1 Melodic, AMCL, costmaps, and `move_base`.

## Validation and safety boundary

The public local preview currently records:

- passing Python syntax checks and four offline test suites;
- no direct `/cmd_vel` publication;
- bounded candidate count and distance;
- second and third safety validation;
- atomic one-time-token claim and replay prevention;
- a single navigation-goal publication attempt;
- correlated move_base status, velocity, AMCL, and odometry evidence;
- earlier real-robot motion evidence from prior revisions.

The exact 0.08 m revision is **not** yet Jetson-deployed or hardware-accepted. The repository intentionally excludes credentials, IP addresses, raw site logs, maps, vendor files of uncertain provenance, and generated/back-up directories.

## Additional context

- Project summary: https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move
- Local-preview prerelease: https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move/releases/tag/v0.1.0-local-preview
- Current upstream baseline reviewed: `dev` at `cb5a3bb78737d0d3c17cf4850b9fbf2de0eec8b0`
