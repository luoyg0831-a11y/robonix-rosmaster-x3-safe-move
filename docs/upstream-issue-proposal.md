# Upstream Issue Record

[Robonix issue #212](https://github.com/syswonder/robonix/issues/212), opened on
2026-08-03, asks where ROS1 support for the Yahboom ROSMASTER X3 should live in
the current Robonix project. The issue is still open as of 2026-08-25.

The proposed integration keeps AMCL, costmaps, `/move_base/make_plan`, and
`move_base` behind a navigation service. Candidate search, repeated validation,
explicit confirmation, and one-time-token handling stay close to goal
publication. Direct goal coordinates, legacy goal tools, and raw `/cmd_vel`
control are not exposed.

The placement question is unchanged:

1. maintain the X3 adapter as an external deployment package;
2. add it as an upstream hardware example; or
3. pair a ROS1 bridge package with the current Robonix navigation service.

## Status since the issue was opened

The issue text describes the 0.08 m revision that existed on 2026-08-03. Since
then, the package has been added to the Catalog, its provider, bridge, and CLI
have been compared with the Jetson copies, and the test suites have passed. An
earlier 0.08 m configuration also completed a live navigation loop.

The current search limit is 0.80 m, not 0.08 m. That configuration has passed
no-motion checks but has not completed a supervised live acceptance run. See
the [latest validation report](../evidence/validation-2026-08-25.md) for the
current status; use the GitHub issue itself for the original proposal text.
