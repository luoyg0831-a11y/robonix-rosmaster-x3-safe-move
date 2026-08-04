# Robonix x ROSMASTER X3 Safe Short Move

Version `0.2.0-candidate.1` is a Catalog-structured ROS1 primitive candidate for
guarded short-distance navigation on the Yahboom ROSMASTER X3. Its Catalog name
is `robonix.primitive.yahboom.rosmaster_x3.safe_move`.

The server generates at most five options within 0.08 m. After selection and a
second validation, the CLI displays the prepared target and requires the exact
phrase `确认执行`. Cancellation or any mismatch leaves the execution gate closed.
The package never publishes `/cmd_vel` directly, caps both displacement watchdog
arguments at 0.10 m, and exposes only eight guarded capabilities.

Jetson Python 3.10 syntax, all four offline suites, and an isolated
`rbnx validate/build/validate` sequence passed. The real-ROS no-goal dry-run is
incomplete because fresh AMCL returned `amcl_unavailable`; the guard observed no
goal and no nonzero velocity, and no hardware movement was performed.

See [validation evidence](evidence/validation-2026-08-04.md), the
[deployment map](docs/deployment-map.md), and the
[Catalog submission notes](docs/catalog-submission.md).
