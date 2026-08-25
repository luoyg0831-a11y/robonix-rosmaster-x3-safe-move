# Robonix x ROSMASTER X3 Safe Short Move

Version `0.3.0-candidate.1` is a Catalog-structured ROS1 primitive candidate for
guarded navigation on the Yahboom ROSMASTER X3. Its Catalog name
is `robonix.primitive.yahboom.rosmaster_x3.safe_move`.

The server searches from 0.80 m down to 0.10 m in the forward ±30° sector and
returns up to five options at the farthest fully validated radius. After a
second validation, the CLI displays the target and requires the exact phrase
`确认执行`. Cancellation or any mismatch leaves the execution gate closed.
The package never publishes `/cmd_vel` directly. Odometry and AMCL watchdog
inputs are capped at 1.00 m, with cancellation triggered at 0.90 m to retain a
0.10 m stopping margin.

The public source matches the current Jetson provider, bridge, and CLI hashes.
Syntax checks, four offline suites, the safety audit, and the public-snapshot
audit pass. A prior 0.08 m configuration completed a live `move_base` loop, and
the 0.90 m preview/prepare workflow passed without publishing a goal. The
current 0.80 m candidate has not completed long-distance hardware acceptance.

See [current validation evidence](evidence/validation-2026-08-25.md), the
[deployment map](docs/deployment-map.md), and the
[Catalog submission notes](docs/catalog-submission.md).

## Demo video

[audio-test01.mp4](https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move/releases/download/v0.3.0-candidate.1/audio-test01.mp4)
is attached to the `v0.3.0-candidate.1` prerelease. It is 65,499,360 bytes and
has SHA256 `ccec8554a6a1a68f3af2f2e81b3ef410870907cda46a57e78dfed2872839d5c1`.
The video is a demonstration, not production authorization or hardware
acceptance of the 0.80 m candidate.
