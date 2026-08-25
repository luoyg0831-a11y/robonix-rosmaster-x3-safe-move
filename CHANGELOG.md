# Changelog

## Unreleased

- Edited the public documentation for clarity; runtime behavior is unchanged.

## 0.3.0-candidate.1 - 2026-08-25

- Synced `main.py`, `x3_bridge.py`, and `move_base_cli.py` with the Jetson files.
- Changed candidate search to 0.80 m through 0.10 m in the forward ±30° sector,
  checking longer radii first and returning only the farthest passing radius.
- Raised the odometry-path and AMCL-displacement argument caps to 1.00 m. The
  bridge uses a 0.10 m stop margin at that setting and cancels at 0.90 m.
- Kept the preparation ceiling at 0.93 m. The numbered-candidate flow separately
  limits preview to 0.80 m and rejects position changes above 0.03 m.
- Increased the navigation-result timeout to 45 seconds.
- Added the updated demo video and refreshed the ROS configuration hashes.

## 0.2.0-candidate.1 - 2026-08-04

- Added Catalog metadata, `config.spec`, capability files, and the GitHub Actions
  test workflow.
- Required operator confirmation after target display.
- Limited the bridge watchdog arguments to 0.10 m.
- Compared the 0.08 m candidate with the deployed 0.07 m provider and created a
  dated Jetson backup before staging.
- Passed Python 3.10 syntax, four offline suites, and the staged Robonix build.
- The real-ROS dry run stopped at `amcl_unavailable`; no goal was sent.

## 0.2.0-stage-a.1 - 2026-08-04

- Added the initial Jetson inventory, deployment map, build scripts, package
  metadata, and upstream integration notes.
- Recorded the deployed provider, bridge, CLI, manifest, wrapper, and installer
  hashes.

## 0.1.0-local-preview - 2026-08-03

- Published the first 0.08 m source preview.
- Added the eight guarded capabilities and four offline test suites.
- Excluded credentials, maps, field logs, generated output, backups, and legacy
  direct-goal capabilities.
