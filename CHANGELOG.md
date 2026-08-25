# Changelog

## Unreleased

## 0.3.0-candidate.1 - 2026-08-25

- Synchronized the public provider, rosbridge adapter, and CLI with the current
  Jetson deployment and recorded byte-for-byte SHA256 matches.
- Changed option discovery to search 0.80 m down to 0.10 m, farthest first,
  limited to the forward ±30° sector, returning only the farthest safe radius.
- Raised the bounded odometry and AMCL watchdog inputs to 1.00 m while adding a
  scaled stop margin that triggers cancellation at 0.90 m for the 1.00 m cap.
- Added a 0.93 m prepare/execute revalidation ceiling for bounded AMCL jitter;
  candidate generation remains capped at 0.80 m.
- Extended the result timeout to 45 seconds and preserved all operator
  confirmation and fail-closed cancellation tests.
- Updated recorded ROS configuration hashes and current validation status
  without redistributing vendor packages, raw maps, or field logs.
- Replaced the demo with the new 65,499,360-byte release asset and recorded its
  SHA256.

## 0.2.0-candidate.1 - 2026-08-04

- Added Catalog-ready root metadata, `config.spec`, capability documentation,
  and an active offline GitHub Actions workflow.
- Restored explicit operator confirmation after target display; cancellation or
  incorrect confirmation never enters the execution gate.
- Capped bridge odometry and AMCL displacement watchdog arguments at 0.10 m.
- Downloaded and hashed the complete production 0.07 m provider; the 0.08 m
  candidate differs only in the radius comment and constant.
- Created and verified a dated Jetson backup before staging any candidate files.
- Passed Jetson Python 3.10 syntax, four offline suites, and
  `rbnx validate/build/validate` in an isolated staging directory.
- Recorded the real-ROS no-goal dry-run as incomplete because the candidate
  fresh-AMCL check returned `amcl_unavailable`; no goal or nonzero velocity was
  observed and no real movement was performed.

## 0.2.0-stage-a.1 - 2026-08-04

- Added a complete sanitized project summary and the first verified Jetson inventory.
- Recorded three successful SSH checks, host identity verification, OS/Python boundaries, Robonix Git state, and production hashes.
- Confirmed that production `main.py` remains at 0.07 m while the public candidate is 0.08 m.
- Confirmed that the bridge, CLI, manifest, wrapper, and install-script hashes match the public snapshot.
- Restored the Jetson-matched package build and start scripts.
- Recorded that the ROS navigation stack was offline, so ROS idleness, dry-run, deployment, and hardware acceptance remain pending.

- Corrected the compatibility notes against the current upstream `dev` branch.
- Added the upstream ROSMASTER X3 navigation integration proposal.
- Aligned the documented checks with the seven-file syntax validation.
- Clarified the status of the offline workflow template and integration record.

## 0.1.0-local-preview - 2026-08-03

- Curated the local 0.08 m candidate into a public, secret-free snapshot.
- Included the safe public capability set and corresponding service definitions.
- Included Stage 1, Stage 2, bridge observer, and CLI offline tests.
- Made the bridge observer test independent of the current working directory.
- Documented architecture, safety invariants, validation evidence, deployment gaps, and upstream migration.
- Excluded raw logs, maps, credentials, generated files, backups, and legacy capability manifests.
