# Robonix × ROSMASTER X3 Safe Move

This repository contains a ROS1 navigation component for the Yahboom ROSMASTER
X3. The current release is `0.3.0-candidate.1`; its Robonix Catalog package name
is `robonix.primitive.yahboom.rosmaster_x3.safe_move`.

## Usage

Run the no-motion check first:

```bash
move_base --dry-run
```

If the environment and candidate list look correct, run:

```bash
move_base
```

The command lists up to five candidates. After the user selects one, the
provider checks AMCL, the global costmap, and `/move_base/make_plan` again and
prints the target. Navigation proceeds only after the exact phrase `确认执行` is
entered at the confirmation prompt; `q` or any other response cancels the
operation.

## Limits

- Radii are checked from 0.80 m down to 0.10 m. Only candidates at the farthest
  passing radius are returned.
- Candidates are limited to the forward ±30° sector.
- The execution call accepts only a one-time token issued by
  `prepare_selected_move` for the active preview session; it does not accept
  target coordinates.
- Project code does not publish `/cmd_vel`; ROS `move_base` remains responsible
  for velocity commands.
- The odometry-path and AMCL-displacement arguments are capped at 1.00 m. At
  that setting, cancellation starts at 0.90 m, leaving 0.10 m for stopping.

The 0.93 m value in the provider is the hard ceiling used by the preparation
check, not the preview radius. The numbered-candidate path independently limits
candidates to 0.80 m and rejects a robot-position change above 0.03 m after
preview. These are separate checks and should not be added together.

## Validation status

| Check | Result |
|---|---|
| Local syntax, four unit suites, and repository checks | Pass |
| Jetson Python 3.10.20 syntax and deployed CLI unit test | Pass |
| Provider, bridge, and CLI hashes against the Jetson files | Match |
| Earlier 0.08 m live run | `move_base=SUCCEEDED` |
| Earlier 0.90 m preview→prepare no-motion checks | 3 passes; no goal sent |
| Live acceptance of the current 0.80 m configuration | Not run |

See the [2026-08-25 validation report](evidence/validation-2026-08-25.md) for
details. Those earlier 0.90 m no-motion checks are not driving tests and do not
establish live acceptance of the 0.80 m configuration.

## Demo

[audio-test01.mp4](https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move/releases/download/v0.3.0-candidate.1/audio-test01.mp4)
is attached to `v0.3.0-candidate.1`. The file is 65,499,360 bytes and has SHA256
`ccec8554a6a1a68f3af2f2e81b3ef410870907cda46a57e78dfed2872839d5c1`.

## Catalog and license

[Robonix Catalog PR #21](https://github.com/syswonder/robonix-package-catalog/pull/21)
was merged on 2026-08-05. Original code in this repository is Apache-2.0.
Third-party Robonix, ROS, and Yahboom components retain their own licenses.
