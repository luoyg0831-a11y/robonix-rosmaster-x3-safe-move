# Validation Status

Status is deliberately split into written, compiled, deployed, run, and hardware-accepted. These labels are not interchangeable.

## Local 0.08 m candidate

Validated on 2026-08-03 with Windows Python 3.13.13:

| Check | Result |
|---|---|
| Seven-file `py_compile` | PASS |
| `stage1_unit_test.py` | PASS |
| `stage2_unit_test.py` | PASS |
| `x3_bridge_navigation_unit_test.py` | PASS |
| `move_base_cli_unit_test.py` | PASS |

The tests cover candidate bounds, failure branches, token consumption, replay prevention, one-publish concurrency, publisher cleanup, unknown outcomes, bounded timeouts, motion evidence, and CLI input handling.

## Real robot evidence from earlier revisions

- ROS navigation prerequisites, rosbridge, AMCL, costmaps, `make_plan`, and move_base were observed running on the Jetson.
- An earlier approximately 0.25 m workflow produced visible robot movement on video.
- Multiple 0.09 m runs produced a new move_base goal, nonzero control output, base movement, and AMCL/odom displacement.
- The latest recorded 0.09 m run ended `PREEMPTED(2)` after odometry cumulative path reached approximately 0.108 m, exceeding the 0.10 m charging-cable watchdog.
- These observations prove that the command path can move the base, but do not prove the current 0.08 m candidate reaches `SUCCEEDED`.

## Not yet validated

- Exact remote file inventory and installed Robonix commit.
- 0.08 m candidate under Jetson Python 3.10.
- `rbnx validate/build/validate` for the 0.08 m candidate.
- Real ROS `move_base --dry-run` with no new goal and no nonzero velocity.
- One controlled 0.08 m run meeting all result gates.
- Three-repeat final acceptance and the complete negative-test matrix.

The repository must remain a prerelease until those items are completed.

