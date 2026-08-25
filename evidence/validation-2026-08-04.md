# Staging Check — 2026-08-04

This report covers the historical 0.08 m revision.

## Jetson staging results

| Check | Result |
|---|---|
| Python | 3.10.20 |
| Seven-file syntax check | PASS |
| Stage 1 candidate tests | PASS |
| Stage 2 token, gate, and concurrency tests | PASS |
| Bridge observer tests | PASS |
| CLI tests | PASS |
| `rbnx validate` before build | PASS |
| `rbnx build --clean` | PASS |
| `rbnx validate` after build | PASS |
| Production file hashes before and after | Unchanged |

The first build used `/usr/bin/python3` and failed because `grpc_tools` was not
installed there. It passed after the path was changed to match production, with
the conda Python environment before the Cargo and system paths. Code generation
skipped four IDL message or service entries whose dependencies were missing.

## ROS no-motion run

Result: `amcl_unavailable`; the run stopped before target selection.

The test started the verified driver, lidar, AMCL, `move_base`, map server, and
rosbridge launch files. A read-only monitor watched both goal topics and
`/cmd_vel`. The staging CLI ran with `--dry-run`, returned code 2 during preview,
and did not send a goal.

```text
goal_messages=0
cmd_nonzero_messages=0
max_linear_mps=0.0
max_angular_rps=0.0
violation=false
```

All processes started by the test were stopped, the ROS ports were closed, and
the production provider hash was unchanged. The robot did not move.

This run verified that the 0.08 m revision failed closed when AMCL was
unavailable. It did not complete a successful dry run or hardware acceptance;
later results are recorded in the
[2026-08-25 report](validation-2026-08-25.md).
