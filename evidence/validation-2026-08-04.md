# Validation Evidence: 2026-08-04

## Passed on Jetson staging

| Check | Result |
|---|---|
| Python | 3.10.20 |
| Seven-file syntax check | PASS |
| Stage 1 candidate tests | PASS |
| Stage 2 token/gate/concurrency tests | PASS |
| Bridge observer tests | PASS |
| CLI tests | PASS |
| `rbnx validate` before build | PASS |
| `rbnx build --clean` | PASS |
| `rbnx validate` after build | PASS |
| Production SHA before/after | unchanged |

The first build attempt selected `/usr/bin/python3` and failed because
`grpc_tools` was unavailable. Repeating with the production-equivalent PATH,
where conda Python precedes Cargo and system paths, passed. Codegen warned that
four IDL message/service entries with unresolved dependencies were skipped.

## Real ROS no-goal dry-run

Status: **INCOMPLETE / FAIL-CLOSED**.

The verified vendor launch sources started the driver, lidar, AMCL, move_base,
map server, and rosbridge. A subscriber-only guard watched both goal topics and
`/cmd_vel`. The staging CLI was invoked with `--dry-run` and returned code 2,
`amcl_unavailable`, during preview. It therefore never reached target selection
or execution.

Guard result:

```text
goal_messages=0
cmd_nonzero_messages=0
max_linear_mps=0.0
max_angular_rps=0.0
violation=false
```

All owned ROS process groups were stopped and the ROS ports were confirmed
closed. The production provider SHA256 remained unchanged. No navigation target,
direct `/cmd_vel` publication, or real robot movement occurred.

## Release boundary

This evidence supports an auditable 0.08 m source and package candidate. It does
not support production deployment or hardware acceptance. Fresh-AMCL readiness,
a successful no-goal dry-run, one authorized 0.08 m run, and repeat acceptance
remain open.
