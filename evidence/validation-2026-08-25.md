# Validation Report — 2026-08-25

## Scope

This report covers `0.3.0-candidate.1`. It records source comparison, software
tests, and earlier field results. A supervised live run of the current 0.80 m
configuration was not performed.

## Jetson file comparison

The provider, rosbridge adapter, CLI, CLI test, and relevant ROS configuration
were copied from the Jetson over SSH. Generated bindings, bytecode, backups,
logs, credentials, and maps were left out of the repository.

| Jetson file | SHA256 | Repository result |
|---|---|---|
| Provider `main.py` | `a3c2a5f4b94def0f84b981c8fa1837d99196622717cf576df9603b57cc932e4b` | Match |
| Adapter `x3_bridge.py` | `425baf888847ca3b2e264ed2e3b50c65118a7ba77718931e55ddac8d503927aa` | Match |
| CLI `move_base_cli.py` | `4f2637d4bac646bf66ba52641e7511831909e42478b5d115798cff70f0997c9d` | Match |
| Deployed CLI unit test | `e2765e637e04a0f580ff35ebdfd4894dfddd54587b3cee1b0f43360821bc16ec` | Repository test keeps additional confirmation cases |

The Jetson still has the older local package identity. The root and `jetson/`
manifests in this repository use the Catalog name and version.

## Limits checked in this build

- Preview checks 0.80 m through 0.10 m, farthest first.
- Only the forward ±30° sector is used.
- Returned candidates all come from the same, farthest passing radius.
- With 1.00 m odometry/AMCL limits, the bridge cancels at 0.90 m.
- The preparation endpoint rejects targets beyond 0.93 m.
- The numbered-candidate path separately caps preview at 0.80 m and rejects a
  position change above 0.03 m after preview.
- The navigation-result timeout is 45 seconds.
- Execution still requires the exact confirmation phrase and a one-time token
  from `prepare_selected_move`.

## Test results

| Check | Result |
|---|---|
| Jetson Python 3.10.20 syntax check of provider, bridge, and CLI | PASS |
| Jetson deployed CLI unit test | PASS |
| Local Python syntax check | PASS |
| `stage1_unit_test.py` | PASS |
| `stage2_unit_test.py` | PASS |
| `x3_bridge_navigation_unit_test.py` | PASS |
| `move_base_cli_unit_test.py` | PASS |
| Safety-limit check | PASS |
| Catalog metadata check | PASS |
| Private-data/file-type check | PASS |
| `SHA256SUMS` verification | PASS |

The unit tests cover radius ordering, forward-sector filtering, distance
limits, watchdog margins, single-goal publication, token reuse, confirmation,
cancellation, and unknown publish results.

## Robot results available at the time of release

| Test | Result |
|---|---|
| Earlier 0.08 m run | `move_base=SUCCEEDED`; robot stopped at the end |
| Earlier 0.08 m stop-margin run | Cancelled at 0.080 m cumulative odometry path, below the 0.10 m cap used at that time |
| Earlier 0.90 m preview→prepare configuration | 3 passes; no navigation goal sent |
| Current 0.80 m live run | Not performed |

The 0.90 m result came from an earlier configuration and covers candidate
generation and preparation only. It does not show that the robot travelled
0.90 m.

## Demo file

- Size: 65,499,360 bytes
- Video: 1280×720, 60 fps, approximately 48 seconds
- SHA256: `ccec8554a6a1a68f3af2f2e81b3ef410870907cda46a57e78dfed2872839d5c1`
