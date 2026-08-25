# Validation Evidence — 2026-08-25

## Release boundary

This record supports the `0.3.0-candidate.1` source and package candidate. It
does not authorize unattended operation or claim long-distance hardware
acceptance of the current 0.80 m candidate.

## Current deployed-source inventory

The current provider, external adapter, CLI, CLI test, and relevant ROS
configuration were retrieved from the Jetson over SSH. Backups, generated
bindings, bytecode, logs, credentials, and raw field evidence were excluded
from the public repository.

| Current file | SHA256 | Public status |
|---|---|---|
| Provider `main.py` | `a3c2a5f4b94def0f84b981c8fa1837d99196622717cf576df9603b57cc932e4b` | Byte-for-byte match |
| Adapter `x3_bridge.py` | `425baf888847ca3b2e264ed2e3b50c65118a7ba77718931e55ddac8d503927aa` | Byte-for-byte match |
| CLI `move_base_cli.py` | `4f2637d4bac646bf66ba52641e7511831909e42478b5d115798cff70f0997c9d` | Byte-for-byte match |
| CLI unit test | `e2765e637e04a0f580ff35ebdfd4894dfddd54587b3cee1b0f43360821bc16ec` | Reviewed; public tests retain additional confirmation checks |

The local deployment manifest still uses its legacy package identity. The
public root and `jetson/` manifests intentionally retain the Catalog identity
and both declare `0.3.0-candidate.1`.

## Safety changes represented by this candidate

- Candidate discovery searches 0.80 m, 0.70 m, …, 0.10 m, farthest first.
- Only the forward ±30° sector is considered.
- Once a fully safe radius is found, options from nearer radii are excluded.
- The 1.00 m odometry and AMCL limits use a 0.10 m scaled stop margin, so the
  current bridge triggers cancellation at 0.90 m.
- Prepare and execute revalidation accept at most 0.93 m, paired with the
  existing 0.03 m pose-change gate to absorb bounded stationary AMCL jitter.
- The CLI result timeout is 45 seconds. Exact operator confirmation remains
  mandatory, and cancellation or mismatch leaves the execution gate closed.

## Software verification

| Check | Result |
|---|---|
| Jetson Python 3.10.20 in-memory syntax check of deployed provider, bridge, and CLI | PASS |
| Jetson deployed CLI unit test with bytecode disabled | PASS |
| Python syntax for provider, bridge, CLI, tests, and audits | PASS |
| `stage1_unit_test.py` | PASS |
| `stage2_unit_test.py` | PASS |
| `x3_bridge_navigation_unit_test.py` | PASS |
| `move_base_cli_unit_test.py` | PASS |
| Candidate safety audit | PASS |
| Catalog package audit | PASS |
| Public snapshot privacy/evidence audit | PASS |
| Tracked-file SHA256 verification | PASS after release snapshot generation |

The tests cover farthest-radius selection, forward-sector limits, 0.93 m
revalidation bounds, scaled 0.90 m watchdog triggers, single-publish behavior,
one-time token consumption, exact confirmation, cancellation, mismatch, and
unknown publish outcomes.

## Layered field evidence

| Layer | Result |
|---|---|
| Prior 0.08 m live workflow | `move_base=SUCCEEDED`; final idle state recorded |
| Prior stop-margin trial | Cancelled at 0.080 m cumulative odometry path; did not cross the then-current 0.10 m hard cap |
| 0.90 m preview→prepare workflow | Three no-motion revalidations passed; no navigation goal sent |
| Current 0.80 m candidate | Deployed source captured; long-distance live motion not performed |

The earlier 0.90 m no-motion result demonstrates candidate generation and
revalidation only. It is not evidence that the robot moved 0.90 m safely.

## Demo asset

The release demo is 65,499,360 bytes, 1280×720 at 60 fps, with an approximately
48-second duration. SHA256:

`ccec8554a6a1a68f3af2f2e81b3ef410870907cda46a57e78dfed2872839d5c1`

The video is demonstrative media, not acceptance evidence.
