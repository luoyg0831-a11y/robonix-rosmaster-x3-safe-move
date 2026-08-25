# Jetson Inventory — 2026-08-04

This is a historical record of the files inspected before the 0.08 m staging
test. Credentials, host keys, private maps, raw logs, and full vendor files are
not included.

## Software found on the robot

| Item | Recorded value |
|---|---|
| Host | Jetson TX2 NX, Ubuntu 18.04.6 LTS, aarch64 |
| ROS | Melodic |
| Robonix runtime | Python 3.10.20 in conda environment `robonix` |
| Production provider | 140,109 bytes, 4,119 lines, SHA256 `9d8a40fab4c5e734d94ac79f31c53c9f06d139788c30f239d6d11a32e783508e` |
| Staged 0.08 m provider | SHA256 `a30eef41f392b440d6902b00f05811b6d7ccd463813d292e9bd5c06de91274bd` |
| Difference | One hunk: a comment change and `0.07` → `0.08` |

The complete production file and diff were kept outside the public repository.
No production file was replaced during this check.

## Backup and staging

Before staging, a dated backup was written to
`/home/jetson/robonix_safe_runs/backups/20260804T200332+0800`. It contained 282
files, and every recorded checksum verified. The backup's `SHA256SUMS` file had
SHA256 `07e704eec84763b7088c5f2d55756bb341ebae884c2d1fa55360834c94a20ef0`.

The 0.08 m test used the separate directory
`/home/jetson/robonix_safe_runs/staging/0.08m-audit-20260804T203758+0800`.
