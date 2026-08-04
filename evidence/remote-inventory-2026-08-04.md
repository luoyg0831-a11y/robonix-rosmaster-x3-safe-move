# Sanitized Remote Inventory: 2026-08-04

The expected Jetson SSH identity was verified before inventory. Network address,
host key, credentials, maps, raw logs, and complete vendor configuration files
are intentionally excluded.

| Item | Verified value |
|---|---|
| OS | Ubuntu 18.04.6, Jetson kernel 4.9.253-tegra |
| Python | system 3.6.9; Robonix conda 3.10.20 |
| `rbnx` | 0.1.0 |
| Robonix source | clean `dev`, commit `6bf549f954fb8bc21997e819741f51a34bb51ec9` |
| Production `main.py` | 140109 bytes, 4119 lines, SHA256 `9d8a40fab4c5e734d94ac79f31c53c9f06d139788c30f239d6d11a32e783508e` |
| Candidate `main.py` | SHA256 `a30eef41f392b440d6902b00f05811b6d7ccd463813d292e9bd5c06de91274bd` |
| Complete diff | SHA256 `d61d184e130a131fde8d7f66269ca817110c1b2666afaa04ebe37caddabb5f35` |
| Diff shape | one hunk; 2 deletions, 3 additions; comment and `0.07 -> 0.08` only |
| Production capability set | eight guarded capabilities; legacy direct-goal sources not exposed |

The complete production file and full diff are retained outside the public
repository in the desktop private-audit directory.

## Backup

Before any candidate upload, a date-stamped backup was created at
`/home/jetson/robonix_safe_runs/backups/20260804T200332+0800`. It contains 282
files. Every checksum verified; the `SHA256SUMS` file itself has SHA256
`07e704eec84763b7088c5f2d55756bb341ebae884c2d1fa55360834c94a20ef0`.

No production file was replaced. Candidate validation used the separate
`/home/jetson/robonix_safe_runs/staging/0.08m-audit-20260804T203758+0800`
directory.
