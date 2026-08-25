# Deployment Map

## Public repository to Jetson

| Public path | Current Jetson path or role |
|---|---|
| `package_manifest.yaml` | Catalog-facing root manifest |
| `jetson/package_manifest.yaml` | Curated release manifest; the existing local deployment still carries its legacy package identity |
| `jetson/main.py` | `rosmaster_x3_deploy/primitives/rosmaster_x3_bridge/rosmaster_x3_bridge/main.py` |
| `jetson/x3_bridge.py` | `x3_rosbridge_adapter/scripts/x3_bridge.py` |
| `jetson/scripts/move_base_cli.py` | `rosmaster_x3_deploy/scripts/move_base_cli.py` |
| `jetson/scripts/move_base` | `rosmaster_x3_deploy/scripts/move_base` |

The source snapshot keeps `main.py` at `jetson/main.py` for offline tests. The
deployed package imports it from the nested `rosmaster_x3_bridge/` module.
`scripts/start.sh` supports both layouts without copying or changing production.

The three current runtime source files were copied over SSH on 2026-08-25 and
match the public files byte for byte:

| File | SHA256 |
|---|---|
| `main.py` | `a3c2a5f4b94def0f84b981c8fa1837d99196622717cf576df9603b57cc932e4b` |
| `x3_bridge.py` | `425baf888847ca3b2e264ed2e3b50c65118a7ba77718931e55ddac8d503927aa` |
| `move_base_cli.py` | `4f2637d4bac646bf66ba52641e7511831909e42478b5d115798cff70f0997c9d` |

## Command and environment

Inside the `robonix` conda environment, the installed `move_base` command calls
the deployment wrapper and then `move_base_cli.py`. The wrapper sources the ROS
and Robonix Python paths and removes legacy execution-gate variables before
starting the CLI.

## Build and codegen

- `jetson/scripts/build.sh` runs Robonix MCP code generation.
- Generated `rbnx-build/` content is reproducible build output and is not
  committed.
- Backups, logs, bytecode, raw maps, and device credentials are excluded from
  the public snapshot.

## ROS configuration boundary

The inspected ROS1 source workspace is the X3-specific Yahboom workspace. The
navigation stack uses the laser, AMCL, map server, move_base, rosbridge, base
driver, and robot_localization files summarized in
[Recorded ROS Configuration](ros-configuration.md). Vendor files are not copied
into this repository; only parameter summaries and SHA256 values are published.
