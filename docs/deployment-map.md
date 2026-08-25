# Deployment Map

| Repository path | Jetson path or purpose |
|---|---|
| `package_manifest.yaml` | Catalog metadata |
| `jetson/package_manifest.yaml` | Release package metadata |
| `jetson/main.py` | `rosmaster_x3_deploy/primitives/rosmaster_x3_bridge/rosmaster_x3_bridge/main.py` |
| `jetson/x3_bridge.py` | `x3_rosbridge_adapter/scripts/x3_bridge.py` |
| `jetson/scripts/move_base_cli.py` | `rosmaster_x3_deploy/scripts/move_base_cli.py` |
| `jetson/scripts/move_base` | `rosmaster_x3_deploy/scripts/move_base` |

The local Jetson manifest still uses its older package identity. The two
manifests in this repository use the Catalog package name and version.

## File comparison — 2026-08-25

| File | SHA256 |
|---|---|
| `main.py` | `a3c2a5f4b94def0f84b981c8fa1837d99196622717cf576df9603b57cc932e4b` |
| `x3_bridge.py` | `425baf888847ca3b2e264ed2e3b50c65118a7ba77718931e55ddac8d503927aa` |
| `move_base_cli.py` | `4f2637d4bac646bf66ba52641e7511831909e42478b5d115798cff70f0997c9d` |

`jetson/main.py` stays at the repository root of the Jetson package so the
offline tests can import it directly. The deployed package uses the nested
module path shown above. `scripts/start.sh` supports both layouts.

## Runtime

The installed `move_base` command starts the wrapper and then
`move_base_cli.py` inside the `robonix` conda environment. The wrapper loads the
ROS and Robonix Python paths and clears old execution-gate variables before the
CLI starts.

`jetson/scripts/build.sh` runs Robonix MCP code generation. Generated files,
backups, logs, bytecode, maps, and credentials are not tracked.

The ROS launch and parameter files used on the X3 are listed by hash in
[ROS Configuration](ros-configuration.md).
