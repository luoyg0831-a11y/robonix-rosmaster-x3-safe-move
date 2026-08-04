# Deployment Map

## Public repository to Jetson

| Public path | Jetson production path or role |
|---|---|
| `package_manifest.yaml` | Catalog-facing root manifest |
| `jetson/package_manifest.yaml` | `/home/jetson/rosmaster_x3_deploy/primitives/rosmaster_x3_bridge/package_manifest.yaml` |
| `jetson/main.py` | `.../rosmaster_x3_bridge/rosmaster_x3_bridge/main.py` |
| `jetson/x3_bridge.py` | `/home/jetson/x3_rosbridge_adapter/scripts/x3_bridge.py` |
| `jetson/scripts/move_base_cli.py` | `/home/jetson/rosmaster_x3_deploy/scripts/move_base_cli.py` |
| `jetson/scripts/move_base` | `/home/jetson/rosmaster_x3_deploy/scripts/move_base` |

The source snapshot keeps `main.py` at `jetson/main.py` for offline tests. The
production package imports it from the nested `rosmaster_x3_bridge/` module.
`scripts/start.sh` supports both layouts without copying or changing production.

## Command and environment

Inside the `robonix` conda environment:

```text
/home/jetson/miniforge3/envs/robonix/bin/move_base
  -> /home/jetson/rosmaster_x3_deploy/scripts/move_base
```

The wrapper activates the `robonix` Python 3.10 environment, sources Cargo,
sets loopback ROS master/IP values, and builds `PYTHONPATH` from the Robonix API,
external adapter, package, MCP types, and generated protobuf modules. It unsets
all legacy and prepared execution gate variables before starting the CLI.

## Build and codegen

- `rbnx`: `/home/jetson/.cargo/bin/rbnx`, version 0.1.0.
- Robonix source: `/home/jetson/robonix`, clean `dev` at
  `6bf549f954fb8bc21997e819741f51a34bb51ec9` during inventory.
- Codegen command: `rbnx codegen -p <package> --mcp --clean` via
  `jetson/scripts/build.sh`.
- Codegen requires conda Python to precede system Python in `PATH`, because the
  conda environment supplies `grpc_tools`.
- Generated `rbnx-build/` content is reproducible build output and is not
  committed.

## ROS configuration source

The ROS1 workspace root is `/home/jetson/yahboomcar_ws`. Navigation starts from
`yahboomcar_nav/launch/laser_bringup.launch` and
`yahboomcar_nav/launch/yahboomcar_navigation.launch`; the latter includes AMCL,
map server, move_base, and rosbridge. Shell startup sets `ROBOT_TYPE=X3` and
`RPLIDAR_TYPE=a1`. Vendor source/config files are not copied into this public
repository because their redistribution license remains unclear.
