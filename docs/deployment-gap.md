# Deployment Gap

This repository was assembled from a local engineering snapshot while the Jetson was unavailable. It intentionally does not invent missing production files.

## Present locally

- 0.08 m candidate `main.py`
- deployed-generation `x3_bridge.py` and CLI snapshots
- public safe capability contracts and service definitions
- wrapper and command-install scripts
- four offline test suites

## Must be recovered or verified over SSH

- exact `/home/jetson/robonix` commit and dirty state;
- current production hashes before any replacement;
- package `scripts/build.sh` and `scripts/start.sh`;
- generated Robonix code provenance and rebuild instructions;
- exact ROS launch/config file provenance and licensed diffs;
- current command symlink and environment setup;
- final no-motion and hardware acceptance evidence.

## Finalization rule

Remote files must first be copied into a dated read-only inventory with SHA256 values. The local candidate must never overwrite an unknown remote version. After comparison, production files can be reconciled in a new release without rewriting the `0.1.0-local-preview` history.

