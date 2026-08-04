#!/usr/bin/env bash
set -euo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
PKG_ROOT="$(cd "$PKG_ROOT" && pwd)"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$PKG_ROOT/rbnx-build/codegen/proto_gen:/home/jetson/x3_rosbridge_adapter/scripts:${PYTHONPATH:-}"

if [[ -f "$PKG_ROOT/rosmaster_x3_bridge/main.py" ]]; then
    exec python3 -m rosmaster_x3_bridge.main
fi

exec python3 "$PKG_ROOT/main.py"
