#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
PKG="$(cd "$PKG" && pwd)"
CLEAN="${RBNX_BUILD_CLEAN:-}"

FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[rosmaster_x3_bridge/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[rosmaster_x3_bridge/build] done."
