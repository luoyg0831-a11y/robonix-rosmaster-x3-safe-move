#!/usr/bin/env bash
set -euo pipefail

SOURCE=/home/jetson/rosmaster_x3_deploy/scripts/move_base
TARGET=/home/jetson/miniforge3/envs/robonix/bin/move_base

test -x "$SOURCE"
ln -sfn "$SOURCE" "$TARGET"
test "$(readlink -f "$TARGET")" = "$(readlink -f "$SOURCE")"
echo "installed: $TARGET -> $SOURCE"
