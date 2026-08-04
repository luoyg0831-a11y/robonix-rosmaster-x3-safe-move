#!/usr/bin/env python3
"""Check the repository fields consumed by Robonix Package Catalog CI."""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "package_manifest.yaml"
JETSON_MANIFEST = ROOT / "jetson/package_manifest.yaml"
EXPECTED_NAME = "robonix.primitive.yahboom.rosmaster_x3.safe_move"
EXPECTED_CAPABILITIES = {
    "get_status",
    "check_nav_ready",
    "preview_safe_waypoint",
    "prepare_safe_navigation",
    "preview_move_options",
    "prepare_selected_move",
    "execute_prepared_navigation",
    "cancel_nav_goal",
}


def fail(message: str) -> None:
    raise SystemExit("catalog_package_audit=FAIL: " + message)


def value(text: str, key: str) -> str:
    match = re.search(rf"^\s{{2}}{re.escape(key)}:\s*(.+?)\s*$", text, re.M)
    if not match:
        fail(f"missing package.{key}")
    return match.group(1)


def tracked_mode(relative: str) -> str:
    completed = subprocess.run(
        ["git", "ls-files", "--stage", "--", relative],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    fields = completed.stdout.split()
    if not fields:
        fail(f"required release file is not tracked: {relative}")
    return fields[0]


def main() -> None:
    text = MANIFEST.read_text(encoding="utf-8")
    nested = JETSON_MANIFEST.read_text(encoding="utf-8")
    version = (ROOT / "VERSION").read_text(encoding="utf-8").strip()

    if not text.startswith("manifestVersion: 1\n"):
        fail("root manifestVersion is not 1")
    if value(text, "name") != EXPECTED_NAME:
        fail("unexpected Catalog package name")
    if value(text, "version") != version:
        fail("root manifest and VERSION differ")
    for key in ("description", "tags", "maintainers", "license"):
        value(text, key)
    if "luoyg0831 <luoyg0831@gmail.com>" not in text:
        fail("maintainer is not in Name <email> format")
    if value(text, "license") != "Apache-2.0":
        fail("license is not an SPDX Apache-2.0 identifier")

    names = {
        item.rsplit("/", 1)[-1]
        for item in re.findall(r"^\s+- name:\s*(\S+)\s*$", text, re.M)
    }
    if names != EXPECTED_CAPABILITIES:
        fail(f"unexpected root capability set: {sorted(names)}")

    paths = re.findall(r"^\s+path:\s*(\S+)\s*$", text, re.M)
    if len(paths) != len(EXPECTED_CAPABILITIES):
        fail("every capability must have one local TOML path")
    for relative in paths:
        if not (ROOT / relative).is_file():
            fail(f"capability path does not exist: {relative}")

    if f"name: {EXPECTED_NAME}" not in nested:
        fail("Jetson manifest package identity differs")
    if f"version: {version}" not in nested:
        fail("Jetson manifest version differs")
    executable_paths = (
        "jetson/scripts/build.sh",
        "jetson/scripts/start.sh",
        "jetson/scripts/install_move_base_command.sh",
        "jetson/scripts/move_base",
    )
    for relative in executable_paths:
        if not os.access(ROOT / relative, os.X_OK):
            fail(f"script is not executable: {relative}")
        if tracked_mode(relative) != "100755":
            fail(f"Git mode is not 100755: {relative}")

    print(f"catalog_package_name={EXPECTED_NAME}")
    print(f"catalog_package_version={version}")
    print("catalog_metadata_complete=1")
    print("catalog_capability_count=8")
    print("catalog_package_audit=PASS")


if __name__ == "__main__":
    main()
