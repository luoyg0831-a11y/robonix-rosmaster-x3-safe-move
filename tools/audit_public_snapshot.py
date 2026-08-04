#!/usr/bin/env python3
"""Reject common field-data and credential artifacts from the public snapshot."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_SUFFIXES = {
    ".bag",
    ".db3",
    ".jpeg",
    ".jpg",
    ".log",
    ".mp4",
    ".pgm",
    ".png",
}
TEXT_PATTERNS = {
    "private IPv4 address": re.compile(
        r"(?<![0-9])(?:10\.(?:[0-9]{1,3}\.){2}[0-9]{1,3}"
        r"|192\.168\.(?:[0-9]{1,3}\.)[0-9]{1,3}"
        r"|172\.(?:1[6-9]|2[0-9]|3[01])\.(?:[0-9]{1,3}\.)[0-9]{1,3})(?![0-9])"
    ),
    "SSH public host key": re.compile(r"\bssh-(?:ed25519|rsa)\s+[A-Za-z0-9+/=]+"),
    "private key block": re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "literal credential assignment": re.compile(
        r"(?i)\b(?:password|passwd|api[_-]?key|access[_-]?token)\b\s*[:=]\s*['\"][^'\"\r\n]+['\"]"
    ),
}


def tracked_files() -> list[Path]:
    completed = subprocess.run(
        ["git", "ls-files", "-co", "--exclude-standard"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return [
        ROOT / line
        for line in completed.stdout.splitlines()
        if line and (ROOT / line).is_file()
    ]


def main() -> None:
    findings: list[str] = []
    files = tracked_files()
    for path in files:
        relative = path.relative_to(ROOT).as_posix()
        if path.suffix.lower() in FORBIDDEN_SUFFIXES:
            findings.append(f"forbidden evidence file: {relative}")
            continue
        try:
            content = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            findings.append(f"unreviewed binary file: {relative}")
            continue
        for label, pattern in TEXT_PATTERNS.items():
            if pattern.search(content):
                findings.append(f"{label}: {relative}")

    if findings:
        print("public_snapshot_audit=FAIL")
        for finding in findings:
            print(finding)
        raise SystemExit(1)

    print(f"reviewed_file_count={len(files)}")
    print("private_network_addresses=0")
    print("credential_artifacts=0")
    print("raw_field_evidence_files=0")
    print("public_snapshot_audit=PASS")


if __name__ == "__main__":
    main()
