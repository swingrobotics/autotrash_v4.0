#!/usr/bin/env python3
"""Fail when the source tree contains public-repository data leak hazards."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

FORBIDDEN_DIRECTORY_PARTS = {
    "recordings",
    "field-tests",
    "gnss-backups",
    "maps",
    "models",
    "datasets",
    "gps-routes",
    "installer-dist",
}

FORBIDDEN_BASENAMES = {
    ".env",
    "ntrip-config.json",
    "credentials.json",
    "id_rsa",
    "id_ed25519",
}

FORBIDDEN_SUFFIXES = {
    ".key",
    ".p12",
    ".pfx",
}

SECRET_PATTERNS = {
    "private key": re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "GitHub token": re.compile(r"\b(?:ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})\b"),
    "AWS access key": re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
}

GPS_CSV_HEADER = re.compile(
    r"(?:^|,)(?:latitude|lat)(?:,|$).*?(?:^|,)(?:longitude|lon|lng)(?:,|$)",
    re.IGNORECASE,
)

TEXT_SUFFIXES = {
    ".py", ".ps1", ".sh", ".cmd", ".bat", ".md", ".txt", ".json",
    ".yaml", ".yml", ".toml", ".ini", ".conf", ".service", ".sudoers",
    ".csv", ".ino",
}


def tracked_files() -> list[Path]:
    try:
        result = subprocess.run(
            ["git", "-C", str(ROOT), "ls-files", "-z"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        return [ROOT / item.decode("utf-8") for item in result.stdout.split(b"\0") if item]
    except (OSError, subprocess.CalledProcessError):
        return [path for path in ROOT.rglob("*") if path.is_file() and ".git" not in path.parts]


def main() -> int:
    errors: list[str] = []

    for path in tracked_files():
        relative = path.relative_to(ROOT)
        parts = set(relative.parts[:-1])
        name = relative.name
        lower_name = name.lower()

        blocked_dirs = parts & FORBIDDEN_DIRECTORY_PARTS
        if blocked_dirs:
            errors.append(f"forbidden runtime/private-data path: {relative}")
            continue

        if lower_name in FORBIDDEN_BASENAMES and lower_name != ".env.example":
            errors.append(f"forbidden credential/config filename: {relative}")
            continue

        if relative.suffix.lower() in FORBIDDEN_SUFFIXES:
            errors.append(f"forbidden private-key/container suffix: {relative}")
            continue

        if relative.suffix.lower() not in TEXT_SUFFIXES and name not in {".gitignore"}:
            continue

        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as error:
            errors.append(f"could not inspect {relative}: {error}")
            continue

        for label, pattern in SECRET_PATTERNS.items():
            if pattern.search(text):
                errors.append(f"possible {label} in {relative}")

        if relative.suffix.lower() == ".csv":
            first_line = text.splitlines()[0] if text else ""
            if GPS_CSV_HEADER.search(first_line):
                errors.append(f"GPS-coordinate CSV must not be committed: {relative}")

    if errors:
        print("PUBLIC_REPOSITORY_AUDIT_FAIL")
        for error in sorted(set(errors)):
            print(f"- {error}")
        return 1

    print("PUBLIC_REPOSITORY_AUDIT_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
