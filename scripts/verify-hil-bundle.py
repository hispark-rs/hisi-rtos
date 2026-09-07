#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Verify downloaded HIL bytes against a separately pinned bundle manifest.

Integrity and identity binding only: this cannot prove that hardware was used.
The caller must retain the immutable source/CI provenance of the expected digest.
"""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(root: Path, expected_manifest: str, expected_runtime: str) -> dict:
    manifest_path = root / "manifest.json"
    if digest(manifest_path) != expected_manifest:
        raise ValueError("bundle manifest checksum mismatch")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != 1 or manifest.get("runtime_commit") != expected_runtime:
        raise ValueError("bundle runtime identity/schema mismatch")
    identity_keys = ("runtime_commit", "parent_commit", "profile", "marker")
    for key in identity_keys:
        if not manifest.get(key):
            raise ValueError(f"missing bundle identity {key}")
    for key in ("runtime_commit", "parent_commit"):
        value = manifest[key]
        if len(value) != 40 or any(c not in "0123456789abcdef" for c in value):
            raise ValueError(f"invalid {key}")
    artifacts = manifest.get("artifacts", [])
    seen, kinds = set(), set()
    elf_hashes = []
    uart_files = set()
    summary = None
    for artifact in artifacts:
        name = artifact["name"]
        path = (root / name).resolve()
        if name in seen or path == root.resolve() or root.resolve() not in path.parents:
            raise ValueError("duplicate or unsafe artifact path")
        seen.add(name)
        kinds.add(artifact["kind"])
        if digest(path) != artifact["sha256"]:
            raise ValueError(f"artifact checksum mismatch: {name}")
        if artifact["kind"] == "firmware-elf":
            if not path.read_bytes().startswith(b"\x7fELF"):
                raise ValueError("firmware is not ELF")
            elf_hashes.append(artifact["sha256"])
        elif artifact["kind"] == "summary":
            if summary is not None:
                raise ValueError("multiple summaries")
            summary = json.loads(path.read_text())
        elif artifact["kind"] == "uart-capture":
            uart_files.add(name)
    if not {"firmware-elf", "summary", "uart-capture"} <= kinds:
        raise ValueError("bundle lacks firmware/summary/raw UART evidence")
    if any(summary.get(key) != manifest[key] for key in identity_keys):
        raise ValueError("summary identity disagrees with bundle")
    if sorted(summary.get("firmware_sha256", [])) != sorted(elf_hashes):
        raise ValueError("summary firmware identity mismatch")
    if summary.get("failed_runs") != 0 or summary.get("successful_runs", 0) < 1:
        raise ValueError("summary does not meet declared pass gate")
    runs = summary.get("runs", [])
    if len(runs) != summary["successful_runs"] or len({run["id"] for run in runs}) != len(runs):
        raise ValueError("summary run count/identity mismatch")
    consumed = set()
    for run in runs:
        names = set(run.get("uart_files", []))
        if run.get("result") != "pass" or not names or not names <= uart_files or names & consumed:
            raise ValueError("run lacks unique verified raw captures")
        consumed.update(names)
        if not any(manifest["marker"] in (root / name).read_text(errors="replace") for name in names):
            raise ValueError("run raw captures lack the declared HIL marker")
    if consumed != uart_files:
        raise ValueError("raw captures not accounted for in run summary")
    return {"schema": 1, "binding": "artifact-verified", "manifest_sha256": expected_manifest,
            "runtime_commit": expected_runtime, "artifacts": artifacts,
            "scope": "downloaded byte integrity and identity, not independent silicon execution"}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--runtime-commit", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.bundle, args.manifest_sha256, args.runtime_commit)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
