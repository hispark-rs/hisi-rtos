#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Negative tests for requirement, proof-run, and HIL evidence contracts."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts/check-requirements.py"
PROOF = ROOT / "scripts/proof-evidence.py"
TEST_COMMIT = "1" * 40


def fixture() -> Path:
    root = Path(tempfile.mkdtemp(prefix="hisi-rtos-evidence-"))
    for relative in ("src", "tests", "spec", "docs/spec", ".github/workflows"):
        shutil.copytree(ROOT / relative, root / relative)
    (root / "scripts").mkdir()
    shutil.copy2(CHECKER, root / "scripts/check-requirements.py")
    shutil.copy2(PROOF, root / "scripts/proof-evidence.py")
    return root


def run_checker(root: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["HISI_RTOS_REQUIREMENTS_ROOT"] = str(root)
    return subprocess.run(
        ["uv", "run", "--script", str(root / "scripts/check-requirements.py")],
        env=env,
        capture_output=True,
        text=True,
    )


def proof_env(root: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["HISI_RTOS_PROOF_ROOT"] = str(root)
    env["HISI_RTOS_SOURCE_COMMIT"] = TEST_COMMIT
    env["GITHUB_SHA"] = TEST_COMMIT
    env["GITHUB_RUN_ID"] = "fixture"
    env["GITHUB_RUN_ATTEMPT"] = "1"
    return env


def run_proof(root: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["uv", "run", "--script", str(root / "scripts/proof-evidence.py"), *arguments],
        env=proof_env(root),
        capture_output=True,
        text=True,
    )


def expect_failure(result: subprocess.CompletedProcess[str], fragment: str) -> None:
    output = result.stdout + result.stderr
    if result.returncode == 0 or fragment not in output:
        raise AssertionError(
            f"expected failure containing {fragment!r}, got rc={result.returncode}:\n{output}"
        )


def test_comment_is_not_a_definition() -> None:
    root = fixture()
    try:
        requirements = root / "docs/spec/requirements.toml"
        requirements.write_text(
            requirements.read_text()
            + '\n[[requirement]]\nid = "RTOS-TEST-999"\n'
            + 'implementation = ["src/runtime.rs:missing_production_helper"]\n'
            + "host_tests = []\n"
        )
        scheduling = root / "docs/spec/scheduling.md"
        scheduling.write_text(scheduling.read_text() + "\nRTOS-TEST-999\n")
        runtime = root / "src/runtime.rs"
        runtime.write_text(
            runtime.read_text() + "\n// fn missing_production_helper() was removed\n"
        )
        expect_failure(run_checker(root), "references missing implementation")
    finally:
        shutil.rmtree(root)


def test_missing_kani_ci_invocation_fails() -> None:
    root = fixture()
    try:
        workflow = root / ".github/workflows/ci.yml"
        workflow.write_text(
            workflow.read_text().replace(
                "--harness remaining_never_exceeds_capacity",
                "--harness omitted_remaining_capacity_harness",
                1,
            )
        )
        expect_failure(run_checker(root), "Kani harness is not executed by CI")
    finally:
        shutil.rmtree(root)


def test_malformed_firmware_hash_fails() -> None:
    root = fixture()
    try:
        manifest = root / "docs/spec/hil-evidence.toml"
        manifest.write_text(
            manifest.read_text().replace(
                "55c047b947912e6e6aea8019da9da87a1c2917b5a3e1940a0a6fd6b3af06d235",
                "not-a-sha256",
                1,
            )
        )
        expect_failure(run_checker(root), "invalid SHA-256")
    finally:
        shutil.rmtree(root)


def test_contract_digest_changes_with_source() -> None:
    root = fixture()
    try:
        first = root / "first.json"
        second = root / "second.json"
        result = run_proof(root, "contract", "--output", str(first))
        if result.returncode != 0:
            raise AssertionError(result.stdout + result.stderr)
        runtime = root / "src/runtime.rs"
        runtime.write_text(runtime.read_text() + "\n// evidence mutation\n")
        result = run_proof(root, "contract", "--output", str(second))
        if result.returncode != 0:
            raise AssertionError(result.stdout + result.stderr)
        before = json.loads(first.read_text())["source_tree_sha256"]
        after = json.loads(second.read_text())["source_tree_sha256"]
        if before == after:
            raise AssertionError("source mutation did not change proof contract digest")
    finally:
        shutil.rmtree(root)


def test_tla_run_rejects_missing_logs() -> None:
    root = fixture()
    try:
        contract = root / "contract.json"
        result = run_proof(root, "contract", "--output", str(contract))
        if result.returncode != 0:
            raise AssertionError(result.stdout + result.stderr)
        logs = root / "empty-logs"
        logs.mkdir()
        expect_failure(
            run_proof(
                root,
                "record-tla",
                "--contract",
                str(contract),
                "--logs",
                str(logs),
                "--output",
                str(root / "tla-run.json"),
                "--allow-local",
            ),
            "missing TLA+ log",
        )
    finally:
        shutil.rmtree(root)


def main() -> None:
    tests = (
        test_comment_is_not_a_definition,
        test_missing_kani_ci_invocation_fails,
        test_malformed_firmware_hash_fails,
        test_contract_digest_changes_with_source,
        test_tla_run_rejects_missing_logs,
    )
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    main()
