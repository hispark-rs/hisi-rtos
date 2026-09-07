#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Negative tests for requirement, proof-run, and HIL evidence contracts."""

from __future__ import annotations

import json
import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
from unittest.mock import patch


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
    for name in ("Cargo.toml", "Cargo.lock", "rust-toolchain.toml"):
        shutil.copy2(ROOT / name, root / name)
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
                "proof-evidence.py run-kani",
                "proof-evidence.py omitted-run-kani",
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
            "missing or unexpected tla execution receipts",
        )
    finally:
        shutil.rmtree(root)


def test_disabled_invariant_fails() -> None:
    root = fixture()
    try:
        config = root / "spec/ReadyOwnership.cfg"
        config.write_text("\n".join(line for line in config.read_text().splitlines() if not line.startswith("INVARIANT")))
        expect_failure(run_checker(root), "not enabled in config")
    finally:
        shutil.rmtree(root)


def test_skipped_kani_cannot_emit_pass() -> None:
    root = fixture()
    try:
        contract = root / "contract.json"
        result = run_proof(root, "contract", "--output", str(contract))
        if result.returncode != 0:
            raise AssertionError(result.stdout + result.stderr)
        expect_failure(run_proof(root, "record-kani", "--contract", str(contract),
                                 "--logs", str(root / "missing"), "--output", str(root / "run.json")),
                       "missing or unexpected kani execution receipts")
        if (root / "run.json").exists():
            raise AssertionError("skipped proof emitted success")
    finally:
        shutil.rmtree(root)


def load_module(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_execution_receipts_fail_closed() -> None:
    module = load_module("proof-evidence")
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        contract_file = root / "contract"
        contract_file.write_text("fixture")
        logs = root / "logs"
        logs.mkdir()
        item = {"name": "example", "reference": "proofs::example"}
        log = logs / "example.log"
        log.write_text("Checking harness proofs::example...\nComplete - 1 successfully verified harnesses, 0 failures, 1 total.\n")
        identity = {"commit": TEST_COMMIT, "run_id": "fixture", "run_attempt": 1}
        contract = {"kani": {"harnesses": [item], "version": "0.67.0"}}
        receipt = {"contract_sha256": module.sha256_file(contract_file), "ci": identity,
                   "command": module.proof_command(item, "kani"), "log_sha256": module.sha256_file(log),
                   "item": item, "version": "0.67.0", "exit_code": 0}
        args = SimpleNamespace(logs=logs, contract=contract_file, allow_local=True)
        with patch.object(module, "ci_identity", return_value=identity):
            for key, value in ((None, None), ("exit_code", 1), ("ci", {}), ("item", {}),
                               ("command", ["true"]), ("log_sha256", "0" * 64), ("version", "wrong"),
                               ("contract_sha256", "0" * 64)):
                altered = receipt.copy()
                if key:
                    altered[key] = value
                module.write_json(logs / "example.json", altered)
                try:
                    module.validate_receipts(args, contract, "kani")
                except SystemExit:
                    if key is None:
                        raise
                else:
                    if key is not None:
                        raise AssertionError(f"accepted mutated {key}")


def test_hil_bundle_checks_bytes_and_identity() -> None:
    module = load_module("verify-hil-bundle")
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "firmware.elf").write_bytes(b"\x7fELF-test-fixture-only")
        (root / "uart.log").write_text("TEST_ONLY\n")
        identity = {"runtime_commit": TEST_COMMIT, "parent_commit": "2" * 40,
                    "profile": "test-only", "marker": "TEST_ONLY"}
        summary = dict(identity, successful_runs=1, failed_runs=0,
                       firmware_sha256=[module.digest(root / "firmware.elf")])
        (root / "summary.json").write_text(json.dumps(summary))
        artifacts = [{"name": name, "kind": kind, "sha256": module.digest(root / name)}
                     for name, kind in (("firmware.elf", "firmware-elf"), ("uart.log", "uart-capture"), ("summary.json", "summary"))]
        manifest = dict(identity, schema=1, artifacts=artifacts)
        (root / "manifest.json").write_text(json.dumps(manifest))
        sha = module.digest(root / "manifest.json")
        module.verify(root, sha, TEST_COMMIT)
        for digest, runtime in (("0" * 64, TEST_COMMIT), (sha, "3" * 40)):
            try:
                module.verify(root, digest, runtime)
            except ValueError:
                pass
            else:
                raise AssertionError("accepted wrong bundle identity")
        (root / "firmware.elf").write_bytes(b"\x7fELF-mutated")
        try:
            module.verify(root, sha, TEST_COMMIT)
        except ValueError:
            pass
        else:
            raise AssertionError("accepted corrupted firmware")


def main() -> None:
    tests = (
        test_comment_is_not_a_definition,
        test_missing_kani_ci_invocation_fails,
        test_malformed_firmware_hash_fails,
        test_contract_digest_changes_with_source,
        test_tla_run_rejects_missing_logs,
        test_disabled_invariant_fails,
        test_skipped_kani_cannot_emit_pass,
        test_execution_receipts_fail_closed,
        test_hil_bundle_checks_bytes_and_identity,
    )
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    main()
