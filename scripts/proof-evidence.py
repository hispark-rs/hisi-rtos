#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Create reproducible proof contracts and completed-job evidence manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tomllib
import time


ROOT = Path(
    os.environ.get("HISI_RTOS_PROOF_ROOT", Path(__file__).resolve().parents[1])
).resolve()
REQUIREMENTS = ROOT / "docs/spec/requirements.toml"
WORKFLOW = ROOT / ".github/workflows/ci.yml"
SHA256 = re.compile(r"^[0-9a-f]{64}$")
TLC_SUCCESS = "Model checking completed. No error has been found."
LEGACY_MODELS = {
    "ReadyOwnershipLegacy": "Invariant ReadyHasExactlyOneOwner is violated",
    "SwitchIntentCreationLegacy": "Invariant PreparedSourceNotResumed is violated",
}


def fail(message: str) -> None:
    raise SystemExit(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def git_commit() -> str:
    override = os.environ.get("HISI_RTOS_SOURCE_COMMIT")
    if override:
        return override
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def source_files() -> list[Path]:
    files = [
        ROOT / "Cargo.toml",
        ROOT / "Cargo.lock",
        ROOT / "rust-toolchain.toml",
        ROOT / "docs/spec/scheduling.md",
        ROOT / "scripts/verify-hil-bundle.py",
        REQUIREMENTS,
        ROOT / "docs/spec/hil-evidence.toml",
        WORKFLOW,
        ROOT / "scripts/check-requirements.py",
        Path(__file__).resolve(),
    ]
    for directory, pattern in ((ROOT / "src", "*.rs"), (ROOT / "tests", "*.rs")):
        if directory.exists():
            files.extend(directory.rglob(pattern))
    files.extend((ROOT / "spec").glob("*.tla"))
    files.extend((ROOT / "spec").glob("*.cfg"))
    files.extend(path for path in (ROOT / "build.rs", ROOT / ".cargo/config.toml") if path.is_file())
    return sorted(set(files))


def tree_digest(files: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in files:
        relative = path.relative_to(ROOT).as_posix().encode()
        digest.update(relative)
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def split_references(value: str) -> list[str]:
    if not value or value.startswith("NotApplicable:"):
        return []
    return [item.strip() for item in value.split(";") if item.strip()]


def find_harness_file(harness: str) -> Path:
    pattern = re.compile(rf"\bfn\s+{re.escape(harness)}\b")
    matches = [
        path
        for path in (ROOT / "src").rglob("*.rs")
        if pattern.search(path.read_text())
    ]
    if len(matches) != 1:
        fail(f"Kani harness {harness} resolves to {len(matches)} source files")
    return matches[0]


def build_contract() -> dict[str, object]:
    manifest = tomllib.loads(REQUIREMENTS.read_text())
    subprocess.run(["uv", "run", "--script", str(ROOT / "scripts/check-requirements.py")], check=True, capture_output=True)
    workflow = WORKFLOW.read_text()
    requirements = manifest.get("requirement", [])
    kani_references = sorted(
        {
            reference
            for entry in requirements
            for reference in split_references(entry.get("kani", ""))
        }
    )
    harnesses = []
    for reference in kani_references:
        name = reference.split("::")[-1]
        source = find_harness_file(name)
        harnesses.append(
            {
                "name": name,
                "reference": reference,
                "source": source.relative_to(ROOT).as_posix(),
                "source_sha256": sha256_file(source),
            }
        )

    kani_versions = set(re.findall(r'kani-version:\s*"([^"]+)"', workflow))
    if len(kani_versions) != 1:
        fail("CI must pin exactly one Kani version")
    tla_version = re.search(r"tlaplus/releases/download/v([^/]+)/", workflow)
    tla_sha = re.search(r'echo "([0-9a-f]{64})\s+/tmp/tla2tools\.jar"', workflow)
    if tla_version is None or tla_sha is None:
        fail("CI must pin TLA+ version and SHA-256")

    model_references = sorted(
        {
            reference.partition(":")[0]
            for entry in requirements
            for reference in split_references(entry.get("tla", ""))
        }
    )
    models = []
    for model_reference in model_references:
        model = ROOT / model_reference
        config = model.with_suffix(".cfg")
        if not model.is_file() or not config.is_file():
            fail(f"missing TLA+ model pair for {model_reference}")
        models.append(
            {
                "name": model.stem,
                "model": model.relative_to(ROOT).as_posix(),
                "model_sha256": sha256_file(model),
                "config": config.relative_to(ROOT).as_posix(),
                "config_sha256": sha256_file(config),
                "expected": TLC_SUCCESS,
            }
        )
    for name, expected in LEGACY_MODELS.items():
        model = ROOT / "spec" / f"{name.removesuffix('Legacy')}.tla"
        config = ROOT / "spec" / f"{name}.cfg"
        if not model.is_file() or not config.is_file():
            fail(f"missing legacy counterexample pair for {name}")
        models.append(
            {
                "name": name,
                "model": model.relative_to(ROOT).as_posix(),
                "model_sha256": sha256_file(model),
                "config": config.relative_to(ROOT).as_posix(),
                "config_sha256": sha256_file(config),
                "expected": expected,
                "expected_result": "counterexample",
            }
        )

    files = source_files()
    return {
        "schema": 2,
        "evidence_scope": "proof-contract",
        "source_commit": git_commit(),
        "source_tree_sha256": tree_digest(files),
        "requirements_sha256": sha256_file(REQUIREMENTS),
        "workflow_sha256": sha256_file(WORKFLOW),
        "kani": {
            "version": next(iter(kani_versions)),
            "harnesses": harnesses,
        },
        "tla": {
            "version": tla_version.group(1),
            "tool_sha256": tla_sha.group(1),
            "models": models,
        },
    }


def load_contract(path: Path) -> dict[str, object]:
    contract = json.loads(path.read_text())
    current = build_contract()
    if contract != current:
        fail("proof contract does not match the checked-out source tree")
    return contract


def ci_identity(allow_local: bool) -> dict[str, object]:
    commit = os.environ.get("GITHUB_SHA")
    run_id = os.environ.get("GITHUB_RUN_ID")
    run_attempt = os.environ.get("GITHUB_RUN_ATTEMPT")
    if not allow_local and not all((commit, run_id, run_attempt)):
        fail("proof run evidence may only be recorded by GitHub Actions")
    return {
        "commit": commit or git_commit(),
        "run_id": run_id or "local",
        "run_attempt": int(run_attempt) if run_attempt else 0,
    }


def record_kani(args: argparse.Namespace) -> None:
    contract = load_contract(args.contract)
    identity = ci_identity(args.allow_local)
    if identity["commit"] != contract["source_commit"]:
        fail("Kani run commit does not match proof contract")
    receipts = validate_receipts(args, contract, "kani")
    write_json(
        args.output,
        {
            "schema": 2,
            "evidence_scope": "completed-proof-run",
            "kind": "kani",
            "result": "pass",
            "contract_sha256": sha256_file(args.contract),
            "source_commit": contract["source_commit"],
            "source_tree_sha256": contract["source_tree_sha256"],
            "ci": identity,
            "version": contract["kani"]["version"],
            "receipts": receipts,
            "harnesses": [
                item["name"] for item in contract["kani"]["harnesses"]
            ],
        },
    )


def record_tla(args: argparse.Namespace) -> None:
    contract = load_contract(args.contract)
    identity = ci_identity(args.allow_local)
    if identity["commit"] != contract["source_commit"]:
        fail("TLA+ run commit does not match proof contract")
    receipts = validate_receipts(args, contract, "tla")
    logs = []
    for model in contract["tla"]["models"]:
        log = args.logs / f"{model['name']}.log"
        if not log.is_file():
            fail(f"missing TLA+ log {log.name}")
        text = log.read_text(errors="replace")
        expected = model["expected"]
        if expected not in text:
            fail(f"TLA+ log {log.name} lacks expected result: {expected}")
        logs.append(
            {
                "name": model["name"],
                "expected_result": model.get("expected_result", "pass"),
                "sha256": sha256_file(log),
            }
        )
    write_json(
        args.output,
        {
            "schema": 2,
            "evidence_scope": "completed-proof-run",
            "kind": "tla",
            "result": "pass",
            "contract_sha256": sha256_file(args.contract),
            "source_commit": contract["source_commit"],
            "source_tree_sha256": contract["source_tree_sha256"],
            "ci": identity,
            "version": contract["tla"]["version"],
            "tool_sha256": contract["tla"]["tool_sha256"],
            "logs": logs,
            "receipts": receipts,
        },
    )


def proof_items(contract: dict, kind: str) -> list[dict]:
    return contract[kind]["harnesses" if kind == "kani" else "models"]


def proof_command(item: dict, kind: str) -> list[str]:
    if kind == "kani":
        return ["cargo", "kani", "--harness", item["reference"]]
    return ["java", "-cp", "/tmp/tla2tools.jar", "tlc2.TLC", "-workers", "4",
            "-config", Path(item["config"]).name, Path(item["model"]).name]


def verify_result(item: dict, kind: str, receipt: dict, log: str) -> None:
    if kind == "kani":
        if (receipt["exit_code"] != 0
                or f"Checking harness {item['reference']}..." not in log
                or "Complete - 1 successfully verified harnesses, 0 failures, 1 total." not in log):
            fail(f"Kani harness did not pass exactly once: {item['name']}")
    else:
        expected_code = 12 if item.get("expected_result") == "counterexample" else 0
        if receipt["exit_code"] != expected_code or item["expected"] not in log:
            fail(f"TLC did not return the expected result: {item['name']}")


def validate_receipts(args: argparse.Namespace, contract: dict, kind: str) -> list[dict]:
    receipts = []
    identity = ci_identity(args.allow_local)
    expected = {item["name"] for item in proof_items(contract, kind)}
    actual = {path.stem for path in args.logs.glob("*.json")}
    if actual != expected:
        fail(f"missing or unexpected {kind} execution receipts: missing={sorted(expected - actual)}, extra={sorted(actual - expected)}")
    for item in proof_items(contract, kind):
        receipt = json.loads((args.logs / f"{item['name']}.json").read_text())
        log_path = args.logs / f"{item['name']}.log"
        if not log_path.is_file():
            fail(f"missing {kind} log {log_path.name}")
        for key, value in {
            "contract_sha256": sha256_file(args.contract), "ci": identity,
            "command": proof_command(item, kind), "log_sha256": sha256_file(log_path),
            "item": item, "version": contract[kind]["version"],
        }.items():
            if receipt.get(key) != value:
                fail(f"{kind} receipt {item['name']} mismatches {key}")
        verify_result(item, kind, receipt, log_path.read_text(errors="replace"))
        receipts.append(receipt)
    return receipts


def execute_proofs(args: argparse.Namespace, kind: str) -> None:
    contract = load_contract(args.contract)
    identity = ci_identity(args.allow_local)
    if identity["commit"] != contract["source_commit"]:
        fail("proof execution commit does not match contract")
    args.logs.mkdir(parents=True, exist_ok=True)
    if any(args.logs.iterdir()):
        fail("proof execution requires a fresh receipt directory")
    if kind == "kani":
        version = subprocess.check_output(["cargo", "kani", "--version"], text=True).strip()
        if contract[kind]["version"] not in version.split():
            fail(f"unexpected Kani version: {version}")
    else:
        if sha256_file(Path("/tmp/tla2tools.jar")) != contract[kind]["tool_sha256"]:
            fail("TLC tool checksum mismatch")
        version = contract[kind]["version"]
    for item in proof_items(contract, kind):
        command = proof_command(item, kind)
        started = time.monotonic()
        completed = subprocess.run(command, cwd=ROOT if kind == "kani" else ROOT / "spec",
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=1800)
        print(completed.stdout, flush=True)
        log = args.logs / f"{item['name']}.log"
        log.write_text(completed.stdout)
        receipt = {"schema": 1, "command": command, "exit_code": completed.returncode,
                   "duration_seconds": time.monotonic() - started,
                   "version": contract[kind]["version"], "tool_identity": version,
                   "ci": identity, "item": item,
                   "contract_sha256": sha256_file(args.contract), "log_sha256": sha256_file(log)}
        write_json(args.logs / f"{item['name']}.json", receipt)
        verify_result(item, kind, receipt, completed.stdout)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    contract = subparsers.add_parser("contract")
    contract.add_argument("--output", type=Path, required=True)

    for name in ("record-kani", "record-tla", "run-kani", "run-tla"):
        command = subparsers.add_parser(name)
        command.add_argument("--contract", type=Path, required=True)
        command.add_argument("--output", type=Path, required=name.startswith("record"))
        command.add_argument("--allow-local", action="store_true")
        command.add_argument("--logs", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.command == "contract":
        write_json(args.output, build_contract())
    elif args.command == "record-kani":
        record_kani(args)
    elif args.command.startswith("run-"):
        execute_proofs(args, args.command.removeprefix("run-"))
    else:
        record_tla(args)


if __name__ == "__main__":
    main()
