#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Validate scheduler evidence references and optionally emit a JSON inventory."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tomllib


ROOT = Path(
    os.environ.get("HISI_RTOS_REQUIREMENTS_ROOT", Path(__file__).resolve().parents[1])
).resolve()
MANIFEST = ROOT / "docs/spec/requirements.toml"
HIL_EVIDENCE_MANIFEST = ROOT / "docs/spec/hil-evidence.toml"
CI_WORKFLOW = ROOT / ".github/workflows/ci.yml"
EVIDENCE_KEYS = ("host_tests", "kani", "tla", "hil")
HIL_MARKER = re.compile(r"^(?:A3|A5R)_[A-Z0-9_]+$")
COMMIT_SHA = re.compile(r"^[0-9a-f]{40}$")
ARTIFACT_SHA = re.compile(r"^[0-9a-f]{64}$")
EVIDENCE_DATE = re.compile(r"^20[0-9]{2}-[0-9]{2}-[0-9]{2}$")
EXACT_HIL_BINDING = "exact-firmware"
LEGACY_HIL_BINDING = "legacy-no-firmware-hash"


def fail(message: str) -> None:
    raise SystemExit(message)


def split_references(value: str) -> list[str]:
    if value.startswith("NotApplicable:"):
        return []
    return [item.strip() for item in value.split(";") if item.strip()]


def source_corpus() -> dict[str, str]:
    roots = (ROOT / "src", ROOT / "tests")
    return {
        path.relative_to(ROOT).as_posix(): path.read_text()
        for root in roots
        if root.exists()
        for path in root.rglob("*.rs")
    }


def strip_rust_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", text)


def definition_pattern(symbol: str) -> re.Pattern[str]:
    leaf = re.escape(symbol.split("::")[-1])
    return re.compile(
        rf"^(?:\s*#\[[^\n]+\]\s*)*\s*(?:(?:pub(?:\([^)]*\))?|unsafe|async|const|extern\s+\"[^\"]+\")\s+)*"
        rf"(?:fn|struct|enum|trait|type|const|static|mod)\s+{leaf}\b"
        rf"|^\s*(?:pub(?:\([^)]*\))?\s+)?{leaf}\s*(?::|\(|\{{|=|,)",
        re.MULTILINE,
    )


def reference_definitions(reference: str, corpus: dict[str, str]) -> list[str]:
    path, separator, symbol = reference.partition(":")
    if separator and path.endswith(".rs"):
        text = corpus.get(path)
        if text is None:
            return []
        return [path] if definition_pattern(symbol).search(strip_rust_comments(text)) else []
    if not separator and reference.endswith(".rs"):
        return [reference] if reference in corpus else []
    pattern = definition_pattern(reference)
    return [
        source_path
        for source_path, text in corpus.items()
        if pattern.search(strip_rust_comments(text))
    ]


def reference_exists(reference: str, corpus: dict[str, str]) -> bool:
    return bool(reference_definitions(reference, corpus))


def validate_tla(reference: str, workflow: str) -> None:
    model, separator, invariant = reference.partition(":")
    if not separator:
        fail(f"invalid TLA reference {reference}")
    model_path = ROOT / model
    if not model_path.is_file():
        fail(f"TLA reference uses missing model {reference}")
    model_text = model_path.read_text()
    if not re.search(rf"\b{re.escape(invariant)}\b", model_text):
        fail(f"TLA reference uses missing invariant {reference}")
    config_path = model_path.with_suffix(".cfg")
    if not config_path.is_file():
        fail(f"TLA model has no config {model}")
    if model_path.name not in workflow or config_path.name not in workflow:
        fail(f"TLA model is not executed by CI: {model}")


def validate_kani(reference: str, corpus: dict[str, str], workflow: str) -> None:
    harness = reference.split("::")[-1]
    locations = reference_definitions(reference, corpus)
    if not locations:
        fail(f"Kani reference uses missing harness {reference}")
    proof_pattern = re.compile(
        rf"#\[kani::proof\]\s*(?:#\[[^\n]+\]\s*)*[^{{;]*\bfn\s+{re.escape(harness)}\b",
        re.DOTALL,
    )
    if not any(
        proof_pattern.search(strip_rust_comments(corpus[source_path]))
        for source_path in locations
    ):
        fail(f"Kani reference is not annotated as a proof harness: {reference}")
    if f"--harness {harness}" not in workflow:
        fail(f"Kani harness is not executed by CI: {reference}")


def validate_host_test(reference: str, corpus: dict[str, str]) -> None:
    locations = reference_definitions(reference, corpus)
    if not locations:
        fail(f"host test reference uses missing definition {reference}")
    if reference.endswith(".rs"):
        return
    test = reference.split("::")[-1]
    test_pattern = re.compile(
        rf"#\[test\]\s*(?:#\[[^\n]+\]\s*)*[^{{;]*\bfn\s+{re.escape(test)}\b",
        re.DOTALL,
    )
    if not any(
        test_pattern.search(strip_rust_comments(corpus[source_path]))
        for source_path in locations
    ):
        fail(f"host test reference is not annotated #[test]: {reference}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report",
        type=Path,
        help="write the validated evidence inventory as deterministic JSON",
    )
    return parser.parse_args()


def validate_artifacts(marker: str, artifacts: object) -> list[dict[str, str]]:
    if not isinstance(artifacts, list) or not artifacts:
        fail(f"{marker} must list at least one artifact")
    normalized: list[dict[str, str]] = []
    for artifact in artifacts:
        if not isinstance(artifact, dict):
            fail(f"{marker} contains a non-table artifact")
        kind = artifact.get("kind")
        name = artifact.get("name")
        sha256 = artifact.get("sha256")
        if not isinstance(kind, str) or not kind:
            fail(f"{marker} artifact has no kind")
        if not isinstance(name, str) or not name:
            fail(f"{marker} artifact has no name")
        if not isinstance(sha256, str) or ARTIFACT_SHA.fullmatch(sha256) is None:
            fail(f"{marker} artifact {name!r} has invalid SHA-256")
        normalized.append({"kind": kind, "name": name, "sha256": sha256})
    return normalized


def load_hil_evidence() -> dict[str, dict[str, object]]:
    manifest = tomllib.loads(HIL_EVIDENCE_MANIFEST.read_text())
    if manifest.get("schema") != 2:
        fail("hil-evidence.toml must use schema 2")
    entries = manifest.get("evidence", [])
    if not isinstance(entries, list):
        fail("hil-evidence.toml evidence entries are not an array")

    evidence_by_marker: dict[str, dict[str, object]] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            fail("hil-evidence.toml contains a non-table evidence entry")
        marker = entry.get("marker")
        if not isinstance(marker, str) or HIL_MARKER.fullmatch(marker) is None:
            fail(f"invalid HIL evidence marker {marker!r}")
        if marker in evidence_by_marker:
            fail(f"duplicate HIL evidence marker {marker}")
        target = entry.get("target")
        if not isinstance(target, str) or not target:
            fail(f"{marker} has no target")
        date = entry.get("date")
        if not isinstance(date, str) or EVIDENCE_DATE.fullmatch(date) is None:
            fail(f"{marker} has invalid evidence date {date!r}")
        if entry.get("result") != "pass":
            fail(f"{marker} evidence result must be pass")
        successful_runs = entry.get("successful_runs")
        if not isinstance(successful_runs, int) or successful_runs < 1:
            fail(f"{marker} successful_runs must be a positive integer")
        reset_mode = entry.get("reset_mode")
        if not isinstance(reset_mode, str) or not reset_mode:
            fail(f"{marker} has no reset_mode")
        parent_commit = entry.get("parent_commit")
        if (
            not isinstance(parent_commit, str)
            or COMMIT_SHA.fullmatch(parent_commit) is None
        ):
            fail(f"{marker} has invalid parent_commit {parent_commit!r}")
        evidence_url = entry.get("evidence_url")
        expected_prefix = (
            "https://github.com/hispark-rs/hisi-riscv-rs/blob/"
            f"{parent_commit}/"
        )
        if not isinstance(evidence_url, str) or not evidence_url.startswith(
            expected_prefix
        ):
            fail(f"{marker} evidence_url must use its immutable parent commit")
        profile = entry.get("profile")
        if not isinstance(profile, str) or not profile:
            fail(f"{marker} has no tested profile")
        claim_scope = entry.get("claim_scope")
        if not isinstance(claim_scope, str) or not claim_scope:
            fail(f"{marker} has no bounded claim_scope")
        binding = entry.get("binding")
        if binding not in (EXACT_HIL_BINDING, LEGACY_HIL_BINDING):
            fail(f"{marker} has invalid evidence binding {binding!r}")
        artifacts_value = entry.get("artifacts")
        artifacts = (
            validate_artifacts(marker, artifacts_value)
            if artifacts_value is not None
            else []
        )
        runtime_commit = entry.get("runtime_commit")
        if binding == EXACT_HIL_BINDING:
            if (
                not isinstance(runtime_commit, str)
                or COMMIT_SHA.fullmatch(runtime_commit) is None
            ):
                fail(f"{marker} exact evidence has invalid runtime_commit")
            if not artifacts:
                fail(f"{marker} exact evidence must list artifacts")
            if not any(artifact["kind"] == "firmware-elf" for artifact in artifacts):
                fail(f"{marker} exact evidence has no firmware-elf artifact")
            if "limitation" in entry:
                fail(f"{marker} exact evidence must not carry a legacy limitation")
        else:
            limitation = entry.get("limitation")
            if not isinstance(limitation, str) or not limitation:
                fail(f"{marker} legacy evidence must explain its limitation")
            if runtime_commit is not None and (
                not isinstance(runtime_commit, str)
                or COMMIT_SHA.fullmatch(runtime_commit) is None
            ):
                fail(f"{marker} legacy evidence has invalid runtime_commit")
        evidence_by_marker[marker] = entry
    return evidence_by_marker


def main() -> None:
    args = parse_args()
    manifest = tomllib.loads(MANIFEST.read_text())
    normative_spec = manifest.get("normative_spec")
    if not isinstance(normative_spec, str):
        fail("requirements.toml has no normative_spec")
    spec_path = ROOT / normative_spec
    if not spec_path.is_file():
        fail(f"requirements.toml references missing normative spec {normative_spec}")

    requirements = manifest.get("requirement", [])
    if not isinstance(requirements, list):
        fail("requirements.toml requirement entries are not an array")
    requirement_ids = [entry.get("id") for entry in requirements]
    if any(not isinstance(item, str) for item in requirement_ids):
        fail("requirement block without id")
    if len(requirement_ids) != len(set(requirement_ids)):
        fail("duplicate requirement id in requirements.toml")

    spec_ids = set(re.findall(r"RTOS-[A-Z]+-\d{3}", spec_path.read_text()))
    manifest_ids = set(requirement_ids)
    missing = sorted(spec_ids - manifest_ids)
    extra = sorted(manifest_ids - spec_ids)
    if missing or extra:
        fail(f"requirement drift: missing={missing}, extra={extra}")

    corpus = source_corpus()
    workflow = CI_WORKFLOW.read_text()
    for command in (
        "scripts/proof-evidence.py contract",
        "scripts/proof-evidence.py record-kani",
        "scripts/proof-evidence.py record-tla",
    ):
        if command not in workflow:
            fail(f"CI does not emit required proof evidence: {command}")
    kani_versions = set(re.findall(r'kani-version:\s*"([^"]+)"', workflow))
    if len(kani_versions) != 1:
        fail(f"CI must pin exactly one Kani version, found {sorted(kani_versions)}")
    tla_version_match = re.search(r"tlaplus/releases/download/v([^/]+)/", workflow)
    tla_hash_match = re.search(r'echo "([0-9a-f]{64})\s+/tmp/tla2tools\.jar"', workflow)
    if tla_version_match is None or tla_hash_match is None:
        fail("CI must pin the TLA+ release and SHA-256")
    hil_evidence = load_hil_evidence()
    referenced_hil_markers: set[str] = set()
    inventory = []
    for entry in requirements:
        requirement_id = entry["id"]
        if not any(key in entry for key in (*EVIDENCE_KEYS, "status")):
            fail(f"{requirement_id} has no evidence or explicit pending status")

        implementations = entry.get("implementation", [])
        if not isinstance(implementations, list):
            fail(f"{requirement_id} implementation must be an array")
        for reference in implementations:
            if not reference_exists(reference, corpus):
                fail(f"{requirement_id} references missing implementation {reference}")

        host_tests = entry.get("host_tests", [])
        if not isinstance(host_tests, list):
            fail(f"{requirement_id} host_tests must be an array")
        for reference in host_tests:
            validate_host_test(reference, corpus)

        tla_value = entry.get("tla", "")
        if tla_value and not isinstance(tla_value, str):
            fail(f"{requirement_id} tla must be a string")
        tla_references = split_references(tla_value)
        for reference in tla_references:
            validate_tla(reference, workflow)

        kani_value = entry.get("kani", "")
        if kani_value and not isinstance(kani_value, str):
            fail(f"{requirement_id} kani must be a string")
        kani_references = split_references(kani_value)
        for reference in kani_references:
            validate_kani(reference, corpus, workflow)

        hil = entry.get("hil", [])
        if not isinstance(hil, list):
            fail(f"{requirement_id} hil must be an array")
        for marker in hil:
            if not isinstance(marker, str) or HIL_MARKER.fullmatch(marker) is None:
                fail(f"{requirement_id} has invalid HIL marker {marker!r}")
            if marker not in hil_evidence:
                fail(f"{requirement_id} has no immutable HIL evidence for {marker}")
            referenced_hil_markers.add(marker)

        hil_records = [hil_evidence[marker] for marker in hil]
        hil_binding = (
            "exact"
            if hil_records
            and all(item["binding"] == EXACT_HIL_BINDING for item in hil_records)
            else "legacy"
            if hil_records
            else "not-required"
        )
        inventory.append(
            {
                "id": requirement_id,
                "status": entry.get(
                    "status",
                    (
                        "hil-evidence-exact"
                        if hil_binding == "exact"
                        else "hil-evidence-legacy"
                        if hil_binding == "legacy"
                        else "software-evidence"
                    ),
                ),
                "implementation": implementations,
                "host_tests": host_tests,
                "tla": tla_references,
                "kani": kani_references,
                "kani_not_applicable": (
                    kani_value if kani_value.startswith("NotApplicable:") else None
                ),
                "hil": hil,
                "hil_binding": hil_binding,
                "hil_evidence": hil_records,
            }
        )

    missing_hil_evidence = sorted(referenced_hil_markers - hil_evidence.keys())
    extra_hil_evidence = sorted(hil_evidence.keys() - referenced_hil_markers)
    if missing_hil_evidence or extra_hil_evidence:
        fail(
            "HIL evidence drift: "
            f"missing={missing_hil_evidence}, extra={extra_hil_evidence}"
        )

    report = {
        "schema": manifest.get("schema"),
        "evidence_scope": "contract-map",
        "evidence_scope_note": (
            "This inventory validates mappings and immutable references; Kani and "
            "TLA+ run manifests are emitted only by their completed CI jobs."
        ),
        "source_corpus_sha256": hashlib.sha256(
            "".join(
                f"{path}\0{corpus[path]}\0" for path in sorted(corpus)
            ).encode()
        ).hexdigest(),
        "normative_spec": normative_spec,
        "verification": {
            "kani": {
                "version": next(iter(kani_versions)),
                "harnesses": sorted(
                    {
                        reference.split("::")[-1]
                        for item in inventory
                        for reference in item["kani"]
                    }
                ),
            },
            "tla": {
                "version": tla_version_match.group(1),
                "sha256": tla_hash_match.group(1),
                "models": sorted(
                    {
                        reference.partition(":")[0]
                        for item in inventory
                        for reference in item["tla"]
                    }
                ),
            },
        },
        "requirements": inventory,
        "summary": {
            "total": len(inventory),
            "software_evidence": sum(
                item["status"] == "software-evidence" for item in inventory
            ),
            "hil_evidence_exact": sum(
                item["status"] == "hil-evidence-exact" for item in inventory
            ),
            "hil_evidence_legacy": sum(
                item["status"] == "hil-evidence-legacy" for item in inventory
            ),
            "hil_markers": len(referenced_hil_markers),
            "hil_markers_with_evidence": len(hil_evidence),
            "hil_markers_exact": sum(
                item["binding"] == EXACT_HIL_BINDING
                for item in hil_evidence.values()
            ),
            "hil_markers_legacy": sum(
                item["binding"] == LEGACY_HIL_BINDING
                for item in hil_evidence.values()
            ),
        },
    }
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    print(
        f"requirements: {len(inventory)} IDs aligned with {normative_spec}; "
        f"{report['summary']['hil_evidence_exact']} exact-HIL and "
        f"{report['summary']['hil_evidence_legacy']} legacy-HIL requirements; "
        f"{report['summary']['hil_markers_exact']}/{len(hil_evidence)} marker "
        "records bind exact firmware"
    )


if __name__ == "__main__":
    main()
