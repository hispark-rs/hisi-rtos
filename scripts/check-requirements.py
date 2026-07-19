#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Fail when normative scheduler requirements drift from their evidence map."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "docs/spec/requirements.toml"

manifest = MANIFEST.read_text()
normative_match = re.search(r'^normative_spec = "([^"]+)"$', manifest, re.MULTILINE)
if normative_match is None:
    raise SystemExit("requirements.toml has no normative_spec")
normative_spec = normative_match.group(1)
spec = (ROOT / normative_spec).read_text()
source_files = {path.relative_to(ROOT).as_posix(): path.read_text() for path in (ROOT / "src").rglob("*.rs")}

spec_ids = set(re.findall(r"RTOS-[A-Z]+-\d{3}", spec))
blocks = manifest.split("[[requirement]]")[1:]
entries = []
for block in blocks:
    id_match = re.search(r'^id = "([^"]+)"$', block, re.MULTILINE)
    if id_match is None:
        raise SystemExit("requirement block without id")
    entries.append((id_match.group(1), block))
manifest_ids = [requirement_id for requirement_id, _ in entries]
if len(manifest_ids) != len(set(manifest_ids)):
    raise SystemExit("duplicate requirement id in requirements.toml")

missing = sorted(spec_ids - set(manifest_ids))
extra = sorted(set(manifest_ids) - spec_ids)
if missing or extra:
    raise SystemExit(f"requirement drift: missing={missing}, extra={extra}")

for requirement_id, block in entries:
    if not re.search(r'^(host_tests|hil|kani|tla|status)\s*=', block, re.MULTILINE):
        raise SystemExit(f"{requirement_id} has no evidence or explicit pending status")
    implementation_match = re.search(r'^implementation = \[(.*?)\]$', block, re.MULTILINE)
    implementations = [] if implementation_match is None else re.findall(r'"([^"]+)"', implementation_match.group(1))
    for implementation in implementations:
        path, separator, symbol = implementation.partition(":")
        if not separator or path not in source_files:
            raise SystemExit(f"{requirement_id} references missing source file {implementation}")
        leaf = symbol.split("::")[-1]
        if leaf and leaf not in source_files[path]:
            raise SystemExit(f"{requirement_id} references missing symbol {implementation}")

print(f"requirements: {len(entries)} IDs aligned with {normative_spec}")
