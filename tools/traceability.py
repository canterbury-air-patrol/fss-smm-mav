#!/usr/bin/env python3
"""
Collect CAP master test-plan (TC-MAV-*/TC-FS-*/...) traceability evidence
from this repo's Catch2 test binaries (todo/66).

Runs each binary with the JSON reporter, extracts every `[TC-...]` Catch2
tag together with the outcome of the test case carrying it, and emits the
same {"generated": ..., "test_ids": {...}} interchange format used by the
Tier-3 pytest collector (CAP/tools/test/helpers/traceability.py), so the
master-plan audit can merge evidence from both without a translation step.

Usage: tools/traceability.py [--out PATH] [BINARY ...]
Defaults to tests/fmu_test and tests/mav_io_test relative to the repo root
if no binaries are given. Exits non-zero if a binary fails to run at all,
or if any test case tagged with a TC ID failed.
"""
import argparse
import datetime
import json
import pathlib
import re
import subprocess
import sys

TC_TAG_RE = re.compile(r"^TC-[A-Z]+-[0-9]+$")


def run_binary(binary):
    """Run one Catch2 binary with the JSON reporter and return its parsed report."""
    result = subprocess.run(
        [str(binary), "--reporter", "JSON"],
        capture_output=True,
        text=True,
        check=False,
    )
    try:
        return json.loads(result.stdout), result.returncode
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"{binary}: did not produce valid JSON output (exit code {result.returncode}); "
            f"stderr:\n{result.stderr}"
        ) from exc


def outcome_for(test_case):
    """Derive passed/failed from a Catch2 JSON test-case's assertion totals."""
    totals = test_case.get("totals", {}).get("assertions", {})
    if totals.get("failed", 0) > 0:
        return "failed"
    return "passed"


def collect_from_binary(binary):
    """Return {test_id: [{"nodeid": ..., "outcome": ...}]} for one binary."""
    report, _ = run_binary(binary)
    test_ids = {}
    for test_case in report.get("test-run", {}).get("test-cases", []):
        info = test_case["test-info"]
        outcome = outcome_for(test_case)
        nodeid = f"{binary.name}::{info['name']}"
        for tag in info.get("tags", []):
            if TC_TAG_RE.match(tag):
                test_ids.setdefault(tag, []).append({"nodeid": nodeid, "outcome": outcome})
    return test_ids


def merge(dst, src):
    for test_id, entries in src.items():
        dst.setdefault(test_id, []).extend(entries)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binaries", nargs="*", type=pathlib.Path, help="Catch2 test binaries to collect from")
    parser.add_argument(
        "--out", type=pathlib.Path, default=pathlib.Path("traceability-report.json"), help="Report output path"
    )
    args = parser.parse_args()

    repo_root = pathlib.Path(__file__).resolve().parent.parent
    binaries = args.binaries or [repo_root / "tests" / "fmu_test", repo_root / "tests" / "mav_io_test"]

    test_ids = {}
    any_failed = False
    for binary in binaries:
        if not binary.exists():
            print(f"error: {binary} does not exist (build it with `make check` first)", file=sys.stderr)
            return 1
        collected = collect_from_binary(binary)
        merge(test_ids, collected)
        if any(entry["outcome"] == "failed" for entries in collected.values() for entry in entries):
            any_failed = True

    test_ids = {test_id: entries for test_id, entries in sorted(test_ids.items())}
    payload = {
        "generated": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "test_ids": test_ids,
    }
    args.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print(f"traceability: {len(test_ids)} TC IDs covered, report written to {args.out}")
    return 1 if any_failed else 0


if __name__ == "__main__":
    sys.exit(main())
