#!/usr/bin/env python3
"""Enforce aggregate coverage for safety-critical core package groups."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, NoReturn, Optional

CRITICAL_COVERAGE_PERCENT = 90.0
GROUP_NAMES = ("runtime/config", "domain/safety", "runtime", "adapters")


class CoveragePolicyError(RuntimeError):
    """Raised when a critical coverage group is absent or below threshold."""


@dataclass(frozen=True)
class GroupCoverage:
    """Statement coverage aggregated without overlapping package groups."""

    covered_statements: int
    total_statements: int

    @property
    def percent(self) -> float:
        """Return statement coverage percentage."""
        if self.total_statements == 0:
            return 0.0
        return 100.0 * self.covered_statements / self.total_statements


def _fail(message: str) -> NoReturn:
    raise CoveragePolicyError(message)


def _source_relative_path(filename: str) -> Optional[str]:
    normalized = filename.replace("\\", "/")
    marker = "pycore/src/"
    marker_index = normalized.find(marker)
    if marker_index < 0:
        return None
    return normalized[marker_index + len(marker) :]


def _critical_group(filename: str) -> Optional[str]:
    relative_path = _source_relative_path(filename)
    if relative_path is None:
        return None
    if relative_path.startswith("runtime/config/"):
        return "runtime/config"
    if relative_path.startswith("domain/safety/"):
        return "domain/safety"
    if relative_path.startswith("runtime/"):
        return "runtime"
    if relative_path.startswith("adapters/"):
        return "adapters"
    return None


def summarize_critical_groups(
    report: Mapping[str, object],
) -> Dict[str, GroupCoverage]:
    """Aggregate coverage JSON files into mutually exclusive critical groups."""
    files = report.get("files")
    if not isinstance(files, dict):
        _fail("Coverage JSON must contain a files mapping.")

    counts = {name: [0, 0] for name in GROUP_NAMES}
    for filename, details in files.items():
        group_name = _critical_group(str(filename))
        if group_name is None:
            continue
        if not isinstance(details, dict):
            _fail(f"Coverage entry must be a mapping: {filename}")
        summary = details.get("summary")
        if not isinstance(summary, dict):
            _fail(f"Coverage entry has no summary mapping: {filename}")
        covered = summary.get("covered_lines")
        statements = summary.get("num_statements")
        if not isinstance(covered, int) or not isinstance(statements, int):
            _fail(f"Coverage counts must be integers: {filename}")
        if covered < 0 or statements < 0 or covered > statements:
            _fail(f"Coverage counts are invalid: {filename}")
        counts[group_name][0] += covered
        counts[group_name][1] += statements

    return {
        name: GroupCoverage(
            covered_statements=counts[name][0],
            total_statements=counts[name][1],
        )
        for name in GROUP_NAMES
    }


def enforce_critical_coverage(
    report: Mapping[str, object],
    minimum_percent: float = CRITICAL_COVERAGE_PERCENT,
) -> Dict[str, GroupCoverage]:
    """Return group summaries or raise when any group misses the contract."""
    summaries = summarize_critical_groups(report)
    failures = []
    for name in GROUP_NAMES:
        summary = summaries[name]
        if summary.total_statements == 0:
            failures.append(f"{name}: no measured source files")
        elif summary.percent + 1.0e-12 < minimum_percent:
            failures.append(
                f"{name}: {summary.percent:.2f}% is below {minimum_percent:.2f}%"
            )
    if failures:
        _fail("Critical coverage policy failed: " + "; ".join(failures))
    return summaries


def _load_report(path: Path) -> Mapping[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        _fail(f"Could not read coverage JSON {path}: {error}")
    if not isinstance(payload, dict):
        _fail(f"Coverage JSON root must be a mapping: {path}")
    return payload


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("coverage_json", type=Path)
    parser.add_argument(
        "--minimum",
        type=float,
        default=CRITICAL_COVERAGE_PERCENT,
        help="Minimum statement coverage percentage for every critical group.",
    )
    return parser.parse_args()


def main() -> int:
    """Validate one coverage JSON report and print exact group evidence."""
    arguments = _arguments()
    summaries = enforce_critical_coverage(
        _load_report(arguments.coverage_json),
        minimum_percent=arguments.minimum,
    )
    for name in GROUP_NAMES:
        summary = summaries[name]
        print(
            f"PASS critical coverage {name}: "
            f"{summary.covered_statements}/{summary.total_statements} "
            f"({summary.percent:.2f}%)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
