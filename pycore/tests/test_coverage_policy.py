import pytest

from tools.check_coverage import (
    CoveragePolicyError,
    enforce_critical_coverage,
    summarize_critical_groups,
)


def _entry(covered_lines: int, num_statements: int):
    return {
        "summary": {
            "covered_lines": covered_lines,
            "num_statements": num_statements,
        }
    }


def _passing_report():
    return {
        "files": {
            "pycore/src/runtime/config/loader.py": _entry(9, 10),
            "pycore/src/domain/safety/validator.py": _entry(18, 20),
            "pycore/src/runtime/executor.py": _entry(10, 10),
            "pycore/src/adapters/mock.py": _entry(19, 20),
            "pycore/src/domain/models.py": _entry(0, 100),
        }
    }


def test_critical_coverage_groups_are_mutually_exclusive() -> None:
    summaries = summarize_critical_groups(_passing_report())

    assert summaries["runtime/config"].covered_statements == 9
    assert summaries["runtime/config"].total_statements == 10
    assert summaries["domain/safety"].percent == 90.0
    assert summaries["runtime"].percent == 100.0
    assert summaries["adapters"].percent == 95.0
    enforce_critical_coverage(_passing_report())


def test_critical_coverage_rejects_low_or_missing_groups() -> None:
    low_report = _passing_report()
    low_report["files"]["pycore/src/adapters/mock.py"] = _entry(17, 20)
    with pytest.raises(CoveragePolicyError, match=r"adapters: 85.00%"):
        enforce_critical_coverage(low_report)

    with pytest.raises(CoveragePolicyError, match="no measured source files"):
        enforce_critical_coverage({"files": {}})


@pytest.mark.parametrize(
    "report",
    [
        {},
        {"files": []},
        {"files": {"pycore/src/runtime/executor.py": []}},
        {"files": {"pycore/src/runtime/executor.py": {}}},
        {
            "files": {
                "pycore/src/runtime/executor.py": {
                    "summary": {"covered_lines": 11, "num_statements": 10}
                }
            }
        },
    ],
)
def test_critical_coverage_rejects_malformed_reports(report) -> None:
    with pytest.raises(CoveragePolicyError):
        summarize_critical_groups(report)
