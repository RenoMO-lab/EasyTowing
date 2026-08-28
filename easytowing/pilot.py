"""Executable validation for supplied Monroc pilot case packages.

The repository intentionally contains no customer CAD or approved values. A
pilot package therefore supplies those inputs as hashed external artifacts and
must include both an independent hand calculation and an approved reference
comparison. This module validates the package without treating a pilot PASS
as manufacturing-release authorization.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
from pathlib import Path
from typing import Any, Mapping

from .acceptance import MonrocAcceptanceCriteria, evaluate_monroc_acceptance


PILOT_SCHEMA_VERSION = 1
PILOT_COMPARISON_IDS = ("hand_calculation", "approved_reference")
PILOT_METRIC_KEYS = (
    "minimum_clearance_mm",
    "max_abs_wheel_error_deg",
    "max_abs_synchronization_error_deg",
    "maximum_mechanism_residual_mm",
)


def _finite_number(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be an object.")
    return value


def _artifact_path(record: Mapping[str, Any], base_dir: Path, label: str) -> tuple[Path, dict[str, object]]:
    raw_path = record.get("path")
    expected_sha256 = record.get("sha256")
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise ValueError(f"{label}.path must be a non-empty string.")
    if not isinstance(expected_sha256, str) or len(expected_sha256) != 64:
        raise ValueError(f"{label}.sha256 must be a 64-character hexadecimal digest.")
    try:
        bytes.fromhex(expected_sha256)
    except ValueError as error:
        raise ValueError(f"{label}.sha256 must be a 64-character hexadecimal digest.") from error
    path = Path(raw_path)
    if not path.is_absolute():
        path = base_dir / path
    try:
        resolved = path.resolve(strict=True)
        content = resolved.read_bytes()
    except OSError as error:
        raise ValueError(f"{label} could not be read: {error}") from error
    actual_sha256 = hashlib.sha256(content).hexdigest()
    if not hmac.compare_digest(actual_sha256, expected_sha256.lower()):
        raise ValueError(f"{label} SHA-256 does not match the manifest.")
    return resolved, {
        "path": str(resolved),
        "sha256": actual_sha256,
        "byte_size": len(content),
    }


def _json_artifact(record: Mapping[str, Any], base_dir: Path, label: str) -> tuple[Mapping[str, Any], dict[str, object]]:
    path, metadata = _artifact_path(record, base_dir, label)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} must contain valid UTF-8 JSON: {error}") from error
    return _mapping(payload, f"{label} JSON"), metadata


def _snapshot_from_payload(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    nested = payload.get("snapshot")
    return _mapping(nested, "engineering snapshot") if isinstance(nested, Mapping) else payload


def _snapshot_metrics(snapshot: Mapping[str, Any]) -> dict[str, float | None]:
    values: dict[str, list[float]] = {key: [] for key in PILOT_METRIC_KEYS}

    def add(key: str, value: object) -> None:
        number = _finite_number(value)
        if number is not None:
            values[key].append(number)

    metrics = snapshot.get("metrics")
    if isinstance(metrics, Mapping):
        add("max_abs_wheel_error_deg", metrics.get("max_abs_wheel_error_deg"))
        add("max_abs_synchronization_error_deg", metrics.get("max_abs_synchronization_error_deg"))
    mechanism_graph = snapshot.get("mechanism_graph")
    if isinstance(mechanism_graph, Mapping):
        graph_state = mechanism_graph.get("state")
        if isinstance(graph_state, Mapping):
            add("maximum_mechanism_residual_mm", graph_state.get("maximum_residual_mm"))
    clearance = snapshot.get("clearance")
    if isinstance(clearance, Mapping):
        add("minimum_clearance_mm", clearance.get("minimum_clearance_mm"))

    sweep = snapshot.get("sweep_validation")
    if isinstance(sweep, Mapping):
        add("minimum_clearance_mm", sweep.get("minimum_clearance_mm"))
        add("max_abs_wheel_error_deg", sweep.get("max_abs_wheel_error_deg"))
        add("max_abs_synchronization_error_deg", sweep.get("max_abs_synchronization_error_deg"))
        add("maximum_mechanism_residual_mm", sweep.get("maximum_mechanism_residual_mm"))
        samples = sweep.get("samples")
        if isinstance(samples, list):
            for sample in samples:
                if not isinstance(sample, Mapping):
                    continue
                add("minimum_clearance_mm", sample.get("minimum_clearance_mm"))
                add("max_abs_wheel_error_deg", sample.get("max_abs_wheel_error_deg"))
                add("max_abs_synchronization_error_deg", sample.get("max_abs_synchronization_error_deg"))
                add("maximum_mechanism_residual_mm", sample.get("maximum_mechanism_residual_mm"))

    return {
        "minimum_clearance_mm": min(values["minimum_clearance_mm"], default=None),
        "max_abs_wheel_error_deg": max(values["max_abs_wheel_error_deg"], default=None),
        "max_abs_synchronization_error_deg": max(values["max_abs_synchronization_error_deg"], default=None),
        "maximum_mechanism_residual_mm": max(values["maximum_mechanism_residual_mm"], default=None),
    }


def _comparison_checks(
    actual_metrics: Mapping[str, float | None],
    evidence: Mapping[str, Any],
    label: str,
) -> list[dict[str, object]]:
    expected = _mapping(evidence.get("metrics"), f"{label}.metrics")
    tolerances = _mapping(evidence.get("tolerances"), f"{label}.tolerances")
    checks: list[dict[str, object]] = []
    for key in PILOT_METRIC_KEYS:
        expected_value = _finite_number(expected.get(key))
        tolerance = _finite_number(tolerances.get(key))
        if expected_value is None or tolerance is None or tolerance < 0.0:
            raise ValueError(f"{label} must provide finite non-negative tolerance and expected value for {key}.")
        actual_value = actual_metrics.get(key)
        delta = None if actual_value is None else abs(actual_value - expected_value)
        passed = delta is not None and delta <= tolerance
        checks.append({
            "id": key,
            "status": "PASS" if passed else "FAIL",
            "actual": actual_value,
            "expected": expected_value,
            "tolerance": tolerance,
            "detail": (
                f"{actual_value:.6f} versus {expected_value:.6f}; "
                f"difference {delta:.6f} is within {tolerance:.6f}."
                if passed
                else "The engineering result is missing or outside the supplied comparison tolerance."
            ),
        })
    return checks


def _validate_pilot_case(manifest: Mapping[str, Any], base_dir: Path) -> dict[str, object]:
    if manifest.get("schema_version") != PILOT_SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {PILOT_SCHEMA_VERSION}.")
    case_id = manifest.get("case_id")
    if not isinstance(case_id, str) or not case_id.strip():
        raise ValueError("case_id must be a non-empty string.")
    criteria = MonrocAcceptanceCriteria.from_dict(_mapping(manifest.get("criteria"), "criteria"))
    if criteria.case_id != case_id:
        raise ValueError("case_id must match criteria.case_id.")

    cad_source = _mapping(manifest.get("cad_source"), "cad_source")
    cad_revision = cad_source.get("revision")
    if not isinstance(cad_revision, str) or not cad_revision.strip():
        raise ValueError("cad_source.revision must be a non-empty string.")
    _, cad_metadata = _artifact_path(cad_source, base_dir, "cad_source")
    cad_metadata["revision"] = cad_revision

    snapshot_payload, snapshot_metadata = _json_artifact(
        _mapping(manifest.get("engineering_snapshot"), "engineering_snapshot"),
        base_dir,
        "engineering_snapshot",
    )
    snapshot = _snapshot_from_payload(snapshot_payload)
    actual_metrics = _snapshot_metrics(snapshot)
    acceptance = evaluate_monroc_acceptance(snapshot, criteria)

    raw_comparisons = manifest.get("comparisons")
    if not isinstance(raw_comparisons, list):
        raise ValueError("comparisons must be an array containing hand_calculation and approved_reference.")
    comparisons_by_id: dict[str, Mapping[str, Any]] = {}
    for raw_comparison in raw_comparisons:
        comparison = _mapping(raw_comparison, "comparison")
        comparison_id = comparison.get("id")
        if not isinstance(comparison_id, str) or not comparison_id.strip():
            raise ValueError("Every comparison must have a non-empty id.")
        if comparison_id in comparisons_by_id:
            raise ValueError(f"Comparison id {comparison_id!r} is duplicated.")
        comparisons_by_id[comparison_id] = comparison
    missing = [comparison_id for comparison_id in PILOT_COMPARISON_IDS if comparison_id not in comparisons_by_id]
    if missing:
        raise ValueError(f"Pilot comparisons missing required evidence: {', '.join(missing)}.")

    comparison_results: list[dict[str, object]] = []
    comparison_pass = True
    for comparison_id, comparison in comparisons_by_id.items():
        evidence, metadata = _json_artifact(comparison, base_dir, comparison_id)
        checks = _comparison_checks(actual_metrics, evidence, comparison_id)
        passed = all(check["status"] == "PASS" for check in checks)
        comparison_pass = comparison_pass and passed
        comparison_results.append({
            "id": comparison_id,
            "status": "PASS" if passed else "FAIL",
            "artifact": metadata,
            "checks": checks,
        })

    acceptance_pass = acceptance.get("status") == "PASS"
    status = "PASS" if acceptance_pass and comparison_pass else "FAIL"
    return {
        "schema_version": PILOT_SCHEMA_VERSION,
        "case_id": case_id,
        "status": status,
        "release_authority": "none",
        "message": (
            "Pilot validation passed against the supplied criteria and comparison evidence; "
            "this result does not authorize manufacturing release."
            if status == "PASS"
            else "Pilot validation failed; retain the case as diagnostic evidence and investigate the failed checks."
        ),
        "cad_source": cad_metadata,
        "engineering_snapshot": snapshot_metadata,
        "actual_metrics": actual_metrics,
        "acceptance": acceptance,
        "comparisons": comparison_results,
    }


def validate_pilot_case(
    manifest: Mapping[str, Any] | None,
    *,
    base_dir: Path | None = None,
) -> dict[str, object]:
    """Validate one frozen pilot package and fail closed on package errors."""

    case_id = manifest.get("case_id") if isinstance(manifest, Mapping) else None
    try:
        normalized = _mapping(manifest, "pilot manifest")
        return _validate_pilot_case(normalized, (base_dir or Path.cwd()).resolve())
    except (OSError, TypeError, ValueError) as error:
        return {
            "schema_version": PILOT_SCHEMA_VERSION,
            "case_id": case_id,
            "status": "INVALID_PACKAGE",
            "release_authority": "none",
            "message": str(error),
            "comparisons": [],
        }


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Validate a frozen EasyTowing Monroc pilot case package.")
    parser.add_argument("manifest", type=Path, help="Path to the pilot case JSON manifest.")
    args = parser.parse_args()
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        print(json.dumps({"status": "INVALID_PACKAGE", "message": str(error)}, indent=2, sort_keys=True))
        return 1
    result = validate_pilot_case(manifest, base_dir=args.manifest.parent)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
