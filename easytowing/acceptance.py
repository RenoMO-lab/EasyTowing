"""Data-driven acceptance checks for approved Monroc reference cases.

The repository does not contain Monroc limits or customer geometry. This
module therefore requires every criterion explicitly and never invents a
default acceptance threshold.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


ACCEPTANCE_EVALUATOR_ID = "easytowing.monroc_acceptance.v1"


@dataclass(frozen=True, slots=True)
class MonrocAcceptanceCriteria:
    """Approved limits for one representative design case."""

    case_id: str
    minimum_clearance_mm: float
    maximum_wheel_error_deg: float
    maximum_synchronization_error_deg: float
    maximum_mechanism_residual_mm: float = 0.01
    require_full_range: bool = True

    def __post_init__(self) -> None:
        if not self.case_id.strip():
            raise ValueError("Acceptance case_id must not be empty.")
        for name in (
            "minimum_clearance_mm",
            "maximum_wheel_error_deg",
            "maximum_synchronization_error_deg",
            "maximum_mechanism_residual_mm",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"Acceptance criterion {name} must be finite and non-negative.")

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "MonrocAcceptanceCriteria":
        required = (
            "case_id",
            "minimum_clearance_mm",
            "maximum_wheel_error_deg",
            "maximum_synchronization_error_deg",
        )
        missing = [name for name in required if name not in raw]
        if missing:
            raise ValueError(f"Acceptance criteria missing required fields: {', '.join(missing)}.")
        case_id = raw["case_id"]
        if not isinstance(case_id, str):
            raise ValueError("Acceptance criteria case_id must be a string.")
        require_full_range = raw.get("require_full_range", True)
        if not isinstance(require_full_range, bool):
            raise ValueError("Acceptance criterion require_full_range must be a JSON boolean.")
        return cls(
            case_id=case_id,
            minimum_clearance_mm=float(raw["minimum_clearance_mm"]),
            maximum_wheel_error_deg=float(raw["maximum_wheel_error_deg"]),
            maximum_synchronization_error_deg=float(raw["maximum_synchronization_error_deg"]),
            maximum_mechanism_residual_mm=float(raw.get("maximum_mechanism_residual_mm", 0.01)),
            require_full_range=require_full_range,
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "case_id": self.case_id,
            "minimum_clearance_mm": self.minimum_clearance_mm,
            "maximum_wheel_error_deg": self.maximum_wheel_error_deg,
            "maximum_synchronization_error_deg": self.maximum_synchronization_error_deg,
            "maximum_mechanism_residual_mm": self.maximum_mechanism_residual_mm,
            "require_full_range": self.require_full_range,
        }


def _finite_number(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _minimum(values: list[float]) -> float | None:
    return min(values) if values else None


def _maximum(values: list[float]) -> float | None:
    return max(values) if values else None


def _metric(snapshot: Mapping[str, Any], key: str) -> float | None:
    metrics = snapshot.get("metrics")
    if not isinstance(metrics, Mapping):
        return None
    return _finite_number(metrics.get(key))


def _mechanism_residual(snapshot: Mapping[str, Any]) -> float | None:
    graph = snapshot.get("mechanism_graph")
    if not isinstance(graph, Mapping):
        return None
    state = graph.get("state")
    if not isinstance(state, Mapping):
        return None
    return _finite_number(state.get("maximum_residual_mm"))


def _clearance(snapshot: Mapping[str, Any]) -> float | None:
    clearance = snapshot.get("clearance")
    if not isinstance(clearance, Mapping):
        return None
    return _finite_number(clearance.get("minimum_clearance_mm"))


def _sweep_values(
    snapshot: Mapping[str, Any],
    key: str,
) -> list[float]:
    sweep = snapshot.get("sweep_validation")
    if not isinstance(sweep, Mapping):
        return []
    values: list[float] = []
    summary_value = _finite_number(sweep.get(key))
    if summary_value is not None:
        values.append(summary_value)
    samples = sweep.get("samples")
    if isinstance(samples, list):
        for sample in samples:
            if isinstance(sample, Mapping):
                value = _finite_number(sample.get(key))
                if value is not None:
                    values.append(value)
    return values


def _sweep_clearances(snapshot: Mapping[str, Any]) -> list[float]:
    sweep = snapshot.get("sweep_validation")
    if not isinstance(sweep, Mapping):
        return []
    values: list[float] = []
    summary_value = _finite_number(sweep.get("minimum_clearance_mm"))
    if summary_value is not None:
        values.append(summary_value)
    samples = sweep.get("samples")
    if isinstance(samples, list):
        for sample in samples:
            if isinstance(sample, Mapping):
                value = _finite_number(sample.get("minimum_clearance_mm"))
                if value is not None:
                    values.append(value)
    return values


def _has_sweep_collision(snapshot: Mapping[str, Any]) -> bool | None:
    sweep = snapshot.get("sweep_validation")
    if not isinstance(sweep, Mapping):
        return None
    values: list[bool] = []
    summary = sweep.get("collision_detected")
    if isinstance(summary, bool):
        values.append(summary)
    samples = sweep.get("samples")
    if isinstance(samples, list):
        for sample in samples:
            if isinstance(sample, Mapping) and isinstance(sample.get("collision_detected"), bool):
                values.append(bool(sample["collision_detected"]))
    return any(values) if values else None


def _check(
    check_id: str,
    label: str,
    actual: object,
    limit: object,
    *,
    passed: bool,
    detail: str,
) -> dict[str, object]:
    return {
        "id": check_id,
        "label": label,
        "status": "PASS" if passed else "FAIL",
        "actual": actual,
        "limit": limit,
        "detail": detail,
    }


def evaluate_monroc_acceptance(
    snapshot: Mapping[str, Any] | None,
    criteria: MonrocAcceptanceCriteria | Mapping[str, Any] | None,
) -> dict[str, object]:
    """Evaluate one saved engineering snapshot against explicit limits."""

    if criteria is None:
        return {
            "status": "NOT_CONFIGURED",
            "configured": False,
            "checks": [],
            "message": "Monroc acceptance criteria have not been configured.",
        }
    normalized = (
        criteria
        if isinstance(criteria, MonrocAcceptanceCriteria)
        else MonrocAcceptanceCriteria.from_dict(criteria)
    )
    if not isinstance(snapshot, Mapping):
        return {
            "status": "FAIL",
            "configured": True,
            "case_id": normalized.case_id,
            "criteria": normalized.to_dict(),
            "checks": [_check(
                "EVIDENCE_MISSING",
                "Engineering evidence",
                None,
                "saved snapshot",
                passed=False,
                detail="No engineering snapshot was supplied.",
            )],
        }

    sweep = snapshot.get("sweep_validation")
    sweep_mapping = sweep if isinstance(sweep, Mapping) else None
    wheel_errors = [value for value in [_metric(snapshot, "max_abs_wheel_error_deg")] if value is not None]
    wheel_errors.extend(_sweep_values(snapshot, "max_abs_wheel_error_deg"))
    sync_errors = [value for value in [_metric(snapshot, "max_abs_synchronization_error_deg")] if value is not None]
    sync_errors.extend(_sweep_values(snapshot, "max_abs_synchronization_error_deg"))
    clearances = [value for value in [_clearance(snapshot)] if value is not None]
    clearances.extend(_sweep_clearances(snapshot))
    residuals = [value for value in [_mechanism_residual(snapshot)] if value is not None]
    residuals.extend(_sweep_values(snapshot, "maximum_mechanism_residual_mm"))

    checks: list[dict[str, object]] = []
    collision = (snapshot.get("clearance") or {}).get("collision_detected") if isinstance(snapshot.get("clearance"), Mapping) else None
    sweep_collision = _has_sweep_collision(snapshot)
    collision_values = [value for value in (collision, sweep_collision) if isinstance(value, bool)]
    collision_failed = any(collision_values) if collision_values else None
    checks.append(_check(
        "COLLISION_FREE",
        "Collision-free envelope",
        None if collision_failed is None else not collision_failed,
        True,
        passed=collision_failed is False,
        detail=(
            "No collision reported in the supplied evidence."
            if collision_failed is False
            else "Collision evidence is missing or reports an overlap."
        ),
    ))

    minimum_clearance = _minimum(clearances)
    checks.append(_check(
        "MINIMUM_CLEARANCE",
        "Minimum clearance",
        minimum_clearance,
        normalized.minimum_clearance_mm,
        passed=minimum_clearance is not None and minimum_clearance >= normalized.minimum_clearance_mm,
        detail=(
            f"{minimum_clearance:.3f} mm available; {normalized.minimum_clearance_mm:.3f} mm required."
            if minimum_clearance is not None
            else "Clearance evidence is missing."
        ),
    ))

    maximum_wheel_error = _maximum(wheel_errors)
    checks.append(_check(
        "STEERING_ACCURACY",
        "Maximum wheel steering error",
        maximum_wheel_error,
        normalized.maximum_wheel_error_deg,
        passed=maximum_wheel_error is not None and maximum_wheel_error <= normalized.maximum_wheel_error_deg,
        detail=(
            f"{maximum_wheel_error:.3f} deg error; {normalized.maximum_wheel_error_deg:.3f} deg maximum."
            if maximum_wheel_error is not None
            else "Wheel steering error evidence is missing."
        ),
    ))

    maximum_sync_error = _maximum(sync_errors)
    checks.append(_check(
        "SYNCHRONIZATION",
        "Maximum synchronization error",
        maximum_sync_error,
        normalized.maximum_synchronization_error_deg,
        passed=maximum_sync_error is not None and maximum_sync_error <= normalized.maximum_synchronization_error_deg,
        detail=(
            f"{maximum_sync_error:.3f} deg error; {normalized.maximum_synchronization_error_deg:.3f} deg maximum."
            if maximum_sync_error is not None
            else "Synchronization error evidence is missing."
        ),
    ))

    maximum_residual = _maximum(residuals)
    checks.append(_check(
        "MECHANISM_RESIDUAL",
        "Maximum mechanism residual",
        maximum_residual,
        normalized.maximum_mechanism_residual_mm,
        passed=maximum_residual is not None and maximum_residual <= normalized.maximum_mechanism_residual_mm,
        detail=(
            f"{maximum_residual:.6f} mm residual; {normalized.maximum_mechanism_residual_mm:.6f} mm maximum."
            if maximum_residual is not None
            else "Mechanism residual evidence is missing."
        ),
    ))

    if normalized.require_full_range:
        sweep_pass = (
            sweep_mapping is not None
            and sweep_mapping.get("status") == "PASS"
            and sweep_mapping.get("sampling_complete") is True
            and _finite_number(sweep_mapping.get("sample_count")) is not None
            and _finite_number(sweep_mapping.get("solved_sample_count"))
            == _finite_number(sweep_mapping.get("sample_count"))
        )
        checks.append(_check(
            "FULL_RANGE",
            "Full articulation range",
            None if sweep_mapping is None else sweep_mapping.get("status"),
            "PASS",
            passed=sweep_pass,
            detail=(
                "Every saved articulation sample passed the engineering sweep."
                if sweep_pass
                else "A complete passing full-range sweep is required."
            ),
        ))

    passed = all(check["status"] == "PASS" for check in checks)
    return {
        "status": "PASS" if passed else "FAIL",
        "configured": True,
        "case_id": normalized.case_id,
        "criteria": normalized.to_dict(),
        "checks": checks,
    }
