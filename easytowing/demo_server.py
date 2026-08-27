from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import json
import math
from urllib.parse import parse_qs, urlparse

from .collision import (
    CapsuleEnvelope,
    CircleEnvelope,
    CollisionItem,
    ClearancePair,
    ClearanceReport,
    PolygonEnvelope,
    analyze_clearance,
)
from .geometry import Point2D
from .design_cases import DesignCase
from .errors import ArticulationLimitExceededError, EngineeringError
from .dxf_import import analyze_dxf_import, apply_dxf_role_overrides, serialize_dxf_import_report
from .linkage import (
    LinkageDemoRig,
    PlanarLinkageSpec,
    build_linkage_rig,
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage,
)
from .model import (
    Axle,
    SteeringSynchronization,
    SteeringTargetPoint,
    VehicleLayout,
    build_reference_demo_combination,
    serialize_vehicle_combination,
)
from .actual_steering import (
    compare_actual_to_ideal,
    serialize_actual_steering,
    solve_actual_steering,
)
from .optimization import (
    OptimizationMetrics,
    OptimizationWeights,
    OptimizedVariable,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from .projects import ProjectStore, serialize_project, serialize_revision
from .reporting import (
    build_dimensioned_svg,
    build_export_bundle,
    build_export_csv,
    build_export_dxf,
    build_export_pdf,
    build_export_png,
    build_steering_curves_svg,
    build_swept_path_svg,
)
from .steering import beta_to_reference_radius_mm, build_demo_solution, solve_ideal_steering_from_radius

WEB_DIR = Path(__file__).resolve().parent / "web"
PROJECT_STORE = ProjectStore.default()


def _point_payload(point: Point2D) -> dict[str, float]:
    return {"x_mm": point.x_mm, "y_mm": point.y_mm}


def _wheel_payload(wheel_solution) -> dict[str, object]:
    return {
        "wheel_id": wheel_solution.wheel_id,
        "axle_id": wheel_solution.axle_id,
        "side": wheel_solution.side,
        "center": _point_payload(wheel_solution.center),
        "heading_rad": wheel_solution.heading_rad,
        "heading_deg": wheel_solution.heading_deg,
        "reference_heading_rad": wheel_solution.reference_heading_rad,
        "reference_heading_deg": wheel_solution.reference_heading_deg,
        "steering_angle_rad": wheel_solution.steering_angle_rad,
        "steering_angle_deg": wheel_solution.steering_angle_deg,
    }


def _axle_payload(axle_solution, axle: Axle | None = None) -> dict[str, object]:
    payload = {
        "axle_id": axle_solution.axle_id,
        "center": _point_payload(axle_solution.center),
        "center_heading_rad": axle_solution.center_heading_rad,
        "center_heading_deg": axle_solution.center_heading_deg,
        "reference_heading_rad": axle_solution.reference_heading_rad,
        "reference_heading_deg": axle_solution.reference_heading_deg,
        "center_steering_angle_rad": axle_solution.center_steering_angle_rad,
        "center_steering_angle_deg": axle_solution.center_steering_angle_deg,
        "left_wheel": _wheel_payload(axle_solution.left_wheel),
        "right_wheel": _wheel_payload(axle_solution.right_wheel),
    }
    if axle is not None:
        payload.update(
            {
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
                "wheel_count": axle.wheel_count,
                "maximum_steering_angle_deg": axle.maximum_steering_angle_deg,
                "steering_stop_deg": axle.steering_stop_deg,
                "load_kg": axle.load_kg,
                "tire_width_mm": axle.tire_width_mm,
                "outside_diameter_mm": axle.outside_diameter_mm,
                "user_defined_steering_angle_deg": math.degrees(axle.user_defined_steering_angle_rad),
            }
        )
    return payload


def _envelope_payload(envelope) -> dict[str, object]:
    if isinstance(envelope, CircleEnvelope):
        return {
            "kind": "circle",
            "center": _point_payload(envelope.center),
            "radius_mm": envelope.radius_mm,
        }
    if isinstance(envelope, CapsuleEnvelope):
        return {
            "kind": "capsule",
            "start": _point_payload(envelope.start),
            "end": _point_payload(envelope.end),
            "radius_mm": envelope.radius_mm,
        }
    if isinstance(envelope, PolygonEnvelope):
        return {
            "kind": "polygon",
            "points": [_point_payload(point) for point in envelope.points],
        }
    raise TypeError(f"Unsupported envelope type: {type(envelope)!r}")


def _collision_item_payload(item: CollisionItem) -> dict[str, object]:
    return {
        "id": item.id,
        "margin_mm": item.margin_mm,
        "excluded_pair_ids": list(item.excluded_pair_ids),
        "envelope": _envelope_payload(item.envelope),
    }


def _clearance_pair_payload(pair: ClearancePair) -> dict[str, object]:
    return {
        "item_a_id": pair.item_a_id,
        "item_b_id": pair.item_b_id,
        "raw_clearance_mm": pair.raw_clearance_mm,
        "required_margin_mm": pair.required_margin_mm,
        "clearance_mm": pair.clearance_mm,
        "overlaps": pair.overlaps,
        "violates_margin": pair.violates_margin,
        "description": pair.description,
    }


def _clearance_report_payload(report: ClearanceReport) -> dict[str, object]:
    return {
        "minimum_clearance_mm": report.minimum_clearance_mm,
        "collision_detected": report.collision_detected,
        "clearance_violation_detected": report.clearance_violation_detected,
        "items": [_collision_item_payload(item) for item in report.items],
        "pairs": [_clearance_pair_payload(pair) for pair in report.pairs],
        "minimum_pair": None if report.minimum_pair is None else _clearance_pair_payload(report.minimum_pair),
    }


def _optimization_metrics_payload(metrics: OptimizationMetrics) -> dict[str, object]:
    return {
        "score": metrics.score,
        "rms_error_deg": metrics.rms_error_deg,
        "mean_abs_error_deg": metrics.mean_abs_error_deg,
        "max_abs_error_deg": metrics.max_abs_error_deg,
        "minimum_clearance_mm": metrics.minimum_clearance_mm,
        "minimum_clearance_beta_deg": metrics.minimum_clearance_beta_deg,
        "failure_index": metrics.failure_index,
        "solved_samples": metrics.solved_samples,
        "sample_count": metrics.sample_count,
        "max_abs_inner_error_deg": metrics.max_abs_inner_error_deg,
        "max_abs_outer_error_deg": metrics.max_abs_outer_error_deg,
        "max_abs_synchronization_error_deg": metrics.max_abs_synchronization_error_deg,
    }


def _optimized_variable_payload(variable: OptimizedVariable) -> dict[str, object]:
    return {
        "id": variable.id,
        "current": variable.current,
        "minimum": variable.minimum,
        "maximum": variable.maximum,
        "enabled": variable.enabled,
        "preferred": variable.preferred,
        "optimized": variable.optimized,
        "delta": variable.delta,
    }


def _optimization_payload(
    mode: str,
    enabled_ids: set[str] | None = None,
    clearance_target_mm: float = 20.0,
    weights: OptimizationWeights | None = None,
    design_cases: tuple[DesignCase, ...] | None = None,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> dict[str, object]:
    problem = build_reference_optimization_problem(
        mode=mode,
        enabled_ids=enabled_ids,
        clearance_target_mm=clearance_target_mm,
        weights=weights,
        design_cases=design_cases,
        base_rig=linkage_rig,
        vehicle=vehicle,
    )
    result = optimize_linkage_problem(problem)
    return {
        "mode": result.mode,
        "iterations": result.iterations,
        "evaluations": result.evaluations,
        "improved": result.improved,
        "improvement": result.improvement,
        "baseline": _optimization_metrics_payload(result.baseline_metrics),
        "optimized": _optimization_metrics_payload(result.optimized_metrics),
        "objective": {
            "clearance_target_mm": result.clearance_target_mm,
            "weights": result.weights.to_dict(),
        },
        "design_cases": [case.to_dict() for case in result.design_cases],
        "variables_before": [_optimized_variable_payload(variable) for variable in result.baseline_variables],
        "variables_after": [_optimized_variable_payload(variable) for variable in result.optimized_variables],
    }


def _dxf_import_payload(dxf_text: str, source_name: str = "") -> dict[str, object]:
    report = analyze_dxf_import(dxf_text, source_name=source_name)
    return serialize_dxf_import_report(report)


def _parse_role_overrides(raw_overrides) -> dict[int, str | None]:
    if raw_overrides is None:
        return {}
    if not isinstance(raw_overrides, dict):
        raise ValueError("role_overrides must be an object mapping entity indexes to roles")

    parsed: dict[int, str | None] = {}
    for key, value in raw_overrides.items():
        try:
            index = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError("role_overrides keys must be numeric entity indexes") from exc
        if value is None:
            parsed[index] = None
        elif isinstance(value, str):
            parsed[index] = value
        else:
            raise ValueError("role_overrides values must be strings or null")
    return parsed


def _parse_enabled_ids(raw_enabled_ids) -> set[str] | None:
    if raw_enabled_ids is None:
        return None
    if not isinstance(raw_enabled_ids, list):
        raise ValueError("enabled_ids must be an array of variable IDs")
    return {str(value).strip() for value in raw_enabled_ids if str(value).strip()}


def _parse_design_cases(raw_cases) -> tuple[DesignCase, ...] | None:
    if raw_cases is None:
        return None
    if isinstance(raw_cases, str):
        try:
            raw_cases = json.loads(raw_cases)
        except json.JSONDecodeError as error:
            raise ValueError("design_cases must be valid JSON") from error
    if not isinstance(raw_cases, list):
        raise ValueError("design_cases must be an array")
    try:
        return tuple(DesignCase.from_dict(case) for case in raw_cases)
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid design case: {error}") from error


def _parse_articulation_bounds(body: dict[str, object]) -> tuple[float, float]:
    try:
        beta_min_deg = float(body.get("beta_min_deg", -45.0))
        beta_max_deg = float(body.get("beta_max_deg", 45.0))
    except (TypeError, ValueError) as error:
        raise ValueError("articulation bounds must be numeric") from error
    if (
        not math.isfinite(beta_min_deg)
        or not math.isfinite(beta_max_deg)
        or beta_min_deg >= beta_max_deg
        or beta_min_deg > 0.0
        or beta_max_deg < 0.0
    ):
        raise ValueError("articulation bounds must straddle zero with min below max")
    return beta_min_deg, beta_max_deg


def _query_float(query: dict[str, list[str]], name: str, default: float) -> float:
    raw_value = query.get(name, [str(default)])[0]
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _config_float(
    raw: dict[str, object],
    name: str,
    default: float | None = None,
    *,
    allow_none: bool = False,
) -> float | None:
    if name not in raw:
        return default
    value = raw[name]
    if value is None and allow_none:
        return None
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    try:
        parsed = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _config_point(
    raw: dict[str, object],
    name: str,
    default: Point2D | None,
    *,
    allow_none: bool = False,
) -> Point2D | None:
    if name not in raw:
        return default
    value = raw[name]
    if value is None and allow_none:
        return None
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain x_mm and y_mm")
    try:
        x_mm = _config_float(value, "x_mm")
        y_mm = _config_float(value, "y_mm")
    except ValueError as error:
        raise ValueError(f"{name} must contain finite x_mm and y_mm") from error
    assert x_mm is not None and y_mm is not None
    return Point2D(x_mm, y_mm)


def _config_polygon(
    raw: dict[str, object],
    name: str,
    default: tuple[Point2D, ...] = (),
) -> tuple[Point2D, ...]:
    if name not in raw:
        return default
    value = raw[name]
    if value is None:
        return default
    if not isinstance(value, list):
        raise ValueError(f"{name} must be an array of points")
    points: list[Point2D] = []
    for index, point in enumerate(value):
        if not isinstance(point, dict):
            raise ValueError(f"{name}[{index}] must contain x_mm and y_mm")
        parsed = _config_point({"point": point}, "point", None)
        assert parsed is not None
        points.append(parsed)
    if points and len(points) < 3:
        raise ValueError(f"{name} must contain at least three points")
    return tuple(points)


def _config_angle(raw: dict[str, object], name: str, default_rad: float) -> float:
    degree_name = f"{name}_deg"
    radian_name = f"{name}_rad"
    if degree_name in raw:
        value = _config_float(raw, degree_name)
        assert value is not None
        return math.radians(value)
    if radian_name in raw:
        value = _config_float(raw, radian_name)
        assert value is not None
        return value
    return default_rad


def _positive_config_value(raw: dict[str, object], name: str, default: float) -> float:
    value = _config_float(raw, name, default)
    assert value is not None
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _parse_linkage_rig(raw_config: object) -> LinkageDemoRig:
    reference_rig = build_reference_linkage_demo()
    if raw_config is None:
        return reference_rig
    if not isinstance(raw_config, dict):
        raise ValueError("linkage must be an object")

    reference = reference_rig.spec
    companion_pivot = _config_point(
        raw_config,
        "companion_steering_pivot",
        reference.companion_steering_pivot,
        allow_none=True,
    )
    companion_arm_length = (
        None
        if companion_pivot is None
        else _positive_config_value(
            raw_config,
            "companion_steering_arm_length_mm",
            reference.companion_steering_arm_length_mm or reference.steering_arm_length_mm,
        )
    )
    companion_tie_length = (
        None
        if companion_pivot is None
        else _positive_config_value(
            raw_config,
            "companion_tie_rod_length_mm",
            reference.companion_tie_rod_length_mm or reference.tie_rod_length_mm,
        )
    )
    steering_stop = _config_float(
        raw_config,
        "steering_stop_deg",
        reference.steering_stop_deg,
        allow_none=True,
    )
    if steering_stop is not None and steering_stop < 0.0:
        raise ValueError("steering_stop_deg must not be negative")

    spec = PlanarLinkageSpec(
        id=str(raw_config.get("id", "custom_linkage")).strip() or "custom_linkage",
        steering_pivot=_config_point(raw_config, "steering_pivot", reference.steering_pivot),  # type: ignore[arg-type]
        steering_arm_length_mm=_positive_config_value(
            raw_config,
            "steering_arm_length_mm",
            reference.steering_arm_length_mm,
        ),
        steering_arm_neutral_angle_rad=_config_angle(
            raw_config,
            "steering_arm_neutral_angle",
            reference.steering_arm_neutral_angle_rad,
        ),
        bell_crank_pivot=_config_point(raw_config, "bell_crank_pivot", reference.bell_crank_pivot),  # type: ignore[arg-type]
        bell_crank_input_arm_length_mm=_positive_config_value(
            raw_config,
            "bell_crank_input_arm_length_mm",
            reference.bell_crank_input_arm_length_mm,
        ),
        bell_crank_input_neutral_angle_rad=_config_angle(
            raw_config,
            "bell_crank_input_neutral_angle",
            reference.bell_crank_input_neutral_angle_rad,
        ),
        bell_crank_output_arm_length_mm=_positive_config_value(
            raw_config,
            "bell_crank_output_arm_length_mm",
            reference.bell_crank_output_arm_length_mm,
        ),
        bell_crank_output_neutral_angle_rad=_config_angle(
            raw_config,
            "bell_crank_output_neutral_angle",
            reference.bell_crank_output_neutral_angle_rad,
        ),
        input_rod_length_mm=_positive_config_value(
            raw_config,
            "input_rod_length_mm",
            reference.input_rod_length_mm,
        ),
        tie_rod_length_mm=_positive_config_value(
            raw_config,
            "tie_rod_length_mm",
            reference.tie_rod_length_mm,
        ),
        steering_stop_deg=steering_stop,
        companion_steering_pivot=companion_pivot,
        companion_steering_arm_length_mm=companion_arm_length,
        companion_steering_arm_neutral_angle_rad=_config_angle(
            raw_config,
            "companion_steering_arm_neutral_angle",
            reference.companion_steering_arm_neutral_angle_rad,
        ),
        companion_tie_rod_length_mm=companion_tie_length,
    )
    driver_arc_center = _config_point(
        raw_config,
        "driver_arc_center",
        reference_rig.driver_arc_center,
    )
    driver_arc_radius = _positive_config_value(
        raw_config,
        "driver_arc_radius_mm",
        reference_rig.driver_arc_radius_mm,
    )
    assert driver_arc_center is not None
    return build_linkage_rig(
        spec,
        driver_arc_center=driver_arc_center,
        driver_arc_radius_mm=driver_arc_radius,
        neutral_hint=reference_rig.branch_hint,
    )


def _validated_linkage_config(raw_config: object) -> dict[str, object] | None:
    if raw_config is None:
        return None
    if not isinstance(raw_config, dict):
        raise ValueError("linkage_config must be an object")
    _parse_linkage_rig(raw_config)
    return dict(raw_config)


def _parse_steering_synchronizations(
    raw_syncs: object,
    axle_ids: set[str],
) -> tuple[SteeringSynchronization, ...]:
    if raw_syncs is None:
        return ()
    if not isinstance(raw_syncs, list):
        raise ValueError("steering_synchronizations must be an array")
    synchronizations: list[SteeringSynchronization] = []
    for index, raw_sync in enumerate(raw_syncs):
        if not isinstance(raw_sync, dict):
            raise ValueError(f"steering_synchronizations[{index}] must be an object")
        try:
            target_axle_raw = raw_sync.get("target_axle_id")
            if target_axle_raw is None:
                target_axle_raw = raw_sync["axle_id"]
            target_axle_id = str(target_axle_raw)
            mode = str(raw_sync.get("mode", "SAME_PHASE")).strip().upper()
            phase_offset_rad = float(raw_sync.get("phase_offset_rad", 0.0))
            if "phase_offset_deg" in raw_sync:
                phase_offset_rad = math.radians(float(raw_sync["phase_offset_deg"]))
            target_curve_raw = raw_sync.get("target_curve", [])
            if not isinstance(target_curve_raw, list):
                raise ValueError("target_curve must be an array")
            target_curve: list[SteeringTargetPoint] = []
            for point_index, raw_point in enumerate(target_curve_raw):
                if not isinstance(raw_point, dict):
                    raise ValueError(f"target_curve[{point_index}] must be an object")
                beta_rad = float(raw_point.get("beta_rad", 0.0))
                steering_angle_rad = float(raw_point.get("steering_angle_rad", 0.0))
                if "beta_deg" in raw_point:
                    beta_rad = math.radians(float(raw_point["beta_deg"]))
                if "steering_angle_deg" in raw_point:
                    steering_angle_rad = math.radians(float(raw_point["steering_angle_deg"]))
                target_curve.append(
                    SteeringTargetPoint(
                        beta_rad=beta_rad,
                        steering_angle_rad=steering_angle_rad,
                    )
                )
            synchronization = SteeringSynchronization(
                id=str(raw_sync.get("id", f"sync_{index + 1}")),
                target_axle_id=target_axle_id,
                mode=mode,  # type: ignore[arg-type]
                source_axle_id=(
                    None
                    if raw_sync.get("source_axle_id") in (None, "")
                    else str(raw_sync["source_axle_id"])
                ),
                ratio=float(raw_sync.get("ratio", 1.0)),
                phase_offset_rad=phase_offset_rad,
                target_curve=tuple(target_curve),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"steering_synchronizations[{index}] is invalid") from error
        if synchronization.target_axle_id not in axle_ids:
            raise ValueError(
                f"steering_synchronizations[{index}] targets an unknown axle"
            )
        synchronizations.append(synchronization)
    return tuple(synchronizations)


def _parse_vehicle_layout(body: dict[str, object]) -> VehicleLayout:
    raw_axles = body.get("axles")
    if not isinstance(raw_axles, list) or not raw_axles:
        raise ValueError("axles must be a non-empty array")

    axles: list[Axle] = []
    for index, raw_axle in enumerate(raw_axles):
        if not isinstance(raw_axle, dict):
            raise ValueError(f"axles[{index}] must be an object")
        try:
            axle_id = str(raw_axle["id"])
            center = Point2D(float(raw_axle.get("x_mm", 0.0)), float(raw_axle.get("y_mm", 0.0)))
            track_mm = float(raw_axle["track_mm"])
            wheel_count = int(raw_axle.get("wheel_count", 2))
            heading_rad = float(raw_axle.get("heading_rad", 0.0))
            if "heading_deg" in raw_axle:
                heading_rad = math.radians(float(raw_axle["heading_deg"]))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"axles[{index}] requires id, x_mm, y_mm, and track_mm") from error
        axles.append(
            Axle(
                id=axle_id,
                center=center,
                track_mm=track_mm,
                wheel_count=wheel_count,
                steerable=bool(raw_axle.get("steerable", True)),
                steering_mode=str(raw_axle.get("steering_mode", "FORCED_STEER")),  # type: ignore[arg-type]
                heading_rad=heading_rad,
                maximum_steering_angle_deg=(
                    None
                    if raw_axle.get("maximum_steering_angle_deg") is None
                    else float(raw_axle["maximum_steering_angle_deg"])
                ),
                steering_stop_deg=(
                    None
                    if raw_axle.get("steering_stop_deg") is None
                    else float(raw_axle["steering_stop_deg"])
                ),
                tire_width_mm=float(raw_axle.get("tire_width_mm", 0.0)),
                outside_diameter_mm=float(raw_axle.get("outside_diameter_mm", 0.0)),
                user_defined_steering_angle_rad=math.radians(float(
                    raw_axle.get("user_defined_steering_angle_deg", 0.0)
                )),
                load_kg=(
                    None
                    if raw_axle.get("load_kg") is None
                    else float(raw_axle["load_kg"])
                ),
            )
        )

    try:
        body_length_mm = float(body.get("body_length_mm", 0.0))
        body_width_mm = float(body.get("body_width_mm", 0.0))
    except (TypeError, ValueError) as error:
        raise ValueError("body_length_mm and body_width_mm must be numeric") from error
    if not math.isfinite(body_length_mm) or not math.isfinite(body_width_mm):
        raise ValueError("body_length_mm and body_width_mm must be finite")
    if body_length_mm < 0.0 or body_width_mm < 0.0:
        raise ValueError("body_length_mm and body_width_mm must not be negative")
    body_length_mm = body_length_mm or max(
        max((axle.center.x_mm for axle in axles), default=0.0)
        - min((axle.center.x_mm for axle in axles), default=0.0)
        + 1800.0,
        1800.0,
    )
    body_width_mm = body_width_mm or max((axle.track_mm for axle in axles), default=0.0) + 700.0
    origin = _config_point(body, "origin", Point2D(0.0, 0.0))
    assert origin is not None
    front_articulation_point = _config_point(
        body,
        "front_articulation_point",
        None,
        allow_none=True,
    )
    rear_articulation_point = _config_point(
        body,
        "rear_articulation_point",
        None,
        allow_none=True,
    )
    kingpin_point = _config_point(body, "kingpin_point", None, allow_none=True)
    maximum_articulation_deg = _config_float(body, "maximum_articulation_deg", 45.0)
    assert maximum_articulation_deg is not None
    body_polygon = _config_polygon(body, "body_polygon")
    steering_synchronizations = _parse_steering_synchronizations(
        body.get("steering_synchronizations", body.get("steering_sync")),
        {axle.id for axle in axles},
    )
    return VehicleLayout(
        id=str(body.get("id", "api_vehicle")),
        name=str(body.get("name", "API vehicle")),
        axles=tuple(axles),
        body_length_mm=body_length_mm,
        body_width_mm=body_width_mm,
        origin=origin,
        steering_synchronizations=steering_synchronizations,
        body_polygon=body_polygon,
        front_articulation_point=front_articulation_point,
        rear_articulation_point=rear_articulation_point,
        kingpin_point=kingpin_point,
        maximum_articulation_deg=maximum_articulation_deg,
    )


def _vehicle_config_payload(vehicle: VehicleLayout) -> dict[str, object]:
    def synchronization_payload(item: SteeringSynchronization) -> dict[str, object]:
        return {
            "id": item.id,
            "target_axle_id": item.target_axle_id,
            "source_axle_id": item.source_axle_id,
            "mode": item.mode,
            "ratio": item.ratio,
            "phase_offset_rad": item.phase_offset_rad,
            "phase_offset_deg": math.degrees(item.phase_offset_rad),
            "target_curve": [
                {
                    "beta_rad": point.beta_rad,
                    "beta_deg": math.degrees(point.beta_rad),
                    "steering_angle_rad": point.steering_angle_rad,
                    "steering_angle_deg": math.degrees(point.steering_angle_rad),
                }
                for point in item.target_curve
            ],
        }

    return {
        "id": vehicle.id,
        "name": vehicle.name,
        "body_length_mm": vehicle.body_length_mm,
        "body_width_mm": vehicle.body_width_mm,
        "origin": _point_payload(vehicle.origin),
        "body_polygon": [_point_payload(point) for point in vehicle.body_polygon],
        "front_articulation_point": (
            None if vehicle.front_articulation_point is None else _point_payload(vehicle.front_articulation_point)
        ),
        "rear_articulation_point": (
            None if vehicle.rear_articulation_point is None else _point_payload(vehicle.rear_articulation_point)
        ),
        "kingpin_point": None if vehicle.kingpin_point is None else _point_payload(vehicle.kingpin_point),
        "maximum_articulation_deg": vehicle.maximum_articulation_deg,
        "axles": [
            {
                "id": axle.id,
                "x_mm": axle.center.x_mm,
                "y_mm": axle.center.y_mm,
                "track_mm": axle.track_mm,
                "wheel_count": axle.wheel_count,
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
                "heading_rad": axle.heading_rad,
                "maximum_steering_angle_deg": axle.maximum_steering_angle_deg,
                "steering_stop_deg": axle.steering_stop_deg,
                "load_kg": axle.load_kg,
                "tire_width_mm": axle.tire_width_mm,
                "outside_diameter_mm": axle.outside_diameter_mm,
                "user_defined_steering_angle_deg": math.degrees(axle.user_defined_steering_angle_rad),
            }
            for axle in vehicle.axles
        ],
        "steering_synchronizations": [
            synchronization_payload(item)
            for item in vehicle.steering_synchronizations
        ],
    }


def _parse_vehicle_config(raw_config: object) -> tuple[VehicleLayout, dict[str, object]] | None:
    if raw_config is None:
        return None
    if not isinstance(raw_config, dict):
        raise ValueError("vehicle_config must be an object")
    vehicle = _parse_vehicle_layout(raw_config)
    return vehicle, _vehicle_config_payload(vehicle)


def _project_vehicle_inputs(
    body: dict[str, object],
) -> tuple[float, float, dict[str, object] | None, VehicleLayout | None]:
    wheelbase_mm = _config_float(body, "wheelbase_mm", 4360.0)
    track_mm = _config_float(body, "track_mm", 2500.0)
    assert wheelbase_mm is not None and track_mm is not None
    if wheelbase_mm <= 0.0 or track_mm <= 0.0:
        raise ValueError("wheelbase_mm and track_mm must be positive")

    parsed_vehicle = _parse_vehicle_config(body.get("vehicle_config"))
    if parsed_vehicle is not None:
        vehicle, normalized_config = parsed_vehicle
        return wheelbase_mm, track_mm, normalized_config, vehicle
    if abs(wheelbase_mm - 4360.0) <= 1e-9 and abs(track_mm - 2500.0) <= 1e-9:
        return wheelbase_mm, track_mm, None, None
    return (
        wheelbase_mm,
        track_mm,
        None,
        build_demo_solution(0.0, wheelbase_mm=wheelbase_mm, track_mm=track_mm)[0],
    )


def _ideal_steering_request_payload(body: dict[str, object]) -> dict[str, object]:
    vehicle = _parse_vehicle_layout(body)
    turn_radius = body.get("turn_radius_mm")
    try:
        radius_mm = None if turn_radius is None else float(turn_radius)
    except (TypeError, ValueError) as error:
        raise ValueError("turn_radius_mm must be numeric or null") from error
    solution = solve_ideal_steering_from_radius(vehicle, radius_mm)
    body_length_mm = vehicle.body_length_mm or max(vehicle.axle_span_mm() + 1800.0, 1800.0)
    body_width_mm = vehicle.body_width_mm or max((axle.track_mm for axle in vehicle.axles), default=0.0) + 700.0
    body_outline_points = list(vehicle.body_polygon)
    if not body_outline_points:
        x_min = min((axle.center.x_mm for axle in vehicle.axles), default=0.0) - 900.0
        x_max = max((axle.center.x_mm for axle in vehicle.axles), default=0.0) + 900.0
        half_width = body_width_mm / 2.0
        body_outline_points = [
            Point2D(x_min, -half_width),
            Point2D(x_max, -half_width),
            Point2D(x_max, half_width),
            Point2D(x_min, half_width),
        ]
    body_outline = [_point_payload(point + vehicle.origin) for point in body_outline_points]
    wheel_angles = solution.wheel_steering_angles_deg()
    axle_angles = solution.axle_center_steering_angles_deg()
    serialized_axles = []
    for axle, axle_solution in zip(vehicle.axles, solution.axles, strict=True):
        serialized = _axle_payload(axle_solution, axle)
        serialized.update(
            {
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
                "wheel_count": axle.wheel_count,
                "maximum_steering_angle_deg": axle.maximum_steering_angle_deg,
                "steering_stop_deg": axle.steering_stop_deg,
                "load_kg": axle.load_kg,
                "tire_width_mm": axle.tire_width_mm,
                "outside_diameter_mm": axle.outside_diameter_mm,
                "user_defined_steering_angle_deg": math.degrees(axle.user_defined_steering_angle_rad),
            }
        )
        serialized_axles.append(serialized)
    return {
        "beta_deg": None,
        "vehicle": {
            "id": vehicle.id,
            "name": vehicle.name,
            "axle_count": len(vehicle.axles),
            "body_length_mm": body_length_mm,
            "body_width_mm": body_width_mm,
        },
        "body_outline": body_outline,
        "turn_radius_mm": radius_mm,
        "icr": None if solution.icr is None else _point_payload(solution.icr),
        "vehicle_config": _vehicle_config_payload(vehicle),
        "axles": serialized_axles,
        "wheel_steering_angles_deg": wheel_angles,
        "axle_center_steering_angles_deg": axle_angles,
        "metrics": {
            "max_abs_wheel_angle_deg": max((abs(value) for value in wheel_angles.values()), default=0.0),
            "front_rear_phase_deg": None,
            "linkage_vs_ideal_front_axle_deg": None,
            "minimum_clearance_mm": None,
        },
    }


def _project_summary_payload(project) -> dict[str, object]:
    return serialize_project(project, include_snapshots=False)


def _project_detail_payload(project) -> dict[str, object]:
    return serialize_project(project, include_snapshots=True)


def _revision_payload(revision, include_snapshot: bool = False) -> dict[str, object]:
    return serialize_revision(revision, include_snapshot=include_snapshot)


def _linkage_payload(rig, state) -> dict[str, object]:
    return {
        "driver_point": _point_payload(state.driver_point),
        "driver_arc_center": _point_payload(rig.driver_arc_center),
        "driver_arc_radius_mm": rig.driver_arc_radius_mm,
        "spec": {
            "id": rig.spec.id,
            "bell_crank_pivot": _point_payload(rig.spec.bell_crank_pivot),
            "steering_pivot": _point_payload(rig.spec.steering_pivot),
            "input_rod_length_mm": rig.spec.input_rod_length_mm,
            "tie_rod_length_mm": rig.spec.tie_rod_length_mm,
            "bell_crank_input_arm_length_mm": rig.spec.bell_crank_input_arm_length_mm,
            "bell_crank_output_arm_length_mm": rig.spec.bell_crank_output_arm_length_mm,
            "steering_arm_length_mm": rig.spec.steering_arm_length_mm,
            "steering_stop_deg": rig.spec.steering_stop_deg,
            "companion_steering_pivot": None if rig.spec.companion_steering_pivot is None else _point_payload(rig.spec.companion_steering_pivot),
            "companion_steering_arm_length_mm": rig.spec.companion_steering_arm_length_mm,
            "companion_steering_arm_neutral_angle_rad": rig.spec.companion_steering_arm_neutral_angle_rad,
            "companion_tie_rod_length_mm": rig.spec.companion_tie_rod_length_mm,
        },
        "state": {
            "input_endpoint": _point_payload(state.input_endpoint),
            "output_endpoint": _point_payload(state.output_endpoint),
            "steering_endpoint": _point_payload(state.steering_endpoint),
            "bell_crank_angle_rad": state.bell_crank_angle_rad,
            "bell_crank_angle_deg": state.bell_crank_angle_deg,
            "steering_angle_rad": state.steering_angle_rad,
            "steering_angle_deg": state.steering_angle_deg,
            "input_stage_error_mm": state.input_stage_error_mm,
            "tie_rod_error_mm": state.tie_rod_error_mm,
            "input_branch_index": state.input_branch_index,
            "steering_branch_index": state.steering_branch_index,
            "companion_steering_endpoint": None if state.companion_steering_endpoint is None else _point_payload(state.companion_steering_endpoint),
            "companion_steering_angle_rad": state.companion_steering_angle_rad,
            "companion_steering_angle_deg": state.companion_steering_angle_deg,
            "companion_tie_rod_error_mm": state.companion_tie_rod_error_mm,
            "companion_branch_index": state.companion_branch_index,
        },
    }


def _project_state_payload() -> dict[str, object]:
    projects = PROJECT_STORE.list_projects()
    active_project = next((project for project in projects if project.active_revision_id is not None), None)
    return {
        "projects": [_project_summary_payload(project) for project in projects],
        "active_project_id": None if active_project is None else active_project.id,
        "active_project": None if active_project is None else _project_detail_payload(active_project),
    }


def _clearance_payload(vehicle, rig, state) -> dict[str, object]:
    items = [
        CollisionItem(
            id="input_rod",
            envelope=CapsuleEnvelope(
                start=state.driver_point,
                end=state.input_endpoint,
                radius_mm=14.0,
            ),
        ),
        CollisionItem(
            id="tie_rod",
            envelope=CapsuleEnvelope(
                start=state.output_endpoint,
                end=state.steering_endpoint,
                radius_mm=14.0,
            ),
        ),
        CollisionItem(
            id="steering_arm",
            envelope=CapsuleEnvelope(
                start=rig.spec.steering_pivot,
                end=state.steering_endpoint,
                radius_mm=14.0,
            ),
        ),
        CollisionItem(
            id="bell_crank_pivot",
            envelope=CircleEnvelope(
                center=rig.spec.bell_crank_pivot,
                radius_mm=28.0,
            ),
        ),
        CollisionItem(
            id="steering_pivot",
            envelope=CircleEnvelope(
                center=rig.spec.steering_pivot,
                radius_mm=28.0,
            ),
        ),
    ]
    if state.companion_steering_endpoint is not None and rig.spec.companion_steering_pivot is not None:
        items.extend(
            (
                CollisionItem(
                    id="companion_tie_rod",
                    envelope=CapsuleEnvelope(
                        start=state.steering_endpoint,
                        end=state.companion_steering_endpoint,
                        radius_mm=14.0,
                    ),
                ),
                CollisionItem(
                    id="companion_steering_arm",
                    envelope=CapsuleEnvelope(
                        start=rig.spec.companion_steering_pivot,
                        end=state.companion_steering_endpoint,
                        radius_mm=14.0,
                    ),
                ),
            )
        )
    for axle in vehicle.axles:
        wheels = axle.wheels()
        if len(wheels) >= 2:
            beam_id = f"{axle.id}_beam"
            items.append(
                CollisionItem(
                    id=beam_id,
                    envelope=CapsuleEnvelope(
                        start=wheels[0].center,
                        end=wheels[-1].center,
                        radius_mm=70.0,
                    ),
                )
            )
            if axle.outside_diameter_mm > 0.0:
                for wheel in wheels:
                    items.append(
                        CollisionItem(
                            id=f"{wheel.id}_tire",
                            envelope=CircleEnvelope(
                                center=wheel.center,
                                radius_mm=axle.outside_diameter_mm / 2.0,
                            ),
                            excluded_pair_ids=(beam_id,),
                        )
                    )
    report = analyze_clearance(items)
    return _clearance_report_payload(report)


def build_demo_payload(
    beta_deg: float,
    wheelbase_mm: float = 4360.0,
    track_mm: float = 2500.0,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> dict[str, object]:
    if wheelbase_mm <= 0.0 or track_mm <= 0.0:
        raise ValueError("wheelbase_mm and track_mm must be positive")
    if vehicle is None:
        vehicle, solution, radius = build_demo_solution(
            beta_deg,
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
        )
    else:
        reference_length_mm = vehicle.axle_span_mm() or wheelbase_mm
        radius = beta_to_reference_radius_mm(math.radians(beta_deg), reference_length_mm)
        solution = solve_ideal_steering_from_radius(vehicle, radius)
    if abs(beta_deg) > vehicle.maximum_articulation_deg + 1e-9:
        raise ArticulationLimitExceededError(beta_deg, vehicle.maximum_articulation_deg)
    rig = linkage_rig or build_reference_linkage_demo()
    driver_point = driver_point_arc(
        rig.driver_arc_center,
        rig.driver_arc_radius_mm,
        math.radians(beta_deg),
    )
    linkage_state = solve_planar_linkage(
        rig.spec,
        driver_point,
        branch_hint=rig.branch_hint,
    )
    linkage = _linkage_payload(rig, linkage_state)
    actual_solution = solve_actual_steering(
        vehicle,
        linkage_state,
        math.radians(beta_deg),
        ideal_solution=solution,
    )
    actual_comparison = compare_actual_to_ideal(
        actual_solution,
        solution,
        vehicle=vehicle,
        beta_rad=math.radians(beta_deg),
    )
    clearance = _clearance_payload(vehicle, rig, linkage_state)
    body_half_length = vehicle.body_length_mm / 2.0
    body_half_width = vehicle.body_width_mm / 2.0

    body_outline_points = list(vehicle.body_polygon)
    if not body_outline_points:
        body_outline_points = [
            Point2D(-body_half_length, -body_half_width),
            Point2D(body_half_length, -body_half_width),
            Point2D(body_half_length, body_half_width),
            Point2D(-body_half_length, body_half_width),
        ]
    body_outline = [
        _point_payload(point + vehicle.origin)
        for point in body_outline_points
    ]

    wheel_angles_deg = solution.wheel_steering_angles_deg()
    front_solution = max(solution.axles, key=lambda item: item.center.x_mm) if solution.axles else None
    rear_solution = min(solution.axles, key=lambda item: item.center.x_mm) if solution.axles else None

    metrics = {
        "max_abs_wheel_angle_deg": max((abs(value) for value in wheel_angles_deg.values()), default=0.0),
        "front_axle_center_angle_deg": None if front_solution is None else front_solution.center_steering_angle_deg,
        "rear_axle_center_angle_deg": None if rear_solution is None else rear_solution.center_steering_angle_deg,
        "linkage_actual_steering_deg": linkage["state"]["steering_angle_deg"],
        "linkage_vs_ideal_front_axle_deg": None,
        "linkage_actual_front_left_deg": linkage["state"]["steering_angle_deg"],
        "linkage_actual_front_right_deg": linkage["state"]["companion_steering_angle_deg"],
        "linkage_vs_ideal_front_left_deg": None,
        "linkage_vs_ideal_front_right_deg": None,
        "minimum_clearance_mm": clearance["minimum_clearance_mm"],
        "max_abs_wheel_error_deg": actual_comparison["max_abs_error_deg"],
        "mean_abs_wheel_error_deg": actual_comparison["mean_abs_error_deg"],
        "rms_wheel_error_deg": actual_comparison["rms_error_deg"],
        "max_abs_inner_wheel_error_deg": actual_comparison["max_abs_inner_error_deg"],
        "max_abs_outer_wheel_error_deg": actual_comparison["max_abs_outer_error_deg"],
        "front_rear_synchronization_error_deg": actual_comparison[
            "front_rear_synchronization_error_deg"
        ],
        "synchronization_errors_deg": actual_comparison["synchronization_errors_deg"],
        "max_abs_synchronization_error_deg": actual_comparison[
            "max_abs_synchronization_error_deg"
        ],
    }

    if front_solution is not None and rear_solution is not None and front_solution.axle_id != rear_solution.axle_id:
        metrics["front_rear_phase_deg"] = (
            front_solution.center_steering_angle_deg
            - rear_solution.center_steering_angle_deg
        )
    else:
        metrics["front_rear_phase_deg"] = None

    if metrics["front_axle_center_angle_deg"] is not None:
        metrics["linkage_vs_ideal_front_axle_deg"] = (
            linkage["state"]["steering_angle_deg"] - metrics["front_axle_center_angle_deg"]
        )
    if front_solution is not None:
        metrics["linkage_vs_ideal_front_left_deg"] = (
            linkage["state"]["steering_angle_deg"] - front_solution.left_wheel.steering_angle_deg
        )
        if linkage["state"]["companion_steering_angle_deg"] is not None:
            metrics["linkage_vs_ideal_front_right_deg"] = (
                linkage["state"]["companion_steering_angle_deg"] - front_solution.right_wheel.steering_angle_deg
            )

    return {
        "beta_deg": beta_deg,
        "beta_rad": math.radians(beta_deg),
        "turn_radius_mm": radius,
        "icr": None if solution.icr is None else _point_payload(solution.icr),
        "vehicle": {
            "id": vehicle.id,
            "name": vehicle.name,
            "axle_count": len(vehicle.axles),
            "body_length_mm": vehicle.body_length_mm,
            "body_width_mm": vehicle.body_width_mm,
            "origin": _point_payload(vehicle.origin),
        },
        "vehicle_config": _vehicle_config_payload(vehicle),
        "actual_steering": serialize_actual_steering(
            actual_solution,
            solution,
            vehicle=vehicle,
            beta_rad=math.radians(beta_deg),
        ),
        "body_outline": body_outline,
        "axles": [
            _axle_payload(axle_solution, axle)
            for axle, axle_solution in zip(vehicle.axles, solution.axles, strict=True)
        ],
        "vehicle_combination": serialize_vehicle_combination(
            build_reference_demo_combination(
                wheelbase_mm=vehicle.axle_span_mm() or wheelbase_mm,
                track_mm=max((axle.track_mm for axle in vehicle.axles), default=track_mm),
                articulation_rad=math.radians(beta_deg),
            )
        ) if len(vehicle.axles) == 2 and vehicle.id == "reference_demo_combination" else None,
        "linkage": linkage,
        "clearance": clearance,
        "metrics": metrics,
    }


class DemoRequestHandler(BaseHTTPRequestHandler):
    server_version = "EasyTowingDemo/0.1"

    def _send_bytes(self, content: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_download(self, content_builder, content_type: str, filename: str) -> None:
        try:
            content = content_builder()
        except EngineeringError as error:
            self._send_json(
                {"error_code": error.code, "message": str(error)},
                status=422,
            )
            return
        except (TypeError, ValueError) as error:
            self.send_error(400, str(error))
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_file(self, filename: str, content_type: str) -> None:
        path = WEB_DIR / filename
        if not path.exists():
            self.send_error(404, "File not found")
            return
        self._send_bytes(path.read_bytes(), content_type)

    def _send_json(self, payload: dict[str, object], status: int = 200) -> None:
        content = json.dumps(payload, indent=2).encode("utf-8")
        self._send_bytes(content, "application/json; charset=utf-8", status=status)

    def _read_json_body(self) -> dict[str, object]:
        content_length = self.headers.get("Content-Length", "0")
        try:
            length = int(content_length)
        except ValueError as exc:
            raise ValueError("Invalid Content-Length header") from exc
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def log_message(self, format: str, *args) -> None:  # noqa: A003 - match BaseHTTPRequestHandler
        return

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler interface
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_file("index.html", "text/html; charset=utf-8")
            return
        if parsed.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if parsed.path == "/app.js":
            self._send_file("app.js", "application/javascript; charset=utf-8")
            return
        if parsed.path == "/styles.css":
            self._send_file("styles.css", "text/css; charset=utf-8")
            return
        if parsed.path == "/api/ideal-steering":
            query = parse_qs(parsed.query)
            try:
                beta_deg = _query_float(query, "beta_deg", 0.0)
                wheelbase_mm = _query_float(query, "wheelbase_mm", 4360.0)
                track_mm = _query_float(query, "track_mm", 2500.0)
                payload = build_demo_payload(
                    beta_deg,
                    wheelbase_mm=wheelbase_mm,
                    track_mm=track_mm,
                )
            except ValueError as error:
                self.send_error(400, str(error))
                return
            except EngineeringError as error:
                self._send_json(
                    {
                        "error_code": error.code,
                        "message": str(error),
                        "beta_deg": beta_deg,
                    },
                    status=422,
                )
                return
            self._send_json(payload)
            return
        if parsed.path == "/api/optimize":
            query = parse_qs(parsed.query)
            mode = query.get("mode", ["quick"])[0]
            if mode not in {"quick", "full"}:
                self.send_error(400, "mode must be quick or full")
                return
            enabled_values = query.get("enabled")
            enabled_ids = None
            if enabled_values is not None:
                enabled_ids = {
                    value.strip()
                    for value in enabled_values[0].split(",")
                    if value.strip()
                }
            try:
                design_cases = _parse_design_cases(query.get("cases", [None])[0])
                clearance_target_mm = _query_float(query, "clearance_target_mm", 20.0)
                steering_error_weight = _query_float(query, "steering_error_weight", 1.0)
                clearance_weight = _query_float(query, "clearance_weight", 12.0)
                clearance_violation_weight = _query_float(query, "clearance_violation_weight", 250.0)
                failure_weight = _query_float(query, "failure_weight", 100000.0)
                preferred_weight = _query_float(query, "preferred_weight", 0.05)
                complexity_weight = _query_float(query, "complexity_weight", 0.02)
                synchronization_error_weight = _query_float(query, "synchronization_error_weight", 0.5)
            except ValueError as error:
                self.send_error(400, str(error))
                return
            if any(
                value < 0.0
                for value in (
                    clearance_target_mm,
                    steering_error_weight,
                    clearance_weight,
                    clearance_violation_weight,
                    failure_weight,
                    preferred_weight,
                    complexity_weight,
                    synchronization_error_weight,
                )
            ):
                self.send_error(400, "optimization targets and weights must be non-negative")
                return
            payload = _optimization_payload(
                mode,
                enabled_ids=enabled_ids,
                clearance_target_mm=clearance_target_mm,
                design_cases=design_cases,
                weights=OptimizationWeights(
                    steering_error=steering_error_weight,
                    clearance=clearance_weight,
                    clearance_violation=clearance_violation_weight,
                    failure=failure_weight,
                    preferred=preferred_weight,
                    complexity=complexity_weight,
                    synchronization_error=synchronization_error_weight,
                ),
            )
            self._send_json(payload)
            return
        if parsed.path == "/api/import.dxf":
            self.send_error(405, "POST required")
            return
        if parsed.path in {
            "/api/export.json",
            "/api/export.csv",
            "/api/export.pdf",
            "/api/export.png",
            "/api/export.svg",
            "/api/export.dxf",
            "/api/steering-curves.svg",
            "/api/swept-path.svg",
        }:
            query = parse_qs(parsed.query)
            mode = query.get("mode", ["quick"])[0]
            if mode not in {"quick", "full"}:
                self.send_error(400, "mode must be quick or full")
                return
            beta_values = query.get("beta_deg", ["0"])
            try:
                beta_deg = float(beta_values[0])
            except ValueError:
                self.send_error(400, "beta_deg must be numeric")
                return
            try:
                wheelbase_mm = _query_float(query, "wheelbase_mm", 4360.0)
                track_mm = _query_float(query, "track_mm", 2500.0)
                if wheelbase_mm <= 0.0 or track_mm <= 0.0:
                    raise ValueError("wheelbase_mm and track_mm must be positive")
                vehicle_config = query.get("vehicle_config", [None])[0]
                if vehicle_config is not None:
                    parsed_vehicle = _parse_vehicle_config(json.loads(vehicle_config))
                    assert parsed_vehicle is not None
                    vehicle = parsed_vehicle[0]
                else:
                    _wheelbase, _track, _normalized_vehicle_config, vehicle = _project_vehicle_inputs(
                        {"wheelbase_mm": wheelbase_mm, "track_mm": track_mm}
                    )
                linkage_rig = None
                raw_linkage = query.get("linkage", [None])[0]
                if raw_linkage is not None:
                    linkage_rig = _parse_linkage_rig(json.loads(raw_linkage))
            except (TypeError, ValueError, json.JSONDecodeError) as error:
                self.send_error(400, str(error))
                return
            except EngineeringError as error:
                self._send_json(
                    {"error_code": error.code, "message": str(error), "beta_deg": beta_deg},
                    status=422,
                )
                return
            try:
                build_demo_payload(
                    beta_deg,
                    wheelbase_mm=wheelbase_mm,
                    track_mm=track_mm,
                    linkage_rig=linkage_rig,
                    vehicle=vehicle,
                )
            except EngineeringError as error:
                self._send_json(
                    {"error_code": error.code, "message": str(error), "beta_deg": beta_deg},
                    status=422,
                )
                return

            if parsed.path == "/api/export.json":
                self._send_download(
                    lambda: json.dumps(
                        build_export_bundle(beta_deg, mode, linkage_rig=linkage_rig, vehicle=vehicle),
                        indent=2,
                    ).encode("utf-8"),
                    "application/json; charset=utf-8",
                    "easytowing-report.json",
                )
                return

            if parsed.path == "/api/export.csv":
                self._send_download(
                    lambda: build_export_csv(beta_deg, mode, linkage_rig=linkage_rig, vehicle=vehicle).encode("utf-8"),
                    "text/csv; charset=utf-8",
                    "easytowing-report.csv",
                )
                return

            if parsed.path == "/api/export.pdf":
                self._send_download(
                    lambda: build_export_pdf(beta_deg, mode, linkage_rig=linkage_rig, vehicle=vehicle),
                    "application/pdf",
                    "easytowing-report.pdf",
                )
                return

            if parsed.path == "/api/export.png":
                self._send_download(
                    lambda: build_export_png(beta_deg, mode, linkage_rig=linkage_rig, vehicle=vehicle),
                    "image/png",
                    "easytowing-snapshot.png",
                )
                return

            if parsed.path == "/api/export.svg":
                self._send_download(
                    lambda: build_dimensioned_svg(
                        beta_deg,
                        mode,
                        linkage_rig=linkage_rig,
                        vehicle=vehicle,
                    ).encode("utf-8"),
                    "image/svg+xml; charset=utf-8",
                    "easytowing-sketch.svg",
                )
                return

            if parsed.path == "/api/export.dxf":
                self._send_download(
                    lambda: build_export_dxf(
                        beta_deg,
                        mode,
                        linkage_rig=linkage_rig,
                        vehicle=vehicle,
                    ).encode("utf-8"),
                    "application/dxf; charset=utf-8",
                    "easytowing-sketch.dxf",
                )
                return

            if parsed.path == "/api/steering-curves.svg":
                try:
                    step_deg = _query_float(query, "step_deg", 1.0)
                    beta_min_deg = _query_float(query, "beta_min_deg", -45.0)
                    beta_max_deg = _query_float(query, "beta_max_deg", 45.0)
                except ValueError as error:
                    self.send_error(400, str(error))
                    return
                self._send_download(
                    lambda: build_steering_curves_svg(
                        beta_deg,
                        mode,
                        step_deg=step_deg,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        linkage_rig=linkage_rig,
                        vehicle=vehicle,
                    ).encode("utf-8"),
                    "image/svg+xml; charset=utf-8",
                    "easytowing-steering-curves.svg",
                )
                return

            if parsed.path == "/api/swept-path.svg":
                try:
                    step_deg = _query_float(query, "step_deg", 1.0)
                    beta_min_deg = _query_float(query, "beta_min_deg", -45.0)
                    beta_max_deg = _query_float(query, "beta_max_deg", 45.0)
                except ValueError as error:
                    self.send_error(400, str(error))
                    return
                self._send_download(
                    lambda: build_swept_path_svg(
                        beta_deg,
                        mode,
                        step_deg=step_deg,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        vehicle=vehicle,
                    ).encode("utf-8"),
                    "image/svg+xml; charset=utf-8",
                    "easytowing-swept-path.svg",
                )
                return

        if parsed.path == "/api/projects":
            PROJECT_STORE.ensure_seed_project()
            self._send_json(_project_state_payload())
            return

        parts = [part for part in parsed.path.split("/") if part]
        if len(parts) == 3 and parts[0] == "api" and parts[1] == "projects":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            self._send_json(_project_detail_payload(project))
            return
        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            self._send_json(
                {
                    "project": _project_detail_payload(project),
                    "revisions": [_revision_payload(revision) for revision in project.revisions],
                }
            )
            return
        if len(parts) == 5 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            revision = project.get_revision(parts[4])
            if revision is None:
                self.send_error(404, "Revision not found")
                return
            self._send_json(
                {
                    "project": _project_detail_payload(project),
                    "revision": _revision_payload(revision, include_snapshot=True),
                }
            )
            return

        self.send_error(404, "Not found")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler interface
        parsed = urlparse(self.path)
        parts = [part for part in parsed.path.split("/") if part]
        try:
            body = self._read_json_body()
        except Exception as exc:  # noqa: BLE001 - return a clear HTTP 400
            self.send_error(400, str(exc))
            return

        if parsed.path == "/api/calculate/ideal-steering":
            try:
                payload = _ideal_steering_request_payload(body)
            except EngineeringError as error:
                self._send_json(
                    {"error_code": error.code, "message": str(error)},
                    status=422,
                )
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            self._send_json(payload)
            return

        if parsed.path == "/api/calculate/kinematic":
            try:
                beta_deg = _config_float(body, "beta_deg", 0.0)
                wheelbase_mm, track_mm, _vehicle_config, vehicle = _project_vehicle_inputs(body)
                assert beta_deg is not None and wheelbase_mm is not None and track_mm is not None
                if not math.isfinite(beta_deg):
                    raise ValueError("beta_deg must be finite")
                rig = _parse_linkage_rig(body.get("linkage", body.get("linkage_config")))
                payload = build_demo_payload(
                    beta_deg,
                    wheelbase_mm=wheelbase_mm,
                    track_mm=track_mm,
                    linkage_rig=rig,
                    vehicle=vehicle,
                )
            except EngineeringError as error:
                self._send_json(
                    {
                        "error_code": error.code,
                        "message": str(error),
                        "beta_deg": body.get("beta_deg", 0.0),
                    },
                    status=422,
                )
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            self._send_json(payload)
            return

        if parsed.path == "/api/calculate/optimization":
            try:
                mode = str(body.get("mode", "quick"))
                if mode not in {"quick", "full"}:
                    raise ValueError("mode must be quick or full")
                wheelbase_mm, track_mm, _vehicle_config, vehicle = _project_vehicle_inputs(body)
                assert wheelbase_mm is not None and track_mm is not None
                clearance_target_mm = _config_float(body, "clearance_target_mm", 20.0)
                assert clearance_target_mm is not None
                weight_values = {
                    "steering_error": _config_float(body, "steering_error_weight", 1.0),
                    "clearance": _config_float(body, "clearance_weight", 12.0),
                    "clearance_violation": _config_float(body, "clearance_violation_weight", 250.0),
                    "failure": _config_float(body, "failure_weight", 100000.0),
                    "preferred": _config_float(body, "preferred_weight", 0.05),
                    "complexity": _config_float(body, "complexity_weight", 0.02),
                    "synchronization_error": _config_float(body, "synchronization_error_weight", 0.5),
                }
                if clearance_target_mm < 0.0 or any(
                    value is None or value < 0.0 for value in weight_values.values()
                ):
                    raise ValueError("optimization targets and weights must be non-negative")
                enabled_ids = _parse_enabled_ids(body.get("enabled_ids"))
                design_cases = _parse_design_cases(body.get("design_cases"))
                raw_linkage = body.get("linkage", body.get("linkage_config"))
                rig = None if raw_linkage is None else _parse_linkage_rig(raw_linkage)
                if vehicle is None:
                    vehicle = build_demo_solution(
                        0.0,
                        wheelbase_mm=wheelbase_mm,
                        track_mm=track_mm,
                    )[0]
                payload = _optimization_payload(
                    mode,
                    enabled_ids=enabled_ids,
                    clearance_target_mm=clearance_target_mm,
                    weights=OptimizationWeights(
                        steering_error=weight_values["steering_error"] or 0.0,
                        clearance=weight_values["clearance"] or 0.0,
                        clearance_violation=weight_values["clearance_violation"] or 0.0,
                        failure=weight_values["failure"] or 0.0,
                        preferred=weight_values["preferred"] or 0.0,
                        complexity=weight_values["complexity"] or 0.0,
                        synchronization_error=weight_values["synchronization_error"] or 0.0,
                    ),
                    design_cases=design_cases,
                    linkage_rig=rig,
                    vehicle=vehicle,
                )
            except EngineeringError as error:
                self._send_json(
                    {"error_code": error.code, "message": str(error)},
                    status=422,
                )
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            self._send_json(payload)
            return

        if len(parts) == 2 and parts[0] == "api" and parts[1] == "projects":
            name = str(body.get("name", "Reference Demo Project"))
            try:
                beta_deg = float(body.get("beta_deg", 0.0))
                beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                wheelbase_mm, track_mm, vehicle_config, vehicle = _project_vehicle_inputs(body)
                design_cases = _parse_design_cases(body.get("design_cases"))
                linkage_config = _validated_linkage_config(
                    body.get("linkage_config", body.get("linkage"))
                )
                linkage_rig = None if linkage_config is None else _parse_linkage_rig(linkage_config)
            except EngineeringError as error:
                self._send_json({"error_code": error.code, "message": str(error)}, status=422)
                return
            except (TypeError, ValueError):
                self.send_error(400, "beta, articulation bounds, design cases, and linkage must be valid")
                return
            optimization_mode = str(body.get("optimization_mode", "quick"))
            note = str(body.get("note", "Initial revision"))
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            project = PROJECT_STORE.create_project(
                name,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
                beta_min_deg=beta_min_deg,
                beta_max_deg=beta_max_deg,
                design_cases=design_cases,
                linkage_config=linkage_config,
                wheelbase_mm=wheelbase_mm,
                track_mm=track_mm,
                linkage_rig=linkage_rig,
                vehicle=vehicle,
                vehicle_config=vehicle_config,
            )
            self._send_json({"project": _project_detail_payload(project)}, status=201)
            return

        if len(parts) == 2 and parts[0] == "api" and parts[1] == "import.dxf":
            dxf_text = str(body.get("dxf_text", ""))
            if not dxf_text.strip():
                self.send_error(400, "dxf_text is required")
                return
            source_name = str(body.get("source_name", ""))
            try:
                role_overrides = _parse_role_overrides(body.get("role_overrides"))
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            try:
                report = analyze_dxf_import(dxf_text, source_name=source_name)
                if role_overrides:
                    report = apply_dxf_role_overrides(report, role_overrides)
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            payload = serialize_dxf_import_report(report)
            self._send_json(payload)
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "optimization":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            action = str(body.get("action", "apply")).lower()
            if action == "reject":
                self._send_json({"status": "rejected", "project": _project_detail_payload(project)})
                return
            if action != "apply":
                self.send_error(400, "action must be apply or reject")
                return
            try:
                beta_deg = float(body.get("beta_deg", 0.0))
                beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                wheelbase_mm, track_mm, vehicle_config, vehicle = _project_vehicle_inputs(body)
                optimization_mode = str(body.get("optimization_mode", "quick"))
                enabled_ids = _parse_enabled_ids(body.get("enabled_ids"))
                design_cases = _parse_design_cases(body.get("design_cases"))
                linkage_config = _validated_linkage_config(
                    body.get("linkage_config", body.get("linkage"))
                )
                linkage_rig = None if linkage_config is None else _parse_linkage_rig(linkage_config)
            except EngineeringError as error:
                self._send_json({"error_code": error.code, "message": str(error)}, status=422)
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            note = str(body.get("note", "Applied optimized design"))
            revision = PROJECT_STORE.append_revision(
                project.id,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
                enabled_ids=enabled_ids,
                accepted_optimization=True,
                beta_min_deg=beta_min_deg,
                beta_max_deg=beta_max_deg,
                design_cases=design_cases,
                linkage_config=linkage_config,
                wheelbase_mm=wheelbase_mm,
                track_mm=track_mm,
                linkage_rig=linkage_rig,
                vehicle=vehicle,
                vehicle_config=vehicle_config,
            )
            latest_project = PROJECT_STORE.get_project(project.id) or project
            self._send_json(
                {
                    "status": "applied",
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision, include_snapshot=True),
                },
                status=201,
            )
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            try:
                beta_deg = float(body.get("beta_deg", 0.0))
                beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                wheelbase_mm, track_mm, vehicle_config, vehicle = _project_vehicle_inputs(body)
            except (TypeError, ValueError):
                self.send_error(400, "beta_deg must be numeric")
                return
            optimization_mode = str(body.get("optimization_mode", "quick"))
            note = str(body.get("note", "Revision"))
            try:
                enabled_ids = _parse_enabled_ids(body.get("enabled_ids"))
                design_cases = _parse_design_cases(body.get("design_cases"))
                linkage_config = _validated_linkage_config(
                    body.get("linkage_config", body.get("linkage"))
                )
                linkage_rig = None if linkage_config is None else _parse_linkage_rig(linkage_config)
            except EngineeringError as error:
                self._send_json({"error_code": error.code, "message": str(error)}, status=422)
                return
            except ValueError as error:
                self.send_error(400, str(error))
                return
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            revision = PROJECT_STORE.append_revision(
                project.id,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
                enabled_ids=enabled_ids,
                beta_min_deg=beta_min_deg,
                beta_max_deg=beta_max_deg,
                design_cases=design_cases,
                linkage_config=linkage_config,
                wheelbase_mm=wheelbase_mm,
                track_mm=track_mm,
                linkage_rig=linkage_rig,
                vehicle=vehicle,
                vehicle_config=vehicle_config,
            )
            latest_project = PROJECT_STORE.get_project(project.id) or project
            self._send_json(
                {
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision),
                },
                status=201,
            )
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "restore":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            revision_id = str(body.get("revision_id", ""))
            if not revision_id:
                self.send_error(400, "revision_id is required")
                return
            try:
                revision = PROJECT_STORE.restore_revision(project.id, revision_id)
            except KeyError:
                self.send_error(404, "Revision not found")
                return
            latest_project = PROJECT_STORE.get_project(project.id) or project
            self._send_json(
                {
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision),
                }
            )
            return

        self.send_error(404, "Not found")


def run_server(host: str = "127.0.0.1", port: int = 8000) -> None:
    if not WEB_DIR.exists():
        raise RuntimeError(f"Web assets not found at {WEB_DIR}")
    PROJECT_STORE.ensure_seed_project()
    server = ThreadingHTTPServer((host, port), DemoRequestHandler)
    address = f"http://{host}:{port}"
    print(f"EasyTowing demo server running at {address}")
    print("Open the URL in a browser to inspect the ideal steering prototype.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the EasyTowing demo server.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8000, type=int)
    args = parser.parse_args()
    run_server(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
