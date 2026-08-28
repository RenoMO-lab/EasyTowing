from __future__ import annotations

from dataclasses import dataclass
import csv
import io
import json
import math
from datetime import datetime
from functools import lru_cache
from typing import Any, Iterable
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw, ImageFont
from reportlab.lib import colors
from reportlab.lib.pagesizes import landscape, letter
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen import canvas

from .collision import CapsuleEnvelope, CircleEnvelope, CollisionItem, ClearanceReport, PolygonEnvelope, analyze_clearance
from .clearance_model import build_linkage_clearance_items
from .actual_steering import (
    ActualSteeringSolution,
    actual_steering_errors_deg,
    compare_actual_to_ideal,
    serialize_actual_steering,
    solve_actual_steering,
)
from .design_cases import DesignCase
from .errors import ArticulationLimitExceededError, InvalidGeometryError
from .geometry import Point2D
from .linkage import (
    LinkageDemoRig,
    PlanarLinkageSpec,
    PlanarLinkageState,
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage,
)
from .model import (
    SteeringSynchronization,
    VehicleLayout,
    build_reference_demo_combination,
    build_reference_demo_layout,
    serialize_vehicle_combination,
)
from .optimization import (
    LinkageOptimizationProblem,
    OptimizationResult,
    OptimizedVariable,
    build_branch_hint,
    build_optimized_spec,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from .steering import (
    IdealSteeringSolution,
    beta_to_reference_radius_mm,
    build_demo_solution,
    solve_ideal_steering_from_radius,
)


@dataclass(frozen=True, slots=True)
class ExportContext:
    beta_deg: float
    optimization_mode: str
    vehicle: VehicleLayout
    ideal_solution: IdealSteeringSolution
    baseline_rig: LinkageDemoRig
    baseline_state: PlanarLinkageState
    baseline_clearance: ClearanceReport
    optimization_problem: LinkageOptimizationProblem
    optimization_result: OptimizationResult
    optimized_spec: PlanarLinkageSpec
    optimized_state: PlanarLinkageState | None
    optimized_clearance: ClearanceReport | None
    baseline_actual: ActualSteeringSolution
    optimized_actual: ActualSteeringSolution | None


def _point_payload(point: Point2D) -> dict[str, float]:
    return {"x_mm": point.x_mm, "y_mm": point.y_mm}


def _vehicle_body_points(vehicle: VehicleLayout) -> tuple[Point2D, ...]:
    """Return the configured body outline in the vehicle coordinate frame."""

    if vehicle.body_polygon:
        return tuple(vehicle.origin + point for point in vehicle.body_polygon)
    half_length = vehicle.body_length_mm / 2.0
    half_width = vehicle.body_width_mm / 2.0
    return tuple(
        vehicle.origin + point
        for point in (
            Point2D(-half_length, -half_width),
            Point2D(half_length, -half_width),
            Point2D(half_length, half_width),
            Point2D(-half_length, half_width),
        )
    )


def _vehicle_local_body_points(vehicle: VehicleLayout) -> tuple[Point2D, ...]:
    if vehicle.body_polygon:
        return vehicle.body_polygon
    half_length = vehicle.body_length_mm / 2.0
    half_width = vehicle.body_width_mm / 2.0
    return (
        Point2D(-half_length, -half_width),
        Point2D(half_length, -half_width),
        Point2D(half_length, half_width),
        Point2D(-half_length, half_width),
    )


def _serialize_envelope(envelope) -> dict[str, object]:
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


def _clearance_items_for_state(vehicle: VehicleLayout, spec: PlanarLinkageSpec, state: PlanarLinkageState) -> tuple[CollisionItem, ...]:
    return build_linkage_clearance_items(spec, state, vehicle=vehicle)


def _build_clearance_report(vehicle: VehicleLayout, spec: PlanarLinkageSpec, state: PlanarLinkageState) -> ClearanceReport:
    return analyze_clearance(_clearance_items_for_state(vehicle, spec, state))


def _serialize_clearance_report(report: ClearanceReport) -> dict[str, object]:
    return {
        "minimum_clearance_mm": report.minimum_clearance_mm,
        "collision_detected": report.collision_detected,
        "clearance_violation_detected": report.clearance_violation_detected,
        "items": [
            {
                "id": item.id,
                "margin_mm": item.margin_mm,
                "excluded_pair_ids": list(item.excluded_pair_ids),
                "envelope": _serialize_envelope(item.envelope),
            }
            for item in report.items
        ],
        "pairs": [
            {
                "item_a_id": pair.item_a_id,
                "item_b_id": pair.item_b_id,
                "raw_clearance_mm": pair.raw_clearance_mm,
                "required_margin_mm": pair.required_margin_mm,
                "clearance_mm": pair.clearance_mm,
                "overlaps": pair.overlaps,
                "violates_margin": pair.violates_margin,
                "description": pair.description,
            }
            for pair in report.pairs
        ],
        "minimum_pair": None
        if report.minimum_pair is None
        else {
            "item_a_id": report.minimum_pair.item_a_id,
            "item_b_id": report.minimum_pair.item_b_id,
            "raw_clearance_mm": report.minimum_pair.raw_clearance_mm,
            "required_margin_mm": report.minimum_pair.required_margin_mm,
            "clearance_mm": report.minimum_pair.clearance_mm,
            "overlaps": report.minimum_pair.overlaps,
            "violates_margin": report.minimum_pair.violates_margin,
            "description": report.minimum_pair.description,
        },
    }


def _serialize_vehicle(vehicle: VehicleLayout) -> dict[str, object]:
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
        "axle_span_mm": vehicle.axle_span_mm(),
        "axles": [
            {
                "id": axle.id,
                "center": _point_payload(axle.center),
                "track_mm": axle.track_mm,
                "wheel_count": axle.wheel_count,
                "wheel_lateral_offsets_mm": (
                    None
                    if axle.wheel_lateral_offsets_mm is None
                    else list(axle.wheel_lateral_offsets_mm)
                ),
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
                "heading_rad": axle.heading_rad,
                "heading_deg": math.degrees(axle.heading_rad),
                "maximum_steering_angle_deg": axle.maximum_steering_angle_deg,
                "steering_stop_deg": axle.steering_stop_deg,
                "load_kg": axle.load_kg,
                "tire_width_mm": axle.tire_width_mm,
                "outside_diameter_mm": axle.outside_diameter_mm,
                "user_defined_steering_angle_deg": math.degrees(axle.user_defined_steering_angle_rad),
            }
            for axle in vehicle.axles
        ],
    }


def _serialize_vehicle_config(vehicle: VehicleLayout) -> dict[str, object]:
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
                "wheel_lateral_offsets_mm": (
                    None
                    if axle.wheel_lateral_offsets_mm is None
                    else list(axle.wheel_lateral_offsets_mm)
                ),
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


def _serialize_linkage_spec(spec: PlanarLinkageSpec) -> dict[str, object]:
    return {
        "id": spec.id,
        "steering_pivot": _point_payload(spec.steering_pivot),
        "steering_arm_length_mm": spec.steering_arm_length_mm,
        "steering_arm_neutral_angle_rad": spec.steering_arm_neutral_angle_rad,
        "bell_crank_pivot": _point_payload(spec.bell_crank_pivot),
        "bell_crank_input_arm_length_mm": spec.bell_crank_input_arm_length_mm,
        "bell_crank_input_neutral_angle_rad": spec.bell_crank_input_neutral_angle_rad,
        "bell_crank_output_arm_length_mm": spec.bell_crank_output_arm_length_mm,
        "bell_crank_output_neutral_angle_rad": spec.bell_crank_output_neutral_angle_rad,
        "input_rod_length_mm": spec.input_rod_length_mm,
        "tie_rod_length_mm": spec.tie_rod_length_mm,
        "steering_stop_deg": spec.steering_stop_deg,
        "companion_steering_pivot": None if spec.companion_steering_pivot is None else _point_payload(spec.companion_steering_pivot),
        "companion_steering_arm_length_mm": spec.companion_steering_arm_length_mm,
        "companion_steering_arm_neutral_angle_rad": spec.companion_steering_arm_neutral_angle_rad,
        "companion_tie_rod_length_mm": spec.companion_tie_rod_length_mm,
    }


def _serialize_linkage_state(state: PlanarLinkageState | None) -> dict[str, object] | None:
    if state is None:
        return None
    return {
        "driver_point": _point_payload(state.driver_point),
        "input_endpoint": _point_payload(state.input_endpoint),
        "bell_crank_angle_rad": state.bell_crank_angle_rad,
        "bell_crank_angle_deg": state.bell_crank_angle_deg,
        "output_endpoint": _point_payload(state.output_endpoint),
        "steering_endpoint": _point_payload(state.steering_endpoint),
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
    }


def _serialize_optimization_result(result: OptimizationResult) -> dict[str, object]:
    def serialize_metrics(metrics) -> dict[str, object]:
        return {
            "feasible": metrics.feasible,
            "violations": list(metrics.violations),
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

    def serialize_variable(variable: OptimizedVariable) -> dict[str, object]:
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

    return {
        "mode": result.mode,
        "iterations": result.iterations,
        "evaluations": result.evaluations,
        "improved": result.improved,
        "improvement": result.improvement,
        "objective": {
            "clearance_target_mm": result.clearance_target_mm,
            "weights": result.weights.to_dict(),
        },
        "design_cases": [case.to_dict() for case in result.design_cases],
        "baseline": serialize_metrics(result.baseline_metrics),
        "optimized": serialize_metrics(result.optimized_metrics),
        "variables_before": [serialize_variable(variable) for variable in result.baseline_variables],
        "variables_after": [serialize_variable(variable) for variable in result.optimized_variables],
    }


def _format_mm(value: float) -> str:
    return f"{value:.1f} mm"


def _format_deg(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f} deg"


def build_export_context(
    beta_deg: float,
    optimization_mode: str = "quick",
    enabled_ids: Iterable[str] | None = None,
    design_cases: Iterable[DesignCase] | None = None,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
    require_feasible: bool = True,
) -> ExportContext:
    provided_vehicle = vehicle
    if vehicle is None:
        vehicle, ideal_solution, _radius = build_demo_solution(beta_deg)
    else:
        reference_length = vehicle.axle_span_mm() or 4360.0
        radius = beta_to_reference_radius_mm(math.radians(beta_deg), reference_length)
        ideal_solution = solve_ideal_steering_from_radius(vehicle, radius)
    if abs(beta_deg) > vehicle.maximum_articulation_deg + 1e-9:
        raise ArticulationLimitExceededError(beta_deg, vehicle.maximum_articulation_deg)
    baseline_rig = linkage_rig or build_reference_linkage_demo()
    baseline_driver_point = driver_point_arc(
        baseline_rig.driver_arc_center,
        baseline_rig.driver_arc_radius_mm,
        math.radians(beta_deg),
    )
    baseline_state = solve_planar_linkage(
        baseline_rig.spec,
        baseline_driver_point,
        branch_hint=baseline_rig.branch_hint,
    )
    baseline_actual = solve_actual_steering(
        vehicle,
        baseline_state,
        math.radians(beta_deg),
        ideal_solution=ideal_solution,
    )
    baseline_clearance = _build_clearance_report(vehicle, baseline_rig.spec, baseline_state)
    optimization_problem = build_reference_optimization_problem(
        mode=optimization_mode,
        enabled_ids=enabled_ids,
        design_cases=design_cases,
        base_rig=baseline_rig if linkage_rig is not None else None,
        vehicle=provided_vehicle,
    )
    optimization_result = optimize_linkage_problem(
        optimization_problem,
        require_feasible=require_feasible,
    )
    optimized_spec = build_optimized_spec(optimization_problem.baseline_spec, optimization_result.optimized_variables)

    driver_point = driver_point_arc(
        optimization_problem.base_rig.driver_arc_center,
        optimization_problem.base_rig.driver_arc_radius_mm,
        math.radians(beta_deg),
    )
    optimized_state: PlanarLinkageState | None
    optimized_clearance: ClearanceReport | None
    try:
        optimized_state = solve_planar_linkage(optimized_spec, driver_point, branch_hint=build_branch_hint(optimized_spec))
        optimized_clearance = _build_clearance_report(vehicle, optimized_spec, optimized_state)
    except Exception:
        optimized_state = None
        optimized_clearance = None
    optimized_actual = (
        None
        if optimized_state is None
        else solve_actual_steering(
            vehicle,
            optimized_state,
            math.radians(beta_deg),
            ideal_solution=ideal_solution,
        )
    )

    return ExportContext(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        vehicle=vehicle,
        ideal_solution=ideal_solution,
        baseline_rig=baseline_rig,
        baseline_state=baseline_state,
        baseline_clearance=baseline_clearance,
        optimization_problem=optimization_problem,
        optimization_result=optimization_result,
        optimized_spec=optimized_spec,
        optimized_state=optimized_state,
        optimized_clearance=optimized_clearance,
        baseline_actual=baseline_actual,
        optimized_actual=optimized_actual,
    )


def build_export_bundle(
    beta_deg: float,
    optimization_mode: str = "quick",
    enabled_ids: Iterable[str] | None = None,
    design_cases: Iterable[DesignCase] | None = None,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
    require_feasible: bool = True,
) -> dict[str, object]:
    context = build_export_context(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        enabled_ids=enabled_ids,
        design_cases=design_cases,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
        require_feasible=require_feasible,
    )
    return {
        "beta_deg": context.beta_deg,
        "optimization_mode": context.optimization_mode,
        "vehicle": _serialize_vehicle(context.vehicle),
        "vehicle_config": _serialize_vehicle_config(context.vehicle),
        "ideal": {
            "wheel_angles_deg": context.ideal_solution.wheel_angles_deg(),
            "axle_center_angles_deg": context.ideal_solution.axle_center_angles_deg(),
            "wheel_steering_angles_deg": context.ideal_solution.wheel_steering_angles_deg(),
            "axle_center_steering_angles_deg": context.ideal_solution.axle_center_steering_angles_deg(),
        },
        "actual_steering": {
            "baseline": serialize_actual_steering(
                context.baseline_actual,
                context.ideal_solution,
                vehicle=context.vehicle,
                beta_rad=math.radians(context.beta_deg),
            ),
            "optimized": None
            if context.optimized_actual is None
            else serialize_actual_steering(
                context.optimized_actual,
                context.ideal_solution,
                vehicle=context.vehicle,
                beta_rad=math.radians(context.beta_deg),
            ),
        },
        "baseline": {
            "spec": _serialize_linkage_spec(context.baseline_rig.spec),
            "state": _serialize_linkage_state(context.baseline_state),
            "clearance": _serialize_clearance_report(context.baseline_clearance),
        },
        "optimized": {
            "spec": _serialize_linkage_spec(context.optimized_spec),
            "state": _serialize_linkage_state(context.optimized_state),
            "clearance": None if context.optimized_clearance is None else _serialize_clearance_report(context.optimized_clearance),
        },
        "optimization": _serialize_optimization_result(context.optimization_result),
        "vehicle_combination": (
            serialize_vehicle_combination(
                build_reference_demo_combination(
                    wheelbase_mm=context.vehicle.axle_span_mm() or 4360.0,
                    track_mm=max((axle.track_mm for axle in context.vehicle.axles), default=2500.0),
                    articulation_rad=math.radians(beta_deg),
                )
            )
            if context.vehicle.id == "reference_demo_combination" and len(context.vehicle.axles) == 2
            else None
        ),
        "comparison": {
            "metrics": _comparison_metric_rows(context),
            "changed_variables": _changed_variable_rows(context.optimization_result),
        },
    }


def build_export_json(
    beta_deg: float,
    optimization_mode: str = "quick",
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> str:
    return json.dumps(
        build_export_bundle(
            beta_deg,
            optimization_mode,
            linkage_rig=linkage_rig,
            vehicle=vehicle,
        ),
        indent=2,
    )


def build_export_csv(
    beta_deg: float,
    optimization_mode: str = "quick",
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> str:
    context = build_export_context(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    output = io.StringIO()
    writer = csv.writer(output)
    wheel_columns = [
        column
        for wheel in context.vehicle.wheels()
        for column in (
            f"ideal_{wheel.id}_steering_deg",
            f"baseline_{wheel.id}_actual_deg",
            f"optimized_{wheel.id}_actual_deg",
            f"baseline_{wheel.id}_error_deg",
            f"optimized_{wheel.id}_error_deg",
        )
    ]
    synchronization_columns = [
        column
        for sync in context.vehicle.steering_synchronizations
        for column in (
            f"baseline_{sync.id}_sync_error_deg",
            f"optimized_{sync.id}_sync_error_deg",
        )
    ]
    writer.writerow(
        [
            "beta_deg",
            "ideal_front_left_deg",
            "ideal_front_right_deg",
            "ideal_rear_left_deg",
            "ideal_rear_right_deg",
            *wheel_columns,
            *synchronization_columns,
            "baseline_actual_deg",
            "baseline_front_left_actual_deg",
            "baseline_front_right_actual_deg",
            "optimized_actual_deg",
            "optimized_front_left_actual_deg",
            "optimized_front_right_actual_deg",
            "baseline_error_deg",
            "baseline_front_left_error_deg",
            "baseline_front_right_error_deg",
            "optimized_error_deg",
            "optimized_front_left_error_deg",
            "optimized_front_right_error_deg",
            "baseline_clearance_mm",
            "optimized_clearance_mm",
            "baseline_status",
            "optimized_status",
        ]
    )

    optimized_spec = context.optimized_spec
    optimized_hint = build_branch_hint(optimized_spec)
    for sample_beta_deg in context.optimization_problem.beta_samples_deg:
        if vehicle is None:
            sample_vehicle, sample_ideal, _ = build_demo_solution(sample_beta_deg)
        else:
            sample_vehicle = vehicle
            reference_length = sample_vehicle.axle_span_mm() or 4360.0
            radius = beta_to_reference_radius_mm(math.radians(sample_beta_deg), reference_length)
            sample_ideal = solve_ideal_steering_from_radius(sample_vehicle, radius)
        sample_driver_point = driver_point_arc(
            context.baseline_rig.driver_arc_center,
            context.baseline_rig.driver_arc_radius_mm,
            math.radians(sample_beta_deg),
        )
        baseline_state = solve_planar_linkage(
            context.baseline_rig.spec,
            sample_driver_point,
            branch_hint=context.baseline_rig.branch_hint,
        )
        baseline_clearance = _build_clearance_report(sample_vehicle, context.baseline_rig.spec, baseline_state)

        driver_point = driver_point_arc(
            context.optimization_problem.base_rig.driver_arc_center,
            context.optimization_problem.base_rig.driver_arc_radius_mm,
            math.radians(sample_beta_deg),
        )
        try:
            optimized_state = solve_planar_linkage(optimized_spec, driver_point, branch_hint=optimized_hint)
            optimized_clearance = _build_clearance_report(sample_vehicle, optimized_spec, optimized_state)
        except Exception:
            optimized_state = None
            optimized_clearance = None

        ideal_front = max(sample_ideal.axles, key=lambda item: item.center.x_mm)
        ideal_rear = min(sample_ideal.axles, key=lambda item: item.center.x_mm)
        baseline_actual = solve_actual_steering(
            sample_vehicle,
            baseline_state,
            math.radians(sample_beta_deg),
            ideal_solution=sample_ideal,
        )
        baseline_comparison = compare_actual_to_ideal(
            baseline_actual,
            sample_ideal,
            vehicle=sample_vehicle,
            beta_rad=math.radians(sample_beta_deg),
        )
        optimized_actual = None
        optimized_comparison = None
        if optimized_state is not None:
            optimized_actual = solve_actual_steering(
                sample_vehicle,
                optimized_state,
                math.radians(sample_beta_deg),
                ideal_solution=sample_ideal,
            )
            optimized_comparison = compare_actual_to_ideal(
                optimized_actual,
                sample_ideal,
                vehicle=sample_vehicle,
                beta_rad=math.radians(sample_beta_deg),
            )
        ideal_wheel_angles = sample_ideal.wheel_steering_angles_deg()
        baseline_actual_angles = baseline_actual.wheel_steering_angles_deg()
        baseline_errors = baseline_comparison["wheel_errors_deg"]
        optimized_actual_angles = {} if optimized_actual is None else optimized_actual.wheel_steering_angles_deg()
        optimized_errors = {} if optimized_comparison is None else optimized_comparison["wheel_errors_deg"]
        baseline_sync_errors = baseline_comparison["synchronization_errors_deg"]
        optimized_sync_errors = {} if optimized_comparison is None else optimized_comparison["synchronization_errors_deg"]
        baseline_error = baseline_comparison["axle_center_errors_deg"].get(ideal_front.axle_id, 0.0)
        optimized_error = "" if optimized_comparison is None else f"{optimized_comparison['axle_center_errors_deg'].get(ideal_front.axle_id, 0.0):.2f}"
        dynamic_wheel_values = [
            value
            for wheel in context.vehicle.wheels()
            for value in (
                f"{ideal_wheel_angles.get(wheel.id, 0.0):.2f}",
                f"{baseline_actual_angles.get(wheel.id, 0.0):.2f}",
                "" if optimized_actual is None else f"{optimized_actual_angles.get(wheel.id, 0.0):.2f}",
                f"{baseline_errors.get(wheel.id, 0.0):.2f}",
                "" if optimized_actual is None else f"{optimized_errors.get(wheel.id, 0.0):.2f}",
            )
        ]
        dynamic_sync_values = [
            value
            for sync in context.vehicle.steering_synchronizations
            for value in (
                f"{baseline_sync_errors.get(sync.id, 0.0):.2f}",
                "" if optimized_comparison is None else f"{optimized_sync_errors.get(sync.id, 0.0):.2f}",
            )
        ]

        writer.writerow(
            [
                f"{sample_beta_deg:.1f}",
                f"{ideal_front.left_wheel.steering_angle_deg:.2f}",
                f"{ideal_front.right_wheel.steering_angle_deg:.2f}",
                f"{ideal_rear.left_wheel.steering_angle_deg:.2f}",
                f"{ideal_rear.right_wheel.steering_angle_deg:.2f}",
                *dynamic_wheel_values,
                *dynamic_sync_values,
                f"{baseline_state.steering_angle_deg:.2f}",
                f"{baseline_state.steering_angle_deg:.2f}",
                "" if baseline_state.companion_steering_angle_deg is None else f"{baseline_state.companion_steering_angle_deg:.2f}",
                "" if optimized_state is None else f"{optimized_state.steering_angle_deg:.2f}",
                "" if optimized_state is None else f"{optimized_state.steering_angle_deg:.2f}",
                "" if optimized_state is None or optimized_state.companion_steering_angle_deg is None else f"{optimized_state.companion_steering_angle_deg:.2f}",
                f"{baseline_error:.2f}",
                f"{baseline_errors.get(ideal_front.left_wheel.wheel_id, 0.0):.2f}",
                "" if baseline_state.companion_steering_angle_deg is None else f"{baseline_errors.get(ideal_front.right_wheel.wheel_id, 0.0):.2f}",
                optimized_error,
                "" if optimized_state is None else f"{optimized_errors.get(ideal_front.left_wheel.wheel_id, 0.0):.2f}",
                "" if optimized_state is None or optimized_state.companion_steering_angle_deg is None else f"{optimized_errors.get(ideal_front.right_wheel.wheel_id, 0.0):.2f}",
                "" if baseline_clearance.minimum_clearance_mm is None else f"{baseline_clearance.minimum_clearance_mm:.1f}",
                "" if optimized_clearance is None or optimized_clearance.minimum_clearance_mm is None else f"{optimized_clearance.minimum_clearance_mm:.1f}",
                "OK",
                "MECHANISM_INVALID" if optimized_state is None else "OK",
            ]
        )

    return output.getvalue()


def _dxf_float(value: float) -> str:
    return f"{value:.3f}"


def _dxf_entity_header(entity_type: str, layer: str) -> list[str]:
    return [
        "0",
        entity_type,
        "8",
        layer,
    ]


def _dxf_line(layer: str, start: Point2D, end: Point2D) -> list[str]:
    return [
        *_dxf_entity_header("LINE", layer),
        "10",
        _dxf_float(start.x_mm),
        "20",
        _dxf_float(start.y_mm),
        "30",
        "0.000",
        "11",
        _dxf_float(end.x_mm),
        "21",
        _dxf_float(end.y_mm),
        "31",
        "0.000",
    ]


def _dxf_circle(layer: str, center: Point2D, radius_mm: float) -> list[str]:
    return [
        *_dxf_entity_header("CIRCLE", layer),
        "10",
        _dxf_float(center.x_mm),
        "20",
        _dxf_float(center.y_mm),
        "30",
        "0.000",
        "40",
        _dxf_float(radius_mm),
    ]


def _dxf_text(layer: str, position: Point2D, text: str, height_mm: float) -> list[str]:
    return [
        *_dxf_entity_header("TEXT", layer),
        "10",
        _dxf_float(position.x_mm),
        "20",
        _dxf_float(position.y_mm),
        "30",
        "0.000",
        "40",
        _dxf_float(height_mm),
        "1",
        text,
        "50",
        "0.000",
    ]


def _dxf_lwpolyline(layer: str, points: Iterable[Point2D], closed: bool = False) -> list[str]:
    point_list = list(points)
    entity = [
        *_dxf_entity_header("LWPOLYLINE", layer),
        "90",
        str(len(point_list)),
        "70",
        "1" if closed else "0",
    ]
    for point in point_list:
        entity.extend(
            [
                "10",
                _dxf_float(point.x_mm),
                "20",
                _dxf_float(point.y_mm),
            ]
        )
    return entity


def _dxf_section(name: str, lines: list[str]) -> list[str]:
    return ["0", "SECTION", "2", name, *lines, "0", "ENDSEC"]


def _linkage_segments(
    state: PlanarLinkageState,
    spec: PlanarLinkageSpec,
) -> tuple[tuple[Point2D, Point2D], ...]:
    segments = [
        (state.driver_point, state.input_endpoint),
        (spec.bell_crank_pivot, state.input_endpoint),
        (spec.bell_crank_pivot, state.output_endpoint),
        (state.output_endpoint, state.steering_endpoint),
        (spec.steering_pivot, state.steering_endpoint),
    ]
    if state.companion_steering_endpoint is not None and spec.companion_steering_pivot is not None:
        segments.extend(
            [
                (state.steering_endpoint, state.companion_steering_endpoint),
                (spec.companion_steering_pivot, state.companion_steering_endpoint),
            ]
        )
    return tuple(segments)


def build_export_dxf(
    beta_deg: float,
    optimization_mode: str = "quick",
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> str:
    context = build_export_context(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    lines: list[str] = []

    layers = [
        ("ANNOTATION", 7),
        ("BODY", 4),
        ("AXLE", 2),
        ("IDEAL", 3),
        ("BASELINE", 8),
        ("OPTIMIZED", 1),
        ("PIVOT", 6),
        ("ICR", 5),
    ]

    lines.extend(["0", "SECTION", "2", "HEADER", "0", "ENDSEC"])
    layer_lines: list[str] = ["0", "TABLE", "2", "LAYER", "70", str(len(layers))]
    for layer_name, color in layers:
      layer_lines.extend(
            [
                "0",
                "LAYER",
                "2",
                layer_name,
                "70",
                "0",
                "62",
                str(color),
                "6",
                "CONTINUOUS",
            ]
        )
    layer_lines.extend(["0", "ENDTAB"])
    lines.extend(_dxf_section("TABLES", layer_lines))

    entities: list[str] = []
    entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 4300.0), "EasyTowing Engineering Sketch", 120.0))
    entities.extend(
        _dxf_text(
            "ANNOTATION",
            Point2D(-3800.0, 4120.0),
            f"Beta {beta_deg:.1f} deg | {optimization_mode.title()} optimization",
            72.0,
        )
    )
    entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3940.0), "DXF export of the current design state", 58.0))

    body_points = list(_vehicle_body_points(context.vehicle))
    entities.extend(_dxf_lwpolyline("BODY", body_points, closed=True))

    for axle in context.vehicle.axles:
        left, right = axle.outer_wheels()
        entities.extend(_dxf_line("AXLE", left.center, right.center))
        for wheel in axle.wheels():
            entities.extend(_dxf_circle("AXLE", wheel.center, 45.0))

    for axle in context.ideal_solution.axles:
        for wheel in axle.wheel_solutions:
            center = wheel.center
            end = Point2D(
                center.x_mm + math.cos(wheel.heading_rad) * 900.0,
                center.y_mm + math.sin(wheel.heading_rad) * 900.0,
            )
            entities.extend(_dxf_line("IDEAL", center, end))

    if context.ideal_solution.icr is not None:
        entities.extend(_dxf_circle("ICR", context.ideal_solution.icr, 85.0))

    entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3660.0), "Existing linkage", 58.0))
    for segment_start, segment_end in _linkage_segments(context.baseline_state, context.baseline_rig.spec):
        entities.extend(_dxf_line("BASELINE", segment_start, segment_end))
    entities.extend(_dxf_circle("BASELINE", context.baseline_rig.spec.bell_crank_pivot, 28.0))
    entities.extend(_dxf_circle("BASELINE", context.baseline_rig.spec.steering_pivot, 28.0))
    if context.baseline_rig.spec.companion_steering_pivot is not None:
        entities.extend(_dxf_circle("BASELINE", context.baseline_rig.spec.companion_steering_pivot, 28.0))

    if context.optimized_state is not None:
        entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3480.0), "Optimized linkage", 58.0))
        for segment_start, segment_end in _linkage_segments(context.optimized_state, context.optimized_spec):
            entities.extend(_dxf_line("OPTIMIZED", segment_start, segment_end))
        entities.extend(_dxf_circle("OPTIMIZED", context.optimized_spec.bell_crank_pivot, 28.0))
        entities.extend(_dxf_circle("OPTIMIZED", context.optimized_spec.steering_pivot, 28.0))
        if context.optimized_spec.companion_steering_pivot is not None:
            entities.extend(_dxf_circle("OPTIMIZED", context.optimized_spec.companion_steering_pivot, 28.0))
    else:
        entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3480.0), "Optimized linkage unavailable", 58.0))

    lines.extend(_dxf_section("ENTITIES", entities))
    lines.extend(["0", "EOF"])
    return "\n".join(lines)


def build_export_png(
    beta_deg: float,
    optimization_mode: str = "quick",
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> bytes:
    """Render a printable raster snapshot of the current engineering state."""

    context = build_export_context(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    image_width = 1600
    image_height = 1000
    plot_top = 120
    plot_bottom = 875
    plot_left = 70
    plot_right = 1530
    image = Image.new("RGB", (image_width, image_height), "#08111d")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    points: list[Point2D] = []
    for axle in context.vehicle.axles:
        points.append(axle.center)
        points.extend(wheel.center for wheel in axle.wheels())

    body_points = _vehicle_body_points(context.vehicle)
    points.extend(body_points)

    baseline_state = context.baseline_state
    baseline_points = [
        baseline_state.driver_point,
        baseline_state.input_endpoint,
        baseline_state.output_endpoint,
        baseline_state.steering_endpoint,
        context.baseline_rig.spec.bell_crank_pivot,
        context.baseline_rig.spec.steering_pivot,
    ]
    if baseline_state.companion_steering_endpoint is not None and context.baseline_rig.spec.companion_steering_pivot is not None:
        baseline_points.extend(
            [baseline_state.companion_steering_endpoint, context.baseline_rig.spec.companion_steering_pivot]
        )
    points.extend(baseline_points)
    if context.optimized_state is not None:
        points.extend(
            [
                context.optimized_state.driver_point,
                context.optimized_state.input_endpoint,
                context.optimized_state.output_endpoint,
                context.optimized_state.steering_endpoint,
                context.optimized_spec.bell_crank_pivot,
                context.optimized_spec.steering_pivot,
            ]
        )
        if context.optimized_state.companion_steering_endpoint is not None and context.optimized_spec.companion_steering_pivot is not None:
            points.extend(
                [
                    context.optimized_state.companion_steering_endpoint,
                    context.optimized_spec.companion_steering_pivot,
                ]
            )
    min_x = min(point.x_mm for point in points)
    max_x = max(point.x_mm for point in points)
    min_y = min(point.y_mm for point in points)
    max_y = max(point.y_mm for point in points)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min((plot_right - plot_left) / span_x, (plot_bottom - plot_top) / span_y)

    def screen(point: Point2D) -> tuple[int, int]:
        return (
            round(plot_left + (point.x_mm - min_x) * scale),
            round(plot_top + (max_y - point.y_mm) * scale),
        )

    def line(start: Point2D, end: Point2D, fill: str, width: int = 3) -> None:
        draw.line((*screen(start), *screen(end)), fill=fill, width=width)

    draw.text((50, 34), "EasyTowing Engineering Snapshot", fill="#e7eef7", font=font)
    draw.text(
        (50, 58),
        f"Beta {beta_deg:.1f} deg | {optimization_mode.title()} optimization | units: mm / deg",
        fill="#96a8be",
        font=font,
    )

    draw.polygon([screen(point) for point in body_points], fill="#0e2a3d", outline="#72e5ff", width=4)
    for axle in context.vehicle.axles:
        left_wheel, right_wheel = axle.outer_wheels()
        line(left_wheel.center, right_wheel.center, "#f4b860", 4)
        for wheel in axle.wheels():
            center_x, center_y = screen(wheel.center)
            radius = max(7, round(55.0 * scale))
            draw.ellipse(
                (center_x - radius, center_y - radius, center_x + radius, center_y + radius),
                fill="#13283a",
                outline="#e7eef7",
                width=2,
            )
        ideal = next(
            item
            for item in context.ideal_solution.axles
            if item.axle_id == axle.id
        )
        for wheel in ideal.wheel_solutions:
            end = Point2D(
                wheel.center.x_mm + math.cos(wheel.heading_rad) * 700.0,
                wheel.center.y_mm + math.sin(wheel.heading_rad) * 700.0,
            )
            line(wheel.center, end, "#ffd799", 2)

    for start, end in _linkage_segments(baseline_state, context.baseline_rig.spec):
        line(start, end, "#ff7d7d", 3)
    if context.optimized_state is not None:
        for start, end in _linkage_segments(context.optimized_state, context.optimized_spec):
            line(start, end, "#69d39d", 4)

    baseline_pivots = [context.baseline_rig.spec.bell_crank_pivot, context.baseline_rig.spec.steering_pivot]
    if context.baseline_rig.spec.companion_steering_pivot is not None:
        baseline_pivots.append(context.baseline_rig.spec.companion_steering_pivot)
    for pivot in baseline_pivots:
        x, y = screen(pivot)
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill="#f4b860", outline="#ffffff", width=2)
    optimized_pivots = [context.optimized_spec.bell_crank_pivot, context.optimized_spec.steering_pivot]
    if context.optimized_spec.companion_steering_pivot is not None:
        optimized_pivots.append(context.optimized_spec.companion_steering_pivot)
    for pivot in optimized_pivots:
        x, y = screen(pivot)
        draw.ellipse((x - 6, y - 6, x + 6, y + 6), fill="#69d39d", outline="#ffffff", width=1)

    icr_in_view = False
    if context.ideal_solution.icr is not None:
        icr_x, icr_y = screen(context.ideal_solution.icr)
        icr_in_view = plot_left <= icr_x <= plot_right and plot_top <= icr_y <= plot_bottom
        if icr_in_view:
            draw.ellipse((icr_x - 8, icr_y - 8, icr_x + 8, icr_y + 8), fill="#ff7d7d", outline="#ffffff", width=2)
            for axle in context.ideal_solution.axles:
                for wheel in axle.wheel_solutions:
                    line(wheel.center, context.ideal_solution.icr, "#ff7d7d", 1)

    if len(context.vehicle.axles) >= 2:
        first_axle = min(context.vehicle.axles, key=lambda axle: axle.center.x_mm)
        last_axle = max(context.vehicle.axles, key=lambda axle: axle.center.x_mm)
        dimension_y = max_y + 350.0
        body_top_y = max(point.y_mm for point in body_points)
        line(Point2D(first_axle.center.x_mm, body_top_y), Point2D(first_axle.center.x_mm, dimension_y), "#96a8be", 1)
        line(Point2D(last_axle.center.x_mm, body_top_y), Point2D(last_axle.center.x_mm, dimension_y), "#96a8be", 1)
        line(Point2D(first_axle.center.x_mm, dimension_y), Point2D(last_axle.center.x_mm, dimension_y), "#96a8be", 2)
        draw.text(
            (screen(Point2D((first_axle.center.x_mm + last_axle.center.x_mm) / 2.0, dimension_y))[0] - 45, screen(Point2D(0.0, dimension_y))[1] - 16),
            f"wheelbase {context.vehicle.axle_span_mm():.0f} mm",
            fill="#96a8be",
            font=font,
        )

    optimized = context.optimization_result.optimized_metrics
    optimized_clearance_text = (
        f"{optimized.minimum_clearance_mm:.1f} mm"
        if optimized.minimum_clearance_mm is not None
        else "n/a"
    )
    draw.rectangle((50, 900, 1530, 970), fill="#0e1d2e", outline="#2a4962", width=2)
    draw.text(
        (70, 920),
        f"Baseline RMS {context.optimization_result.baseline_metrics.rms_error_deg:.2f} deg | "
        f"Optimized RMS {optimized.rms_error_deg:.2f} deg | "
        f"Optimized clearance {optimized_clearance_text}",
        fill="#e7eef7",
        font=font,
    )
    if context.ideal_solution.icr is not None and not icr_in_view:
        draw.text(
            (1100, 58),
            f"ICR {context.ideal_solution.icr.x_mm:.0f}, {context.ideal_solution.icr.y_mm:.0f} mm (off-frame)",
            fill="#ff9d9d",
            font=font,
        )

    output = io.BytesIO()
    image.save(output, format="PNG")
    return output.getvalue()


@lru_cache(maxsize=16)
def build_steering_sweep_bundle(
    optimization_mode: str = "quick",
    step_deg: float = 1.0,
    beta_min_deg: float = -45.0,
    beta_max_deg: float = 45.0,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> dict[str, object]:
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")
    if beta_min_deg >= beta_max_deg:
        raise ValueError("beta_min_deg must be smaller than beta_max_deg")

    context = build_export_context(
        beta_deg=0.0,
        optimization_mode=optimization_mode,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    if max(abs(beta_min_deg), abs(beta_max_deg)) > context.vehicle.maximum_articulation_deg + 1e-9:
        raise ArticulationLimitExceededError(
            max(abs(beta_min_deg), abs(beta_max_deg)),
            context.vehicle.maximum_articulation_deg,
        )
    optimized_spec = context.optimized_spec
    optimized_hint = build_branch_hint(optimized_spec)

    sample_count = int(round((beta_max_deg - beta_min_deg) / step_deg)) + 1

    samples: list[dict[str, object]] = []
    baseline_errors: list[float] = []
    optimized_errors: list[float] = []
    baseline_synchronization_errors: list[float] = []
    optimized_synchronization_errors: list[float] = []
    baseline_clearances: list[float] = []
    optimized_clearances: list[float] = []
    baseline_clearance_cases: list[tuple[float, float]] = []
    optimized_clearance_cases: list[tuple[float, float]] = []
    baseline_valid_sample_count = 0
    optimized_valid_sample_count = 0

    for index in range(sample_count):
        beta_deg = beta_min_deg + index * step_deg
        if beta_deg > beta_max_deg + 1e-9:
            break

        if vehicle is None:
            sample_vehicle, ideal_solution, _ = build_demo_solution(beta_deg)
        else:
            sample_vehicle = vehicle
            reference_length = sample_vehicle.axle_span_mm() or 4360.0
            radius = beta_to_reference_radius_mm(math.radians(beta_deg), reference_length)
            ideal_solution = solve_ideal_steering_from_radius(sample_vehicle, radius)
        baseline_driver_point = driver_point_arc(
            context.baseline_rig.driver_arc_center,
            context.baseline_rig.driver_arc_radius_mm,
            math.radians(beta_deg),
        )
        baseline_state = solve_planar_linkage(
            context.baseline_rig.spec,
            baseline_driver_point,
            branch_hint=context.baseline_rig.branch_hint,
        )
        baseline_clearance = _build_clearance_report(sample_vehicle, context.baseline_rig.spec, baseline_state)

        driver_point = driver_point_arc(
            context.optimization_problem.base_rig.driver_arc_center,
            context.optimization_problem.base_rig.driver_arc_radius_mm,
            math.radians(beta_deg),
        )
        try:
            optimized_state = solve_planar_linkage(optimized_spec, driver_point, branch_hint=optimized_hint)
            optimized_clearance = _build_clearance_report(sample_vehicle, optimized_spec, optimized_state)
        except Exception:
            optimized_state = None
            optimized_clearance = None

        ideal_front = max(ideal_solution.axles, key=lambda item: item.center.x_mm)
        ideal_rear = min(ideal_solution.axles, key=lambda item: item.center.x_mm)
        baseline_actual = solve_actual_steering(
            sample_vehicle,
            baseline_state,
            math.radians(beta_deg),
            ideal_solution=ideal_solution,
        )
        baseline_error_map = actual_steering_errors_deg(baseline_actual, ideal_solution)
        baseline_comparison = compare_actual_to_ideal(
            baseline_actual,
            ideal_solution,
            vehicle=sample_vehicle,
            beta_rad=math.radians(beta_deg),
        )
        optimized_actual = None
        optimized_error_map: dict[str, float] = {}
        optimized_comparison: dict[str, object] | None = None
        if optimized_state is not None:
            optimized_actual = solve_actual_steering(
                sample_vehicle,
                optimized_state,
                math.radians(beta_deg),
                ideal_solution=ideal_solution,
            )
            optimized_error_map = actual_steering_errors_deg(optimized_actual, ideal_solution)
            optimized_comparison = compare_actual_to_ideal(
                optimized_actual,
                ideal_solution,
                vehicle=sample_vehicle,
                beta_rad=math.radians(beta_deg),
            )
        baseline_front_left_error = baseline_error_map.get(ideal_front.left_wheel.wheel_id)
        baseline_front_right_error = baseline_error_map.get(ideal_front.right_wheel.wheel_id)
        optimized_front_left_error = optimized_error_map.get(ideal_front.left_wheel.wheel_id)
        optimized_front_right_error = optimized_error_map.get(ideal_front.right_wheel.wheel_id)
        baseline_error = baseline_comparison["axle_center_errors_deg"].get(ideal_front.axle_id, 0.0)
        optimized_error = (
            None
            if optimized_comparison is None
            else optimized_comparison["axle_center_errors_deg"].get(ideal_front.axle_id, 0.0)
        )
        baseline_valid_sample_count += 1
        if optimized_actual is not None:
            optimized_valid_sample_count += 1

        samples.append(
            {
                "beta_deg": beta_deg,
                "ideal_front_left_deg": ideal_front.left_wheel.steering_angle_deg,
                "ideal_front_right_deg": ideal_front.right_wheel.steering_angle_deg,
                "ideal_rear_left_deg": ideal_rear.left_wheel.steering_angle_deg,
                "ideal_rear_right_deg": ideal_rear.right_wheel.steering_angle_deg,
                "baseline_steer_deg": baseline_state.steering_angle_deg,
                "baseline_front_left_deg": baseline_state.steering_angle_deg,
                "baseline_front_right_deg": baseline_state.companion_steering_angle_deg,
                "optimized_steer_deg": None if optimized_state is None else optimized_state.steering_angle_deg,
                "optimized_front_left_deg": None if optimized_state is None else optimized_state.steering_angle_deg,
                "optimized_front_right_deg": None if optimized_state is None else optimized_state.companion_steering_angle_deg,
                "baseline_error_deg": baseline_error,
                "baseline_front_left_error_deg": baseline_front_left_error,
                "baseline_front_right_error_deg": baseline_front_right_error,
                "optimized_error_deg": optimized_error,
                "optimized_front_left_error_deg": optimized_front_left_error,
                "optimized_front_right_error_deg": optimized_front_right_error,
                "ideal_wheel_angles_deg": ideal_solution.wheel_steering_angles_deg(),
                "baseline_actual_wheel_angles_deg": baseline_actual.wheel_steering_angles_deg(),
                "baseline_wheel_errors_deg": baseline_error_map,
                "ideal_axle_center_angles_deg": ideal_solution.axle_center_steering_angles_deg(),
                "baseline_actual_axle_center_angles_deg": baseline_actual.axle_center_steering_angles_deg(),
                "baseline_axle_center_errors_deg": baseline_comparison["axle_center_errors_deg"],
                "optimized_actual_wheel_angles_deg": None
                if optimized_actual is None
                else optimized_actual.wheel_steering_angles_deg(),
                "optimized_wheel_errors_deg": None if optimized_actual is None else optimized_error_map,
                "optimized_actual_axle_center_angles_deg": None
                if optimized_actual is None
                else optimized_actual.axle_center_steering_angles_deg(),
                "optimized_axle_center_errors_deg": None
                if optimized_comparison is None
                else optimized_comparison["axle_center_errors_deg"],
                "baseline_synchronization_error_deg": baseline_comparison[
                    "front_rear_synchronization_error_deg"
                ],
                "baseline_synchronization_errors_deg": baseline_comparison[
                    "synchronization_errors_deg"
                ],
                "optimized_synchronization_error_deg": None
                if optimized_comparison is None
                else optimized_comparison["front_rear_synchronization_error_deg"],
                "optimized_synchronization_errors_deg": None
                if optimized_comparison is None
                else optimized_comparison["synchronization_errors_deg"],
                "baseline_clearance_mm": baseline_clearance.minimum_clearance_mm,
                "optimized_clearance_mm": None if optimized_clearance is None else optimized_clearance.minimum_clearance_mm,
                "baseline_status": "OK",
                "optimized_status": "MECHANISM_INVALID" if optimized_state is None else "OK",
            }
        )

        baseline_errors.extend(baseline_error_map.values())
        baseline_channel_errors = baseline_comparison["synchronization_errors_deg"]
        if baseline_channel_errors:
            baseline_synchronization_errors.extend(
                float(error) for error in baseline_channel_errors.values()
            )
        else:
            baseline_sync_error = baseline_comparison["front_rear_synchronization_error_deg"]
            if baseline_sync_error is not None:
                baseline_synchronization_errors.append(float(baseline_sync_error))
        baseline_clearance_value = baseline_clearance.minimum_clearance_mm
        if baseline_clearance_value is not None:
            baseline_clearances.append(baseline_clearance_value)
            baseline_clearance_cases.append((baseline_clearance_value, beta_deg))
        optimized_errors.extend(optimized_error_map.values())
        if optimized_comparison is not None:
            optimized_channel_errors = optimized_comparison["synchronization_errors_deg"]
            if optimized_channel_errors:
                optimized_synchronization_errors.extend(
                    float(error) for error in optimized_channel_errors.values()
                )
            else:
                optimized_sync_error = optimized_comparison["front_rear_synchronization_error_deg"]
                if optimized_sync_error is not None:
                    optimized_synchronization_errors.append(float(optimized_sync_error))
        optimized_clearance_value = None if optimized_clearance is None else optimized_clearance.minimum_clearance_mm
        if optimized_clearance_value is not None:
            optimized_clearances.append(optimized_clearance_value)
            optimized_clearance_cases.append((optimized_clearance_value, beta_deg))

    def _rms(values: list[float]) -> float | None:
        if not values:
            return None
        return math.sqrt(sum(value * value for value in values) / len(values))

    def _mean_abs(values: list[float]) -> float | None:
        if not values:
            return None
        return sum(abs(value) for value in values) / len(values)

    def _max_abs(values: list[float]) -> float | None:
        if not values:
            return None
        return max(abs(value) for value in values)

    return {
        "mode": optimization_mode,
        "step_deg": step_deg,
        "beta_min_deg": beta_min_deg,
        "beta_max_deg": beta_max_deg,
        "sample_count": len(samples),
        "samples": samples,
        "summary": {
            "baseline_rms_error_deg": _rms(baseline_errors),
            "baseline_mean_abs_error_deg": _mean_abs(baseline_errors),
            "baseline_max_abs_error_deg": _max_abs(baseline_errors),
            "optimized_rms_error_deg": _rms(optimized_errors),
            "optimized_mean_abs_error_deg": _mean_abs(optimized_errors),
            "optimized_max_abs_error_deg": _max_abs(optimized_errors),
            "baseline_max_abs_synchronization_error_deg": _max_abs(baseline_synchronization_errors),
            "optimized_max_abs_synchronization_error_deg": _max_abs(optimized_synchronization_errors),
            "baseline_min_clearance_mm": None if not baseline_clearances else min(baseline_clearances),
            "optimized_min_clearance_mm": None if not optimized_clearances else min(optimized_clearances),
            "baseline_min_clearance_beta_deg": None if not baseline_clearance_cases else min(baseline_clearance_cases)[1],
            "optimized_min_clearance_beta_deg": None if not optimized_clearance_cases else min(optimized_clearance_cases)[1],
            "baseline_valid_samples": baseline_valid_sample_count,
            "optimized_valid_samples": optimized_valid_sample_count,
        },
    }


def build_steering_curves_svg(
    current_beta_deg: float,
    optimization_mode: str = "quick",
    step_deg: float = 1.0,
    beta_min_deg: float = -45.0,
    beta_max_deg: float = 45.0,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> str:
    sweep = build_steering_sweep_bundle(
        optimization_mode=optimization_mode,
        step_deg=step_deg,
        beta_min_deg=beta_min_deg,
        beta_max_deg=beta_max_deg,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    samples = sweep["samples"]
    summary = sweep["summary"]
    beta_min_deg = float(sweep["beta_min_deg"])
    beta_max_deg = float(sweep["beta_max_deg"])
    current_beta_deg = max(beta_min_deg, min(beta_max_deg, current_beta_deg))

    width = 1200
    height = 940
    panel_x = 60
    panel_w = 1080
    top_panel_y = 170
    top_panel_h = 330
    bottom_panel_y = 550
    bottom_panel_h = 280
    plot_left = 110
    plot_w = 980

    top_y_min = -45.0
    top_y_max = 45.0
    top_plot_y = top_panel_y + 80
    top_plot_h = 150
    bottom_y_extent = max(
        5.0,
        float(summary["baseline_max_abs_error_deg"] or 0.0),
        float(summary["optimized_max_abs_error_deg"] or 0.0),
        float(summary["baseline_max_abs_synchronization_error_deg"] or 0.0),
        float(summary["optimized_max_abs_synchronization_error_deg"] or 0.0),
    ) + 2.0
    bottom_y_min = -bottom_y_extent
    bottom_y_max = bottom_y_extent
    bottom_plot_y = bottom_panel_y + 80
    bottom_plot_h = 120

    def scale_x(beta_deg: float) -> float:
        return plot_left + ((beta_deg - beta_min_deg) / (beta_max_deg - beta_min_deg)) * plot_w

    def scale_y(value: float, minimum: float, maximum: float, top: float, plot_height: float) -> float:
        return top + (1.0 - ((value - minimum) / (maximum - minimum))) * plot_height

    def path_for(key: str, minimum: float, maximum: float, top: float, plot_height: float) -> str:
        points: list[str] = []
        for sample in samples:
            value = sample[key]
            if value is None:
                continue
            x = scale_x(float(sample["beta_deg"]))
            y = scale_y(float(value), minimum, maximum, top, plot_height)
            points.append(f"{x:.1f},{y:.1f}")
        if len(points) < 2:
            return ""
        return "M " + " L ".join(points)

    def path_for_nested(
        container_key: str,
        nested_key: str,
        minimum: float,
        maximum: float,
        top: float,
        plot_height: float,
    ) -> str:
        points: list[str] = []
        for sample in samples:
            values = sample.get(container_key) or {}
            value = values.get(nested_key)
            if value is None:
                continue
            x = scale_x(float(sample["beta_deg"]))
            y = scale_y(float(value), minimum, maximum, top, plot_height)
            points.append(f"{x:.1f},{y:.1f}")
        if len(points) < 2:
            return ""
        return "M " + " L ".join(points)

    def line_path(x1: float, y1: float, x2: float, y2: float) -> str:
        return f"M {x1:.1f},{y1:.1f} L {x2:.1f},{y2:.1f}"

    def render_grid(minimum: float, maximum: float, top: float, plot_height: float, tick_step: float = 15.0) -> list[str]:
        pieces: list[str] = []
        value = math.ceil(minimum / tick_step) * tick_step
        while value <= maximum + 1e-9:
            y = scale_y(value, minimum, maximum, top, plot_height)
            pieces.append(f'<line x1="{plot_left:.1f}" y1="{y:.1f}" x2="{plot_left + plot_w:.1f}" y2="{y:.1f}" class="curve-grid" />')
            pieces.append(_svg_text(plot_left - 46.0, y + 8.0, f"{value:.0f}", "curve-axis-label"))
            value += tick_step

        for beta_tick in range(int(beta_min_deg), int(beta_max_deg) + 1, 15):
            x = scale_x(float(beta_tick))
            pieces.append(f'<line x1="{x:.1f}" y1="{top:.1f}" x2="{x:.1f}" y2="{top + plot_height:.1f}" class="curve-grid" />')
            pieces.append(_svg_text(x - 18.0, top + plot_height + 34.0, f"{beta_tick}", "curve-axis-label"))

        y_zero = scale_y(0.0, minimum, maximum, top, plot_height)
        x_zero = scale_x(0.0)
        pieces.append(f'<line x1="{plot_left:.1f}" y1="{y_zero:.1f}" x2="{plot_left + plot_w:.1f}" y2="{y_zero:.1f}" class="curve-axis" />')
        pieces.append(f'<line x1="{x_zero:.1f}" y1="{top:.1f}" x2="{x_zero:.1f}" y2="{top + plot_height:.1f}" class="curve-axis" />')
        return pieces

    def render_series(path_data: str, stroke: str, width_value: float = 4.0, dash: str | None = None, opacity: float = 1.0) -> str:
        if not path_data:
            return ""
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return (
            f'<path d="{path_data}" fill="none" stroke="{stroke}" stroke-width="{width_value:.1f}" '
            f'stroke-linecap="round" stroke-linejoin="round" opacity="{opacity:.2f}"{dash_attr} />'
        )

    def render_legend(items: list[tuple[str, str, str | None]], start_x: float, start_y: float, columns: int) -> list[str]:
        pieces: list[str] = []
        column_width = 340.0
        row_height = 30.0
        for index, (label, stroke, dash) in enumerate(items):
            row = index // columns
            column = index % columns
            x = start_x + column * column_width
            y = start_y + row * row_height
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            pieces.append(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{x + 26.0:.1f}" y2="{y:.1f}" stroke="{stroke}" stroke-width="5" stroke-linecap="round"{dash_attr} />')
            pieces.append(_svg_text(x + 38.0, y + 6.0, label, "curve-legend"))
        return pieces

    current_x = scale_x(current_beta_deg)
    axle_ids = list((samples[0].get("ideal_axle_center_angles_deg") or {}).keys()) if samples else []

    styles = """
      .canvas { fill: #08111d; }
      .panel { fill: rgba(11, 22, 36, 0.96); stroke: rgba(138,171,204,0.25); stroke-width: 3; }
      .title { fill: #e7eef7; font-size: 46px; font-family: "Bahnschrift", "Segoe UI", sans-serif; font-weight: 700; }
      .subtitle { fill: #96a8be; font-size: 28px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .summary { fill: #72e5ff; font-size: 26px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .curve-grid { stroke: rgba(255,255,255,0.08); stroke-width: 1.5; }
      .curve-axis { stroke: rgba(255,255,255,0.42); stroke-width: 2.5; }
      .curve-axis-label { fill: #96a8be; font-size: 22px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .curve-legend { fill: #e7eef7; font-size: 22px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .marker { stroke: rgba(244,184,96,0.85); stroke-width: 3.5; stroke-dasharray: 14 10; }
      .marker-label { fill: #f4b860; font-size: 24px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
    """

    def summary_value(value: float | None) -> str:
        return "n/a" if value is None else f"{value:.2f}"

    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-label="Steering curves">',
        f"<style>{styles}</style>",
        f'<rect x="0" y="0" width="{width}" height="{height}" class="canvas" />',
        _svg_text(56.0, 92.0, f"Beta sweep {beta_min_deg:.0f} to {beta_max_deg:.0f} deg | {step_deg:.1f} deg step | {optimization_mode.title()} optimization", "subtitle"),
        _svg_text(
            56.0,
            128.0,
            f"Baseline RMS {summary_value(summary['baseline_rms_error_deg'])} deg | Optimized RMS {summary_value(summary['optimized_rms_error_deg'])} deg | Current beta {current_beta_deg:.1f} deg",
            "summary",
        ),
    ]

    # Top panel: steering angle curves.
    pieces.append(f'<rect x="{panel_x}" y="{top_panel_y}" width="{panel_w}" height="{top_panel_h}" rx="28" class="panel" />')
    pieces.append(_svg_text(panel_x + 26.0, top_panel_y + 38.0, "Wheel and linkage angles", "title"))
    channel_note = "" if not axle_ids else f" Axle center channels: {', '.join(axle_ids)}."
    pieces.append(_svg_text(panel_x + 26.0, top_panel_y + 74.0, f"Solid curves show ideal wheel headings; dashed channels show actual responses.{channel_note}", "subtitle"))
    pieces.extend(render_grid(top_y_min, top_y_max, top_plot_y, top_plot_h))

    top_series = [
        ("ideal_front_left_deg", "#f4b860", None, 4.0),
        ("ideal_front_right_deg", "#ffd799", "14 10", 4.0),
        ("ideal_rear_left_deg", "#72e5ff", None, 4.0),
        ("ideal_rear_right_deg", "#9db4ff", "14 10", 4.0),
        ("baseline_steer_deg", "#ff7d7d", None, 4.0),
        ("optimized_steer_deg", "#69d39d", None, 5.0),
    ]
    for key, stroke, dash, width_value in top_series:
        pieces.append(render_series(path_for(key, top_y_min, top_y_max, top_plot_y, top_plot_h), stroke, width_value, dash))
    axle_palette = ["#e5a6ff", "#a6f3dc", "#ffadad", "#b8c7ff", "#f7df8d", "#a9d9ff"]
    for index, axle_id in enumerate(axle_ids):
        color = axle_palette[index % len(axle_palette)]
        pieces.append(render_series(
            path_for_nested("ideal_axle_center_angles_deg", axle_id, top_y_min, top_y_max, top_plot_y, top_plot_h),
            color,
            2.5,
            "6 8",
            0.65,
        ))
        pieces.append(render_series(
            path_for_nested("baseline_actual_axle_center_angles_deg", axle_id, top_y_min, top_y_max, top_plot_y, top_plot_h),
            "#ff9d56",
            2.0,
            "3 8",
            0.6,
        ))
        pieces.append(render_series(
            path_for_nested("optimized_actual_axle_center_angles_deg", axle_id, top_y_min, top_y_max, top_plot_y, top_plot_h),
            "#8ef2c2",
            2.0,
            "3 8",
            0.6,
        ))
    pieces.append(f'<line x1="{current_x:.1f}" y1="{top_plot_y:.1f}" x2="{current_x:.1f}" y2="{top_plot_y + top_plot_h:.1f}" class="marker" />')
    pieces.append(_svg_text(current_x + 12.0, top_plot_y + 22.0, f"beta {current_beta_deg:.0f}", "marker-label"))
    pieces.extend(render_legend([
        ("Ideal front left", "#f4b860", None),
        ("Ideal front right", "#ffd799", "14 10"),
        ("Ideal rear left", "#72e5ff", None),
        ("Ideal rear right", "#9db4ff", "14 10"),
        ("Baseline steer", "#ff7d7d", None),
        ("Optimized steer", "#69d39d", None),
    ], panel_x + 26.0, top_panel_y + top_panel_h - 34.0, 3))

    # Bottom panel: error curves.
    pieces.append(f'<rect x="{panel_x}" y="{bottom_panel_y}" width="{panel_w}" height="{bottom_panel_h}" rx="28" class="panel" />')
    pieces.append(_svg_text(panel_x + 26.0, bottom_panel_y + 38.0, "Front-axle steering error + synchronization", "title"))
    pieces.append(_svg_text(panel_x + 26.0, bottom_panel_y + 74.0, "Front-axle error plus front/rear synchronization error; full wheel and axle errors are in JSON and CSV.", "subtitle"))
    pieces.extend(render_grid(bottom_y_min, bottom_y_max, bottom_plot_y, bottom_plot_h, tick_step=max(1.0, round(bottom_y_extent / 3.0))))
    for key, stroke, dash, width_value in [
        ("baseline_error_deg", "#ff9d56", "10 8", 4.0),
        ("optimized_error_deg", "#8ef2c2", None, 5.0),
        ("baseline_synchronization_error_deg", "#f4b860", "4 8", 3.0),
        ("optimized_synchronization_error_deg", "#72e5ff", "4 8", 3.0),
    ]:
        pieces.append(render_series(path_for(key, bottom_y_min, bottom_y_max, bottom_plot_y, bottom_plot_h), stroke, width_value, dash))
    pieces.append(f'<line x1="{current_x:.1f}" y1="{bottom_plot_y:.1f}" x2="{current_x:.1f}" y2="{bottom_plot_y + bottom_plot_h:.1f}" class="marker" />')
    pieces.append(_svg_text(current_x + 12.0, bottom_plot_y + 20.0, f"beta {current_beta_deg:.0f}", "marker-label"))
    pieces.extend(render_legend([
        ("Baseline error", "#ff9d56", "10 8"),
        ("Optimized error", "#8ef2c2", None),
    ], panel_x + 26.0, bottom_panel_y + bottom_panel_h - 34.0, 2))

    pieces.append("</svg>")
    return "".join(pieces)


def _transform_points(points: Iterable[Point2D], origin: Point2D, heading_rad: float) -> tuple[Point2D, ...]:
    return tuple(origin + point.rotated_ccw(heading_rad) for point in points)


def _convex_hull(points: Iterable[Point2D]) -> tuple[Point2D, ...]:
    unique_points = list({point.to_tuple(): point for point in points}.values())
    if len(unique_points) <= 1:
        return tuple(unique_points)

    ordered = sorted(unique_points, key=lambda point: (point.x_mm, point.y_mm))

    def cross(origin: Point2D, a: Point2D, b: Point2D) -> float:
        return (a.x_mm - origin.x_mm) * (b.y_mm - origin.y_mm) - (a.y_mm - origin.y_mm) * (b.x_mm - origin.x_mm)

    lower: list[Point2D] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)

    upper: list[Point2D] = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)

    hull = lower[:-1] + upper[:-1]
    return tuple(hull)


def _swept_path_pose(vehicle: VehicleLayout, beta_deg: float, reference_length_mm: float) -> dict[str, object]:
    beta_rad = math.radians(beta_deg)
    radius_mm = beta_to_reference_radius_mm(beta_rad, reference_length_mm)
    if radius_mm is None:
        origin = Point2D(reference_length_mm, 0.0)
        heading_rad = 0.0
    else:
        travel_angle_rad = abs(beta_rad)
        direction = 1.0 if radius_mm > 0 else -1.0
        radius_abs = abs(radius_mm)
        origin = Point2D(
            reference_length_mm + radius_abs * math.sin(travel_angle_rad),
            direction * radius_abs * (1.0 - math.cos(travel_angle_rad)),
        )
        heading_rad = direction * travel_angle_rad

    local_body = _vehicle_local_body_points(vehicle)
    body_origin = origin + vehicle.origin.rotated_ccw(heading_rad)
    body_outline = _transform_points(local_body, body_origin, heading_rad)

    wheel_centers = [
        {
            "wheel_id": wheel.id,
            "axle_id": wheel.axle_id,
            "side": wheel.side,
            "tire_width_mm": wheel.tire_width_mm,
            "outside_diameter_mm": wheel.outside_diameter_mm,
            "radius_mm": wheel.outside_diameter_mm / 2.0,
            "point": _point_payload(origin + wheel.center.rotated_ccw(heading_rad)),
        }
        for wheel in vehicle.wheels()
    ]

    axle_centers = [
        {
            "axle_id": axle.id,
            "point": _point_payload(origin + axle.center.rotated_ccw(heading_rad)),
        }
        for axle in vehicle.axles
    ]

    return {
        "beta_deg": beta_deg,
        "radius_mm": radius_mm,
        "origin": _point_payload(origin),
        "heading_rad": heading_rad,
        "heading_deg": math.degrees(heading_rad),
        "body_outline": [_point_payload(point) for point in body_outline],
        "wheel_centers": wheel_centers,
        "axle_centers": axle_centers,
    }


@lru_cache(maxsize=16)
def build_swept_path_bundle(
    current_beta_deg: float,
    optimization_mode: str = "quick",
    step_deg: float = 1.0,
    beta_min_deg: float = -45.0,
    beta_max_deg: float = 45.0,
    vehicle: VehicleLayout | None = None,
) -> dict[str, object]:
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")
    if beta_min_deg >= beta_max_deg:
        raise ValueError("beta_min_deg must be smaller than beta_max_deg")

    active_vehicle = vehicle or build_reference_demo_layout()
    if max(abs(beta_min_deg), abs(beta_max_deg), abs(current_beta_deg)) > active_vehicle.maximum_articulation_deg + 1e-9:
        raise ArticulationLimitExceededError(
            max(abs(beta_min_deg), abs(beta_max_deg), abs(current_beta_deg)),
            active_vehicle.maximum_articulation_deg,
        )
    reference_length_mm = active_vehicle.axle_span_mm() or 4360.0
    sample_count = int(round((beta_max_deg - beta_min_deg) / step_deg)) + 1

    samples: list[dict[str, object]] = []
    left_points: list[Point2D] = []
    right_points: list[Point2D] = []
    all_points: list[Point2D] = []

    def pose_envelope_points(pose: dict[str, object]) -> list[Point2D]:
        points = [
            Point2D(point["x_mm"], point["y_mm"])
            for point in pose["body_outline"]
        ]
        for wheel in pose["wheel_centers"]:
            center = Point2D(wheel["point"]["x_mm"], wheel["point"]["y_mm"])
            radius = float(wheel.get("radius_mm", 0.0))
            if radius > 0.0:
                points.extend(
                    (
                        center + Point2D(radius, 0.0),
                        center + Point2D(-radius, 0.0),
                        center + Point2D(0.0, radius),
                        center + Point2D(0.0, -radius),
                    )
                )
        return points

    for index in range(sample_count):
        beta_deg = beta_min_deg + index * step_deg
        if beta_deg > beta_max_deg + 1e-9:
            break
        pose = _swept_path_pose(active_vehicle, beta_deg, reference_length_mm)
        samples.append(pose)
        envelope_points = pose_envelope_points(pose)
        all_points.extend(envelope_points)
        if beta_deg >= 0:
            left_points.extend(envelope_points)
        if beta_deg <= 0:
            right_points.extend(envelope_points)

    current_pose = _swept_path_pose(active_vehicle, current_beta_deg, reference_length_mm)
    current_points = tuple(pose_envelope_points(current_pose))
    all_points.extend(current_points)

    if not all_points:
        raise ValueError("No swept-path samples were generated.")

    min_x = min(point.x_mm for point in all_points)
    max_x = max(point.x_mm for point in all_points)
    min_y = min(point.y_mm for point in all_points)
    max_y = max(point.y_mm for point in all_points)

    def _width(points: list[Point2D]) -> float | None:
        if not points:
            return None
        return max(point.y_mm for point in points) - min(point.y_mm for point in points)

    left_hull = _convex_hull(left_points)
    right_hull = _convex_hull(right_points)
    current_hull = _convex_hull(current_points)

    return {
        "mode": optimization_mode,
        "step_deg": step_deg,
        "beta_min_deg": beta_min_deg,
        "beta_max_deg": beta_max_deg,
        "sample_count": len(samples),
        "vehicle": {
            "id": active_vehicle.id,
            "name": active_vehicle.name,
            "body_length_mm": active_vehicle.body_length_mm,
            "body_width_mm": active_vehicle.body_width_mm,
        },
        "samples": samples,
        "current_pose": current_pose,
        "envelopes": {
            "left_turn": [_point_payload(point) for point in left_hull],
            "right_turn": [_point_payload(point) for point in right_hull],
            "current": [_point_payload(point) for point in current_hull],
        },
        "metrics": {
            "swept_width_mm": max_y - min_y,
            "swept_length_mm": max_x - min_x,
            "min_x_mm": min_x,
            "max_x_mm": max_x,
            "min_y_mm": min_y,
            "max_y_mm": max_y,
            "left_turn_width_mm": _width(left_points),
            "right_turn_width_mm": _width(right_points),
        },
    }


def build_swept_path_svg(
    current_beta_deg: float,
    optimization_mode: str = "quick",
    step_deg: float = 1.0,
    beta_min_deg: float = -45.0,
    beta_max_deg: float = 45.0,
    vehicle: VehicleLayout | None = None,
) -> str:
    bundle = build_swept_path_bundle(
        current_beta_deg=current_beta_deg,
        optimization_mode=optimization_mode,
        step_deg=step_deg,
        beta_min_deg=beta_min_deg,
        beta_max_deg=beta_max_deg,
        vehicle=vehicle,
    )
    active_vehicle = vehicle or build_reference_demo_layout()
    left_hull = [Point2D(point["x_mm"], point["y_mm"]) for point in bundle["envelopes"]["left_turn"]]
    right_hull = [Point2D(point["x_mm"], point["y_mm"]) for point in bundle["envelopes"]["right_turn"]]
    current_outline = [Point2D(point["x_mm"], point["y_mm"]) for point in bundle["current_pose"]["body_outline"]]
    current_origin = Point2D(bundle["current_pose"]["origin"]["x_mm"], bundle["current_pose"]["origin"]["y_mm"])
    current_heading_deg = float(bundle["current_pose"]["heading_deg"])
    samples = bundle["samples"]

    # Points used to define the view box.
    render_points: list[Point2D] = []
    for pose in samples:
        render_points.append(Point2D(pose["origin"]["x_mm"], pose["origin"]["y_mm"]))
        render_points.extend(Point2D(point["x_mm"], point["y_mm"]) for point in pose["body_outline"])
        render_points.extend(Point2D(point["point"]["x_mm"], point["point"]["y_mm"]) for point in pose["wheel_centers"])
    render_points.extend(left_hull)
    render_points.extend(right_hull)
    render_points.extend(current_outline)

    min_x = min(point.x_mm for point in render_points)
    max_x = max(point.x_mm for point in render_points)
    min_y = min(point.y_mm for point in render_points)
    max_y = max(point.y_mm for point in render_points)

    width = 1500
    height = 1060
    margin_x = 70
    plot_top = 190
    plot_bottom = 820
    plot_width = width - 2 * margin_x
    plot_height = plot_bottom - plot_top
    world_width = max(max_x - min_x, 1.0)
    world_height = max(max_y - min_y, 1.0)
    scale = min(plot_width / world_width, plot_height / world_height)
    pad_x = (plot_width - world_width * scale) / 2.0
    pad_y = (plot_height - world_height * scale) / 2.0

    def project(point: Point2D) -> tuple[float, float]:
        x = margin_x + pad_x + (point.x_mm - min_x) * scale
        y = plot_top + pad_y + (max_y - point.y_mm) * scale
        return x, y

    def points_to_path(points: Iterable[Point2D], close_path: bool = False) -> str:
        point_list = list(points)
        if not point_list:
            return ""
        commands: list[str] = []
        for index, point in enumerate(point_list):
            x, y = project(point)
            commands.append(f"{'M' if index == 0 else 'L'} {x:.1f},{y:.1f}")
        if close_path:
            commands.append("Z")
        return " ".join(commands)

    def polyline_path(points: Iterable[Point2D]) -> str:
        return points_to_path(points, close_path=False)

    def polygon_path(points: Iterable[Point2D]) -> str:
        return points_to_path(points, close_path=True)

    def render_path(points: Iterable[Point2D], stroke: str, fill: str = "none", width_value: float = 4.0, opacity: float = 1.0, dash: str | None = None, close_path: bool = False) -> str:
        path_data = points_to_path(points, close_path=close_path)
        if not path_data:
            return ""
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return (
            f'<path d="{path_data}" fill="{fill}" stroke="{stroke}" stroke-width="{width_value:.1f}" '
            f'stroke-linecap="round" stroke-linejoin="round" opacity="{opacity:.2f}"{dash_attr} />'
        )

    def render_circle(point: Point2D, radius_mm: float, class_name: str) -> str:
        x, y = project(point)
        radius = radius_mm * scale
        return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{max(radius, 2.0):.1f}" class="{class_name}" />'

    def render_text(point: Point2D, text: str, class_name: str) -> str:
        x, y = project(point)
        return _svg_text(x, y, text, class_name)

    def hull_text(label: str, hull: list[Point2D], color: str, offset_y: float) -> str:
        if not hull:
            return ""
        centroid = Point2D(
            sum(point.x_mm for point in hull) / len(hull),
            sum(point.y_mm for point in hull) / len(hull),
        )
        x, y = project(centroid)
        return _svg_text(x + 16.0, y + offset_y, label, "sweep-label")

    left_pose_points = [Point2D(point["x_mm"], point["y_mm"]) for point in bundle["current_pose"]["body_outline"]]
    left_sweep_width = bundle["metrics"]["left_turn_width_mm"]
    right_sweep_width = bundle["metrics"]["right_turn_width_mm"]

    styles = """
      .canvas { fill: #08111d; }
      .panel { fill: rgba(11, 22, 36, 0.96); stroke: rgba(138,171,204,0.25); stroke-width: 3; }
      .title { fill: #e7eef7; font-size: 44px; font-family: "Bahnschrift", "Segoe UI", sans-serif; font-weight: 700; }
      .subtitle { fill: #96a8be; font-size: 26px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .summary { fill: #72e5ff; font-size: 24px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .sweep-grid { stroke: rgba(255,255,255,0.08); stroke-width: 1.5; }
      .sweep-axis { stroke: rgba(255,255,255,0.42); stroke-width: 2.5; }
      .sweep-label { fill: #96a8be; font-size: 22px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .sweep-left { fill: rgba(114,229,255,0.10); stroke: rgba(114,229,255,0.78); stroke-width: 4; }
      .sweep-right { fill: rgba(244,184,96,0.10); stroke: rgba(244,184,96,0.78); stroke-width: 4; }
      .sweep-current { fill: rgba(255,255,255,0.04); stroke: rgba(255,255,255,0.95); stroke-width: 5; }
      .sweep-wheel { fill: rgba(255,255,255,0.92); stroke: rgba(8,17,29,0.92); stroke-width: 4; }
      .sweep-track { stroke: rgba(255,255,255,0.22); stroke-width: 3.5; stroke-dasharray: 12 10; fill: none; }
      .sweep-current-origin { fill: #72e5ff; stroke: #08111d; stroke-width: 4; }
      .sweep-legend { fill: #e7eef7; font-size: 22px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .sweep-value { fill: #f4b860; font-size: 22px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
    """

    # Simple grid and axis at the geometric center of the rendered points.
    axis_x = width / 2.0
    axis_y = (plot_top + plot_bottom) / 2.0

    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-label="Swept path preview">',
        f"<style>{styles}</style>",
        f'<rect x="0" y="0" width="{width}" height="{height}" class="canvas" />',
        _svg_text(56.0, 72.0, "Swept path preview", "title"),
        _svg_text(56.0, 106.0, f"Articulation sweep -45 to 45 deg | {step_deg:.1f} deg step | {optimization_mode.title()} optimization", "subtitle"),
        _svg_text(
            56.0,
            140.0,
            f"Swept width {bundle['metrics']['swept_width_mm']:.1f} mm | Left span {left_sweep_width:.1f} mm | Right span {right_sweep_width:.1f} mm",
            "summary",
        ),
    ]

    for x in range(0, width + 1, 150):
        pieces.append(f'<line x1="{x}" y1="{plot_top:.1f}" x2="{x}" y2="{plot_bottom:.1f}" class="sweep-grid" />')
    for y in range(int(plot_top), int(plot_bottom) + 1, 120):
        pieces.append(f'<line x1="{margin_x:.1f}" y1="{y:.1f}" x2="{width - margin_x:.1f}" y2="{y:.1f}" class="sweep-grid" />')
    pieces.append(f'<line x1="{margin_x:.1f}" y1="{axis_y:.1f}" x2="{width - margin_x:.1f}" y2="{axis_y:.1f}" class="sweep-axis" />')
    pieces.append(f'<line x1="{axis_x:.1f}" y1="{plot_top:.1f}" x2="{axis_x:.1f}" y2="{plot_bottom:.1f}" class="sweep-axis" />')

    if left_hull:
        pieces.append(f'<path d="{polygon_path(left_hull)}" class="sweep-left" />')
        pieces.append(hull_text("Left-turn envelope", left_hull, "#72e5ff", -16.0))
    if right_hull:
        pieces.append(f'<path d="{polygon_path(right_hull)}" class="sweep-right" />')
        pieces.append(hull_text("Right-turn envelope", right_hull, "#f4b860", 10.0))

    pieces.append(render_path((Point2D(pose["origin"]["x_mm"], pose["origin"]["y_mm"]) for pose in samples), stroke="#96a8be", width_value=2.5, dash="12 10"))

    wheel_ids = [wheel.id for wheel in active_vehicle.wheels()]
    wheel_paths: dict[str, list[Point2D]] = {wheel_id: [] for wheel_id in wheel_ids}
    axle_paths: dict[str, list[Point2D]] = {axle.id: [] for axle in active_vehicle.axles}
    for pose in samples:
        origin = Point2D(pose["origin"]["x_mm"], pose["origin"]["y_mm"])
        heading_rad = float(pose["heading_rad"])
        for wheel in active_vehicle.wheels():
            wheel_paths[wheel.id].append(origin + wheel.center.rotated_ccw(heading_rad))
        for axle in active_vehicle.axles:
            axle_paths[axle.id].append(origin + axle.center.rotated_ccw(heading_rad))

    wheel_colors = ["#f4b860", "#ffd799", "#72e5ff", "#9db4ff"]
    for index, wheel_id in enumerate(wheel_ids):
        pieces.append(render_path(wheel_paths[wheel_id], stroke=wheel_colors[index % len(wheel_colors)], width_value=3.0, dash="8 8"))

    for axle in active_vehicle.axles:
        pieces.append(render_path(axle_paths[axle.id], stroke="#69d39d", width_value=2.5, dash="14 10"))

    pieces.append(f'<path d="{polygon_path(current_outline)}" class="sweep-current" />')
    pieces.append(render_text(current_origin, f"beta {current_beta_deg:.0f}", "sweep-value"))
    pieces.append(render_circle(current_origin, 18.0, "sweep-current-origin"))
    for wheel, wheel_payload in zip(active_vehicle.wheels(), bundle["current_pose"]["wheel_centers"], strict=True):
        point = current_origin + wheel.center.rotated_ccw(float(bundle["current_pose"]["heading_rad"]))
        radius_mm = max(10.0, float(wheel_payload.get("radius_mm", 0.0)))
        pieces.append(render_circle(point, radius_mm, "sweep-wheel"))

    pieces.extend([
        _svg_text(56.0, 892.0, "Blue: left-turn envelope | Gold: right-turn envelope | Dashed tracks: wheel-center trajectories", "sweep-legend"),
        _svg_text(56.0, 928.0, "White outline: current pose | Teal origin: current maneuver snapshot", "sweep-legend"),
    ])

    pieces.append("</svg>")
    return "".join(pieces)


def _pdf_wrap_text(text: str, font_name: str, font_size: float, max_width: float) -> list[str]:
    words = text.split()
    if not words:
        return [""]

    lines: list[str] = []
    current = words[0]
    for word in words[1:]:
        candidate = f"{current} {word}"
        if stringWidth(candidate, font_name, font_size) <= max_width:
            current = candidate
        else:
            lines.append(current)
            current = word
    lines.append(current)
    return lines


def _pdf_draw_wrapped_text(
    pdf: canvas.Canvas,
    x: float,
    y: float,
    text: str,
    *,
    font_name: str = "Helvetica",
    font_size: float = 10.0,
    leading: float = 13.0,
    color=colors.white,
    max_width: float,
) -> float:
    pdf.setFont(font_name, font_size)
    pdf.setFillColor(color)
    current_y = y
    for line in _pdf_wrap_text(text, font_name, font_size, max_width):
        pdf.drawString(x, current_y, line)
        current_y -= leading
    return current_y


def _pdf_draw_section_title(pdf: canvas.Canvas, x: float, y: float, title: str, subtitle: str | None = None) -> float:
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 14)
    pdf.drawString(x, y, title)
    current_y = y - 16
    if subtitle:
        current_y = _pdf_draw_wrapped_text(
            pdf,
            x,
            current_y,
            subtitle,
            font_name="Helvetica",
            font_size=8.5,
            leading=10.0,
            color=colors.HexColor("#96a8be"),
            max_width=320.0,
        )
    return current_y


def _pdf_draw_kv_rows(
    pdf: canvas.Canvas,
    x: float,
    y: float,
    rows: list[tuple[str, str]],
    *,
    label_width: float,
    value_width: float,
    row_height: float = 18.0,
    font_size: float = 9.5,
) -> float:
    current_y = y
    for index, (label, value) in enumerate(rows):
        fill_color = colors.HexColor("#0d1827") if index % 2 == 0 else colors.HexColor("#0b1624")
        pdf.setFillColor(fill_color)
        pdf.setStrokeColor(colors.HexColor("#21324a"))
        pdf.roundRect(x, current_y - row_height + 3, label_width + value_width, row_height, 5, stroke=1, fill=1)
        pdf.setFont("Helvetica", font_size)
        pdf.setFillColor(colors.HexColor("#96a8be"))
        pdf.drawString(x + 8, current_y - 10, label)
        pdf.setFillColor(colors.HexColor("#e7eef7"))
        pdf.drawRightString(x + label_width + value_width - 8, current_y - 10, value)
        current_y -= row_height + 4
    return current_y


def _pdf_project_summary_rows(context: ExportContext) -> list[tuple[str, str]]:
    return [
        ("Vehicle", context.vehicle.name),
        ("Body", f"{context.vehicle.body_length_mm:.0f} x {context.vehicle.body_width_mm:.0f} mm"),
        ("Axle span", f"{context.vehicle.axle_span_mm():.0f} mm"),
        ("Axles", str(len(context.vehicle.axles))),
        ("Current beta", f"{context.beta_deg:.1f} deg"),
        ("Optimization", context.optimization_mode.title()),
    ]


def _pdf_optimization_rows(context: ExportContext) -> list[tuple[str, str]]:
    baseline = context.optimization_result.baseline_metrics
    optimized = context.optimization_result.optimized_metrics
    optimized_clearance = "n/a" if context.optimized_clearance is None or context.optimized_clearance.minimum_clearance_mm is None else _format_mm(context.optimized_clearance.minimum_clearance_mm)
    return [
        ("Baseline score", f"{baseline.score:.2f}"),
        ("Optimized score", f"{optimized.score:.2f}"),
        ("Improvement", f"{context.optimization_result.improvement:+.2f}"),
        ("Baseline RMS", _format_deg(baseline.rms_error_deg)),
        ("Optimized RMS", _format_deg(optimized.rms_error_deg)),
        ("Baseline sync error", _format_deg(baseline.max_abs_synchronization_error_deg)),
        ("Optimized sync error", _format_deg(optimized.max_abs_synchronization_error_deg)),
        ("Baseline clearance", "n/a" if context.baseline_clearance.minimum_clearance_mm is None else _format_mm(context.baseline_clearance.minimum_clearance_mm)),
        ("Optimized clearance", optimized_clearance),
        ("Run stats", f"{context.optimization_result.mode} / {context.optimization_result.iterations} it / {context.optimization_result.evaluations} eval"),
    ]


def _pdf_change_rows(result: OptimizationResult) -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    for variable in result.optimized_variables:
        if abs(variable.delta) < 0.005:
            continue
        rows.append((variable.id, f"{variable.current:.2f} -> {variable.optimized:.2f} ({variable.delta:+.2f})"))
    return rows


def _pdf_draw_linkage_schematic(pdf: canvas.Canvas, context: ExportContext, x: float, y: float, width: float, height: float) -> None:
    baseline_spec = context.baseline_rig.spec
    optimized_spec = context.optimized_spec
    baseline_state = context.baseline_state
    optimized_state = context.optimized_state or context.baseline_state

    body_points = list(_vehicle_body_points(context.vehicle))
    points = [
        *body_points,
        baseline_spec.bell_crank_pivot,
        baseline_spec.steering_pivot,
        baseline_state.driver_point,
        baseline_state.input_endpoint,
        baseline_state.output_endpoint,
        baseline_state.steering_endpoint,
        optimized_spec.bell_crank_pivot,
        optimized_spec.steering_pivot,
        optimized_state.driver_point,
        optimized_state.input_endpoint,
        optimized_state.output_endpoint,
        optimized_state.steering_endpoint,
    ]
    if baseline_state.companion_steering_endpoint is not None and baseline_spec.companion_steering_pivot is not None:
        points.extend([baseline_state.companion_steering_endpoint, baseline_spec.companion_steering_pivot])
    if optimized_state.companion_steering_endpoint is not None and optimized_spec.companion_steering_pivot is not None:
        points.extend([optimized_state.companion_steering_endpoint, optimized_spec.companion_steering_pivot])
    min_x = min(point.x_mm for point in points)
    max_x = max(point.x_mm for point in points)
    min_y = min(point.y_mm for point in points)
    max_y = max(point.y_mm for point in points)
    world_w = max(max_x - min_x, 1.0)
    world_h = max(max_y - min_y, 1.0)
    scale = min(width / world_w, height / world_h)
    offset_x = x + (width - world_w * scale) / 2.0
    offset_y = y + (height - world_h * scale) / 2.0

    def project(point: Point2D) -> tuple[float, float]:
        px = offset_x + (point.x_mm - min_x) * scale
        py = offset_y + (max_y - point.y_mm) * scale
        return px, py

    def line(start: Point2D, end: Point2D, color_hex: str, width_value: float = 1.4, dash: tuple[int, int] | None = None) -> None:
        x1, y1 = project(start)
        x2, y2 = project(end)
        pdf.setStrokeColor(colors.HexColor(color_hex))
        pdf.setLineWidth(width_value)
        if dash is not None:
            pdf.setDash(dash[0], dash[1])
        else:
            pdf.setDash()
        pdf.line(x1, y1, x2, y2)

    def circle(center: Point2D, radius_mm: float, color_hex: str, fill: bool = True, width_value: float = 1.0) -> None:
        cx, cy = project(center)
        radius = max(radius_mm * scale, 2.0)
        pdf.setStrokeColor(colors.HexColor(color_hex))
        pdf.setLineWidth(width_value)
        pdf.setDash()
        if fill:
            pdf.setFillColor(colors.HexColor(color_hex))
        pdf.circle(cx, cy, radius, stroke=1, fill=1 if fill else 0)

    def polygon(points: list[Point2D], stroke_hex: str, fill_hex: str | None = None, width_value: float = 1.5, dash: tuple[int, int] | None = None) -> None:
        if not points:
            return
        path = pdf.beginPath()
        first_x, first_y = project(points[0])
        path.moveTo(first_x, first_y)
        for point in points[1:]:
            px, py = project(point)
            path.lineTo(px, py)
        path.close()
        pdf.setStrokeColor(colors.HexColor(stroke_hex))
        pdf.setLineWidth(width_value)
        if dash is not None:
            pdf.setDash(dash[0], dash[1])
        else:
            pdf.setDash()
        if fill_hex is not None:
            pdf.setFillColor(colors.HexColor(fill_hex))
        pdf.drawPath(path, stroke=1, fill=1 if fill_hex is not None else 0)

    baseline_body = body_points
    optimized_body = body_points

    pdf.setFillColor(colors.HexColor("#0b1624"))
    pdf.setStrokeColor(colors.HexColor("#21324a"))
    pdf.roundRect(x, y, width, height, 10, stroke=1, fill=1)
    pdf.setFont("Helvetica-Bold", 11)
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.drawString(x + 12, y + height - 18, "Linkage schematic")
    pdf.setFont("Helvetica", 8)
    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.drawString(x + 12, y + height - 30, "Baseline dashed; optimized solid; configured body outline shown.")

    # Baseline overlay.
    polygon(baseline_body, "#7c8da6", "#0d1827", width_value=1.0, dash=(6, 4))
    polygon(optimized_body, "#72e5ff", None, width_value=1.0)
    for start, end in _linkage_segments(baseline_state, baseline_spec):
        line(start, end, "#9aa7b8", 1.2, (6, 4))

    # Optimized overlay.
    for index, (start, end) in enumerate(_linkage_segments(optimized_state, optimized_spec)):
        color = "#72e5ff" if index == 0 else ("#f4b860" if index in {1, 2} else "#69d39d")
        line(start, end, color, 1.6)

    # Pivots and nodes.
    baseline_nodes = [baseline_spec.bell_crank_pivot, baseline_spec.steering_pivot, baseline_state.input_endpoint, baseline_state.output_endpoint, baseline_state.steering_endpoint]
    if baseline_spec.companion_steering_pivot is not None:
        baseline_nodes.append(baseline_spec.companion_steering_pivot)
    if baseline_state.companion_steering_endpoint is not None:
        baseline_nodes.append(baseline_state.companion_steering_endpoint)
    for center in baseline_nodes:
        circle(center, 16.0, "#93a2b5", fill=True, width_value=0.8)
    optimized_nodes = [optimized_spec.bell_crank_pivot, optimized_spec.steering_pivot, optimized_state.input_endpoint, optimized_state.output_endpoint, optimized_state.steering_endpoint]
    if optimized_spec.companion_steering_pivot is not None:
        optimized_nodes.append(optimized_spec.companion_steering_pivot)
    if optimized_state.companion_steering_endpoint is not None:
        optimized_nodes.append(optimized_state.companion_steering_endpoint)
    for center in optimized_nodes:
        circle(center, 14.0, "#f4b860", fill=True, width_value=0.8)

    # Labels.
    pdf.setFillColor(colors.HexColor("#b9c7d8"))
    pdf.setFont("Helvetica", 8)
    bx, by = project(baseline_spec.bell_crank_pivot)
    sx, sy = project(baseline_spec.steering_pivot)
    pdf.drawString(bx - 58, by + 18, "Baseline pivot")
    pdf.drawString(sx + 14, sy + 18, "Baseline knuckle")
    if baseline_spec.companion_steering_pivot is not None:
        cx, cy = project(baseline_spec.companion_steering_pivot)
        pdf.drawString(cx + 14, cy + 34, "Baseline companion knuckle")
    bx, by = project(optimized_spec.bell_crank_pivot)
    sx, sy = project(optimized_spec.steering_pivot)
    pdf.setFillColor(colors.HexColor("#72e5ff"))
    pdf.drawString(bx - 58, by - 20, "Optimized pivot")
    pdf.drawString(sx + 14, sy - 22, "Optimized knuckle")
    if optimized_spec.companion_steering_pivot is not None:
        cx, cy = project(optimized_spec.companion_steering_pivot)
        pdf.drawString(cx + 14, cy - 82, "Optimized companion knuckle")


def _pdf_draw_metric_table(
    pdf: canvas.Canvas,
    x: float,
    y: float,
    title: str,
    rows: list[tuple[str, str]],
    width: float,
    *,
    row_height: float = 17.0,
    font_size: float = 8.8,
) -> float:
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(x, y, title)
    return _pdf_draw_kv_rows(
        pdf,
        x,
        y - 18,
        rows,
        label_width=width * 0.53,
        value_width=width * 0.47,
        row_height=row_height,
        font_size=font_size,
    )


def _pdf_draw_sweep_table(pdf: canvas.Canvas, x: float, y: float, rows: list[dict[str, object]]) -> float:
    headers = ["Beta", "Ideal FL", "Ideal FR", "Baseline steer", "Optimized steer", "Baseline err", "Optimized err"]
    col_widths = [50, 70, 70, 95, 95, 90, 90]
    total_width = sum(col_widths)
    row_height = 18.0

    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(x, y, "Steering sweep")
    top = y - 14

    # Header row.
    pdf.setFillColor(colors.HexColor("#0f1c2c"))
    pdf.setStrokeColor(colors.HexColor("#21324a"))
    pdf.roundRect(x, top - row_height, total_width, row_height, 4, stroke=1, fill=1)
    pdf.setFont("Helvetica-Bold", 8.2)
    pdf.setFillColor(colors.HexColor("#96a8be"))
    cursor_x = x
    for header, col_width in zip(headers, col_widths):
        pdf.drawCentredString(cursor_x + col_width / 2.0, top - 12, header)
        cursor_x += col_width

    current_y = top - row_height - 4
    pdf.setFont("Helvetica", 7.8)
    for index, row in enumerate(rows):
        fill_color = colors.HexColor("#0d1827") if index % 2 == 0 else colors.HexColor("#0b1624")
        pdf.setFillColor(fill_color)
        pdf.roundRect(x, current_y - row_height + 3, total_width, row_height, 3, stroke=1, fill=1)
        cursor_x = x
        values = [
            f"{row['beta_deg']:.0f}",
            f"{row['ideal_front_left_deg']:.2f}",
            f"{row['ideal_front_right_deg']:.2f}",
            "n/a" if row["baseline_steer_deg"] is None else f"{row['baseline_steer_deg']:.2f}",
            "n/a" if row["optimized_steer_deg"] is None else f"{row['optimized_steer_deg']:.2f}",
            f"{row['baseline_error_deg']:.2f}",
            "n/a" if row["optimized_error_deg"] is None else f"{row['optimized_error_deg']:.2f}",
        ]
        pdf.setFillColor(colors.HexColor("#e7eef7"))
        for value, col_width in zip(values, col_widths):
            pdf.drawCentredString(cursor_x + col_width / 2.0, current_y - 10.5, value)
            cursor_x += col_width
        current_y -= row_height + 3
    return current_y


def _pdf_actual_steering_rows(context: ExportContext) -> list[tuple[str, str]]:
    ideal_by_axle = {axle.axle_id: axle for axle in context.ideal_solution.axles}
    baseline_errors = actual_steering_errors_deg(context.baseline_actual, context.ideal_solution)
    optimized_errors = (
        {}
        if context.optimized_actual is None
        else actual_steering_errors_deg(context.optimized_actual, context.ideal_solution)
    )
    baseline_comparison = compare_actual_to_ideal(
        context.baseline_actual,
        context.ideal_solution,
        vehicle=context.vehicle,
        beta_rad=math.radians(context.beta_deg),
    )
    rows = [
        ("Max wheel error", _format_deg(max((abs(value) for value in baseline_errors.values()), default=0.0))),
        ("Sync error", _format_deg(baseline_comparison["front_rear_synchronization_error_deg"])),
    ]
    for actual_axle in context.baseline_actual.axles:
        ideal_axle = ideal_by_axle[actual_axle.axle_id]
        optimized_axle = (
            None
            if context.optimized_actual is None
            else next(
                (item for item in context.optimized_actual.axles if item.axle_id == actual_axle.axle_id),
                None,
            )
        )
        optimized_center = "n/a" if optimized_axle is None else f"{optimized_axle.center_steering_angle_deg:+.1f}"
        optimized_left = "n/a" if optimized_axle is None else f"{optimized_errors.get(optimized_axle.left_wheel.wheel_id, 0.0):+.1f}"
        optimized_right = "n/a" if optimized_axle is None else f"{optimized_errors.get(optimized_axle.right_wheel.wheel_id, 0.0):+.1f}"
        rows.append(
            (
                f"{actual_axle.axle_id} center / L-R error",
                f"I/B/O {ideal_axle.center_steering_angle_deg:+.1f}/{actual_axle.center_steering_angle_deg:+.1f}/{optimized_center} | err B {baseline_errors.get(actual_axle.left_wheel.wheel_id, 0.0):+.1f}/{baseline_errors.get(actual_axle.right_wheel.wheel_id, 0.0):+.1f} O {optimized_left}/{optimized_right}",
            )
        )
    return rows


def build_export_pdf(
    beta_deg: float,
    optimization_mode: str = "quick",
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> bytes:
    context = build_export_context(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    sweep = build_steering_sweep_bundle(
        optimization_mode=optimization_mode,
        step_deg=15.0,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    swept = build_swept_path_bundle(
        current_beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        step_deg=1.0,
        vehicle=vehicle,
    )

    output = io.BytesIO()
    pdf = canvas.Canvas(output, pagesize=landscape(letter), pageCompression=0)
    width, height = landscape(letter)
    pdf.setTitle("EasyTowing Engineering Report")
    pdf.setAuthor("EasyTowing")
    pdf.setSubject("Trailer steering design and optimization report")
    background_color = colors.HexColor("#08111d")

    def draw_page_background() -> None:
        pdf.setFillColor(background_color)
        pdf.rect(0, 0, width, height, stroke=0, fill=1)

    # Page 1: summary and schematic.
    draw_page_background()
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 20)
    pdf.drawString(36, height - 34, "EasyTowing Engineering Report")
    pdf.setFont("Helvetica", 9.5)
    pdf.setFillColor(colors.HexColor("#72e5ff"))
    pdf.drawString(36, height - 50, f"Beta {beta_deg:.1f} deg | {optimization_mode.title()} optimization | Generated {datetime.utcnow().isoformat(timespec='seconds')}Z")

    left_x = 36
    left_width = 332
    right_x = 384
    schematic_y = 108
    schematic_width = width - right_x - 36
    schematic_height = 360

    cursor_y = height - 72
    cursor_y = _pdf_draw_metric_table(pdf, left_x, cursor_y, "Project and vehicle", _pdf_project_summary_rows(context), left_width)
    cursor_y -= 14
    cursor_y = _pdf_draw_metric_table(pdf, left_x, cursor_y, "Optimization result", _pdf_optimization_rows(context), left_width)
    cursor_y -= 14
    _pdf_draw_metric_table(
        pdf,
        left_x,
        cursor_y,
        "Changed dimensions",
        _pdf_change_rows(context.optimization_result),
        left_width,
        row_height=13.0,
        font_size=7.4,
    )

    _pdf_draw_linkage_schematic(pdf, context, right_x, schematic_y, schematic_width, schematic_height)

    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.setFont("Helvetica", 8)
    pdf.drawString(36, 28, "Baseline design is dashed; optimized design is solid. All geometry is reported in millimeters and degrees.")
    pdf.showPage()

    # Page 2: steering sweep and swept path.
    draw_page_background()
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 18)
    pdf.drawString(36, height - 34, "Steering sweep and swept path")
    pdf.setFont("Helvetica", 9.5)
    pdf.setFillColor(colors.HexColor("#72e5ff"))
    pdf.drawString(36, height - 50, f"Articulation sweep and envelope summary for beta {beta_deg:.1f} deg")

    sweep_rows = [row for row in sweep["samples"] if abs(float(row["beta_deg"]) % 15.0) < 1e-9 or abs(float(row["beta_deg"])) < 1e-9 or abs(float(row["beta_deg"]) - 45.0) < 1e-9]
    # Keep the sweep table compact and in order.
    sweep_rows = sorted(sweep_rows, key=lambda row: float(row["beta_deg"]))
    table_bottom = _pdf_draw_sweep_table(pdf, 36, height - 78, sweep_rows)

    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(36, table_bottom - 18, "Swept path metrics")
    pdf.setFont("Helvetica", 9)
    pdf.setFillColor(colors.HexColor("#96a8be"))
    metrics_rows = [
        ("Swept width", _format_mm(swept["metrics"]["swept_width_mm"])),
        ("Swept length", _format_mm(swept["metrics"]["swept_length_mm"])),
        ("Left span", "n/a" if swept["metrics"]["left_turn_width_mm"] is None else _format_mm(swept["metrics"]["left_turn_width_mm"])),
        ("Right span", "n/a" if swept["metrics"]["right_turn_width_mm"] is None else _format_mm(swept["metrics"]["right_turn_width_mm"])),
        ("Sample count", str(swept["sample_count"])),
    ]
    _pdf_draw_kv_rows(pdf, 36, table_bottom - 30, metrics_rows, label_width=150, value_width=150, row_height=18.0, font_size=8.8)
    actual_table_x = 360
    actual_table_y = table_bottom - 18
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(actual_table_x, actual_table_y, "Actual vs ideal at current beta")
    _pdf_draw_kv_rows(
        pdf,
        actual_table_x,
        actual_table_y - 18,
        _pdf_actual_steering_rows(context),
        label_width=170,
        value_width=226,
        row_height=15.0,
        font_size=6.0,
    )

    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.setFont("Helvetica", 8)
    pdf.drawString(36, 28, "Swept-path preview uses the current beta surrogate and traces the body extents plus wheel-center trajectories.")

    pdf.save()
    return output.getvalue()


ENGINEERING_FAILURE_GUIDANCE: dict[str, dict[str, str]] = {
    "MODEL_COMPLETENESS": {
        "check_id": "MODEL_COMPLETENESS",
        "title": "Complete the vehicle combination",
        "action": "Define at least two rigid bodies and provide a positive rectangular envelope or a valid CAD outline for every body before trusting clearance results.",
    },
    "KINEMATICS": {
        "check_id": "KINEMATICS",
        "title": "Check body and joint geometry",
        "action": "Verify body dimensions, joint anchors, articulation bounds, and the explicit maneuver radius.",
    },
    "MECHANISM": {
        "check_id": "MECHANISM",
        "title": "Make the mechanism solvable",
        "action": "Check rigid member lengths, fixed and driven point positions, branch continuity, and wheel-output mappings.",
    },
    "COLLISION": {
        "check_id": "COLLISION",
        "title": "Remove component overlap",
        "action": "Open Clearance focus, inspect the highlighted pair, then move the components or correct their envelopes. Connected joints are excluded; other overlaps are hard failures.",
    },
    "CLEARANCE": {
        "check_id": "CLEARANCE",
        "title": "Increase minimum clearance",
        "action": "Move the conflicting pivot or link, or revise the envelope until the configured clearance target is met.",
    },
    "STEERING_LIMIT_EXCEEDED": {
        "check_id": "STEERING_LIMIT_EXCEEDED",
        "title": "Respect the steering stop",
        "action": "Change the linkage ratio or geometry, or confirm a larger physical steering stop. Do not treat the rod as an implicit stop.",
    },
    "DRAWBAR_LIMIT_EXCEEDED": {
        "check_id": "DRAWBAR_LIMIT_EXCEEDED",
        "title": "Respect the articulation stop",
        "action": "Reduce the requested articulation range or update the approved drawbar limit.",
    },
    "MULTIBODY_KINEMATIC_INCONSISTENT": {
        "check_id": "MULTIBODY_KINEMATIC_INCONSISTENT",
        "title": "Resolve multi-body closure",
        "action": "Check joint anchors, body-local coordinates, and the common maneuver radius for the failing body.",
    },
    "JOINT_CLOSURE": {
        "check_id": "JOINT_CLOSURE",
        "title": "Verify articulation joint closure",
        "action": "Check every parent and child joint anchor and rerun the maneuver until the maximum joint closure error is within tolerance.",
    },
    "LINKAGE_NO_SOLUTION": {
        "check_id": "LINKAGE_NO_SOLUTION",
        "title": "Check linkage reach",
        "action": "Adjust link lengths or pivot locations so the fixed-length circles intersect throughout the requested range.",
    },
    "LINKAGE_BRANCH_CHANGE": {
        "check_id": "LINKAGE_BRANCH_CHANGE",
        "title": "Prevent branch switching",
        "action": "Check the neutral assembly branch and incremental motion, then redesign near toggle positions.",
    },
    "ACTUAL_STEERING_UNSOLVED": {
        "check_id": "ACTUAL_STEERING_UNSOLVED",
        "title": "Complete wheel mapping",
        "action": "Map every required wheel to a valid mechanism output and verify steering direction and ratio.",
    },
    "OPTIMIZATION_NO_FEASIBLE_SOLUTION": {
        "check_id": "OPTIMIZATION_NO_FEASIBLE_SOLUTION",
        "title": "No feasible candidate",
        "action": "Relax only approved design bounds or targets, or change the mechanism. Do not apply an infeasible proposal.",
    },
}


def engineering_failure_guidance(check_ids: Iterable[str]) -> list[dict[str, str]]:
    """Return ordered operator actions for hard-check failures."""
    guidance: list[dict[str, str]] = []
    seen: set[str] = set()
    for raw_check_id in check_ids:
        check_id = str(raw_check_id)
        if not check_id or check_id in seen:
            continue
        seen.add(check_id)
        guidance.append(
            ENGINEERING_FAILURE_GUIDANCE.get(
                check_id,
                {
                    "check_id": check_id,
                    "title": f"Investigate {check_id}",
                    "action": "Review the detailed failure and correct the design before saving or submitting this revision.",
                },
            )
        )
    return guidance


def _serialized_combination_geometry_status(
    raw_combination: object,
) -> tuple[bool, str]:
    """Ensure multi-body evidence includes geometry for every body."""

    if raw_combination is None:
        return True, "single-layout study"
    if not isinstance(raw_combination, dict):
        return False, "vehicle combination is not an object"

    raw_bodies = raw_combination.get("bodies")
    if not isinstance(raw_bodies, list) or len(raw_bodies) < 2:
        return False, "at least two rigid bodies are required"
    raw_body_count = raw_combination.get("body_count")
    try:
        body_count = int(raw_body_count)
    except (TypeError, ValueError):
        body_count = -1
    if body_count != len(raw_bodies):
        return False, "body count does not match the serialized bodies"

    body_ids: set[str] = set()
    for index, raw_body in enumerate(raw_bodies):
        if not isinstance(raw_body, dict):
            return False, f"body {index + 1} is not an object"
        body_id = str(raw_body.get("id", ""))
        if not body_id or body_id in body_ids:
            return False, f"body {index + 1} has a missing or duplicate ID"
        body_ids.add(body_id)

        try:
            length_mm = float(raw_body.get("body_length_mm"))
            width_mm = float(raw_body.get("body_width_mm"))
        except (TypeError, ValueError):
            length_mm = width_mm = 0.0
        has_positive_dimensions = (
            math.isfinite(length_mm)
            and math.isfinite(width_mm)
            and length_mm > 0.0
            and width_mm > 0.0
        )

        raw_polygon = raw_body.get("body_polygon")
        try:
            polygon = tuple(
                Point2D(
                    float(point.get("x_mm")),
                    float(point.get("y_mm")),
                )
                for point in raw_polygon
            )
            PolygonEnvelope(polygon)
            has_valid_outline = isinstance(raw_polygon, list)
        except (AttributeError, TypeError, ValueError, InvalidGeometryError):
            has_valid_outline = False
        if not has_positive_dimensions and not has_valid_outline:
            return False, f"body {body_id!r} has no positive envelope dimensions or CAD outline"

    return True, f"{len(raw_bodies)} rigid bodies have usable envelopes"


def evaluate_engineering_snapshot(
    snapshot: dict[str, Any],
    *,
    clearance_target_mm: float = 20.0,
) -> dict[str, object]:
    combination = snapshot.get("combination_kinematics")
    combination = combination if isinstance(combination, dict) else {}
    mechanism = snapshot.get("mechanism_graph")
    mechanism = mechanism if isinstance(mechanism, dict) else {}
    graph_state = mechanism.get("state")
    graph_state = graph_state if isinstance(graph_state, dict) else {}
    clearance = snapshot.get("clearance")
    clearance = clearance if isinstance(clearance, dict) else {}
    model_complete, model_detail = _serialized_combination_geometry_status(
        snapshot.get("vehicle_combination")
    )
    raw_kinematics_residual = combination.get("maximum_constraint_residual_mm")
    try:
        kinematics_residual = float(raw_kinematics_residual)
    except (TypeError, ValueError):
        kinematics_residual = math.inf
    raw_joint_closure = combination.get("maximum_joint_closure_error_mm")
    try:
        joint_closure = float(raw_joint_closure)
    except (TypeError, ValueError):
        joint_closure = math.inf
    raw_mechanism_residual = graph_state.get("maximum_residual_mm")
    try:
        mechanism_residual = float(raw_mechanism_residual)
    except (TypeError, ValueError):
        mechanism_residual = math.inf
    raw_minimum_clearance = clearance.get("minimum_clearance_mm")
    try:
        minimum_clearance = float(raw_minimum_clearance)
    except (TypeError, ValueError):
        minimum_clearance = None
    checks = [
        {
            "id": "MODEL_COMPLETENESS",
            "pass": model_complete,
            "detail": model_detail,
        },
        {
            "id": "KINEMATICS",
            "pass": math.isfinite(kinematics_residual) and kinematics_residual <= 0.01,
            "detail": (
                "not evaluated"
                if raw_kinematics_residual is None
                else f"{kinematics_residual:.3f} mm maximum rolling residual"
            ),
        },
    ]
    if snapshot.get("vehicle_combination") is not None:
        checks.append(
            {
                "id": "JOINT_CLOSURE",
                "pass": math.isfinite(joint_closure) and joint_closure <= 0.01,
                "detail": (
                    "not evaluated"
                    if raw_joint_closure is None
                    else f"{joint_closure:.3f} mm maximum joint closure error"
                ),
            }
        )
    checks.extend([
        {
            "id": "MECHANISM",
            "pass": bool(graph_state) and math.isfinite(mechanism_residual) and mechanism_residual <= 0.01,
            "detail": (
                "not solved"
                if not graph_state
                else (
                    "not solved"
                    if not math.isfinite(mechanism_residual)
                    else f"{mechanism_residual:.3f} mm maximum member residual"
                )
            ),
        },
        {
            "id": "COLLISION",
            "pass": clearance.get("collision_detected") is False,
            "detail": "clear" if clearance.get("collision_detected") is False else "collision detected or not evaluated",
        },
        {
            "id": "CLEARANCE",
            "pass": (
                minimum_clearance is not None
                and math.isfinite(minimum_clearance)
                and minimum_clearance >= clearance_target_mm
            ),
            "detail": (
                "not evaluated"
                if minimum_clearance is None or not math.isfinite(minimum_clearance)
                else f"{minimum_clearance:.1f} mm available; {clearance_target_mm:.1f} mm required"
            ),
        },
    ])
    result_scope = snapshot.get("result_scope")
    all_checks_pass = all(bool(check["pass"]) for check in checks)
    return {
        "status": (
            "PASS"
            if all_checks_pass
            else ("INCOMPLETE" if result_scope == "ideal_kinematics" else "FAIL")
        ),
        "result_scope": result_scope,
        "checks": checks,
        "guidance": engineering_failure_guidance(
            check["id"] for check in checks if not bool(check["pass"])
        ),
        "clearance_target_mm": clearance_target_mm,
        "steering_acceptance_included": False,
    }


def _report_clearance_target_mm(
    snapshot: dict[str, Any],
    override: float | None,
) -> float:
    """Use the target captured with a saved sweep instead of a display default."""

    candidates: list[object] = [override]
    sweep = snapshot.get("sweep_validation")
    if isinstance(sweep, dict):
        candidates.append(sweep.get("clearance_target_mm"))
    optimization = snapshot.get("optimization")
    if isinstance(optimization, dict):
        objective = optimization.get("objective")
        if isinstance(objective, dict):
            candidates.append(objective.get("clearance_target_mm"))
    candidates.append(20.0)
    for candidate in candidates:
        try:
            target = float(candidate)
        except (TypeError, ValueError):
            continue
        if math.isfinite(target) and target >= 0.0:
            return target
    return 20.0


def build_engineering_snapshot_csv(
    snapshot: dict[str, Any],
    *,
    project_name: str,
    revision_id: str,
    clearance_target_mm: float | None = None,
) -> str:
    evaluation = evaluate_engineering_snapshot(
        snapshot,
        clearance_target_mm=_report_clearance_target_mm(snapshot, clearance_target_mm),
    )
    actual = snapshot.get("actual_steering") or {}
    actual_angles = actual.get("wheel_angles_deg") or {}
    errors = actual.get("errors_deg") or {}
    output_by_wheel = {
        assignment["wheel_id"]: assignment["output_id"]
        for assignment in (snapshot.get("mechanism_mapping") or {}).get("steering_assignments", [])
    }
    ideal_by_wheel: dict[str, float] = {}
    axle_by_wheel: dict[str, str] = {}
    for axle in snapshot.get("axles", []):
        axle_id = str(axle.get("axle_id", ""))
        raw_wheels = axle.get("wheels")
        wheels = raw_wheels if isinstance(raw_wheels, list) else [
            axle.get("left_wheel") or {},
            axle.get("right_wheel") or {},
        ]
        for wheel in wheels:
            if not isinstance(wheel, dict):
                continue
            wheel_id = str(wheel.get("wheel_id", wheel.get("id", "")))
            if not wheel_id:
                continue
            ideal_by_wheel[wheel_id] = float(wheel.get("steering_angle_deg", 0.0))
            axle_by_wheel[wheel_id] = axle_id

    clearance = snapshot.get("clearance") or {}
    graph_state = (snapshot.get("mechanism_graph") or {}).get("state") or {}
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(
        [
            "project_name",
            "revision_id",
            "engineering_status",
            "beta_deg",
            "turn_radius_mm",
            "wheel_id",
            "axle_id",
            "mechanism_output_id",
            "ideal_steering_deg",
            "actual_steering_deg",
            "error_deg",
            "mechanism_residual_mm",
            "minimum_clearance_mm",
            "collision_detected",
        ]
    )
    for wheel_id in sorted(set(ideal_by_wheel) | set(actual_angles)):
        writer.writerow(
            [
                project_name,
                revision_id,
                evaluation["status"],
                snapshot.get("beta_deg"),
                snapshot.get("turn_radius_mm"),
                wheel_id,
                axle_by_wheel.get(wheel_id, ""),
                output_by_wheel.get(wheel_id, ""),
                ideal_by_wheel.get(wheel_id),
                actual_angles.get(wheel_id),
                errors.get(wheel_id),
                graph_state.get("maximum_residual_mm"),
                clearance.get("minimum_clearance_mm"),
                clearance.get("collision_detected"),
            ]
        )
    return output.getvalue()


def build_engineering_snapshot_pdf(
    snapshot: dict[str, Any],
    *,
    project_name: str,
    revision_id: str,
    clearance_target_mm: float | None = None,
) -> bytes:
    evaluation = evaluate_engineering_snapshot(
        snapshot,
        clearance_target_mm=_report_clearance_target_mm(snapshot, clearance_target_mm),
    )
    status = str(evaluation["status"])
    combination = snapshot.get("vehicle_combination") or {}
    graph = (snapshot.get("mechanism_graph") or {}).get("mechanism") or {}
    graph_state = (snapshot.get("mechanism_graph") or {}).get("state") or {}
    clearance = snapshot.get("clearance") or {}
    metrics = snapshot.get("metrics") or {}

    output = io.BytesIO()
    pdf = canvas.Canvas(output, pagesize=landscape(letter), pageCompression=0)
    width, height = landscape(letter)
    pdf.setTitle(f"EasyTowing {status} Engineering Evaluation")
    pdf.setAuthor("EasyTowing")
    pdf.setSubject("Multi-body steering mechanism diagnostic report")

    def page_header(title: str, subtitle: str) -> None:
        pdf.setFillColor(colors.HexColor("#08111d"))
        pdf.rect(0, 0, width, height, stroke=0, fill=1)
        pdf.setFillColor(colors.HexColor("#e7eef7"))
        pdf.setFont("Helvetica-Bold", 20)
        pdf.drawString(36, height - 36, title)
        pdf.setFont("Helvetica", 9)
        pdf.setFillColor(colors.HexColor("#72e5ff"))
        pdf.drawString(36, height - 52, subtitle)

    page_header(
        "EasyTowing Multi-body Engineering Evaluation",
        f"{project_name} | Revision {revision_id} | Generated {datetime.utcnow().isoformat(timespec='seconds')}Z",
    )
    status_color = "#69d39d" if status == "PASS" else "#ff7d7d"
    pdf.setFillColor(colors.HexColor(status_color))
    pdf.setFont("Helvetica-Bold", 28)
    pdf.drawString(36, height - 94, status)
    pdf.setFont("Helvetica", 9)
    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.drawString(112, height - 88, "HARD ENGINEERING CHECKS")
    if status != "PASS":
        pdf.drawString(112, height - 102, "DIAGNOSTIC ONLY - NOT APPROVED FOR MANUFACTURING")

    summary_rows = [
        ("Bodies / joints", f"{combination.get('body_count', 0)} / {combination.get('joint_count', 0)}"),
        ("Mounted axles", str(combination.get("mounted_axle_count", len(snapshot.get("axles", []))))),
        ("Articulation", f"{float(snapshot.get('beta_deg', 0.0)):.1f} deg"),
        ("Root turn radius", _format_mm(float(snapshot.get("turn_radius_mm", 0.0)))),
        ("Graph points / members", f"{len(graph.get('points', []))} / {len(graph.get('members', []))}"),
        ("Wheel mappings", str(len((snapshot.get("mechanism_mapping") or {}).get("steering_assignments", [])))),
        ("Graph residual", _format_mm(float(graph_state.get("maximum_residual_mm", math.inf)))),
        ("Minimum clearance", "n/a" if clearance.get("minimum_clearance_mm") is None else _format_mm(float(clearance["minimum_clearance_mm"]))),
        ("Max actual steering error", _format_deg(metrics.get("max_abs_wheel_error_deg"))),
        ("Max synchronization error", _format_deg(metrics.get("max_abs_synchronization_error_deg"))),
    ]
    _pdf_draw_metric_table(pdf, 36, height - 130, "Design configuration", summary_rows, 340)
    check_rows = [
        (f"{check['id']} - {'PASS' if check['pass'] else 'FAIL'}", str(check["detail"]))
        for check in evaluation["checks"]
    ]
    _pdf_draw_metric_table(pdf, 408, height - 130, "Hard checks", check_rows, 348)
    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.setFont("Helvetica", 8)
    pdf.drawString(36, 28, "Steering-error acceptance is excluded until Monroc approves project-specific limits.")
    pdf.showPage()

    page_header(
        "Wheel results and mechanism traceability",
        f"{status} evaluation at beta {float(snapshot.get('beta_deg', 0.0)):.1f} deg",
    )
    actual = snapshot.get("actual_steering") or {}
    actual_angles = actual.get("wheel_angles_deg") or {}
    errors = actual.get("errors_deg") or {}
    mapping_by_wheel = {
        item["wheel_id"]: item["output_id"]
        for item in (snapshot.get("mechanism_mapping") or {}).get("steering_assignments", [])
    }
    wheel_rows = []
    for axle in snapshot.get("axles", []):
        raw_wheels = axle.get("wheels")
        wheels = raw_wheels if isinstance(raw_wheels, list) else [
            axle.get("left_wheel") or {},
            axle.get("right_wheel") or {},
        ]
        for wheel in wheels:
            if not isinstance(wheel, dict):
                continue
            wheel_id = str(wheel.get("wheel_id", wheel.get("id", "")))
            if not wheel_id:
                continue
            wheel_rows.append(
                (
                    wheel_id,
                    f"ideal {float(wheel.get('steering_angle_deg', 0.0)):+.2f} deg | actual {float(actual_angles.get(wheel_id, 0.0)):+.2f} deg | error {float(errors.get(wheel_id, 0.0)):+.2f} deg",
                )
            )
    _pdf_draw_metric_table(pdf, 36, height - 78, "Wheel steering", wheel_rows, 720, row_height=17.0, font_size=7.7)
    mapping_rows = [(wheel_id, output_id) for wheel_id, output_id in sorted(mapping_by_wheel.items())]
    _pdf_draw_metric_table(pdf, 36, height - 78 - 38 - max(len(wheel_rows), 1) * 17, "Named mechanism outputs", mapping_rows, 720, row_height=15.0, font_size=6.8)
    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.setFont("Helvetica", 8)
    pdf.drawString(36, 28, "This report records the saved revision. CAD/manufacturing export requires a PASS verdict and controlled approval.")
    pdf.save()
    return output.getvalue()


def _svg_escape_text(value: str) -> str:
    return escape(value, {"'": "&apos;", '"': "&quot;"})


def _svg_point(point: Point2D) -> str:
    return f"{point.x_mm:.1f},{-point.y_mm:.1f}"


def _svg_line(x1: float, y1: float, x2: float, y2: float, class_name: str) -> str:
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" class="{class_name}" />'


def _svg_circle(center: Point2D, radius_mm: float, class_name: str) -> str:
    return f'<circle cx="{center.x_mm:.1f}" cy="{-center.y_mm:.1f}" r="{radius_mm:.1f}" class="{class_name}" />'


def _svg_text(x: float, y: float, value: str, class_name: str) -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" class="{class_name}">{_svg_escape_text(value)}</text>'


def _svg_polygon(points: Iterable[Point2D], class_name: str) -> str:
    point_text = " ".join(_svg_point(point) for point in points)
    return f'<polygon points="{point_text}" class="{class_name}" />'


def _draw_linkage(state: PlanarLinkageState | None, spec: PlanarLinkageSpec, prefix: str, label: str, opacity: float) -> str:
    if state is None:
        return ""
    pieces = [f'<g opacity="{opacity:.2f}" data-label="{_svg_escape_text(label)}">']
    pieces.append(_svg_line(state.driver_point.x_mm, -state.driver_point.y_mm, state.input_endpoint.x_mm, -state.input_endpoint.y_mm, f"{prefix}-rod"))
    pieces.append(_svg_line(spec.bell_crank_pivot.x_mm, -spec.bell_crank_pivot.y_mm, state.input_endpoint.x_mm, -state.input_endpoint.y_mm, f"{prefix}-arm"))
    pieces.append(_svg_line(spec.bell_crank_pivot.x_mm, -spec.bell_crank_pivot.y_mm, state.output_endpoint.x_mm, -state.output_endpoint.y_mm, f"{prefix}-arm-2"))
    pieces.append(_svg_line(state.output_endpoint.x_mm, -state.output_endpoint.y_mm, state.steering_endpoint.x_mm, -state.steering_endpoint.y_mm, f"{prefix}-tie"))
    pieces.append(_svg_line(spec.steering_pivot.x_mm, -spec.steering_pivot.y_mm, state.steering_endpoint.x_mm, -state.steering_endpoint.y_mm, f"{prefix}-steer"))
    if state.companion_steering_endpoint is not None and spec.companion_steering_pivot is not None:
        pieces.append(_svg_line(state.steering_endpoint.x_mm, -state.steering_endpoint.y_mm, state.companion_steering_endpoint.x_mm, -state.companion_steering_endpoint.y_mm, f"{prefix}-companion-tie"))
        pieces.append(_svg_line(spec.companion_steering_pivot.x_mm, -spec.companion_steering_pivot.y_mm, state.companion_steering_endpoint.x_mm, -state.companion_steering_endpoint.y_mm, f"{prefix}-companion-steer"))
    pieces.append(_svg_circle(state.driver_point, 18.0, f"{prefix}-driver"))
    pieces.append(_svg_circle(state.input_endpoint, 18.0, f"{prefix}-node"))
    pieces.append(_svg_circle(state.output_endpoint, 18.0, f"{prefix}-node"))
    pieces.append(_svg_circle(state.steering_endpoint, 18.0, f"{prefix}-node"))
    pieces.append(_svg_circle(spec.bell_crank_pivot, 24.0, f"{prefix}-pivot"))
    pieces.append(_svg_circle(spec.steering_pivot, 24.0, f"{prefix}-pivot"))
    if state.companion_steering_endpoint is not None and spec.companion_steering_pivot is not None:
        pieces.append(_svg_circle(state.companion_steering_endpoint, 18.0, f"{prefix}-node"))
        pieces.append(_svg_circle(spec.companion_steering_pivot, 24.0, f"{prefix}-pivot"))
    pieces.append(_svg_text(spec.bell_crank_pivot.x_mm + 40, -spec.bell_crank_pivot.y_mm - 30, f"{label} bell crank", f"{prefix}-label"))
    pieces.append(_svg_text(spec.steering_pivot.x_mm + 40, -spec.steering_pivot.y_mm - 30, f"{label} knuckle", f"{prefix}-label"))
    if spec.companion_steering_pivot is not None:
        pieces.append(_svg_text(spec.companion_steering_pivot.x_mm + 40, -spec.companion_steering_pivot.y_mm - 30, f"{label} companion knuckle", f"{prefix}-label"))
    pieces.append("</g>")
    return "".join(pieces)


def _dimension_line(start: Point2D, end: Point2D, offset_mm: float, label: str, value: str, class_name: str) -> str:
    vector = end - start
    length = vector.length()
    if length <= 1e-9:
        return ""
    direction = vector.scale(1.0 / length)
    normal = Point2D(-direction.y_mm, direction.x_mm)
    offset = normal.scale(offset_mm)
    dim_start = start + offset
    dim_end = end + offset
    arrow = 12.0
    midpoint = Point2D((dim_start.x_mm + dim_end.x_mm) / 2.0, (dim_start.y_mm + dim_end.y_mm) / 2.0)
    label_point = midpoint + normal.scale(22.0)
    pieces = [f'<g class="{class_name}">']
    pieces.append(_svg_line(start.x_mm, -start.y_mm, dim_start.x_mm, -dim_start.y_mm, f"{class_name}-ext"))
    pieces.append(_svg_line(end.x_mm, -end.y_mm, dim_end.x_mm, -dim_end.y_mm, f"{class_name}-ext"))
    pieces.append(_svg_line(dim_start.x_mm, -dim_start.y_mm, dim_end.x_mm, -dim_end.y_mm, f"{class_name}-main"))
    pieces.append(_svg_line(dim_start.x_mm, -dim_start.y_mm, dim_start.x_mm + direction.x_mm * arrow, -(dim_start.y_mm + direction.y_mm * arrow), f"{class_name}-arrow"))
    pieces.append(_svg_line(dim_end.x_mm, -dim_end.y_mm, dim_end.x_mm - direction.x_mm * arrow, -(dim_end.y_mm - direction.y_mm * arrow), f"{class_name}-arrow"))
    pieces.append(_svg_text(label_point.x_mm, -label_point.y_mm, f"{label}: {value}", f"{class_name}-label"))
    pieces.append("</g>")
    return "".join(pieces)


def _snapshot_point(value: object) -> Point2D | None:
    if not isinstance(value, dict):
        return None
    try:
        x_mm = float(value["x_mm"])
        y_mm = float(value["y_mm"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(x_mm) or not math.isfinite(y_mm):
        return None
    return Point2D(x_mm, y_mm)


def _snapshot_transform_point(pose: dict[str, object], point: Point2D) -> Point2D:
    x_mm = float(pose.get("x_mm", 0.0))
    y_mm = float(pose.get("y_mm", 0.0))
    yaw_rad = float(pose.get("yaw_rad", 0.0))
    return Point2D(
        x_mm + point.x_mm * math.cos(yaw_rad) - point.y_mm * math.sin(yaw_rad),
        y_mm + point.x_mm * math.sin(yaw_rad) + point.y_mm * math.cos(yaw_rad),
    )


def _snapshot_body_outline(body: dict[str, object]) -> tuple[Point2D, ...]:
    raw_polygon = body.get("body_polygon")
    raw_points = raw_polygon if isinstance(raw_polygon, list) else []
    polygon = tuple(
        point
        for raw_point in raw_points
        if (point := _snapshot_point(raw_point)) is not None
    )
    if len(polygon) >= 3:
        return polygon
    try:
        length_mm = float(body.get("body_length_mm") or 0.0)
        width_mm = float(body.get("body_width_mm") or 0.0)
    except (TypeError, ValueError):
        return ()
    if length_mm <= 0.0 or width_mm <= 0.0:
        return ()
    half_length = length_mm / 2.0
    half_width = width_mm / 2.0
    return (
        Point2D(-half_length, -half_width),
        Point2D(half_length, -half_width),
        Point2D(half_length, half_width),
        Point2D(-half_length, half_width),
    )


def _snapshot_multibody_geometry(snapshot: dict[str, Any]) -> dict[str, object]:
    """Normalize saved multi-body geometry for all diagnostic renderers."""

    combination = snapshot.get("vehicle_combination")
    combination = combination if isinstance(combination, dict) else {}
    bodies: list[dict[str, object]] = []
    for raw_body in combination.get("bodies", []):
        if not isinstance(raw_body, dict):
            continue
        raw_pose = raw_body.get("pose")
        pose = raw_pose if isinstance(raw_pose, dict) else {}
        outline = tuple(
            _snapshot_transform_point(pose, point)
            for point in _snapshot_body_outline(raw_body)
        )
        if outline:
            bodies.append(
                {
                    "id": str(raw_body.get("id", "body")),
                    "name": str(raw_body.get("name", raw_body.get("id", "body"))),
                    "outline": outline,
                    "pose": pose,
                    "length_mm": raw_body.get("body_length_mm"),
                    "width_mm": raw_body.get("body_width_mm"),
                }
            )
    if not bodies:
        outline = tuple(
            point
            for raw_point in snapshot.get("body_outline", [])
            if (point := _snapshot_point(raw_point)) is not None
        )
        if outline:
            bodies.append(
                {
                    "id": "vehicle",
                    "name": str((snapshot.get("vehicle") or {}).get("name", "Vehicle")),
                    "outline": outline,
                    "pose": {},
                    "length_mm": None,
                    "width_mm": None,
                }
            )

    body_by_id = {str(body["id"]): body for body in bodies}
    joints: list[dict[str, object]] = []
    for raw_joint in combination.get("joints", []):
        if not isinstance(raw_joint, dict):
            continue
        parent_body = body_by_id.get(str(raw_joint.get("parent_body_id", "")))
        child_body = body_by_id.get(str(raw_joint.get("child_body_id", "")))
        parent_anchor = _snapshot_point(raw_joint.get("parent_anchor"))
        child_anchor = _snapshot_point(raw_joint.get("child_anchor"))
        if parent_body is None or child_body is None or parent_anchor is None or child_anchor is None:
            continue
        joints.append(
            {
                "id": str(raw_joint.get("id", "joint")),
                "parent_anchor": _snapshot_transform_point(parent_body["pose"], parent_anchor),
                "child_anchor": _snapshot_transform_point(child_body["pose"], child_anchor),
                "articulation_deg": float(raw_joint.get("articulation_deg", 0.0)),
                "maximum_articulation_deg": float(raw_joint.get("maximum_articulation_deg", 45.0)),
            }
        )

    raw_axles = snapshot.get("axles")
    if not isinstance(raw_axles, list):
        vehicle = snapshot.get("vehicle")
        raw_axles = vehicle.get("axles", []) if isinstance(vehicle, dict) else []
    ideal_angles = ((snapshot.get("ideal") or {}).get("wheel_angles_deg") or {})
    axles: list[dict[str, object]] = []
    for raw_axle in raw_axles:
        if not isinstance(raw_axle, dict):
            continue
        center = _snapshot_point(raw_axle.get("center"))
        raw_wheels = raw_axle.get("wheels")
        wheel_records = [item for item in raw_wheels if isinstance(item, dict)] if isinstance(raw_wheels, list) else [
            raw_axle.get("left_wheel") or {},
            raw_axle.get("right_wheel") or {},
        ]
        parsed_wheels = [
            {
                "center": wheel_center,
                "id": str(item.get("wheel_id", item.get("id", ""))),
                "heading_rad": float(item.get("heading_rad", 0.0)),
                "side": str(item.get("side", "")),
            }
            for item in wheel_records
            if (wheel_center := _snapshot_point(item.get("center"))) is not None
        ]
        left_record = next((item for item in parsed_wheels if item["side"] == "left"), None)
        right_record = next((item for item in parsed_wheels if item["side"] == "right"), None)
        left = None if left_record is None else left_record["center"]
        right = None if right_record is None else right_record["center"]
        axle_id = str(raw_axle.get("axle_id", raw_axle.get("id", "axle")))
        if center is None:
            center = Point2D(float(raw_axle.get("x_mm", 0.0)), float(raw_axle.get("y_mm", 0.0)))
        if left is None or right is None:
            track_mm = float(raw_axle.get("track_mm", 0.0))
            heading_rad = float(raw_axle.get("heading_rad", 0.0))
            lateral = Point2D(-math.sin(heading_rad), math.cos(heading_rad)).scale(track_mm / 2.0)
            left = center + lateral
            right = center - lateral
        if center is None or left is None or right is None:
            continue
        left_id = str(
            (raw_axle.get("left_wheel") or {}).get("wheel_id", f"{axle_id}_left")
            if left_record is None
            else left_record["id"]
        )
        right_id = str(
            (raw_axle.get("right_wheel") or {}).get("wheel_id", f"{axle_id}_right")
            if right_record is None
            else right_record["id"]
        )
        base_heading_rad = float(raw_axle.get("heading_rad", raw_axle.get("center_heading_rad", 0.0)))
        for item in parsed_wheels:
            if not item["id"]:
                item["id"] = f"{axle_id}_{item['side'] or 'wheel'}"
            if item["heading_rad"] == 0.0:
                item["heading_rad"] = base_heading_rad + math.radians(float(ideal_angles.get(item["id"], 0.0)))
        if left_record is None:
            left_heading_rad = float((raw_axle.get("left_wheel") or {}).get("heading_rad", base_heading_rad + math.radians(float(ideal_angles.get(left_id, 0.0)))))
        else:
            left_heading_rad = float(left_record["heading_rad"])
        if right_record is None:
            right_heading_rad = float((raw_axle.get("right_wheel") or {}).get("heading_rad", base_heading_rad + math.radians(float(ideal_angles.get(right_id, 0.0)))))
        else:
            right_heading_rad = float(right_record["heading_rad"])
        axles.append(
            {
                "id": axle_id,
                "center": center,
                "left": left,
                "right": right,
                "wheels": parsed_wheels,
                "left_id": left_id,
                "right_id": right_id,
                "left_heading_rad": left_heading_rad,
                "right_heading_rad": right_heading_rad,
            }
        )

    graph = snapshot.get("mechanism_graph")
    graph = graph if isinstance(graph, dict) else {}
    graph_definition = graph.get("mechanism")
    graph_definition = graph_definition if isinstance(graph_definition, dict) else {}
    graph_state = graph.get("state")
    graph_state = graph_state if isinstance(graph_state, dict) else {}
    graph_positions = graph_state.get("point_positions")
    graph_positions = graph_positions if isinstance(graph_positions, dict) else {}
    points: list[dict[str, object]] = []
    for raw_point in graph_definition.get("points", []):
        if not isinstance(raw_point, dict):
            continue
        position = _snapshot_point(graph_positions.get(str(raw_point.get("id", ""))))
        if position is None:
            position = _snapshot_point(raw_point.get("neutral_position"))
        if position is not None:
            points.append(
                {
                    "id": str(raw_point.get("id", "point")),
                    "position": position,
                    "radius_mm": float(raw_point.get("envelope_radius_mm", 0.0)),
                    "body_id": raw_point.get("body_id"),
                }
            )
    position_by_id = {str(point["id"]): point["position"] for point in points}
    members: list[dict[str, object]] = []
    for raw_member in graph_definition.get("members", []):
        if not isinstance(raw_member, dict):
            continue
        start = position_by_id.get(str(raw_member.get("point_a_id", "")))
        end = position_by_id.get(str(raw_member.get("point_b_id", "")))
        if start is not None and end is not None:
            members.append(
                {
                    "id": str(raw_member.get("id", "member")),
                    "start": start,
                    "end": end,
                    "radius_mm": float(raw_member.get("envelope_radius_mm", 0.0)),
                }
            )
    outputs: list[dict[str, object]] = []
    for raw_output in graph_definition.get("angle_outputs", []):
        if not isinstance(raw_output, dict):
            continue
        start = position_by_id.get(str(raw_output.get("pivot_point_id", "")))
        end = position_by_id.get(str(raw_output.get("endpoint_point_id", "")))
        if start is not None and end is not None:
            outputs.append({"id": str(raw_output.get("id", "output")), "start": start, "end": end})

    icr = _snapshot_point(snapshot.get("icr"))
    all_points = [point for body in bodies for point in body["outline"]]
    all_points.extend(
        point
        for axle in axles
        for wheel in axle["wheels"]
        for point in (wheel["center"],)
    )
    all_points.extend(point["position"] for point in points)
    all_points.extend(
        point
        for joint in joints
        for point in (joint["parent_anchor"], joint["child_anchor"])
    )
    if icr is not None:
        all_points.append(icr)
    if not all_points:
        all_points = [Point2D(0.0, 0.0)]
    return {
        "bodies": bodies,
        "joints": joints,
        "axles": axles,
        "points": points,
        "members": members,
        "outputs": outputs,
        "icr": icr,
        "all_points": all_points,
    }


def _snapshot_bounds(geometry: dict[str, object]) -> tuple[float, float, float, float]:
    points = geometry["all_points"]
    min_x = min(point.x_mm for point in points)
    max_x = max(point.x_mm for point in points)
    min_y = min(point.y_mm for point in points)
    max_y = max(point.y_mm for point in points)
    return min_x, max_x, min_y, max_y


def build_engineering_snapshot_svg(
    snapshot: dict[str, Any],
    *,
    project_name: str,
    revision_id: str,
    clearance_target_mm: float | None = None,
) -> str:
    """Render the saved multi-body revision as a dimensioned diagnostic sketch."""

    geometry = _snapshot_multibody_geometry(snapshot)
    min_x, max_x, min_y, max_y = _snapshot_bounds(geometry)
    margin_x = 650.0
    margin_y = 550.0
    header_height = 950.0
    view_min_x = min_x - margin_x
    view_min_y = -(max_y + margin_y + header_height)
    view_width = max(max_x - min_x + 2.0 * margin_x, 1.0)
    view_height = max(max_y - min_y + 2.0 * margin_y + header_height, 1.0)
    evaluation = evaluate_engineering_snapshot(
        snapshot,
        clearance_target_mm=_report_clearance_target_mm(snapshot, clearance_target_mm),
    )
    status = str(evaluation["status"])
    status_color = "#69d39d" if status == "PASS" else "#ff7d7d"
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="1800" height="1100" viewBox="{view_min_x:.1f} {view_min_y:.1f} {view_width:.1f} {view_height:.1f}" role="img" aria-label="Multi-body dimensioned engineering sketch">',
        """<style>
          .canvas { fill: #08111d; }
          .grid { stroke: rgba(255,255,255,0.06); stroke-width: 8; }
          .body { fill: rgba(114,229,255,0.10); stroke: #72e5ff; stroke-width: 18; }
          .joint { stroke: #f4b860; stroke-width: 18; stroke-dasharray: 45 25; }
          .axle { stroke: #f4b860; stroke-width: 18; }
          .wheel { fill: #13283a; stroke: #e7eef7; stroke-width: 12; }
          .wheel-heading { stroke: #e7eef7; stroke-width: 10; }
          .member { stroke: #69d39d; stroke-width: 18; }
          .output { stroke: #ff9b8f; stroke-width: 24; }
          .point { fill: #ffffff; stroke: #08111d; stroke-width: 12; }
          .icr { fill: none; stroke: #ff7d7d; stroke-width: 14; stroke-dasharray: 36 24; }
          .dimension-main { stroke: #f4b860; stroke-width: 12; }
          .dimension-ext, .dimension-arrow { stroke: #f4b860; stroke-width: 8; }
          .dimension-label { fill: #f4b860; font-size: 72px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
          .title { fill: #e7eef7; font-size: 100px; font-family: "Bahnschrift", "Segoe UI", sans-serif; font-weight: 700; }
          .subtitle { fill: #96a8be; font-size: 58px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
          .label { fill: #e7eef7; font-size: 58px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
          .small-label { fill: #96a8be; font-size: 48px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
          .status { font-size: 82px; font-family: "Bahnschrift", "Segoe UI", sans-serif; font-weight: 700; }
        </style>""",
        f'<rect x="{view_min_x:.1f}" y="{view_min_y:.1f}" width="{view_width:.1f}" height="{view_height:.1f}" class="canvas" />',
    ]
    for x in range(math.floor(min_x / 1000.0) * 1000, math.ceil(max_x / 1000.0) * 1000 + 1, 1000):
        pieces.append(_svg_line(x, -(max_y + margin_y), x, -(min_y - margin_y), "grid"))
    for y in range(math.floor(min_y / 1000.0) * 1000, math.ceil(max_y / 1000.0) * 1000 + 1, 1000):
        pieces.append(_svg_line(min_x - margin_x, -y, max_x + margin_x, -y, "grid"))

    header_x = min_x - margin_x + 100.0
    header_world_y = max_y + margin_y + header_height - 160.0
    pieces.append(_svg_text(header_x, -header_world_y, "EasyTowing Multi-body Engineering Sketch", "title"))
    pieces.append(_svg_text(header_x, -(header_world_y - 150.0), f"{_svg_escape_text(project_name)} | Revision {_svg_escape_text(revision_id)} | beta {float(snapshot.get('beta_deg', 0.0)):.1f} deg", "subtitle"))
    pieces.append(_svg_text(header_x, -(header_world_y - 285.0), "Diagnostic geometry from the saved revision; not a manufacturing release.", "subtitle"))
    pieces.append(_svg_text(max_x - 1100.0, -header_world_y, status, "status"))

    for body in geometry["bodies"]:
        pieces.append(_svg_polygon(body["outline"], "body"))
        outline = body["outline"]
        center = Point2D(
            sum(point.x_mm for point in outline) / len(outline),
            sum(point.y_mm for point in outline) / len(outline),
        )
        dimension_text = ""
        if body["length_mm"] is not None and body["width_mm"] is not None:
            dimension_text = f" | {float(body['length_mm']):.0f} x {float(body['width_mm']):.0f} mm"
        pieces.append(_svg_text(center.x_mm, -center.y_mm, f"{body['name']}{dimension_text}", "label"))
    for joint in geometry["joints"]:
        pieces.append(_svg_line(joint["parent_anchor"].x_mm, -joint["parent_anchor"].y_mm, joint["child_anchor"].x_mm, -joint["child_anchor"].y_mm, "joint"))
        pieces.append(_svg_circle(joint["parent_anchor"], 65.0, "point"))
        pieces.append(_svg_circle(joint["child_anchor"], 65.0, "point"))
        midpoint = Point2D(
            (joint["parent_anchor"].x_mm + joint["child_anchor"].x_mm) / 2.0,
            (joint["parent_anchor"].y_mm + joint["child_anchor"].y_mm) / 2.0,
        )
        pieces.append(_svg_text(midpoint.x_mm, -midpoint.y_mm - 110.0, f"{joint['id']} {float(joint['articulation_deg']):+.1f} deg / stop {float(joint['maximum_articulation_deg']):.1f} deg", "small-label"))
    for axle in geometry["axles"]:
        pieces.append(_svg_line(axle["left"].x_mm, -axle["left"].y_mm, axle["right"].x_mm, -axle["right"].y_mm, "axle"))
        for wheel in axle["wheels"]:
            center = wheel["center"]
            heading = wheel["heading_rad"]
            pieces.append(_svg_circle(center, 120.0, "wheel"))
            end = Point2D(center.x_mm + math.cos(heading) * 450.0, center.y_mm + math.sin(heading) * 450.0)
            pieces.append(_svg_line(center.x_mm, -center.y_mm, end.x_mm, -end.y_mm, "wheel-heading"))
        pieces.append(_svg_text(axle["center"].x_mm + 100.0, -axle["center"].y_mm + 180.0, str(axle["id"]), "small-label"))
    for member in geometry["members"]:
        pieces.append(_svg_line(member["start"].x_mm, -member["start"].y_mm, member["end"].x_mm, -member["end"].y_mm, "member"))
    for output in geometry["outputs"]:
        pieces.append(_svg_line(output["start"].x_mm, -output["start"].y_mm, output["end"].x_mm, -output["end"].y_mm, "output"))
        pieces.append(_svg_text(output["end"].x_mm + 90.0, -output["end"].y_mm, str(output["id"]), "small-label"))
    for point in geometry["points"]:
        pieces.append(_svg_circle(point["position"], max(float(point["radius_mm"]), 45.0), "point"))
        pieces.append(_svg_text(point["position"].x_mm + 80.0, -point["position"].y_mm - 80.0, str(point["id"]), "small-label"))
    if geometry["icr"] is not None:
        pieces.append(_svg_circle(geometry["icr"], 120.0, "icr"))
        pieces.append(_svg_text(geometry["icr"].x_mm + 150.0, -geometry["icr"].y_mm, "ICR", "small-label"))

    pieces.append(_dimension_line(Point2D(min_x, min_y), Point2D(max_x, min_y), -350.0, "Overall length", _format_mm(max_x - min_x), "dimension"))
    pieces.append(_dimension_line(Point2D(min_x, min_y), Point2D(min_x, max_y), 350.0, "Overall width", _format_mm(max_y - min_y), "dimension"))
    checks = ", ".join(f"{check['id']} {'PASS' if check['pass'] else 'FAIL'}" for check in evaluation["checks"])
    pieces.append(_svg_text(header_x, -(header_world_y - 420.0), checks, "small-label"))
    pieces.append("</svg>")
    return "".join(pieces)


def _multibody_dxf_layers() -> list[tuple[str, int]]:
    return [
        ("ANNOTATION", 7),
        ("BODY", 4),
        ("JOINT", 2),
        ("AXLE", 2),
        ("WHEEL", 7),
        ("MECHANISM", 3),
        ("OUTPUT", 1),
        ("DIMENSION", 6),
        ("ICR", 5),
    ]


def build_engineering_snapshot_dxf(
    snapshot: dict[str, Any],
    *,
    project_name: str,
    revision_id: str,
    clearance_target_mm: float | None = None,
) -> str:
    """Export saved multi-body geometry as a diagnostic ASCII DXF."""

    geometry = _snapshot_multibody_geometry(snapshot)
    min_x, max_x, min_y, max_y = _snapshot_bounds(geometry)
    evaluation = evaluate_engineering_snapshot(
        snapshot,
        clearance_target_mm=_report_clearance_target_mm(snapshot, clearance_target_mm),
    )
    lines: list[str] = ["0", "SECTION", "2", "HEADER", "0", "ENDSEC"]
    layer_lines: list[str] = ["0", "TABLE", "2", "LAYER", "70", str(len(_multibody_dxf_layers()))]
    for layer_name, color in _multibody_dxf_layers():
        layer_lines.extend(["0", "LAYER", "2", layer_name, "70", "0", "62", str(color), "6", "CONTINUOUS"])
    layer_lines.extend(["0", "ENDTAB"])
    lines.extend(_dxf_section("TABLES", layer_lines))
    entities: list[str] = []
    entities.extend(_dxf_text("ANNOTATION", Point2D(min_x, max_y + 1200.0), "EasyTowing Multi-body Engineering Sketch", 120.0))
    entities.extend(_dxf_text("ANNOTATION", Point2D(min_x, max_y + 950.0), f"{project_name} | Revision {revision_id} | beta {float(snapshot.get('beta_deg', 0.0)):.1f} deg", 72.0))
    entities.extend(_dxf_text("ANNOTATION", Point2D(min_x, max_y + 700.0), f"DIAGNOSTIC ONLY | ENGINEERING {evaluation['status']} | NOT A MANUFACTURING RELEASE", 58.0))
    for body in geometry["bodies"]:
        entities.extend(_dxf_lwpolyline("BODY", body["outline"], closed=True))
        center = Point2D(
            sum(point.x_mm for point in body["outline"]) / len(body["outline"]),
            sum(point.y_mm for point in body["outline"]) / len(body["outline"]),
        )
        entities.extend(_dxf_text("ANNOTATION", center, str(body["name"]), 58.0))
    for joint in geometry["joints"]:
        entities.extend(_dxf_line("JOINT", joint["parent_anchor"], joint["child_anchor"]))
        entities.extend(_dxf_circle("JOINT", joint["parent_anchor"], 65.0))
        entities.extend(_dxf_circle("JOINT", joint["child_anchor"], 65.0))
        entities.extend(_dxf_text("ANNOTATION", joint["parent_anchor"], f"{joint['id']} {float(joint['articulation_deg']):+.1f} deg", 48.0))
    for axle in geometry["axles"]:
        entities.extend(_dxf_line("AXLE", axle["left"], axle["right"]))
        for wheel in axle["wheels"]:
            center = wheel["center"]
            heading = wheel["heading_rad"]
            entities.extend(_dxf_circle("WHEEL", center, 120.0))
            entities.extend(_dxf_line("WHEEL", center, Point2D(center.x_mm + math.cos(heading) * 450.0, center.y_mm + math.sin(heading) * 450.0)))
        entities.extend(_dxf_text("ANNOTATION", axle["center"], str(axle["id"]), 48.0))
    for member in geometry["members"]:
        entities.extend(_dxf_line("MECHANISM", member["start"], member["end"]))
    for output in geometry["outputs"]:
        entities.extend(_dxf_line("OUTPUT", output["start"], output["end"]))
        entities.extend(_dxf_text("ANNOTATION", output["end"], str(output["id"]), 48.0))
    for point in geometry["points"]:
        entities.extend(_dxf_circle("MECHANISM", point["position"], max(float(point["radius_mm"]), 45.0)))
        entities.extend(_dxf_text("ANNOTATION", point["position"], str(point["id"]), 42.0))
    if geometry["icr"] is not None:
        entities.extend(_dxf_circle("ICR", geometry["icr"], 120.0))
        entities.extend(_dxf_text("ANNOTATION", geometry["icr"], "ICR", 48.0))
    dimension_y = min_y - 500.0
    entities.extend(_dxf_line("DIMENSION", Point2D(min_x, min_y), Point2D(min_x, dimension_y)))
    entities.extend(_dxf_line("DIMENSION", Point2D(max_x, min_y), Point2D(max_x, dimension_y)))
    entities.extend(_dxf_line("DIMENSION", Point2D(min_x, dimension_y), Point2D(max_x, dimension_y)))
    entities.extend(_dxf_text("DIMENSION", Point2D((min_x + max_x) / 2.0, dimension_y), f"Overall length {_format_mm(max_x - min_x)}", 58.0))
    dimension_x = min_x - 500.0
    entities.extend(_dxf_line("DIMENSION", Point2D(min_x, min_y), Point2D(dimension_x, min_y)))
    entities.extend(_dxf_line("DIMENSION", Point2D(min_x, max_y), Point2D(dimension_x, max_y)))
    entities.extend(_dxf_line("DIMENSION", Point2D(dimension_x, min_y), Point2D(dimension_x, max_y)))
    entities.extend(_dxf_text("DIMENSION", Point2D(dimension_x, (min_y + max_y) / 2.0), f"Overall width {_format_mm(max_y - min_y)}", 58.0))
    lines.extend(_dxf_section("ENTITIES", entities))
    lines.extend(["0", "EOF"])
    return "\n".join(lines)


def build_engineering_snapshot_png(
    snapshot: dict[str, Any],
    *,
    project_name: str,
    revision_id: str,
    clearance_target_mm: float | None = None,
) -> bytes:
    """Render saved multi-body geometry as a diagnostic PNG snapshot."""

    geometry = _snapshot_multibody_geometry(snapshot)
    min_x, max_x, min_y, max_y = _snapshot_bounds(geometry)
    evaluation = evaluate_engineering_snapshot(
        snapshot,
        clearance_target_mm=_report_clearance_target_mm(snapshot, clearance_target_mm),
    )
    width, height = 1800, 1100
    plot_left, plot_right = 80, width - 80
    plot_top, plot_bottom = 170, height - 150
    image = Image.new("RGB", (width, height), "#08111d")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    world_width = max(max_x - min_x, 1.0)
    world_height = max(max_y - min_y, 1.0)
    scale = min((plot_right - plot_left) / world_width, (plot_bottom - plot_top) / world_height)
    offset_x = plot_left + ((plot_right - plot_left) - world_width * scale) / 2.0
    offset_y = plot_top + ((plot_bottom - plot_top) - world_height * scale) / 2.0

    def screen(point: Point2D) -> tuple[int, int]:
        return (round(offset_x + (point.x_mm - min_x) * scale), round(offset_y + (max_y - point.y_mm) * scale))

    def line(start: Point2D, end: Point2D, color: str, width_value: int = 4) -> None:
        draw.line((*screen(start), *screen(end)), fill=color, width=width_value)

    draw.text((80, 35), "EasyTowing Multi-body Engineering Snapshot", fill="#e7eef7", font=font)
    draw.text((80, 65), f"{project_name} | Revision {revision_id} | beta {float(snapshot.get('beta_deg', 0.0)):.1f} deg", fill="#96a8be", font=font)
    draw.text((80, 95), "Diagnostic geometry from the saved revision; not a manufacturing release.", fill="#96a8be", font=font)
    status_color = "#69d39d" if evaluation["status"] == "PASS" else "#ff7d7d"
    draw.text((width - 280, 45), str(evaluation["status"]), fill=status_color, font=font)
    for body in geometry["bodies"]:
        draw.polygon([screen(point) for point in body["outline"]], fill="#0e2a3d", outline="#72e5ff")
        center = Point2D(sum(point.x_mm for point in body["outline"]) / len(body["outline"]), sum(point.y_mm for point in body["outline"]) / len(body["outline"]))
        draw.text(screen(center), str(body["name"]), fill="#e7eef7", font=font)
    for joint in geometry["joints"]:
        line(joint["parent_anchor"], joint["child_anchor"], "#f4b860", 5)
        for anchor in (joint["parent_anchor"], joint["child_anchor"]):
            x, y = screen(anchor)
            draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill="#f4b860", outline="#ffffff")
    for axle in geometry["axles"]:
        line(axle["left"], axle["right"], "#f4b860", 4)
        for wheel in axle["wheels"]:
            center = wheel["center"]
            heading = wheel["heading_rad"]
            x, y = screen(center)
            radius = max(7, round(120.0 * scale))
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill="#13283a", outline="#e7eef7")
            line(center, Point2D(center.x_mm + math.cos(heading) * 450.0, center.y_mm + math.sin(heading) * 450.0), "#e7eef7", 2)
        draw.text(screen(axle["center"]), str(axle["id"]), fill="#96a8be", font=font)
    for member in geometry["members"]:
        line(member["start"], member["end"], "#69d39d", 4)
    for output in geometry["outputs"]:
        line(output["start"], output["end"], "#ff9b8f", 5)
        draw.text(screen(output["end"]), str(output["id"]), fill="#ff9b8f", font=font)
    for point in geometry["points"]:
        x, y = screen(point["position"])
        draw.ellipse((x - 7, y - 7, x + 7, y + 7), fill="#ffffff", outline="#08111d")
        draw.text((x + 10, y - 10), str(point["id"]), fill="#96a8be", font=font)
    if geometry["icr"] is not None:
        x, y = screen(geometry["icr"])
        draw.ellipse((x - 9, y - 9, x + 9, y + 9), outline="#ff7d7d", width=3)
        draw.text((x + 12, y - 10), "ICR", fill="#ff7d7d", font=font)
    dimension_y = min_y - max(world_height * 0.08, 300.0)
    line(Point2D(min_x, dimension_y), Point2D(max_x, dimension_y), "#f4b860", 2)
    draw.text(screen(Point2D((min_x + max_x) / 2.0, dimension_y)), f"L {_format_mm(max_x - min_x)}", fill="#f4b860", font=font)
    draw.rectangle((80, height - 105, width - 80, height - 35), fill="#0e1d2e", outline="#2a4962")
    check_text = " | ".join(f"{check['id']} {'PASS' if check['pass'] else 'FAIL'}" for check in evaluation["checks"])
    draw.text((100, height - 82), check_text, fill="#e7eef7", font=font)
    output = io.BytesIO()
    image.save(output, format="PNG")
    return output.getvalue()


def _comparison_metric_rows(context: ExportContext) -> list[tuple[str, str, str]]:
    baseline = context.optimization_result.baseline_metrics
    optimized = context.optimization_result.optimized_metrics
    optimized_state = context.optimized_state
    optimized_clearance = context.optimized_clearance.minimum_clearance_mm if context.optimized_clearance else None
    return [
        ("Max Ackermann error", _format_deg(baseline.max_abs_error_deg), _format_deg(optimized.max_abs_error_deg)),
        ("RMS error", _format_deg(baseline.rms_error_deg), _format_deg(optimized.rms_error_deg)),
        ("Max synchronization error", _format_deg(baseline.max_abs_synchronization_error_deg), _format_deg(optimized.max_abs_synchronization_error_deg)),
        ("Minimum clearance", "n/a" if context.baseline_clearance.minimum_clearance_mm is None else _format_mm(context.baseline_clearance.minimum_clearance_mm), "n/a" if optimized_clearance is None else _format_mm(optimized_clearance)),
        ("Optimization score", f"{baseline.score:.2f}", f"{optimized.score:.2f}"),
        ("Actual steer", _format_deg(context.baseline_state.steering_angle_deg), "n/a" if optimized_state is None else _format_deg(optimized_state.steering_angle_deg)),
    ]


def _changed_variable_rows(result: OptimizationResult) -> list[tuple[str, float, float, float]]:
    return [(before.id, before.current, after.optimized, after.delta) for before, after in zip(result.baseline_variables, result.optimized_variables)]


def build_dimensioned_svg(
    beta_deg: float,
    optimization_mode: str = "quick",
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> str:
    context = build_export_context(
        beta_deg=beta_deg,
        optimization_mode=optimization_mode,
        linkage_rig=linkage_rig,
        vehicle=vehicle,
    )
    width = 20000
    height = 9800
    baseline_spec = context.baseline_rig.spec
    optimized_spec = context.optimized_spec
    reference_state = context.optimized_state or context.baseline_state
    reference_spec = optimized_spec if context.optimized_state is not None else baseline_spec

    styles = """
      .canvas { fill: #08111d; }
      .grid { stroke: rgba(255,255,255,0.05); stroke-width: 1.5; }
      .body { fill: rgba(114,229,255,0.04); stroke: rgba(114,229,255,0.55); stroke-width: 14; }
      .axle { stroke: rgba(244,184,96,0.7); stroke-width: 16; }
      .baseline-rod { stroke: rgba(180,190,200,0.45); stroke-width: 9; stroke-dasharray: 24 16; }
      .baseline-arm, .baseline-arm-2, .baseline-tie, .baseline-steer, .baseline-companion-tie, .baseline-companion-steer { stroke: rgba(180,190,200,0.45); stroke-width: 10; stroke-dasharray: 24 16; }
      .baseline-node, .baseline-driver, .baseline-pivot { fill: rgba(220,225,230,0.8); stroke: #08111d; stroke-width: 6; }
      .baseline-label { fill: #b9c7d8; font-size: 70px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .optimized-rod { stroke: rgba(114,229,255,0.95); stroke-width: 10; }
      .optimized-arm, .optimized-arm-2 { stroke: rgba(244,184,96,0.95); stroke-width: 12; }
      .optimized-tie { stroke: rgba(105,211,157,0.95); stroke-width: 10; }
      .optimized-steer, .optimized-companion-steer { stroke: rgba(255,125,125,0.95); stroke-width: 11; }
      .optimized-companion-tie { stroke: rgba(105,211,157,0.95); stroke-width: 10; stroke-dasharray: 18 10; }
      .optimized-node, .optimized-driver, .optimized-pivot { fill: rgba(255,255,255,0.95); stroke: #08111d; stroke-width: 7; }
      .optimized-label { fill: #72e5ff; font-size: 70px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .dimension-ext { stroke: rgba(255,255,255,0.35); stroke-width: 3; }
      .dimension-main { stroke: rgba(244,184,96,0.95); stroke-width: 5; }
      .dimension-arrow { stroke: rgba(244,184,96,0.95); stroke-width: 4; }
      .dimension-label { fill: #f4b860; font-size: 64px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .panel { fill: rgba(11, 22, 36, 0.96); stroke: rgba(138,171,204,0.25); stroke-width: 3; }
      .panel-title { fill: #e7eef7; font-size: 94px; font-family: "Bahnschrift", "Segoe UI", sans-serif; font-weight: 700; }
      .panel-subtitle { fill: #96a8be; font-size: 56px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .metric-label { fill: #96a8be; font-size: 56px; font-family: "Bahnschrift", "Segoe UI", sans-serif; text-transform: uppercase; letter-spacing: 0.16em; }
      .metric-value { fill: #e7eef7; font-size: 72px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .metric-compare { fill: #72e5ff; font-size: 64px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .legend-line { stroke: rgba(255,255,255,0.55); stroke-width: 6; stroke-dasharray: 22 12; }
    """

    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="-4200 -5400 20000 11000">',
        f"<style>{styles}</style>",
        '<rect x="-4200" y="-5400" width="20000" height="11000" class="canvas" />',
    ]

    for x in range(-4000, 4001, 500):
        pieces.append(f'<line x1="{x}" y1="-5000" x2="{x}" y2="5000" class="grid" />')
    for y in range(-5000, 5001, 500):
        pieces.append(f'<line x1="-4000" y1="{y}" x2="3800" y2="{y}" class="grid" />')
    pieces.append(_svg_line(-4000, 0, 3800, 0, "legend-line"))
    pieces.append(_svg_line(0, -5000, 0, 5000, "legend-line"))

    pieces.append(_svg_polygon(_vehicle_body_points(context.vehicle), "body"))

    for axle in context.vehicle.axles:
        left, right = axle.outer_wheels()
        pieces.append(_svg_line(left.center.x_mm, -left.center.y_mm, right.center.x_mm, -right.center.y_mm, "axle"))

    pieces.append(_draw_linkage(context.baseline_state, baseline_spec, "baseline", "Existing", 0.32))
    pieces.append(_draw_linkage(context.optimized_state, optimized_spec, "optimized", "Optimized", 0.95))

    pieces.append(
        _dimension_line(
            reference_state.driver_point,
            reference_state.input_endpoint,
            -220.0,
            "Input rod",
            _format_mm(reference_spec.input_rod_length_mm),
            "dimension",
        )
    )
    pieces.append(
        _dimension_line(
            reference_state.output_endpoint,
            reference_state.steering_endpoint,
            220.0,
            "Tie rod",
            _format_mm(reference_spec.tie_rod_length_mm),
            "dimension",
        )
    )
    pieces.append(
        _dimension_line(
            reference_spec.bell_crank_pivot,
            reference_state.input_endpoint,
            -180.0,
            "Bell crank input",
            _format_mm(reference_spec.bell_crank_input_arm_length_mm),
            "dimension",
        )
    )
    pieces.append(
        _dimension_line(
            reference_spec.bell_crank_pivot,
            reference_state.output_endpoint,
            180.0,
            "Bell crank output",
            _format_mm(reference_spec.bell_crank_output_arm_length_mm),
            "dimension",
        )
    )
    pieces.append(
        _dimension_line(
            reference_spec.steering_pivot,
            reference_state.steering_endpoint,
            180.0,
            "Steering arm",
            _format_mm(reference_spec.steering_arm_length_mm),
            "dimension",
        )
    )
    if reference_state.companion_steering_endpoint is not None and reference_spec.companion_steering_pivot is not None:
        companion_tie_length = reference_spec.companion_tie_rod_length_mm
        companion_arm_length = reference_spec.companion_steering_arm_length_mm
        if companion_tie_length is not None:
            pieces.append(
                _dimension_line(
                    reference_state.steering_endpoint,
                    reference_state.companion_steering_endpoint,
                    -260.0,
                    "Companion tie rod",
                    _format_mm(companion_tie_length),
                    "dimension",
                )
            )
        if companion_arm_length is not None:
            pieces.append(
                _dimension_line(
                    reference_spec.companion_steering_pivot,
                    reference_state.companion_steering_endpoint,
                    260.0,
                    "Companion steering arm",
                    _format_mm(companion_arm_length),
                    "dimension",
                )
            )
    pieces.append(
        _dimension_line(
            reference_spec.bell_crank_pivot,
            reference_spec.steering_pivot,
            260.0,
            "Pivot spacing",
            _format_mm((reference_spec.steering_pivot - reference_spec.bell_crank_pivot).length()),
            "dimension",
        )
    )

    panel_x = 4300.0
    panel_y = -4700.0
    panel_w = 11200.0
    panel_h = 9300.0
    pieces.append(f'<rect x="{panel_x:.1f}" y="{panel_y:.1f}" width="{panel_w:.1f}" height="{panel_h:.1f}" rx="72" class="panel" />')
    pieces.append(_svg_text(panel_x + 260, panel_y + 240, "EasyTowing Engineering Sketch", "panel-title"))
    pieces.append(_svg_text(panel_x + 260, panel_y + 360, f"Beta {beta_deg:.1f} deg  |  {optimization_mode.title()} optimization", "panel-subtitle"))
    pieces.append(_svg_text(panel_x + 260, panel_y + 520, "Before / after linkage comparison with dimension callouts", "panel-subtitle"))

    y = panel_y + 820
    for label, before, after in _comparison_metric_rows(context):
        pieces.append(_svg_text(panel_x + 260, y, label, "metric-label"))
        pieces.append(_svg_text(panel_x + 2700, y, before, "metric-value"))
        pieces.append(_svg_text(panel_x + 5300, y, "->", "metric-value"))
        pieces.append(_svg_text(panel_x + 5700, y, after, "metric-compare"))
        y += 650

    y += 200
    pieces.append(_svg_text(panel_x + 260, y, "Changed dimensions", "metric-label"))
    y += 360
    for name, before, after, delta in _changed_variable_rows(context.optimization_result):
        pieces.append(_svg_text(panel_x + 260, y, name, "panel-subtitle"))
        pieces.append(_svg_text(panel_x + 4200, y, f"{before:.2f} -> {after:.2f}", "metric-value"))
        pieces.append(_svg_text(panel_x + 8000, y, f"{delta:+.2f}", "metric-compare"))
        y += 540

    pieces.append(_svg_text(panel_x + 260, panel_y + panel_h - 320, "Existing design = dashed; optimized design = solid.", "panel-subtitle"))
    pieces.append("</svg>")
    return "".join(pieces)
