from __future__ import annotations

from dataclasses import dataclass
import csv
import io
import json
import math
from datetime import datetime
from functools import lru_cache
from typing import Iterable
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.pagesizes import landscape, letter
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen import canvas

from .collision import CapsuleEnvelope, CircleEnvelope, CollisionItem, ClearanceReport, PolygonEnvelope, analyze_clearance
from .geometry import Point2D
from .linkage import (
    LinkageDemoRig,
    PlanarLinkageSpec,
    PlanarLinkageState,
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage,
    solve_reference_linkage_demo,
)
from .model import VehicleLayout, build_reference_demo_layout
from .optimization import (
    LinkageOptimizationProblem,
    OptimizationResult,
    OptimizedVariable,
    build_branch_hint,
    build_optimized_spec,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from .steering import IdealSteeringSolution, beta_to_reference_radius_mm, build_demo_solution


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


def _point_payload(point: Point2D) -> dict[str, float]:
    return {"x_mm": point.x_mm, "y_mm": point.y_mm}


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
    rear_axle = next(axle for axle in vehicle.axles if axle.id == "rear_axle")
    front_axle = next(axle for axle in vehicle.axles if axle.id == "front_axle")
    rear_left, rear_right = rear_axle.wheels()
    front_left, front_right = front_axle.wheels()

    return (
        CollisionItem("input_rod", CapsuleEnvelope(state.driver_point, state.input_endpoint, 14.0)),
        CollisionItem("tie_rod", CapsuleEnvelope(state.output_endpoint, state.steering_endpoint, 14.0)),
        CollisionItem("bell_crank_pivot", CircleEnvelope(spec.bell_crank_pivot, 28.0)),
        CollisionItem("steering_pivot", CircleEnvelope(spec.steering_pivot, 28.0)),
        CollisionItem("front_axle_beam", CapsuleEnvelope(front_left.center, front_right.center, 70.0)),
        CollisionItem("rear_axle_beam", CapsuleEnvelope(rear_left.center, rear_right.center, 70.0)),
    )


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
        "axle_span_mm": vehicle.axle_span_mm(),
        "axles": [
            {
                "id": axle.id,
                "center": _point_payload(axle.center),
                "track_mm": axle.track_mm,
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
            }
            for axle in vehicle.axles
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
    }


def _serialize_optimization_result(result: OptimizationResult) -> dict[str, object]:
    def serialize_metrics(metrics) -> dict[str, object]:
        return {
            "score": metrics.score,
            "rms_error_deg": metrics.rms_error_deg,
            "mean_abs_error_deg": metrics.mean_abs_error_deg,
            "max_abs_error_deg": metrics.max_abs_error_deg,
            "minimum_clearance_mm": metrics.minimum_clearance_mm,
            "failure_index": metrics.failure_index,
            "solved_samples": metrics.solved_samples,
            "sample_count": metrics.sample_count,
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
        "baseline": serialize_metrics(result.baseline_metrics),
        "optimized": serialize_metrics(result.optimized_metrics),
        "variables_before": [serialize_variable(variable) for variable in result.baseline_variables],
        "variables_after": [serialize_variable(variable) for variable in result.optimized_variables],
    }


def _format_mm(value: float) -> str:
    return f"{value:.1f} mm"


def _format_deg(value: float) -> str:
    return f"{value:.2f} deg"


def build_export_context(beta_deg: float, optimization_mode: str = "quick") -> ExportContext:
    vehicle, ideal_solution, _radius = build_demo_solution(beta_deg)
    baseline_rig = build_reference_linkage_demo()
    baseline_state = solve_reference_linkage_demo(beta_deg)
    baseline_clearance = _build_clearance_report(vehicle, baseline_rig.spec, baseline_state)
    optimization_problem = build_reference_optimization_problem(mode=optimization_mode)
    optimization_result = optimize_linkage_problem(optimization_problem)
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
    )


def build_export_bundle(beta_deg: float, optimization_mode: str = "quick") -> dict[str, object]:
    context = build_export_context(beta_deg=beta_deg, optimization_mode=optimization_mode)
    return {
        "beta_deg": context.beta_deg,
        "optimization_mode": context.optimization_mode,
        "vehicle": _serialize_vehicle(context.vehicle),
        "ideal": {
            "wheel_angles_deg": context.ideal_solution.wheel_angles_deg(),
            "axle_center_angles_deg": context.ideal_solution.axle_center_angles_deg(),
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
        "comparison": {
            "metrics": _comparison_metric_rows(context),
            "changed_variables": _changed_variable_rows(context.optimization_result),
        },
    }


def build_export_json(beta_deg: float, optimization_mode: str = "quick") -> str:
    return json.dumps(build_export_bundle(beta_deg, optimization_mode), indent=2)


def build_export_csv(beta_deg: float, optimization_mode: str = "quick") -> str:
    context = build_export_context(beta_deg=beta_deg, optimization_mode=optimization_mode)
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(
        [
            "beta_deg",
            "ideal_front_left_deg",
            "ideal_front_right_deg",
            "ideal_rear_left_deg",
            "ideal_rear_right_deg",
            "baseline_actual_deg",
            "optimized_actual_deg",
            "baseline_error_deg",
            "optimized_error_deg",
            "baseline_clearance_mm",
            "optimized_clearance_mm",
            "baseline_status",
            "optimized_status",
        ]
    )

    optimized_spec = context.optimized_spec
    optimized_hint = build_branch_hint(optimized_spec)
    for sample_beta_deg in context.optimization_problem.beta_samples_deg:
        sample_vehicle, sample_ideal, _ = build_demo_solution(sample_beta_deg)
        baseline_state = solve_reference_linkage_demo(sample_beta_deg)
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

        ideal_front = sample_ideal.axles[-1]
        ideal_rear = sample_ideal.axles[0]
        baseline_error = baseline_state.steering_angle_deg - ideal_front.center_heading_deg
        optimized_error = "" if optimized_state is None else f"{optimized_state.steering_angle_deg - ideal_front.center_heading_deg:.2f}"

        writer.writerow(
            [
                f"{sample_beta_deg:.1f}",
                f"{ideal_front.left_wheel.heading_deg:.2f}",
                f"{ideal_front.right_wheel.heading_deg:.2f}",
                f"{ideal_rear.left_wheel.heading_deg:.2f}",
                f"{ideal_rear.right_wheel.heading_deg:.2f}",
                f"{baseline_state.steering_angle_deg:.2f}",
                "" if optimized_state is None else f"{optimized_state.steering_angle_deg:.2f}",
                f"{baseline_error:.2f}",
                optimized_error,
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


def build_export_dxf(beta_deg: float, optimization_mode: str = "quick") -> str:
    context = build_export_context(beta_deg=beta_deg, optimization_mode=optimization_mode)
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

    body_points = [
        Point2D(-context.vehicle.body_length_mm / 2.0, -context.vehicle.body_width_mm / 2.0),
        Point2D(context.vehicle.body_length_mm / 2.0, -context.vehicle.body_width_mm / 2.0),
        Point2D(context.vehicle.body_length_mm / 2.0, context.vehicle.body_width_mm / 2.0),
        Point2D(-context.vehicle.body_length_mm / 2.0, context.vehicle.body_width_mm / 2.0),
    ]
    entities.extend(_dxf_lwpolyline("BODY", body_points, closed=True))

    for axle in context.vehicle.axles:
        left, right = axle.wheels()
        entities.extend(_dxf_line("AXLE", left.center, right.center))
        entities.extend(_dxf_circle("AXLE", left.center, 45.0))
        entities.extend(_dxf_circle("AXLE", right.center, 45.0))

    for axle in context.ideal_solution.axles:
        for wheel in [axle.left_wheel, axle.right_wheel]:
            center = wheel.center
            end = Point2D(
                center.x_mm + math.cos(wheel.heading_rad) * 900.0,
                center.y_mm + math.sin(wheel.heading_rad) * 900.0,
            )
            entities.extend(_dxf_line("IDEAL", center, end))

    if context.ideal_solution.icr is not None:
        entities.extend(_dxf_circle("ICR", context.ideal_solution.icr, 85.0))

    entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3660.0), "Existing linkage", 58.0))
    for segment_start, segment_end in [
        (context.baseline_state.driver_point, context.baseline_state.input_endpoint),
        (context.baseline_rig.spec.bell_crank_pivot, context.baseline_state.input_endpoint),
        (context.baseline_rig.spec.bell_crank_pivot, context.baseline_state.output_endpoint),
        (context.baseline_state.output_endpoint, context.baseline_state.steering_endpoint),
        (context.baseline_rig.spec.steering_pivot, context.baseline_state.steering_endpoint),
    ]:
        entities.extend(_dxf_line("BASELINE", segment_start, segment_end))
    entities.extend(_dxf_circle("BASELINE", context.baseline_rig.spec.bell_crank_pivot, 28.0))
    entities.extend(_dxf_circle("BASELINE", context.baseline_rig.spec.steering_pivot, 28.0))

    if context.optimized_state is not None:
        entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3480.0), "Optimized linkage", 58.0))
        for segment_start, segment_end in [
            (context.optimized_state.driver_point, context.optimized_state.input_endpoint),
            (context.optimized_spec.bell_crank_pivot, context.optimized_state.input_endpoint),
            (context.optimized_spec.bell_crank_pivot, context.optimized_state.output_endpoint),
            (context.optimized_state.output_endpoint, context.optimized_state.steering_endpoint),
            (context.optimized_spec.steering_pivot, context.optimized_state.steering_endpoint),
        ]:
            entities.extend(_dxf_line("OPTIMIZED", segment_start, segment_end))
        entities.extend(_dxf_circle("OPTIMIZED", context.optimized_spec.bell_crank_pivot, 28.0))
        entities.extend(_dxf_circle("OPTIMIZED", context.optimized_spec.steering_pivot, 28.0))
    else:
        entities.extend(_dxf_text("ANNOTATION", Point2D(-3800.0, 3480.0), "Optimized linkage unavailable", 58.0))

    lines.extend(_dxf_section("ENTITIES", entities))
    lines.extend(["0", "EOF"])
    return "\n".join(lines)


@lru_cache(maxsize=8)
def build_steering_sweep_bundle(optimization_mode: str = "quick", step_deg: float = 1.0) -> dict[str, object]:
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")

    context = build_export_context(beta_deg=0.0, optimization_mode=optimization_mode)
    optimized_spec = context.optimized_spec
    optimized_hint = build_branch_hint(optimized_spec)

    beta_min_deg = -45.0
    beta_max_deg = 45.0
    sample_count = int(round((beta_max_deg - beta_min_deg) / step_deg)) + 1

    samples: list[dict[str, object]] = []
    baseline_errors: list[float] = []
    optimized_errors: list[float] = []
    baseline_clearances: list[float] = []
    optimized_clearances: list[float] = []

    for index in range(sample_count):
        beta_deg = beta_min_deg + index * step_deg
        if beta_deg > beta_max_deg + 1e-9:
            break

        vehicle, ideal_solution, _ = build_demo_solution(beta_deg)
        baseline_state = solve_reference_linkage_demo(beta_deg)
        baseline_clearance = _build_clearance_report(vehicle, context.baseline_rig.spec, baseline_state)

        driver_point = driver_point_arc(
            context.optimization_problem.base_rig.driver_arc_center,
            context.optimization_problem.base_rig.driver_arc_radius_mm,
            math.radians(beta_deg),
        )
        try:
            optimized_state = solve_planar_linkage(optimized_spec, driver_point, branch_hint=optimized_hint)
            optimized_clearance = _build_clearance_report(vehicle, optimized_spec, optimized_state)
        except Exception:
            optimized_state = None
            optimized_clearance = None

        ideal_front = ideal_solution.axles[-1]
        ideal_rear = ideal_solution.axles[0]
        baseline_error = baseline_state.steering_angle_deg - ideal_front.center_heading_deg
        optimized_error = None if optimized_state is None else optimized_state.steering_angle_deg - ideal_front.center_heading_deg

        samples.append(
            {
                "beta_deg": beta_deg,
                "ideal_front_left_deg": ideal_front.left_wheel.heading_deg,
                "ideal_front_right_deg": ideal_front.right_wheel.heading_deg,
                "ideal_rear_left_deg": ideal_rear.left_wheel.heading_deg,
                "ideal_rear_right_deg": ideal_rear.right_wheel.heading_deg,
                "baseline_steer_deg": baseline_state.steering_angle_deg,
                "optimized_steer_deg": None if optimized_state is None else optimized_state.steering_angle_deg,
                "baseline_error_deg": baseline_error,
                "optimized_error_deg": optimized_error,
                "baseline_clearance_mm": baseline_clearance.minimum_clearance_mm,
                "optimized_clearance_mm": None if optimized_clearance is None else optimized_clearance.minimum_clearance_mm,
                "baseline_status": "OK",
                "optimized_status": "MECHANISM_INVALID" if optimized_state is None else "OK",
            }
        )

        baseline_errors.append(baseline_error)
        baseline_clearance_value = baseline_clearance.minimum_clearance_mm
        if baseline_clearance_value is not None:
            baseline_clearances.append(baseline_clearance_value)
        if optimized_error is not None:
            optimized_errors.append(optimized_error)
        optimized_clearance_value = None if optimized_clearance is None else optimized_clearance.minimum_clearance_mm
        if optimized_clearance_value is not None:
            optimized_clearances.append(optimized_clearance_value)

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
            "baseline_min_clearance_mm": None if not baseline_clearances else min(baseline_clearances),
            "optimized_min_clearance_mm": None if not optimized_clearances else min(optimized_clearances),
            "baseline_valid_samples": len(baseline_errors),
            "optimized_valid_samples": len(optimized_errors),
        },
    }


def build_steering_curves_svg(current_beta_deg: float, optimization_mode: str = "quick", step_deg: float = 1.0) -> str:
    sweep = build_steering_sweep_bundle(optimization_mode=optimization_mode, step_deg=step_deg)
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

    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-label="Steering curves">',
        f"<style>{styles}</style>",
        f'<rect x="0" y="0" width="{width}" height="{height}" class="canvas" />',
        _svg_text(56.0, 92.0, f"Beta sweep {beta_min_deg:.0f} to {beta_max_deg:.0f} deg | {step_deg:.1f} deg step | {optimization_mode.title()} optimization", "subtitle"),
        _svg_text(
            56.0,
            128.0,
            f"Baseline RMS {summary['baseline_rms_error_deg']:.2f} deg | Optimized RMS {summary['optimized_rms_error_deg']:.2f} deg | Current beta {current_beta_deg:.1f} deg",
            "summary",
        ),
    ]

    # Top panel: steering angle curves.
    pieces.append(f'<rect x="{panel_x}" y="{top_panel_y}" width="{panel_w}" height="{top_panel_h}" rx="28" class="panel" />')
    pieces.append(_svg_text(panel_x + 26.0, top_panel_y + 38.0, "Wheel and linkage angles", "title"))
    pieces.append(_svg_text(panel_x + 26.0, top_panel_y + 74.0, "Solid curves show the ideal wheel headings. Linkage curves show the real mechanism response.", "subtitle"))
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
    pieces.append(_svg_text(panel_x + 26.0, bottom_panel_y + 38.0, "Front-axle steering error", "title"))
    pieces.append(_svg_text(panel_x + 26.0, bottom_panel_y + 74.0, "Error = actual linkage steer minus ideal front-axle center heading.", "subtitle"))
    pieces.extend(render_grid(bottom_y_min, bottom_y_max, bottom_plot_y, bottom_plot_h, tick_step=max(1.0, round(bottom_y_extent / 3.0))))
    for key, stroke, dash, width_value in [
        ("baseline_error_deg", "#ff9d56", "10 8", 4.0),
        ("optimized_error_deg", "#8ef2c2", None, 5.0),
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

    body_half_length = vehicle.body_length_mm / 2.0
    body_half_width = vehicle.body_width_mm / 2.0
    local_body = (
        Point2D(-body_half_length, -body_half_width),
        Point2D(body_half_length, -body_half_width),
        Point2D(body_half_length, body_half_width),
        Point2D(-body_half_length, body_half_width),
    )
    body_outline = _transform_points(local_body, origin, heading_rad)

    wheel_centers = [
        {
            "wheel_id": wheel.id,
            "axle_id": wheel.axle_id,
            "side": wheel.side,
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


@lru_cache(maxsize=8)
def build_swept_path_bundle(current_beta_deg: float, optimization_mode: str = "quick", step_deg: float = 1.0) -> dict[str, object]:
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")

    vehicle = build_reference_demo_layout()
    reference_length_mm = vehicle.axle_span_mm() or 4360.0
    beta_min_deg = -45.0
    beta_max_deg = 45.0
    sample_count = int(round((beta_max_deg - beta_min_deg) / step_deg)) + 1

    samples: list[dict[str, object]] = []
    left_points: list[Point2D] = []
    right_points: list[Point2D] = []
    all_points: list[Point2D] = []

    for index in range(sample_count):
        beta_deg = beta_min_deg + index * step_deg
        if beta_deg > beta_max_deg + 1e-9:
            break
        pose = _swept_path_pose(vehicle, beta_deg, reference_length_mm)
        samples.append(pose)
        body_outline = tuple(Point2D(point["x_mm"], point["y_mm"]) for point in pose["body_outline"])
        all_points.extend(body_outline)
        if beta_deg >= 0:
            left_points.extend(body_outline)
        if beta_deg <= 0:
            right_points.extend(body_outline)

    current_pose = _swept_path_pose(vehicle, current_beta_deg, reference_length_mm)
    current_points = tuple(Point2D(point["x_mm"], point["y_mm"]) for point in current_pose["body_outline"])
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
            "id": vehicle.id,
            "name": vehicle.name,
            "body_length_mm": vehicle.body_length_mm,
            "body_width_mm": vehicle.body_width_mm,
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


def build_swept_path_svg(current_beta_deg: float, optimization_mode: str = "quick", step_deg: float = 1.0) -> str:
    bundle = build_swept_path_bundle(current_beta_deg=current_beta_deg, optimization_mode=optimization_mode, step_deg=step_deg)
    vehicle = build_reference_demo_layout()
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

    wheel_ids = [wheel.id for wheel in vehicle.wheels()]
    wheel_paths: dict[str, list[Point2D]] = {wheel_id: [] for wheel_id in wheel_ids}
    axle_paths: dict[str, list[Point2D]] = {axle.id: [] for axle in vehicle.axles}
    for pose in samples:
        origin = Point2D(pose["origin"]["x_mm"], pose["origin"]["y_mm"])
        heading_rad = float(pose["heading_rad"])
        for wheel in vehicle.wheels():
            wheel_paths[wheel.id].append(origin + wheel.center.rotated_ccw(heading_rad))
        for axle in vehicle.axles:
            axle_paths[axle.id].append(origin + axle.center.rotated_ccw(heading_rad))

    wheel_colors = ["#f4b860", "#ffd799", "#72e5ff", "#9db4ff"]
    for index, wheel_id in enumerate(wheel_ids):
        pieces.append(render_path(wheel_paths[wheel_id], stroke=wheel_colors[index % len(wheel_colors)], width_value=3.0, dash="8 8"))

    for axle in vehicle.axles:
        pieces.append(render_path(axle_paths[axle.id], stroke="#69d39d", width_value=2.5, dash="14 10"))

    pieces.append(f'<path d="{polygon_path(current_outline)}" class="sweep-current" />')
    pieces.append(render_text(current_origin, f"beta {current_beta_deg:.0f}", "sweep-value"))
    pieces.append(render_circle(current_origin, 18.0, "sweep-current-origin"))
    for wheel in vehicle.wheels():
        point = current_origin + wheel.center.rotated_ccw(float(bundle["current_pose"]["heading_rad"]))
        pieces.append(render_circle(point, 10.0, "sweep-wheel"))

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
        ("Baseline clearance", "n/a" if context.baseline_clearance.minimum_clearance_mm is None else _format_mm(context.baseline_clearance.minimum_clearance_mm)),
        ("Optimized clearance", optimized_clearance),
        ("Run stats", f"{context.optimization_result.mode} / {context.optimization_result.iterations} it / {context.optimization_result.evaluations} eval"),
    ]


def _pdf_change_rows(result: OptimizationResult) -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    for variable in result.optimized_variables:
        if abs(variable.delta) < 1e-9:
            continue
        rows.append((variable.id, f"{variable.current:.2f} -> {variable.optimized:.2f} ({variable.delta:+.2f})"))
    return rows


def _pdf_draw_linkage_schematic(pdf: canvas.Canvas, context: ExportContext, x: float, y: float, width: float, height: float) -> None:
    baseline_spec = context.baseline_rig.spec
    optimized_spec = context.optimized_spec
    baseline_state = context.baseline_state
    optimized_state = context.optimized_state or context.baseline_state

    body_half_length = context.vehicle.body_length_mm / 2.0
    body_half_width = context.vehicle.body_width_mm / 2.0
    points = [
        Point2D(-body_half_length, -body_half_width),
        Point2D(body_half_length, -body_half_width),
        Point2D(body_half_length, body_half_width),
        Point2D(-body_half_length, body_half_width),
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

    body_points = [
        Point2D(-body_half_length, -body_half_width),
        Point2D(body_half_length, -body_half_width),
        Point2D(body_half_length, body_half_width),
        Point2D(-body_half_length, body_half_width),
    ]
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
    pdf.drawString(x + 12, y + height - 30, "Baseline dashed; optimized solid; body rectangle shown at the current beta.")

    # Baseline overlay.
    polygon(baseline_body, "#7c8da6", "#0d1827", width_value=1.0, dash=(6, 4))
    line(baseline_state.driver_point, baseline_state.input_endpoint, "#9aa7b8", 1.2, (6, 4))
    line(baseline_spec.bell_crank_pivot, baseline_state.input_endpoint, "#9aa7b8", 1.2, (6, 4))
    line(baseline_spec.bell_crank_pivot, baseline_state.output_endpoint, "#9aa7b8", 1.2, (6, 4))
    line(baseline_state.output_endpoint, baseline_state.steering_endpoint, "#9aa7b8", 1.2, (6, 4))
    line(baseline_spec.steering_pivot, baseline_state.steering_endpoint, "#9aa7b8", 1.2, (6, 4))

    # Optimized overlay.
    line(optimized_state.driver_point, optimized_state.input_endpoint, "#72e5ff", 1.6)
    line(optimized_spec.bell_crank_pivot, optimized_state.input_endpoint, "#f4b860", 1.6)
    line(optimized_spec.bell_crank_pivot, optimized_state.output_endpoint, "#f4b860", 1.6)
    line(optimized_state.output_endpoint, optimized_state.steering_endpoint, "#69d39d", 1.6)
    line(optimized_spec.steering_pivot, optimized_state.steering_endpoint, "#69d39d", 1.6)
    polygon(optimized_body, "#72e5ff", "#0a1726", width_value=1.0)

    # Pivots and nodes.
    for center in [baseline_spec.bell_crank_pivot, baseline_spec.steering_pivot, baseline_state.input_endpoint, baseline_state.output_endpoint, baseline_state.steering_endpoint]:
        circle(center, 16.0, "#93a2b5", fill=True, width_value=0.8)
    for center in [optimized_spec.bell_crank_pivot, optimized_spec.steering_pivot, optimized_state.input_endpoint, optimized_state.output_endpoint, optimized_state.steering_endpoint]:
        circle(center, 14.0, "#f4b860", fill=True, width_value=0.8)

    # Labels.
    pdf.setFillColor(colors.HexColor("#b9c7d8"))
    pdf.setFont("Helvetica", 8)
    bx, by = project(baseline_spec.bell_crank_pivot)
    sx, sy = project(baseline_spec.steering_pivot)
    pdf.drawString(bx + 8, by + 8, "Baseline pivot")
    pdf.drawString(sx + 8, sy + 8, "Baseline knuckle")
    bx, by = project(optimized_spec.bell_crank_pivot)
    sx, sy = project(optimized_spec.steering_pivot)
    pdf.setFillColor(colors.HexColor("#72e5ff"))
    pdf.drawString(bx + 8, by - 10, "Optimized pivot")
    pdf.drawString(sx + 8, sy - 10, "Optimized knuckle")


def _pdf_draw_metric_table(pdf: canvas.Canvas, x: float, y: float, title: str, rows: list[tuple[str, str]], width: float) -> float:
    pdf.setFillColor(colors.HexColor("#e7eef7"))
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(x, y, title)
    return _pdf_draw_kv_rows(pdf, x, y - 18, rows, label_width=width * 0.53, value_width=width * 0.47, row_height=17.0, font_size=8.8)


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


def build_export_pdf(beta_deg: float, optimization_mode: str = "quick") -> bytes:
    context = build_export_context(beta_deg=beta_deg, optimization_mode=optimization_mode)
    sweep = build_steering_sweep_bundle(optimization_mode=optimization_mode, step_deg=15.0)
    swept = build_swept_path_bundle(current_beta_deg=beta_deg, optimization_mode=optimization_mode, step_deg=1.0)

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
    _pdf_draw_metric_table(pdf, left_x, cursor_y, "Changed dimensions", _pdf_change_rows(context.optimization_result), left_width)

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

    pdf.setFillColor(colors.HexColor("#96a8be"))
    pdf.setFont("Helvetica", 8)
    pdf.drawString(36, 28, "Swept-path preview uses the current beta surrogate and traces the body extents plus wheel-center trajectories.")

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
    pieces.append(_svg_circle(state.driver_point, 18.0, f"{prefix}-driver"))
    pieces.append(_svg_circle(state.input_endpoint, 18.0, f"{prefix}-node"))
    pieces.append(_svg_circle(state.output_endpoint, 18.0, f"{prefix}-node"))
    pieces.append(_svg_circle(state.steering_endpoint, 18.0, f"{prefix}-node"))
    pieces.append(_svg_circle(spec.bell_crank_pivot, 24.0, f"{prefix}-pivot"))
    pieces.append(_svg_circle(spec.steering_pivot, 24.0, f"{prefix}-pivot"))
    pieces.append(_svg_text(spec.bell_crank_pivot.x_mm + 40, -spec.bell_crank_pivot.y_mm - 30, f"{label} bell crank", f"{prefix}-label"))
    pieces.append(_svg_text(spec.steering_pivot.x_mm + 40, -spec.steering_pivot.y_mm - 30, f"{label} knuckle", f"{prefix}-label"))
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


def _comparison_metric_rows(context: ExportContext) -> list[tuple[str, str, str]]:
    baseline = context.optimization_result.baseline_metrics
    optimized = context.optimization_result.optimized_metrics
    optimized_state = context.optimized_state
    optimized_clearance = context.optimized_clearance.minimum_clearance_mm if context.optimized_clearance else None
    return [
        ("Max Ackermann error", _format_deg(baseline.max_abs_error_deg), _format_deg(optimized.max_abs_error_deg)),
        ("RMS error", _format_deg(baseline.rms_error_deg), _format_deg(optimized.rms_error_deg)),
        ("Minimum clearance", "n/a" if context.baseline_clearance.minimum_clearance_mm is None else _format_mm(context.baseline_clearance.minimum_clearance_mm), "n/a" if optimized_clearance is None else _format_mm(optimized_clearance)),
        ("Optimization score", f"{baseline.score:.2f}", f"{optimized.score:.2f}"),
        ("Actual steer", _format_deg(context.baseline_state.steering_angle_deg), "n/a" if optimized_state is None else _format_deg(optimized_state.steering_angle_deg)),
    ]


def _changed_variable_rows(result: OptimizationResult) -> list[tuple[str, float, float, float]]:
    return [(before.id, before.current, after.optimized, after.delta) for before, after in zip(result.baseline_variables, result.optimized_variables)]


def build_dimensioned_svg(beta_deg: float, optimization_mode: str = "quick") -> str:
    context = build_export_context(beta_deg=beta_deg, optimization_mode=optimization_mode)
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
      .baseline-arm, .baseline-arm-2, .baseline-tie, .baseline-steer { stroke: rgba(180,190,200,0.45); stroke-width: 10; stroke-dasharray: 24 16; }
      .baseline-node, .baseline-driver, .baseline-pivot { fill: rgba(220,225,230,0.8); stroke: #08111d; stroke-width: 6; }
      .baseline-label { fill: #b9c7d8; font-size: 70px; font-family: "Bahnschrift", "Segoe UI", sans-serif; }
      .optimized-rod { stroke: rgba(114,229,255,0.95); stroke-width: 10; }
      .optimized-arm, .optimized-arm-2 { stroke: rgba(244,184,96,0.95); stroke-width: 12; }
      .optimized-tie { stroke: rgba(105,211,157,0.95); stroke-width: 10; }
      .optimized-steer { stroke: rgba(255,125,125,0.95); stroke-width: 11; }
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

    pieces.append(_svg_polygon([
        Point2D(-context.vehicle.body_length_mm / 2.0, -context.vehicle.body_width_mm / 2.0),
        Point2D(context.vehicle.body_length_mm / 2.0, -context.vehicle.body_width_mm / 2.0),
        Point2D(context.vehicle.body_length_mm / 2.0, context.vehicle.body_width_mm / 2.0),
        Point2D(-context.vehicle.body_length_mm / 2.0, context.vehicle.body_width_mm / 2.0),
    ], "body"))

    for axle in context.vehicle.axles:
        left, right = axle.wheels()
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
