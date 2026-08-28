from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import hmac
import json
import math
import mimetypes
import os
from urllib.parse import parse_qs, urlparse
from uuid import uuid4

from .acceptance import (
    ACCEPTANCE_EVALUATOR_ID,
    MonrocAcceptanceCriteria,
    evaluate_monroc_acceptance,
)
from .collision import (
    CapsuleEnvelope,
    CircleEnvelope,
    CollisionItem,
    ClearancePair,
    ClearanceReport,
    PolygonEnvelope,
    analyze_clearance,
)
from .clearance_model import (
    build_combination_body_clearance_items,
    build_linkage_clearance_items,
    build_mechanism_graph_clearance_items,
)
from .combination_kinematics import CombinationKinematicSolution, solve_combination_kinematics
from .combination_sweep import (
    JointSweepRange,
    build_joint_sweep_grid,
    normalize_joint_sweep_ranges,
)
from .geometry import Point2D
from .design_cases import DesignCase
from .errors import (
    ArticulationLimitExceededError,
    EngineeringError,
    OptimizationNoFeasibleSolutionError,
    SweepSampleLimitError,
)
from .graph_optimization import (
    build_mechanism_graph_optimization_problem,
    optimize_mechanism_graph_problem,
)
from .dxf_import import (
    DXF_COORDINATE_OPTIONS,
    DXF_UNIT_OPTIONS,
    analyze_dxf_import,
    apply_dxf_role_overrides,
    serialize_dxf_import_report,
)
from .linkage import (
    LinkageDemoRig,
    PlanarLinkageSpec,
    build_linkage_rig,
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage,
)
from .mechanism_graph import (
    MechanismAngleOutput,
    MechanismDriverArc,
    MechanismGraphState,
    MechanismPoint,
    MechanismSteeringAssignment,
    PlanarMechanismGraph,
    RigidMember,
    resolve_driver_arc_positions,
    solve_mechanism_graph,
)
from .model import (
    ArticulationJoint,
    Axle,
    MountedAxle,
    Pose2D,
    RigidBody,
    SteeringSynchronization,
    SteeringTargetPoint,
    VehicleCombination,
    VehicleLayout,
    build_reference_demo_combination,
    serialize_vehicle_combination,
)
from .actual_steering import (
    compare_actual_to_ideal,
    serialize_actual_steering,
    solve_actual_steering,
    solve_actual_steering_from_graph,
)
from .optimization import (
    OptimizationMetrics,
    OptimizationWeights,
    OptimizedVariable,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from .projects import PostgreSQLProjectStore, ProjectStore, serialize_project, serialize_revision
from .reporting import (
    build_dimensioned_svg,
    build_engineering_snapshot_dxf,
    build_engineering_snapshot_png,
    build_engineering_snapshot_svg,
    build_engineering_snapshot_csv,
    build_engineering_snapshot_pdf,
    build_export_bundle,
    build_export_csv,
    build_export_dxf,
    build_export_pdf,
    build_export_png,
    build_steering_curves_svg,
    build_swept_path_svg,
    engineering_failure_guidance,
    evaluate_engineering_snapshot,
)
from .saas import (
    ApprovalStatus,
    ArtifactStorageError,
    EngineeringJobRunner,
    FileArtifactStore,
    Principal,
    PostgreSQLSaaSStore,
    SaaSAuthorizationError,
    SaaSBootstrapError,
    SaaSControlStore,
    UserRole,
    principal_payload,
    serialize_approval,
    serialize_artifact,
    serialize_audit_event,
    serialize_job,
    serialize_user,
)
from .steering import beta_to_reference_radius_mm, build_demo_solution, solve_ideal_steering_from_radius

WEB_DIR = Path(__file__).resolve().parent / "web"
DATABASE_URL = os.environ.get("EASYTOWING_DATABASE_URL", "").strip()
ARTIFACT_STORAGE_DIR = os.environ.get("EASYTOWING_ARTIFACT_STORAGE_DIR", "").strip()
ARTIFACT_STORAGE_REQUIRED = os.environ.get(
    "EASYTOWING_REQUIRE_ARTIFACT_STORAGE",
    "0",
).strip().lower() in {"1", "true", "yes"}
WORKER_REQUIRED = os.environ.get(
    "EASYTOWING_REQUIRE_WORKER",
    "0",
).strip().lower() in {"1", "true", "yes"}


def _positive_env_float(name: str, default: float) -> float:
    raw_value = os.environ.get(name, "").strip()
    if not raw_value:
        return default
    try:
        value = float(raw_value)
    except ValueError:
        return default
    return value if math.isfinite(value) and value > 0.0 else default


def _parse_required_bool(body: dict[str, object], name: str) -> bool:
    """Read a protocol boolean without Python truthiness coercion."""

    value = body.get(name)
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a JSON boolean.")
    return value


WORKER_MAX_AGE_SECONDS = _positive_env_float(
    "EASYTOWING_WORKER_MAX_AGE_SECONDS",
    120.0,
)
MAX_CAD_SOURCE_BYTES = 10 * 1024 * 1024
ARTIFACT_BLOB_STORE = (
    FileArtifactStore(ARTIFACT_STORAGE_DIR)
    if ARTIFACT_STORAGE_DIR
    else None
)
if DATABASE_URL:
    PROJECT_STORE = PostgreSQLProjectStore(DATABASE_URL)
    SAAS_CONTROL = PostgreSQLSaaSStore(DATABASE_URL)
    SAAS_CONTROL.migrate()
else:
    PROJECT_STORE = ProjectStore.default()
    SAAS_CONTROL = SaaSControlStore()
# Database mode is enqueue-only; a separate worker process owns execution so
# queued work survives an API restart. Local mode keeps the lightweight runner
# for development and tests.
SAAS_JOBS = None if DATABASE_URL else EngineeringJobRunner(SAAS_CONTROL)
SAAS_AUTH_REQUIRED = bool(DATABASE_URL) or os.environ.get("EASYTOWING_AUTH_REQUIRED", "0").strip().lower() in {"1", "true", "yes"}
LOCAL_DEVELOPER = Principal(
    user_id="local_developer",
    organization_id="local_development",
    email="local@development.invalid",
    role=UserRole.ADMIN,
    display_name="Local developer",
)


def _point_payload(point: Point2D) -> dict[str, float]:
    return {"x_mm": point.x_mm, "y_mm": point.y_mm}


def _wheel_payload(wheel_solution) -> dict[str, object]:
    payload = {
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
    return payload


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
        "wheels": [
            _wheel_payload(wheel)
            for wheel in axle_solution.wheel_solutions
        ],
    }
    if axle is not None:
        payload.update(
            {
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
                "wheel_count": axle.wheel_count,
                "wheel_lateral_offsets_mm": (
                    None
                    if axle.wheel_lateral_offsets_mm is None
                    else list(axle.wheel_lateral_offsets_mm)
                ),
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


def _mechanism_driver_payload(driver: MechanismDriverArc) -> dict[str, object]:
    return {
        "point_id": driver.point_id,
        "center": _point_payload(driver.center),
        "radius_mm": driver.radius_mm,
        "neutral_angle_rad": driver.neutral_angle_rad,
        "neutral_angle_deg": math.degrees(driver.neutral_angle_rad),
        "input_ratio": driver.input_ratio,
        "phase_offset_rad": driver.phase_offset_rad,
        "phase_offset_deg": math.degrees(driver.phase_offset_rad),
        "input_id": driver.input_id,
    }


def _mechanism_assignment_payload(
    assignment: MechanismSteeringAssignment,
) -> dict[str, object]:
    return {
        "output_id": assignment.output_id,
        "wheel_id": assignment.wheel_id,
        "ratio": assignment.ratio,
        "phase_offset_rad": assignment.phase_offset_rad,
        "phase_offset_deg": math.degrees(assignment.phase_offset_rad),
    }


def _mechanism_graph_optimization_payload(
    *,
    mode: str,
    combination: VehicleCombination,
    graph: PlanarMechanismGraph,
    drivers: tuple[MechanismDriverArc, ...],
    assignments: tuple[MechanismSteeringAssignment, ...],
    beta_min_deg: float,
    beta_max_deg: float,
    primary_joint_id: str | None,
    root_turn_radius_mm: float | None,
    clearance_target_mm: float,
    enabled_ids: set[str] | None,
    weights: OptimizationWeights,
    design_cases: tuple[DesignCase, ...] | None,
    joint_ranges: object | None = None,
    graph_result=None,
) -> dict[str, object]:
    problem = build_mechanism_graph_optimization_problem(
        combination=combination,
        graph=graph,
        drivers=drivers,
        assignments=assignments,
        beta_min_deg=beta_min_deg,
        beta_max_deg=beta_max_deg,
        mode=mode,  # type: ignore[arg-type]
        primary_joint_id=primary_joint_id,
        root_turn_radius_mm=root_turn_radius_mm,
        clearance_target_mm=clearance_target_mm,
        weights=weights,
        enabled_ids=enabled_ids,
        design_cases=design_cases,
        joint_ranges=joint_ranges,
    )
    if graph_result is None:
        graph_result = optimize_mechanism_graph_problem(problem)
    result = graph_result.result
    primary_range = next(
        item for item in problem.joint_ranges if item.joint_id == problem.primary_joint_id
    )
    return {
        "mode": result.mode,
        "graph_native": True,
        "iterations": result.iterations,
        "evaluations": result.evaluations,
        "improved": result.improved,
        "improvement": result.improvement,
        "baseline": _optimization_metrics_payload(result.baseline_metrics),
        "optimized": _optimization_metrics_payload(result.optimized_metrics),
        "objective": {
            "clearance_target_mm": result.clearance_target_mm,
            "weights": result.weights.to_dict(),
            "beta_min_deg": primary_range.minimum_deg,
            "beta_max_deg": primary_range.maximum_deg,
            "primary_joint_id": problem.primary_joint_id,
            "joint_ranges": [item.to_dict() for item in problem.joint_ranges],
            "joint_sample_count": len(problem.joint_sample_values()),
            "root_turn_radius_mm": root_turn_radius_mm,
        },
        "design_cases": [case.to_dict() for case in result.design_cases],
        "variables_before": [
            _optimized_variable_payload(variable)
            for variable in result.baseline_variables
        ],
        "variables_after": [
            _optimized_variable_payload(variable)
            for variable in result.optimized_variables
        ],
        "mechanism_drivers_before": [
            _mechanism_driver_payload(driver)
            for driver in graph_result.baseline_drivers
        ],
        "mechanism_drivers_after": [
            _mechanism_driver_payload(driver)
            for driver in graph_result.optimized_drivers
        ],
        "steering_assignments_before": [
            _mechanism_assignment_payload(assignment)
            for assignment in graph_result.baseline_assignments
        ],
        "steering_assignments_after": [
            _mechanism_assignment_payload(assignment)
            for assignment in graph_result.optimized_assignments
        ],
    }


def _optimization_job_payload(request: dict[str, object]) -> dict[str, object]:
    """Run the same optimizer selected by a queued optimization request."""

    mode = str(request.get("mode", "quick"))
    if mode not in {"quick", "full"}:
        raise ValueError("mode must be quick or full")
    raw_combination = request.get("combination", request.get("combination_config"))
    if raw_combination is not None:
        combination = _parse_vehicle_combination(raw_combination)
        raw_graph = request.get("mechanism_graph", request.get("mechanism_graph_config"))
        if raw_graph is None:
            raise ValueError("A graph optimization request requires mechanism_graph.")
        graph = _parse_mechanism_graph(raw_graph)
        drivers = _parse_mechanism_drivers(request.get("mechanism_drivers"))
        assignments = _parse_steering_assignments(request.get("steering_assignments"))
        beta_min_deg, beta_max_deg = _parse_articulation_bounds(request)
        root_turn_radius_mm = _config_float(
            request,
            "root_turn_radius_mm",
            None,
            allow_none=True,
        )
        return _mechanism_graph_optimization_payload(
            mode=mode,
            combination=combination,
            graph=graph,
            drivers=drivers,
            assignments=assignments,
            beta_min_deg=beta_min_deg,
            beta_max_deg=beta_max_deg,
            primary_joint_id=(
                None
                if request.get("primary_joint_id") in (None, "")
                else str(request["primary_joint_id"])
            ),
            root_turn_radius_mm=root_turn_radius_mm,
            clearance_target_mm=float(request.get("clearance_target_mm", 20.0)),
            enabled_ids=_parse_enabled_ids(request.get("enabled_ids")),
            weights=OptimizationWeights(
                steering_error=float(request.get("steering_error_weight", 1.0)),
                synchronization_error=float(request.get("synchronization_error_weight", 0.5)),
                clearance=float(request.get("clearance_weight", 12.0)),
                clearance_violation=float(request.get("clearance_violation_weight", 250.0)),
                failure=float(request.get("failure_weight", 100000.0)),
                preferred=float(request.get("preferred_weight", 0.05)),
                complexity=float(request.get("complexity_weight", 0.02)),
            ),
            design_cases=_parse_design_cases(request.get("design_cases")),
            joint_ranges=request.get(
                "joint_ranges",
                raw_combination.get("joint_ranges")
                if isinstance(raw_combination, dict)
                else None,
            ),
        )
    return _optimization_payload(
        mode,
        enabled_ids=_parse_enabled_ids(request.get("enabled_ids")),
        clearance_target_mm=float(request.get("clearance_target_mm", 20.0)),
        design_cases=_parse_design_cases(request.get("design_cases")),
    )


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


def _parse_joint_sweep_ranges(
    combination: VehicleCombination,
    raw_ranges: object | None,
    *,
    beta_min_deg: float,
    beta_max_deg: float,
    step_deg: float,
    primary_joint_id: str | None,
) -> tuple[JointSweepRange, ...]:
    """Normalize explicit per-joint ranges, retaining the legacy beta alias."""

    if raw_ranges is None:
        configured_ranges = {
            joint.id: {
                "min_deg": joint.sweep_min_deg,
                "max_deg": joint.sweep_max_deg,
                "step_deg": joint.sweep_step_deg,
            }
            for joint in combination.joints
            if joint.sweep_min_deg is not None
            and joint.sweep_max_deg is not None
            and joint.sweep_step_deg is not None
        }
        raw_ranges = configured_ranges or None
    return normalize_joint_sweep_ranges(
        (joint.id for joint in combination.joints),
        raw_ranges,
        default_min_deg=beta_min_deg,
        default_max_deg=beta_max_deg,
        default_step_deg=step_deg,
        primary_joint_id=primary_joint_id,
    )


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


def _config_wheel_lateral_offsets(
    raw: dict[str, object],
) -> tuple[float, ...] | None:
    value = raw.get("wheel_lateral_offsets_mm")
    if value is None:
        return None
    if not isinstance(value, list):
        raise ValueError("wheel_lateral_offsets_mm must be an array")
    try:
        return tuple(float(offset) for offset in value)
    except (TypeError, ValueError) as error:
        raise ValueError("wheel_lateral_offsets_mm must contain numeric values") from error


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


def _optional_config_angle(raw: dict[str, object], name: str) -> float | None:
    if f"{name}_deg" not in raw and f"{name}_rad" not in raw:
        return None
    return _config_angle(raw, name, 0.0)


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
                wheel_lateral_offsets_mm=_config_wheel_lateral_offsets(raw_axle),
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


def _parse_mechanism_graph(raw_graph: object) -> PlanarMechanismGraph:
    if not isinstance(raw_graph, dict):
        raise ValueError("mechanism must be an object")
    raw_points = raw_graph.get("points")
    raw_members = raw_graph.get("members")
    raw_outputs = raw_graph.get("angle_outputs", [])
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("mechanism.points must be a non-empty array")
    if not isinstance(raw_members, list):
        raise ValueError("mechanism.members must be an array")
    if not isinstance(raw_outputs, list):
        raise ValueError("mechanism.angle_outputs must be an array")

    points: list[MechanismPoint] = []
    for index, raw_point in enumerate(raw_points):
        if not isinstance(raw_point, dict):
            raise ValueError(f"mechanism.points[{index}] must be an object")
        position = _config_point(raw_point, "neutral_position", None)
        if position is None and "x_mm" in raw_point and "y_mm" in raw_point:
            position = _config_point(
                {"position": {"x_mm": raw_point["x_mm"], "y_mm": raw_point["y_mm"]}},
                "position",
                None,
            )
        if position is None:
            raise ValueError(f"mechanism.points[{index}] requires neutral_position")
        mode = str(raw_point.get("mode", "free"))
        if mode not in {"fixed", "driven", "free"}:
            raise ValueError(f"mechanism.points[{index}].mode is invalid")
        points.append(
            MechanismPoint(
                id=str(raw_point.get("id", "")),
                neutral_position=position,
                mode=mode,  # type: ignore[arg-type]
                envelope_radius_mm=float(raw_point.get("envelope_radius_mm", 0.0)),
                body_id=(
                    None
                    if raw_point.get("body_id") in (None, "")
                    else str(raw_point["body_id"])
                ),
            )
        )

    members: list[RigidMember] = []
    for index, raw_member in enumerate(raw_members):
        if not isinstance(raw_member, dict):
            raise ValueError(f"mechanism.members[{index}] must be an object")
        kind = str(raw_member.get("kind", "rod"))
        if kind not in {"arm", "rod", "rigid_brace"}:
            raise ValueError(f"mechanism.members[{index}].kind is invalid")
        try:
            members.append(
                RigidMember(
                    id=str(raw_member["id"]),
                    point_a_id=str(raw_member["point_a_id"]),
                    point_b_id=str(raw_member["point_b_id"]),
                    length_mm=float(raw_member["length_mm"]),
                    kind=kind,  # type: ignore[arg-type]
                    envelope_radius_mm=float(raw_member.get("envelope_radius_mm", 0.0)),
                    assembly_id=(
                        None
                        if raw_member.get("assembly_id") in (None, "")
                        else str(raw_member["assembly_id"])
                    ),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"mechanism.members[{index}] is invalid") from error

    outputs: list[MechanismAngleOutput] = []
    for index, raw_output in enumerate(raw_outputs):
        if not isinstance(raw_output, dict):
            raise ValueError(f"mechanism.angle_outputs[{index}] must be an object")
        try:
            outputs.append(
                MechanismAngleOutput(
                    id=str(raw_output["id"]),
                    pivot_point_id=str(raw_output["pivot_point_id"]),
                    endpoint_point_id=str(raw_output["endpoint_point_id"]),
                    neutral_angle_rad=_config_angle(raw_output, "neutral_angle", 0.0),
                    minimum_angle_rad=_optional_config_angle(raw_output, "minimum_angle"),
                    maximum_angle_rad=_optional_config_angle(raw_output, "maximum_angle"),
                    reference_body_id=(
                        None
                        if raw_output.get("reference_body_id") in (None, "")
                        else str(raw_output["reference_body_id"])
                    ),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"mechanism.angle_outputs[{index}] is invalid") from error

    return PlanarMechanismGraph(
        id=str(raw_graph.get("id", "api_mechanism")),
        points=tuple(points),
        members=tuple(members),
        angle_outputs=tuple(outputs),
    )


def _parse_mechanism_drivers(raw_drivers: object) -> tuple[MechanismDriverArc, ...]:
    if raw_drivers is None:
        return ()
    if not isinstance(raw_drivers, list):
        raise ValueError("mechanism_drivers must be an array")
    drivers: list[MechanismDriverArc] = []
    for index, raw_driver in enumerate(raw_drivers):
        if not isinstance(raw_driver, dict):
            raise ValueError(f"mechanism_drivers[{index}] must be an object")
        center = _config_point(raw_driver, "center", None)
        if center is None:
            raise ValueError(f"mechanism_drivers[{index}] requires center")
        try:
            drivers.append(
                MechanismDriverArc(
                    point_id=str(raw_driver["point_id"]),
                    center=center,
                    radius_mm=float(raw_driver["radius_mm"]),
                    neutral_angle_rad=_config_angle(raw_driver, "neutral_angle", 0.0),
                    input_ratio=float(raw_driver.get("input_ratio", 1.0)),
                    phase_offset_rad=_config_angle(raw_driver, "phase_offset", 0.0),
                    input_id=str(raw_driver.get("input_id", "articulation")),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"mechanism_drivers[{index}] is invalid") from error
    return tuple(drivers)


def _parse_steering_assignments(
    raw_assignments: object,
) -> tuple[MechanismSteeringAssignment, ...]:
    if raw_assignments is None:
        return ()
    if not isinstance(raw_assignments, list):
        raise ValueError("steering_assignments must be an array")
    assignments: list[MechanismSteeringAssignment] = []
    for index, raw_assignment in enumerate(raw_assignments):
        if not isinstance(raw_assignment, dict):
            raise ValueError(f"steering_assignments[{index}] must be an object")
        try:
            assignments.append(
                MechanismSteeringAssignment(
                    output_id=str(raw_assignment["output_id"]),
                    wheel_id=str(raw_assignment["wheel_id"]),
                    ratio=float(raw_assignment.get("ratio", 1.0)),
                    phase_offset_rad=_config_angle(raw_assignment, "phase_offset", 0.0),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"steering_assignments[{index}] is invalid") from error
    return tuple(assignments)


def _parse_driven_positions(raw_positions: object) -> dict[str, Point2D]:
    if raw_positions is None:
        return {}
    if not isinstance(raw_positions, dict):
        raise ValueError("driven_positions must be an object keyed by point ID")
    positions: dict[str, Point2D] = {}
    for point_id, raw_position in raw_positions.items():
        position = _config_point({"position": raw_position}, "position", None)
        assert position is not None
        positions[str(point_id)] = position
    return positions


def _mechanism_graph_payload(
    graph: PlanarMechanismGraph,
    state: MechanismGraphState,
) -> dict[str, object]:
    return {
        "mechanism": {
            "id": graph.id,
            "points": [
                {
                    "id": point.id,
                    "mode": point.mode,
                    "neutral_position": _point_payload(point.neutral_position),
                    "envelope_radius_mm": point.envelope_radius_mm,
                    "body_id": point.body_id,
                }
                for point in graph.points
            ],
            "members": [
                {
                    "id": member.id,
                    "point_a_id": member.point_a_id,
                    "point_b_id": member.point_b_id,
                    "length_mm": member.length_mm,
                    "kind": member.kind,
                    "envelope_radius_mm": member.envelope_radius_mm,
                    "assembly_id": member.assembly_id,
                }
                for member in graph.members
            ],
            "angle_outputs": [
                {
                    "id": output.id,
                    "pivot_point_id": output.pivot_point_id,
                    "endpoint_point_id": output.endpoint_point_id,
                    "neutral_angle_rad": output.neutral_angle_rad,
                    "neutral_angle_deg": math.degrees(output.neutral_angle_rad),
                    "minimum_angle_rad": output.minimum_angle_rad,
                    "maximum_angle_rad": output.maximum_angle_rad,
                    "reference_body_id": output.reference_body_id,
                }
                for output in graph.angle_outputs
            ],
            "connected_member_pairs": [
                sorted(pair)
                for pair in sorted(
                    graph.connected_member_pairs(),
                    key=lambda item: sorted(item),
                )
            ],
        },
        "state": {
            "point_positions": {
                point_id: _point_payload(position)
                for point_id, position in state.point_positions.items()
            },
            "member_residuals_mm": dict(state.member_residuals_mm),
            "maximum_residual_mm": state.maximum_residual_mm,
            "output_angles_rad": dict(state.output_angles_rad),
            "output_angles_deg": {
                output_id: math.degrees(angle_rad)
                for output_id, angle_rad in state.output_angles_rad.items()
            },
            "iterations": state.iterations,
        },
    }


def _parse_pose(raw_pose: object, *, field_name: str) -> Pose2D | None:
    if raw_pose is None:
        return None
    if not isinstance(raw_pose, dict):
        raise ValueError(f"{field_name} must be an object")
    x_mm = _config_float(raw_pose, "x_mm", 0.0)
    y_mm = _config_float(raw_pose, "y_mm", 0.0)
    assert x_mm is not None and y_mm is not None
    return Pose2D(
        x_mm=x_mm,
        y_mm=y_mm,
        yaw_rad=_config_angle(raw_pose, "yaw", 0.0),
    )


def _parse_mounted_axle(raw_mounted: dict[str, object], index: int) -> MountedAxle:
    raw_axle = raw_mounted.get("axle", raw_mounted)
    if not isinstance(raw_axle, dict):
        raise ValueError(f"mounted_axles[{index}].axle must be an object")
    try:
        axle_id = str(raw_axle.get("id") or raw_axle["axle_id"])
        track_mm = float(raw_axle["track_mm"])
        body_id = str(raw_mounted["body_id"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"mounted_axles[{index}] requires body_id, axle.id or axle_id, and axle.track_mm"
        ) from error
    local_center = _config_point(
        raw_mounted,
        "local_center",
        Point2D(
            float(raw_axle.get("x_mm", 0.0)),
            float(raw_axle.get("y_mm", 0.0)),
        ),
    )
    assert local_center is not None
    return MountedAxle(
        axle=Axle(
            id=axle_id,
            center=Point2D(0.0, 0.0),
            track_mm=track_mm,
            wheel_count=int(raw_axle.get("wheel_count", 2)),
            wheel_lateral_offsets_mm=_config_wheel_lateral_offsets(raw_axle),
            steerable=bool(raw_axle.get("steerable", True)),
            steering_mode=str(raw_axle.get("steering_mode", "FORCED_STEER")),  # type: ignore[arg-type]
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
            load_kg=None if raw_axle.get("load_kg") is None else float(raw_axle["load_kg"]),
            heading_rad=_config_angle(raw_axle, "heading", 0.0),
            user_defined_steering_angle_rad=_config_angle(
                raw_axle,
                "user_defined_steering_angle",
                0.0,
            ),
            tire_width_mm=float(raw_axle.get("tire_width_mm", 0.0)),
            outside_diameter_mm=float(raw_axle.get("outside_diameter_mm", 0.0)),
        ),
        body_id=body_id,
        local_center=local_center,
    )


def _parse_vehicle_combination(raw_combination: object) -> VehicleCombination:
    if not isinstance(raw_combination, dict):
        raise ValueError("combination must be an object")
    raw_bodies = raw_combination.get("bodies")
    raw_joints = raw_combination.get("joints", [])
    raw_mounted_axles = raw_combination.get("mounted_axles")
    if not isinstance(raw_bodies, list) or not raw_bodies:
        raise ValueError("combination.bodies must be a non-empty array")
    if not isinstance(raw_joints, list):
        raise ValueError("combination.joints must be an array")
    if not isinstance(raw_mounted_axles, list) or not raw_mounted_axles:
        raise ValueError("combination.mounted_axles must be a non-empty array")

    bodies: list[RigidBody] = []
    for index, raw_body in enumerate(raw_bodies):
        if not isinstance(raw_body, dict):
            raise ValueError(f"combination.bodies[{index}] must be an object")
        try:
            body_id = str(raw_body["id"])
        except KeyError as error:
            raise ValueError(f"combination.bodies[{index}] requires id") from error
        pose = _parse_pose(raw_body.get("pose"), field_name=f"combination.bodies[{index}].pose")
        bodies.append(
            RigidBody(
                id=body_id,
                name=str(raw_body.get("name", body_id)),
                pose=pose or Pose2D(),
                body_length_mm=(
                    None
                    if raw_body.get("body_length_mm") is None
                    else float(raw_body["body_length_mm"])
                ),
                body_width_mm=(
                    None
                    if raw_body.get("body_width_mm") is None
                    else float(raw_body["body_width_mm"])
                ),
                parent_joint_id=(
                    None
                    if raw_body.get("parent_joint_id") in (None, "")
                    else str(raw_body["parent_joint_id"])
                ),
                child_joint_ids=tuple(str(value) for value in raw_body.get("child_joint_ids", [])),
                body_polygon=_config_polygon(
                    raw_body,
                    "body_polygon",
                ),
            )
        )

    joints: list[ArticulationJoint] = []
    raw_joint_ranges = raw_combination.get("joint_ranges", {})
    for index, raw_joint in enumerate(raw_joints):
        if not isinstance(raw_joint, dict):
            raise ValueError(f"combination.joints[{index}] must be an object")
        try:
            parent_anchor = _config_point(raw_joint, "parent_anchor", None)
            child_anchor = _config_point(raw_joint, "child_anchor", None)
            assert parent_anchor is not None and child_anchor is not None
            raw_range = None
            if isinstance(raw_joint_ranges, dict):
                raw_range = raw_joint_ranges.get(str(raw_joint["id"]))
            elif isinstance(raw_joint_ranges, list):
                raw_range = next(
                    (
                        item
                        for item in raw_joint_ranges
                        if isinstance(item, dict)
                        and str(item.get("joint_id", item.get("id", ""))) == str(raw_joint["id"])
                    ),
                    None,
                )
            if raw_range is not None and not isinstance(raw_range, dict):
                raise ValueError(f"combination.joint_ranges[{raw_joint['id']!r}] must be an object")
            joints.append(
                ArticulationJoint(
                    id=str(raw_joint["id"]),
                    parent_body_id=str(raw_joint["parent_body_id"]),
                    child_body_id=str(raw_joint["child_body_id"]),
                    parent_anchor=parent_anchor,
                    child_anchor=child_anchor,
                    articulation_rad=_config_angle(raw_joint, "articulation", 0.0),
                    sweep_min_deg=(
                        None if raw_range is None or raw_range.get("min_deg") is None
                        else float(raw_range["min_deg"])
                    ),
                    sweep_max_deg=(
                        None if raw_range is None or raw_range.get("max_deg") is None
                        else float(raw_range["max_deg"])
                    ),
                    sweep_step_deg=(
                        None if raw_range is None or raw_range.get("step_deg") is None
                        else float(raw_range["step_deg"])
                    ),
                    maximum_articulation_deg=float(
                        raw_joint.get("maximum_articulation_deg", 45.0)
                    ),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"combination.joints[{index}] is invalid") from error

    mounted_axles: list[MountedAxle] = []
    for index, raw_mounted in enumerate(raw_mounted_axles):
        if not isinstance(raw_mounted, dict):
            raise ValueError(f"combination.mounted_axles[{index}] must be an object")
        mounted_axles.append(_parse_mounted_axle(raw_mounted, index))
    steering_synchronizations = _parse_steering_synchronizations(
        raw_combination.get("steering_synchronizations", raw_combination.get("steering_sync")),
        {mounted.axle.id for mounted in mounted_axles},
    )

    return VehicleCombination(
        id=str(raw_combination.get("id", "api_combination")),
        name=str(raw_combination.get("name", "API vehicle combination")),
        bodies=tuple(bodies),
        joints=tuple(joints),
        mounted_axles=tuple(mounted_axles),
        root_body_id=(
            None
            if raw_combination.get("root_body_id") in (None, "")
            else str(raw_combination["root_body_id"])
        ),
        steering_synchronizations=steering_synchronizations,
    )


def _combination_kinematic_payload(
    combination: VehicleCombination,
    solution: CombinationKinematicSolution,
    *,
    root_pose: Pose2D | None,
) -> dict[str, object]:
    axle_by_id = {
        axle.id: axle
        for axle in combination.resolve_mounted_axles(root_pose=root_pose)
    }
    return {
        "combination": serialize_vehicle_combination(combination, root_pose=root_pose),
        "kinematics": {
            "icr": None if solution.icr is None else _point_payload(solution.icr),
            "root_turn_radius_mm": solution.root_turn_radius_mm,
            "root_icr_longitudinal_offset_mm": solution.root_icr_longitudinal_offset_mm,
            "maximum_constraint_residual_mm": solution.maximum_constraint_residual_mm,
            "maximum_joint_closure_error_mm": solution.maximum_joint_closure_error_mm,
            "body_poses": {
                body_id: {
                    "x_mm": pose.x_mm,
                    "y_mm": pose.y_mm,
                    "yaw_rad": pose.yaw_rad,
                    "yaw_deg": math.degrees(pose.yaw_rad),
                }
                for body_id, pose in solution.body_poses.items()
            },
            "axle_constraints": [
                {
                    "axle_id": item.axle_id,
                    "body_id": item.body_id,
                    "center": _point_payload(item.center),
                    "heading_rad": item.heading_rad,
                    "heading_deg": math.degrees(item.heading_rad),
                    "residual_mm": item.residual_mm,
                }
                for item in solution.axle_constraints
            ],
            "joint_states": [
                {
                    "joint_id": item.joint_id,
                    "parent_body_id": item.parent_body_id,
                    "child_body_id": item.child_body_id,
                    "parent_anchor_world": _point_payload(item.parent_anchor_world),
                    "child_anchor_world": _point_payload(item.child_anchor_world),
                    "closure_error_mm": item.closure_error_mm,
                    "articulation_rad": item.articulation_rad,
                    "articulation_deg": math.degrees(item.articulation_rad),
                }
                for item in solution.joint_states
            ],
        },
        "ideal_steering": {
            "axles": [
                _axle_payload(axle_solution, axle_by_id.get(axle_solution.axle_id))
                for axle_solution in solution.ideal_steering.axles
            ],
            "wheel_steering_angles_deg": solution.ideal_steering.wheel_steering_angles_deg(),
            "axle_center_steering_angles_deg": solution.ideal_steering.axle_center_steering_angles_deg(),
        },
    }


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


def _validated_cad_source(raw_source: object) -> dict[str, object] | None:
    if raw_source is None:
        return None
    if not isinstance(raw_source, dict):
        raise ValueError("cad_source must be an object")
    source_name = str(raw_source.get("source_name", "")).strip()
    source_sha256 = str(raw_source.get("source_sha256", "")).lower()
    source_units = str(raw_source.get("source_units", "")).strip().lower()
    coordinate_system = str(raw_source.get("coordinate_system", "")).strip().lower()
    scale_to_mm = raw_source.get("unit_scale_to_mm")
    try:
        scale_to_mm = float(scale_to_mm)
    except (TypeError, ValueError) as error:
        raise ValueError("cad_source.unit_scale_to_mm must be numeric") from error
    allowed_units = {
        value: scale
        for value, _label, scale in DXF_UNIT_OPTIONS
        if scale is not None
    }
    allowed_coordinate_systems = {value for value, _label in DXF_COORDINATE_OPTIONS}
    if not source_name:
        raise ValueError("cad_source.source_name is required")
    if (
        len(source_name) > 255
        or source_name in {".", ".."}
        or any(character in source_name for character in "/\\\r\n\x00")
    ):
        raise ValueError("cad_source.source_name must be a safe filename without path separators")
    if len(source_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in source_sha256
    ):
        raise ValueError("cad_source.source_sha256 must be a SHA-256 hex digest")
    if source_units not in allowed_units or abs(scale_to_mm - allowed_units[source_units]) > 1e-9:
        raise ValueError("cad_source source units and scale_to_mm are inconsistent")
    if coordinate_system not in allowed_coordinate_systems:
        raise ValueError("cad_source.coordinate_system must be an approved model frame")
    if raw_source.get("metadata_confirmed") is not True:
        raise ValueError("cad_source.metadata_confirmed must be true")
    return {
        "source_name": source_name,
        "source_sha256": source_sha256,
        "source_units": source_units,
        "unit_scale_to_mm": scale_to_mm,
        "coordinate_system": coordinate_system,
        "metadata_confirmed": True,
    }


def _parse_vehicle_config(raw_config: object) -> tuple[VehicleLayout, dict[str, object]] | None:
    if raw_config is None:
        return None
    if not isinstance(raw_config, dict):
        raise ValueError("vehicle_config must be an object")
    vehicle = _parse_vehicle_layout(raw_config)
    normalized = _vehicle_config_payload(vehicle)
    cad_source = _validated_cad_source(raw_config.get("cad_source"))
    if cad_source is not None:
        normalized["cad_source"] = cad_source
    return vehicle, normalized


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


def _acceptance_criteria_fingerprint(criteria: MonrocAcceptanceCriteria) -> str:
    canonical = json.dumps(
        criteria.to_dict(),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _monroc_acceptance_profile_status(
    organization_id: str,
    criteria: MonrocAcceptanceCriteria,
) -> dict[str, object]:
    """Verify entered limits against an operator-configured approved profile.

    Criteria entered in the browser are useful for pilot diagnostics, but they
    are not trusted release configuration. The production deployment supplies
    exact profiles through a protected environment variable until a dedicated
    profile-management service is introduced.
    """

    fingerprint = _acceptance_criteria_fingerprint(criteria)
    raw_profiles = os.environ.get("EASYTOWING_MONROC_ACCEPTANCE_PROFILES_JSON", "").strip()
    if not raw_profiles:
        return {
            "status": "UNAPPROVED",
            "profile_id": None,
            "criteria_fingerprint": fingerprint,
            "message": "No approved Monroc acceptance profile is configured for this deployment.",
        }
    try:
        profiles = json.loads(raw_profiles)
    except json.JSONDecodeError:
        return {
            "status": "UNAPPROVED",
            "profile_id": None,
            "criteria_fingerprint": fingerprint,
            "message": "The configured Monroc acceptance profiles are invalid; release is blocked.",
        }
    if not isinstance(profiles, dict):
        return {
            "status": "UNAPPROVED",
            "profile_id": None,
            "criteria_fingerprint": fingerprint,
            "message": "The configured Monroc acceptance profiles must be an object; release is blocked.",
        }
    profile_id = f"{organization_id}:{criteria.case_id}"
    raw_profile = profiles.get(profile_id)
    if not isinstance(raw_profile, dict):
        return {
            "status": "UNAPPROVED",
            "profile_id": profile_id,
            "criteria_fingerprint": fingerprint,
            "message": f"No approved Monroc acceptance profile exists for {profile_id}.",
        }
    try:
        approved_criteria = MonrocAcceptanceCriteria.from_dict(raw_profile)
    except (TypeError, ValueError):
        return {
            "status": "UNAPPROVED",
            "profile_id": profile_id,
            "criteria_fingerprint": fingerprint,
            "message": f"The approved Monroc acceptance profile {profile_id} is invalid; release is blocked.",
        }
    approved_fingerprint = _acceptance_criteria_fingerprint(approved_criteria)
    if not hmac.compare_digest(fingerprint, approved_fingerprint):
        return {
            "status": "UNAPPROVED",
            "profile_id": profile_id,
            "criteria_fingerprint": fingerprint,
            "message": f"Entered limits do not match the approved Monroc profile {profile_id}.",
        }
    return {
        "status": "APPROVED",
        "profile_id": profile_id,
        "criteria_fingerprint": fingerprint,
        "message": f"Limits match the approved Monroc profile {profile_id}.",
    }


def _revision_clearance_target_mm(revision) -> float:
    """Read the target captured with the saved full-range engineering evidence."""

    sweep = (revision.snapshot or {}).get("sweep_validation") or {}
    raw_target = sweep.get("clearance_target_mm", 20.0)
    try:
        target = float(raw_target)
    except (TypeError, ValueError) as error:
        raise ValueError("Saved clearance target is invalid.") from error
    if not math.isfinite(target) or target < 0.0:
        raise ValueError("Saved clearance target must be finite and non-negative.")
    return target


def _require_engineering_pass_for_approval(revision) -> None:
    acceptance = (revision.snapshot or {}).get("monroc_acceptance")
    if not isinstance(acceptance, dict) or (acceptance.get("result") or {}).get("status") != "PASS":
        raise ValueError("A passing configured Monroc acceptance evaluation is required before approval.")
    if revision.combination_config is not None:
        sweep_result = (revision.snapshot or {}).get("sweep_validation") or {}
        if not isinstance(sweep_result, dict):
            raise ValueError("Explicit full-range sweep evidence is required before approval.")
        current_result = evaluate_engineering_snapshot(
            revision.snapshot or {},
            clearance_target_mm=_revision_clearance_target_mm(revision),
        )
        try:
            sample_count = int(sweep_result.get("sample_count", 0))
            solved_sample_count = int(sweep_result.get("solved_sample_count", -1))
        except (TypeError, ValueError) as error:
            raise ValueError("Saved full-range sweep evidence is invalid.") from error
        sweep_complete = (
            sweep_result.get("status") == "PASS"
            and sweep_result.get("sampling_complete") is True
            and sample_count > 0
            and solved_sample_count == sample_count
            and not sweep_result.get("violations")
        )
        if (
            current_result["status"] != "PASS"
            or not sweep_complete
        ):
            raise ValueError("Engineering PASS across the saved full-range snapshot is required before approval.")
    criteria_approval = acceptance.get("criteria_approval")
    if not isinstance(criteria_approval, dict) or criteria_approval.get("status") != "APPROVED":
        raise ValueError(
            "The evaluated limits must match a server-configured approved Monroc acceptance profile before approval."
        )
    if revision.combination_config is not None:
        return
    if not revision.accepted_optimization:
        raise ValueError("Only a hard-feasible accepted optimization revision can be approved.")


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


def _project_state_payload(organization_id: str | None = None) -> dict[str, object]:
    projects = PROJECT_STORE.list_projects(organization_id)
    active_project = next((project for project in projects if project.active_revision_id is not None), None)
    return {
        "projects": [_project_summary_payload(project) for project in projects],
        "active_project_id": None if active_project is None else active_project.id,
        "active_project": None if active_project is None else _project_detail_payload(active_project),
    }


def _should_seed_reference_project(database_url: str | None) -> bool:
    """Keep simulation seed data out of authenticated database tenants."""

    return not bool((database_url or "").strip())


def _project_combination_inputs(
    body: dict[str, object],
    *,
    beta_deg: float,
) -> tuple[
    dict[str, object] | None,
    float | None,
    dict[str, object] | None,
    tuple[MechanismDriverArc, ...],
    tuple[MechanismSteeringAssignment, ...],
    dict[str, object] | None,
]:
    raw_combination = body.get("combination_config", body.get("combination"))
    raw_graph = body.get("mechanism_graph_config", body.get("mechanism_graph"))
    raw_drivers = body.get("mechanism_drivers")
    raw_assignments = body.get("steering_assignments")
    if raw_combination is None:
        if raw_graph is not None or raw_drivers is not None or raw_assignments is not None:
            raise ValueError("A mechanism graph revision requires combination_config.")
        return None, None, None, (), (), None
    if not isinstance(raw_combination, dict):
        raise ValueError("combination_config must be an object.")
    if raw_graph is not None and not isinstance(raw_graph, dict):
        raise ValueError("mechanism_graph_config must be an object.")

    combination = _parse_vehicle_combination(raw_combination)
    raw_joint_ranges = body.get("joint_ranges")
    if raw_joint_ranges is None:
        raw_joint_ranges = raw_combination.get("joint_ranges")
    root_turn_radius_mm = _config_float(
        body,
        "root_turn_radius_mm",
        None,
        allow_none=True,
    )
    mechanism_graph = None if raw_graph is None else _parse_mechanism_graph(raw_graph)
    mechanism_drivers = _parse_mechanism_drivers(raw_drivers)
    steering_assignments = _parse_steering_assignments(raw_assignments)
    clearance_target_mm = _config_float(body, "clearance_target_mm", 20.0)
    assert clearance_target_mm is not None
    snapshot = build_demo_payload(
        beta_deg,
        combination=combination,
        root_turn_radius_mm=root_turn_radius_mm,
        mechanism_graph=mechanism_graph,
        mechanism_drivers=mechanism_drivers,
        steering_assignments=steering_assignments,
        clearance_target_mm=clearance_target_mm,
    )
    if mechanism_graph is not None:
        beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
        step_deg = _config_float(body, "sweep_step_deg", 1.0)
        assert step_deg is not None and clearance_target_mm is not None
        snapshot["sweep_validation"] = build_combination_sweep_payload(
            combination,
            root_turn_radius_mm=root_turn_radius_mm,
            mechanism_graph=mechanism_graph,
            mechanism_drivers=mechanism_drivers,
            steering_assignments=steering_assignments,
            beta_min_deg=beta_min_deg,
            beta_max_deg=beta_max_deg,
            step_deg=step_deg,
            primary_joint_id=(
                None
                if body.get("primary_joint_id") is None
                else str(body["primary_joint_id"])
            ),
            clearance_target_mm=clearance_target_mm,
            joint_ranges=raw_joint_ranges,
        )
    return (
        json.loads(json.dumps(raw_combination)),
        root_turn_radius_mm,
        None if raw_graph is None else json.loads(json.dumps(raw_graph)),
        mechanism_drivers,
        steering_assignments,
        snapshot,
    )


def _clearance_payload(vehicle, rig, state) -> dict[str, object]:
    items = build_linkage_clearance_items(rig.spec, state, vehicle=vehicle)
    report = analyze_clearance(items)
    return _clearance_report_payload(report)


def build_demo_payload(
    beta_deg: float,
    wheelbase_mm: float = 4360.0,
    track_mm: float = 2500.0,
    linkage_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
    combination: VehicleCombination | None = None,
    root_turn_radius_mm: float | None = None,
    mechanism_graph: PlanarMechanismGraph | None = None,
    mechanism_drivers: tuple[MechanismDriverArc, ...] = (),
    steering_assignments: tuple[MechanismSteeringAssignment, ...] = (),
    clearance_target_mm: float = 20.0,
    mechanism_previous_state: MechanismGraphState | None = None,
    mechanism_state_sink: list[MechanismGraphState] | None = None,
) -> dict[str, object]:
    if wheelbase_mm <= 0.0 or track_mm <= 0.0:
        raise ValueError("wheelbase_mm and track_mm must be positive")
    if not math.isfinite(clearance_target_mm) or clearance_target_mm < 0.0:
        raise ValueError("clearance_target_mm must be finite and non-negative")
    combination_solution: CombinationKinematicSolution | None = None
    if combination is not None:
        combination_solution = solve_combination_kinematics(
            combination,
            root_turn_radius_mm=root_turn_radius_mm,
        )
        vehicle = combination.to_vehicle_layout()
        solution = combination_solution.ideal_steering
        radius = combination_solution.root_turn_radius_mm
    elif vehicle is None:
        vehicle, solution, radius = build_demo_solution(
            beta_deg,
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
        )
    else:
        reference_length_mm = vehicle.axle_span_mm() or wheelbase_mm
        radius = beta_to_reference_radius_mm(math.radians(beta_deg), reference_length_mm)
        solution = solve_ideal_steering_from_radius(vehicle, radius)
    if combination is None and abs(beta_deg) > vehicle.maximum_articulation_deg + 1e-9:
        raise ArticulationLimitExceededError(beta_deg, vehicle.maximum_articulation_deg)
    linkage: dict[str, object] | None
    mechanism_payload: dict[str, object] | None = None
    if mechanism_graph is not None:
        body_poses = (
            {}
            if combination_solution is None
            else combination_solution.body_poses
        )
        driven_point_ids = {
            point.id
            for point in mechanism_graph.points
            if point.mode == "driven"
        }
        driver_ids = {driver.point_id for driver in mechanism_drivers}
        if driven_point_ids != driver_ids:
            missing = sorted(driven_point_ids - driver_ids)
            extra = sorted(driver_ids - driven_point_ids)
            raise ValueError(
                f"mechanism driver mapping mismatch; missing={missing!r}, extra={extra!r}"
            )
        driver_inputs = {"articulation": math.radians(beta_deg)}
        if combination is not None:
            driver_inputs.update(
                {joint.id: joint.articulation_rad for joint in combination.joints}
            )
        driven_positions = resolve_driver_arc_positions(
            mechanism_graph,
            mechanism_drivers,
            driver_inputs,
            body_poses=body_poses,
        )
        mechanism_state = solve_mechanism_graph(
            mechanism_graph,
            driven_positions,
            previous_state=mechanism_previous_state,
            body_poses=body_poses,
        )
        if mechanism_state_sink is not None:
            mechanism_state_sink.append(mechanism_state)
        mechanism_payload = _mechanism_graph_payload(mechanism_graph, mechanism_state)
        actual_solution = solve_actual_steering_from_graph(
            vehicle,
            mechanism_state,
            steering_assignments,
        )
        clearance_report = analyze_clearance(
            build_mechanism_graph_clearance_items(
                mechanism_graph,
                mechanism_state,
                vehicle=vehicle,
                combination=combination,
                body_poses=combination_solution.body_poses if combination_solution else None,
            )
        )
        clearance = _clearance_report_payload(clearance_report)
        linkage = None
    else:
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
        if combination is None or combination_solution is None:
            clearance = _clearance_payload(vehicle, rig, linkage_state)
        else:
            linkage_items = build_linkage_clearance_items(
                rig.spec,
                linkage_state,
                vehicle=vehicle,
            )
            body_items = build_combination_body_clearance_items(
                combination,
                combination_solution.body_poses,
                mounted_component_ids={
                    combination.root_body_id or combination.bodies[0].id: tuple(
                        item.id for item in linkage_items
                    ),
                },
            )
            clearance = _clearance_report_payload(
                analyze_clearance((*linkage_items, *body_items))
            )
    actual_comparison = compare_actual_to_ideal(
        actual_solution,
        solution,
        vehicle=vehicle,
        beta_rad=math.radians(beta_deg),
    )
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
    front_actual = max(actual_solution.axles, key=lambda item: item.center.x_mm) if actual_solution.axles else None

    metrics = {
        "max_abs_wheel_angle_deg": max((abs(value) for value in wheel_angles_deg.values()), default=0.0),
        "front_axle_center_angle_deg": None if front_solution is None else front_solution.center_steering_angle_deg,
        "rear_axle_center_angle_deg": None if rear_solution is None else rear_solution.center_steering_angle_deg,
        "linkage_actual_steering_deg": (
            None
            if front_actual is None
            else front_actual.center_steering_angle_deg
        ),
        "linkage_vs_ideal_front_axle_deg": None,
        "linkage_actual_front_left_deg": (
            None
            if front_actual is None
            else front_actual.left_wheel.steering_angle_deg
        ),
        "linkage_actual_front_right_deg": (
            None
            if front_actual is None
            else front_actual.right_wheel.steering_angle_deg
        ),
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
            metrics["linkage_actual_steering_deg"] - metrics["front_axle_center_angle_deg"]
        )
    if front_solution is not None and front_actual is not None:
        metrics["linkage_vs_ideal_front_left_deg"] = (
            front_actual.left_wheel.steering_angle_deg - front_solution.left_wheel.steering_angle_deg
        )
        metrics["linkage_vs_ideal_front_right_deg"] = (
            front_actual.right_wheel.steering_angle_deg - front_solution.right_wheel.steering_angle_deg
        )

    payload = {
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
        "vehicle_combination": (
            serialize_vehicle_combination(combination)
            if combination is not None
            else (
                serialize_vehicle_combination(
                    build_reference_demo_combination(
                        wheelbase_mm=vehicle.axle_span_mm() or wheelbase_mm,
                        track_mm=max((axle.track_mm for axle in vehicle.axles), default=track_mm),
                        articulation_rad=math.radians(beta_deg),
                    )
                )
                if len(vehicle.axles) == 2 and vehicle.id == "reference_demo_combination"
                else None
            )
        ),
        "combination_kinematics": (
            None
            if combination is None or combination_solution is None
            else _combination_kinematic_payload(
                combination,
                combination_solution,
                root_pose=None,
            )["kinematics"]
        ),
        "linkage": linkage,
        "mechanism_graph": mechanism_payload,
        "mechanism_mapping": (
            None
            if mechanism_graph is None
            else {
                "drivers": [
                    {
                        "point_id": driver.point_id,
                        "input_id": driver.input_id,
                        "center": _point_payload(driver.center),
                        "radius_mm": driver.radius_mm,
                        "neutral_angle_rad": driver.neutral_angle_rad,
                        "neutral_angle_deg": math.degrees(driver.neutral_angle_rad),
                        "input_ratio": driver.input_ratio,
                        "phase_offset_rad": driver.phase_offset_rad,
                        "phase_offset_deg": math.degrees(driver.phase_offset_rad),
                    }
                    for driver in mechanism_drivers
                ],
                "steering_assignments": [
                    {
                        "output_id": assignment.output_id,
                        "wheel_id": assignment.wheel_id,
                        "ratio": assignment.ratio,
                        "phase_offset_rad": assignment.phase_offset_rad,
                        "phase_offset_deg": math.degrees(assignment.phase_offset_rad),
                    }
                    for assignment in steering_assignments
                ],
            }
        ),
        "clearance": clearance,
        "metrics": metrics,
    }
    payload["engineering_evaluation"] = evaluate_engineering_snapshot(
        payload,
        clearance_target_mm=clearance_target_mm,
    )
    return payload


def build_combination_sweep_payload(
    combination: VehicleCombination,
    *,
    root_turn_radius_mm: float | None,
    mechanism_graph: PlanarMechanismGraph,
    mechanism_drivers: tuple[MechanismDriverArc, ...],
    steering_assignments: tuple[MechanismSteeringAssignment, ...],
    beta_min_deg: float = -45.0,
    beta_max_deg: float = 45.0,
    step_deg: float = 1.0,
    primary_joint_id: str | None = None,
    clearance_target_mm: float = 20.0,
    joint_ranges: object | None = None,
    maximum_samples: int = 10_000,
) -> dict[str, object]:
    if beta_min_deg >= beta_max_deg or beta_min_deg > 0.0 or beta_max_deg < 0.0:
        raise ValueError("Sweep articulation bounds must straddle zero.")
    if not math.isfinite(step_deg) or step_deg <= 0.0:
        raise ValueError("Sweep step must be a positive finite angle.")
    ranges = _parse_joint_sweep_ranges(
        combination,
        joint_ranges,
        beta_min_deg=beta_min_deg,
        beta_max_deg=beta_max_deg,
        step_deg=step_deg,
        primary_joint_id=primary_joint_id,
    )
    joint_id = primary_joint_id or ranges[0].joint_id
    primary_range = next(item for item in ranges if item.joint_id == joint_id)
    reported_beta_min_deg = primary_range.minimum_deg
    reported_beta_max_deg = primary_range.maximum_deg
    reported_step_deg = primary_range.step_deg
    try:
        joint_samples = build_joint_sweep_grid(
            ranges,
            maximum_samples=maximum_samples,
        )
    except SweepSampleLimitError as error:
        return {
            "status": "FAIL",
            "primary_joint_id": joint_id,
            "joint_ranges": [item.to_dict() for item in ranges],
            "joint_articulation_limits_deg": {
                joint.id: joint.maximum_articulation_deg for joint in combination.joints
            },
            "joint_ids": [item.joint_id for item in ranges],
            "fixed_joint_angles_deg": {
                joint.id: math.degrees(joint.articulation_rad)
                for joint in combination.joints
                if joint.id not in {item.joint_id for item in ranges}
            },
            "beta_min_deg": reported_beta_min_deg,
            "beta_max_deg": reported_beta_max_deg,
            "step_deg": reported_step_deg,
            "sample_count": error.requested_count,
            "solved_sample_count": 0,
            "minimum_clearance_mm": None,
            "minimum_clearance_beta_deg": None,
            "minimum_clearance_joint_angles_deg": None,
            "max_abs_wheel_error_deg": 0.0,
            "max_abs_synchronization_error_deg": 0.0,
            "clearance_target_mm": clearance_target_mm,
            "violations": [{
                "checks": [error.code],
                "guidance": [
                    "Increase the joint grid step or split the validation into approved design cases.",
                    f"The requested Cartesian grid contains {error.requested_count} poses; the limit is {error.maximum_count}.",
                ],
                "error": str(error),
            }],
            "failure_guidance": [
                "Increase the joint grid step or split the validation into approved design cases.",
                f"The requested Cartesian grid contains {error.requested_count} poses; the limit is {error.maximum_count}.",
            ],
            "samples": [],
            "sampling_complete": False,
            "steering_acceptance_included": False,
        }

    samples: list[dict[str, object]] = []
    violations: list[dict[str, object]] = []
    minimum_clearance_mm: float | None = None
    minimum_clearance_beta_deg: float | None = None
    maximum_wheel_error_deg = 0.0
    maximum_sync_error_deg = 0.0
    range_ids = {item.joint_id for item in ranges}
    fixed_joint_angles_deg = {
        joint.id: math.degrees(joint.articulation_rad)
        for joint in combination.joints
        if joint.id not in range_ids
    }
    minimum_clearance_joint_angles_deg: dict[str, float] | None = None
    previous_mechanism_state: MechanismGraphState | None = None
    for sample_joint_values in joint_samples:
        joint_values = {
            joint.id: math.degrees(joint.articulation_rad)
            for joint in combination.joints
        }
        joint_values.update(sample_joint_values)
        sample_beta_deg = joint_values[joint_id]
        sample_combination = VehicleCombination(
            id=combination.id,
            name=combination.name,
            bodies=combination.bodies,
            joints=tuple(
                ArticulationJoint(
                    id=joint.id,
                    parent_body_id=joint.parent_body_id,
                    child_body_id=joint.child_body_id,
                    parent_anchor=joint.parent_anchor,
                    child_anchor=joint.child_anchor,
                    articulation_rad=math.radians(joint_values[joint.id]),
                    sweep_min_deg=joint.sweep_min_deg,
                    sweep_max_deg=joint.sweep_max_deg,
                    sweep_step_deg=joint.sweep_step_deg,
                    maximum_articulation_deg=joint.maximum_articulation_deg,
                )
                for joint in combination.joints
            ),
            mounted_axles=combination.mounted_axles,
            root_body_id=combination.root_body_id,
            steering_synchronizations=combination.steering_synchronizations,
        )
        try:
            mechanism_state_sink: list[MechanismGraphState] = []
            snapshot = build_demo_payload(
                sample_beta_deg,
                combination=sample_combination,
                root_turn_radius_mm=root_turn_radius_mm,
                mechanism_graph=mechanism_graph,
                mechanism_drivers=mechanism_drivers,
                steering_assignments=steering_assignments,
                clearance_target_mm=clearance_target_mm,
                mechanism_previous_state=previous_mechanism_state,
                mechanism_state_sink=mechanism_state_sink,
            )
            if mechanism_state_sink:
                previous_mechanism_state = mechanism_state_sink[-1]
            evaluation = evaluate_engineering_snapshot(
                snapshot,
                clearance_target_mm=clearance_target_mm,
            )
            clearance_mm = snapshot["clearance"]["minimum_clearance_mm"]
            if clearance_mm is not None and (
                minimum_clearance_mm is None or float(clearance_mm) < minimum_clearance_mm
            ):
                minimum_clearance_mm = float(clearance_mm)
                minimum_clearance_beta_deg = sample_beta_deg
                minimum_clearance_joint_angles_deg = dict(joint_values)
            wheel_error_deg = float(snapshot["metrics"].get("max_abs_wheel_error_deg") or 0.0)
            sync_error_deg = float(
                snapshot["metrics"].get("max_abs_synchronization_error_deg") or 0.0
            )
            maximum_wheel_error_deg = max(maximum_wheel_error_deg, wheel_error_deg)
            maximum_sync_error_deg = max(maximum_sync_error_deg, sync_error_deg)
            failed_checks = [
                str(check["id"])
                for check in evaluation["checks"]
                if not bool(check["pass"])
            ]
            sample = {
                "beta_deg": sample_beta_deg,
                "joint_angles_deg": joint_values,
                "status": evaluation["status"],
                "failed_checks": failed_checks,
                "maximum_mechanism_residual_mm": snapshot["mechanism_graph"]["state"]["maximum_residual_mm"],
                "minimum_clearance_mm": clearance_mm,
                "collision_detected": snapshot["clearance"]["collision_detected"],
                "max_abs_wheel_error_deg": wheel_error_deg,
                "max_abs_synchronization_error_deg": sync_error_deg,
            }
            samples.append(sample)
            if failed_checks:
                violations.append(
                    {
                        "beta_deg": sample_beta_deg,
                        "joint_angles_deg": joint_values,
                        "checks": failed_checks,
                        "guidance": evaluation["guidance"],
                    }
                )
        except (EngineeringError, ValueError) as error:
            samples.append(
                {
                    "beta_deg": sample_beta_deg,
                    "joint_angles_deg": joint_values,
                    "status": "FAIL",
                    "failed_checks": [getattr(error, "code", "MECHANISM_UNSOLVED")],
                    "error": str(error),
                }
            )
            violations.append(
                {
                        "beta_deg": sample_beta_deg,
                        "joint_angles_deg": joint_values,
                        "checks": [getattr(error, "code", "MECHANISM_UNSOLVED")],
                    "guidance": engineering_failure_guidance(
                        [getattr(error, "code", "MECHANISM_UNSOLVED")]
                    ),
                    "error": str(error),
                }
            )

    failed_check_ids = [
        check_id
        for violation in violations
        for check_id in violation.get("checks", [])
    ]
    return {
        "status": "PASS" if not violations else "FAIL",
        "primary_joint_id": joint_id,
        "joint_ranges": [item.to_dict() for item in ranges],
        "joint_articulation_limits_deg": {
            joint.id: joint.maximum_articulation_deg for joint in combination.joints
        },
        "joint_ids": [item.joint_id for item in ranges],
        "fixed_joint_angles_deg": fixed_joint_angles_deg,
        "beta_min_deg": reported_beta_min_deg,
        "beta_max_deg": reported_beta_max_deg,
        "step_deg": reported_step_deg,
        "sample_count": len(samples),
        "solved_sample_count": sum(1 for sample in samples if "error" not in sample),
        "sampling_complete": True,
        "minimum_clearance_mm": minimum_clearance_mm,
        "minimum_clearance_beta_deg": minimum_clearance_beta_deg,
        "minimum_clearance_joint_angles_deg": minimum_clearance_joint_angles_deg,
        "max_abs_wheel_error_deg": maximum_wheel_error_deg,
        "max_abs_synchronization_error_deg": maximum_sync_error_deg,
        "clearance_target_mm": clearance_target_mm,
        "violations": violations,
        "failure_guidance": engineering_failure_guidance(failed_check_ids),
        "samples": samples,
        "steering_acceptance_included": False,
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

    def _send_download(
        self,
        content_builder,
        content_type: str,
        filename: str,
        *,
        extra_headers: dict[str, str] | None = None,
    ) -> None:
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
        for name, value in (extra_headers or {}).items():
            self.send_header(name, value)
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
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON request body must be an object.")
        return payload

    def log_message(self, format: str, *args) -> None:  # noqa: A003 - match BaseHTTPRequestHandler
        return

    def _principal(self, permission: str | None = None) -> Principal:
        authorization = self.headers.get("Authorization", "")
        token = authorization.removeprefix("Bearer ").strip()
        if token:
            principal = SAAS_CONTROL.authenticate(token)
        elif SAAS_AUTH_REQUIRED:
            raise SaaSAuthorizationError("Authentication required.")
        else:
            principal = LOCAL_DEVELOPER
        if permission is not None:
            SAAS_CONTROL.require(principal, permission)
        return principal

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
        if parsed.path == "/api/health":
            self._send_json(
                {
                    "status": "ok",
                    "service": "easytowing-demo",
                    "backend": "postgresql" if os.environ.get("EASYTOWING_DATABASE_URL") else "local-development",
                    "artifact_storage": (
                        ARTIFACT_BLOB_STORE.backend
                        if ARTIFACT_BLOB_STORE is not None
                        else "response-only"
                    ),
                }
            )
            return
        if parsed.path == "/api/ready":
            backend = "postgresql" if DATABASE_URL else "local-development"
            artifact_storage_status = (
                ARTIFACT_BLOB_STORE.backend
                if ARTIFACT_BLOB_STORE is not None
                else "response-only"
            )
            worker_status = "local-in-process" if not DATABASE_URL else "not_required"
            if ARTIFACT_BLOB_STORE is not None:
                try:
                    ARTIFACT_BLOB_STORE.health_check()
                except ArtifactStorageError:
                    artifact_storage_status = "unavailable"
            if artifact_storage_status == "unavailable" or (
                ARTIFACT_STORAGE_REQUIRED and artifact_storage_status == "response-only"
            ):
                self._send_json(
                    {
                        "status": "not_ready",
                        "service": "easytowing-demo",
                        "backend": backend,
                        "dependencies": {
                            "database": "configured" if DATABASE_URL else "local",
                            "artifact_storage": artifact_storage_status,
                            "worker": worker_status,
                        },
                    },
                    status=503,
                )
                return
            if DATABASE_URL:
                try:
                    PROJECT_STORE.health_check()
                except Exception:  # noqa: BLE001 - readiness must not expose database details
                    self._send_json(
                        {
                            "status": "not_ready",
                            "service": "easytowing-demo",
                            "backend": backend,
                            "dependencies": {
                                "database": "unavailable",
                                "artifact_storage": artifact_storage_status,
                                "worker": worker_status,
                            },
                        },
                        status=503,
                    )
                    return
                if WORKER_REQUIRED:
                    try:
                        worker_health = SAAS_CONTROL.worker_health(
                            max_age_seconds=WORKER_MAX_AGE_SECONDS,
                        )
                        worker_status = (
                            "healthy" if worker_health["healthy"] else "no_live_worker"
                        )
                    except Exception:  # noqa: BLE001 - readiness must not expose database details
                        worker_status = "unavailable"
                    if worker_status != "healthy":
                        self._send_json(
                            {
                                "status": "not_ready",
                                "service": "easytowing-demo",
                                "backend": backend,
                                "dependencies": {
                                    "database": "configured",
                                    "artifact_storage": artifact_storage_status,
                                    "worker": worker_status,
                                },
                                "worker_required": True,
                                "worker_max_age_seconds": WORKER_MAX_AGE_SECONDS,
                            },
                            status=503,
                        )
                        return
            self._send_json(
                {
                    "status": "ready",
                    "service": "easytowing-demo",
                    "backend": backend,
                    "dependencies": {
                        "database": "configured" if DATABASE_URL else "local",
                        "artifact_storage": artifact_storage_status,
                        "worker": worker_status,
                    },
                }
            )
            return
        if parsed.path == "/api/saas/status":
            self._send_json(
                {
                    "auth_required": SAAS_AUTH_REQUIRED,
                    "backend": "postgresql" if os.environ.get("EASYTOWING_DATABASE_URL") else "local-development",
                    "artifact_storage": (
                        ARTIFACT_BLOB_STORE.backend
                        if ARTIFACT_BLOB_STORE is not None
                        else "response-only"
                    ),
                    "artifact_storage_required": ARTIFACT_STORAGE_REQUIRED,
                    "worker_required": WORKER_REQUIRED,
                    "worker_max_age_seconds": WORKER_MAX_AGE_SECONDS,
                    "roles": [role.value for role in UserRole],
                    "approval_states": ["draft", "submitted", "approved", "rejected"],
                    "job_states": ["queued", "running", "succeeded", "failed", "cancelled"],
                }
            )
            return
        if parsed.path == "/api/auth/session":
            try:
                principal = self._principal()
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "AUTHENTICATION_REQUIRED", "message": str(error)}, status=401)
                return
            self._send_json({"authenticated": True, "principal": principal_payload(principal)})
            return
        if parsed.path == "/api/users":
            try:
                principal = self._principal("user:manage")
                users = SAAS_CONTROL.list_users(principal)
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            self._send_json({"users": [serialize_user(user) for user in users]})
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
            try:
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
            except EngineeringError as error:
                self._send_json(
                    {"error_code": error.code, "message": str(error)},
                    status=422,
                )
                return
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
            try:
                principal = self._principal("project:read")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            # Seed only the dependency-free local demo. A PostgreSQL tenant
            # must start empty so reference data cannot be mistaken for a
            # customer's engineering input.
            if _should_seed_reference_project(DATABASE_URL):
                PROJECT_STORE.ensure_seed_project(principal.organization_id)
            self._send_json(_project_state_payload(principal.organization_id))
            return

        parts = [part for part in parsed.path.split("/") if part]
        if len(parts) == 3 and parts[0] == "api" and parts[1] == "jobs":
            try:
                principal = self._principal("project:read")
                job = SAAS_CONTROL.get_job(principal, parts[2])
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except KeyError:
                self.send_error(404, "Job not found")
                return
            self._send_json(serialize_job(job))
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "audit":
            try:
                principal = self._principal("audit:read")
                events = SAAS_CONTROL.audit_events(principal, target_id=parts[2])
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            self._send_json({"events": [serialize_audit_event(event) for event in events]})
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] in {"export.json", "export.csv", "export.pdf", "export.svg", "export.dxf", "export.png"}
        ):
            try:
                principal = self._principal("report:read")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
            if project is None:
                self.send_error(404, "Project not found")
                return
            revision = project.get_revision(parts[4])
            if revision is None:
                self.send_error(404, "Revision not found")
                return
            if revision.combination_config is None:
                self.send_error(400, "This endpoint requires a multi-body project revision")
                return
            if parts[5] == "export.json":
                self._send_download(
                    lambda: json.dumps(
                        {
                            "project": project.summary(),
                            "revision": revision.summary(),
                            "engineering_snapshot": revision.snapshot,
                        },
                        indent=2,
                    ).encode("utf-8"),
                    "application/json; charset=utf-8",
                    "easytowing-multibody-evaluation.json",
                )
                return
            if parts[5] == "export.csv":
                self._send_download(
                    lambda: build_engineering_snapshot_csv(
                        revision.snapshot,
                        project_name=project.name,
                        revision_id=revision.id,
                        clearance_target_mm=_revision_clearance_target_mm(revision),
                    ).encode("utf-8"),
                    "text/csv; charset=utf-8",
                    "easytowing-multibody-wheel-results.csv",
                )
                return
            if parts[5] == "export.pdf":
                self._send_download(
                    lambda: build_engineering_snapshot_pdf(
                        revision.snapshot,
                        project_name=project.name,
                        revision_id=revision.id,
                        clearance_target_mm=_revision_clearance_target_mm(revision),
                    ),
                    "application/pdf",
                    "easytowing-multibody-evaluation.pdf",
                )
                return
            if parts[5] == "export.svg":
                self._send_download(
                    lambda: build_engineering_snapshot_svg(
                        revision.snapshot,
                        project_name=project.name,
                        revision_id=revision.id,
                        clearance_target_mm=_revision_clearance_target_mm(revision),
                    ).encode("utf-8"),
                    "image/svg+xml; charset=utf-8",
                    "easytowing-multibody-sketch.svg",
                )
                return
            if parts[5] == "export.dxf":
                self._send_download(
                    lambda: build_engineering_snapshot_dxf(
                        revision.snapshot,
                        project_name=project.name,
                        revision_id=revision.id,
                        clearance_target_mm=_revision_clearance_target_mm(revision),
                    ).encode("utf-8"),
                    "application/dxf; charset=utf-8",
                    "easytowing-multibody-sketch.dxf",
                )
                return
            self._send_download(
                lambda: build_engineering_snapshot_png(
                    revision.snapshot,
                    project_name=project.name,
                    revision_id=revision.id,
                    clearance_target_mm=_revision_clearance_target_mm(revision),
                ),
                "image/png",
                "easytowing-multibody-snapshot.png",
            )
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "release.json"
        ):
            try:
                principal = self._principal("report:read")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                revision = project.get_revision(parts[4])
                if revision is None:
                    self.send_error(404, "Revision not found")
                    return
                approval = SAAS_CONTROL.get_approval(principal, parts[2], parts[4])
                if approval is None or approval.status != ApprovalStatus.APPROVED:
                    self._send_json(
                        {
                            "error_code": "RELEASE_NOT_APPROVED",
                            "message": "Independent approval is required before a controlled release manifest can be exported.",
                        },
                        status=409,
                    )
                    return
                _require_engineering_pass_for_approval(revision)
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ValueError as error:
                self._send_json({"error_code": "RELEASE_BLOCKED", "message": str(error)}, status=409)
                return
            engineering = (
                evaluate_engineering_snapshot(
                    revision.snapshot or {},
                    clearance_target_mm=_revision_clearance_target_mm(revision),
                )
                if revision.combination_config is not None
                else {"status": "PASS", "basis": "accepted hard-feasible optimization"}
            )
            acceptance = (revision.snapshot or {}).get("monroc_acceptance") or {}
            if ARTIFACT_STORAGE_REQUIRED and ARTIFACT_BLOB_STORE is None:
                self._send_json(
                    {
                        "error_code": "RELEASE_STORAGE_NOT_CONFIGURED",
                        "message": "Controlled release requires EASYTOWING_ARTIFACT_STORAGE_DIR when artifact retention is required.",
                    },
                    status=409,
                )
                return
            artifact_id = f"artifact_{uuid4().hex[:12]}"
            artifact_created_at = datetime.now(timezone.utc)
            artifact_filename = "easytowing-controlled-release.json"
            artifact_storage_backend = (
                ARTIFACT_BLOB_STORE.backend
                if ARTIFACT_BLOB_STORE is not None
                else "response-only"
            )
            manifest = {
                "release_status": "APPROVED",
                "artifact_type": "controlled-engineering-release-manifest",
                "artifact": {
                    "id": artifact_id,
                    "filename": artifact_filename,
                    "generated_at": artifact_created_at.isoformat(),
                    "storage_backend": artifact_storage_backend,
                },
                "project": _project_summary_payload(project),
                "revision": _revision_payload(revision, include_snapshot=True),
                "approval": serialize_approval(approval),
                "engineering": engineering,
                "acceptance": acceptance,
                "release_controls": {
                    "engineering_pass": True,
                    "monroc_acceptance_pass": True,
                    "acceptance_evaluator_id": acceptance.get("evaluator_id"),
                    "independent_approval": True,
                    "submitter": approval.submitted_by,
                    "approver": approval.decided_by,
                    "same_actor": approval.submitted_by == approval.decided_by,
                },
            }
            content = json.dumps(manifest, indent=2).encode("utf-8")
            if ARTIFACT_BLOB_STORE is not None:
                try:
                    ARTIFACT_BLOB_STORE.put(artifact_id, content)
                except ArtifactStorageError as error:
                    self._send_json(
                        {"error_code": "ARTIFACT_STORAGE_FAILED", "message": str(error)},
                        status=500,
                    )
                    return
            try:
                artifact = SAAS_CONTROL.record_artifact(
                    principal,
                    project_id=project.id,
                    revision_id=revision.id,
                    artifact_type=manifest["artifact_type"],
                    filename=artifact_filename,
                    content=content,
                    artifact_id=artifact_id,
                    created_at=artifact_created_at,
                    storage_backend=artifact_storage_backend,
                )
            except SaaSAuthorizationError as error:
                if ARTIFACT_BLOB_STORE is not None:
                    ARTIFACT_BLOB_STORE.delete(artifact_id)
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ValueError as error:
                if ARTIFACT_BLOB_STORE is not None:
                    ARTIFACT_BLOB_STORE.delete(artifact_id)
                self._send_json({"error_code": "ARTIFACT_RECORDING_FAILED", "message": str(error)}, status=500)
                return
            self._send_download(
                lambda: content,
                "application/json; charset=utf-8",
                artifact_filename,
                extra_headers={
                    "X-EasyTowing-Artifact-Id": artifact.id,
                    "X-EasyTowing-Artifact-SHA256": artifact.content_sha256,
                    "X-EasyTowing-Artifact-Storage": artifact.storage_backend,
                },
            )
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "approval-history"
        ):
            try:
                principal = self._principal("project:read")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                if project.get_revision(parts[4]) is None:
                    self.send_error(404, "Revision not found")
                    return
                events = SAAS_CONTROL.approval_history(principal, parts[2], parts[4])
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            self._send_json({"events": [serialize_audit_event(event) for event in events]})
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "artifacts"
        ):
            try:
                principal = self._principal("report:read")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                if project.get_revision(parts[4]) is None:
                    self.send_error(404, "Revision not found")
                    return
                artifacts = SAAS_CONTROL.list_artifacts(principal, parts[2], parts[4])
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            serialized_artifacts = []
            for artifact in artifacts:
                payload = serialize_artifact(artifact)
                if artifact.storage_backend == "filesystem" and ARTIFACT_BLOB_STORE is not None:
                    payload["download_url"] = (
                        f"/api/projects/{parts[2]}/revisions/{parts[4]}/artifacts/{artifact.id}"
                    )
                serialized_artifacts.append(payload)
            self._send_json({"artifacts": serialized_artifacts})
            return

        if (
            len(parts) == 7
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "artifacts"
        ):
            try:
                principal = self._principal("report:read")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                if project.get_revision(parts[4]) is None:
                    self.send_error(404, "Revision not found")
                    return
                artifacts = SAAS_CONTROL.list_artifacts(principal, parts[2], parts[4])
                artifact = next(
                    (item for item in artifacts if item.id == parts[6]),
                    None,
                )
                if artifact is None:
                    self.send_error(404, "Artifact not found")
                    return
                if artifact.storage_backend != "filesystem" or ARTIFACT_BLOB_STORE is None:
                    self._send_json(
                        {
                            "error_code": "ARTIFACT_NOT_RETAINED",
                            "message": "This artifact was delivered as a response and is not retained by the configured storage backend.",
                        },
                        status=409,
                    )
                    return
                content = ARTIFACT_BLOB_STORE.read(
                    artifact.id,
                    expected_sha256=artifact.content_sha256,
                    expected_size=artifact.byte_size,
                )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ArtifactStorageError as error:
                self._send_json(
                    {"error_code": "ARTIFACT_UNAVAILABLE", "message": str(error)},
                    status=410,
                )
                return
            content_type = mimetypes.guess_type(artifact.filename)[0] or "application/octet-stream"
            self._send_download(lambda: content, content_type, artifact.filename)
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "approval"
        ):
            try:
                principal = self._principal("project:read")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                revision = project.get_revision(parts[4])
                if revision is None:
                    self.send_error(404, "Revision not found")
                    return
                approval = SAAS_CONTROL.get_approval(principal, parts[2], parts[4])
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            self._send_json({"approval": None if approval is None else serialize_approval(approval)})
            return

        if len(parts) == 3 and parts[0] == "api" and parts[1] == "projects":
            try:
                principal = self._principal("project:read")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
            if project is None:
                self.send_error(404, "Project not found")
                return
            self._send_json(_project_detail_payload(project))
            return
        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            try:
                principal = self._principal("project:read")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
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
            try:
                principal = self._principal("report:read")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
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

        if parsed.path == "/api/auth/bootstrap":
            configured_token = os.environ.get("EASYTOWING_BOOTSTRAP_TOKEN", "")
            provided_token = str(body.get("bootstrap_token", ""))
            if not configured_token or not hmac.compare_digest(provided_token, configured_token):
                self._send_json({"error_code": "FORBIDDEN", "message": "Bootstrap is not enabled."}, status=403)
                return
            try:
                account = SAAS_CONTROL.bootstrap_admin(
                    str(body.get("organization_id", "")),
                    str(body.get("email", "")),
                    str(body.get("password", "")),
                    display_name=str(body.get("display_name", "")),
                    organization_name=str(body.get("organization_name", body.get("organization_id", ""))),
                )
            except SaaSBootstrapError as error:
                self._send_json({"error_code": "BOOTSTRAP_ALREADY_COMPLETED", "message": str(error)}, status=409)
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            self._send_json(
                {
                    "user_id": account.id,
                    "organization_id": account.organization_id,
                    "email": account.email,
                    "role": account.role.value,
                },
                status=201,
            )
            return

        if parsed.path == "/api/auth/login":
            try:
                token, principal = SAAS_CONTROL.login(
                    str(body.get("organization_id", "")),
                    str(body.get("email", "")),
                    str(body.get("password", "")),
                )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "INVALID_CREDENTIALS", "message": str(error)}, status=401)
                return
            self._send_json({"token": token, "principal": principal_payload(principal)})
            return

        if parsed.path == "/api/auth/logout":
            authorization = self.headers.get("Authorization", "")
            token = authorization.removeprefix("Bearer ").strip()
            if token:
                SAAS_CONTROL.logout(token)
            self._send_json({"status": "logged_out"})
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "cad-source"
        ):
            try:
                principal = self._principal("project:write")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                revision = project.get_revision(parts[4])
                if revision is None:
                    self.send_error(404, "Revision not found")
                    return
                if ARTIFACT_BLOB_STORE is None:
                    self._send_json(
                        {
                            "error_code": "CAD_SOURCE_STORAGE_NOT_CONFIGURED",
                            "message": "Configure EASYTOWING_ARTIFACT_STORAGE_DIR before retaining CAD source bytes.",
                        },
                        status=409,
                    )
                    return
                source_name = str(body.get("source_name", "")).strip()
                if (
                    not source_name
                    or len(source_name) > 255
                    or source_name in {".", ".."}
                    or any(character in source_name for character in "/\\\r\n\x00")
                ):
                    raise ValueError("source_name must be a safe filename without path separators")
                dxf_text = body.get("dxf_text")
                if not isinstance(dxf_text, str) or not dxf_text.strip():
                    raise ValueError("dxf_text is required")
                content = dxf_text.encode("utf-8")
                if len(content) > MAX_CAD_SOURCE_BYTES:
                    raise ValueError(
                        f"CAD source exceeds the {MAX_CAD_SOURCE_BYTES // (1024 * 1024)} MiB limit"
                    )
                content_sha256 = hashlib.sha256(content).hexdigest()
                vehicle_config = revision.vehicle_config or {}
                cad_source = vehicle_config.get("cad_source") if isinstance(vehicle_config, dict) else None
                if not isinstance(cad_source, dict):
                    self._send_json(
                        {
                            "error_code": "CAD_SOURCE_METADATA_MISSING",
                            "message": "The saved revision does not contain confirmed CAD source metadata.",
                        },
                        status=409,
                    )
                    return
                if (
                    str(cad_source.get("source_name", "")) != source_name
                    or str(cad_source.get("source_sha256", "")).lower() != content_sha256
                ):
                    self._send_json(
                        {
                            "error_code": "CAD_SOURCE_MISMATCH",
                            "message": "The uploaded source does not match the filename and SHA-256 saved on this revision.",
                        },
                        status=409,
                    )
                    return
                existing = next(
                    (
                        artifact
                        for artifact in SAAS_CONTROL.list_artifacts(principal, project.id, revision.id)
                        if artifact.artifact_type == "cad-source-dxf"
                        and artifact.content_sha256 == content_sha256
                    ),
                    None,
                )
                if existing is not None:
                    payload = serialize_artifact(existing)
                    payload["download_url"] = (
                        f"/api/projects/{project.id}/revisions/{revision.id}/artifacts/{existing.id}"
                    )
                    self._send_json({"artifact": payload})
                    return
                artifact_id = f"artifact_{uuid4().hex[:12]}"
                created_at = datetime.now(timezone.utc)
                ARTIFACT_BLOB_STORE.put(
                    artifact_id,
                    content,
                    expected_sha256=content_sha256,
                    expected_size=len(content),
                )
                try:
                    artifact = SAAS_CONTROL.record_artifact(
                        principal,
                        project_id=project.id,
                        revision_id=revision.id,
                        artifact_type="cad-source-dxf",
                        filename=source_name,
                        content=content,
                        artifact_id=artifact_id,
                        created_at=created_at,
                        storage_backend=ARTIFACT_BLOB_STORE.backend,
                    )
                except Exception:
                    ARTIFACT_BLOB_STORE.delete(artifact_id)
                    raise
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ArtifactStorageError as error:
                self._send_json({"error_code": "ARTIFACT_STORAGE_FAILED", "message": str(error)}, status=500)
                return
            except (TypeError, ValueError) as error:
                self._send_json({"error_code": "CAD_SOURCE_INVALID", "message": str(error)}, status=400)
                return
            payload = serialize_artifact(artifact)
            payload["download_url"] = (
                f"/api/projects/{project.id}/revisions/{revision.id}/artifacts/{artifact.id}"
            )
            self._send_json({"artifact": payload}, status=201)
            return

        if parsed.path == "/api/jobs/optimization":
            try:
                principal = self._principal("job:submit")
                request = dict(body)
                project_id = request.get("project_id")
                if DATABASE_URL:
                    job = SAAS_CONTROL.create_job(
                        principal,
                        kind="optimization",
                        request=request,
                        project_id=None if project_id is None else str(project_id),
                    )
                else:
                    assert SAAS_JOBS is not None
                    job = SAAS_JOBS.submit(
                        principal,
                        kind="optimization",
                        request=request,
                        project_id=None if project_id is None else str(project_id),
                        operation=_optimization_job_payload,
                    )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            self._send_json(serialize_job(job), status=202)
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

        if parsed.path == "/api/calculate/mechanism":
            try:
                graph = _parse_mechanism_graph(body.get("mechanism", body))
                driven_positions = _parse_driven_positions(body.get("driven_positions"))
                tolerance_mm = _config_float(body, "geometric_tolerance_mm", 0.01)
                maximum_iterations = int(body.get("maximum_iterations", 80))
                assert tolerance_mm is not None
                state = solve_mechanism_graph(
                    graph,
                    driven_positions,
                    geometric_tolerance_mm=tolerance_mm,
                    maximum_iterations=maximum_iterations,
                )
                payload = _mechanism_graph_payload(graph, state)
                payload["clearance"] = _clearance_report_payload(
                    analyze_clearance(
                        build_mechanism_graph_clearance_items(graph, state)
                    )
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

        if parsed.path == "/api/calculate/combination":
            try:
                combination = _parse_vehicle_combination(body.get("combination", body))
                root_pose = _parse_pose(body.get("root_pose"), field_name="root_pose")
                root_turn_radius_mm = _config_float(
                    body,
                    "root_turn_radius_mm",
                    None,
                    allow_none=True,
                )
                tolerance_mm = _config_float(body, "constraint_tolerance_mm", 0.01)
                assert tolerance_mm is not None
                solution = solve_combination_kinematics(
                    combination,
                    root_pose=root_pose,
                    root_turn_radius_mm=root_turn_radius_mm,
                    constraint_tolerance_mm=tolerance_mm,
                )
                payload = _combination_kinematic_payload(
                    combination,
                    solution,
                    root_pose=root_pose,
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

        if parsed.path == "/api/calculate/combination-sweep":
            try:
                combination = _parse_vehicle_combination(body.get("combination", body))
                graph = _parse_mechanism_graph(
                    body.get("mechanism_graph", body.get("mechanism"))
                )
                mechanism_drivers = _parse_mechanism_drivers(
                    body.get("mechanism_drivers")
                )
                steering_assignments = _parse_steering_assignments(
                    body.get("steering_assignments")
                )
                root_turn_radius_mm = _config_float(
                    body,
                    "root_turn_radius_mm",
                    None,
                    allow_none=True,
                )
                beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                step_deg = _config_float(body, "step_deg", 5.0)
                clearance_target_mm = _config_float(body, "clearance_target_mm", 20.0)
                assert step_deg is not None and clearance_target_mm is not None
                payload = build_combination_sweep_payload(
                    combination,
                    root_turn_radius_mm=root_turn_radius_mm,
                    mechanism_graph=graph,
                    mechanism_drivers=mechanism_drivers,
                    steering_assignments=steering_assignments,
                    beta_min_deg=beta_min_deg,
                    beta_max_deg=beta_max_deg,
                    step_deg=step_deg,
                    primary_joint_id=(
                        None
                        if body.get("primary_joint_id") is None
                        else str(body["primary_joint_id"])
                    ),
                    clearance_target_mm=clearance_target_mm,
                    joint_ranges=body.get(
                        "joint_ranges",
                        body.get("combination", {}).get("joint_ranges")
                        if isinstance(body.get("combination"), dict)
                        else None,
                    ),
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

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "acceptance"
        ):
            try:
                principal = self._principal("project:write")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                revision = project.get_revision(parts[4])
                if revision is None:
                    self.send_error(404, "Revision not found")
                    return
                existing_approval = SAAS_CONTROL.get_approval(principal, parts[2], parts[4])
                if existing_approval is not None and existing_approval.status in {
                    ApprovalStatus.SUBMITTED,
                    ApprovalStatus.APPROVED,
                }:
                    raise ValueError("Acceptance criteria cannot be changed after submission; create a new revision.")
                raw_criteria = body.get("criteria", body.get("acceptance_criteria"))
                if not isinstance(raw_criteria, dict):
                    raise ValueError("criteria must be an object with explicit Monroc limits.")
                criteria = MonrocAcceptanceCriteria.from_dict(raw_criteria)
                result = evaluate_monroc_acceptance(revision.snapshot, criteria)
                criteria_approval = _monroc_acceptance_profile_status(
                    principal.organization_id,
                    criteria,
                )
                result = {
                    **result,
                    "criteria_approval": criteria_approval,
                }
                if result.get("status") == "PASS" and criteria_approval.get("status") != "APPROVED":
                    result["status"] = "UNAPPROVED"
                    result["message"] = str(criteria_approval["message"])
                acceptance = {
                    "criteria": criteria.to_dict(),
                    "result": result,
                    "criteria_approval": criteria_approval,
                    "evaluated_by": principal.user_id,
                    "evaluator_id": ACCEPTANCE_EVALUATOR_ID,
                }
                stored_revision = PROJECT_STORE.record_acceptance(
                    parts[2],
                    parts[4],
                    acceptance,
                    principal.organization_id,
                )
                SAAS_CONTROL.bind_project(principal, parts[2])
                SAAS_CONTROL.record_event(
                    principal,
                    project_id=parts[2],
                    event_type="ACCEPTANCE_EVALUATED",
                    target_type="revision",
                    target_id=parts[4],
                    metadata={
                        "case_id": criteria.case_id,
                        "status": result["status"],
                    },
                )
                latest_project = PROJECT_STORE.get_project(
                    parts[2],
                    principal.organization_id,
                ) or project
                self._send_json(
                    {
                        "acceptance": result,
                        "revision": _revision_payload(stored_revision, include_snapshot=True),
                        "project": _project_detail_payload(latest_project),
                    },
                    status=201,
                )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
            except (TypeError, ValueError) as error:
                self._send_json({"error_code": "ACCEPTANCE_INVALID", "message": str(error)}, status=400)
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "reviewer"
        ):
            try:
                principal = self._principal("user:manage")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                if project.get_revision(parts[4]) is None:
                    self.send_error(404, "Revision not found")
                    return
                reviewer_user_id = body.get("reviewer_user_id")
                if reviewer_user_id is not None and not isinstance(reviewer_user_id, str):
                    raise ValueError("reviewer_user_id must be a string or null.")
                approval = SAAS_CONTROL.assign_reviewer(
                    principal,
                    parts[2],
                    parts[4],
                    reviewer_user_id,
                )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ValueError as error:
                self._send_json({"error_code": "REVIEWER_ASSIGNMENT_INVALID", "message": str(error)}, status=409)
                return
            self._send_json({"approval": serialize_approval(approval)}, status=201)
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "submit"
        ):
            try:
                principal = self._principal("revision:submit")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                if project.get_revision(parts[4]) is None:
                    self.send_error(404, "Revision not found")
                    return
                SAAS_CONTROL.bind_project(principal, parts[2])
                approval = SAAS_CONTROL.submit_revision(
                    principal,
                    parts[2],
                    parts[4],
                    note=str(body.get("note", "")),
                )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ValueError as error:
                self.send_error(409, str(error))
                return
            self._send_json({"approval": serialize_approval(approval)}, status=201)
            return

        if (
            len(parts) == 6
            and parts[0] == "api"
            and parts[1] == "projects"
            and parts[3] == "revisions"
            and parts[5] == "approval"
        ):
            try:
                principal = self._principal("revision:approve")
                project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
                if project is None:
                    self.send_error(404, "Project not found")
                    return
                revision = project.get_revision(parts[4])
                if revision is None:
                    self.send_error(404, "Revision not found")
                    return
                approved = _parse_required_bool(body, "approved")
                if approved:
                    _require_engineering_pass_for_approval(revision)
                approval = SAAS_CONTROL.decide_revision(
                    principal,
                    parts[2],
                    parts[4],
                    approved=approved,
                    note=str(body.get("note", "")),
                )
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except ValueError as error:
                self._send_json({"error_code": "APPROVAL_BLOCKED", "message": str(error)}, status=409)
                return
            self._send_json({"approval": serialize_approval(approval)}, status=201)
            return

        if parsed.path == "/api/calculate/kinematic":
            try:
                beta_deg = _config_float(body, "beta_deg", 0.0)
                raw_combination = body.get("combination")
                combination = (
                    None
                    if raw_combination is None
                    else _parse_vehicle_combination(raw_combination)
                )
                if combination is None:
                    wheelbase_mm, track_mm, _vehicle_config, vehicle = _project_vehicle_inputs(body)
                else:
                    vehicle = None
                    resolved_axles = combination.resolve_mounted_axles()
                    x_values = [axle.center.x_mm for axle in resolved_axles]
                    wheelbase_mm = (
                        max(x_values) - min(x_values)
                        if len(x_values) > 1
                        else 4360.0
                    )
                    track_mm = max((axle.track_mm for axle in resolved_axles), default=2500.0)
                assert beta_deg is not None and wheelbase_mm is not None and track_mm is not None
                if not math.isfinite(beta_deg):
                    raise ValueError("beta_deg must be finite")
                rig = _parse_linkage_rig(body.get("linkage", body.get("linkage_config")))
                raw_mechanism = body.get("mechanism_graph")
                mechanism_graph = (
                    None
                    if raw_mechanism is None
                    else _parse_mechanism_graph(raw_mechanism)
                )
                mechanism_drivers = _parse_mechanism_drivers(
                    body.get("mechanism_drivers")
                )
                steering_assignments = _parse_steering_assignments(
                    body.get("steering_assignments")
                )
                root_turn_radius_mm = _config_float(
                    body,
                    "root_turn_radius_mm",
                    None,
                    allow_none=True,
                )
                clearance_target_mm = _config_float(body, "clearance_target_mm", 20.0)
                assert clearance_target_mm is not None
                payload = build_demo_payload(
                    beta_deg,
                    wheelbase_mm=wheelbase_mm,
                    track_mm=track_mm,
                    linkage_rig=rig,
                    vehicle=vehicle,
                    combination=combination,
                    root_turn_radius_mm=root_turn_radius_mm,
                    mechanism_graph=mechanism_graph,
                    mechanism_drivers=mechanism_drivers,
                    steering_assignments=steering_assignments,
                    clearance_target_mm=clearance_target_mm,
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
                raw_combination = body.get("combination", body.get("combination_config"))
                if raw_combination is not None:
                    combination = _parse_vehicle_combination(raw_combination)
                    raw_graph = body.get("mechanism_graph", body.get("mechanism_graph_config"))
                    if raw_graph is None:
                        raise ValueError("A graph optimization request requires mechanism_graph.")
                    graph = _parse_mechanism_graph(raw_graph)
                    drivers = _parse_mechanism_drivers(body.get("mechanism_drivers"))
                    assignments = _parse_steering_assignments(body.get("steering_assignments"))
                    beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                    root_turn_radius_mm = _config_float(
                        body,
                        "root_turn_radius_mm",
                        None,
                        allow_none=True,
                    )
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
                    payload = _mechanism_graph_optimization_payload(
                        mode=mode,
                        combination=combination,
                        graph=graph,
                        drivers=drivers,
                        assignments=assignments,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        primary_joint_id=(
                            None
                            if body.get("primary_joint_id") in (None, "")
                            else str(body["primary_joint_id"])
                        ),
                        root_turn_radius_mm=root_turn_radius_mm,
                        clearance_target_mm=clearance_target_mm,
                        enabled_ids=_parse_enabled_ids(body.get("enabled_ids")),
                        weights=OptimizationWeights(
                            steering_error=weight_values["steering_error"] or 0.0,
                            clearance=weight_values["clearance"] or 0.0,
                            clearance_violation=weight_values["clearance_violation"] or 0.0,
                            failure=weight_values["failure"] or 0.0,
                            preferred=weight_values["preferred"] or 0.0,
                            complexity=weight_values["complexity"] or 0.0,
                            synchronization_error=weight_values["synchronization_error"] or 0.0,
                        ),
                        design_cases=_parse_design_cases(body.get("design_cases")),
                        joint_ranges=body.get(
                            "joint_ranges",
                            raw_combination.get("joint_ranges")
                            if isinstance(raw_combination, dict)
                            else None,
                        ),
                    )
                    self._send_json(payload)
                    return
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
                principal = self._principal("project:write")
                beta_deg = float(body.get("beta_deg", 0.0))
                beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                wheelbase_mm, track_mm, vehicle_config, vehicle = _project_vehicle_inputs(body)
                design_cases = _parse_design_cases(body.get("design_cases"))
                linkage_config = _validated_linkage_config(
                    body.get("linkage_config", body.get("linkage"))
                )
                linkage_rig = None if linkage_config is None else _parse_linkage_rig(linkage_config)
                (
                    combination_config,
                    root_turn_radius_mm,
                    mechanism_graph_config,
                    mechanism_drivers,
                    steering_assignments,
                    engineering_snapshot,
                ) = _project_combination_inputs(body, beta_deg=beta_deg)
            except EngineeringError as error:
                self._send_json({"error_code": error.code, "message": str(error)}, status=422)
                return
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
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
                organization_id=principal.organization_id,
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
                combination_config=combination_config,
                root_turn_radius_mm=root_turn_radius_mm,
                mechanism_graph_config=mechanism_graph_config,
                mechanism_drivers=[
                    {
                        "point_id": driver.point_id,
                        "center": _point_payload(driver.center),
                        "radius_mm": driver.radius_mm,
                        "neutral_angle_rad": driver.neutral_angle_rad,
                        "input_ratio": driver.input_ratio,
                        "phase_offset_rad": driver.phase_offset_rad,
                        "input_id": driver.input_id,
                    }
                    for driver in mechanism_drivers
                ],
                steering_assignments=[
                    {
                        "output_id": assignment.output_id,
                        "wheel_id": assignment.wheel_id,
                        "ratio": assignment.ratio,
                        "phase_offset_rad": assignment.phase_offset_rad,
                    }
                    for assignment in steering_assignments
                ],
                engineering_snapshot=engineering_snapshot,
            )
            SAAS_CONTROL.bind_project(principal, project.id)
            self._send_json({"project": _project_detail_payload(project)}, status=201)
            return

        if len(parts) == 2 and parts[0] == "api" and parts[1] == "import.dxf":
            dxf_text = str(body.get("dxf_text", ""))
            if not dxf_text.strip():
                self.send_error(400, "dxf_text is required")
                return
            source_name = str(body.get("source_name", ""))
            source_units = body.get("source_units")
            coordinate_system = body.get("coordinate_system")
            confirm_metadata = body.get("confirm_metadata", False) is True
            try:
                role_overrides = _parse_role_overrides(body.get("role_overrides"))
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            try:
                report = analyze_dxf_import(
                    dxf_text,
                    source_name=source_name,
                    source_units=None if source_units is None else str(source_units),
                    coordinate_system=(
                        None if coordinate_system is None else str(coordinate_system)
                    ),
                    confirm_metadata=confirm_metadata,
                )
                if role_overrides:
                    if not report.import_ready:
                        self.send_error(
                            400,
                            "CAD activation requires confirmed source units and coordinate_system.",
                        )
                        return
                    report = apply_dxf_role_overrides(report, role_overrides)
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            payload = serialize_dxf_import_report(report)
            self._send_json(payload)
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "optimization":
            try:
                principal = self._principal("project:write")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
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
            raw_combination = body.get("combination_config", body.get("combination"))
            if raw_combination is not None:
                try:
                    combination = _parse_vehicle_combination(raw_combination)
                    raw_graph = body.get("mechanism_graph_config", body.get("mechanism_graph"))
                    if raw_graph is None:
                        raise ValueError("A graph optimization request requires mechanism_graph_config.")
                    graph = _parse_mechanism_graph(raw_graph)
                    drivers = _parse_mechanism_drivers(body.get("mechanism_drivers"))
                    assignments = _parse_steering_assignments(body.get("steering_assignments"))
                    beta_deg = float(body.get("beta_deg", 0.0))
                    beta_min_deg, beta_max_deg = _parse_articulation_bounds(body)
                    root_turn_radius_mm = _config_float(
                        body,
                        "root_turn_radius_mm",
                        None,
                        allow_none=True,
                    )
                    clearance_target_mm = float(body.get("clearance_target_mm", 20.0))
                    enabled_ids = _parse_enabled_ids(body.get("enabled_ids"))
                    design_cases = _parse_design_cases(body.get("design_cases"))
                    optimization_mode = str(body.get("optimization_mode", "quick"))
                    if optimization_mode not in {"quick", "full"}:
                        raise ValueError("optimization_mode must be quick or full")
                    weights = OptimizationWeights(
                        steering_error=float(body.get("steering_error_weight", 1.0)),
                        synchronization_error=float(body.get("synchronization_error_weight", 0.5)),
                        clearance=float(body.get("clearance_weight", 12.0)),
                        clearance_violation=float(body.get("clearance_violation_weight", 250.0)),
                        failure=float(body.get("failure_weight", 100000.0)),
                        preferred=float(body.get("preferred_weight", 0.05)),
                        complexity=float(body.get("complexity_weight", 0.02)),
                    )
                    problem = build_mechanism_graph_optimization_problem(
                        combination=combination,
                        graph=graph,
                        drivers=drivers,
                        assignments=assignments,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        mode=optimization_mode,  # type: ignore[arg-type]
                        primary_joint_id=(
                            None
                            if body.get("primary_joint_id") in (None, "")
                            else str(body["primary_joint_id"])
                        ),
                        root_turn_radius_mm=root_turn_radius_mm,
                        clearance_target_mm=clearance_target_mm,
                        enabled_ids=enabled_ids,
                        design_cases=design_cases,
                        weights=weights,
                        joint_ranges=body.get(
                            "joint_ranges",
                            raw_combination.get("joint_ranges")
                            if isinstance(raw_combination, dict)
                            else None,
                        ),
                    )
                    graph_result = optimize_mechanism_graph_problem(problem)
                    optimization_payload = _mechanism_graph_optimization_payload(
                        mode=optimization_mode,
                        combination=combination,
                        graph=graph,
                        drivers=drivers,
                        assignments=assignments,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        primary_joint_id=problem.primary_joint_id,
                        root_turn_radius_mm=root_turn_radius_mm,
                        clearance_target_mm=clearance_target_mm,
                        enabled_ids=enabled_ids,
                        weights=weights,
                        design_cases=design_cases,
                        joint_ranges=body.get(
                            "joint_ranges",
                            raw_combination.get("joint_ranges")
                            if isinstance(raw_combination, dict)
                            else None,
                        ),
                        graph_result=graph_result,
                    )
                    optimized_snapshot = build_demo_payload(
                        beta_deg,
                        combination=combination,
                        root_turn_radius_mm=root_turn_radius_mm,
                        mechanism_graph=graph,
                        mechanism_drivers=graph_result.optimized_drivers,
                        steering_assignments=graph_result.optimized_assignments,
                        clearance_target_mm=clearance_target_mm,
                    )
                    optimized_snapshot["optimization"] = optimization_payload
                    optimized_snapshot["sweep_validation"] = build_combination_sweep_payload(
                        combination,
                        root_turn_radius_mm=root_turn_radius_mm,
                        mechanism_graph=graph,
                        mechanism_drivers=graph_result.optimized_drivers,
                        steering_assignments=graph_result.optimized_assignments,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        step_deg=float(body.get("sweep_step_deg", 1.0)),
                        primary_joint_id=problem.primary_joint_id,
                        clearance_target_mm=clearance_target_mm,
                        joint_ranges=body.get(
                            "joint_ranges",
                            raw_combination.get("joint_ranges")
                            if isinstance(raw_combination, dict)
                            else None,
                        ),
                    )
                    if optimized_snapshot["sweep_validation"]["status"] != "PASS":
                        raise OptimizationNoFeasibleSolutionError(
                            tuple(
                                str(item.get("checks", ["SWEEP_VALIDATION_FAILED"])[0])
                                for item in optimized_snapshot["sweep_validation"].get("violations", [])
                            )
                        )
                    resolved_axles = combination.resolve_mounted_axles()
                    x_values = [axle.center.x_mm for axle in resolved_axles]
                    revision = PROJECT_STORE.append_revision(
                        project.id,
                        organization_id=principal.organization_id,
                        beta_deg=beta_deg,
                        optimization_mode=optimization_mode,
                        note=str(body.get("note", "Applied graph optimization")),
                        enabled_ids=enabled_ids,
                        accepted_optimization=False,
                        beta_min_deg=beta_min_deg,
                        beta_max_deg=beta_max_deg,
                        design_cases=design_cases,
                        wheelbase_mm=(
                            max(x_values) - min(x_values)
                            if len(x_values) > 1
                            else 4360.0
                        ),
                        track_mm=max((axle.track_mm for axle in resolved_axles), default=2500.0),
                        combination_config=json.loads(json.dumps(raw_combination)),
                        root_turn_radius_mm=root_turn_radius_mm,
                        mechanism_graph_config=json.loads(json.dumps(raw_graph)),
                        mechanism_drivers=[
                            _mechanism_driver_payload(driver)
                            for driver in graph_result.optimized_drivers
                        ],
                        steering_assignments=[
                            _mechanism_assignment_payload(assignment)
                            for assignment in graph_result.optimized_assignments
                        ],
                        engineering_snapshot=optimized_snapshot,
                    )
                    SAAS_CONTROL.bind_project(principal, project.id)
                    latest_project = PROJECT_STORE.get_project(
                        project.id,
                        principal.organization_id,
                    ) or project
                    self._send_json(
                        {
                            "status": "applied",
                            "validation": _optimization_metrics_payload(
                                graph_result.result.optimized_metrics
                            ),
                            "project": _project_detail_payload(latest_project),
                            "revision": _revision_payload(revision, include_snapshot=True),
                        },
                        status=201,
                    )
                except EngineeringError as error:
                    self._send_json(
                        {"error_code": error.code, "message": str(error)},
                        status=422,
                    )
                except (TypeError, ValueError) as error:
                    self.send_error(400, str(error))
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
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            except (TypeError, ValueError) as error:
                self.send_error(400, str(error))
                return
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            if linkage_rig is None or linkage_config is None:
                self.send_error(400, "linkage_config is required when applying an optimized design")
                return
            try:
                clearance_target_mm = float(body.get("clearance_target_mm", 20.0))
                validation_problem = build_reference_optimization_problem(
                    mode=optimization_mode,
                    enabled_ids=set(),
                    clearance_target_mm=clearance_target_mm,
                    design_cases=design_cases,
                    base_rig=linkage_rig,
                    vehicle=vehicle,
                )
                validation_result = optimize_linkage_problem(validation_problem)
            except EngineeringError as error:
                self._send_json(
                    {"error_code": error.code, "message": str(error)},
                    status=422,
                )
                return
            except ValueError as error:
                self.send_error(400, str(error))
                return
            note = str(body.get("note", "Applied optimized design"))
            revision = PROJECT_STORE.append_revision(
                project.id,
                organization_id=principal.organization_id,
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
            SAAS_CONTROL.bind_project(principal, project.id)
            latest_project = PROJECT_STORE.get_project(project.id, principal.organization_id) or project
            self._send_json(
                {
                    "status": "applied",
                    "validation": _optimization_metrics_payload(
                        validation_result.optimized_metrics
                    ),
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision, include_snapshot=True),
                },
                status=201,
            )
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            try:
                principal = self._principal("project:write")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
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
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
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
                (
                    combination_config,
                    root_turn_radius_mm,
                    mechanism_graph_config,
                    mechanism_drivers,
                    steering_assignments,
                    engineering_snapshot,
                ) = _project_combination_inputs(body, beta_deg=beta_deg)
            except EngineeringError as error:
                self._send_json({"error_code": error.code, "message": str(error)}, status=422)
                return
            except ValueError as error:
                self.send_error(400, str(error))
                return
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            revision = PROJECT_STORE.append_revision(
                project.id,
                organization_id=principal.organization_id,
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
                combination_config=combination_config,
                root_turn_radius_mm=root_turn_radius_mm,
                mechanism_graph_config=mechanism_graph_config,
                mechanism_drivers=[
                    {
                        "point_id": driver.point_id,
                        "center": _point_payload(driver.center),
                        "radius_mm": driver.radius_mm,
                        "neutral_angle_rad": driver.neutral_angle_rad,
                        "input_ratio": driver.input_ratio,
                        "phase_offset_rad": driver.phase_offset_rad,
                        "input_id": driver.input_id,
                    }
                    for driver in mechanism_drivers
                ],
                steering_assignments=[
                    {
                        "output_id": assignment.output_id,
                        "wheel_id": assignment.wheel_id,
                        "ratio": assignment.ratio,
                        "phase_offset_rad": assignment.phase_offset_rad,
                    }
                    for assignment in steering_assignments
                ],
                engineering_snapshot=engineering_snapshot,
            )
            SAAS_CONTROL.bind_project(principal, project.id)
            latest_project = PROJECT_STORE.get_project(project.id, principal.organization_id) or project
            self._send_json(
                {
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision),
                },
                status=201,
            )
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "restore":
            try:
                principal = self._principal("project:write")
            except SaaSAuthorizationError as error:
                self._send_json({"error_code": "FORBIDDEN", "message": str(error)}, status=403)
                return
            project = PROJECT_STORE.get_project(parts[2], principal.organization_id)
            if project is None:
                self.send_error(404, "Project not found")
                return
            revision_id = str(body.get("revision_id", ""))
            if not revision_id:
                self.send_error(400, "revision_id is required")
                return
            try:
                revision = PROJECT_STORE.restore_revision(
                    project.id,
                    revision_id,
                    principal.organization_id,
                )
            except KeyError:
                self.send_error(404, "Revision not found")
                return
            latest_project = PROJECT_STORE.get_project(project.id, principal.organization_id) or project
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
    if not DATABASE_URL:
        PROJECT_STORE.ensure_seed_project(LOCAL_DEVELOPER.organization_id)
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
