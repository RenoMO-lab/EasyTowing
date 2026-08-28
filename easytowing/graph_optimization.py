from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
from typing import Iterable

from .actual_steering import (
    actual_steering_errors_deg,
    compare_actual_to_ideal,
    solve_actual_steering_from_graph,
)
from .clearance_model import build_mechanism_graph_clearance_items
from .combination_kinematics import solve_combination_kinematics
from .combination_sweep import (
    JointSweepRange,
    build_joint_sweep_grid,
    normalize_joint_sweep_ranges,
)
from .design_cases import DesignCase
from .errors import EngineeringError, OptimizationNoFeasibleSolutionError, SweepSampleLimitError
from .mechanism_graph import (
    MechanismDriverArc,
    MechanismGraphState,
    MechanismSteeringAssignment,
    PlanarMechanismGraph,
    solve_mechanism_graph,
    resolve_driver_arc_positions,
)
from .model import ArticulationJoint, VehicleCombination
from .optimization import (
    OptimizationMetrics,
    OptimizationMode,
    OptimizationResult,
    OptimizationVariable,
    OptimizationWeights,
    OptimizedVariable,
)
from .collision import analyze_clearance


@dataclass(frozen=True, slots=True)
class MechanismGraphOptimizationProblem:
    combination: VehicleCombination
    graph: PlanarMechanismGraph
    drivers: tuple[MechanismDriverArc, ...]
    assignments: tuple[MechanismSteeringAssignment, ...]
    beta_samples_deg: tuple[float, ...]
    primary_joint_id: str
    root_turn_radius_mm: float | None
    variables: tuple[OptimizationVariable, ...]
    clearance_target_mm: float = 20.0
    weights: OptimizationWeights = OptimizationWeights()
    mode: OptimizationMode = "quick"
    seed: int = 17
    design_cases: tuple[DesignCase, ...] = ()
    sample_weights: tuple[float, ...] = ()
    joint_samples_deg: tuple[dict[str, float], ...] = ()
    joint_ranges: tuple[JointSweepRange, ...] = ()

    def __post_init__(self) -> None:
        if not self.beta_samples_deg:
            raise ValueError("Graph optimization requires at least one articulation sample.")
        if any(not math.isfinite(value) for value in self.beta_samples_deg):
            raise ValueError("Graph optimization samples must be finite.")
        if not math.isfinite(self.clearance_target_mm) or self.clearance_target_mm < 0.0:
            raise ValueError("clearance_target_mm must be a non-negative finite value")
        if self.primary_joint_id not in {joint.id for joint in self.combination.joints}:
            raise ValueError(f"Unknown primary articulation joint {self.primary_joint_id!r}.")
        if self.sample_weights and len(self.sample_weights) != len(self.joint_sample_values()):
            raise ValueError("sample_weights must match beta_samples_deg")

    def joint_sample_values(self) -> tuple[dict[str, float], ...]:
        if self.joint_samples_deg:
            return self.joint_samples_deg
        return tuple(
            {self.primary_joint_id: beta_deg}
            for beta_deg in self.beta_samples_deg
        )


@dataclass(frozen=True, slots=True)
class MechanismGraphOptimizationResult:
    result: OptimizationResult
    baseline_drivers: tuple[MechanismDriverArc, ...]
    optimized_drivers: tuple[MechanismDriverArc, ...]
    baseline_assignments: tuple[MechanismSteeringAssignment, ...]
    optimized_assignments: tuple[MechanismSteeringAssignment, ...]


def _driver_variable_id(point_id: str, field: str) -> str:
    return f"driver:{point_id}:{field}"


def _assignment_variable_id(wheel_id: str, field: str) -> str:
    return f"assignment:{wheel_id}:{field}"


def _default_variables(
    drivers: tuple[MechanismDriverArc, ...],
    assignments: tuple[MechanismSteeringAssignment, ...],
    enabled_ids: Iterable[str] | None,
) -> tuple[OptimizationVariable, ...]:
    variables: list[OptimizationVariable] = []
    for driver in drivers:
        ratio_id = _driver_variable_id(driver.point_id, "input_ratio")
        phase_id = _driver_variable_id(driver.point_id, "phase_offset_deg")
        variables.extend(
            (
                OptimizationVariable(
                    id=ratio_id,
                    current=driver.input_ratio,
                    minimum=-2.0,
                    maximum=2.0,
                    enabled=True,
                    preferred=driver.input_ratio,
                ),
                OptimizationVariable(
                    id=phase_id,
                    current=math.degrees(driver.phase_offset_rad),
                    minimum=math.degrees(driver.phase_offset_rad) - 20.0,
                    maximum=math.degrees(driver.phase_offset_rad) + 20.0,
                    enabled=False,
                    preferred=math.degrees(driver.phase_offset_rad),
                ),
            )
        )
    for assignment in assignments:
        ratio_id = _assignment_variable_id(assignment.wheel_id, "ratio")
        phase_id = _assignment_variable_id(assignment.wheel_id, "phase_offset_deg")
        variables.extend(
            (
                OptimizationVariable(
                    id=ratio_id,
                    current=assignment.ratio,
                    minimum=-1.5,
                    maximum=1.5,
                    enabled=False,
                    preferred=assignment.ratio,
                ),
                OptimizationVariable(
                    id=phase_id,
                    current=math.degrees(assignment.phase_offset_rad),
                    minimum=math.degrees(assignment.phase_offset_rad) - 20.0,
                    maximum=math.degrees(assignment.phase_offset_rad) + 20.0,
                    enabled=False,
                    preferred=math.degrees(assignment.phase_offset_rad),
                ),
            )
        )
    if enabled_ids is not None:
        enabled_set = set(enabled_ids)
        variables = [replace(variable, enabled=variable.id in enabled_set) for variable in variables]
    return tuple(variables)


def _sample_combination(
    combination: VehicleCombination,
    joint_values_deg: dict[str, float],
) -> VehicleCombination:
    return VehicleCombination(
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
                articulation_rad=(
                    math.radians(joint_values_deg[joint.id])
                    if joint.id in joint_values_deg
                    else joint.articulation_rad
                ),
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


def _apply_values(
    problem: MechanismGraphOptimizationProblem,
    values: dict[str, float],
) -> tuple[tuple[MechanismDriverArc, ...], tuple[MechanismSteeringAssignment, ...]]:
    drivers = tuple(
        replace(
            driver,
            input_ratio=values.get(
                _driver_variable_id(driver.point_id, "input_ratio"),
                driver.input_ratio,
            ),
            phase_offset_rad=math.radians(
                values.get(
                    _driver_variable_id(driver.point_id, "phase_offset_deg"),
                    math.degrees(driver.phase_offset_rad),
                )
            ),
        )
        for driver in problem.drivers
    )
    assignments = tuple(
        replace(
            assignment,
            ratio=values.get(
                _assignment_variable_id(assignment.wheel_id, "ratio"),
                assignment.ratio,
            ),
            phase_offset_rad=math.radians(
                values.get(
                    _assignment_variable_id(assignment.wheel_id, "phase_offset_deg"),
                    math.degrees(assignment.phase_offset_rad),
                )
            ),
        )
        for assignment in problem.assignments
    )
    return drivers, assignments


def _failure_metrics(
    problem: MechanismGraphOptimizationProblem,
    violation: str,
    sample_index: int,
    *,
    solved_samples: int = 0,
    minimum_clearance_mm: float | None = None,
    errors_deg: Iterable[float] = (),
) -> OptimizationMetrics:
    penalty = 1.0e12 + problem.weights.failure * (1.0 + float(sample_index))
    errors = tuple(errors_deg)
    return OptimizationMetrics(
        score=penalty,
        rms_error_deg=float("inf"),
        mean_abs_error_deg=float("inf"),
        max_abs_error_deg=float("inf"),
        minimum_clearance_mm=minimum_clearance_mm,
        failure_index=sample_index,
        solved_samples=solved_samples,
        sample_count=len(problem.joint_sample_values()),
        feasible=False,
        violations=(violation,),
        max_abs_inner_error_deg=max((abs(value) for value in errors), default=0.0),
        max_abs_outer_error_deg=max((abs(value) for value in errors), default=0.0),
    )


def _evaluate_candidate(
    problem: MechanismGraphOptimizationProblem,
    values: dict[str, float],
) -> OptimizationMetrics:
    drivers, assignments = _apply_values(problem, values)
    errors_deg: list[float] = []
    synchronization_errors_deg: list[float] = []
    clearance_values: list[tuple[float, float]] = []
    violations: set[str] = set()
    penalty = 0.0
    failure_index: int | None = None
    previous_state: MechanismGraphState | None = None
    joint_samples = problem.joint_sample_values()
    sample_weights = problem.sample_weights or tuple(
        1.0 + 0.04 * abs(
            sample.get(
                problem.primary_joint_id,
                math.degrees(problem.combination.joints[0].articulation_rad),
            )
        )
        for sample in joint_samples
    )

    for sample_index, (joint_values_deg, sample_weight) in enumerate(
        zip(joint_samples, sample_weights, strict=True)
    ):
        beta_deg = joint_values_deg.get(
            problem.primary_joint_id,
            math.degrees(
                next(
                    joint.articulation_rad
                    for joint in problem.combination.joints
                    if joint.id == problem.primary_joint_id
                )
            ),
        )
        try:
            combination = _sample_combination(
                problem.combination,
                joint_values_deg,
            )
            kinematics = solve_combination_kinematics(
                combination,
                root_turn_radius_mm=problem.root_turn_radius_mm,
            )
            input_angles = {
                "articulation": math.radians(beta_deg),
                **{
                    joint.id: joint.articulation_rad
                    for joint in combination.joints
                },
            }
            driven_positions = resolve_driver_arc_positions(
                problem.graph,
                drivers,
                input_angles,
                body_poses=kinematics.body_poses,
            )
            graph_state = solve_mechanism_graph(
                problem.graph,
                driven_positions,
                previous_state=previous_state,
                body_poses=kinematics.body_poses,
            )
            actual = solve_actual_steering_from_graph(
                combination.to_vehicle_layout(),
                graph_state,
                assignments,
            )
            ideal = kinematics.ideal_steering
            sample_errors = tuple(actual_steering_errors_deg(actual, ideal).values())
            errors_deg.extend(sample_errors)
            penalty += problem.weights.steering_error * sample_weight * sum(
                error * error for error in sample_errors
            )
            comparison = compare_actual_to_ideal(
                actual,
                ideal,
                vehicle=combination.to_vehicle_layout(),
                beta_rad=math.radians(beta_deg),
            )
            channel_errors = comparison["synchronization_errors_deg"]
            sample_synchronization_errors: list[float] = []
            if channel_errors:
                sample_synchronization_errors.extend(
                    float(error) for error in channel_errors.values()
                )
            else:
                synchronization_error = comparison["front_rear_synchronization_error_deg"]
                if synchronization_error is not None:
                    sample_synchronization_errors.append(float(synchronization_error))
            synchronization_errors_deg.extend(sample_synchronization_errors)
            penalty += problem.weights.synchronization_error * sample_weight * sum(
                error * error for error in sample_synchronization_errors
            )
            clearance_report = analyze_clearance(
                build_mechanism_graph_clearance_items(
                    problem.graph,
                    graph_state,
                    vehicle=combination.to_vehicle_layout(),
                    combination=combination,
                    body_poses=kinematics.body_poses,
                )
            )
            clearance_mm = clearance_report.minimum_clearance_mm
            if clearance_mm is None:
                violations.add("CLEARANCE_NOT_EVALUATED")
                failure_index = sample_index if failure_index is None else failure_index
            else:
                clearance_values.append((clearance_mm, beta_deg))
                if clearance_mm < problem.clearance_target_mm:
                    violations.add("MIN_CLEARANCE_VIOLATED")
                    failure_index = sample_index if failure_index is None else failure_index
                    gap = problem.clearance_target_mm - clearance_mm
                    penalty += problem.weights.clearance * gap
                    penalty += problem.weights.clearance_violation * gap * gap
                if clearance_report.collision_detected:
                    violations.add("COLLISION_DETECTED")
                    failure_index = sample_index if failure_index is None else failure_index
                    penalty += problem.weights.failure * (1.0 + sample_index)
            previous_state = graph_state
        except (EngineeringError, KeyError, TypeError, ValueError) as error:
            code = getattr(error, "code", "MECHANISM_UNSOLVED")
            return _failure_metrics(
                problem,
                str(code),
                sample_index,
                solved_samples=sample_index,
                minimum_clearance_mm=(
                    min(clearance_values, default=(None, 0.0))[0]
                    if clearance_values
                    else None
                ),
                errors_deg=errors_deg,
            )

    if not errors_deg:
        return _failure_metrics(problem, "ACTUAL_STEERING_UNSOLVED", 0)
    baseline_abs = [abs(error) for error in errors_deg]
    minimum_clearance_case = min(clearance_values, key=lambda item: item[0], default=None)
    minimum_clearance_mm = (
        None if minimum_clearance_case is None else minimum_clearance_case[0]
    )
    minimum_clearance_beta_deg = (
        None if minimum_clearance_case is None else minimum_clearance_case[1]
    )
    for variable in problem.variables:
        if not variable.enabled or variable.preferred is None:
            continue
        span = max(variable.maximum - variable.minimum, 1e-9)
        delta = (values[variable.id] - variable.preferred) / span
        penalty += problem.weights.preferred * delta * delta
    penalty += problem.weights.complexity * sum(
        1.0 for variable in problem.variables if variable.enabled
    )
    return OptimizationMetrics(
        score=penalty,
        rms_error_deg=math.sqrt(sum(error * error for error in errors_deg) / len(errors_deg)),
        mean_abs_error_deg=sum(baseline_abs) / len(baseline_abs),
        max_abs_error_deg=max(baseline_abs),
        minimum_clearance_mm=minimum_clearance_mm,
        failure_index=failure_index,
        solved_samples=len(joint_samples),
        sample_count=len(joint_samples),
        feasible=not violations,
        violations=tuple(sorted(violations)),
        minimum_clearance_beta_deg=minimum_clearance_beta_deg,
        max_abs_inner_error_deg=max(baseline_abs),
        max_abs_outer_error_deg=max(baseline_abs),
        max_abs_synchronization_error_deg=max(
            (abs(error) for error in synchronization_errors_deg),
            default=0.0,
        ),
    )


def _metrics_better(candidate: OptimizationMetrics, incumbent: OptimizationMetrics) -> bool:
    if candidate.feasible != incumbent.feasible:
        return candidate.feasible
    return candidate.score + 1e-12 < incumbent.score


def optimize_mechanism_graph_problem(
    problem: MechanismGraphOptimizationProblem,
    *,
    require_feasible: bool = True,
) -> MechanismGraphOptimizationResult:
    rng = random.Random(problem.seed)
    baseline_values = {variable.id: variable.current for variable in problem.variables}
    baseline_metrics = _evaluate_candidate(problem, baseline_values)
    best_values = dict(baseline_values)
    best_metrics = baseline_metrics
    evaluations = 1
    iterations = 0
    enabled_variables = [variable for variable in problem.variables if variable.enabled]

    if enabled_variables:
        step_sizes = {
            variable.id: max(
                (variable.maximum - variable.minimum)
                * (0.22 if problem.mode == "quick" else 0.16),
                1e-6,
            )
            for variable in enabled_variables
        }
        random_samples = 10 if problem.mode == "quick" else 24
        for _ in range(random_samples):
            candidate = dict(best_values)
            for variable in enabled_variables:
                candidate[variable.id] = variable.clamp(
                    best_values[variable.id]
                    + step_sizes[variable.id] * (rng.random() * 2.0 - 1.0)
                )
            metrics = _evaluate_candidate(problem, candidate)
            evaluations += 1
            if _metrics_better(metrics, best_metrics):
                best_values, best_metrics = candidate, metrics

        max_iterations = 8 if problem.mode == "quick" else 16
        for iteration in range(max_iterations):
            iterations = iteration + 1
            improved = False
            for variable in enabled_variables:
                current = best_values[variable.id]
                step = step_sizes[variable.id]
                proposals = (
                    current - step,
                    current + step,
                    current - 0.5 * step,
                    current + 0.5 * step,
                    variable.minimum,
                    variable.maximum,
                )
                for proposal in proposals:
                    candidate = dict(best_values)
                    candidate[variable.id] = variable.clamp(proposal)
                    metrics = _evaluate_candidate(problem, candidate)
                    evaluations += 1
                    if _metrics_better(metrics, best_metrics):
                        best_values, best_metrics = candidate, metrics
                        improved = True
            if not improved:
                for variable in enabled_variables:
                    step_sizes[variable.id] *= 0.65
                if max(step_sizes.values()) < 0.01:
                    break
            else:
                for variable in enabled_variables:
                    step_sizes[variable.id] *= 0.88

    if require_feasible and not best_metrics.feasible:
        raise OptimizationNoFeasibleSolutionError(
            best_metrics.violations,
            minimum_clearance_mm=best_metrics.minimum_clearance_mm,
            clearance_target_mm=problem.clearance_target_mm,
        )

    optimized_variables = tuple(
        OptimizedVariable(
            id=variable.id,
            current=variable.current,
            minimum=variable.minimum,
            maximum=variable.maximum,
            enabled=variable.enabled,
            preferred=variable.preferred,
            optimized=best_values[variable.id],
        )
        for variable in problem.variables
    )
    baseline_variables = tuple(
        replace(variable, optimized=variable.current)
        for variable in optimized_variables
    )
    result = OptimizationResult(
        mode=problem.mode,
        baseline_variables=baseline_variables,
        optimized_variables=optimized_variables,
        baseline_metrics=baseline_metrics,
        optimized_metrics=best_metrics,
        iterations=iterations,
        evaluations=evaluations,
        clearance_target_mm=problem.clearance_target_mm,
        weights=problem.weights,
        design_cases=problem.design_cases,
    )
    baseline_drivers, baseline_assignments = _apply_values(problem, baseline_values)
    optimized_drivers, optimized_assignments = _apply_values(problem, best_values)
    return MechanismGraphOptimizationResult(
        result=result,
        baseline_drivers=baseline_drivers,
        optimized_drivers=optimized_drivers,
        baseline_assignments=baseline_assignments,
        optimized_assignments=optimized_assignments,
    )


def build_mechanism_graph_optimization_problem(
    *,
    combination: VehicleCombination,
    graph: PlanarMechanismGraph,
    drivers: tuple[MechanismDriverArc, ...],
    assignments: tuple[MechanismSteeringAssignment, ...],
    beta_min_deg: float = -45.0,
    beta_max_deg: float = 45.0,
    mode: OptimizationMode = "quick",
    primary_joint_id: str | None = None,
    root_turn_radius_mm: float | None = None,
    clearance_target_mm: float = 20.0,
    weights: OptimizationWeights | None = None,
    enabled_ids: Iterable[str] | None = None,
    design_cases: Iterable[DesignCase] | None = None,
    joint_ranges: object | None = None,
    maximum_samples: int = 10_000,
) -> MechanismGraphOptimizationProblem:
    if not math.isfinite(beta_min_deg) or not math.isfinite(beta_max_deg):
        raise ValueError("Graph optimization articulation bounds must be finite.")
    if beta_min_deg >= beta_max_deg or beta_min_deg > 0.0 or beta_max_deg < 0.0:
        raise ValueError("Graph optimization bounds must straddle zero.")
    if not combination.joints:
        raise ValueError("Graph optimization requires at least one articulation joint.")
    joint_id = primary_joint_id or combination.joints[0].id
    sample_step = 10.0 if mode == "quick" else 5.0
    if joint_ranges is None:
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
        joint_ranges = configured_ranges or None
    ranges = normalize_joint_sweep_ranges(
        (joint.id for joint in combination.joints),
        joint_ranges,
        default_min_deg=beta_min_deg,
        default_max_deg=beta_max_deg,
        default_step_deg=sample_step,
        primary_joint_id=joint_id,
    )
    samples = list(
        build_joint_sweep_grid(
            ranges,
            maximum_samples=maximum_samples,
        )
    )
    
    normalized_cases = tuple(design_cases or ())
    case_weights: dict[tuple[tuple[str, float], ...], float] = {}
    reference_length = combination.to_vehicle_layout().axle_span_mm() or 4360.0
    for case in normalized_cases:
        if case.enabled:
            beta_case = case.resolved_beta_deg(reference_length)
            case_sample = {joint_id: beta_case}
            case_key = tuple(sorted(case_sample.items()))
            case_weights[case_key] = case_weights.get(case_key, 0.0) + case.weight
            if case_sample not in samples:
                if len(samples) >= maximum_samples:
                    raise SweepSampleLimitError(len(samples) + 1, maximum_samples)
                samples.append(case_sample)
    samples.sort(key=lambda sample: tuple(sorted(sample.items())))
    sample_weights = tuple(
        case_weights.get(
            tuple(sorted(sample.items())),
            1.0 + 0.04 * abs(sample.get(joint_id, 0.0)),
        )
        for sample in samples
    )
    return MechanismGraphOptimizationProblem(
        combination=combination,
        graph=graph,
        drivers=drivers,
        assignments=assignments,
        beta_samples_deg=tuple(sample.get(joint_id, 0.0) for sample in samples),
        primary_joint_id=joint_id,
        root_turn_radius_mm=root_turn_radius_mm,
        variables=_default_variables(drivers, assignments, enabled_ids),
        clearance_target_mm=clearance_target_mm,
        weights=weights or OptimizationWeights(),
        mode=mode,
        design_cases=normalized_cases,
        sample_weights=sample_weights,
        joint_samples_deg=tuple(dict(sample) for sample in samples),
        joint_ranges=ranges,
    )


__all__ = [
    "MechanismGraphOptimizationProblem",
    "MechanismGraphOptimizationResult",
    "build_mechanism_graph_optimization_problem",
    "optimize_mechanism_graph_problem",
]
