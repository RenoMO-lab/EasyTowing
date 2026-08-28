from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
from typing import Iterable, Literal

from .collision import CollisionItem, analyze_clearance
from .clearance_model import build_linkage_clearance_items
from .design_cases import DesignCase
from .errors import EngineeringError, OptimizationNoFeasibleSolutionError
from .actual_steering import actual_steering_errors_deg, compare_actual_to_ideal, solve_actual_steering
from .geometry import Point2D
from .linkage import (
    LinkageDemoRig,
    PlanarLinkageBranchHint,
    PlanarLinkageSpec,
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage_sweep,
)
from .model import VehicleLayout
from .steering import (
    beta_to_reference_radius_mm,
    build_demo_solution,
    solve_ideal_steering_from_radius,
)

OptimizationMode = Literal["quick", "full"]


@dataclass(frozen=True, slots=True)
class OptimizationVariable:
    id: str
    current: float
    minimum: float
    maximum: float
    enabled: bool = True
    preferred: float | None = None

    def clamp(self, value: float) -> float:
        return max(self.minimum, min(self.maximum, value))


@dataclass(frozen=True, slots=True)
class OptimizationWeights:
    steering_error: float = 1.0
    clearance: float = 12.0
    clearance_violation: float = 250.0
    failure: float = 100000.0
    preferred: float = 0.05
    complexity: float = 0.02
    synchronization_error: float = 0.5

    def __post_init__(self) -> None:
        values = {
            "steering_error": self.steering_error,
            "clearance": self.clearance,
            "clearance_violation": self.clearance_violation,
            "failure": self.failure,
            "preferred": self.preferred,
            "complexity": self.complexity,
            "synchronization_error": self.synchronization_error,
        }
        if any(not math.isfinite(value) or value < 0.0 for value in values.values()):
            raise ValueError("optimization weights must be non-negative finite values")

    def to_dict(self) -> dict[str, float]:
        return {
            "steering_error": self.steering_error,
            "clearance": self.clearance,
            "clearance_violation": self.clearance_violation,
            "failure": self.failure,
            "preferred": self.preferred,
            "complexity": self.complexity,
            "synchronization_error": self.synchronization_error,
        }


@dataclass(frozen=True, slots=True)
class LinkageOptimizationProblem:
    base_rig: LinkageDemoRig
    variables: tuple[OptimizationVariable, ...]
    beta_samples_deg: tuple[float, ...]
    clearance_target_mm: float = 20.0
    weights: OptimizationWeights = OptimizationWeights()
    mode: OptimizationMode = "quick"
    seed: int = 7
    design_cases: tuple[DesignCase, ...] = ()
    sample_weights: tuple[float, ...] = ()
    vehicle: VehicleLayout | None = None

    @property
    def baseline_spec(self) -> PlanarLinkageSpec:
        return self.base_rig.spec


@dataclass(frozen=True, slots=True)
class OptimizationMetrics:
    score: float
    rms_error_deg: float
    mean_abs_error_deg: float
    max_abs_error_deg: float
    minimum_clearance_mm: float | None
    failure_index: int | None
    solved_samples: int
    sample_count: int
    feasible: bool = True
    violations: tuple[str, ...] = ()
    minimum_clearance_beta_deg: float | None = None
    max_abs_inner_error_deg: float | None = None
    max_abs_outer_error_deg: float | None = None
    max_abs_synchronization_error_deg: float | None = None


@dataclass(frozen=True, slots=True)
class OptimizedVariable:
    id: str
    current: float
    minimum: float
    maximum: float
    enabled: bool
    preferred: float | None
    optimized: float

    @property
    def delta(self) -> float:
        return self.optimized - self.current


@dataclass(frozen=True, slots=True)
class OptimizationResult:
    mode: OptimizationMode
    baseline_variables: tuple[OptimizedVariable, ...]
    optimized_variables: tuple[OptimizedVariable, ...]
    baseline_metrics: OptimizationMetrics
    optimized_metrics: OptimizationMetrics
    iterations: int
    evaluations: int
    clearance_target_mm: float = 20.0
    weights: OptimizationWeights = OptimizationWeights()
    design_cases: tuple[DesignCase, ...] = ()

    @property
    def improvement(self) -> float:
        return self.baseline_metrics.score - self.optimized_metrics.score

    @property
    def improved(self) -> bool:
        if self.optimized_metrics.feasible != self.baseline_metrics.feasible:
            return self.optimized_metrics.feasible
        return self.optimized_metrics.score < self.baseline_metrics.score


def _metrics_better(candidate: OptimizationMetrics, incumbent: OptimizationMetrics) -> bool:
    if candidate.feasible != incumbent.feasible:
        return candidate.feasible
    return candidate.score + 1e-12 < incumbent.score


def _variables_to_map(variables: Iterable[OptimizationVariable | OptimizedVariable]) -> dict[str, float]:
    return {variable.id: variable.current if hasattr(variable, "current") else variable.optimized for variable in variables}  # type: ignore[attr-defined]


def _variable_lookup(variables: tuple[OptimizationVariable, ...]) -> dict[str, OptimizationVariable]:
    return {variable.id: variable for variable in variables}


def _build_spec_from_values(base_spec: PlanarLinkageSpec, values: dict[str, float]) -> PlanarLinkageSpec:
    updated = base_spec

    def angle_value(rad_key: str, deg_key: str, current: float) -> float:
        if rad_key in values:
            return values[rad_key]
        if deg_key in values:
            return math.radians(values[deg_key])
        return current

    if "bell_crank_pivot_x_mm" in values or "bell_crank_pivot_y_mm" in values:
        pivot_x = values.get("bell_crank_pivot_x_mm", updated.bell_crank_pivot.x_mm)
        pivot_y = values.get("bell_crank_pivot_y_mm", updated.bell_crank_pivot.y_mm)
        updated = replace(updated, bell_crank_pivot=Point2D(pivot_x, pivot_y))

    if "steering_pivot_x_mm" in values or "steering_pivot_y_mm" in values:
        pivot_x = values.get("steering_pivot_x_mm", updated.steering_pivot.x_mm)
        pivot_y = values.get("steering_pivot_y_mm", updated.steering_pivot.y_mm)
        updated = replace(updated, steering_pivot=Point2D(pivot_x, pivot_y))

    if "bell_crank_input_arm_length_mm" in values:
        updated = replace(updated, bell_crank_input_arm_length_mm=values["bell_crank_input_arm_length_mm"])
    if "bell_crank_output_arm_length_mm" in values:
        updated = replace(updated, bell_crank_output_arm_length_mm=values["bell_crank_output_arm_length_mm"])
    if "steering_arm_length_mm" in values:
        updated = replace(updated, steering_arm_length_mm=values["steering_arm_length_mm"])
    if "input_rod_length_mm" in values:
        updated = replace(updated, input_rod_length_mm=values["input_rod_length_mm"])
    if "tie_rod_length_mm" in values:
        updated = replace(updated, tie_rod_length_mm=values["tie_rod_length_mm"])
    if "companion_steering_arm_length_mm" in values:
        updated = replace(updated, companion_steering_arm_length_mm=values["companion_steering_arm_length_mm"])
    if "companion_tie_rod_length_mm" in values:
        updated = replace(updated, companion_tie_rod_length_mm=values["companion_tie_rod_length_mm"])
    if "companion_steering_pivot_x_mm" in values or "companion_steering_pivot_y_mm" in values:
        if updated.companion_steering_pivot is not None:
            pivot_x = values.get("companion_steering_pivot_x_mm", updated.companion_steering_pivot.x_mm)
            pivot_y = values.get("companion_steering_pivot_y_mm", updated.companion_steering_pivot.y_mm)
            updated = replace(updated, companion_steering_pivot=Point2D(pivot_x, pivot_y))

    updated = replace(
        updated,
        steering_arm_neutral_angle_rad=angle_value(
            "steering_arm_neutral_angle_rad",
            "steering_arm_neutral_angle_deg",
            updated.steering_arm_neutral_angle_rad,
        ),
        bell_crank_input_neutral_angle_rad=angle_value(
            "bell_crank_input_neutral_angle_rad",
            "bell_crank_input_neutral_angle_deg",
            updated.bell_crank_input_neutral_angle_rad,
        ),
        bell_crank_output_neutral_angle_rad=angle_value(
            "bell_crank_output_neutral_angle_rad",
            "bell_crank_output_neutral_angle_deg",
            updated.bell_crank_output_neutral_angle_rad,
        ),
        companion_steering_arm_neutral_angle_rad=angle_value(
            "companion_steering_arm_neutral_angle_rad",
            "companion_steering_arm_neutral_angle_deg",
            updated.companion_steering_arm_neutral_angle_rad,
        ),
    )

    return updated


def _build_branch_hint(spec: PlanarLinkageSpec) -> PlanarLinkageBranchHint:
    companion_endpoint = None
    if spec.companion_steering_pivot is not None and spec.companion_steering_arm_length_mm is not None:
        companion_endpoint = spec.companion_steering_pivot + Point2D(
            math.cos(spec.companion_steering_arm_neutral_angle_rad)
            * spec.companion_steering_arm_length_mm,
            math.sin(spec.companion_steering_arm_neutral_angle_rad)
            * spec.companion_steering_arm_length_mm,
        )
    return PlanarLinkageBranchHint(
        input_endpoint=spec.bell_crank_pivot
        + Point2D(
            math.cos(spec.bell_crank_input_neutral_angle_rad)
            * spec.bell_crank_input_arm_length_mm,
            math.sin(spec.bell_crank_input_neutral_angle_rad)
            * spec.bell_crank_input_arm_length_mm,
        ),
        steering_endpoint=spec.steering_pivot
        + Point2D(
            math.cos(spec.steering_arm_neutral_angle_rad) * spec.steering_arm_length_mm,
            math.sin(spec.steering_arm_neutral_angle_rad) * spec.steering_arm_length_mm,
        ),
        companion_steering_endpoint=companion_endpoint,
    )


def _sample_weights(beta_samples_deg: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(1.0 + 0.04 * abs(beta_deg) for beta_deg in beta_samples_deg)


def build_branch_hint(spec: PlanarLinkageSpec) -> PlanarLinkageBranchHint:
    return _build_branch_hint(spec)


def _clearance_items_for_state(
    base_rig: LinkageDemoRig,
    spec: PlanarLinkageSpec,
    state,
    vehicle: VehicleLayout | None = None,
) -> tuple[CollisionItem, ...]:
    return build_linkage_clearance_items(spec, state, vehicle=vehicle)


def _evaluate_candidate(
    problem: LinkageOptimizationProblem,
    values: dict[str, float],
) -> tuple[OptimizationMetrics, tuple[float, ...]]:
    spec = _build_spec_from_values(problem.baseline_spec, values)
    branch_hint = _build_branch_hint(spec)
    beta_samples_deg = problem.beta_samples_deg
    beta_samples_rad = tuple(math.radians(beta_deg) for beta_deg in beta_samples_deg)
    driver_points = tuple(
        driver_point_arc(problem.base_rig.driver_arc_center, problem.base_rig.driver_arc_radius_mm, beta_rad)
        for beta_rad in beta_samples_rad
    )
    sweep = solve_planar_linkage_sweep(
        spec,
        driver_points,
        branch_hint=branch_hint,
    )

    if not sweep.succeeded:
        failure_index = sweep.failure_index
        # Mechanism solvability is a hard constraint, not a tradeoff against
        # clearance or tracking error. Keep failed candidates above any
        # finite feasible design even when the configured failure weight is 0.
        penalty = 1.0e12 + problem.weights.failure * (1.0 + float(failure_index or 0))
        return (
            OptimizationMetrics(
                score=penalty,
                rms_error_deg=float("inf"),
                mean_abs_error_deg=float("inf"),
                max_abs_error_deg=float("inf"),
                minimum_clearance_mm=None,
                failure_index=failure_index,
                solved_samples=len(sweep.states),
                sample_count=len(beta_samples_deg),
                feasible=False,
                violations=("MECHANISM_UNSOLVED",),
            ),
            (),
        )

    errors_deg: list[float] = []
    inner_errors_deg: list[float] = []
    outer_errors_deg: list[float] = []
    synchronization_errors_deg: list[float] = []
    clearance_values: list[tuple[float, float]] = []
    penalty = 0.0
    constraint_failure_index: int | None = None
    violations: set[str] = set()

    sample_weights = problem.sample_weights or _sample_weights(beta_samples_deg)
    if len(sample_weights) != len(beta_samples_deg):
        raise ValueError("sample_weights must match beta_samples_deg")

    for sample_index, (beta_deg, state, sample_weight) in enumerate(
        zip(beta_samples_deg, sweep.states, sample_weights, strict=True)
    ):
        if problem.vehicle is None:
            ideal_vehicle, ideal_solution, _ = build_demo_solution(beta_deg)
        else:
            ideal_vehicle = problem.vehicle
            reference_length = ideal_vehicle.axle_span_mm() or 4360.0
            radius = beta_to_reference_radius_mm(math.radians(beta_deg), reference_length)
            try:
                ideal_solution = solve_ideal_steering_from_radius(ideal_vehicle, radius)
            except EngineeringError:
                penalty = 1.0e12 + problem.weights.failure * (1.0 + float(sample_index))
                return (
                    OptimizationMetrics(
                        score=penalty,
                        rms_error_deg=float("inf"),
                        mean_abs_error_deg=float("inf"),
                        max_abs_error_deg=float("inf"),
                        minimum_clearance_mm=None,
                        failure_index=sample_index,
                        solved_samples=sample_index,
                        sample_count=len(beta_samples_deg),
                        feasible=False,
                        violations=("IDEAL_STEERING_UNSOLVED",),
                    ),
                    tuple(errors_deg),
                )
        try:
            actual_solution = solve_actual_steering(
                ideal_vehicle,
                state,
                math.radians(beta_deg),
                ideal_solution=ideal_solution,
            )
        except EngineeringError:
            penalty = 1.0e12 + problem.weights.failure * (1.0 + float(sample_index))
            return (
                OptimizationMetrics(
                    score=penalty,
                    rms_error_deg=float("inf"),
                    mean_abs_error_deg=float("inf"),
                    max_abs_error_deg=float("inf"),
                    minimum_clearance_mm=None,
                    failure_index=sample_index,
                    solved_samples=sample_index,
                    sample_count=len(beta_samples_deg),
                    feasible=False,
                    violations=("ACTUAL_STEERING_UNSOLVED",),
                ),
                tuple(errors_deg),
            )
        sample_error_map = actual_steering_errors_deg(actual_solution, ideal_solution)
        sample_errors = tuple(sample_error_map.values())
        errors_deg.extend(sample_errors)
        for axle in actual_solution.axles:
            for wheel in axle.wheel_solutions:
                if wheel.wheel_id not in sample_error_map:
                    continue
                if wheel.side == "left":
                    inner_errors_deg.append(sample_error_map[wheel.wheel_id])
                else:
                    outer_errors_deg.append(sample_error_map[wheel.wheel_id])
        comparison = compare_actual_to_ideal(
            actual_solution,
            ideal_solution,
            vehicle=ideal_vehicle,
            beta_rad=math.radians(beta_deg),
        )
        channel_errors = comparison["synchronization_errors_deg"]
        if channel_errors:
            synchronization_errors_deg.extend(float(error) for error in channel_errors.values())
        else:
            synchronization_error = comparison["front_rear_synchronization_error_deg"]
            if synchronization_error is not None:
                synchronization_errors_deg.append(float(synchronization_error))

        clearance_report = analyze_clearance(
            _clearance_items_for_state(problem.base_rig, spec, state, vehicle=ideal_vehicle)
        )
        clearance_mm = clearance_report.minimum_clearance_mm
        if clearance_mm is None:
            violations.add("CLEARANCE_NOT_EVALUATED")
            if constraint_failure_index is None:
                constraint_failure_index = sample_index
        else:
            clearance_values.append((clearance_mm, beta_deg))
            if clearance_mm < problem.clearance_target_mm:
                violations.add("MIN_CLEARANCE_VIOLATED")
                if constraint_failure_index is None:
                    constraint_failure_index = sample_index
                clearance_gap = problem.clearance_target_mm - clearance_mm
                penalty += problem.weights.clearance * clearance_gap
                penalty += problem.weights.clearance_violation * clearance_gap ** 2
            if clearance_report.collision_detected:
                violations.add("COLLISION_DETECTED")
                if constraint_failure_index is None:
                    constraint_failure_index = sample_index
                penalty += problem.weights.failure * (1.0 + sample_index)

        penalty += problem.weights.steering_error * sample_weight * sum(error ** 2 for error in sample_errors)
        for synchronization_error in channel_errors.values():
            penalty += (
                problem.weights.synchronization_error
                * sample_weight
                * float(synchronization_error) ** 2
            )

    baseline_abs = [abs(error_deg) for error_deg in errors_deg]
    rms_error_deg = math.sqrt(sum(error_deg ** 2 for error_deg in errors_deg) / len(errors_deg))
    mean_abs_error_deg = sum(baseline_abs) / len(baseline_abs)
    max_abs_error_deg = max(baseline_abs)
    minimum_clearance_case = min(clearance_values, key=lambda item: item[0]) if clearance_values else None
    minimum_clearance_mm = None if minimum_clearance_case is None else minimum_clearance_case[0]
    minimum_clearance_beta_deg = None if minimum_clearance_case is None else minimum_clearance_case[1]

    preferred_penalty = 0.0
    for variable in problem.variables:
        if not variable.enabled or variable.preferred is None:
            continue
        value = values[variable.id]
        span = max(variable.maximum - variable.minimum, 1e-9)
        normalized_delta = (value - variable.preferred) / span
        preferred_penalty += problem.weights.preferred * normalized_delta * normalized_delta

    complexity_penalty = problem.weights.complexity * sum(1.0 for variable in problem.variables if variable.enabled)
    total_score = penalty + preferred_penalty + complexity_penalty

    return (
        OptimizationMetrics(
            score=total_score,
            rms_error_deg=rms_error_deg,
            mean_abs_error_deg=mean_abs_error_deg,
            max_abs_error_deg=max_abs_error_deg,
            minimum_clearance_mm=minimum_clearance_mm,
            failure_index=constraint_failure_index,
            solved_samples=len(beta_samples_deg),
            sample_count=len(beta_samples_deg),
            feasible=not violations,
            violations=tuple(sorted(violations)),
            minimum_clearance_beta_deg=minimum_clearance_beta_deg,
            max_abs_inner_error_deg=max(abs(error) for error in inner_errors_deg),
            max_abs_outer_error_deg=max(abs(error) for error in outer_errors_deg),
            max_abs_synchronization_error_deg=max(
                (abs(error) for error in synchronization_errors_deg),
                default=0.0,
            ),
        ),
        errors_deg,
    )


def _initial_values(problem: LinkageOptimizationProblem) -> dict[str, float]:
    return {variable.id: variable.current for variable in problem.variables}


def _candidate_from_values(problem: LinkageOptimizationProblem, values: dict[str, float]) -> dict[str, float]:
    candidate = {}
    for variable in problem.variables:
        candidate[variable.id] = variable.clamp(values.get(variable.id, variable.current))
    return candidate


def build_spec_from_values(base_spec: PlanarLinkageSpec, values: dict[str, float]) -> PlanarLinkageSpec:
    return _build_spec_from_values(base_spec, values)


def build_optimized_spec(
    base_spec: PlanarLinkageSpec,
    optimized_variables: Iterable[OptimizedVariable],
) -> PlanarLinkageSpec:
    values = {variable.id: variable.optimized for variable in optimized_variables}
    return _build_spec_from_values(base_spec, values)


def _sample_random_candidate(
    rng: random.Random,
    problem: LinkageOptimizationProblem,
    center: dict[str, float],
    scale: dict[str, float],
) -> dict[str, float]:
    candidate = dict(center)
    for variable in problem.variables:
        if not variable.enabled:
            continue
        span = variable.maximum - variable.minimum
        jitter = scale[variable.id] * (rng.random() * 2.0 - 1.0)
        if rng.random() < 0.35:
            candidate_value = variable.preferred if variable.preferred is not None else center[variable.id] + jitter
        else:
            candidate_value = center[variable.id] + jitter
        if rng.random() < 0.12:
            candidate_value = variable.minimum + rng.random() * span
        candidate[variable.id] = variable.clamp(candidate_value)
    return candidate


def optimize_linkage_problem(
    problem: LinkageOptimizationProblem,
    *,
    require_feasible: bool = True,
) -> OptimizationResult:
    rng = random.Random(problem.seed)
    baseline_values = _initial_values(problem)
    baseline_metrics, _ = _evaluate_candidate(problem, baseline_values)
    best_values = dict(baseline_values)
    best_metrics = baseline_metrics
    evaluations = 1
    iterations = 0

    enabled_variables = [variable for variable in problem.variables if variable.enabled]
    if not enabled_variables:
        if require_feasible and not baseline_metrics.feasible:
            raise OptimizationNoFeasibleSolutionError(
                baseline_metrics.violations,
                minimum_clearance_mm=baseline_metrics.minimum_clearance_mm,
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
                optimized=variable.current,
            )
            for variable in problem.variables
        )
        return OptimizationResult(
            mode=problem.mode,
            baseline_variables=optimized_variables,
            optimized_variables=optimized_variables,
            baseline_metrics=baseline_metrics,
            optimized_metrics=baseline_metrics,
            iterations=0,
            evaluations=evaluations,
            clearance_target_mm=problem.clearance_target_mm,
            weights=problem.weights,
            design_cases=problem.design_cases,
        )

    step_sizes = {
        variable.id: max((variable.maximum - variable.minimum) * (0.22 if problem.mode == "quick" else 0.16), 1e-6)
        for variable in enabled_variables
    }

    random_samples = 18 if problem.mode == "quick" else 40
    for _ in range(random_samples):
        candidate_values = _sample_random_candidate(rng, problem, best_values, step_sizes)
        metrics, _ = _evaluate_candidate(problem, candidate_values)
        evaluations += 1
        if _metrics_better(metrics, best_metrics):
            best_values = candidate_values
            best_metrics = metrics

    max_iterations = 10 if problem.mode == "quick" else 20
    for iteration in range(max_iterations):
        iterations = iteration + 1
        improved = False

        for variable in enabled_variables:
            current_value = best_values[variable.id]
            step = step_sizes[variable.id]
            proposal_values = [
                current_value,
                current_value - step,
                current_value + step,
                current_value - 0.5 * step,
                current_value + 0.5 * step,
                variable.minimum,
                variable.maximum,
            ]
            if variable.preferred is not None:
                proposal_values.append(variable.preferred)

            for proposal_value in proposal_values:
                candidate_values = dict(best_values)
                candidate_values[variable.id] = variable.clamp(proposal_value)
                metrics, _ = _evaluate_candidate(problem, candidate_values)
                evaluations += 1
                if _metrics_better(metrics, best_metrics):
                    best_values = candidate_values
                    best_metrics = metrics
                    improved = True

        if not improved:
            for variable in enabled_variables:
                step_sizes[variable.id] *= 0.65
            if max(step_sizes.values()) < 0.05:
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
        OptimizedVariable(
            id=variable.id,
            current=variable.current,
            minimum=variable.minimum,
            maximum=variable.maximum,
            enabled=variable.enabled,
            preferred=variable.preferred,
            optimized=variable.current,
        )
        for variable in problem.variables
    )

    return OptimizationResult(
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


def _custom_linkage_variables(rig: LinkageDemoRig) -> tuple[OptimizationVariable, ...]:
    """Expose editable dimensions for a non-reference linkage rig."""

    spec = rig.spec

    def length_variable(identifier: str, current: float) -> OptimizationVariable:
        span = max(abs(current) * 0.5, 50.0)
        return OptimizationVariable(
            id=identifier,
            current=current,
            minimum=max(1.0, current - span),
            maximum=current + span,
            enabled=True,
            preferred=current,
        )

    def position_variable(identifier: str, current: float) -> OptimizationVariable:
        span = max(abs(current) * 0.5, 300.0)
        return OptimizationVariable(
            id=identifier,
            current=current,
            minimum=current - span,
            maximum=current + span,
            enabled=False,
            preferred=current,
        )

    def angle_variable(identifier: str, current_rad: float) -> OptimizationVariable:
        current_deg = math.degrees(current_rad)
        return OptimizationVariable(
            id=identifier,
            current=current_deg,
            minimum=current_deg - 45.0,
            maximum=current_deg + 45.0,
            enabled=False,
            preferred=current_deg,
        )

    variables = [
        length_variable("bell_crank_input_arm_length_mm", spec.bell_crank_input_arm_length_mm),
        length_variable("bell_crank_output_arm_length_mm", spec.bell_crank_output_arm_length_mm),
        length_variable("steering_arm_length_mm", spec.steering_arm_length_mm),
        length_variable("input_rod_length_mm", spec.input_rod_length_mm),
        length_variable("tie_rod_length_mm", spec.tie_rod_length_mm),
        position_variable("bell_crank_pivot_x_mm", spec.bell_crank_pivot.x_mm),
        position_variable("bell_crank_pivot_y_mm", spec.bell_crank_pivot.y_mm),
        position_variable("steering_pivot_x_mm", spec.steering_pivot.x_mm),
        position_variable("steering_pivot_y_mm", spec.steering_pivot.y_mm),
        angle_variable("steering_arm_neutral_angle_deg", spec.steering_arm_neutral_angle_rad),
        angle_variable("bell_crank_input_neutral_angle_deg", spec.bell_crank_input_neutral_angle_rad),
        angle_variable("bell_crank_output_neutral_angle_deg", spec.bell_crank_output_neutral_angle_rad),
    ]
    if spec.companion_steering_pivot is not None:
        variables.extend(
            [
                length_variable(
                    "companion_steering_arm_length_mm",
                    spec.companion_steering_arm_length_mm or spec.steering_arm_length_mm,
                ),
                length_variable(
                    "companion_tie_rod_length_mm",
                    spec.companion_tie_rod_length_mm or spec.tie_rod_length_mm,
                ),
                position_variable("companion_steering_pivot_x_mm", spec.companion_steering_pivot.x_mm),
                position_variable("companion_steering_pivot_y_mm", spec.companion_steering_pivot.y_mm),
                angle_variable(
                    "companion_steering_arm_neutral_angle_deg",
                    spec.companion_steering_arm_neutral_angle_rad,
                ),
            ]
        )
    return tuple(variables)


def build_reference_optimization_problem(
    mode: OptimizationMode = "quick",
    enabled_ids: Iterable[str] | None = None,
    clearance_target_mm: float = 20.0,
    weights: OptimizationWeights | None = None,
    design_cases: Iterable[DesignCase] | None = None,
    base_rig: LinkageDemoRig | None = None,
    vehicle: VehicleLayout | None = None,
) -> LinkageOptimizationProblem:
    if not math.isfinite(clearance_target_mm) or clearance_target_mm < 0.0:
        raise ValueError("clearance_target_mm must be a non-negative finite value")
    rig = base_rig or build_reference_linkage_demo()
    variables = (
        OptimizationVariable(
            id="bell_crank_input_arm_length_mm",
            current=rig.spec.bell_crank_input_arm_length_mm,
            minimum=150.0,
            maximum=280.0,
            enabled=True,
            preferred=rig.spec.bell_crank_input_arm_length_mm,
        ),
        OptimizationVariable(
            id="bell_crank_output_arm_length_mm",
            current=rig.spec.bell_crank_output_arm_length_mm,
            minimum=120.0,
            maximum=260.0,
            enabled=True,
            preferred=rig.spec.bell_crank_output_arm_length_mm,
        ),
        OptimizationVariable(
            id="steering_arm_length_mm",
            current=rig.spec.steering_arm_length_mm,
            minimum=140.0,
            maximum=260.0,
            enabled=True,
            preferred=rig.spec.steering_arm_length_mm,
        ),
        OptimizationVariable(
            id="input_rod_length_mm",
            current=rig.spec.input_rod_length_mm,
            minimum=80.0,
            maximum=180.0,
            enabled=True,
            preferred=rig.spec.input_rod_length_mm,
        ),
        OptimizationVariable(
            id="tie_rod_length_mm",
            current=rig.spec.tie_rod_length_mm,
            minimum=420.0,
            maximum=720.0,
            enabled=True,
            preferred=rig.spec.tie_rod_length_mm,
        ),
        OptimizationVariable(
            id="bell_crank_pivot_x_mm",
            current=rig.spec.bell_crank_pivot.x_mm,
            minimum=-120.0,
            maximum=120.0,
            enabled=False,
            preferred=rig.spec.bell_crank_pivot.x_mm,
        ),
        OptimizationVariable(
            id="bell_crank_pivot_y_mm",
            current=rig.spec.bell_crank_pivot.y_mm,
            minimum=-120.0,
            maximum=120.0,
            enabled=False,
            preferred=rig.spec.bell_crank_pivot.y_mm,
        ),
        OptimizationVariable(
            id="steering_pivot_x_mm",
            current=rig.spec.steering_pivot.x_mm,
            minimum=360.0,
            maximum=760.0,
            enabled=False,
            preferred=rig.spec.steering_pivot.x_mm,
        ),
        OptimizationVariable(
            id="steering_pivot_y_mm",
            current=rig.spec.steering_pivot.y_mm,
            minimum=80.0,
            maximum=320.0,
            enabled=False,
            preferred=rig.spec.steering_pivot.y_mm,
        ),
        OptimizationVariable(
            id="steering_arm_neutral_angle_deg",
            current=math.degrees(rig.spec.steering_arm_neutral_angle_rad),
            minimum=-180.0,
            maximum=180.0,
            enabled=False,
            preferred=math.degrees(rig.spec.steering_arm_neutral_angle_rad),
        ),
        OptimizationVariable(
            id="bell_crank_input_neutral_angle_deg",
            current=math.degrees(rig.spec.bell_crank_input_neutral_angle_rad),
            minimum=-180.0,
            maximum=180.0,
            enabled=False,
            preferred=math.degrees(rig.spec.bell_crank_input_neutral_angle_rad),
        ),
        OptimizationVariable(
            id="bell_crank_output_neutral_angle_deg",
            current=math.degrees(rig.spec.bell_crank_output_neutral_angle_rad),
            minimum=-180.0,
            maximum=180.0,
            enabled=False,
            preferred=math.degrees(rig.spec.bell_crank_output_neutral_angle_rad),
        ),
    )
    if base_rig is not None:
        variables = _custom_linkage_variables(rig)
    if enabled_ids is not None:
        enabled_set = set(enabled_ids)
        variables = tuple(
            replace(variable, enabled=variable.id in enabled_set)
            for variable in variables
        )

    beta_samples_deg = (-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0) if mode == "quick" else (
        -45.0,
        -40.0,
        -35.0,
        -30.0,
        -25.0,
        -20.0,
        -15.0,
        -10.0,
        -5.0,
        0.0,
        5.0,
        10.0,
        15.0,
        20.0,
        25.0,
        30.0,
        35.0,
        40.0,
        45.0,
    )
    normalized_cases = tuple(design_cases or ())
    case_weight_by_beta: dict[float, float] = {}
    for case in normalized_cases:
        if not case.enabled:
            continue
        reference_length = (vehicle.axle_span_mm() if vehicle is not None else 0.0) or 4360.0
        case_beta = case.resolved_beta_deg(reference_length_mm=reference_length)
        case_weight_by_beta[case_beta] = case_weight_by_beta.get(case_beta, 0.0) + case.weight

    if case_weight_by_beta:
        beta_sample_list = list(beta_samples_deg)
        for case_beta in case_weight_by_beta:
            if not any(abs(existing - case_beta) <= 1e-9 for existing in beta_sample_list):
                beta_sample_list.append(case_beta)
        beta_sample_list.sort()
        beta_samples_deg = tuple(beta_sample_list)
        sample_weights = tuple(
            case_weight_by_beta.get(beta_deg, 1.0 + 0.04 * abs(beta_deg))
            for beta_deg in beta_samples_deg
        )
    else:
        sample_weights = ()
    return LinkageOptimizationProblem(
        base_rig=rig,
        variables=variables,
        beta_samples_deg=beta_samples_deg,
        clearance_target_mm=clearance_target_mm,
        weights=weights or OptimizationWeights(),
        mode=mode,
        seed=7,
        design_cases=normalized_cases,
        sample_weights=sample_weights,
        vehicle=vehicle,
    )
