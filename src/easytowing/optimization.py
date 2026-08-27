from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
from typing import Iterable, Literal

from .collision import CapsuleEnvelope, CircleEnvelope, CollisionItem, analyze_clearance
from .geometry import Point2D
from .linkage import (
    LinkageDemoRig,
    PlanarLinkageBranchHint,
    PlanarLinkageSpec,
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage_sweep,
)
from .steering import build_demo_solution

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


@dataclass(frozen=True, slots=True)
class LinkageOptimizationProblem:
    base_rig: LinkageDemoRig
    variables: tuple[OptimizationVariable, ...]
    beta_samples_deg: tuple[float, ...]
    clearance_target_mm: float = 20.0
    weights: OptimizationWeights = OptimizationWeights()
    mode: OptimizationMode = "quick"
    seed: int = 7

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

    @property
    def improvement(self) -> float:
        return self.baseline_metrics.score - self.optimized_metrics.score

    @property
    def improved(self) -> bool:
        return self.optimized_metrics.score < self.baseline_metrics.score


def _variables_to_map(variables: Iterable[OptimizationVariable | OptimizedVariable]) -> dict[str, float]:
    return {variable.id: variable.current if hasattr(variable, "current") else variable.optimized for variable in variables}  # type: ignore[attr-defined]


def _variable_lookup(variables: tuple[OptimizationVariable, ...]) -> dict[str, OptimizationVariable]:
    return {variable.id: variable for variable in variables}


def _build_spec_from_values(base_spec: PlanarLinkageSpec, values: dict[str, float]) -> PlanarLinkageSpec:
    updated = base_spec

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

    return updated


def _build_branch_hint(spec: PlanarLinkageSpec) -> PlanarLinkageBranchHint:
    return PlanarLinkageBranchHint(
        input_endpoint=Point2D(
            spec.bell_crank_pivot.x_mm + spec.bell_crank_input_arm_length_mm,
            spec.bell_crank_pivot.y_mm,
        ),
        steering_endpoint=Point2D(
            spec.steering_pivot.x_mm + spec.steering_arm_length_mm,
            spec.steering_pivot.y_mm,
        ),
    )


def _sample_weights(beta_samples_deg: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(1.0 + 0.04 * abs(beta_deg) for beta_deg in beta_samples_deg)


def build_branch_hint(spec: PlanarLinkageSpec) -> PlanarLinkageBranchHint:
    return _build_branch_hint(spec)


def _clearance_items_for_state(base_rig: LinkageDemoRig, state) -> tuple[CollisionItem, ...]:
    rear_axle_center = Point2D(-2180.0, 0.0)
    front_axle_center = Point2D(2180.0, 0.0)
    rear_half_track = 1250.0
    front_half_track = 1250.0

    return (
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
            id="bell_crank_pivot",
            envelope=CircleEnvelope(
                center=base_rig.spec.bell_crank_pivot,
                radius_mm=28.0,
            ),
        ),
        CollisionItem(
            id="steering_pivot",
            envelope=CircleEnvelope(
                center=base_rig.spec.steering_pivot,
                radius_mm=28.0,
            ),
        ),
        CollisionItem(
            id="front_axle_beam",
            envelope=CapsuleEnvelope(
                start=Point2D(front_axle_center.x_mm, -front_half_track),
                end=Point2D(front_axle_center.x_mm, front_half_track),
                radius_mm=70.0,
            ),
        ),
        CollisionItem(
            id="rear_axle_beam",
            envelope=CapsuleEnvelope(
                start=Point2D(rear_axle_center.x_mm, -rear_half_track),
                end=Point2D(rear_axle_center.x_mm, rear_half_track),
                radius_mm=70.0,
            ),
        ),
    )


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
        penalty = problem.weights.failure * (1.0 + float(failure_index or 0))
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
            ),
            (),
        )

    errors_deg: list[float] = []
    clearance_values: list[float] = []
    penalty = 0.0

    for beta_deg, state, sample_weight in zip(beta_samples_deg, sweep.states, _sample_weights(beta_samples_deg)):
        ideal_solution = build_demo_solution(beta_deg)[1]
        ideal_front_angle_deg = ideal_solution.axles[-1].center_heading_deg
        error_deg = state.steering_angle_deg - ideal_front_angle_deg
        errors_deg.append(error_deg)

        clearance_report = analyze_clearance(_clearance_items_for_state(problem.base_rig, state))
        clearance_mm = clearance_report.minimum_clearance_mm
        if clearance_mm is not None:
            clearance_values.append(clearance_mm)
            if clearance_mm < problem.clearance_target_mm:
                penalty += problem.weights.clearance_violation * (problem.clearance_target_mm - clearance_mm) ** 2

        penalty += problem.weights.steering_error * sample_weight * (error_deg ** 2)

    baseline_abs = [abs(error_deg) for error_deg in errors_deg]
    rms_error_deg = math.sqrt(sum(error_deg ** 2 for error_deg in errors_deg) / len(errors_deg))
    mean_abs_error_deg = sum(baseline_abs) / len(baseline_abs)
    max_abs_error_deg = max(baseline_abs)
    minimum_clearance_mm = min(clearance_values) if clearance_values else None

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
            failure_index=None,
            solved_samples=len(errors_deg),
            sample_count=len(beta_samples_deg),
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


def optimize_linkage_problem(problem: LinkageOptimizationProblem) -> OptimizationResult:
    rng = random.Random(problem.seed)
    baseline_values = _initial_values(problem)
    baseline_metrics, _ = _evaluate_candidate(problem, baseline_values)
    best_values = dict(baseline_values)
    best_metrics = baseline_metrics
    evaluations = 1
    iterations = 0

    enabled_variables = [variable for variable in problem.variables if variable.enabled]
    if not enabled_variables:
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
        if metrics.score < best_metrics.score:
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
                if metrics.score + 1e-12 < best_metrics.score:
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
    )


def build_reference_optimization_problem(mode: OptimizationMode = "quick") -> LinkageOptimizationProblem:
    rig = build_reference_linkage_demo()
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
    return LinkageOptimizationProblem(
        base_rig=rig,
        variables=variables,
        beta_samples_deg=beta_samples_deg,
        clearance_target_mm=20.0,
        weights=OptimizationWeights(),
        mode=mode,
        seed=7,
    )
