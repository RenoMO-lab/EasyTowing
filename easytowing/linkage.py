from __future__ import annotations

from dataclasses import dataclass
import math
from functools import lru_cache
from typing import Iterable

from .errors import (
    LinkageBranchChangeError,
    LinkageNoSolutionError,
    SteeringLimitExceededError,
)
from .geometry import EPSILON_MM, Point2D, heading_vector, normalize_angle
from .tolerances import DEFAULT_TOLERANCES


@dataclass(frozen=True, slots=True)
class SteeringPivot:
    """Fixed or body-mounted steering-axis reference."""

    id: str
    position: Point2D
    steering_axis_rad: float = 0.0
    steering_arm_length_mm: float = 0.0
    neutral_steering_arm_angle_rad: float = 0.0
    steering_stop_deg: float | None = None


@dataclass(frozen=True, slots=True)
class SteeringArm:
    """Rigid arm rotating about a named steering pivot."""

    id: str
    pivot_id: str
    length_mm: float
    neutral_angle_rad: float = 0.0
    attachment_radius_mm: float | None = None


@dataclass(frozen=True, slots=True)
class TieRod:
    """Rigid planar rod with an explicit design length and envelope radius."""

    id: str
    point_a_id: str
    point_b_id: str
    fixed_length_mm: float
    radius_mm: float = 0.0


@dataclass(frozen=True, slots=True)
class BellCrank:
    """Two-arm rigid lever about a fixed or body-mounted pivot."""

    id: str
    pivot: Point2D
    input_arm_length_mm: float
    output_arm_length_mm: float
    input_neutral_angle_rad: float = 0.0
    output_neutral_angle_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class Drawbar:
    """Articulation input geometry kept separate from wheel steering angles."""

    id: str
    pivot: Point2D
    effective_length_mm: float
    connection_point: Point2D
    maximum_articulation_deg: float = 45.0


@dataclass(frozen=True, slots=True)
class LongitudinalSteeringRod:
    """Rigid rod connecting steering systems on separate axles or bodies."""

    id: str
    point_a_id: str
    point_b_id: str
    fixed_length_mm: float
    radius_mm: float = 0.0


@dataclass(frozen=True, slots=True)
class PlanarMechanism:
    """Stable-ID component collection for a generalized planar linkage."""

    id: str
    pivots: tuple[SteeringPivot, ...] = ()
    steering_arms: tuple[SteeringArm, ...] = ()
    tie_rods: tuple[TieRod, ...] = ()
    bell_cranks: tuple[BellCrank, ...] = ()
    drawbars: tuple[Drawbar, ...] = ()
    longitudinal_rods: tuple[LongitudinalSteeringRod, ...] = ()

    def validate_unique_ids(self) -> None:
        components = (
            *self.pivots,
            *self.steering_arms,
            *self.tie_rods,
            *self.bell_cranks,
            *self.drawbars,
            *self.longitudinal_rods,
        )
        ids = [component.id for component in components]
        if len(ids) != len(set(ids)):
            raise ValueError("PlanarMechanism component IDs must be unique.")


@dataclass(frozen=True, slots=True)
class PlanarLinkageSpec:
    """Analytical planar linkage using two rigid circle-intersection stages.

    Stage 1:
        Driver point -> bell-crank input arm via a fixed-length rod.

    Stage 2:
        Bell-crank output arm -> steering arm via a fixed-length tie rod.

    The solver is intentionally explicit so the branch decision remains visible.
    """

    id: str
    steering_pivot: Point2D
    steering_arm_length_mm: float
    steering_arm_neutral_angle_rad: float
    bell_crank_pivot: Point2D
    bell_crank_input_arm_length_mm: float
    bell_crank_input_neutral_angle_rad: float
    bell_crank_output_arm_length_mm: float
    bell_crank_output_neutral_angle_rad: float
    input_rod_length_mm: float
    tie_rod_length_mm: float
    steering_stop_deg: float | None = None
    companion_steering_pivot: Point2D | None = None
    companion_steering_arm_length_mm: float | None = None
    companion_steering_arm_neutral_angle_rad: float = 0.0
    companion_tie_rod_length_mm: float | None = None


@dataclass(frozen=True, slots=True)
class PlanarLinkageBranchHint:
    input_endpoint: Point2D | None = None
    steering_endpoint: Point2D | None = None
    companion_steering_endpoint: Point2D | None = None


@dataclass(frozen=True, slots=True)
class PlanarLinkageState:
    driver_point: Point2D
    input_endpoint: Point2D
    bell_crank_angle_rad: float
    output_endpoint: Point2D
    steering_endpoint: Point2D
    steering_angle_rad: float
    input_stage_error_mm: float
    tie_rod_error_mm: float
    input_branch_index: int
    steering_branch_index: int
    companion_steering_endpoint: Point2D | None = None
    companion_steering_angle_rad: float | None = None
    companion_tie_rod_error_mm: float | None = None
    companion_branch_index: int | None = None

    @property
    def steering_angle_deg(self) -> float:
        return math.degrees(self.steering_angle_rad)

    @property
    def bell_crank_angle_deg(self) -> float:
        return math.degrees(self.bell_crank_angle_rad)

    @property
    def companion_steering_angle_deg(self) -> float | None:
        return None if self.companion_steering_angle_rad is None else math.degrees(self.companion_steering_angle_rad)


@dataclass(frozen=True, slots=True)
class PlanarLinkageSweepResult:
    states: tuple[PlanarLinkageState, ...]
    failure_index: int | None = None
    failure_error: LinkageNoSolutionError | LinkageBranchChangeError | SteeringLimitExceededError | None = None

    @property
    def succeeded(self) -> bool:
        return self.failure_index is None


@dataclass(frozen=True, slots=True)
class LinkageDemoRig:
    spec: PlanarLinkageSpec
    branch_hint: PlanarLinkageBranchHint
    driver_arc_center: Point2D
    driver_arc_radius_mm: float


def _distance(a: Point2D, b: Point2D) -> float:
    return (a - b).length()


def _circle_circle_intersections(
    center_a: Point2D,
    radius_a: float,
    center_b: Point2D,
    radius_b: float,
) -> tuple[Point2D, ...]:
    if radius_a < 0 or radius_b < 0:
        raise LinkageNoSolutionError("circle-circle", "Link lengths must be non-negative.")

    delta = center_b - center_a
    distance = delta.length()

    if distance <= EPSILON_MM:
        if abs(radius_a - radius_b) <= 1e-9:
            raise LinkageNoSolutionError("circle-circle", "Coincident circles produce infinite solutions.")
        raise LinkageNoSolutionError("circle-circle", "Concentric circles do not intersect.")

    if distance > radius_a + radius_b + 1e-9:
        raise LinkageNoSolutionError("circle-circle", "Circles are too far apart to intersect.")
    if distance < abs(radius_a - radius_b) - 1e-9:
        raise LinkageNoSolutionError("circle-circle", "One circle lies fully inside the other.")

    unit = delta.scale(1.0 / distance)
    a = (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance)
    h_sq = radius_a * radius_a - a * a
    if h_sq < -1e-9:
        raise LinkageNoSolutionError("circle-circle", "No real intersection exists.")

    base = center_a + unit.scale(a)
    if abs(h_sq) <= 1e-9:
        return (base,)

    height = math.sqrt(max(0.0, h_sq))
    perpendicular = Point2D(-unit.y_mm, unit.x_mm)
    return (
        base + perpendicular.scale(height),
        base - perpendicular.scale(height),
    )


def _select_candidate(
    candidates: tuple[Point2D, ...],
    *,
    previous_point: Point2D | None = None,
    preferred_point: Point2D | None = None,
    branch_tolerance_mm: float = 1e9,
) -> tuple[Point2D, int]:
    if not candidates:
        raise LinkageNoSolutionError("branch-selection", "No candidate intersection points were found.")

    def score(candidate: Point2D) -> tuple[float, float, float]:
        if previous_point is not None:
            return (
                _distance(candidate, previous_point),
                -candidate.y_mm,
                candidate.x_mm,
            )
        if preferred_point is not None:
            return (
                _distance(candidate, preferred_point),
                -candidate.y_mm,
                candidate.x_mm,
            )
        return (-candidate.y_mm, candidate.x_mm, _distance(candidate, Point2D(0.0, 0.0)))

    ranked = sorted(enumerate(candidates), key=lambda item: score(item[1]))
    index, chosen = ranked[0]

    if previous_point is not None and _distance(chosen, previous_point) > branch_tolerance_mm:
        raise LinkageBranchChangeError(
            "branch-selection",
            "Continuous solution jumped beyond the allowed branch tolerance.",
        )

    return chosen, index


def _rotate_point(pivot: Point2D, angle_rad: float, length_mm: float) -> Point2D:
    return pivot + heading_vector(angle_rad).scale(length_mm)


def _angle_from_pivot(pivot: Point2D, point: Point2D, neutral_angle_rad: float) -> float:
    vector = point - pivot
    if vector.length() <= EPSILON_MM:
        raise LinkageNoSolutionError("angle", "Linkage point coincides with its pivot.")
    return normalize_angle(math.atan2(vector.y_mm, vector.x_mm) - neutral_angle_rad)


def solve_planar_linkage(
    spec: PlanarLinkageSpec,
    driver_point: Point2D,
    previous_state: PlanarLinkageState | None = None,
    branch_hint: PlanarLinkageBranchHint | None = None,
    branch_tolerance_mm: float = DEFAULT_TOLERANCES.branch_continuity_mm,
) -> PlanarLinkageState:
    previous_input = previous_state.input_endpoint if previous_state is not None else None
    previous_steering = previous_state.steering_endpoint if previous_state is not None else None
    previous_companion = previous_state.companion_steering_endpoint if previous_state is not None else None
    hint_input = branch_hint.input_endpoint if branch_hint is not None else None
    hint_steering = branch_hint.steering_endpoint if branch_hint is not None else None
    hint_companion = branch_hint.companion_steering_endpoint if branch_hint is not None else None

    input_candidates = _circle_circle_intersections(
        spec.bell_crank_pivot,
        spec.bell_crank_input_arm_length_mm,
        driver_point,
        spec.input_rod_length_mm,
    )
    input_endpoint, input_index = _select_candidate(
        input_candidates,
        previous_point=previous_input,
        preferred_point=hint_input,
        branch_tolerance_mm=branch_tolerance_mm,
    )

    bell_crank_angle_rad = _angle_from_pivot(
        spec.bell_crank_pivot,
        input_endpoint,
        spec.bell_crank_input_neutral_angle_rad,
    )
    output_endpoint = _rotate_point(
        spec.bell_crank_pivot,
        spec.bell_crank_output_neutral_angle_rad + bell_crank_angle_rad,
        spec.bell_crank_output_arm_length_mm,
    )

    steering_candidates = _circle_circle_intersections(
        spec.steering_pivot,
        spec.steering_arm_length_mm,
        output_endpoint,
        spec.tie_rod_length_mm,
    )
    steering_endpoint, steering_index = _select_candidate(
        steering_candidates,
        previous_point=previous_steering,
        preferred_point=hint_steering,
        branch_tolerance_mm=branch_tolerance_mm,
    )

    steering_angle_rad = _angle_from_pivot(
        spec.steering_pivot,
        steering_endpoint,
        spec.steering_arm_neutral_angle_rad,
    )
    if spec.steering_stop_deg is not None:
        steering_angle_deg = math.degrees(steering_angle_rad)
        if abs(steering_angle_deg) > spec.steering_stop_deg + 1e-9:
            raise SteeringLimitExceededError(steering_angle_deg, spec.steering_stop_deg)

    input_stage_error_mm = _distance(driver_point, input_endpoint) - spec.input_rod_length_mm
    tie_rod_error_mm = _distance(output_endpoint, steering_endpoint) - spec.tie_rod_length_mm

    companion_endpoint: Point2D | None = None
    companion_angle_rad: float | None = None
    companion_tie_rod_error_mm: float | None = None
    companion_index: int | None = None
    companion_values = (
        spec.companion_steering_pivot,
        spec.companion_steering_arm_length_mm,
        spec.companion_tie_rod_length_mm,
    )
    if any(value is not None for value in companion_values):
        if (
            spec.companion_steering_pivot is None
            or spec.companion_steering_arm_length_mm is None
            or spec.companion_tie_rod_length_mm is None
            or spec.companion_steering_arm_length_mm <= 0.0
            or spec.companion_tie_rod_length_mm <= 0.0
        ):
            raise LinkageNoSolutionError(
                "companion-steering",
                "Companion steering geometry must define positive arm and tie-rod lengths.",
            )
        companion_candidates = _circle_circle_intersections(
            spec.companion_steering_pivot,
            spec.companion_steering_arm_length_mm,
            steering_endpoint,
            spec.companion_tie_rod_length_mm,
        )
        companion_endpoint, companion_index = _select_candidate(
            companion_candidates,
            previous_point=previous_companion,
            preferred_point=hint_companion,
            branch_tolerance_mm=branch_tolerance_mm,
        )
        companion_angle_rad = _angle_from_pivot(
            spec.companion_steering_pivot,
            companion_endpoint,
            spec.companion_steering_arm_neutral_angle_rad,
        )
        companion_tie_rod_error_mm = (
            _distance(steering_endpoint, companion_endpoint) - spec.companion_tie_rod_length_mm
        )
        if spec.steering_stop_deg is not None:
            companion_angle_deg = math.degrees(companion_angle_rad)
            if abs(companion_angle_deg) > spec.steering_stop_deg + 1e-9:
                raise SteeringLimitExceededError(companion_angle_deg, spec.steering_stop_deg)

    return PlanarLinkageState(
        driver_point=driver_point,
        input_endpoint=input_endpoint,
        bell_crank_angle_rad=bell_crank_angle_rad,
        output_endpoint=output_endpoint,
        steering_endpoint=steering_endpoint,
        steering_angle_rad=steering_angle_rad,
        input_stage_error_mm=input_stage_error_mm,
        tie_rod_error_mm=tie_rod_error_mm,
        input_branch_index=input_index,
        steering_branch_index=steering_index,
        companion_steering_endpoint=companion_endpoint,
        companion_steering_angle_rad=companion_angle_rad,
        companion_tie_rod_error_mm=companion_tie_rod_error_mm,
        companion_branch_index=companion_index,
    )


def solve_planar_linkage_sweep(
    spec: PlanarLinkageSpec,
    driver_points: Iterable[Point2D],
    initial_state: PlanarLinkageState | None = None,
    branch_hint: PlanarLinkageBranchHint | None = None,
    branch_tolerance_mm: float = DEFAULT_TOLERANCES.branch_continuity_mm,
) -> PlanarLinkageSweepResult:
    states: list[PlanarLinkageState] = []
    previous_state = initial_state
    for index, driver_point in enumerate(driver_points):
        try:
            current_state = solve_planar_linkage(
                spec,
                driver_point,
                previous_state=previous_state,
                branch_hint=branch_hint if previous_state is None else None,
                branch_tolerance_mm=branch_tolerance_mm,
            )
        except (LinkageNoSolutionError, LinkageBranchChangeError, SteeringLimitExceededError) as error:
            return PlanarLinkageSweepResult(
                states=tuple(states),
                failure_index=index,
                failure_error=error,
            )
        states.append(current_state)
        previous_state = current_state

    return PlanarLinkageSweepResult(states=tuple(states))


def driver_point_arc(
    center: Point2D,
    radius_mm: float,
    angle_rad: float,
) -> Point2D:
    return _rotate_point(center, angle_rad, radius_mm)


def build_linkage_rig(
    spec: PlanarLinkageSpec,
    *,
    driver_arc_center: Point2D,
    driver_arc_radius_mm: float,
    neutral_hint: PlanarLinkageBranchHint | None = None,
) -> LinkageDemoRig:
    """Build a solvable rig for a user-defined planar linkage.

    The neutral solve establishes a stable branch hint so subsequent slider
    steps can reuse the same physical configuration without branch jumps.
    """

    if not math.isfinite(driver_arc_radius_mm) or driver_arc_radius_mm <= 0.0:
        raise LinkageNoSolutionError("driver-arc", "The articulation driver radius must be positive and finite.")
    neutral_driver_point = driver_point_arc(driver_arc_center, driver_arc_radius_mm, 0.0)
    neutral_state = solve_planar_linkage(
        spec,
        neutral_driver_point,
        branch_hint=neutral_hint,
    )
    return LinkageDemoRig(
        spec=spec,
        branch_hint=PlanarLinkageBranchHint(
            input_endpoint=neutral_state.input_endpoint,
            steering_endpoint=neutral_state.steering_endpoint,
            companion_steering_endpoint=neutral_state.companion_steering_endpoint,
        ),
        driver_arc_center=driver_arc_center,
        driver_arc_radius_mm=driver_arc_radius_mm,
    )


@lru_cache(maxsize=1)
def build_reference_linkage_demo() -> LinkageDemoRig:
    """Build a small neutralized linkage demo used by the browser prototype."""

    base_spec = PlanarLinkageSpec(
        id="reference_demo_linkage_base",
        steering_pivot=Point2D(560.0, 180.0),
        steering_arm_length_mm=180.0,
        steering_arm_neutral_angle_rad=0.0,
        bell_crank_pivot=Point2D(0.0, 0.0),
        bell_crank_input_arm_length_mm=200.0,
        bell_crank_input_neutral_angle_rad=0.0,
        bell_crank_output_arm_length_mm=180.0,
        bell_crank_output_neutral_angle_rad=math.pi / 2.0,
        input_rod_length_mm=120.0,
        tie_rod_length_mm=560.0,
    )
    neutral_driver_point = Point2D(200.0, 120.0)
    neutral_hint = PlanarLinkageBranchHint(
        input_endpoint=Point2D(200.0, 0.0),
        steering_endpoint=Point2D(740.0, 180.0),
    )
    neutral_state = solve_planar_linkage(
        base_spec,
        neutral_driver_point,
        branch_hint=neutral_hint,
    )

    primary_neutral_angle_rad = math.atan2(
        (neutral_state.steering_endpoint - base_spec.steering_pivot).y_mm,
        (neutral_state.steering_endpoint - base_spec.steering_pivot).x_mm,
    )
    companion_pivot = Point2D(560.0, -180.0)
    companion_tie_rod_length_mm = 600.0
    companion_candidates = _circle_circle_intersections(
        companion_pivot,
        base_spec.steering_arm_length_mm,
        neutral_state.steering_endpoint,
        companion_tie_rod_length_mm,
    )
    companion_neutral_endpoint, _ = _select_candidate(
        companion_candidates,
        preferred_point=Point2D(740.0, -180.0),
    )
    companion_neutral_angle_rad = _angle_from_pivot(
        companion_pivot,
        companion_neutral_endpoint,
        0.0,
    )

    final_spec = PlanarLinkageSpec(
        id="reference_demo_linkage",
        steering_pivot=base_spec.steering_pivot,
        steering_arm_length_mm=base_spec.steering_arm_length_mm,
        steering_arm_neutral_angle_rad=primary_neutral_angle_rad,
        bell_crank_pivot=base_spec.bell_crank_pivot,
        bell_crank_input_arm_length_mm=base_spec.bell_crank_input_arm_length_mm,
        bell_crank_input_neutral_angle_rad=math.atan2(
            (neutral_state.input_endpoint - base_spec.bell_crank_pivot).y_mm,
            (neutral_state.input_endpoint - base_spec.bell_crank_pivot).x_mm,
        ),
        bell_crank_output_arm_length_mm=base_spec.bell_crank_output_arm_length_mm,
        bell_crank_output_neutral_angle_rad=math.atan2(
            (neutral_state.output_endpoint - base_spec.bell_crank_pivot).y_mm,
            (neutral_state.output_endpoint - base_spec.bell_crank_pivot).x_mm,
        ),
        input_rod_length_mm=base_spec.input_rod_length_mm,
        tie_rod_length_mm=base_spec.tie_rod_length_mm,
        companion_steering_pivot=companion_pivot,
        companion_steering_arm_length_mm=base_spec.steering_arm_length_mm,
        companion_steering_arm_neutral_angle_rad=companion_neutral_angle_rad,
        companion_tie_rod_length_mm=companion_tie_rod_length_mm,
    )
    return LinkageDemoRig(
        spec=final_spec,
        branch_hint=PlanarLinkageBranchHint(
            input_endpoint=neutral_state.input_endpoint,
            steering_endpoint=neutral_state.steering_endpoint,
            companion_steering_endpoint=companion_neutral_endpoint,
        ),
        # `driver_point_arc` evaluates angle zero at center + radius on +X.
        # Offset the center so the declared neutral point is actually beta=0.
        driver_arc_center=Point2D(neutral_driver_point.x_mm - 20.0, neutral_driver_point.y_mm),
        driver_arc_radius_mm=20.0,
    )


def solve_reference_linkage_demo(beta_deg: float) -> PlanarLinkageState:
    rig = build_reference_linkage_demo()
    driver_point = driver_point_arc(rig.driver_arc_center, rig.driver_arc_radius_mm, math.radians(beta_deg))
    return solve_planar_linkage(rig.spec, driver_point, branch_hint=rig.branch_hint)
