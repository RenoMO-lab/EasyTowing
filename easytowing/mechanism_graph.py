from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Literal, Mapping

from .errors import (
    InvalidGeometryError,
    LinkageBranchChangeError,
    LinkageNoSolutionError,
    SteeringLimitExceededError,
)
from .geometry import EPSILON_MM, Point2D, heading_vector, normalize_angle
from .linkage import PlanarLinkageBranchHint, PlanarLinkageSpec
from .model import Pose2D
from .tolerances import DEFAULT_TOLERANCES

PointMode = Literal["fixed", "driven", "free"]
MemberKind = Literal["arm", "rod", "rigid_brace"]


@dataclass(frozen=True, slots=True)
class MechanismPoint:
    id: str
    neutral_position: Point2D
    mode: PointMode = "free"
    envelope_radius_mm: float = 0.0
    body_id: str | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("Mechanism point IDs must not be empty.")
        if not math.isfinite(self.envelope_radius_mm) or self.envelope_radius_mm < 0.0:
            raise ValueError(f"Mechanism point {self.id!r} has an invalid envelope radius.")
        if self.body_id is not None and not self.body_id.strip():
            raise ValueError(f"Mechanism point {self.id!r} has an empty body ID.")


@dataclass(frozen=True, slots=True)
class MechanismDriverArc:
    point_id: str
    center: Point2D
    radius_mm: float
    neutral_angle_rad: float = 0.0
    input_ratio: float = 1.0
    phase_offset_rad: float = 0.0
    input_id: str = "articulation"

    def __post_init__(self) -> None:
        values = (
            self.radius_mm,
            self.neutral_angle_rad,
            self.input_ratio,
            self.phase_offset_rad,
        )
        if not self.point_id.strip():
            raise ValueError("Mechanism driver point ID must not be empty.")
        if not self.input_id.strip():
            raise ValueError("Mechanism driver input ID must not be empty.")
        if any(not math.isfinite(value) for value in values) or self.radius_mm <= 0.0:
            raise ValueError("Mechanism driver arc parameters must be finite with positive radius.")

    def local_position(self, input_angle_rad: float) -> Point2D:
        if not math.isfinite(input_angle_rad):
            raise ValueError("Mechanism driver input angle must be finite.")
        angle = self.neutral_angle_rad + self.input_ratio * input_angle_rad + self.phase_offset_rad
        return self.center + heading_vector(angle).scale(self.radius_mm)


@dataclass(frozen=True, slots=True)
class MechanismSteeringAssignment:
    output_id: str
    wheel_id: str
    ratio: float = 1.0
    phase_offset_rad: float = 0.0

    def __post_init__(self) -> None:
        if not self.output_id.strip() or not self.wheel_id.strip():
            raise ValueError("Mechanism steering assignments require output and wheel IDs.")
        if not math.isfinite(self.ratio) or not math.isfinite(self.phase_offset_rad):
            raise ValueError("Mechanism steering assignment parameters must be finite.")


@dataclass(frozen=True, slots=True)
class RigidMember:
    id: str
    point_a_id: str
    point_b_id: str
    length_mm: float
    kind: MemberKind = "rod"
    envelope_radius_mm: float = 0.0
    assembly_id: str | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("Rigid member IDs must not be empty.")
        if not math.isfinite(self.length_mm) or self.length_mm <= 0.0:
            raise ValueError(f"Rigid member {self.id!r} requires a positive finite length.")
        if not math.isfinite(self.envelope_radius_mm) or self.envelope_radius_mm < 0.0:
            raise ValueError(f"Rigid member {self.id!r} has an invalid envelope radius.")
        if self.point_a_id == self.point_b_id:
            raise ValueError(f"Rigid member {self.id!r} must connect two different points.")


@dataclass(frozen=True, slots=True)
class MechanismAngleOutput:
    id: str
    pivot_point_id: str
    endpoint_point_id: str
    neutral_angle_rad: float
    minimum_angle_rad: float | None = None
    maximum_angle_rad: float | None = None
    reference_body_id: str | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("Mechanism angle-output IDs must not be empty.")
        values = (
            self.neutral_angle_rad,
            self.minimum_angle_rad,
            self.maximum_angle_rad,
        )
        if any(value is not None and not math.isfinite(value) for value in values):
            raise ValueError(f"Angle output {self.id!r} requires finite angles.")
        if self.reference_body_id is not None and not self.reference_body_id.strip():
            raise ValueError(f"Angle output {self.id!r} has an empty reference body ID.")
        if (
            self.minimum_angle_rad is not None
            and self.maximum_angle_rad is not None
            and self.minimum_angle_rad >= self.maximum_angle_rad
        ):
            raise ValueError(f"Angle output {self.id!r} has invalid limits.")


@dataclass(frozen=True, slots=True)
class PlanarMechanismGraph:
    id: str
    points: tuple[MechanismPoint, ...]
    members: tuple[RigidMember, ...]
    angle_outputs: tuple[MechanismAngleOutput, ...] = ()

    def __post_init__(self) -> None:
        point_ids = [point.id for point in self.points]
        member_ids = [member.id for member in self.members]
        output_ids = [output.id for output in self.angle_outputs]
        if not self.points:
            raise ValueError("A mechanism graph requires at least one point.")
        if len(point_ids) != len(set(point_ids)):
            raise ValueError("Mechanism point IDs must be unique.")
        if len(member_ids) != len(set(member_ids)):
            raise ValueError("Mechanism member IDs must be unique.")
        if len(output_ids) != len(set(output_ids)):
            raise ValueError("Mechanism angle-output IDs must be unique.")
        if set(point_ids) & set(member_ids):
            raise ValueError("Mechanism point and member IDs must not overlap.")

        known_points = set(point_ids)
        for member in self.members:
            if member.point_a_id not in known_points or member.point_b_id not in known_points:
                raise ValueError(f"Rigid member {member.id!r} references an unknown point.")
        for output in self.angle_outputs:
            if (
                output.pivot_point_id not in known_points
                or output.endpoint_point_id not in known_points
            ):
                raise ValueError(f"Angle output {output.id!r} references an unknown point.")

    def point_by_id(self) -> dict[str, MechanismPoint]:
        return {point.id: point for point in self.points}

    def connected_member_pairs(self) -> frozenset[frozenset[str]]:
        member_points = {
            member.id: {member.point_a_id, member.point_b_id}
            for member in self.members
        }
        return frozenset(
            frozenset((left.id, right.id))
            for left_index, left in enumerate(self.members)
            for right in self.members[left_index + 1 :]
            if member_points[left.id] & member_points[right.id]
        )


@dataclass(frozen=True, slots=True)
class MechanismGraphState:
    point_positions: Mapping[str, Point2D]
    member_residuals_mm: Mapping[str, float]
    output_angles_rad: Mapping[str, float]
    iterations: int
    body_poses: Mapping[str, Pose2D] = field(default_factory=dict)

    @property
    def maximum_residual_mm(self) -> float:
        return max((abs(value) for value in self.member_residuals_mm.values()), default=0.0)

    def output_angle_deg(self, output_id: str) -> float:
        return math.degrees(self.output_angles_rad[output_id])


def resolve_driver_arc_positions(
    graph: PlanarMechanismGraph,
    drivers: tuple[MechanismDriverArc, ...],
    input_angles_rad: float | Mapping[str, float],
    *,
    body_poses: Mapping[str, Pose2D] | None = None,
) -> dict[str, Point2D]:
    point_by_id = graph.point_by_id()
    body_poses = body_poses or {}
    positions: dict[str, Point2D] = {}
    for driver in drivers:
        point = point_by_id.get(driver.point_id)
        if point is None:
            raise ValueError(f"Driver arc references unknown point {driver.point_id!r}.")
        if point.mode != "driven":
            raise ValueError(f"Driver arc point {driver.point_id!r} is not marked driven.")
        if driver.point_id in positions:
            raise ValueError(f"Multiple driver arcs reference point {driver.point_id!r}.")
        if isinstance(input_angles_rad, Mapping):
            if driver.input_id not in input_angles_rad:
                raise ValueError(f"Driver arc requires input {driver.input_id!r}.")
            input_angle_rad = input_angles_rad[driver.input_id]
        else:
            input_angle_rad = input_angles_rad
        local_position = driver.local_position(input_angle_rad)
        if point.body_id is None:
            positions[point.id] = local_position
        else:
            body_pose = body_poses.get(point.body_id)
            if body_pose is None:
                raise ValueError(f"Driver arc requires body pose {point.body_id!r}.")
            positions[point.id] = body_pose.transform_point(local_position)
    return positions


def _solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    augmented = [row[:] + [value] for row, value in zip(matrix, vector, strict=True)]
    for column in range(size):
        pivot_row = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot_row][column]) <= 1e-14:
            raise LinkageNoSolutionError("mechanism-graph", "Constraint Jacobian is singular.")
        augmented[column], augmented[pivot_row] = augmented[pivot_row], augmented[column]
        pivot = augmented[column][column]
        for item in range(column, size + 1):
            augmented[column][item] /= pivot
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            if abs(factor) <= 1e-18:
                continue
            for item in range(column, size + 1):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[row][size] for row in range(size)]


def _member_residual(member: RigidMember, positions: Mapping[str, Point2D]) -> float:
    return (positions[member.point_b_id] - positions[member.point_a_id]).length() - member.length_mm


def _build_positions(
    graph: PlanarMechanismGraph,
    driven_positions: Mapping[str, Point2D],
    previous_state: MechanismGraphState | None,
    body_poses: Mapping[str, Pose2D],
) -> dict[str, Point2D]:
    known_points = graph.point_by_id()
    unknown_drivers = set(driven_positions) - set(known_points)
    if unknown_drivers:
        raise ValueError(f"Unknown driven mechanism points: {sorted(unknown_drivers)!r}.")

    unknown_body_ids = {
        point.body_id
        for point in graph.points
        if point.body_id is not None and point.body_id not in body_poses
    }
    if unknown_body_ids:
        raise ValueError(f"Missing body poses for mechanism points: {sorted(unknown_body_ids)!r}.")

    def neutral_world(point: MechanismPoint) -> Point2D:
        if point.body_id is None:
            return point.neutral_position
        return body_poses[point.body_id].transform_point(point.neutral_position)

    positions: dict[str, Point2D] = {}
    for point in graph.points:
        if point.mode == "fixed":
            if point.id in driven_positions:
                raise ValueError(f"Fixed point {point.id!r} cannot be driven.")
            positions[point.id] = neutral_world(point)
        elif point.mode == "driven":
            positions[point.id] = driven_positions.get(point.id, neutral_world(point))
        elif previous_state is not None and point.id in previous_state.point_positions:
            previous_position = previous_state.point_positions[point.id]
            if (
                point.body_id is not None
                and point.body_id in previous_state.body_poses
                and point.body_id in body_poses
            ):
                previous_local_position = previous_state.body_poses[
                    point.body_id
                ].inverse_transform_point(previous_position)
                positions[point.id] = body_poses[point.body_id].transform_point(
                    previous_local_position
                )
            else:
                positions[point.id] = previous_position
        else:
            positions[point.id] = neutral_world(point)
    return positions


def _constraint_jacobian(
    graph: PlanarMechanismGraph,
    positions: Mapping[str, Point2D],
    variable_indexes: Mapping[str, int],
) -> tuple[list[float], list[list[float]]]:
    variable_count = len(variable_indexes) * 2
    residuals: list[float] = []
    jacobian: list[list[float]] = []
    for member in graph.members:
        point_a = positions[member.point_a_id]
        point_b = positions[member.point_b_id]
        delta = point_b - point_a
        distance = delta.length()
        if distance <= EPSILON_MM:
            raise LinkageNoSolutionError(
                "mechanism-graph",
                f"Member {member.id!r} has coincident endpoints.",
            )
        residuals.append(distance - member.length_mm)
        row = [0.0] * variable_count
        derivative_x = delta.x_mm / distance
        derivative_y = delta.y_mm / distance
        if member.point_a_id in variable_indexes:
            index = variable_indexes[member.point_a_id] * 2
            row[index] -= derivative_x
            row[index + 1] -= derivative_y
        if member.point_b_id in variable_indexes:
            index = variable_indexes[member.point_b_id] * 2
            row[index] += derivative_x
            row[index + 1] += derivative_y
        jacobian.append(row)
    return residuals, jacobian


def _normal_equations(
    residuals: list[float],
    jacobian: list[list[float]],
    variable_count: int,
    damping: float,
) -> tuple[list[list[float]], list[float]]:
    matrix = [[0.0] * variable_count for _ in range(variable_count)]
    vector = [0.0] * variable_count
    for residual, row in zip(residuals, jacobian, strict=True):
        for left in range(variable_count):
            vector[left] -= row[left] * residual
            for right in range(variable_count):
                matrix[left][right] += row[left] * row[right]
    for index in range(variable_count):
        matrix[index][index] += damping
    return matrix, vector


def solve_mechanism_graph(
    graph: PlanarMechanismGraph,
    driven_positions: Mapping[str, Point2D] | None = None,
    *,
    previous_state: MechanismGraphState | None = None,
    body_poses: Mapping[str, Pose2D] | None = None,
    geometric_tolerance_mm: float = DEFAULT_TOLERANCES.solver_residual_mm,
    branch_tolerance_mm: float = DEFAULT_TOLERANCES.branch_continuity_mm,
    maximum_iterations: int = 80,
) -> MechanismGraphState:
    if not math.isfinite(geometric_tolerance_mm) or geometric_tolerance_mm <= 0.0:
        raise ValueError("geometric_tolerance_mm must be positive and finite.")
    if not math.isfinite(branch_tolerance_mm) or branch_tolerance_mm <= 0.0:
        raise ValueError("branch_tolerance_mm must be positive and finite.")
    if maximum_iterations <= 0:
        raise ValueError("maximum_iterations must be positive.")

    driven_positions = driven_positions or {}
    body_poses = body_poses or {}
    point_by_id = graph.point_by_id()
    unknown_drivers = set(driven_positions) - set(point_by_id)
    if unknown_drivers:
        raise ValueError(f"Unknown driven mechanism points: {sorted(unknown_drivers)!r}.")
    invalid_driver_points = sorted(
        point_id
        for point_id in driven_positions
        if point_by_id[point_id].mode != "driven"
    )
    if invalid_driver_points:
        raise ValueError(
            f"Only driven mechanism points can receive positions: {invalid_driver_points!r}."
        )
    missing_drivers = sorted(
        point.id
        for point in graph.points
        if point.mode == "driven" and point.id not in driven_positions
    )
    if missing_drivers:
        raise InvalidGeometryError(
            f"Driven mechanism points have no resolved driver position: {missing_drivers!r}."
        )
    positions = _build_positions(graph, driven_positions, previous_state, body_poses)
    free_points = tuple(point for point in graph.points if point.mode == "free")
    variable_indexes = {point.id: index for index, point in enumerate(free_points)}
    variable_count = len(free_points) * 2
    active_member_count = sum(
        member.point_a_id in variable_indexes or member.point_b_id in variable_indexes
        for member in graph.members
    )
    if variable_count and active_member_count < variable_count:
        raise LinkageNoSolutionError(
            "mechanism-graph",
            f"Mechanism is underconstrained ({active_member_count} constraints for {variable_count} coordinates).",
        )

    damping = 1e-3
    iteration_count = 0
    for iteration in range(maximum_iterations + 1):
        iteration_count = iteration
        residuals, jacobian = _constraint_jacobian(graph, positions, variable_indexes)
        maximum_residual = max((abs(value) for value in residuals), default=0.0)
        if maximum_residual <= geometric_tolerance_mm:
            break
        if not variable_count:
            raise LinkageNoSolutionError(
                "mechanism-graph",
                f"Fixed mechanism violates a member length by {maximum_residual:.6f} mm.",
            )

        current_cost = sum(value * value for value in residuals)
        accepted = False
        for _ in range(12):
            matrix, vector = _normal_equations(
                residuals,
                jacobian,
                variable_count,
                damping,
            )
            step = _solve_linear_system(matrix, vector)
            trial = dict(positions)
            for point in free_points:
                index = variable_indexes[point.id] * 2
                trial[point.id] = Point2D(
                    positions[point.id].x_mm + step[index],
                    positions[point.id].y_mm + step[index + 1],
                )
            trial_residuals = [_member_residual(member, trial) for member in graph.members]
            trial_cost = sum(value * value for value in trial_residuals)
            if trial_cost < current_cost:
                positions = trial
                damping = max(damping * 0.3, 1e-12)
                accepted = True
                break
            damping = min(damping * 10.0, 1e12)
        if not accepted:
            raise LinkageNoSolutionError(
                "mechanism-graph",
                f"Constraint solve stalled with {maximum_residual:.6f} mm residual.",
            )
    else:
        raise LinkageNoSolutionError(
            "mechanism-graph",
            f"Constraint solve did not converge in {maximum_iterations} iterations.",
        )

    member_residuals = {
        member.id: _member_residual(member, positions)
        for member in graph.members
    }
    final_maximum_residual = max(
        (abs(value) for value in member_residuals.values()),
        default=0.0,
    )
    if final_maximum_residual > geometric_tolerance_mm:
        raise LinkageNoSolutionError(
            "mechanism-graph",
            f"Constraint residual {final_maximum_residual:.6f} mm exceeds tolerance.",
        )

    if previous_state is not None:
        for point in free_points:
            previous = previous_state.point_positions.get(point.id)
            if previous is None:
                continue
            if (
                point.body_id is not None
                and point.body_id in previous_state.body_poses
                and point.body_id in body_poses
            ):
                previous_reference = previous_state.body_poses[
                    point.body_id
                ].inverse_transform_point(previous)
                current_reference = body_poses[point.body_id].inverse_transform_point(
                    positions[point.id]
                )
                displacement = (current_reference - previous_reference).length()
            else:
                displacement = (positions[point.id] - previous).length()
            if displacement > branch_tolerance_mm:
                raise LinkageBranchChangeError(
                    "mechanism-graph",
                    f"Point {point.id!r} moved {displacement:.6f} mm between sweep samples.",
                )

    output_angles: dict[str, float] = {}
    for output in graph.angle_outputs:
        vector = positions[output.endpoint_point_id] - positions[output.pivot_point_id]
        if vector.length() <= EPSILON_MM:
            raise LinkageNoSolutionError(
                "mechanism-graph",
                f"Angle output {output.id!r} has coincident points.",
            )
        pivot_body_id = point_by_id[output.pivot_point_id].body_id
        endpoint_body_id = point_by_id[output.endpoint_point_id].body_id
        reference_yaw_rad = 0.0
        reference_body_id = output.reference_body_id
        if reference_body_id is None:
            reference_body_id = endpoint_body_id or pivot_body_id
        if reference_body_id is not None:
            if reference_body_id not in body_poses:
                raise ValueError(f"Angle output {output.id!r} requires body pose {reference_body_id!r}.")
            reference_yaw_rad = body_poses[reference_body_id].yaw_rad
        angle = normalize_angle(
            math.atan2(vector.y_mm, vector.x_mm)
            - reference_yaw_rad
            - output.neutral_angle_rad
        )
        if output.minimum_angle_rad is not None and angle < output.minimum_angle_rad - 1e-9:
            limit_deg = max(abs(math.degrees(output.minimum_angle_rad)), 0.0)
            raise SteeringLimitExceededError(math.degrees(angle), limit_deg)
        if output.maximum_angle_rad is not None and angle > output.maximum_angle_rad + 1e-9:
            limit_deg = max(abs(math.degrees(output.maximum_angle_rad)), 0.0)
            raise SteeringLimitExceededError(math.degrees(angle), limit_deg)
        output_angles[output.id] = angle

    return MechanismGraphState(
        point_positions=dict(positions),
        member_residuals_mm=member_residuals,
        output_angles_rad=output_angles,
        iterations=iteration_count,
        body_poses=dict(body_poses),
    )


def _rigid_chord_length(length_a: float, angle_a: float, length_b: float, angle_b: float) -> float:
    endpoint_a = heading_vector(angle_a).scale(length_a)
    endpoint_b = heading_vector(angle_b).scale(length_b)
    return (endpoint_b - endpoint_a).length()


def planar_linkage_to_mechanism_graph(
    spec: PlanarLinkageSpec,
    *,
    driver_neutral_position: Point2D,
    branch_hint: PlanarLinkageBranchHint | None = None,
) -> PlanarMechanismGraph:
    hint = branch_hint or PlanarLinkageBranchHint()
    input_endpoint = hint.input_endpoint or (
        spec.bell_crank_pivot
        + heading_vector(spec.bell_crank_input_neutral_angle_rad).scale(
            spec.bell_crank_input_arm_length_mm
        )
    )
    output_endpoint = spec.bell_crank_pivot + heading_vector(
        spec.bell_crank_output_neutral_angle_rad
    ).scale(spec.bell_crank_output_arm_length_mm)
    steering_endpoint = hint.steering_endpoint or (
        spec.steering_pivot
        + heading_vector(spec.steering_arm_neutral_angle_rad).scale(
            spec.steering_arm_length_mm
        )
    )

    points = [
        MechanismPoint("driver", driver_neutral_position, "driven"),
        MechanismPoint("bell_crank_pivot", spec.bell_crank_pivot, "fixed", 28.0),
        MechanismPoint("bell_crank_input", input_endpoint),
        MechanismPoint("bell_crank_output", output_endpoint),
        MechanismPoint("steering_pivot", spec.steering_pivot, "fixed", 28.0),
        MechanismPoint("steering_endpoint", steering_endpoint),
    ]
    members = [
        RigidMember("input_rod", "driver", "bell_crank_input", spec.input_rod_length_mm, "rod", 14.0),
        RigidMember("bell_crank_input_arm", "bell_crank_pivot", "bell_crank_input", spec.bell_crank_input_arm_length_mm, "arm", 14.0, "bell_crank"),
        RigidMember("bell_crank_output_arm", "bell_crank_pivot", "bell_crank_output", spec.bell_crank_output_arm_length_mm, "arm", 14.0, "bell_crank"),
        RigidMember(
            "bell_crank_rigid_brace",
            "bell_crank_input",
            "bell_crank_output",
            _rigid_chord_length(
                spec.bell_crank_input_arm_length_mm,
                spec.bell_crank_input_neutral_angle_rad,
                spec.bell_crank_output_arm_length_mm,
                spec.bell_crank_output_neutral_angle_rad,
            ),
            "rigid_brace",
            assembly_id="bell_crank",
        ),
        RigidMember("tie_rod", "bell_crank_output", "steering_endpoint", spec.tie_rod_length_mm, "rod", 14.0),
        RigidMember("steering_arm", "steering_pivot", "steering_endpoint", spec.steering_arm_length_mm, "arm", 14.0),
    ]
    stop_rad = None if spec.steering_stop_deg is None else math.radians(spec.steering_stop_deg)
    outputs = [
        MechanismAngleOutput(
            "bell_crank",
            "bell_crank_pivot",
            "bell_crank_input",
            spec.bell_crank_input_neutral_angle_rad,
        ),
        MechanismAngleOutput(
            "steering",
            "steering_pivot",
            "steering_endpoint",
            spec.steering_arm_neutral_angle_rad,
            None if stop_rad is None else -stop_rad,
            stop_rad,
        ),
    ]

    companion_values = (
        spec.companion_steering_pivot,
        spec.companion_steering_arm_length_mm,
        spec.companion_tie_rod_length_mm,
    )
    if any(value is not None for value in companion_values):
        if not all(value is not None for value in companion_values):
            raise ValueError("Companion linkage requires pivot, arm length, and tie-rod length.")
        assert spec.companion_steering_pivot is not None
        assert spec.companion_steering_arm_length_mm is not None
        assert spec.companion_tie_rod_length_mm is not None
        companion_endpoint = hint.companion_steering_endpoint or (
            spec.companion_steering_pivot
            + heading_vector(spec.companion_steering_arm_neutral_angle_rad).scale(
                spec.companion_steering_arm_length_mm
            )
        )
        points.extend(
            (
                MechanismPoint(
                    "companion_steering_pivot",
                    spec.companion_steering_pivot,
                    "fixed",
                    28.0,
                ),
                MechanismPoint("companion_steering_endpoint", companion_endpoint),
            )
        )
        members.extend(
            (
                RigidMember(
                    "companion_tie_rod",
                    "steering_endpoint",
                    "companion_steering_endpoint",
                    spec.companion_tie_rod_length_mm,
                    "rod",
                    14.0,
                ),
                RigidMember(
                    "companion_steering_arm",
                    "companion_steering_pivot",
                    "companion_steering_endpoint",
                    spec.companion_steering_arm_length_mm,
                    "arm",
                    14.0,
                ),
            )
        )
        outputs.append(
            MechanismAngleOutput(
                "companion_steering",
                "companion_steering_pivot",
                "companion_steering_endpoint",
                spec.companion_steering_arm_neutral_angle_rad,
                None if stop_rad is None else -stop_rad,
                stop_rad,
            )
        )

    return PlanarMechanismGraph(
        id=f"{spec.id}_graph",
        points=tuple(points),
        members=tuple(members),
        angle_outputs=tuple(outputs),
    )
