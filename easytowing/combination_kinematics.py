from __future__ import annotations

from dataclasses import dataclass
import math

from .errors import InvalidGeometryError, MultiBodyKinematicConstraintError
from .geometry import Point2D, heading_vector
from .model import Axle, Pose2D, VehicleCombination, combination_to_vehicle_layout
from .steering import IdealSteeringSolution, solve_ideal_steering


@dataclass(frozen=True, slots=True)
class AxleKinematicConstraint:
    axle_id: str
    body_id: str
    center: Point2D
    heading_rad: float
    residual_mm: float


@dataclass(frozen=True, slots=True)
class JointKinematicState:
    joint_id: str
    parent_body_id: str
    child_body_id: str
    parent_anchor_world: Point2D
    child_anchor_world: Point2D
    closure_error_mm: float
    articulation_rad: float


@dataclass(frozen=True, slots=True)
class CombinationKinematicSolution:
    combination_id: str
    body_poses: dict[str, Pose2D]
    icr: Point2D | None
    root_turn_radius_mm: float | None
    root_icr_longitudinal_offset_mm: float | None
    ideal_steering: IdealSteeringSolution
    axle_constraints: tuple[AxleKinematicConstraint, ...]
    joint_states: tuple[JointKinematicState, ...]
    maximum_constraint_residual_mm: float
    maximum_joint_closure_error_mm: float


def _fixed_mounted_axles(combination: VehicleCombination, body_poses: dict[str, Pose2D]) -> tuple[tuple[str, Axle], ...]:
    fixed: list[tuple[str, Axle]] = []
    for mounted in combination.mounted_axles:
        if mounted.axle.steerable and mounted.axle.steering_mode != "FIXED":
            continue
        fixed.append((mounted.body_id, mounted.resolve(body_poses[mounted.body_id])))
    return tuple(fixed)


def _axle_line_residual_mm(axle: Axle, icr: Point2D) -> float:
    rolling_direction = heading_vector(axle.heading_rad)
    return abs((icr - axle.center).dot(rolling_direction))


def _infer_icr_from_fixed_axles(fixed_axles: tuple[tuple[str, Axle], ...]) -> Point2D | None:
    if not fixed_axles:
        raise InvalidGeometryError(
            "A multi-body maneuver requires an explicit root turn radius when no fixed axle constrains the ICR."
        )

    directions = [heading_vector(axle.heading_rad) for _body_id, axle in fixed_axles]
    reference = directions[0]
    if all(
        abs(reference.x_mm * direction.y_mm - reference.y_mm * direction.x_mm) <= 1e-10
        for direction in directions[1:]
    ):
        return None

    xx = 0.0
    xy = 0.0
    yy = 0.0
    bx = 0.0
    by = 0.0
    for (_body_id, axle), direction in zip(fixed_axles, directions, strict=True):
        rhs = direction.dot(axle.center)
        xx += direction.x_mm * direction.x_mm
        xy += direction.x_mm * direction.y_mm
        yy += direction.y_mm * direction.y_mm
        bx += direction.x_mm * rhs
        by += direction.y_mm * rhs

    determinant = xx * yy - xy * xy
    if abs(determinant) <= 1e-12:
        raise InvalidGeometryError("Fixed-axle constraints do not define a unique instantaneous center of rotation.")
    return Point2D(
        (bx * yy - by * xy) / determinant,
        (xx * by - xy * bx) / determinant,
    )


def _joint_states(
    combination: VehicleCombination,
    body_poses: dict[str, Pose2D],
) -> tuple[JointKinematicState, ...]:
    states: list[JointKinematicState] = []
    for joint in combination.joints:
        parent_anchor = body_poses[joint.parent_body_id].transform_point(joint.parent_anchor)
        child_anchor = body_poses[joint.child_body_id].transform_point(joint.child_anchor)
        states.append(
            JointKinematicState(
                joint_id=joint.id,
                parent_body_id=joint.parent_body_id,
                child_body_id=joint.child_body_id,
                parent_anchor_world=parent_anchor,
                child_anchor_world=child_anchor,
                closure_error_mm=(parent_anchor - child_anchor).length(),
                articulation_rad=joint.articulation_rad,
            )
        )
    return tuple(states)


def solve_combination_kinematics(
    combination: VehicleCombination,
    *,
    root_pose: Pose2D | None = None,
    root_turn_radius_mm: float | None = None,
    constraint_tolerance_mm: float = 0.01,
) -> CombinationKinematicSolution:
    """Solve one steady-state articulated maneuver around a common world ICR.

    A root turn radius is mandatory when steerable axles leave the maneuver
    under-constrained. If omitted, the solver derives the ICR from fixed axle
    rolling constraints. Every additional fixed axle is then validated against
    that same ICR, so incompatible body articulations fail explicitly.
    """

    if not math.isfinite(constraint_tolerance_mm) or constraint_tolerance_mm <= 0.0:
        raise InvalidGeometryError("The multi-body constraint tolerance must be positive and finite.")
    if root_turn_radius_mm is not None and (
        not math.isfinite(root_turn_radius_mm) or abs(root_turn_radius_mm) <= constraint_tolerance_mm
    ):
        raise InvalidGeometryError("The root turn radius must be finite and larger than the constraint tolerance.")

    body_poses = combination.resolve_body_poses(root_pose=root_pose)
    if not body_poses:
        raise InvalidGeometryError("A multi-body maneuver requires at least one rigid body.")
    root_id = combination.root_body_id or combination.bodies[0].id
    resolved_root_pose = body_poses[root_id]
    fixed_axles = _fixed_mounted_axles(combination, body_poses)

    if root_turn_radius_mm is None:
        icr = _infer_icr_from_fixed_axles(fixed_axles)
    else:
        icr = resolved_root_pose.transform_point(Point2D(0.0, root_turn_radius_mm))

    constraints: list[AxleKinematicConstraint] = []
    if icr is not None:
        for body_id, axle in fixed_axles:
            residual = _axle_line_residual_mm(axle, icr)
            constraints.append(
                AxleKinematicConstraint(
                    axle_id=axle.id,
                    body_id=body_id,
                    center=axle.center,
                    heading_rad=axle.heading_rad,
                    residual_mm=residual,
                )
            )
        maximum_residual = max((item.residual_mm for item in constraints), default=0.0)
        if maximum_residual > constraint_tolerance_mm:
            worst = max(constraints, key=lambda item: item.residual_mm)
            raise MultiBodyKinematicConstraintError(
                worst.axle_id,
                worst.residual_mm,
                constraint_tolerance_mm,
            )
    else:
        maximum_residual = 0.0

    joints = _joint_states(combination, body_poses)
    maximum_joint_error = max((state.closure_error_mm for state in joints), default=0.0)
    if maximum_joint_error > constraint_tolerance_mm:
        worst_joint = max(joints, key=lambda item: item.closure_error_mm)
        raise MultiBodyKinematicConstraintError(
            worst_joint.joint_id,
            worst_joint.closure_error_mm,
            constraint_tolerance_mm,
        )

    vehicle = combination_to_vehicle_layout(combination, root_pose=root_pose)
    ideal = solve_ideal_steering(vehicle, icr)
    if icr is None:
        resolved_radius = None
        longitudinal_offset = None
    else:
        root_local_icr = resolved_root_pose.inverse_transform_point(icr)
        resolved_radius = root_local_icr.y_mm
        longitudinal_offset = root_local_icr.x_mm

    return CombinationKinematicSolution(
        combination_id=combination.id,
        body_poses=body_poses,
        icr=icr,
        root_turn_radius_mm=resolved_radius,
        root_icr_longitudinal_offset_mm=longitudinal_offset,
        ideal_steering=ideal,
        axle_constraints=tuple(constraints),
        joint_states=joints,
        maximum_constraint_residual_mm=maximum_residual,
        maximum_joint_closure_error_mm=maximum_joint_error,
    )
