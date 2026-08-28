from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING

from .errors import InvalidGeometryError, SteeringLimitExceededError
from .geometry import Point2D, normalize_angle
from .linkage import PlanarLinkageState
from .mechanism_graph import MechanismGraphState, MechanismSteeringAssignment
from .model import Axle, SteeringSynchronization, VehicleLayout

if TYPE_CHECKING:
    from .steering import IdealSteeringSolution


@dataclass(frozen=True, slots=True)
class ActualWheelSteering:
    wheel_id: str
    axle_id: str
    side: str
    center: Point2D
    heading_rad: float
    steering_angle_rad: float
    source: str

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.heading_rad)

    @property
    def steering_angle_deg(self) -> float:
        return math.degrees(self.steering_angle_rad)


@dataclass(frozen=True, slots=True)
class ActualAxleSteering:
    axle_id: str
    center: Point2D
    center_heading_rad: float
    center_steering_angle_rad: float
    left_wheel: ActualWheelSteering
    right_wheel: ActualWheelSteering
    source: str
    synchronization_mode: str | None
    wheels: tuple[ActualWheelSteering, ...] = ()

    @property
    def wheel_solutions(self) -> tuple[ActualWheelSteering, ...]:
        """Return every wheel, including legacy left/right compatibility fields."""

        return self.wheels or (self.left_wheel, self.right_wheel)

    @property
    def center_heading_deg(self) -> float:
        return math.degrees(self.center_heading_rad)

    @property
    def center_steering_angle_deg(self) -> float:
        return math.degrees(self.center_steering_angle_rad)


@dataclass(frozen=True, slots=True)
class ActualSteeringSolution:
    axles: tuple[ActualAxleSteering, ...]
    wheel_angles_rad: dict[str, float]
    axle_center_angles_rad: dict[str, float]
    synchronization_target_angles_rad: dict[str, float] = field(default_factory=dict)
    synchronization_errors_rad: dict[str, float] = field(default_factory=dict)

    def wheel_steering_angles_deg(self) -> dict[str, float]:
        return {wheel_id: math.degrees(angle) for wheel_id, angle in self.wheel_angles_rad.items()}

    def axle_center_steering_angles_deg(self) -> dict[str, float]:
        return {axle_id: math.degrees(angle) for axle_id, angle in self.axle_center_angles_rad.items()}

    def synchronization_target_angles_deg(self) -> dict[str, float]:
        return {
            sync_id: math.degrees(angle)
            for sync_id, angle in self.synchronization_target_angles_rad.items()
        }

    def synchronization_errors_deg(self) -> dict[str, float]:
        return {
            sync_id: math.degrees(angle)
            for sync_id, angle in self.synchronization_errors_rad.items()
        }


def _interpolate_target_curve(sync: SteeringSynchronization, beta_rad: float) -> float:
    points = sync.target_curve
    if not points:
        raise InvalidGeometryError(
            f"Independent synchronization {sync.id!r} has no target curve points."
        )
    betas = [point.beta_rad for point in points]
    index = bisect_left(betas, beta_rad)
    if index <= 0:
        return points[0].steering_angle_rad
    if index >= len(points):
        return points[-1].steering_angle_rad
    left = points[index - 1]
    right = points[index]
    span = right.beta_rad - left.beta_rad
    fraction = (beta_rad - left.beta_rad) / span
    return left.steering_angle_rad + fraction * (right.steering_angle_rad - left.steering_angle_rad)


def _sync_by_target(vehicle: VehicleLayout) -> dict[str, SteeringSynchronization]:
    return {item.target_axle_id: item for item in vehicle.steering_synchronizations}


def solve_actual_steering(
    vehicle: VehicleLayout,
    linkage_state: PlanarLinkageState,
    beta_rad: float,
    ideal_solution: IdealSteeringSolution | None = None,
) -> ActualSteeringSolution:
    """Resolve linkage output into actual steering commands for every axle.

    The primary linkage output drives the foremost steerable axle. A configured
    synchronization channel then derives other axle commands. The primary
    companion output remains the second wheel on that axle, while secondary
    axle wheel angles are reported as center-driven commands until a generalized
    multi-axle rigid linkage network is supplied.
    """

    if not math.isfinite(beta_rad):
        raise InvalidGeometryError("Actual steering beta must be finite.")
    steerable_axles = [
        axle for axle in vehicle.axles if axle.steerable and axle.steering_mode != "FIXED"
    ]
    primary_axle = max(steerable_axles, key=lambda axle: axle.center.x_mm) if steerable_axles else None
    primary_axle_id = primary_axle.id if primary_axle is not None else None
    primary_left_rad = linkage_state.steering_angle_rad
    primary_right_rad = (
        linkage_state.companion_steering_angle_rad
        if linkage_state.companion_steering_angle_rad is not None
        else primary_left_rad
    )
    primary_center_rad = (primary_left_rad + primary_right_rad) / 2.0
    sync_by_axle = _sync_by_target(vehicle)
    axle_by_id = {axle.id: axle for axle in vehicle.axles}
    center_cache: dict[str, float] = {}
    source_cache: dict[str, str] = {}
    resolving: set[str] = set()

    def ideal_center(axle_id: str) -> float | None:
        if ideal_solution is None:
            return None
        for solution in ideal_solution.axles:
            if solution.axle_id == axle_id:
                return solution.center_steering_angle_rad
        return None

    def resolve_center(axle_id: str) -> float:
        if axle_id in center_cache:
            return center_cache[axle_id]
        if axle_id in resolving:
            raise InvalidGeometryError("Steering synchronization graph contains a cycle.")
        axle = axle_by_id[axle_id]
        if not axle.steerable or axle.steering_mode == "FIXED":
            center_cache[axle_id] = 0.0
            source_cache[axle_id] = "fixed"
            return 0.0
        if axle.steering_mode == "USER_DEFINED":
            center_cache[axle_id] = axle.user_defined_steering_angle_rad
            source_cache[axle_id] = "user_defined"
            return center_cache[axle_id]
        if axle.steering_mode == "SELF_STEER":
            target = ideal_center(axle_id)
            if target is not None:
                center_cache[axle_id] = target
                source_cache[axle_id] = "self_steer_target"
                return target
        if axle_id == primary_axle_id:
            center_cache[axle_id] = primary_center_rad
            source_cache[axle_id] = "linkage_primary"
            return primary_center_rad

        resolving.add(axle_id)
        sync = sync_by_axle.get(axle_id)
        if sync is None:
            source_angle = primary_center_rad
            source = "linkage_primary:same_phase"
        elif sync.mode == "INDEPENDENT_TARGET":
            center_cache[axle_id] = _interpolate_target_curve(sync, beta_rad)
            source_cache[axle_id] = f"target_curve:{sync.id}"
            resolving.remove(axle_id)
            return center_cache[axle_id]
        else:
            source_id = sync.source_axle_id or primary_axle_id
            if source_id is None:
                source_angle = 0.0
                source = "no_steerable_source"
            else:
                source_angle = resolve_center(source_id)
                source = f"{sync.mode.lower()}:{source_id}"
            if sync.mode == "OPPOSITE_PHASE":
                source_angle = -abs(sync.ratio) * source_angle
            else:
                source_angle *= sync.ratio
            source_angle += sync.phase_offset_rad
        resolving.remove(axle_id)
        center_cache[axle_id] = normalize_angle(source_angle)
        source_cache[axle_id] = source
        return center_cache[axle_id]

    actual_axles: list[ActualAxleSteering] = []
    wheel_angles_rad: dict[str, float] = {}
    axle_center_angles_rad: dict[str, float] = {}
    for axle in vehicle.axles:
        center_angle_rad = resolve_center(axle.id)
        wheels = axle.wheels()
        wheel_source = source_cache[axle.id]
        limits = [
            limit
            for limit in (axle.maximum_steering_angle_deg, axle.steering_stop_deg)
            if limit is not None
        ]
        if limits:
            limit_deg = min(abs(limit) for limit in limits)
        actual_wheels: list[ActualWheelSteering] = []
        for wheel in wheels:
            angle_rad = (
                primary_left_rad if wheel.side == "left" else primary_right_rad
            ) if axle.id == primary_axle_id else center_angle_rad
            if limits:
                angle_deg = math.degrees(angle_rad)
                if abs(angle_deg) > limit_deg + 1e-9:
                    raise SteeringLimitExceededError(angle_deg, limit_deg)
            actual_wheels.append(
                ActualWheelSteering(
                    wheel_id=wheel.id,
                    axle_id=axle.id,
                    side=wheel.side,
                    center=wheel.center,
                    heading_rad=axle.heading_rad + angle_rad,
                    steering_angle_rad=angle_rad,
                    source=wheel_source,
                )
            )
        actual_left = next(wheel for wheel in actual_wheels if wheel.side == "left")
        actual_right = next(wheel for wheel in actual_wheels if wheel.side == "right")
        axle_solution = ActualAxleSteering(
            axle_id=axle.id,
            center=axle.center,
            center_heading_rad=axle.heading_rad + center_angle_rad,
            center_steering_angle_rad=center_angle_rad,
            left_wheel=actual_left,
            right_wheel=actual_right,
            source=wheel_source,
            synchronization_mode=sync_by_axle.get(axle.id).mode if axle.id in sync_by_axle else None,
            wheels=tuple(actual_wheels),
        )
        actual_axles.append(axle_solution)
        axle_center_angles_rad[axle.id] = center_angle_rad
        for wheel in actual_wheels:
            wheel_angles_rad[wheel.wheel_id] = wheel.steering_angle_rad

    synchronization_target_angles_rad: dict[str, float] = {}
    synchronization_errors_rad: dict[str, float] = {}
    for sync in vehicle.steering_synchronizations:
        if sync.mode == "INDEPENDENT_TARGET":
            target_angle_rad = _interpolate_target_curve(sync, beta_rad)
        else:
            source_id = sync.source_axle_id or primary_axle_id
            source_angle_rad = 0.0 if source_id is None else resolve_center(source_id)
            target_angle_rad = (
                -abs(sync.ratio) * source_angle_rad
                if sync.mode == "OPPOSITE_PHASE"
                else sync.ratio * source_angle_rad
            ) + sync.phase_offset_rad
        target_angle_rad = normalize_angle(target_angle_rad)
        actual_angle_rad = axle_center_angles_rad[sync.target_axle_id]
        synchronization_target_angles_rad[sync.id] = target_angle_rad
        synchronization_errors_rad[sync.id] = normalize_angle(actual_angle_rad - target_angle_rad)

    return ActualSteeringSolution(
        axles=tuple(actual_axles),
        wheel_angles_rad=wheel_angles_rad,
        axle_center_angles_rad=axle_center_angles_rad,
        synchronization_target_angles_rad=synchronization_target_angles_rad,
        synchronization_errors_rad=synchronization_errors_rad,
    )


def solve_actual_steering_from_graph(
    vehicle: VehicleLayout,
    graph_state: MechanismGraphState,
    assignments: tuple[MechanismSteeringAssignment, ...],
) -> ActualSteeringSolution:
    """Map named mechanism outputs to named wheels without positional assumptions."""

    wheel_by_id = {
        wheel.id: wheel
        for axle in vehicle.axles
        for wheel in axle.wheels()
    }
    axle_by_wheel = {
        wheel.id: axle
        for axle in vehicle.axles
        for wheel in axle.wheels()
    }
    assignment_by_wheel: dict[str, MechanismSteeringAssignment] = {}
    for assignment in assignments:
        if assignment.output_id not in graph_state.output_angles_rad:
            raise InvalidGeometryError(
                f"Mechanism assignment references unknown output {assignment.output_id!r}."
            )
        if assignment.wheel_id not in wheel_by_id:
            raise InvalidGeometryError(
                f"Mechanism assignment references unknown wheel {assignment.wheel_id!r}."
            )
        if assignment.wheel_id in assignment_by_wheel:
            raise InvalidGeometryError(
                f"Wheel {assignment.wheel_id!r} has multiple mechanism assignments."
            )
        axle = axle_by_wheel[assignment.wheel_id]
        if not axle.steerable or axle.steering_mode == "FIXED":
            raise InvalidGeometryError(
                f"Fixed wheel {assignment.wheel_id!r} cannot receive a mechanism output."
            )
        assignment_by_wheel[assignment.wheel_id] = assignment

    actual_axles: list[ActualAxleSteering] = []
    wheel_angles_rad: dict[str, float] = {}
    axle_center_angles_rad: dict[str, float] = {}
    for axle in vehicle.axles:
        wheel_solutions: list[ActualWheelSteering] = []
        for wheel in axle.wheels():
            assignment = assignment_by_wheel.get(wheel.id)
            if not axle.steerable or axle.steering_mode == "FIXED":
                angle_rad = 0.0
                source = "fixed"
            elif assignment is None:
                raise InvalidGeometryError(
                    f"Steerable wheel {wheel.id!r} has no mechanism output assignment."
                )
            else:
                angle_rad = normalize_angle(
                    assignment.ratio * graph_state.output_angles_rad[assignment.output_id]
                    + assignment.phase_offset_rad
                )
                source = f"mechanism_graph:{assignment.output_id}"

            limits = [
                limit
                for limit in (axle.maximum_steering_angle_deg, axle.steering_stop_deg)
                if limit is not None
            ]
            if limits:
                limit_deg = min(abs(limit) for limit in limits)
                angle_deg = math.degrees(angle_rad)
                if abs(angle_deg) > limit_deg + 1e-9:
                    raise SteeringLimitExceededError(angle_deg, limit_deg)

            wheel_solutions.append(
                ActualWheelSteering(
                    wheel_id=wheel.id,
                    axle_id=axle.id,
                    side=wheel.side,
                    center=wheel.center,
                    heading_rad=normalize_angle(axle.heading_rad + angle_rad),
                    steering_angle_rad=angle_rad,
                    source=source,
                )
            )
            wheel_angles_rad[wheel.id] = angle_rad

        left_actual = next(wheel for wheel in wheel_solutions if wheel.side == "left")
        right_actual = next(wheel for wheel in wheel_solutions if wheel.side == "right")
        center_angle_rad = normalize_angle(
            sum(wheel.steering_angle_rad for wheel in wheel_solutions) / len(wheel_solutions)
        )
        axle_center_angles_rad[axle.id] = center_angle_rad
        actual_axles.append(
            ActualAxleSteering(
                axle_id=axle.id,
                center=axle.center,
                center_heading_rad=normalize_angle(axle.heading_rad + center_angle_rad),
                center_steering_angle_rad=center_angle_rad,
                left_wheel=left_actual,
                right_wheel=right_actual,
                source="mechanism_graph",
                synchronization_mode="LINKED_MECHANICALLY",
                wheels=tuple(wheel_solutions),
            )
        )

    return ActualSteeringSolution(
        axles=tuple(actual_axles),
        wheel_angles_rad=wheel_angles_rad,
        axle_center_angles_rad=axle_center_angles_rad,
    )


def actual_steering_errors_deg(
    actual: ActualSteeringSolution,
    ideal: IdealSteeringSolution,
) -> dict[str, float]:
    return {
        wheel_id: math.degrees(normalize_angle(actual_angle - ideal_angle))
        for wheel_id, actual_angle in actual.wheel_angles_rad.items()
        if (ideal_angle := ideal.wheel_steering_angles_rad().get(wheel_id)) is not None
    }


def compare_actual_to_ideal(
    actual: ActualSteeringSolution,
    ideal: IdealSteeringSolution,
    vehicle: VehicleLayout | None = None,
    beta_rad: float | None = None,
) -> dict[str, object]:
    """Return traceable angular error metrics for every solved wheel and axle."""

    ideal_wheel_angles = ideal.wheel_steering_angles_rad()
    errors_deg = actual_steering_errors_deg(actual, ideal)
    left_errors: list[float] = []
    right_errors: list[float] = []
    for axle in actual.axles:
        for wheel in axle.wheel_solutions:
            bucket = left_errors if wheel.side == "left" else right_errors
            if wheel.wheel_id in ideal_wheel_angles:
                bucket.append(errors_deg[wheel.wheel_id])
    all_errors = list(errors_deg.values())
    ideal_axle_angles = ideal.axle_center_steering_angles_rad()
    axle_errors_deg = {
        axle_id: math.degrees(
            normalize_angle(actual_angle - ideal_axle_angles[axle_id])
        )
        for axle_id, actual_angle in actual.axle_center_angles_rad.items()
        if axle_id in ideal_axle_angles
    }
    synchronization_errors_deg = actual.synchronization_errors_deg()
    synchronization_target_angles_deg = actual.synchronization_target_angles_deg()
    if vehicle is not None and beta_rad is not None:
        steerable_axles = [
            axle for axle in vehicle.axles if axle.steerable and axle.steering_mode != "FIXED"
        ]
        primary_axle_id = (
            max(steerable_axles, key=lambda axle: axle.center.x_mm).id
            if steerable_axles
            else None
        )
        for sync in vehicle.steering_synchronizations:
            if sync.mode == "INDEPENDENT_TARGET":
                target_angle_rad = _interpolate_target_curve(sync, beta_rad)
            else:
                source_id = sync.source_axle_id or primary_axle_id
                source_angle_rad = 0.0 if source_id is None else ideal_axle_angles.get(source_id, 0.0)
                target_angle_rad = (
                    -abs(sync.ratio) * source_angle_rad
                    if sync.mode == "OPPOSITE_PHASE"
                    else sync.ratio * source_angle_rad
                ) + sync.phase_offset_rad
            actual_angle_rad = actual.axle_center_angles_rad.get(sync.target_axle_id, 0.0)
            synchronization_target_angles_deg[sync.id] = math.degrees(
                normalize_angle(target_angle_rad)
            )
            synchronization_errors_deg[sync.id] = math.degrees(
                normalize_angle(actual_angle_rad - target_angle_rad)
            )
    front = max(actual.axles, key=lambda axle: axle.center.x_mm) if actual.axles else None
    rear = min(actual.axles, key=lambda axle: axle.center.x_mm) if actual.axles else None
    sync_error_deg = None
    if front is not None and rear is not None and front.axle_id != rear.axle_id:
        ideal_front = ideal_axle_angles.get(front.axle_id)
        ideal_rear = ideal_axle_angles.get(rear.axle_id)
        if ideal_front is not None and ideal_rear is not None:
            actual_phase = front.center_steering_angle_rad - rear.center_steering_angle_rad
            ideal_phase = ideal_front - ideal_rear
            sync_error_deg = math.degrees(normalize_angle(actual_phase - ideal_phase))

    return {
        "wheel_errors_deg": errors_deg,
        "axle_center_errors_deg": axle_errors_deg,
        "max_abs_error_deg": max((abs(value) for value in all_errors), default=0.0),
        "mean_abs_error_deg": (
            sum(abs(value) for value in all_errors) / len(all_errors)
            if all_errors
            else 0.0
        ),
        "rms_error_deg": (
            math.sqrt(sum(value * value for value in all_errors) / len(all_errors))
            if all_errors
            else 0.0
        ),
        "max_abs_inner_error_deg": max(
            (abs(value) for value in left_errors),
            default=0.0,
        ),
        "max_abs_outer_error_deg": max(
            (abs(value) for value in right_errors),
            default=0.0,
        ),
        "synchronization_target_angles_deg": synchronization_target_angles_deg,
        "synchronization_errors_deg": synchronization_errors_deg,
        "max_abs_synchronization_error_deg": max(
            (abs(value) for value in synchronization_errors_deg.values()),
            default=0.0,
        ),
        "front_rear_synchronization_error_deg": sync_error_deg,
    }


def serialize_actual_steering(
    actual: ActualSteeringSolution,
    ideal: IdealSteeringSolution | None = None,
    vehicle: VehicleLayout | None = None,
    beta_rad: float | None = None,
) -> dict[str, object]:
    errors = {} if ideal is None else actual_steering_errors_deg(actual, ideal)
    comparison = (
        None
        if ideal is None
        else compare_actual_to_ideal(actual, ideal, vehicle=vehicle, beta_rad=beta_rad)
    )
    return {
        "wheel_angles_deg": actual.wheel_steering_angles_deg(),
        "axle_center_angles_deg": actual.axle_center_steering_angles_deg(),
        "errors_deg": errors,
        "synchronization_target_angles_deg": actual.synchronization_target_angles_deg(),
        "synchronization_command_errors_deg": actual.synchronization_errors_deg(),
        "synchronization_errors_deg": (
            actual.synchronization_errors_deg()
            if comparison is None
            else comparison["synchronization_errors_deg"]
        ),
        "synchronization_ideal_target_angles_deg": (
            {}
            if comparison is None
            else comparison["synchronization_target_angles_deg"]
        ),
        "axles": [
            {
                "axle_id": axle.axle_id,
                "center": {"x_mm": axle.center.x_mm, "y_mm": axle.center.y_mm},
                "center_heading_rad": axle.center_heading_rad,
                "center_heading_deg": axle.center_heading_deg,
                "center_steering_angle_rad": axle.center_steering_angle_rad,
                "center_steering_angle_deg": axle.center_steering_angle_deg,
                "source": axle.source,
                "synchronization_mode": axle.synchronization_mode,
                "left_wheel": {
                    "wheel_id": axle.left_wheel.wheel_id,
                    "heading_rad": axle.left_wheel.heading_rad,
                    "heading_deg": axle.left_wheel.heading_deg,
                    "steering_angle_rad": axle.left_wheel.steering_angle_rad,
                    "steering_angle_deg": axle.left_wheel.steering_angle_deg,
                    "source": axle.left_wheel.source,
                },
                "right_wheel": {
                    "wheel_id": axle.right_wheel.wheel_id,
                    "heading_rad": axle.right_wheel.heading_rad,
                    "heading_deg": axle.right_wheel.heading_deg,
                    "steering_angle_rad": axle.right_wheel.steering_angle_rad,
                    "steering_angle_deg": axle.right_wheel.steering_angle_deg,
                    "source": axle.right_wheel.source,
                },
                "wheels": [
                    {
                        "wheel_id": wheel.wheel_id,
                        "side": wheel.side,
                        "center": {"x_mm": wheel.center.x_mm, "y_mm": wheel.center.y_mm},
                        "heading_rad": wheel.heading_rad,
                        "heading_deg": wheel.heading_deg,
                        "steering_angle_rad": wheel.steering_angle_rad,
                        "steering_angle_deg": wheel.steering_angle_deg,
                        "source": wheel.source,
                    }
                    for wheel in axle.wheel_solutions
                ],
            }
            for axle in actual.axles
        ],
    }
