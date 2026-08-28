from __future__ import annotations

from dataclasses import dataclass
import math

from .errors import InvalidGeometryError, SteeringLimitExceededError
from .geometry import Point2D, heading_vector, normalize_angle, tangent_heading_from_icr
from .model import Axle, VehicleLayout, Wheel, build_reference_demo_layout


@dataclass(frozen=True, slots=True)
class IdealWheelSolution:
    wheel_id: str
    axle_id: str
    side: str
    center: Point2D
    heading_rad: float
    reference_heading_rad: float = 0.0
    steering_angle_rad: float = 0.0

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.heading_rad)

    @property
    def reference_heading_deg(self) -> float:
        return math.degrees(self.reference_heading_rad)

    @property
    def steering_angle_deg(self) -> float:
        return math.degrees(self.steering_angle_rad)


@dataclass(frozen=True, slots=True)
class IdealAxleSolution:
    axle_id: str
    center: Point2D
    center_heading_rad: float
    left_wheel: IdealWheelSolution
    right_wheel: IdealWheelSolution
    reference_heading_rad: float = 0.0
    center_steering_angle_rad: float = 0.0
    wheels: tuple[IdealWheelSolution, ...] = ()

    @property
    def wheel_solutions(self) -> tuple[IdealWheelSolution, ...]:
        """Return every wheel, including legacy left/right compatibility fields."""

        return self.wheels or (self.left_wheel, self.right_wheel)

    @property
    def center_heading_deg(self) -> float:
        return math.degrees(self.center_heading_rad)

    @property
    def reference_heading_deg(self) -> float:
        return math.degrees(self.reference_heading_rad)

    @property
    def center_steering_angle_deg(self) -> float:
        return math.degrees(self.center_steering_angle_rad)


@dataclass(frozen=True, slots=True)
class IdealSteeringSolution:
    icr: Point2D | None
    axles: tuple[IdealAxleSolution, ...]
    wheel_angles_rad: dict[str, float]
    axle_center_angles_rad: dict[str, float]

    def wheel_angles_deg(self) -> dict[str, float]:
        return {wheel_id: math.degrees(angle) for wheel_id, angle in self.wheel_angles_rad.items()}

    def axle_center_angles_deg(self) -> dict[str, float]:
        return {axle_id: math.degrees(angle) for axle_id, angle in self.axle_center_angles_rad.items()}

    def wheel_steering_angles_rad(self) -> dict[str, float]:
        return {
            wheel.wheel_id: wheel.steering_angle_rad
            for axle in self.axles
            for wheel in axle.wheel_solutions
        }

    def wheel_steering_angles_deg(self) -> dict[str, float]:
        return {
            wheel_id: math.degrees(angle)
            for wheel_id, angle in self.wheel_steering_angles_rad().items()
        }

    def axle_center_steering_angles_rad(self) -> dict[str, float]:
        return {
            axle.axle_id: axle.center_steering_angle_rad
            for axle in self.axles
        }

    def axle_center_steering_angles_deg(self) -> dict[str, float]:
        return {
            axle_id: math.degrees(angle)
            for axle_id, angle in self.axle_center_steering_angles_rad().items()
        }


def beta_to_reference_radius_mm(beta_rad: float, reference_length_mm: float) -> float | None:
    """Temporary surrogate mapping used by the prototype UI.

    This is not the final trailer articulation model. It simply converts a signed
    slider angle into a signed turning radius so the ideal steering solver can be
    exercised interactively.
    """

    if abs(beta_rad) < 1e-9:
        return None
    return math.copysign(reference_length_mm / math.tan(abs(beta_rad)), beta_rad)


def _solve_wheel_solution(
    wheel: Wheel,
    icr: Point2D | None,
    reference_heading_rad: float,
) -> IdealWheelSolution:
    if icr is None:
        heading = reference_heading_rad
    else:
        heading = tangent_heading_from_icr(
            wheel.center,
            icr,
            forward_axis=heading_vector(reference_heading_rad),
        )
    return IdealWheelSolution(
        wheel_id=wheel.id,
        axle_id=wheel.axle_id,
        side=wheel.side,
        center=wheel.center,
        heading_rad=heading,
        reference_heading_rad=reference_heading_rad,
        steering_angle_rad=normalize_angle(heading - reference_heading_rad),
    )


def _solve_axle_solution(axle: Axle, icr: Point2D | None) -> IdealAxleSolution:
    wheels = axle.wheels()
    is_fixed = not axle.steerable or axle.steering_mode == "FIXED"
    target_icr = None if is_fixed or axle.steering_mode == "USER_DEFINED" else icr

    if axle.steering_mode == "USER_DEFINED" and not is_fixed:
        user_heading = axle.heading_rad + axle.user_defined_steering_angle_rad
        user_angle_rad = axle.user_defined_steering_angle_rad
        wheel_solutions = tuple(
            IdealWheelSolution(
                wheel_id=wheel.id,
                axle_id=wheel.axle_id,
                side=wheel.side,
                center=wheel.center,
                heading_rad=user_heading,
                reference_heading_rad=axle.heading_rad,
                steering_angle_rad=user_angle_rad,
            )
            for wheel in wheels
        )
    else:
        wheel_solutions = tuple(
            _solve_wheel_solution(wheel, target_icr, axle.heading_rad)
            for wheel in wheels
        )
    left_solution = next(solution for solution in wheel_solutions if solution.side == "left")
    right_solution = next(solution for solution in wheel_solutions if solution.side == "right")

    if target_icr is None:
        center_heading = axle.heading_rad
    else:
        center_heading = tangent_heading_from_icr(
            axle.center,
            icr,
            forward_axis=heading_vector(axle.heading_rad),
        )
    if axle.steering_mode == "USER_DEFINED" and not is_fixed:
        center_heading = axle.heading_rad + axle.user_defined_steering_angle_rad
    limits = [
        limit
        for limit in (axle.maximum_steering_angle_deg, axle.steering_stop_deg)
        if limit is not None
    ]
    if limits:
        limit_deg = min(abs(limit) for limit in limits)
        for wheel_solution in wheel_solutions:
            if abs(wheel_solution.steering_angle_deg) > limit_deg + 1e-9:
                raise SteeringLimitExceededError(
                    wheel_solution.steering_angle_deg,
                    limit_deg,
                )
    return IdealAxleSolution(
        axle_id=axle.id,
        center=axle.center,
        center_heading_rad=center_heading,
        left_wheel=left_solution,
        right_wheel=right_solution,
        reference_heading_rad=axle.heading_rad,
        center_steering_angle_rad=normalize_angle(center_heading - axle.heading_rad),
        wheels=wheel_solutions,
    )


def solve_ideal_steering(vehicle: VehicleLayout, icr: Point2D | None) -> IdealSteeringSolution:
    axle_solutions = tuple(_solve_axle_solution(axle, icr) for axle in vehicle.axles)
    wheel_angles_rad: dict[str, float] = {}
    axle_center_angles_rad: dict[str, float] = {}

    for axle_solution in axle_solutions:
        axle_center_angles_rad[axle_solution.axle_id] = axle_solution.center_heading_rad
        for wheel in axle_solution.wheel_solutions:
            wheel_angles_rad[wheel.wheel_id] = wheel.heading_rad

    return IdealSteeringSolution(
        icr=icr,
        axles=axle_solutions,
        wheel_angles_rad=wheel_angles_rad,
        axle_center_angles_rad=axle_center_angles_rad,
    )


def solve_ideal_steering_from_radius(
    vehicle: VehicleLayout,
    turn_radius_mm: float | None,
) -> IdealSteeringSolution:
    if turn_radius_mm is None:
        return solve_ideal_steering(vehicle, None)
    return solve_ideal_steering(vehicle, Point2D(0.0, turn_radius_mm))


def ackermann_expected_angles(
    wheelbase_mm: float,
    track_mm: float,
    turn_radius_mm: float,
) -> tuple[float, float]:
    """Return expected inner/outer angles in radians for a single steering axle.

    Positive radius means a left turn. Negative radius means a right turn.
    """

    if abs(turn_radius_mm) <= track_mm / 2.0:
        raise InvalidGeometryError("Turn radius must exceed half the track width.")

    sign = 1.0 if turn_radius_mm > 0 else -1.0
    radius = abs(turn_radius_mm)
    inner = math.atan(wheelbase_mm / (radius - track_mm / 2.0))
    outer = math.atan(wheelbase_mm / (radius + track_mm / 2.0))
    return sign * inner, sign * outer


def solve_single_axle_ackermann(
    wheelbase_mm: float,
    track_mm: float,
    turn_radius_mm: float,
) -> tuple[float, float]:
    """Convenience wrapper returning the left and right wheel headings."""

    vehicle = VehicleLayout(
        id="single_axle_case",
        name="Single Axle Ackermann Case",
        axles=(
            Axle(
                id="front_axle",
                center=Point2D(wheelbase_mm, 0.0),
                track_mm=track_mm,
            ),
        ),
        body_length_mm=wheelbase_mm + 1000.0,
        body_width_mm=track_mm + 300.0,
    )
    solution = solve_ideal_steering_from_radius(vehicle, turn_radius_mm)
    axle_solution = solution.axles[0]
    return axle_solution.left_wheel.heading_rad, axle_solution.right_wheel.heading_rad


def steering_error_deg(actual_rad: float, ideal_rad: float) -> float:
    return math.degrees(normalize_angle(actual_rad - ideal_rad))


def build_demo_solution(
    beta_deg: float,
    wheelbase_mm: float = 4360.0,
    track_mm: float = 2500.0,
) -> tuple[VehicleLayout, IdealSteeringSolution, float | None]:
    vehicle = build_reference_demo_layout(
        wheelbase_mm=wheelbase_mm,
        track_mm=track_mm,
        articulation_rad=math.radians(beta_deg),
    )
    beta_rad = math.radians(beta_deg)
    radius = beta_to_reference_radius_mm(beta_rad, reference_length_mm=wheelbase_mm)
    solution = solve_ideal_steering_from_radius(vehicle, radius)
    return vehicle, solution, radius
