from __future__ import annotations

from dataclasses import dataclass
import math

from .errors import InvalidGeometryError
from .geometry import Point2D, normalize_angle, tangent_heading_from_icr
from .model import Axle, VehicleLayout, Wheel, build_reference_demo_layout


@dataclass(frozen=True, slots=True)
class IdealWheelSolution:
    wheel_id: str
    axle_id: str
    side: str
    center: Point2D
    heading_rad: float

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.heading_rad)


@dataclass(frozen=True, slots=True)
class IdealAxleSolution:
    axle_id: str
    center: Point2D
    center_heading_rad: float
    left_wheel: IdealWheelSolution
    right_wheel: IdealWheelSolution

    @property
    def center_heading_deg(self) -> float:
        return math.degrees(self.center_heading_rad)


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


def beta_to_reference_radius_mm(beta_rad: float, reference_length_mm: float) -> float | None:
    """Temporary surrogate mapping used by the prototype UI.

    This is not the final trailer articulation model. It simply converts a signed
    slider angle into a signed turning radius so the ideal steering solver can be
    exercised interactively.
    """

    if abs(beta_rad) < 1e-9:
        return None
    return math.copysign(reference_length_mm / math.tan(abs(beta_rad)), beta_rad)


def _solve_wheel_solution(wheel: Wheel, icr: Point2D | None) -> IdealWheelSolution:
    if icr is None:
        heading = 0.0
    else:
        heading = tangent_heading_from_icr(wheel.center, icr)
    return IdealWheelSolution(
        wheel_id=wheel.id,
        axle_id=wheel.axle_id,
        side=wheel.side,
        center=wheel.center,
        heading_rad=heading,
    )


def _solve_axle_solution(axle: Axle, icr: Point2D | None) -> IdealAxleSolution:
    left_wheel, right_wheel = axle.wheels()
    left_solution = _solve_wheel_solution(left_wheel, icr)
    right_solution = _solve_wheel_solution(right_wheel, icr)
    if icr is None:
        center_heading = 0.0
    else:
        center_heading = tangent_heading_from_icr(axle.center, icr)
    return IdealAxleSolution(
        axle_id=axle.id,
        center=axle.center,
        center_heading_rad=center_heading,
        left_wheel=left_solution,
        right_wheel=right_solution,
    )


def solve_ideal_steering(vehicle: VehicleLayout, icr: Point2D | None) -> IdealSteeringSolution:
    axle_solutions = tuple(_solve_axle_solution(axle, icr) for axle in vehicle.axles)
    wheel_angles_rad: dict[str, float] = {}
    axle_center_angles_rad: dict[str, float] = {}

    for axle_solution in axle_solutions:
        axle_center_angles_rad[axle_solution.axle_id] = axle_solution.center_heading_rad
        wheel_angles_rad[axle_solution.left_wheel.wheel_id] = axle_solution.left_wheel.heading_rad
        wheel_angles_rad[axle_solution.right_wheel.wheel_id] = axle_solution.right_wheel.heading_rad

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


def build_demo_solution(beta_deg: float) -> tuple[VehicleLayout, IdealSteeringSolution, float | None]:
    vehicle = build_reference_demo_layout()
    beta_rad = math.radians(beta_deg)
    radius = beta_to_reference_radius_mm(beta_rad, reference_length_mm=4360.0)
    solution = solve_ideal_steering_from_radius(vehicle, radius)
    return vehicle, solution, radius

