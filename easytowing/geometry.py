from __future__ import annotations

from dataclasses import dataclass
import math

from .errors import InvalidGeometryError

EPSILON_MM = 1e-9


@dataclass(frozen=True, slots=True)
class Point2D:
    x_mm: float
    y_mm: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.x_mm) or not math.isfinite(self.y_mm):
            raise InvalidGeometryError("Point coordinates must be finite.")

    def __add__(self, other: "Point2D") -> "Point2D":
        return Point2D(self.x_mm + other.x_mm, self.y_mm + other.y_mm)

    def __sub__(self, other: "Point2D") -> "Point2D":
        return Point2D(self.x_mm - other.x_mm, self.y_mm - other.y_mm)

    def scale(self, factor: float) -> "Point2D":
        return Point2D(self.x_mm * factor, self.y_mm * factor)

    def dot(self, other: "Point2D") -> float:
        return self.x_mm * other.x_mm + self.y_mm * other.y_mm

    def length(self) -> float:
        return math.hypot(self.x_mm, self.y_mm)

    def normalized(self) -> "Point2D":
        magnitude = self.length()
        if magnitude <= EPSILON_MM:
            raise ValueError("Cannot normalize a zero-length vector.")
        return Point2D(self.x_mm / magnitude, self.y_mm / magnitude)

    def rotated_ccw(self, angle_rad: float) -> "Point2D":
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)
        return Point2D(
            self.x_mm * cosine - self.y_mm * sine,
            self.x_mm * sine + self.y_mm * cosine,
        )

    def to_tuple(self) -> tuple[float, float]:
        return (self.x_mm, self.y_mm)


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def tangent_heading_from_icr(
    point: Point2D,
    icr: Point2D,
    forward_axis: Point2D | None = None,
) -> float:
    """Return the tangent heading at `point` for a circle centered on `icr`.

    The returned heading is chosen so that it points most closely along the
    nominal forward axis. For the prototype, the nominal forward axis is +X.
    """

    axis = forward_axis or Point2D(1.0, 0.0)
    radius = point - icr
    if radius.length() <= EPSILON_MM:
        raise ValueError("Wheel center cannot coincide with the ICR.")

    candidate_a = Point2D(-radius.y_mm, radius.x_mm)
    candidate_b = Point2D(radius.y_mm, -radius.x_mm)

    chosen = candidate_a if candidate_a.dot(axis) >= candidate_b.dot(axis) else candidate_b
    return normalize_angle(math.atan2(chosen.y_mm, chosen.x_mm))


def heading_vector(angle_rad: float) -> Point2D:
    return Point2D(math.cos(angle_rad), math.sin(angle_rad))
