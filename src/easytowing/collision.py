from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from .geometry import EPSILON_MM, Point2D


@dataclass(frozen=True, slots=True)
class CircleEnvelope:
    center: Point2D
    radius_mm: float


@dataclass(frozen=True, slots=True)
class CapsuleEnvelope:
    start: Point2D
    end: Point2D
    radius_mm: float


@dataclass(frozen=True, slots=True)
class PolygonEnvelope:
    points: tuple[Point2D, ...]


Envelope = CircleEnvelope | CapsuleEnvelope | PolygonEnvelope


@dataclass(frozen=True, slots=True)
class CollisionItem:
    id: str
    envelope: Envelope
    margin_mm: float = 0.0


@dataclass(frozen=True, slots=True)
class ClearancePair:
    item_a_id: str
    item_b_id: str
    raw_clearance_mm: float
    required_margin_mm: float
    clearance_mm: float
    overlaps: bool
    violates_margin: bool
    description: str


@dataclass(frozen=True, slots=True)
class ClearanceReport:
    items: tuple[CollisionItem, ...]
    pairs: tuple[ClearancePair, ...]
    minimum_pair: ClearancePair | None

    @property
    def minimum_clearance_mm(self) -> float | None:
        return None if self.minimum_pair is None else self.minimum_pair.clearance_mm

    @property
    def collision_detected(self) -> bool:
        return any(pair.overlaps for pair in self.pairs)

    @property
    def clearance_violation_detected(self) -> bool:
        return any(pair.violates_margin for pair in self.pairs)


def axis_aligned_rectangle(center: Point2D, width_mm: float, height_mm: float) -> PolygonEnvelope:
    half_width = width_mm / 2.0
    half_height = height_mm / 2.0
    return PolygonEnvelope(
        points=(
            Point2D(center.x_mm - half_width, center.y_mm - half_height),
            Point2D(center.x_mm + half_width, center.y_mm - half_height),
            Point2D(center.x_mm + half_width, center.y_mm + half_height),
            Point2D(center.x_mm - half_width, center.y_mm + half_height),
        ),
    )


def _cross(a: Point2D, b: Point2D) -> float:
    return a.x_mm * b.y_mm - a.y_mm * b.x_mm


def _distance(a: Point2D, b: Point2D) -> float:
    return (a - b).length()


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _point_on_segment(point: Point2D, start: Point2D, end: Point2D) -> bool:
    if abs(_cross(end - start, point - start)) > 1e-9:
        return False
    return (
        min(start.x_mm, end.x_mm) - EPSILON_MM <= point.x_mm <= max(start.x_mm, end.x_mm) + EPSILON_MM
        and min(start.y_mm, end.y_mm) - EPSILON_MM <= point.y_mm <= max(start.y_mm, end.y_mm) + EPSILON_MM
    )


def _segment_intersection(start_a: Point2D, end_a: Point2D, start_b: Point2D, end_b: Point2D) -> bool:
    def orientation(a: Point2D, b: Point2D, c: Point2D) -> float:
        return _cross(b - a, c - a)

    o1 = orientation(start_a, end_a, start_b)
    o2 = orientation(start_a, end_a, end_b)
    o3 = orientation(start_b, end_b, start_a)
    o4 = orientation(start_b, end_b, end_a)

    if ((o1 > EPSILON_MM and o2 < -EPSILON_MM) or (o1 < -EPSILON_MM and o2 > EPSILON_MM)) and (
        (o3 > EPSILON_MM and o4 < -EPSILON_MM) or (o3 < -EPSILON_MM and o4 > EPSILON_MM)
    ):
        return True

    if abs(o1) <= EPSILON_MM and _point_on_segment(start_b, start_a, end_a):
        return True
    if abs(o2) <= EPSILON_MM and _point_on_segment(end_b, start_a, end_a):
        return True
    if abs(o3) <= EPSILON_MM and _point_on_segment(start_a, start_b, end_b):
        return True
    if abs(o4) <= EPSILON_MM and _point_on_segment(end_a, start_b, end_b):
        return True

    return False


def _distance_point_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    segment = end - start
    segment_length_sq = segment.dot(segment)
    if segment_length_sq <= EPSILON_MM:
        return _distance(point, start)
    t = _clamp((point - start).dot(segment) / segment_length_sq, 0.0, 1.0)
    closest = start + segment.scale(t)
    return _distance(point, closest)


def _distance_segment_segment(start_a: Point2D, end_a: Point2D, start_b: Point2D, end_b: Point2D) -> float:
    if _segment_intersection(start_a, end_a, start_b, end_b):
        return 0.0
    return min(
        _distance_point_segment(start_a, start_b, end_b),
        _distance_point_segment(end_a, start_b, end_b),
        _distance_point_segment(start_b, start_a, end_a),
        _distance_point_segment(end_b, start_a, end_a),
    )


def _polygon_edges(points: Sequence[Point2D]) -> Iterable[tuple[Point2D, Point2D]]:
    count = len(points)
    for index in range(count):
        yield points[index], points[(index + 1) % count]


def _point_in_polygon(point: Point2D, polygon: PolygonEnvelope) -> bool:
    points = polygon.points
    if len(points) < 3:
        raise ValueError("PolygonEnvelope requires at least three points.")

    for start, end in _polygon_edges(points):
        if _point_on_segment(point, start, end):
            return True

    inside = False
    x = point.x_mm
    y = point.y_mm
    for start, end in _polygon_edges(points):
        y1 = start.y_mm
        y2 = end.y_mm
        x1 = start.x_mm
        x2 = end.x_mm
        intersects = (y1 > y) != (y2 > y)
        if not intersects:
            continue
        x_intersect = x1 + (y - y1) * (x2 - x1) / ((y2 - y1) if abs(y2 - y1) > EPSILON_MM else EPSILON_MM)
        if x_intersect >= x:
            inside = not inside
    return inside


def _circle_overlaps_polygon(circle: CircleEnvelope, polygon: PolygonEnvelope) -> bool:
    if _point_in_polygon(circle.center, polygon):
        return True
    if any(_distance(circle.center, vertex) <= circle.radius_mm + EPSILON_MM for vertex in polygon.points):
        return True
    return any(_distance_point_segment(circle.center, start, end) <= circle.radius_mm + EPSILON_MM for start, end in _polygon_edges(polygon.points))


def _capsule_overlaps_polygon(capsule: CapsuleEnvelope, polygon: PolygonEnvelope) -> bool:
    if _point_in_polygon(capsule.start, polygon) or _point_in_polygon(capsule.end, polygon):
        return True
    if any(_distance_point_segment(vertex, capsule.start, capsule.end) <= capsule.radius_mm + EPSILON_MM for vertex in polygon.points):
        return True
    return any(_distance_segment_segment(capsule.start, capsule.end, start, end) <= capsule.radius_mm + EPSILON_MM for start, end in _polygon_edges(polygon.points))


def _polygon_overlaps_polygon(left: PolygonEnvelope, right: PolygonEnvelope) -> bool:
    if any(_segment_intersection(a1, a2, b1, b2) for a1, a2 in _polygon_edges(left.points) for b1, b2 in _polygon_edges(right.points)):
        return True
    if _point_in_polygon(left.points[0], right):
        return True
    if _point_in_polygon(right.points[0], left):
        return True
    return False


def _clearance_circle_circle(left: CircleEnvelope, right: CircleEnvelope) -> tuple[float, bool, str]:
    boundary_clearance = _distance(left.center, right.center) - left.radius_mm - right.radius_mm
    overlaps = boundary_clearance <= 0.0
    return boundary_clearance, overlaps, "circle-circle"


def _clearance_capsule_capsule(left: CapsuleEnvelope, right: CapsuleEnvelope) -> tuple[float, bool, str]:
    boundary_clearance = _distance_segment_segment(left.start, left.end, right.start, right.end) - left.radius_mm - right.radius_mm
    overlaps = boundary_clearance <= 0.0
    return boundary_clearance, overlaps, "capsule-capsule"


def _clearance_circle_capsule(circle: CircleEnvelope, capsule: CapsuleEnvelope) -> tuple[float, bool, str]:
    boundary_clearance = _distance_point_segment(circle.center, capsule.start, capsule.end) - circle.radius_mm - capsule.radius_mm
    overlaps = boundary_clearance <= 0.0
    return boundary_clearance, overlaps, "circle-capsule"


def _clearance_polygon_polygon(left: PolygonEnvelope, right: PolygonEnvelope) -> tuple[float, bool, str]:
    boundary_clearance = min(
        _distance_segment_segment(a1, a2, b1, b2)
        for a1, a2 in _polygon_edges(left.points)
        for b1, b2 in _polygon_edges(right.points)
    )
    overlaps = _polygon_overlaps_polygon(left, right)
    if overlaps:
        boundary_clearance = -max(boundary_clearance, EPSILON_MM)
    return boundary_clearance, overlaps, "polygon-polygon"


def _clearance_circle_polygon(circle: CircleEnvelope, polygon: PolygonEnvelope) -> tuple[float, bool, str]:
    boundary_clearance = min(
        _distance_point_segment(circle.center, start, end)
        for start, end in _polygon_edges(polygon.points)
    ) - circle.radius_mm
    overlaps = _circle_overlaps_polygon(circle, polygon)
    if overlaps:
        boundary_clearance = -max(abs(boundary_clearance), EPSILON_MM)
    return boundary_clearance, overlaps, "circle-polygon"


def _clearance_capsule_polygon(capsule: CapsuleEnvelope, polygon: PolygonEnvelope) -> tuple[float, bool, str]:
    boundary_clearance = min(
        _distance_segment_segment(capsule.start, capsule.end, start, end)
        for start, end in _polygon_edges(polygon.points)
    ) - capsule.radius_mm
    overlaps = _capsule_overlaps_polygon(capsule, polygon)
    if overlaps:
        boundary_clearance = -max(abs(boundary_clearance), EPSILON_MM)
    return boundary_clearance, overlaps, "capsule-polygon"


def _clearance_between_envelopes(left: Envelope, right: Envelope) -> tuple[float, bool, str]:
    if isinstance(left, CircleEnvelope) and isinstance(right, CircleEnvelope):
        return _clearance_circle_circle(left, right)
    if isinstance(left, CapsuleEnvelope) and isinstance(right, CapsuleEnvelope):
        return _clearance_capsule_capsule(left, right)
    if isinstance(left, CircleEnvelope) and isinstance(right, CapsuleEnvelope):
        return _clearance_circle_capsule(left, right)
    if isinstance(left, CapsuleEnvelope) and isinstance(right, CircleEnvelope):
        clearance, overlaps, kind = _clearance_circle_capsule(right, left)
        return clearance, overlaps, kind
    if isinstance(left, PolygonEnvelope) and isinstance(right, PolygonEnvelope):
        return _clearance_polygon_polygon(left, right)
    if isinstance(left, CircleEnvelope) and isinstance(right, PolygonEnvelope):
        return _clearance_circle_polygon(left, right)
    if isinstance(left, PolygonEnvelope) and isinstance(right, CircleEnvelope):
        clearance, overlaps, kind = _clearance_circle_polygon(right, left)
        return clearance, overlaps, kind
    if isinstance(left, CapsuleEnvelope) and isinstance(right, PolygonEnvelope):
        return _clearance_capsule_polygon(left, right)
    if isinstance(left, PolygonEnvelope) and isinstance(right, CapsuleEnvelope):
        clearance, overlaps, kind = _clearance_capsule_polygon(right, left)
        return clearance, overlaps, kind
    raise TypeError(f"Unsupported envelope pair: {type(left)!r}, {type(right)!r}")


def clearance_between_items(item_a: CollisionItem, item_b: CollisionItem) -> ClearancePair:
    raw_clearance_mm, overlaps, description = _clearance_between_envelopes(item_a.envelope, item_b.envelope)
    margin_mm = item_a.margin_mm + item_b.margin_mm
    clearance_mm = raw_clearance_mm - margin_mm
    violates_margin = clearance_mm <= 0.0
    return ClearancePair(
        item_a_id=item_a.id,
        item_b_id=item_b.id,
        raw_clearance_mm=raw_clearance_mm,
        required_margin_mm=margin_mm,
        clearance_mm=clearance_mm,
        overlaps=overlaps,
        violates_margin=violates_margin,
        description=description,
    )


def analyze_clearance(items: Iterable[CollisionItem]) -> ClearanceReport:
    item_tuple = tuple(items)
    pair_results = tuple(
        clearance_between_items(item_tuple[left_index], item_tuple[right_index])
        for left_index in range(len(item_tuple))
        for right_index in range(left_index + 1, len(item_tuple))
    )
    minimum_pair = min(pair_results, key=lambda pair: pair.clearance_mm, default=None)
    return ClearanceReport(items=item_tuple, pairs=pair_results, minimum_pair=minimum_pair)

