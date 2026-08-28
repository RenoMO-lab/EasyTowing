from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from .errors import InvalidGeometryError
from .geometry import EPSILON_MM, Point2D


@dataclass(frozen=True, slots=True)
class CircleEnvelope:
    center: Point2D
    radius_mm: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.radius_mm) or self.radius_mm < 0.0:
            raise InvalidGeometryError("Circle envelope radius must be finite and non-negative.")


@dataclass(frozen=True, slots=True)
class CapsuleEnvelope:
    start: Point2D
    end: Point2D
    radius_mm: float

    def __post_init__(self) -> None:
        if (self.end - self.start).length() <= EPSILON_MM:
            raise InvalidGeometryError("Capsule envelope endpoints must be distinct.")
        if not math.isfinite(self.radius_mm) or self.radius_mm < 0.0:
            raise InvalidGeometryError("Capsule envelope radius must be finite and non-negative.")


@dataclass(frozen=True, slots=True)
class PolygonEnvelope:
    points: tuple[Point2D, ...]

    def __post_init__(self) -> None:
        if len(self.points) < 3:
            raise InvalidGeometryError("Polygon envelope requires at least three points.")
        if any(
            (self.points[index] - self.points[(index + 1) % len(self.points)]).length()
            <= EPSILON_MM
            for index in range(len(self.points))
        ):
            raise InvalidGeometryError("Polygon envelope contains a zero-length edge.")
        if len(set(self.points)) != len(self.points):
            raise InvalidGeometryError("Polygon envelope contains duplicate vertices.")
        twice_area = sum(
            point.x_mm * self.points[(index + 1) % len(self.points)].y_mm
            - self.points[(index + 1) % len(self.points)].x_mm * point.y_mm
            for index, point in enumerate(self.points)
        )
        if abs(twice_area) <= EPSILON_MM:
            raise InvalidGeometryError("Polygon envelope must enclose a non-zero area.")

        def cross(start: Point2D, end: Point2D, point: Point2D) -> float:
            edge = end - start
            offset = point - start
            return edge.x_mm * offset.y_mm - edge.y_mm * offset.x_mm

        def on_segment(point: Point2D, start: Point2D, end: Point2D) -> bool:
            return (
                abs(cross(start, end, point)) <= EPSILON_MM
                and min(start.x_mm, end.x_mm) - EPSILON_MM <= point.x_mm <= max(start.x_mm, end.x_mm) + EPSILON_MM
                and min(start.y_mm, end.y_mm) - EPSILON_MM <= point.y_mm <= max(start.y_mm, end.y_mm) + EPSILON_MM
            )

        def intersects(
            start_a: Point2D,
            end_a: Point2D,
            start_b: Point2D,
            end_b: Point2D,
        ) -> bool:
            first = cross(start_a, end_a, start_b)
            second = cross(start_a, end_a, end_b)
            third = cross(start_b, end_b, start_a)
            fourth = cross(start_b, end_b, end_a)
            if (
                ((first > EPSILON_MM and second < -EPSILON_MM)
                 or (first < -EPSILON_MM and second > EPSILON_MM))
                and ((third > EPSILON_MM and fourth < -EPSILON_MM)
                     or (third < -EPSILON_MM and fourth > EPSILON_MM))
            ):
                return True
            return (
                (abs(first) <= EPSILON_MM and on_segment(start_b, start_a, end_a))
                or (abs(second) <= EPSILON_MM and on_segment(end_b, start_a, end_a))
                or (abs(third) <= EPSILON_MM and on_segment(start_a, start_b, end_b))
                or (abs(fourth) <= EPSILON_MM and on_segment(end_a, start_b, end_b))
            )

        edge_count = len(self.points)
        for left_index in range(edge_count):
            left_start = self.points[left_index]
            left_end = self.points[(left_index + 1) % edge_count]
            for right_index in range(left_index + 1, edge_count):
                if right_index == left_index + 1 or (left_index == 0 and right_index == edge_count - 1):
                    continue
                right_start = self.points[right_index]
                right_end = self.points[(right_index + 1) % edge_count]
                if intersects(left_start, left_end, right_start, right_end):
                    raise InvalidGeometryError("Polygon envelope self-intersects.")


Envelope = CircleEnvelope | CapsuleEnvelope | PolygonEnvelope


@dataclass(frozen=True, slots=True)
class CollisionItem:
    id: str
    envelope: Envelope
    margin_mm: float = 0.0
    excluded_pair_ids: tuple[str, ...] = ()
    mounted_body_id: str | None = None
    connectivity_points: tuple[Point2D, ...] = ()

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise InvalidGeometryError("Collision item IDs must not be empty.")
        if not math.isfinite(self.margin_mm) or self.margin_mm < 0.0:
            raise InvalidGeometryError(
                f"Collision item {self.id!r} margin must be finite and non-negative."
            )
        if self.mounted_body_id is not None and not self.mounted_body_id.strip():
            raise InvalidGeometryError(
                f"Collision item {self.id!r} has an empty mounted body ID."
            )
        if any(
            not math.isfinite(point.x_mm) or not math.isfinite(point.y_mm)
            for point in self.connectivity_points
        ):
            raise InvalidGeometryError(
                f"Collision item {self.id!r} has non-finite connectivity points."
            )


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
    if (
        not math.isfinite(width_mm)
        or not math.isfinite(height_mm)
        or width_mm <= 0.0
        or height_mm <= 0.0
    ):
        raise InvalidGeometryError("Rectangle width and height must be positive and finite.")
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


def _segment_intersection_points(
    start_a: Point2D,
    end_a: Point2D,
    start_b: Point2D,
    end_b: Point2D,
) -> tuple[Point2D, ...]:
    """Return distinct centerline intersections, including collinear overlap ends."""

    vector_a = end_a - start_a
    vector_b = end_b - start_b
    offset = start_b - start_a
    determinant = _cross(vector_a, vector_b)
    tolerance = 1e-8

    def unique(points: Iterable[Point2D]) -> tuple[Point2D, ...]:
        result: list[Point2D] = []
        for point in points:
            if not any((point - existing).length() <= tolerance for existing in result):
                result.append(point)
        return tuple(result)

    if abs(determinant) <= tolerance:
        if abs(_cross(offset, vector_a)) > tolerance:
            return ()
        candidates = (
            point
            for point in (start_a, end_a, start_b, end_b)
            if _point_on_segment(point, start_a, end_a)
            and _point_on_segment(point, start_b, end_b)
        )
        return unique(candidates)

    first = _cross(offset, vector_b) / determinant
    second = _cross(offset, vector_a) / determinant
    if -tolerance <= first <= 1.0 + tolerance and -tolerance <= second <= 1.0 + tolerance:
        return (start_a + vector_a.scale(first),)
    return ()


def _same_connectivity_point(left: Point2D, right: Point2D) -> bool:
    return (left - right).length() <= 1e-6


def _connected_contact_is_local(item_a: CollisionItem, item_b: CollisionItem) -> bool:
    """Permit a connected pair only when its sole centerline contact is a joint."""

    if not item_a.connectivity_points or not item_b.connectivity_points:
        # Existing exclusions without point topology (for example a tire and its
        # axle beam or a component mounted inside its own body) remain explicit
        # whole-pair exclusions.
        return True

    common_points = tuple(
        left
        for left in item_a.connectivity_points
        if any(_same_connectivity_point(left, right) for right in item_b.connectivity_points)
    )
    if not common_points:
        return False

    left = item_a.envelope
    right = item_b.envelope
    if isinstance(left, CapsuleEnvelope) and isinstance(right, CapsuleEnvelope):
        intersections = _segment_intersection_points(left.start, left.end, right.start, right.end)
        return len(intersections) == 1 and any(
            _same_connectivity_point(intersections[0], point) for point in common_points
        )
    if isinstance(left, CircleEnvelope) and isinstance(right, CapsuleEnvelope):
        return any(
            _same_connectivity_point(left.center, point) for point in common_points
        ) and _point_on_segment(left.center, right.start, right.end)
    if isinstance(left, CapsuleEnvelope) and isinstance(right, CircleEnvelope):
        return any(
            _same_connectivity_point(right.center, point) for point in common_points
        ) and _point_on_segment(right.center, left.start, left.end)
    if isinstance(left, CircleEnvelope) and isinstance(right, CircleEnvelope):
        return any(
            _same_connectivity_point(left.center, point) for point in common_points
        ) and _same_connectivity_point(left.center, right.center)
    return False


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
    item_ids = [item.id for item in item_tuple]
    if len(item_ids) != len(set(item_ids)):
        raise InvalidGeometryError("Collision item IDs must be unique within a clearance report.")

    def pair_is_excluded(item_a: CollisionItem, item_b: CollisionItem) -> bool:
        excluded = (
            item_b.id in item_a.excluded_pair_ids
            or item_a.id in item_b.excluded_pair_ids
        )
        return excluded and _connected_contact_is_local(item_a, item_b)

    pair_results = tuple(
        clearance_between_items(item_tuple[left_index], item_tuple[right_index])
        for left_index in range(len(item_tuple))
        for right_index in range(left_index + 1, len(item_tuple))
        if not pair_is_excluded(item_tuple[left_index], item_tuple[right_index])
    )
    minimum_pair = min(pair_results, key=lambda pair: pair.clearance_mm, default=None)
    return ClearanceReport(items=item_tuple, pairs=pair_results, minimum_pair=minimum_pair)
