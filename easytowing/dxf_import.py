from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from typing import Iterable
import math

from .geometry import Point2D
from .model import Axle, VehicleLayout


SUPPORTED_ENTITY_TYPES = {
    "LINE",
    "CIRCLE",
    "TEXT",
    "POINT",
    "ARC",
    "LWPOLYLINE",
    "POLYLINE",
    "INSERT",
}

DXF_ROLE_OPTIONS: tuple[tuple[str, str], ...] = (
    ("", "Unassigned"),
    ("body_envelope", "Body envelope"),
    ("chassis_outline", "Chassis outline"),
    ("axle_centerline", "Axle centerline"),
    ("linkage_segment", "Linkage segment"),
    ("steering_arm", "Steering arm"),
    ("tie_rod", "Tie rod"),
    ("bell_crank", "Bell crank"),
    ("articulation_pivot", "Articulation pivot"),
    ("pivot", "Pivot"),
    ("wheel_marker", "Wheel marker"),
    ("icr_marker", "ICR marker"),
    ("annotation", "Annotation"),
    ("reference_point", "Reference point"),
    ("arc_reference", "Arc reference"),
    ("block_reference", "Block reference"),
    ("drawbar_or_frame", "Drawbar / frame"),
)
_PARAMETRIC_SNAP_TOLERANCE_MM = 0.01
_DXF_ROLE_VALUES = {value for value, _label in DXF_ROLE_OPTIONS}


@dataclass(frozen=True, slots=True)
class RawDxfEntity:
    entity_type: str
    groups: tuple[tuple[str, str], ...]
    vertices: tuple["RawDxfEntity", ...] = ()


@dataclass(frozen=True, slots=True)
class DxfImportedEntity:
    index: int
    entity_type: str
    layer: str | None
    summary: str
    bounds_mm: tuple[float, float, float, float] | None
    suggested_role: str | None
    assigned_role: str | None
    confidence: float
    reason: str
    geometry: dict[str, object]


@dataclass(frozen=True, slots=True)
class DxfImportReport:
    source_name: str
    entity_count: int
    supported_entity_count: int
    unsupported_entity_count: int
    counts_by_type: dict[str, int]
    counts_by_layer: dict[str, int]
    bounds_mm: tuple[float, float, float, float] | None
    entities: tuple[DxfImportedEntity, ...]
    warnings: tuple[str, ...]
    reconstructed_vehicle: VehicleLayout | None
    parametric_mechanism: "DxfParametricMechanism | None" = None


@dataclass(frozen=True, slots=True)
class DxfParametricReferencePoint:
    id: str
    point: Point2D
    source_entity_index: int
    role: str


@dataclass(frozen=True, slots=True)
class DxfParametricComponent:
    id: str
    role: str
    source_entity_index: int
    point_a_id: str | None = None
    point_b_id: str | None = None
    center_id: str | None = None
    length_mm: float | None = None
    radius_mm: float | None = None


@dataclass(frozen=True, slots=True)
class DxfParametricMechanism:
    source_name: str
    reference_points: tuple[DxfParametricReferencePoint, ...]
    components: tuple[DxfParametricComponent, ...]
    warnings: tuple[str, ...] = ()


def _point_payload(point: Point2D) -> dict[str, float]:
    return {"x_mm": point.x_mm, "y_mm": point.y_mm}


def _serialize_vehicle(vehicle: VehicleLayout) -> dict[str, object]:
    return {
        "id": vehicle.id,
        "name": vehicle.name,
        "body_length_mm": vehicle.body_length_mm,
        "body_width_mm": vehicle.body_width_mm,
        "origin": _point_payload(vehicle.origin),
        "axle_span_mm": vehicle.axle_span_mm(),
        "axles": [
            {
                "id": axle.id,
                "center": _point_payload(axle.center),
                "track_mm": axle.track_mm,
                "steerable": axle.steerable,
                "steering_mode": axle.steering_mode,
            }
            for axle in vehicle.axles
        ],
    }


def _serialize_geometry(value: object) -> object:
    if isinstance(value, Point2D):
        return _point_payload(value)
    if isinstance(value, dict):
        return {key: _serialize_geometry(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_serialize_geometry(item) for item in value]
    if isinstance(value, tuple):
        return [_serialize_geometry(item) for item in value]
    return value


def _resolved_role(entity: DxfImportedEntity) -> str | None:
    if entity.assigned_role is not None:
        return entity.assigned_role
    return entity.suggested_role


def _serialize_parametric_mechanism(mechanism: DxfParametricMechanism) -> dict[str, object]:
    return {
        "source_name": mechanism.source_name,
        "reference_points": [
            {
                "id": point.id,
                "point": _point_payload(point.point),
                "source_entity_index": point.source_entity_index,
                "role": point.role,
            }
            for point in mechanism.reference_points
        ],
        "components": [
            {
                "id": component.id,
                "role": component.role,
                "source_entity_index": component.source_entity_index,
                "point_a_id": component.point_a_id,
                "point_b_id": component.point_b_id,
                "center_id": component.center_id,
                "length_mm": component.length_mm,
                "radius_mm": component.radius_mm,
            }
            for component in mechanism.components
        ],
        "warnings": list(mechanism.warnings),
    }


def serialize_dxf_import_report(report: DxfImportReport) -> dict[str, object]:
    return {
        "source_name": report.source_name,
        "entity_count": report.entity_count,
        "supported_entity_count": report.supported_entity_count,
        "unsupported_entity_count": report.unsupported_entity_count,
        "counts_by_type": report.counts_by_type,
        "counts_by_layer": report.counts_by_layer,
        "bounds_mm": None
        if report.bounds_mm is None
        else {
            "min_x_mm": report.bounds_mm[0],
            "max_x_mm": report.bounds_mm[1],
            "min_y_mm": report.bounds_mm[2],
            "max_y_mm": report.bounds_mm[3],
        },
        "entities": [
            {
                "index": entity.index,
                "entity_type": entity.entity_type,
                "layer": entity.layer,
                "summary": entity.summary,
                "bounds_mm": None
                if entity.bounds_mm is None
                else {
                    "min_x_mm": entity.bounds_mm[0],
                    "max_x_mm": entity.bounds_mm[1],
                    "min_y_mm": entity.bounds_mm[2],
                    "max_y_mm": entity.bounds_mm[3],
                },
                "suggested_role": entity.suggested_role,
                "assigned_role": entity.assigned_role,
                "confidence": entity.confidence,
                "reason": entity.reason,
                "geometry": _serialize_geometry(entity.geometry),
            }
            for entity in report.entities
        ],
        "warnings": list(report.warnings),
        "role_options": [
            {"value": value, "label": label}
            for value, label in DXF_ROLE_OPTIONS
        ],
        "reconstructed_vehicle": None if report.reconstructed_vehicle is None else _serialize_vehicle(report.reconstructed_vehicle),
        "parametric_mechanism": (
            None
            if report.parametric_mechanism is None
            else _serialize_parametric_mechanism(report.parametric_mechanism)
        ),
    }


def _pair_tokens(lines: Iterable[str]) -> list[tuple[str, str]]:
    raw_lines = [line.rstrip("\r\n") for line in lines if line.strip() != ""]
    if len(raw_lines) % 2 != 0:
        raw_lines = raw_lines[:-1]
    pairs: list[tuple[str, str]] = []
    for index in range(0, len(raw_lines), 2):
        pairs.append((raw_lines[index].strip(), raw_lines[index + 1].strip()))
    return pairs


def _read_section_name(pairs: list[tuple[str, str]], index: int) -> tuple[str | None, int]:
    if index + 1 >= len(pairs):
        return None, index
    code, value = pairs[index + 1]
    if code != "2":
        return None, index
    return value, index + 1


def _collect_raw_entities(pairs: list[tuple[str, str]]) -> tuple[list[RawDxfEntity], list[str]]:
    raw_entities: list[RawDxfEntity] = []
    warnings: list[str] = []
    inside_entities = False
    current_type: str | None = None
    current_groups: list[tuple[str, str]] = []

    index = 0
    while index < len(pairs):
        code, value = pairs[index]
        if code == "0" and value == "SECTION":
            section_name, section_index = _read_section_name(pairs, index)
            if section_name is None:
                warnings.append("Encountered a DXF SECTION without a valid name.")
                index += 1
                continue
            inside_entities = section_name.upper() == "ENTITIES"
            current_type = None
            current_groups = []
            index = section_index + 1
            continue
        if code == "0" and value == "ENDSEC":
            if inside_entities and current_type is not None:
                raw_entities.append(RawDxfEntity(current_type, tuple(current_groups)))
            inside_entities = False
            current_type = None
            current_groups = []
            index += 1
            continue
        if not inside_entities:
            index += 1
            continue
        if code == "0":
            if current_type is not None:
                raw_entities.append(RawDxfEntity(current_type, tuple(current_groups)))
            current_type = value
            current_groups = []
        elif current_type is not None:
            current_groups.append((code, value))
        index += 1

    if inside_entities and current_type is not None:
        raw_entities.append(RawDxfEntity(current_type, tuple(current_groups)))

    return raw_entities, warnings


def _attach_polyline_vertices(records: list[RawDxfEntity]) -> tuple[list[RawDxfEntity], list[str]]:
    combined: list[RawDxfEntity] = []
    warnings: list[str] = []
    index = 0
    while index < len(records):
        record = records[index]
        entity_type = record.entity_type.upper()
        if entity_type in {"VERTEX", "SEQEND"}:
            index += 1
            continue
        if entity_type != "POLYLINE":
            combined.append(record)
            index += 1
            continue

        vertices: list[RawDxfEntity] = []
        probe = index + 1
        while probe < len(records):
            probe_type = records[probe].entity_type.upper()
            if probe_type == "VERTEX":
                vertices.append(records[probe])
                probe += 1
                continue
            if probe_type == "SEQEND":
                probe += 1
                break
            break

        if not vertices:
            warnings.append("POLYLINE entity without vertices was encountered.")
        combined.append(RawDxfEntity("POLYLINE", record.groups, tuple(vertices)))
        index = probe

    return combined, warnings


def _group_values(groups: tuple[tuple[str, str], ...], code: str) -> list[str]:
    return [value for group_code, value in groups if group_code == code]


def _first_float(groups: tuple[tuple[str, str], ...], code: str, default: float = 0.0) -> float:
    values = _group_values(groups, code)
    if not values:
        return default
    try:
        return float(values[-1])
    except ValueError:
        return default


def _first_string(groups: tuple[tuple[str, str], ...], code: str, default: str = "") -> str:
    values = _group_values(groups, code)
    if not values:
        return default
    return values[-1]


def _point_from_groups(groups: tuple[tuple[str, str], ...], x_code: str, y_code: str) -> Point2D:
    return Point2D(_first_float(groups, x_code), _first_float(groups, y_code))


def _points_from_lwpolyline(groups: tuple[tuple[str, str], ...]) -> tuple[Point2D, ...]:
    x_values = _group_values(groups, "10")
    y_values = _group_values(groups, "20")
    return tuple(Point2D(float(x), float(y)) for x, y in zip(x_values, y_values))


def _points_from_polyline_vertices(vertices: tuple[RawDxfEntity, ...]) -> tuple[Point2D, ...]:
    return tuple(_point_from_groups(vertex.groups, "10", "20") for vertex in vertices)


def _bounds_from_points(points: Iterable[Point2D]) -> tuple[float, float, float, float] | None:
    point_list = list(points)
    if not point_list:
        return None
    return (
        min(point.x_mm for point in point_list),
        max(point.x_mm for point in point_list),
        min(point.y_mm for point in point_list),
        max(point.y_mm for point in point_list),
    )


def _combine_bounds(bounds: Iterable[tuple[float, float, float, float] | None]) -> tuple[float, float, float, float] | None:
    filtered = [bound for bound in bounds if bound is not None]
    if not filtered:
        return None
    return (
        min(bound[0] for bound in filtered),
        max(bound[1] for bound in filtered),
        min(bound[2] for bound in filtered),
        max(bound[3] for bound in filtered),
    )


def _geometry_summary(record: RawDxfEntity) -> tuple[str, dict[str, object], tuple[float, float, float, float] | None]:
    entity_type = record.entity_type.upper()
    if entity_type == "LINE":
        start = _point_from_groups(record.groups, "10", "20")
        end = _point_from_groups(record.groups, "11", "21")
        length_mm = (end - start).length()
        geometry = {"start": start, "end": end, "length_mm": length_mm}
        layer = _first_string(record.groups, "8", "0")
        return (
            f"LINE layer={layer} length={length_mm:.1f} mm",
            geometry,
            _bounds_from_points((start, end)),
        )
    if entity_type == "CIRCLE":
        center = _point_from_groups(record.groups, "10", "20")
        radius_mm = _first_float(record.groups, "40")
        geometry = {"center": center, "radius_mm": radius_mm}
        return (
            f"CIRCLE r={radius_mm:.1f} mm",
            geometry,
            (center.x_mm - radius_mm, center.x_mm + radius_mm, center.y_mm - radius_mm, center.y_mm + radius_mm),
        )
    if entity_type == "TEXT":
        insert = _point_from_groups(record.groups, "10", "20")
        text = _first_string(record.groups, "1", "")
        height_mm = _first_float(record.groups, "40")
        geometry = {"insert": insert, "text": text, "height_mm": height_mm}
        return (f"TEXT {text[:32]}", geometry, _bounds_from_points((insert,)))
    if entity_type == "POINT":
        point = _point_from_groups(record.groups, "10", "20")
        geometry = {"point": point}
        return ("POINT", geometry, _bounds_from_points((point,)))
    if entity_type == "ARC":
        center = _point_from_groups(record.groups, "10", "20")
        radius_mm = _first_float(record.groups, "40")
        geometry = {
            "center": center,
            "radius_mm": radius_mm,
            "start_angle_deg": _first_float(record.groups, "50"),
            "end_angle_deg": _first_float(record.groups, "51"),
        }
        return (
            f"ARC r={radius_mm:.1f} mm",
            geometry,
            (center.x_mm - radius_mm, center.x_mm + radius_mm, center.y_mm - radius_mm, center.y_mm + radius_mm),
        )
    if entity_type == "LWPOLYLINE":
        points = _points_from_lwpolyline(record.groups)
        closed = bool(int(_first_float(record.groups, "70", 0.0)) & 1)
        geometry = {"points": points, "closed": closed, "vertex_count": len(points)}
        return (
            f"LWPOLYLINE {len(points)} vertices{' closed' if closed else ''}",
            geometry,
            _bounds_from_points(points),
        )
    if entity_type == "POLYLINE":
        points = _points_from_polyline_vertices(record.vertices)
        closed = bool(int(_first_float(record.groups, "70", 0.0)) & 1)
        geometry = {"points": points, "closed": closed, "vertex_count": len(points)}
        return (
            f"POLYLINE {len(points)} vertices{' closed' if closed else ''}",
            geometry,
            _bounds_from_points(points),
        )
    if entity_type == "INSERT":
        insert = _point_from_groups(record.groups, "10", "20")
        block_name = _first_string(record.groups, "2", "")
        geometry = {"insert": insert, "block_name": block_name}
        return (f"INSERT {block_name}", geometry, _bounds_from_points((insert,)))

    geometry = {"raw_groups": list(record.groups)}
    return (entity_type, geometry, None)


def _suggest_role(entity_type: str, layer: str | None, geometry: dict[str, object]) -> tuple[str | None, float, str]:
    layer_upper = (layer or "").upper()
    entity_type = entity_type.upper()

    if entity_type in {"LWPOLYLINE", "POLYLINE"}:
        points = geometry.get("points", ())
        closed = bool(geometry.get("closed", False))
        if isinstance(points, tuple) and closed and len(points) >= 4 and "BODY" in layer_upper:
            return "body_envelope", 0.98, "Closed polygon on BODY layer."
        if isinstance(points, tuple) and closed and len(points) >= 4:
            return "chassis_outline", 0.82, "Closed polygon is likely a vehicle outline."

    if entity_type == "LINE":
        start = geometry.get("start")
        end = geometry.get("end")
        if isinstance(start, Point2D) and isinstance(end, Point2D):
            dx = abs(end.x_mm - start.x_mm)
            dy = abs(end.y_mm - start.y_mm)
            length_mm = float(geometry.get("length_mm", 0.0))
            if "AXLE" in layer_upper and dx <= max(30.0, dy * 0.1) and length_mm >= 200.0:
                return "axle_centerline", 0.97, "Vertical line on AXLE layer matches a centerline."
            if "BASELINE" in layer_upper or "OPTIMIZED" in layer_upper or "IDEAL" in layer_upper:
                return "linkage_segment", 0.88, "Line on a linkage-related layer."
            if dx > dy and length_mm >= 600.0:
                return "drawbar_or_frame", 0.66, "Long horizontal line may be a drawbar or frame member."

    if entity_type == "CIRCLE":
        radius_mm = float(geometry.get("radius_mm", 0.0))
        if "PIVOT" in layer_upper:
            return "pivot", 0.95, "Circle on PIVOT layer."
        if "AXLE" in layer_upper and radius_mm <= 80.0:
            return "wheel_marker", 0.86, "Small circle on AXLE layer."
        if "ICR" in layer_upper:
            return "icr_marker", 0.94, "Circle on ICR layer."

    if entity_type == "TEXT":
        return "annotation", 0.9, "Text annotation."

    if entity_type == "POINT":
        if "PIVOT" in layer_upper:
            return "pivot", 0.8, "Point on PIVOT layer."
        return "reference_point", 0.55, "Standalone point reference."

    if entity_type == "ARC":
        return "arc_reference", 0.6, "Arc geometry that could be an alignment reference."

    if entity_type == "INSERT":
        return "block_reference", 0.55, "Block insert reference."

    return None, 0.0, "No heuristic assignment available."


def _geometry_point_pair(entity: DxfImportedEntity) -> tuple[Point2D, Point2D] | None:
    start = entity.geometry.get("start")
    end = entity.geometry.get("end")
    if isinstance(start, Point2D) and isinstance(end, Point2D):
        return start, end
    points = entity.geometry.get("points")
    if isinstance(points, tuple) and len(points) >= 2 and all(isinstance(point, Point2D) for point in points):
        return points[0], points[-1]  # type: ignore[return-value]
    return None


def _geometry_center(entity: DxfImportedEntity) -> Point2D | None:
    for key in ("center", "point", "insert"):
        point = entity.geometry.get(key)
        if isinstance(point, Point2D):
            return point
    return None


def _build_parametric_mechanism(
    entities: list[DxfImportedEntity],
    source_name: str,
) -> DxfParametricMechanism | None:
    assigned_entities = [entity for entity in entities if _resolved_role(entity)]
    if not assigned_entities:
        return None

    reference_points: list[DxfParametricReferencePoint] = []
    warnings: list[str] = []

    def reference_point_id(point: Point2D, entity: DxfImportedEntity, role: str, suffix: str) -> str:
        for existing in reference_points:
            if (existing.point - point).length() <= _PARAMETRIC_SNAP_TOLERANCE_MM:
                return existing.id
        point_id = f"point_{entity.index}_{suffix}"
        reference_points.append(
            DxfParametricReferencePoint(
                id=point_id,
                point=point,
                source_entity_index=entity.index,
                role=role,
            )
        )
        return point_id

    def segment_intersection(
        first: tuple[Point2D, Point2D],
        second: tuple[Point2D, Point2D],
    ) -> Point2D | None:
        start_a, end_a = first
        start_b, end_b = second
        vector_a = end_a - start_a
        vector_b = end_b - start_b
        denominator = vector_a.x_mm * vector_b.y_mm - vector_a.y_mm * vector_b.x_mm
        if abs(denominator) <= 1e-9:
            return None
        offset = start_b - start_a
        t = (offset.x_mm * vector_b.y_mm - offset.y_mm * vector_b.x_mm) / denominator
        u = (offset.x_mm * vector_a.y_mm - offset.y_mm * vector_a.x_mm) / denominator
        if not (-_PARAMETRIC_SNAP_TOLERANCE_MM <= t <= 1.0 + _PARAMETRIC_SNAP_TOLERANCE_MM):
            return None
        if not (-_PARAMETRIC_SNAP_TOLERANCE_MM <= u <= 1.0 + _PARAMETRIC_SNAP_TOLERANCE_MM):
            return None
        return start_a + vector_a.scale(t)

    # Seed pivot centers first so line endpoints snap to named mechanical points.
    for entity in assigned_entities:
        role = _resolved_role(entity)
        if role not in {"pivot", "articulation_pivot", "bell_crank", "wheel_marker"}:
            continue
        center = _geometry_center(entity)
        if center is not None:
            reference_point_id(center, entity, role, "center")

    line_entities = [
        entity
        for entity in assigned_entities
        if _resolved_role(entity) in {
            "axle_centerline",
            "linkage_segment",
            "steering_arm",
            "tie_rod",
            "drawbar_or_frame",
        }
        and _geometry_point_pair(entity) is not None
    ]
    for left_index, left_entity in enumerate(line_entities):
        left_pair = _geometry_point_pair(left_entity)
        assert left_pair is not None
        for right_entity in line_entities[left_index + 1 :]:
            right_pair = _geometry_point_pair(right_entity)
            assert right_pair is not None
            intersection = segment_intersection(left_pair, right_pair)
            if intersection is not None:
                reference_point_id(
                    intersection,
                    left_entity,
                    "intersection",
                    f"intersection_{right_entity.index}",
                )

    components: list[DxfParametricComponent] = []
    line_roles = {
        "axle_centerline",
        "linkage_segment",
        "steering_arm",
        "tie_rod",
        "drawbar_or_frame",
    }
    for entity in assigned_entities:
        role = _resolved_role(entity)
        if role in line_roles:
            pair = _geometry_point_pair(entity)
            if pair is None:
                warnings.append(f"Entity {entity.index} assigned as {role} has no usable endpoints.")
                continue
            point_a_id = reference_point_id(pair[0], entity, role, "a")
            point_b_id = reference_point_id(pair[1], entity, role, "b")
            components.append(
                DxfParametricComponent(
                    id=f"component_{entity.index}",
                    role=role or "unassigned",
                    source_entity_index=entity.index,
                    point_a_id=point_a_id,
                    point_b_id=point_b_id,
                    length_mm=(pair[1] - pair[0]).length(),
                )
            )
            continue

        if role in {"pivot", "articulation_pivot", "bell_crank", "wheel_marker"}:
            center = _geometry_center(entity)
            if center is None:
                warnings.append(f"Entity {entity.index} assigned as {role} has no usable center.")
                continue
            center_id = reference_point_id(center, entity, role or "reference", "center")
            radius = entity.geometry.get("radius_mm")
            components.append(
                DxfParametricComponent(
                    id=f"component_{entity.index}",
                    role=role or "reference",
                    source_entity_index=entity.index,
                    center_id=center_id,
                    radius_mm=float(radius) if isinstance(radius, (int, float)) else None,
                )
            )

    if not components and not reference_points:
        return None
    return DxfParametricMechanism(
        source_name=source_name,
        reference_points=tuple(reference_points),
        components=tuple(components),
        warnings=tuple(warnings),
    )


def _entity_center(entity: DxfImportedEntity) -> Point2D | None:
    """Return a geometric center for entities that can define an axle reference."""
    pair = _geometry_point_pair(entity)
    if pair is not None:
        return Point2D(
            (pair[0].x_mm + pair[1].x_mm) / 2.0,
            (pair[0].y_mm + pair[1].y_mm) / 2.0,
        )
    return _geometry_center(entity)


def _reconstruct_vehicle_layout(entities: list[DxfImportedEntity], source_name: str) -> VehicleLayout | None:
    body_entity = next((entity for entity in entities if _resolved_role(entity) in {"body_envelope", "chassis_outline"}), None)
    axle_entities = [entity for entity in entities if _resolved_role(entity) == "axle_centerline" and entity.entity_type == "LINE"]

    if body_entity is None and not axle_entities:
        return None

    if body_entity is not None and body_entity.bounds_mm is not None:
        origin = Point2D(
            (body_entity.bounds_mm[0] + body_entity.bounds_mm[1]) / 2.0,
            (body_entity.bounds_mm[2] + body_entity.bounds_mm[3]) / 2.0,
        )
        body_length_mm = body_entity.bounds_mm[1] - body_entity.bounds_mm[0]
        body_width_mm = body_entity.bounds_mm[3] - body_entity.bounds_mm[2]
    elif axle_entities:
        axle_centers = [
            center
            for entity in axle_entities
            if (center := _entity_center(entity)) is not None
        ]
        if axle_centers:
            origin = Point2D(
                sum(point.x_mm for point in axle_centers) / len(axle_centers),
                sum(point.y_mm for point in axle_centers) / len(axle_centers),
            )
        else:
            origin = Point2D(0.0, 0.0)
        body_length_mm = 0.0
        body_width_mm = 0.0
    else:
        origin = Point2D(0.0, 0.0)
        body_length_mm = 0.0
        body_width_mm = 0.0

    ordered_axles = sorted(
        axle_entities,
        key=lambda entity: (_entity_center(entity).x_mm if _entity_center(entity) is not None else 0.0),
    )
    axles: list[Axle] = []
    for index, entity in enumerate(ordered_axles):
        start = entity.geometry.get("start")
        end = entity.geometry.get("end")
        if not isinstance(start, Point2D) or not isinstance(end, Point2D):
            continue
        center = Point2D((start.x_mm + end.x_mm) / 2.0 - origin.x_mm, (start.y_mm + end.y_mm) / 2.0 - origin.y_mm)
        track_mm = (end - start).length()
        axles.append(
            Axle(
                id=f"imported_axle_{index + 1}",
                center=center,
                track_mm=track_mm,
                steerable=True,
                steering_mode="USER_DEFINED",
            )
        )

    if not axles and body_entity is None:
        return None

    return VehicleLayout(
        id="imported_dxf",
        name=f"Imported DXF ({source_name})" if source_name else "Imported DXF",
        axles=tuple(axles),
        body_length_mm=body_length_mm,
        body_width_mm=body_width_mm,
        origin=origin,
    )


def apply_dxf_role_overrides(report: DxfImportReport, role_overrides: dict[int, str | None]) -> DxfImportReport:
    overridden_entities: list[DxfImportedEntity] = []
    for entity in report.entities:
        if entity.index not in role_overrides:
            overridden_entities.append(entity)
            continue
        override = role_overrides[entity.index]
        if override in {"", None}:
            overridden_entities.append(
                replace(
                    entity,
                    assigned_role="",
                    confidence=0.0,
                    reason="Manually cleared in the DXF assignment workflow.",
                )
            )
            continue
        if override not in _DXF_ROLE_VALUES:
            raise ValueError(f"Unsupported DXF role: {override!r}")
        reason = "Manual assignment from DXF import workflow."
        overridden_entities.append(
            replace(
                entity,
                assigned_role=override,
                confidence=1.0,
                reason=reason,
            )
        )

    reconstructed_vehicle = _reconstruct_vehicle_layout(overridden_entities, report.source_name)
    parametric_mechanism = _build_parametric_mechanism(overridden_entities, report.source_name)
    return DxfImportReport(
        source_name=report.source_name,
        entity_count=report.entity_count,
        supported_entity_count=report.supported_entity_count,
        unsupported_entity_count=report.unsupported_entity_count,
        counts_by_type=report.counts_by_type,
        counts_by_layer=report.counts_by_layer,
        bounds_mm=report.bounds_mm,
        entities=tuple(overridden_entities),
        warnings=report.warnings,
        reconstructed_vehicle=reconstructed_vehicle,
        parametric_mechanism=parametric_mechanism,
    )


def analyze_dxf_import(dxf_text: str, source_name: str = "") -> DxfImportReport:
    pairs = _pair_tokens(dxf_text.splitlines())
    raw_records, warnings = _collect_raw_entities(pairs)
    raw_records, polyline_warnings = _attach_polyline_vertices(raw_records)
    warnings.extend(polyline_warnings)

    entities: list[DxfImportedEntity] = []
    entity_bounds: list[tuple[float, float, float, float] | None] = []
    supported_count = 0
    unsupported_count = 0

    for entity_index, record in enumerate(raw_records):
        entity_type = record.entity_type.upper()
        if entity_type in {"VERTEX", "SEQEND"}:
            continue
        layer = _first_string(record.groups, "8", "") or None
        summary, geometry, bounds_mm = _geometry_summary(record)
        role, confidence, reason = _suggest_role(entity_type, layer, geometry)
        if entity_type in SUPPORTED_ENTITY_TYPES:
            supported_count += 1
        else:
            unsupported_count += 1
        entities.append(
            DxfImportedEntity(
                index=entity_index,
                entity_type=entity_type,
                layer=layer,
                summary=summary,
                bounds_mm=bounds_mm,
                suggested_role=role,
                assigned_role=role,
                confidence=confidence,
                reason=reason,
                geometry=geometry,
            )
        )
        entity_bounds.append(bounds_mm)

    reconstructed_vehicle = _reconstruct_vehicle_layout(entities, source_name)
    parametric_mechanism = _build_parametric_mechanism(entities, source_name)
    counts_by_type = dict(sorted(Counter(entity.entity_type for entity in entities).items()))
    counts_by_layer = dict(sorted(Counter(entity.layer or "UNLAYERED" for entity in entities).items()))

    if not entities:
        warnings.append("No DXF entities were found inside the ENTITIES section.")

    return DxfImportReport(
        source_name=source_name,
        entity_count=len(entities),
        supported_entity_count=supported_count,
        unsupported_entity_count=unsupported_count,
        counts_by_type=counts_by_type,
        counts_by_layer=counts_by_layer,
        bounds_mm=_combine_bounds(entity_bounds),
        entities=tuple(entities),
        warnings=tuple(warnings),
        reconstructed_vehicle=reconstructed_vehicle,
        parametric_mechanism=parametric_mechanism,
    )
