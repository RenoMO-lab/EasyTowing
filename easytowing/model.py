from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Literal

from .errors import ArticulationLimitExceededError, InvalidGeometryError
from .geometry import EPSILON_MM, Point2D, normalize_angle

Side = Literal["left", "right"]
SteeringMode = Literal["FIXED", "FORCED_STEER", "SELF_STEER", "USER_DEFINED", "OPTIMIZED"]
SteeringSynchronizationMode = Literal[
    "SAME_PHASE",
    "OPPOSITE_PHASE",
    "RATIO",
    "LINKED_MECHANICALLY",
    "INDEPENDENT_TARGET",
]


def _validate_polygon(points: tuple[Point2D, ...], owner: str) -> None:
    """Reject polygon input that would make collision results ambiguous."""

    if not points:
        return
    if len(points) < 3:
        raise InvalidGeometryError(f"{owner} polygons must contain at least three points.")
    if any(
        not math.isfinite(point.x_mm) or not math.isfinite(point.y_mm)
        for point in points
    ):
        raise InvalidGeometryError(f"{owner} polygon coordinates must be finite.")

    x_values = [point.x_mm for point in points]
    y_values = [point.y_mm for point in points]
    coordinate_scale = max(
        1.0,
        max(x_values) - min(x_values),
        max(y_values) - min(y_values),
    )
    point_tolerance = max(EPSILON_MM, coordinate_scale * 1e-12)
    cross_tolerance = point_tolerance * coordinate_scale
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        if (next_point - point).length() <= point_tolerance:
            raise InvalidGeometryError(f"{owner} polygon contains a zero-length edge.")

    for left_index, left_point in enumerate(points):
        for right_point in points[left_index + 1 :]:
            if (right_point - left_point).length() <= point_tolerance:
                raise InvalidGeometryError(f"{owner} polygon contains duplicate vertices.")

    twice_area = sum(
        point.x_mm * points[(index + 1) % len(points)].y_mm
        - points[(index + 1) % len(points)].x_mm * point.y_mm
        for index, point in enumerate(points)
    )
    if abs(twice_area) <= coordinate_scale * coordinate_scale * 1e-12:
        raise InvalidGeometryError(f"{owner} polygon must enclose a non-zero area.")

    def cross(start: Point2D, end: Point2D, point: Point2D) -> float:
        vector = end - start
        offset = point - start
        return vector.x_mm * offset.y_mm - vector.y_mm * offset.x_mm

    def on_segment(point: Point2D, start: Point2D, end: Point2D) -> bool:
        return (
            abs(cross(start, end, point)) <= cross_tolerance
            and min(start.x_mm, end.x_mm) - point_tolerance <= point.x_mm <= max(start.x_mm, end.x_mm) + point_tolerance
            and min(start.y_mm, end.y_mm) - point_tolerance <= point.y_mm <= max(start.y_mm, end.y_mm) + point_tolerance
        )

    def intersects(
        start_a: Point2D,
        end_a: Point2D,
        start_b: Point2D,
        end_b: Point2D,
    ) -> bool:
        orientations = (
            cross(start_a, end_a, start_b),
            cross(start_a, end_a, end_b),
            cross(start_b, end_b, start_a),
            cross(start_b, end_b, end_a),
        )
        first, second, third, fourth = orientations
        if (
            ((first > cross_tolerance and second < -cross_tolerance)
             or (first < -cross_tolerance and second > cross_tolerance))
            and ((third > cross_tolerance and fourth < -cross_tolerance)
                 or (third < -cross_tolerance and fourth > cross_tolerance))
        ):
            return True
        return (
            (abs(first) <= cross_tolerance and on_segment(start_b, start_a, end_a))
            or (abs(second) <= cross_tolerance and on_segment(end_b, start_a, end_a))
            or (abs(third) <= cross_tolerance and on_segment(start_a, start_b, end_b))
            or (abs(fourth) <= cross_tolerance and on_segment(end_a, start_b, end_b))
        )

    edge_count = len(points)
    for left_index in range(edge_count):
        left_start = points[left_index]
        left_end = points[(left_index + 1) % edge_count]
        for right_index in range(left_index + 1, edge_count):
            right_start = points[right_index]
            right_end = points[(right_index + 1) % edge_count]
            if not intersects(left_start, left_end, right_start, right_end):
                continue
            adjacent = (
                right_index == left_index + 1
                or (left_index == 0 and right_index == edge_count - 1)
            )
            if adjacent:
                left_other = left_start if right_index == left_index + 1 else left_end
                right_other = right_end if right_index == left_index + 1 else right_start
                if on_segment(left_other, right_start, right_end) or on_segment(right_other, left_start, left_end):
                    raise InvalidGeometryError(f"{owner} polygon contains overlapping edges.")
                continue
            raise InvalidGeometryError(f"{owner} polygon self-intersects.")


@dataclass(frozen=True, slots=True)
class Pose2D:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if any(not math.isfinite(value) for value in (self.x_mm, self.y_mm, self.yaw_rad)):
            raise InvalidGeometryError("Pose coordinates and yaw must be finite.")

    def transform_point(self, local_point: Point2D) -> Point2D:
        rotated = local_point.rotated_ccw(self.yaw_rad)
        return Point2D(self.x_mm + rotated.x_mm, self.y_mm + rotated.y_mm)

    def inverse_transform_point(self, world_point: Point2D) -> Point2D:
        relative = world_point - Point2D(self.x_mm, self.y_mm)
        return relative.rotated_ccw(-self.yaw_rad)

    def compose(self, relative_pose: "Pose2D") -> "Pose2D":
        world_origin = self.transform_point(Point2D(relative_pose.x_mm, relative_pose.y_mm))
        return Pose2D(
            x_mm=world_origin.x_mm,
            y_mm=world_origin.y_mm,
            yaw_rad=normalize_angle(self.yaw_rad + relative_pose.yaw_rad),
        )


@dataclass(frozen=True, slots=True)
class Wheel:
    id: str
    axle_id: str
    side: Side
    center: Point2D
    tire_width_mm: float = 0.0
    outside_diameter_mm: float = 0.0
    lateral_offset_mm: float = 0.0


@dataclass(frozen=True, slots=True)
class Axle:
    id: str
    center: Point2D
    track_mm: float
    wheel_count: int = 2
    steerable: bool = True
    steering_mode: SteeringMode = "FORCED_STEER"
    maximum_steering_angle_deg: float | None = None
    steering_stop_deg: float | None = None
    load_kg: float | None = None
    heading_rad: float = 0.0
    user_defined_steering_angle_rad: float = 0.0
    tire_width_mm: float = 0.0
    outside_diameter_mm: float = 0.0
    wheel_lateral_offsets_mm: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise InvalidGeometryError("Axle IDs must not be empty.")
        if not math.isfinite(self.track_mm) or self.track_mm <= 0.0:
            raise InvalidGeometryError("Axle track must be a positive finite value.")
        if not isinstance(self.wheel_count, int) or isinstance(self.wheel_count, bool):
            raise InvalidGeometryError("Axle wheel_count must be an integer.")
        if self.wheel_count < 2 or self.wheel_count % 2:
            raise InvalidGeometryError("Axle wheel_count must be an even integer of at least two.")
        offsets = self.wheel_lateral_offsets_mm
        if offsets is None:
            if self.wheel_count != 2:
                raise InvalidGeometryError(
                    "Multi-wheel axles require explicit wheel_lateral_offsets_mm positions."
                )
        else:
            if len(offsets) != self.wheel_count:
                raise InvalidGeometryError(
                    f"Axle {self.id!r} requires one lateral offset per wheel position."
                )
            if any(not math.isfinite(offset) for offset in offsets):
                raise InvalidGeometryError(
                    f"Axle {self.id!r} wheel lateral offsets must be finite."
                )
            if len(set(offsets)) != len(offsets):
                raise InvalidGeometryError(
                    f"Axle {self.id!r} wheel lateral offsets must be unique."
                )
            if any(abs(offset) <= EPSILON_MM for offset in offsets):
                raise InvalidGeometryError(
                    f"Axle {self.id!r} wheel lateral offsets must not be on the axle centreline."
                )
            positive_count = sum(offset > 0.0 for offset in offsets)
            negative_count = sum(offset < 0.0 for offset in offsets)
            if positive_count != negative_count:
                raise InvalidGeometryError(
                    f"Axle {self.id!r} must have the same number of left and right wheel positions."
                )
        if any(not math.isfinite(value) for value in (self.center.x_mm, self.center.y_mm)):
            raise InvalidGeometryError("Axle center coordinates must be finite.")
        if self.steering_mode not in {"FIXED", "FORCED_STEER", "SELF_STEER", "USER_DEFINED", "OPTIMIZED"}:
            raise InvalidGeometryError(f"Unsupported steering mode: {self.steering_mode!r}.")
        numeric_values = (
            self.heading_rad,
            self.user_defined_steering_angle_rad,
            self.load_kg,
            self.maximum_steering_angle_deg,
            self.steering_stop_deg,
            self.tire_width_mm,
            self.outside_diameter_mm,
        )
        if any(value is not None and not math.isfinite(value) for value in numeric_values):
            raise InvalidGeometryError(f"Axle {self.id!r} contains a non-finite parameter.")
        for name, value in (
            ("maximum_steering_angle_deg", self.maximum_steering_angle_deg),
            ("steering_stop_deg", self.steering_stop_deg),
        ):
            if value is not None and value < 0.0:
                raise InvalidGeometryError(f"{name} must not be negative.")
        if self.tire_width_mm < 0.0 or self.outside_diameter_mm < 0.0:
            raise InvalidGeometryError("Tire dimensions must not be negative.")

    def wheels(self) -> tuple[Wheel, ...]:
        if self.wheel_lateral_offsets_mm is None:
            offsets = (self.track_mm / 2.0, -self.track_mm / 2.0)
        else:
            # Keep wheel IDs deterministic and group the two steering sides.
            offsets = tuple(
                sorted(
                    (offset for offset in self.wheel_lateral_offsets_mm if offset > 0.0),
                    reverse=True,
                )
            ) + tuple(
                sorted(
                    (offset for offset in self.wheel_lateral_offsets_mm if offset < 0.0),
                )
            )
        left_count = sum(offset > 0.0 for offset in offsets)
        wheels: list[Wheel] = []
        side_indices = {"left": 0, "right": 0}
        for offset in offsets:
            side: Side = "left" if offset > 0.0 else "right"
            side_indices[side] += 1
            suffix = side if left_count == 1 else f"{side}_{side_indices[side]}"
            lateral = Point2D(0.0, offset).rotated_ccw(self.heading_rad)
            wheels.append(
                Wheel(
                    id=f"{self.id}_{suffix}",
                    axle_id=self.id,
                    side=side,
                    center=self.center + lateral,
                    tire_width_mm=self.tire_width_mm,
                    outside_diameter_mm=self.outside_diameter_mm,
                    lateral_offset_mm=offset,
                )
            )
        return tuple(wheels)

    def outer_wheels(self) -> tuple[Wheel, Wheel]:
        """Return the outermost left and right wheels for axle/beam geometry."""

        wheels = self.wheels()
        left = max((wheel for wheel in wheels if wheel.side == "left"), key=lambda wheel: wheel.lateral_offset_mm)
        right = min((wheel for wheel in wheels if wheel.side == "right"), key=lambda wheel: wheel.lateral_offset_mm)
        return left, right


@dataclass(frozen=True, slots=True)
class SteeringTargetPoint:
    """One point in an independent axle steering target curve."""

    beta_rad: float
    steering_angle_rad: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.beta_rad) or not math.isfinite(self.steering_angle_rad):
            raise InvalidGeometryError("Steering target curve points must be finite.")


@dataclass(frozen=True, slots=True)
class SteeringSynchronization:
    """Describe how one axle receives a coordinated steering command."""

    id: str
    target_axle_id: str
    mode: SteeringSynchronizationMode = "SAME_PHASE"
    source_axle_id: str | None = None
    ratio: float = 1.0
    phase_offset_rad: float = 0.0
    target_curve: tuple[SteeringTargetPoint, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.target_axle_id.strip():
            raise InvalidGeometryError("Steering synchronization IDs must not be empty.")
        if self.source_axle_id is not None and not self.source_axle_id.strip():
            raise InvalidGeometryError("Steering synchronization source ID must not be empty.")
        if self.source_axle_id == self.target_axle_id:
            raise InvalidGeometryError(
                f"Steering synchronization {self.id!r} cannot target its source axle."
            )
        if self.mode not in {
            "SAME_PHASE",
            "OPPOSITE_PHASE",
            "RATIO",
            "LINKED_MECHANICALLY",
            "INDEPENDENT_TARGET",
        }:
            raise InvalidGeometryError(f"Unsupported steering synchronization mode: {self.mode!r}.")
        if not math.isfinite(self.ratio) or not math.isfinite(self.phase_offset_rad):
            raise InvalidGeometryError("Steering synchronization parameters must be finite.")
        beta_values = [point.beta_rad for point in self.target_curve]
        if beta_values != sorted(beta_values) or len(beta_values) != len(set(beta_values)):
            raise InvalidGeometryError("Steering target curve beta values must be strictly increasing.")
        if self.mode == "INDEPENDENT_TARGET" and not self.target_curve:
            raise InvalidGeometryError("Independent steering targets require at least one curve point.")


@dataclass(frozen=True, slots=True)
class RigidBody:
    id: str
    name: str
    pose: Pose2D = field(default_factory=Pose2D)
    body_length_mm: float | None = None
    body_width_mm: float | None = None
    parent_joint_id: str | None = None
    child_joint_ids: tuple[str, ...] = field(default_factory=tuple)
    body_polygon: tuple[Point2D, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise InvalidGeometryError("Rigid body IDs must not be empty.")
        if self.body_length_mm is not None and (
            not math.isfinite(self.body_length_mm) or self.body_length_mm < 0.0
        ):
            raise InvalidGeometryError("Rigid body length must be finite and non-negative.")
        if self.body_width_mm is not None and (
            not math.isfinite(self.body_width_mm) or self.body_width_mm < 0.0
        ):
            raise InvalidGeometryError("Rigid body width must be finite and non-negative.")
        _validate_polygon(self.body_polygon, f"Rigid body {self.id!r}")


@dataclass(frozen=True, slots=True)
class Trailer:
    """Parametric trailer envelope that can be mounted as a rigid body."""

    id: str
    name: str
    overall_length_mm: float
    overall_width_mm: float
    origin: Point2D = field(default_factory=lambda: Point2D(0.0, 0.0))
    body_polygon: tuple[Point2D, ...] = field(default_factory=tuple)
    front_articulation_point: Point2D | None = None
    rear_articulation_point: Point2D | None = None
    kingpin_point: Point2D | None = None
    maximum_articulation_deg: float = 45.0

    def as_rigid_body(self) -> RigidBody:
        return RigidBody(
            id=self.id,
            name=self.name,
            pose=Pose2D(self.origin.x_mm, self.origin.y_mm, 0.0),
            body_length_mm=self.overall_length_mm,
            body_width_mm=self.overall_width_mm,
            body_polygon=self.body_polygon,
        )


@dataclass(frozen=True, slots=True)
class ArticulationJoint:
    id: str
    parent_body_id: str
    child_body_id: str
    parent_anchor: Point2D
    child_anchor: Point2D
    articulation_rad: float = 0.0
    sweep_min_deg: float | None = None
    sweep_max_deg: float | None = None
    sweep_step_deg: float | None = None
    maximum_articulation_deg: float = 45.0

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise InvalidGeometryError("Articulation joint IDs must not be empty.")
        if not self.parent_body_id.strip() or not self.child_body_id.strip():
            raise InvalidGeometryError("Articulation joints require parent and child body IDs.")
        if self.parent_body_id == self.child_body_id:
            raise InvalidGeometryError("An articulation joint cannot connect a body to itself.")
        for name, point in (("parent_anchor", self.parent_anchor), ("child_anchor", self.child_anchor)):
            if not math.isfinite(point.x_mm) or not math.isfinite(point.y_mm):
                raise InvalidGeometryError(f"Joint {self.id!r} {name} coordinates must be finite.")
        if not math.isfinite(self.articulation_rad):
            raise InvalidGeometryError(f"Joint {self.id!r} articulation must be finite.")
        if not math.isfinite(self.maximum_articulation_deg) or self.maximum_articulation_deg < 0.0:
            raise InvalidGeometryError(
                f"Joint {self.id!r} maximum articulation must be non-negative and finite."
            )
        sweep_values = (self.sweep_min_deg, self.sweep_max_deg, self.sweep_step_deg)
        if any(value is not None and not math.isfinite(value) for value in sweep_values):
            raise InvalidGeometryError(f"Joint {self.id!r} has non-finite sweep metadata.")
        if any(value is not None for value in sweep_values):
            if any(value is None for value in sweep_values):
                raise InvalidGeometryError(
                    f"Joint {self.id!r} sweep metadata requires min, max, and step."
                )
            assert self.sweep_min_deg is not None
            assert self.sweep_max_deg is not None
            assert self.sweep_step_deg is not None
            if self.sweep_min_deg >= self.sweep_max_deg:
                raise InvalidGeometryError(f"Joint {self.id!r} sweep minimum must be below maximum.")
            if self.sweep_min_deg > 0.0 or self.sweep_max_deg < 0.0:
                raise InvalidGeometryError(f"Joint {self.id!r} sweep bounds must straddle zero.")
            if self.sweep_step_deg <= 0.0:
                raise InvalidGeometryError(f"Joint {self.id!r} sweep step must be positive.")

    def resolve_child_pose(self, parent_pose: Pose2D) -> Pose2D:
        articulation_deg = math.degrees(self.articulation_rad)
        if abs(articulation_deg) > self.maximum_articulation_deg + 1e-9:
            raise ArticulationLimitExceededError(
                articulation_deg,
                self.maximum_articulation_deg,
                joint_id=self.id,
            )
        child_yaw = normalize_angle(parent_pose.yaw_rad + self.articulation_rad)
        parent_anchor_world = parent_pose.transform_point(self.parent_anchor)
        child_anchor_offset = self.child_anchor.rotated_ccw(child_yaw)
        return Pose2D(
            x_mm=parent_anchor_world.x_mm - child_anchor_offset.x_mm,
            y_mm=parent_anchor_world.y_mm - child_anchor_offset.y_mm,
            yaw_rad=child_yaw,
        )


@dataclass(frozen=True, slots=True)
class MountedAxle:
    axle: Axle
    body_id: str
    local_center: Point2D

    def __post_init__(self) -> None:
        if not self.body_id.strip():
            raise InvalidGeometryError("Mounted axles require a body ID.")
        if not math.isfinite(self.local_center.x_mm) or not math.isfinite(self.local_center.y_mm):
            raise InvalidGeometryError(f"Mounted axle {self.axle.id!r} local coordinates must be finite.")

    def resolve(self, body_pose: Pose2D) -> Axle:
        return replace(
            self.axle,
            center=body_pose.transform_point(self.local_center),
            heading_rad=normalize_angle(body_pose.yaw_rad + self.axle.heading_rad),
        )


@dataclass(frozen=True, slots=True)
class VehicleCombination:
    id: str
    name: str
    bodies: tuple[RigidBody, ...] = field(default_factory=tuple)
    joints: tuple[ArticulationJoint, ...] = field(default_factory=tuple)
    mounted_axles: tuple[MountedAxle, ...] = field(default_factory=tuple)
    root_body_id: str | None = None
    steering_synchronizations: tuple[SteeringSynchronization, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        body_ids = [body.id for body in self.bodies]
        if len(body_ids) != len(set(body_ids)):
            raise InvalidGeometryError("VehicleCombination body IDs must be unique.")
        body_id_set = set(body_ids)
        root_id = self.root_body_id or (body_ids[0] if body_ids else None)
        if root_id is not None and root_id not in body_id_set:
            raise InvalidGeometryError(f"VehicleCombination root body not found: {root_id!r}.")

        joint_ids: set[str] = set()
        child_body_ids: set[str] = set()
        parent_by_child: dict[str, str] = {}
        for joint in self.joints:
            if joint.id in joint_ids:
                raise InvalidGeometryError(f"VehicleCombination joint IDs must be unique: {joint.id!r}.")
            joint_ids.add(joint.id)
            if joint.parent_body_id not in body_id_set or joint.child_body_id not in body_id_set:
                raise InvalidGeometryError(
                    f"VehicleCombination joint {joint.id!r} references an unknown body."
                )
            if joint.child_body_id in child_body_ids:
                raise InvalidGeometryError(
                    f"Multiple joints reference the same child body: {joint.child_body_id!r}."
                )
            child_body_ids.add(joint.child_body_id)
            parent_by_child[joint.child_body_id] = joint.parent_body_id
        if root_id is not None and root_id in child_body_ids:
            raise InvalidGeometryError(f"Root body {root_id!r} cannot also be a child body.")
        for body_id in body_id_set:
            if body_id == root_id:
                continue
            visited: set[str] = set()
            current = body_id
            while current != root_id:
                if current in visited:
                    raise InvalidGeometryError("Articulation graph contains a cycle.")
                visited.add(current)
                current = parent_by_child.get(current, "")
                if not current:
                    raise InvalidGeometryError(
                        f"Body {body_id!r} is missing a parent articulation joint."
                    )

        mounted_axle_ids = [mounted.axle.id for mounted in self.mounted_axles]
        if len(mounted_axle_ids) != len(set(mounted_axle_ids)):
            raise InvalidGeometryError("Mounted axle IDs must be unique.")
        if any(mounted.body_id not in body_id_set for mounted in self.mounted_axles):
            raise InvalidGeometryError("Mounted axles must reference an existing body.")

        sync_ids = [item.id for item in self.steering_synchronizations]
        if len(sync_ids) != len(set(sync_ids)):
            raise InvalidGeometryError("VehicleCombination steering synchronization IDs must be unique.")
        sync_targets = [item.target_axle_id for item in self.steering_synchronizations]
        if len(sync_targets) != len(set(sync_targets)):
            raise InvalidGeometryError(
                "VehicleCombination steering synchronization targets must be unique."
            )
        axle_ids = {mounted.axle.id for mounted in self.mounted_axles}
        for item in self.steering_synchronizations:
            if item.target_axle_id not in axle_ids:
                raise InvalidGeometryError(
                    f"VehicleCombination synchronization target axle not found: {item.target_axle_id!r}."
                )
            if item.source_axle_id is not None and item.source_axle_id not in axle_ids:
                raise InvalidGeometryError(
                    f"VehicleCombination synchronization source axle not found: {item.source_axle_id!r}."
                )

    def _body_map(self) -> dict[str, RigidBody]:
        body_map = {body.id: body for body in self.bodies}
        if len(body_map) != len(self.bodies):
            raise InvalidGeometryError("VehicleCombination body IDs must be unique.")
        return body_map

    def _joint_by_child(self) -> dict[str, ArticulationJoint]:
        joint_map: dict[str, ArticulationJoint] = {}
        joint_ids: set[str] = set()
        for joint in self.joints:
            if joint.id in joint_ids:
                raise InvalidGeometryError(f"VehicleCombination joint IDs must be unique: {joint.id!r}.")
            joint_ids.add(joint.id)
            if joint.child_body_id in joint_map:
                raise InvalidGeometryError(
                    f"Multiple joints reference the same child body: {joint.child_body_id!r}."
                )
            joint_map[joint.child_body_id] = joint
        return joint_map

    def resolve_body_poses(self, root_pose: Pose2D | None = None) -> dict[str, Pose2D]:
        if not self.bodies:
            return {}

        body_map = self._body_map()
        joint_by_child = self._joint_by_child()
        root_id = self.root_body_id or self.bodies[0].id
        if root_id not in body_map:
            raise InvalidGeometryError(f"Root body {root_id!r} not found in the combination.")
        if root_id in joint_by_child:
            raise InvalidGeometryError(f"Root body {root_id!r} cannot also be a child body.")

        missing_children = [
            joint.child_body_id for joint in self.joints if joint.child_body_id not in body_map
        ]
        if missing_children:
            raise InvalidGeometryError(
                f"Joint references missing child body {missing_children[0]!r}."
            )

        unresolved_parents = {
            joint.child_body_id: joint.parent_body_id
            for joint in self.joints
            if joint.parent_body_id not in body_map
        }
        if unresolved_parents:
            child_id, parent_id = next(iter(unresolved_parents.items()))
            raise InvalidGeometryError(
                f"Joint {child_id!r} references missing parent body {parent_id!r}."
            )

        resolved: dict[str, Pose2D] = {}
        visiting: set[str] = set()

        def resolve(body_id: str) -> Pose2D:
            if body_id in resolved:
                return resolved[body_id]
            if body_id in visiting:
                raise InvalidGeometryError("Articulation graph contains a cycle.")
            visiting.add(body_id)
            body = body_map[body_id]
            joint = joint_by_child.get(body_id)
            if joint is None:
                if body_id != root_id:
                    raise InvalidGeometryError(
                        f"Body {body_id!r} is missing a parent articulation joint."
                    )
                pose = root_pose if root_pose is not None else body.pose
            else:
                parent_pose = resolve(joint.parent_body_id)
                pose = joint.resolve_child_pose(parent_pose)
            visiting.remove(body_id)
            resolved[body_id] = pose
            return pose

        for body_id in body_map:
            resolve(body_id)

        return resolved

    def resolve_mounted_axles(self, root_pose: Pose2D | None = None) -> tuple[Axle, ...]:
        body_poses = self.resolve_body_poses(root_pose=root_pose)
        resolved_axles: list[Axle] = []
        axle_ids: set[str] = set()
        for mounted_axle in self.mounted_axles:
            if mounted_axle.axle.id in axle_ids:
                raise InvalidGeometryError(
                    f"Mounted axle IDs must be unique: {mounted_axle.axle.id!r}."
                )
            axle_ids.add(mounted_axle.axle.id)
            body_pose = body_poses.get(mounted_axle.body_id)
            if body_pose is None:
                raise InvalidGeometryError(
                    f"Mounted axle {mounted_axle.axle.id!r} references missing body {mounted_axle.body_id!r}."
                )
            resolved_axles.append(mounted_axle.resolve(body_pose))
        return tuple(resolved_axles)

    def to_vehicle_layout(self, root_pose: Pose2D | None = None) -> "VehicleLayout":
        resolved_axles = self.resolve_mounted_axles(root_pose=root_pose)
        body_length_mm = max((body.body_length_mm or 0.0 for body in self.bodies), default=0.0)
        body_width_mm = max((body.body_width_mm or 0.0 for body in self.bodies), default=0.0)
        origin = root_pose or (self.bodies[0].pose if self.bodies else Pose2D())
        return VehicleLayout(
            id=self.id,
            name=self.name,
            axles=resolved_axles,
            body_length_mm=body_length_mm,
            body_width_mm=body_width_mm,
            origin=Point2D(origin.x_mm, origin.y_mm),
            steering_synchronizations=self.steering_synchronizations,
        )


@dataclass(frozen=True, slots=True)
class VehicleLayout:
    id: str
    name: str
    axles: tuple[Axle, ...] = field(default_factory=tuple)
    body_length_mm: float = 0.0
    body_width_mm: float = 0.0
    origin: Point2D = field(default_factory=lambda: Point2D(0.0, 0.0))
    steering_synchronizations: tuple[SteeringSynchronization, ...] = field(default_factory=tuple)
    body_polygon: tuple[Point2D, ...] = field(default_factory=tuple)
    front_articulation_point: Point2D | None = None
    rear_articulation_point: Point2D | None = None
    kingpin_point: Point2D | None = None
    maximum_articulation_deg: float = 45.0

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise InvalidGeometryError("Vehicle IDs must not be empty.")
        axle_ids = [axle.id for axle in self.axles]
        if len(axle_ids) != len(set(axle_ids)):
            raise InvalidGeometryError("Vehicle axle IDs must be unique.")
        if not math.isfinite(self.body_length_mm) or not math.isfinite(self.body_width_mm):
            raise InvalidGeometryError("Vehicle body dimensions must be finite.")
        if self.body_length_mm < 0.0 or self.body_width_mm < 0.0:
            raise InvalidGeometryError("Vehicle body dimensions must not be negative.")
        _validate_polygon(self.body_polygon, f"Vehicle {self.id!r}")
        metadata_points = (
            ("origin", self.origin),
            ("front_articulation_point", self.front_articulation_point),
            ("rear_articulation_point", self.rear_articulation_point),
            ("kingpin_point", self.kingpin_point),
        )
        for name, point in metadata_points:
            if point is not None and (
                not math.isfinite(point.x_mm) or not math.isfinite(point.y_mm)
            ):
                raise InvalidGeometryError(f"Vehicle {name} coordinates must be finite.")
        if not math.isfinite(self.maximum_articulation_deg) or self.maximum_articulation_deg < 0.0:
            raise InvalidGeometryError("Vehicle maximum articulation must be non-negative and finite.")
        synchronization_ids = [item.id for item in self.steering_synchronizations]
        if len(synchronization_ids) != len(set(synchronization_ids)):
            raise InvalidGeometryError("Steering synchronization IDs must be unique.")
        synchronization_targets = [
            item.target_axle_id for item in self.steering_synchronizations
        ]
        if len(synchronization_targets) != len(set(synchronization_targets)):
            raise InvalidGeometryError("Steering synchronization targets must be unique.")
        axle_id_set = set(axle_ids)
        for item in self.steering_synchronizations:
            if item.target_axle_id not in axle_id_set:
                raise InvalidGeometryError(
                    f"Steering synchronization target axle not found: {item.target_axle_id!r}."
                )
            if item.source_axle_id is not None and item.source_axle_id not in axle_id_set:
                raise InvalidGeometryError(
                    f"Steering synchronization source axle not found: {item.source_axle_id!r}."
                )

    def wheels(self) -> tuple[Wheel, ...]:
        ordered: list[Wheel] = []
        for axle in self.axles:
            ordered.extend(axle.wheels())
        return tuple(ordered)

    def axle_span_mm(self) -> float:
        if not self.axles:
            return 0.0
        x_values = [axle.center.x_mm for axle in self.axles]
        return max(x_values) - min(x_values)


def build_reference_demo_layout(
    wheelbase_mm: float = 4360.0,
    track_mm: float = 2500.0,
    articulation_rad: float = 0.0,
) -> VehicleLayout:
    """Return the reference layout through the articulated domain model."""

    layout = combination_to_vehicle_layout(
        build_reference_demo_combination(
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            articulation_rad=articulation_rad,
        )
    )
    return replace(
        layout,
        steering_synchronizations=(
            SteeringSynchronization(
                id="rear_to_front_sync",
                target_axle_id="rear_axle",
                source_axle_id="front_axle",
                mode="OPPOSITE_PHASE",
            ),
        ),
    )


def build_reference_demo_combination(
    wheelbase_mm: float = 4360.0,
    track_mm: float = 2500.0,
    articulation_rad: float = 0.0,
) -> VehicleCombination:
    """Build the two-body reference combination used by the prototype.

    The joint is deliberately part of the data model even though the first
    steering demo solves a fixed planar layout.  This keeps the reference case
    compatible with tractor, dolly, and multi-trailer extensions.
    """

    half_wheelbase = wheelbase_mm / 2.0
    body_length_mm = 1800.0
    body_width_mm = track_mm + 700.0
    return VehicleCombination(
        id="reference_demo_combination",
        name="Reference Four-Wheel Steering Combination",
        bodies=(
            RigidBody(
                id="rear_body",
                name="Rear Body",
                pose=Pose2D(-half_wheelbase, 0.0, 0.0),
                body_length_mm=body_length_mm,
                body_width_mm=body_width_mm,
                child_joint_ids=("front_joint",),
            ),
            RigidBody(
                id="front_body",
                name="Front Body",
                pose=Pose2D(half_wheelbase, 0.0, 0.0),
                body_length_mm=body_length_mm,
                body_width_mm=body_width_mm,
                parent_joint_id="front_joint",
            ),
        ),
        joints=(
            ArticulationJoint(
                id="front_joint",
                parent_body_id="rear_body",
                child_body_id="front_body",
                parent_anchor=Point2D(half_wheelbase, 0.0),
                child_anchor=Point2D(-half_wheelbase, 0.0),
                articulation_rad=articulation_rad,
            ),
        ),
        mounted_axles=(
            MountedAxle(
                axle=Axle(
                    id="rear_axle",
                    center=Point2D(0.0, 0.0),
                    track_mm=track_mm,
                    steerable=True,
                    steering_mode="FORCED_STEER",
                ),
                body_id="rear_body",
                local_center=Point2D(0.0, 0.0),
            ),
            MountedAxle(
                axle=Axle(
                    id="front_axle",
                    center=Point2D(0.0, 0.0),
                    track_mm=track_mm,
                    steerable=True,
                    steering_mode="FORCED_STEER",
                ),
                body_id="front_body",
                local_center=Point2D(0.0, 0.0),
            ),
        ),
        root_body_id="rear_body",
        steering_synchronizations=(
            SteeringSynchronization(
                id="rear_to_front_sync",
                target_axle_id="rear_axle",
                source_axle_id="front_axle",
                mode="OPPOSITE_PHASE",
            ),
        ),
    )


def combination_to_vehicle_layout(
    combination: VehicleCombination,
    root_pose: Pose2D | None = None,
) -> VehicleLayout:
    """Flatten an articulated combination into resolved axle coordinates."""

    body_poses = combination.resolve_body_poses(root_pose=root_pose)
    x_values: list[float] = []
    y_values: list[float] = []
    for body in combination.bodies:
        pose = body_poses[body.id]
        if body.body_polygon:
            local_outline = body.body_polygon
        else:
            half_length = (body.body_length_mm or 0.0) / 2.0
            half_width = (body.body_width_mm or 0.0) / 2.0
            local_outline = (
                Point2D(-half_length, -half_width),
                Point2D(half_length, -half_width),
                Point2D(half_length, half_width),
                Point2D(-half_length, half_width),
            )
        for corner in local_outline:
            world_corner = pose.transform_point(corner)
            x_values.append(world_corner.x_mm)
            y_values.append(world_corner.y_mm)

    root_id = combination.root_body_id or (combination.bodies[0].id if combination.bodies else "")
    origin_pose = body_poses.get(root_id, root_pose or Pose2D())
    return VehicleLayout(
        id=combination.id,
        name=combination.name,
        axles=combination.resolve_mounted_axles(root_pose=root_pose),
        body_length_mm=max(x_values) - min(x_values) if x_values else 0.0,
        body_width_mm=max(y_values) - min(y_values) if y_values else 0.0,
        origin=Point2D(origin_pose.x_mm, origin_pose.y_mm),
        steering_synchronizations=combination.steering_synchronizations,
    )


def serialize_vehicle_combination(
    combination: VehicleCombination,
    root_pose: Pose2D | None = None,
) -> dict[str, object]:
    """Return traceable body, joint, and mounted-axle data for API/export use."""

    body_poses = combination.resolve_body_poses(root_pose=root_pose)
    layout = combination_to_vehicle_layout(combination, root_pose=root_pose)

    def point_payload(point: Point2D) -> dict[str, float]:
        return {"x_mm": point.x_mm, "y_mm": point.y_mm}

    def pose_payload(pose: Pose2D) -> dict[str, float]:
        return {
            "x_mm": pose.x_mm,
            "y_mm": pose.y_mm,
            "yaw_rad": pose.yaw_rad,
            "yaw_deg": math.degrees(pose.yaw_rad),
        }

    return {
        "id": combination.id,
        "name": combination.name,
        "root_body_id": combination.root_body_id,
        "body_count": len(combination.bodies),
        "joint_count": len(combination.joints),
        "overall_body_length_mm": layout.body_length_mm,
        "overall_body_width_mm": layout.body_width_mm,
        "bodies": [
            {
                "id": body.id,
                "name": body.name,
                "pose": pose_payload(body_poses[body.id]),
                "body_length_mm": body.body_length_mm,
                "body_width_mm": body.body_width_mm,
                "parent_joint_id": body.parent_joint_id,
                "child_joint_ids": list(body.child_joint_ids),
                "body_polygon": [point_payload(point) for point in body.body_polygon],
            }
            for body in combination.bodies
        ],
        "joints": [
            {
                "id": joint.id,
                "parent_body_id": joint.parent_body_id,
                "child_body_id": joint.child_body_id,
                "parent_anchor": point_payload(joint.parent_anchor),
                "child_anchor": point_payload(joint.child_anchor),
                "articulation_rad": joint.articulation_rad,
                "articulation_deg": math.degrees(joint.articulation_rad),
                "sweep_min_deg": joint.sweep_min_deg,
                "sweep_max_deg": joint.sweep_max_deg,
                "sweep_step_deg": joint.sweep_step_deg,
                "maximum_articulation_deg": joint.maximum_articulation_deg,
            }
            for joint in combination.joints
        ],
        "joint_ranges": {
            joint.id: {
                "min_deg": joint.sweep_min_deg,
                "max_deg": joint.sweep_max_deg,
                "step_deg": joint.sweep_step_deg,
            }
            for joint in combination.joints
            if joint.sweep_min_deg is not None
            and joint.sweep_max_deg is not None
            and joint.sweep_step_deg is not None
        },
        "mounted_axles": [
            {
                "axle_id": mounted_axle.axle.id,
                "body_id": mounted_axle.body_id,
                "local_center": point_payload(mounted_axle.local_center),
                "resolved_center": point_payload(
                    mounted_axle.resolve(body_poses[mounted_axle.body_id]).center
                ),
                "track_mm": mounted_axle.axle.track_mm,
                "wheel_count": mounted_axle.axle.wheel_count,
                "wheel_lateral_offsets_mm": (
                    None
                    if mounted_axle.axle.wheel_lateral_offsets_mm is None
                    else list(mounted_axle.axle.wheel_lateral_offsets_mm)
                ),
                "steerable": mounted_axle.axle.steerable,
                "steering_mode": mounted_axle.axle.steering_mode,
                "user_defined_steering_angle_deg": math.degrees(
                    mounted_axle.axle.user_defined_steering_angle_rad
                ),
                "heading_rad": mounted_axle.resolve(body_poses[mounted_axle.body_id]).heading_rad,
                "heading_deg": math.degrees(
                    mounted_axle.resolve(body_poses[mounted_axle.body_id]).heading_rad
                ),
            }
            for mounted_axle in combination.mounted_axles
        ],
        "steering_synchronizations": [
            {
                "id": synchronization.id,
                "target_axle_id": synchronization.target_axle_id,
                "source_axle_id": synchronization.source_axle_id,
                "mode": synchronization.mode,
                "ratio": synchronization.ratio,
                "phase_offset_rad": synchronization.phase_offset_rad,
                "phase_offset_deg": math.degrees(synchronization.phase_offset_rad),
                "target_curve": [
                    {
                        "beta_rad": point.beta_rad,
                        "beta_deg": math.degrees(point.beta_rad),
                        "steering_angle_rad": point.steering_angle_rad,
                        "steering_angle_deg": math.degrees(point.steering_angle_rad),
                    }
                    for point in synchronization.target_curve
                ],
            }
            for synchronization in combination.steering_synchronizations
        ],
    }
