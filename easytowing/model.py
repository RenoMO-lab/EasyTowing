from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Literal

from .errors import InvalidGeometryError
from .geometry import Point2D, normalize_angle

Side = Literal["left", "right"]
SteeringMode = Literal["FIXED", "FORCED_STEER", "SELF_STEER", "USER_DEFINED", "OPTIMIZED"]
SteeringSynchronizationMode = Literal[
    "SAME_PHASE",
    "OPPOSITE_PHASE",
    "RATIO",
    "LINKED_MECHANICALLY",
    "INDEPENDENT_TARGET",
]


@dataclass(frozen=True, slots=True)
class Pose2D:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_rad: float = 0.0

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

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise InvalidGeometryError("Axle IDs must not be empty.")
        if not math.isfinite(self.track_mm) or self.track_mm <= 0.0:
            raise InvalidGeometryError("Axle track must be a positive finite value.")
        if self.wheel_count < 2:
            raise InvalidGeometryError("An axle must define at least two wheels.")
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

    def wheels(self) -> tuple[Wheel, Wheel]:
        half_track = self.track_mm / 2.0
        lateral = Point2D(0.0, half_track).rotated_ccw(self.heading_rad)
        left = Wheel(
            id=f"{self.id}_left",
            axle_id=self.id,
            side="left",
            center=self.center + lateral,
            tire_width_mm=self.tire_width_mm,
            outside_diameter_mm=self.outside_diameter_mm,
        )
        right = Wheel(
            id=f"{self.id}_right",
            axle_id=self.id,
            side="right",
            center=self.center - lateral,
            tire_width_mm=self.tire_width_mm,
            outside_diameter_mm=self.outside_diameter_mm,
        )
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
        )


@dataclass(frozen=True, slots=True)
class ArticulationJoint:
    id: str
    parent_body_id: str
    child_body_id: str
    parent_anchor: Point2D
    child_anchor: Point2D
    articulation_rad: float = 0.0

    def resolve_child_pose(self, parent_pose: Pose2D) -> Pose2D:
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
        if self.body_polygon and len(self.body_polygon) < 3:
            raise InvalidGeometryError("Vehicle body polygons must contain at least three points.")
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
        if any(
            not math.isfinite(point.x_mm) or not math.isfinite(point.y_mm)
            for point in self.body_polygon
        ):
            raise InvalidGeometryError("Vehicle body polygon coordinates must be finite.")
        if not math.isfinite(self.maximum_articulation_deg) or self.maximum_articulation_deg < 0.0:
            raise InvalidGeometryError("Vehicle maximum articulation must be non-negative and finite.")
        synchronization_ids = [item.id for item in self.steering_synchronizations]
        if len(synchronization_ids) != len(set(synchronization_ids)):
            raise InvalidGeometryError("Steering synchronization IDs must be unique.")
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
        half_length = (body.body_length_mm or 0.0) / 2.0
        half_width = (body.body_width_mm or 0.0) / 2.0
        for corner in (
            Point2D(-half_length, -half_width),
            Point2D(half_length, -half_width),
            Point2D(half_length, half_width),
            Point2D(-half_length, half_width),
        ):
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
            }
            for joint in combination.joints
        ],
        "mounted_axles": [
            {
                "axle_id": mounted_axle.axle.id,
                "body_id": mounted_axle.body_id,
                "local_center": point_payload(mounted_axle.local_center),
                "resolved_center": point_payload(
                    mounted_axle.resolve(body_poses[mounted_axle.body_id]).center
                ),
                "track_mm": mounted_axle.axle.track_mm,
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
    }
