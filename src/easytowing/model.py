from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Literal

from .errors import InvalidGeometryError
from .geometry import Point2D, normalize_angle

Side = Literal["left", "right"]
SteeringMode = Literal["FIXED", "FORCED_STEER", "SELF_STEER", "USER_DEFINED", "OPTIMIZED"]


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

    def wheels(self) -> tuple[Wheel, Wheel]:
        half_track = self.track_mm / 2.0
        left = Wheel(
            id=f"{self.id}_left",
            axle_id=self.id,
            side="left",
            center=Point2D(self.center.x_mm, self.center.y_mm + half_track),
        )
        right = Wheel(
            id=f"{self.id}_right",
            axle_id=self.id,
            side="right",
            center=Point2D(self.center.x_mm, self.center.y_mm - half_track),
        )
        return left, right


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
        return replace(self.axle, center=body_pose.transform_point(self.local_center))


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
        for joint in self.joints:
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
        for mounted_axle in self.mounted_axles:
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
) -> VehicleLayout:
    """Reference two-axle layout used by the initial prototype."""

    half_wheelbase = wheelbase_mm / 2.0
    axles = (
        Axle(
            id="rear_axle",
            center=Point2D(-half_wheelbase, 0.0),
            track_mm=track_mm,
            steerable=True,
            steering_mode="FORCED_STEER",
        ),
        Axle(
            id="front_axle",
            center=Point2D(half_wheelbase, 0.0),
            track_mm=track_mm,
            steerable=True,
            steering_mode="FORCED_STEER",
        ),
    )
    body_length_mm = wheelbase_mm + 1800.0
    body_width_mm = track_mm + 700.0
    return VehicleLayout(
        id="reference_demo",
        name="Reference Four-Wheel Steering Demo",
        axles=axles,
        body_length_mm=body_length_mm,
        body_width_mm=body_width_mm,
    )
