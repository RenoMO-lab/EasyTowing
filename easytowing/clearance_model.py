from __future__ import annotations

from typing import Mapping

from .collision import CapsuleEnvelope, CircleEnvelope, CollisionItem
from .collision import PolygonEnvelope
from .geometry import Point2D
from .linkage import PlanarLinkageSpec, PlanarLinkageState
from .mechanism_graph import MechanismGraphState, PlanarMechanismGraph
from .model import Pose2D, RigidBody, VehicleCombination, VehicleLayout


def build_linkage_clearance_items(
    spec: PlanarLinkageSpec,
    state: PlanarLinkageState,
    *,
    vehicle: VehicleLayout | None = None,
) -> tuple[CollisionItem, ...]:
    """Build one connectivity-aware set of mechanism clearance envelopes."""

    items: list[CollisionItem] = [
        CollisionItem(
            id="input_rod",
            envelope=CapsuleEnvelope(state.driver_point, state.input_endpoint, 14.0),
        ),
        CollisionItem(
            id="tie_rod",
            envelope=CapsuleEnvelope(state.output_endpoint, state.steering_endpoint, 14.0),
            excluded_pair_ids=("steering_arm", "companion_tie_rod"),
        ),
        CollisionItem(
            id="steering_arm",
            envelope=CapsuleEnvelope(spec.steering_pivot, state.steering_endpoint, 14.0),
            excluded_pair_ids=("steering_pivot", "tie_rod", "companion_tie_rod"),
        ),
        CollisionItem(
            id="bell_crank_pivot",
            envelope=CircleEnvelope(spec.bell_crank_pivot, 28.0),
        ),
        CollisionItem(
            id="steering_pivot",
            envelope=CircleEnvelope(spec.steering_pivot, 28.0),
            excluded_pair_ids=("steering_arm",),
        ),
    ]

    if state.companion_steering_endpoint is not None and spec.companion_steering_pivot is not None:
        items.extend(
            (
                CollisionItem(
                    id="companion_tie_rod",
                    envelope=CapsuleEnvelope(
                        state.steering_endpoint,
                        state.companion_steering_endpoint,
                        14.0,
                    ),
                    excluded_pair_ids=(
                        "tie_rod",
                        "steering_arm",
                        "companion_steering_arm",
                    ),
                ),
                CollisionItem(
                    id="companion_steering_arm",
                    envelope=CapsuleEnvelope(
                        spec.companion_steering_pivot,
                        state.companion_steering_endpoint,
                        14.0,
                    ),
                    excluded_pair_ids=("companion_tie_rod", "companion_steering_pivot"),
                ),
                CollisionItem(
                    id="companion_steering_pivot",
                    envelope=CircleEnvelope(spec.companion_steering_pivot, 28.0),
                    excluded_pair_ids=("companion_steering_arm",),
                ),
            )
        )

    axle_sources = (
        (
            ("front_axle", Point2D(2180.0, 0.0), 1250.0, None),
            ("rear_axle", Point2D(-2180.0, 0.0), 1250.0, None),
        )
        if vehicle is None
        else tuple(
            (axle.id, axle.center, axle.track_mm / 2.0, axle)
            for axle in vehicle.axles
        )
    )
    for axle_id, center, half_track, axle in axle_sources:
        beam_id = f"{axle_id}_beam"
        if axle is None:
            beam_start = Point2D(center.x_mm, center.y_mm - half_track)
            beam_end = Point2D(center.x_mm, center.y_mm + half_track)
        else:
            left_wheel, right_wheel = axle.outer_wheels()
            beam_start = left_wheel.center
            beam_end = right_wheel.center
        items.append(
            CollisionItem(
                id=beam_id,
                envelope=CapsuleEnvelope(
                    beam_start,
                    beam_end,
                    70.0,
                ),
            )
        )
        if axle is not None and axle.outside_diameter_mm > 0.0:
            for wheel in axle.wheels():
                items.append(
                    CollisionItem(
                        id=f"{wheel.id}_tire",
                        envelope=CircleEnvelope(
                            wheel.center,
                            axle.outside_diameter_mm / 2.0,
                        ),
                        excluded_pair_ids=(beam_id,),
                    )
                )
    return tuple(items)


def build_mechanism_graph_clearance_items(
    graph: PlanarMechanismGraph,
    state: MechanismGraphState,
    *,
    vehicle: VehicleLayout | None = None,
    combination: VehicleCombination | None = None,
    body_poses: Mapping[str, Pose2D] | None = None,
) -> tuple[CollisionItem, ...]:
    """Build graph-member and articulated-body envelopes.

    Body-to-link and body-to-wheel pairs are excluded only when the component is
    mounted on that same body. Connected graph members are exempt only at their
    shared centerline joint; any additional overlap remains a collision. Cross-
    body component intersections and body-to-body pairs remain active, including
    across an articulation joint.
    """

    connected_pairs = graph.connected_member_pairs()
    incident_members: dict[str, set[str]] = {point.id: set() for point in graph.points}
    for member in graph.members:
        incident_members[member.point_a_id].add(member.id)
        incident_members[member.point_b_id].add(member.id)

    items: list[CollisionItem] = []
    point_by_id = graph.point_by_id()
    for member in graph.members:
        if member.envelope_radius_mm <= 0.0:
            continue
        member_body_ids = {
            point_by_id[point_id].body_id
            for point_id in (member.point_a_id, member.point_b_id)
        }
        mounted_body_id = (
            next(iter(member_body_ids))
            if len(member_body_ids) == 1 and None not in member_body_ids
            else None
        )
        connected_members = {
            other_id
            for pair in connected_pairs
            if member.id in pair
            for other_id in pair
            if other_id != member.id
        }
        items.append(
            CollisionItem(
                id=member.id,
                envelope=CapsuleEnvelope(
                    state.point_positions[member.point_a_id],
                    state.point_positions[member.point_b_id],
                    member.envelope_radius_mm,
                ),
                excluded_pair_ids=tuple(
                    sorted(
                        connected_members
                        | {member.point_a_id, member.point_b_id}
                    )
                ),
                mounted_body_id=mounted_body_id,
                connectivity_points=(
                    state.point_positions[member.point_a_id],
                    state.point_positions[member.point_b_id],
                ),
            )
        )

    for point in graph.points:
        if point.envelope_radius_mm <= 0.0:
            continue
        items.append(
            CollisionItem(
                id=point.id,
                envelope=CircleEnvelope(
                    state.point_positions[point.id],
                    point.envelope_radius_mm,
                ),
                excluded_pair_ids=tuple(sorted(incident_members[point.id])),
                mounted_body_id=point.body_id,
                connectivity_points=(state.point_positions[point.id],),
            )
        )

    if vehicle is not None:
        mounted_body_by_axle_id = {
            mounted.axle.id: mounted.body_id
            for mounted in combination.mounted_axles
        } if combination is not None else {}
        for axle in vehicle.axles:
            beam_id = f"{axle.id}_beam"
            mounted_body_id = mounted_body_by_axle_id.get(axle.id)
            left_wheel, right_wheel = axle.outer_wheels()
            items.append(
                CollisionItem(
                    id=beam_id,
                    envelope=CapsuleEnvelope(
                        left_wheel.center,
                        right_wheel.center,
                        70.0,
                    ),
                    mounted_body_id=mounted_body_id,
                )
            )
            if axle.outside_diameter_mm > 0.0:
                for wheel in axle.wheels():
                    items.append(
                        CollisionItem(
                            id=f"{wheel.id}_tire",
                            envelope=CircleEnvelope(
                                wheel.center,
                                axle.outside_diameter_mm / 2.0,
                            ),
                            excluded_pair_ids=(beam_id,),
                            mounted_body_id=mounted_body_id,
                        )
                    )

    if combination is not None:
        resolved_body_poses = (
            dict(body_poses)
            if body_poses is not None
            else combination.resolve_body_poses()
        )
        items.extend(
            build_combination_body_clearance_items(
                combination,
                resolved_body_poses,
                mounted_component_ids={
                    body_id: tuple(
                        item.id
                        for item in items
                        if item.mounted_body_id == body_id
                    )
                    for body_id in (body.id for body in combination.bodies)
                },
            )
        )
    return tuple(items)


def _rigid_body_outline(body: RigidBody) -> tuple[Point2D, ...]:
    """Return a local body outline, falling back to its dimensional rectangle."""

    if body.body_polygon:
        return body.body_polygon
    if (
        body.body_length_mm is None
        or body.body_width_mm is None
        or body.body_length_mm <= 0.0
        or body.body_width_mm <= 0.0
    ):
        return ()
    half_length = body.body_length_mm / 2.0
    half_width = body.body_width_mm / 2.0
    return (
        Point2D(-half_length, -half_width),
        Point2D(half_length, -half_width),
        Point2D(half_length, half_width),
        Point2D(-half_length, half_width),
    )


def build_combination_body_clearance_items(
    combination: VehicleCombination,
    body_poses: Mapping[str, Pose2D] | None = None,
    *,
    excluded_pair_ids: tuple[str, ...] = (),
    mounted_component_ids: Mapping[str, tuple[str, ...]] | None = None,
) -> tuple[CollisionItem, ...]:
    """Build world-space body envelopes while retaining body-to-body checks."""

    resolved_body_poses = (
        dict(body_poses)
        if body_poses is not None
        else combination.resolve_body_poses()
    )
    items: list[CollisionItem] = []
    for body in combination.bodies:
        pose = resolved_body_poses.get(body.id)
        if pose is None:
            raise ValueError(f"Missing resolved pose for rigid body {body.id!r}.")
        local_outline = _rigid_body_outline(body)
        if not local_outline:
            continue
        items.append(
            CollisionItem(
                id=f"body:{body.id}",
                envelope=PolygonEnvelope(
                    tuple(pose.transform_point(point) for point in local_outline)
                ),
                excluded_pair_ids=tuple(sorted(
                    set(excluded_pair_ids)
                    | set((mounted_component_ids or {}).get(body.id, ()))
                )),
                mounted_body_id=body.id,
            )
        )
    return tuple(items)
