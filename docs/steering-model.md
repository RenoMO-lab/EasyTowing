# Steering Model Specification

## Coordinate system

- X axis: longitudinal, positive forward
- Y axis: lateral, positive left
- Units: millimeters internally, degrees only for UI/reporting
- Angles: radians internally, counterclockwise positive

## Core objects

- `Point2D`: Cartesian point in trailer-local coordinates
- `Pose2D`: rigid-body pose with translation and yaw
- `Axle`: axle center position and track
- `Wheel`: wheel center position and side
- `VehicleLayout`: ordered collection of axles
- `RigidBody`: articulated body node with pose metadata
- `ArticulationJoint`: parent/child body connection
- `VehicleCombination`: chain of rigid bodies and mounted axles

## Instantaneous Center of Rotation

For ideal planar rolling, every wheel heading is tangent to a circle centered on the instantaneous center of rotation (ICR).

Given:

- wheel center `p = (x, y)`
- ICR `c = (cx, cy)`

Define the radius vector:

```text
r = p - c
```

The two tangential directions are:

```text
t1 = (-r_y, r_x)
t2 = (r_y, -r_x)
```

The solver chooses the tangent that points most closely along the nominal forward axis of the vehicle. In the initial prototype the nominal forward axis is the positive X axis.

The wheel heading angle is:

```text
delta = atan2(t_y, t_x)
```

## Ackermann special case

For a single steering axle on a vehicle with wheelbase `L` and track `T`, and a left-turn ICR on the rear axle centerline at radius `R`:

```text
delta_inner = atan(L / (R - T/2))
delta_outer = atan(L / (R + T/2))
```

For right turns, the same magnitudes apply with negative sign.

## Multi-body maneuver kinematics

`solve_combination_kinematics` resolves every rigid-body pose and mounted axle
before solving wheel headings around one world-space ICR. The maneuver is
defined in one of two engineering-valid ways:

- an explicit signed turn radius relative to the root body; or
- an ICR derived from at least two non-parallel fixed-axle rolling constraints.

Every additional fixed axle must pass through the same ICR within the configured
length tolerance. Incompatible articulation chains raise
`MULTIBODY_KINEMATIC_INCONSISTENT`; they are never silently approximated.

## Legacy single-layout beta mapping

The legacy single-layout browser path uses a surrogate `beta -> radius` mapping.
It remains available for quick reference studies, but it is not used by the
explicit multi-body combination path described above.

Production design acceptance must use an explicit maneuver definition and a
Monroc-approved reference case rather than relying on this surrogate mapping.

## Generalized mechanical component graph

`PlanarMechanismGraph` represents a mechanism as stable-ID points and rigid
members rather than a fixed primary/companion schema. Points are fixed, driven,
or free. Steering arms, rods, bell cranks, and longer synchronization chains are
expressed as distance constraints; a bell crank is a rigid triangle whose third
member preserves the included arm angle.

`solve_mechanism_graph` solves all free point coordinates together, checks every
member residual against the configured geometric tolerance, maintains the
previous sweep branch, and enforces declared angle limits. Shared endpoints
derive collision exclusions automatically. Non-connected members remain in the
clearance analysis even when they cross near another component's joint.

The legacy analytical linkage has an adapter into this graph and is regression
checked against the graph solution. New mechanisms should use the graph API;
the analytical solver remains as an independently testable reference for the
single-layout path.

## Tolerances

- length comparisons: `0.01 mm` where practical
- angular convergence: `1e-5 rad`
- floating-point equality: never exact, always tolerance-based
