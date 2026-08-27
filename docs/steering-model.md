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

## Prototype beta mapping

The initial browser demo uses a surrogate `beta -> radius` mapping only to drive an interactive visualization. It is not the final multi-body trailer articulation solver.

This temporary mapping is acceptable for the first geometric prototype because it keeps the steering math testable while the trailer articulation model is still being defined.

## Tolerances

- length comparisons: `0.01 mm` where practical
- angular convergence: `1e-5 rad`
- floating-point equality: never exact, always tolerance-based
