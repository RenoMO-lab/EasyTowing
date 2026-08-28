# Linkage Model

## Purpose

Model rigid planar steering mechanisms independently from the ideal steering
solver. The model is intended to make every mechanism component and every wheel
assignment explicit and traceable.

## Current solver shape

The generalized `PlanarMechanismGraph` supports:

- fixed, driven, and free mechanism points;
- rigid members with fixed lengths;
- multiple connected members and shared joints;
- body-local points transformed by articulated body pose;
- named angle outputs and named wheel assignments; cross-body outputs are
  measured in the endpoint body frame by default, or an explicit reference
  body frame;
- deterministic nonlinear closure solving with residual reporting;
- collision exclusions derived only from genuinely connected members, with a
  connected pair exempted only at its shared joint rather than across the full
  member envelopes.

The legacy primary/companion linkage remains available as a compatibility model
and as the current optimization target. It can be adapted into the generalized
graph and is regression-tested against the analytical linkage behavior.

## Data flow

1. The combination solver resolves body poses and the maneuver ICR.
2. Named mechanism drivers provide input points for each articulation state.
3. The graph solver resolves all free point coordinates while preserving member
   lengths.
4. Named angle outputs map mechanism motion to explicit wheels.
5. The actual-steering and clearance layers evaluate the same solved state.

## Current product limitation

The graph core is generalized, but the browser builder currently generates a
repeatable reference graph per steerable axle. It does not yet provide a full
CAD-grade editor for arbitrary shared tie-rod networks or optimized graph
geometry. Those capabilities require pilot geometry and Monroc acceptance cases
before they should be treated as release functionality.
