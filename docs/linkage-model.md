# Linkage Model

## Purpose

Model rigid planar steering linkages independently from the ideal steering solver.

## Current solver shape

The current linkage solver models a primary steered axle with an optional companion
steered axle. Each path uses two analytic circle-intersection stages:

1. Driver point to bell-crank input arm.
2. Bell-crank output arm to steering arm.

This is enough to validate:

- fixed link lengths;
- branch continuity via previous-state selection;
- impossible mechanism detection;
- traceable residuals.

Vehicle layouts may contain any number of axles for ideal steering and export.
The rigid-link solver currently has explicit primary and companion paths rather
than a generalized multi-axle tie-rod network. Other axles can still receive
explicit `SAME_PHASE`, `OPPOSITE_PHASE`, `RATIO`, or `INDEPENDENT_TARGET`
commands, and the `LINKED_MECHANICALLY` mode is retained as a traceable
coordination channel until its fixed-length component graph is implemented.

## Data flow

- The caller provides a driver point for each articulation state.
- The solver returns the chosen input endpoint, output endpoint, and steering endpoint.
- The returned state includes rod-length residuals and branch indices.

## Next extension points

- generalized multi-axle mechanical channels with multiple bell cranks and shared tie rods;
- generalized fixed-length multi-axle component graphs;
- collision envelopes for additional generalized rods and arms;
- optimization variables on additional mechanism components.
