# Optimization Model

## Purpose

Search over selected linkage dimensions or bounded graph driver/mapping
parameters to reduce steering error while respecting fixed rod lengths,
clearance constraints, and branch continuity.

## Inputs

- Baseline linkage rig
- Enabled optimization variables with bounds
- Beta sweep samples
- Minimum clearance target
- Objective weights
- Optimization mode: `quick` or `full`
- Optional axle synchronization channels and independent target curves

## Variables

The legacy optimizer supports scalar linkage variables with:

- `id`
- `current`
- `minimum`
- `maximum`
- `enabled`
- `preferred`

Supported parameter names include:

- bell crank pivot X/Y
- steering pivot X/Y
- bell crank input arm length
- bell crank output arm length
- steering arm length
- input rod length
- tie rod length

## Objective

The score combines:

- squared steering error against ideal wheel headings
- synchronization error between actual and ideal axle phase relationships
- minimum-clearance penalties
- failure penalties for unsolved articulation samples
- preferred-value regularization
- complexity regularization

Lower is better.

## Search strategy

The current implementation uses a deterministic pure-Python coordinate search with:

- seeded random exploration
- progressive step shrinkage
- per-variable bound clamping
- quick/full iteration presets

This keeps the optimizer dependency-free for the current repo while preserving the future ability to swap in SciPy-based solvers later.

## Graph optimization

`graph_optimization.py` provides a separate bounded search for an articulated
`VehicleCombination` and `PlanarMechanismGraph`. It can tune driver input ratios,
driver phase offsets, wheel-assignment ratios, and wheel-assignment phase
offsets. Every candidate is solved over the configured articulation samples;
mechanism errors, steering stops, body/link/wheel collisions, and minimum
clearance are hard feasibility constraints. It does not alter graph topology or
CAD geometry, and a failed candidate cannot be applied as a revision.
