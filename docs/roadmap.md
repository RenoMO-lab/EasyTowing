# Development Roadmap

## Current status

- Completed in the current prototype: phases 0 through 9, covering the mathematical specification, ideal steering, browser visualization, rigid-link linkage, ideal-vs-actual comparison, collision and clearance analysis, optimization, steering-curve preview, swept-path preview, export helpers, DXF export, PDF engineering report output, basic DXF import parsing, dimensioned engineering sketch output, project history, and manual DXF entity assignment.
- Also implemented: arbitrary-axle ideal layouts, normalized vehicle configuration persistence, editable primary/companion linkage paths, and export/report reconstruction from saved geometry.
- Still deferred: full multi-trailer coordination and generalized multi-axle mechanical linkage networks.

## Phase 0

- Write the mathematical specification
- Define the domain model
- Establish tolerances and coordinate conventions

## Phase 1

- Implement ideal steering geometry
- Validate against analytical Ackermann cases
- Keep the solver independent from the browser

## Phase 2

- Add a minimal interactive top-view prototype
- Show body, axles, wheels, ICR, and steering rays

## Phase 3

- Add rigid-link mechanical steering model
- Maintain fixed-length constraints
- Track branch continuity

## Phase 4

- Compare ideal vs actual steering
- Add error metrics and plots

## Phase 5

- Add collision and clearance checks

## Phase 6

- Add optimization variables, objectives, and constraints
- Implement deterministic pure-Python search
- Expose quick/full optimization runs in the demo UI

## Phase 7

- Generate dimensioned engineering drawings

## Phase 8

- Add DXF import workflows and geometry assignment

## Phase 9

- Add swept-path simulation and report generation
