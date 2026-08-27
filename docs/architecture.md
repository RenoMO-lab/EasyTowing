# Architecture Proposal

## Goal

Build an engineering application where the calculation core is independent from the UI:

- trailer geometry and rigid-body model;
- ideal steering and ICR solver;
- mechanical linkage solver;
- collision and clearance analysis;
- optimization;
- visualization and export.

## Current implementation shape

Implemented so far:

- geometry primitives and ideal steering solver;
- arbitrary-axle vehicle layouts with persisted axle metadata;
- rigid-link planar linkage solver;
- editable linkage configuration with neutral branch establishment;
- clearance and overlap analysis;
- deterministic pure-Python linkage optimizer;
- SVG browser demo with live beta slider;
- project/revision persistence with save, load, restore, geometry, vehicle, and linkage configuration;
- export bundle helpers for JSON, CSV, PDF, SVG sketch output, and DXF sketch output.
- steering-curve sweep SVG preview for ideal, baseline, and optimized linkage response;
- swept-path preview for wheel-center trajectories and body extents.
- manual DXF import assignment workflow with entity suggestions and a reconstructed parametric layout preview.
- articulated body-chain primitives for future multi-trailer coordination.

## Recommended target architecture

- Frontend: React / Next.js / TypeScript
- API: FastAPI
- Core math: Python
- Geometry and optimization: Python numeric libraries later, starting with pure deterministic math
- Visualization: SVG-first 2D top view
- Export: CSV, SVG, DXF, JSON

## Why this separation matters

- The ideal steering math must be unit tested without a browser.
- The eventual linkage and optimization solvers will need repeatable inputs and outputs.
- The UI should only render results and capture design intent.

## Phase breakdown

1. Mathematical specification and data model
2. Ideal steering core
3. Minimal interactive top-view prototype
4. Mechanical linkage solver
5. Actual vs ideal comparison
6. Collision and clearance checks
7. Optimization
8. Dimensioned engineering output
9. Import/export workflows

## Proposed folder structure

```text
docs/
  audit.md
  architecture.md
  steering-model.md
  domain-model.md
  roadmap.md
 easytowing/
   __init__.py
   __main__.py
   collision.py
   errors.py
   geometry.py
   linkage.py
   optimization.py
   model.py
   steering.py
   web/
     app.js
     index.html
     styles.css
 tests/
   test_collision.py
   test_linkage.py
   test_optimization.py
   test_steering.py
```

The repository uses the root `easytowing/` package as its canonical source, as
declared by `pyproject.toml`. The historical `src/easytowing/` path is retained
only as a compatibility redirect for older `PYTHONPATH` setups.

## Current scope boundary

- Implemented now: geometry primitives, arbitrary-axle ideal steering, editable primary/companion linkage solver, collision analysis, optimization core, analytical validation, project history, browser exports, manual DXF assignment with parametric reconstruction, and body-chain primitives.
- Deferred: full multi-trailer coordination and generalized multi-axle mechanical linkage networks.
