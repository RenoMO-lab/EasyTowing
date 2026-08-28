# Development Roadmap

## Current status

The repository is an engineering prototype with a usable calculation path for
explicit multi-body combinations and generalized planar mechanisms. It is not
yet a production SaaS product or an approved Monroc design-release system.

## Completed foundation

- Mathematical specification, coordinate conventions, and tolerances.
- Ideal steering for arbitrary axle layouts.
- Articulated body-chain inputs and common-ICR combination solving.
- Fixed-length planar linkage solver and generalized mechanism graph solver.
- Named mechanism-output-to-wheel assignments.
- Actual-versus-ideal steering comparison.
- Centralized collision and clearance analysis.
- Hard feasibility-aware legacy optimization.
- Full articulation-range sweep diagnostics.
- Explicit Cartesian multi-joint range sweeps with bounded, fail-closed sample
  budgets.
- Guided browser workflow, project revisions, and saved diagnostic snapshots.
- Guided multi-body body-outline input with a rectangular fallback and
  fail-closed malformed-outline feedback.
- Workflow navigation reports the actual Results state (`PASS`, `FAIL`, or
  `INCOMPLETE`) instead of equating an available diagnostic with a release-ready
  design.
- The Project dashboard surfaces engineering verdict, review gate, and active
  model scope before the detailed workflow panels.
- Results explicitly label ideal target, mechanism output, and signed error in
  degrees for wheel and synchronization comparisons.
- JSON/CSV/PDF reporting plus saved multi-body dimensioned SVG, ASCII DXF, and
  PNG diagnostic exports generated from one revision snapshot.
- Supported DXF import now requires explicit source-unit and CAD-axis-frame
  confirmation, rejects omitted entity types, and retains a source hash and
  transform metadata with the applied vehicle configuration.
- Confirmed DXF source bytes can be retained against a saved revision through the
  authenticated artifact boundary with checksum verification.
- Explicit multi-body steering synchronization channels carried through
  parsing, calculation, sweep, revision serialization, and the guided UI.
- Local tenant, role, approval, job, audit, PostgreSQL schema, and backup
  foundations.

## Release blockers

- Monroc-approved acceptance thresholds and representative CAD/hand-calculation
  cases are not defined in the repository.
- The default demo graph is intentionally a reference mechanism and currently
  produces visible clearance and steering-limit failures across part of the
  range. This is useful diagnostic behavior, not a release result.
- The browser graph builder creates a repeatable reference graph per steerable
  axle; it does not yet replace a CAD-grade editor for arbitrary shared
  tie-rod networks.
- Graph-native optimization now covers bounded driver and wheel-mapping
  variables with hard full-range feasibility; CAD topology/geometry search and
  sensitivity analysis remain incomplete.
- Diagnostic SVG/DXF/PNG exports now show the saved multi-body geometry, but
  they are not yet controlled CAD/manufacturing deliverables.
- Local mode still uses JSON projects and in-memory SaaS controls; configured
  PostgreSQL mode is wired and has passed adapter and HTTP round-trip checks.
- Production deployment and worker supervision, cloud object storage, identity
  integration, and executed restore drills remain incomplete. A durable
  PostgreSQL worker command and a checksum-verified filesystem artifact adapter
  now exist for the pilot deployment slice.
- Basic role-aware review controls, approval history, a release checklist, and
  tenant-scoped reviewer assignment now exist. Monroc-specific acceptance
  evidence remains incomplete.

## Next delivery slices

### Pilot specification

- Review and complete [the Monroc acceptance plan](monroc-acceptance-plan.md).
- Capture Monroc terminology, approval roles, project metadata, and release
  rules.
- Agree on steering-error, clearance, envelope, articulation, and axle-load
  thresholds.
- Import two representative towing combinations and reproduce trusted manual
  results.

### Engineering core

- Validate chained multi-trailer closure against CAD and hand calculations,
  including representative simultaneous joint-range cases.
- Extend the graph builder to shared tie rods, multiple bell cranks, and
  cross-body mechanisms.
- Extend graph-native design variables to validated geometry/topology choices
  and add sensitivity outputs.

### Product workflow

- Extend reviewed CAD import beyond the supported DXF entity set, with Monroc
  feature recognition, shared mechanism topology mapping, and a signed source
  file reference.
- Pilot the reviewed DXF path against representative Monroc files and record
  approved coordinate conventions, mapping responsibilities, and rejection
  rules.
- Extend role-aware submit/review/approve/reject UI with reviewer notifications
  and Monroc-specific release gating.
- Extend first-failure guidance per pose with validated geometry/action paths.
- Add controlled manufacturing CAD export after engineering PASS and approval;
  the current controlled release manifest is the audit boundary, not a CAD
  release package.

### SaaS deployment

- Replace the pilot filesystem artifact adapter with encrypted object storage
  for generated CAD/report artifacts and configure
  `EASYTOWING_REQUIRE_ARTIFACT_STORAGE=1`.
- Add production identity integration, worker supervision, observability,
  secret management, backup restore drills, and deployment automation.
