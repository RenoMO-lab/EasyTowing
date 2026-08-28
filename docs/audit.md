# Repository Audit

Date: 2026-08-28

## Verdict

The result is fit for an engineering calculation prototype and a structured
Monroc pilot discussion. It is not yet fit for production use by the Monroc
design department as an operational SaaS/tool for releasing towing-axis
designs.

The main reason is not that the solver is absent. The calculation path now
exists, including explicit multi-body inputs, a generalized mechanism graph,
hard clearance checks, and bounded Cartesian multi-joint full-range validation.
The remaining issue is that
the product does not yet provide enough domain validation, deployment integrity,
or release control to make a result trustworthy for real design work.

## Proposed operating flow

The tool should be used as a gated engineering workspace, not as a steering
angle calculator:

1. **Project:** open a vehicle study and create a named revision.
2. **Vehicle:** enter every rigid body, axle, tire envelope, articulation joint,
   and available CAD-derived outline. Multi-trailer combinations belong here.
3. **Maneuver:** define the signed root turn radius and the articulation ranges
   to be studied.
4. **Mechanism:** define the physical linkage graph, drivers, rigid members, and
   the wheel outputs each component controls.
5. **Validate:** solve the current pose, then sweep the configured joint grid.
   A result is only physical-feasibility PASS when closure, collision, and
   clearance checks pass for the required range. Steering accuracy and
   synchronization are classified separately against approved Monroc limits.
6. **Optimize:** generate a constrained proposal only when the mechanism model
   supports it. An infeasible proposal is diagnostic and cannot be applied as
   the accepted design.
7. **Results and review:** compare ideal versus actual wheel headings, inspect
   residuals and swept paths, save the evidence, evaluate signed-off Monroc
   criteria, and submit the saved revision for independent approval.

The result is interpreted as follows: **PASS** means the configured
physical-feasibility checks pass; **FAIL** means the design is diagnostic only;
**INCOMPLETE** means the required validation evidence has not been produced.
Steering accuracy and synchronization remain separate acceptance checks.
Manufacturing release also requires Monroc acceptance and independent approval,
so a physical-feasibility PASS alone is never a release authorization.

## What is usable

- A user can define bodies, axles, articulation limits, mechanism geometry, and
  named wheel mappings in the guided workspace.
- Each body exposes an optional CAD-outline editor using local perimeter points;
  blank input is explicitly labeled as the rectangular length/width fallback,
  and malformed outlines block maneuver resolution instead of being silently
  accepted. Polygon input now rejects zero-area, duplicate, overlapping, and
  self-intersecting outlines before collision calculations. Mechanism body
  placement and output reference bodies use choices from the active combination
  rather than free-text IDs.
- DXF import is now preview-only until source units and the CAD-to-model axis
  frame are explicitly selected. Supported geometry is rescaled into mm,
  mirrored when requested, omitted entity types block activation, and the
  applied vehicle retains the source filename, SHA-256, units, scale, and frame.
- A saved confirmed DXF can now be attached as a revision-scoped source artifact;
  the server verifies the exact SHA-256 before retention and the UI distinguishes
  durable source bytes from metadata-only response mode.
- Each articulated joint now carries an explicit physical stop that is enforced
  during direct solves, Cartesian multi-joint sweeps, and optimizer candidates;
  an over-wide sweep is retained as a visible failed engineering case.
- The system can resolve a multi-body maneuver, solve a fixed-length mechanism,
  compare actual and ideal steering, and report residuals and clearance.
- Body-mounted mechanism points preserve branch continuity in their owning
  body's local coordinates while the combination articulates, avoiding false
  branch-change failures caused only by rigid-body motion.
- The full articulation range can be swept and the first failing pose and
  failure codes are retained in the revision snapshot. When more than one
  joint range is configured, every Cartesian pose is evaluated; oversized
  grids fail closed rather than being silently truncated.
- Failed calculations are visibly diagnostic and cannot be treated as accepted
  optimized designs.
- Graph collision connectivity permits contact at a shared mechanism joint but
  still reports overlapping or duplicate connected members as hard collisions.
- Saved revisions preserve the engineering inputs and can produce JSON, CSV,
  PDF, dimensioned SVG, ASCII DXF, and PNG evidence from the same multi-body
  snapshot. These artifacts are diagnostic geometry, not an approved CAD
  manufacturing package.
- Multi-body combinations retain explicit steering coordination channels. The
  Vehicle step lets the designer define target axle, source axle, phase mode,
  ratio, and phase offset; Results reports ideal, actual, and error values for
  each saved channel.
- The Results step now presents one decision summary showing current-pose
  checks, full-range evidence, Monroc acceptance, independent approval, and the
  manufacturing-release gate together. The authentication controls are a real
  form and the validation control stays disabled unless the multi-body graph is
  actually active and solved.
- The Validate step labels its four current checks as physical-feasibility
  checks and shows the measured ideal-versus-actual steering error separately.
  Steering acceptance remains `PENDING` until signed-off Monroc limits are
  evaluated, preventing a feasibility PASS from being read as steering approval.
- The Project dashboard now surfaces the active revision's engineering verdict,
  review gate, and model scope with plain-language detail, so the designer can
  understand the current state before opening the detailed Results step.
- Diagnostic steering-curve and swept-path previews now show an in-page blocked
  state when current hard checks fail, instead of surfacing predictable API
  errors as browser resource failures. Optimization is explicitly user-triggered
  so opening a saved revision does not start an expensive or predictably
  infeasible search without the designer choosing it.
- Workflow navigation now reports `PASS`, `FAIL`, or `INCOMPLETE` for Results
  instead of showing `READY` merely because a diagnostic payload exists. This
  keeps the navigation verdict aligned with the engineering decision card.
- Results now label `Ideal`, `Actual`, and `Error` explicitly, including the
  synchronization table, so a designer does not need to decode solver
  abbreviations before interpreting a steering mismatch.
- Local demo workspaces receive a two-body reference combination with a
  configured articulation range, so the primary workflow starts in the
  multi-body path rather than silently defaulting to the legacy single-layout
  study.
- The reference project is seeded only in local development; PostgreSQL tenants
  start empty rather than receiving unowned simulation data.
- Applying graph edits invalidates prior calculation, sweep, optimization, and
  acceptance evidence and disables review submission until a new revision is
  saved. This prevents a changed mechanism from being represented by stale
  approval evidence.
- Engineering evaluation now fails closed when a multi-body snapshot has fewer
  than two bodies or omits a usable envelope for any body, so clearance cannot
  pass by silently skipping incomplete body geometry.
- Mechanism build and graph edits preserve the resolved maneuver state while
  invalidating mechanism evidence. Every subsequent design edit also clears
  the current result, preventing stale calculations from remaining visible.
- Automated browser checks verified the workflow at 1280 px and 390 px with no
  horizontal page overflow. Automated browser evidence is not a substitute for
  Monroc human usability sign-off.
- Restoring a saved project now opens the first incomplete workflow step instead
  of leaving the engineer on the Project dashboard while recommending a later
  mechanism or validation action. Combination edits also clear the previous
  maneuver status immediately, so stale body and axle counts are not presented
  as current evidence.
- The workspace now labels seeded inputs as simulation/reference data and places
  a plain-language PASS, FAIL, PENDING, and BLOCKED reading guide beside the
  engineering decision.
- An explicit multi-body maneuver now returns ideal body poses and wheel targets
  without falling through to the legacy linkage solver. Until the physical
  mechanism graph is built and solved, actual steering, collision, clearance,
  and engineering feasibility remain visibly PENDING rather than appearing as
  failed or completed evidence.
- A live browser check also resolved a synthetic three-body, three-axle
  combination, built three articulation drivers with six wheel mappings, and
  solved the generalized graph. This verifies the product path only; it is not
  evidence for a Monroc design.
- Runtime checks now distinguish HTTP liveness from dependency readiness. A
  PostgreSQL deployment can fail readiness when no worker heartbeat is fresh;
  worker supervision and object-storage health remain deployment responsibilities.
- A reproducible Docker Compose profile now separates the API, PostgreSQL, and
  durable worker processes with named database/artifact volumes. The stack was
  rebuilt and verified on 2026-08-28 with healthy API and database containers,
  clean API/worker logs, HTTP health 200, and dependency readiness 200. Schema
  startup is serialized with a PostgreSQL advisory lock so API and worker
  startup cannot race their first migration.
- An isolated live Compose run also bootstrapped an administrator, authenticated
  against PostgreSQL, completed a queued optimization through the separate
  worker at 100%, and restored a `pg_dump` into a fresh database. The restored
  database retained the organization, user, job result, and five audit events.
- Deployment bootstrap now creates only the first administrator and ignores
  caller-supplied roles; repeated or concurrent bootstrap attempts are rejected
  after the deployment has any user. PostgreSQL worker claims carry unique lease
  tokens, so a stale worker cannot overwrite a replacement worker's result after
  recovery requeues the job.
- API request boundaries reject non-object JSON bodies and require approval
  decisions to use actual JSON booleans, preventing truthiness coercion from
  turning malformed approval requests into approvals.
- Administrators can now provision tenant users from the Workspace access panel
  or `POST /api/users`; the organization is derived from the authenticated
  administrator, role values are validated, passwords are never serialized,
  and the creation event records the administrator actor.
- PostgreSQL persistence now has composite tenant-scoped foreign keys across
  sessions, projects, revisions, approvals, jobs, artifacts, and audit actors;
  application checks remain in place as the first authorization boundary.
- Verification on 2026-08-28: 220 automated tests passed with 3 expected
  PostgreSQL integration skips; Python compilation, browser JavaScript syntax,
  whitespace checks, and real-browser desktop/mobile workflow checks passed.

## Why it is not release-ready

- The new-study walkthrough now uses a physically separated reference graph and
  a deliberately narrow +/-15 degree demonstration range, so the hard physical
  checks pass without hiding collision or clearance failures. It remains
  simulation data, not Monroc geometry, and expanding the range must be tested
  against the actual approved mechanism and wheel limits.
- Steering-error acceptance is explicitly pending Monroc-approved thresholds.
- The browser graph editor now supports editable points, members, outputs,
  driver arcs, and wheel mappings, including shared point connections. It is
  still not a complete CAD mechanism editor: CAD feature recognition, automatic
  topology import, and production-grade geometry authoring remain outstanding.
- The reviewed DXF path supports common 2D entities and manual role mapping,
  but it is not a CAD-grade feature/topology importer. Unsupported entities are
  intentionally blocked rather than silently ignored, so real Monroc CAD still
  needs a representative-file pilot and mapping review.
- The reference combination is intentionally a solver demonstration with
  compact body envelopes around the axles and a linkage operating at the
  articulation gap. Its dimensions and mechanism are not Monroc CAD data and
  must not be used as a physical design reference.
- Graph-native optimization now covers bounded driver and wheel-mapping
  variables with hard Cartesian full-range feasibility. It is not a CAD
  topology or geometry optimizer.
- Axles support two-wheel defaults and explicit dual-wheel or other multi-wheel
  assemblies when each lateral wheel
  position is explicitly supplied. The mechanism graph maps all wheel
  positions to named outputs, while a steering-side command is applied to each
  tire in the same wheel end. Real Monroc wheel-end conventions still require
  pilot validation.
 - Local mode is a standard-library demo using JSON project storage and
   in-memory SaaS controls. Configured PostgreSQL mode now covers project,
   revision, session, membership, approval, job, and audit persistence. The
   protected filesystem adapter and optional S3-compatible adapter can retain
   controlled release bytes, but production deployment policy remains incomplete.
- Basic role-aware submit/approve/reject controls, approval history, and a
  release checklist now exist. Administrators can assign an active reviewer
  or administrator to a revision; assignment is tenant-scoped, audited, and
  enforced when the decision is made.
- Acceptance criteria are now fail-closed against a protected,
  organization-scoped approved profile. Without that deployment configuration,
  entered limits remain `UNAPPROVED` and cannot authorize release.
- No real Monroc CAD dataset, trusted manual calculation, or approved design
  threshold is checked in as an acceptance fixture.
- The repository now includes an executable, fail-closed pilot package
  validator. It requires those external artifacts and compares the saved
  result against both hand-calculation and approved-reference metrics, but it
  intentionally has no customer case data and cannot authorize release.
- Multi-body sweep evidence now retains complete per-pose steering series, and
  the pilot validator checks every selected wheel, axle-center, and
  synchronization value rather than only summary maxima.

## Observed user-flow problem

The original interface exposed calculation concepts before explaining the
engineering decision flow. The current guided sequence improves this by using:

1. Project context.
2. Vehicle and body definition.
3. Maneuver and articulation range.
4. Mechanism graph and wheel mapping.
5. Current-pose and full-range validation.
6. Optimization where the selected model supports it.
7. Results, revision evidence, and controlled review boundary.

This is a better prototype flow, but it still needs domain-specific labels,
examples, import assistance, and validated failure-resolution guidance from
Monroc users.

## Required acceptance work

- Use [the Monroc acceptance plan](monroc-acceptance-plan.md) to agree the
  criteria and case matrix before treating any result as a release candidate.
- Select representative single-body, two-body, and multi-trailer combinations.
- Capture trusted CAD dimensions and independent hand calculations.
- Agree on steering-error, clearance, collision, articulation, and export
  acceptance thresholds.
- Verify the same cases through the browser as a designer and as an independent
  reviewer.
 - Deploy the PostgreSQL-backed mode with production identity and worker
   supervision, configured encrypted object storage, monitoring, and an
   executed backup restore/audit-continuity procedure. The repository now
   provides the durable worker, checksum-verified filesystem/S3 retention, and
   restore-drill foundations, but not the production deployment controls
   themselves.
