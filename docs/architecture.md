# Architecture

## Product goal

EasyTowing is an engineering tool for defining and checking steering axes on
single-body and articulated towing combinations. The calculation boundary must
remain independent from the browser so that every result can be replayed from
saved geometry, mechanism inputs, and the selected maneuver.

## Current implementation

The root `easytowing/` package currently contains:

- arbitrary-axle ideal steering and common-ICR calculation;
- articulated body-chain inputs with explicit joint closure checks;
- a generalized planar fixed-length mechanism graph with named wheel-output
  mappings;
- actual-versus-ideal steering comparison for graph-mapped wheels;
- centralized clearance, point-envelope, body-beam, and tire checks;
- hard feasibility rules that prevent an infeasible optimization result from
  being treated as an accepted design;
- bounded Cartesian full articulation-range sweep validation for every
  articulation joint, with explicit ranges overriding safe defaults and
  per-pose failure reasons;
- project/revision persistence for vehicle, combination, mechanism, mapping,
  and diagnostic snapshot data;
- JSON, CSV, PDF, SVG, and DXF reporting helpers;
- a guided browser workspace that exposes the engineering sequence instead of
  presenting raw solver controls first;
- local SaaS control-plane primitives for tenant, role, session, approval,
  asynchronous-job, and audit behavior;
- a PostgreSQL control-plane schema, adapter foundation, and backup operation.

The demo runtime is intentionally small: a Python standard-library HTTP server
and browser JavaScript. Without `EASYTOWING_DATABASE_URL` it uses JSON projects
and in-memory SaaS controls for local validation. With that variable and the
PostgreSQL extra installed, the same HTTP routes use the PostgreSQL project and
control-plane adapters.

## Calculation flow

1. Define the bodies, axles, wheel geometry, and each joint's physical
   articulation stop.
2. Resolve the selected maneuver into a common instantaneous center of rotation.
3. Solve the explicit mechanism graph for the current articulation pose.
4. Map named graph outputs to named wheels and compare actual steering with the
   ideal target.
5. Evaluate member, point, body, tire, and wheel clearance.
6. Repeat the hard checks across the full articulation range.
7. Save the complete revision evidence before any review decision.

Mechanism graph feasibility also requires topology, not only numeric closure:
each connected component must include a fixed or driven reference point. A
free-only closed loop is rejected because its internally consistent geometry is
not physically tied to the vehicle or the steering input.

For articulated combinations, "full range" means the Cartesian product of the
signed range for every articulation joint. Explicit joint ranges override the
configured/default bounds; an omitted joint is never silently held at nominal.
Each sampled joint angle is also checked against that joint's physical stop;
the sweep range is not a substitute for the stop and an out-of-stop sample is
a hard failure.
The request is rejected as incomplete when the grid exceeds its sample budget;
the server never truncates a multi-joint sweep without reporting failure.

A failed hard check is diagnostic evidence only. It is not an approved
manufacturing design.

## Production target

The production boundary should be split into:

- Frontend: a typed application with the same guided workflow and explicit
  review states;
- API: a production HTTP service with request validation, authorization, and
  job queues;
- Core math: the current deterministic Python package, extended with validated
  multi-trailer and mechanism cases;
- Persistence: PostgreSQL for organizations, users, projects, revisions,
  approvals, jobs, and audit events, with object storage for CAD/report files;
- Visualization/export: SVG/Canvas previews and controlled engineering output.

The current repository wires the live demo server to PostgreSQL when configured
and provides a separate durable worker command for engineering jobs. It also
provides a checksum-verified filesystem adapter for pilots and an optional
S3-compatible object-storage adapter; production process supervision, managed
identity integration, and the cloud deployment policy remain outside this
repository.

## Current scope boundary

Implemented now: explicit multi-body kinematics, generalized graph solving,
named steering mappings, hard clearance feasibility, range sweeps, revision
snapshots, diagnostic reporting, and local approval/job/audit primitives.

Still required for a Monroc operational release:

- production identity, secret management, deployment, monitoring, and backup
  restore drills;
- production worker supervision and deployment of the configured object-storage
  backend for generated CAD/report files;
- graph-native topology/geometry optimization beyond the current bounded
  driver-and-wheel-mapping optimizer;
- richer CAD feature import/assignment, review/approval, and failure guidance;
- validated multi-trailer coordination and real Monroc CAD/hand-calculation
  acceptance cases;
- Monroc-approved thresholds for steering error, clearance, envelopes, and
  manufacturing release.
