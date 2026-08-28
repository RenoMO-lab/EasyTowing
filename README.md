# EasyTowing

Engineering foundation for multi-axle trailer steering design, simulation, and optimization.

Current status:

- Repository audit, architecture notes, and the steering model specification are documented.
- Dependency-free Python engineering core is implemented.
- Arbitrary-axle ideal steering, editable primary/companion rigid-link linkage, clearance analysis, deterministic optimization, and export bundle generation are implemented.
- SVG browser demo includes live articulation, project creation, revision history, and restore.
- The browser linkage editor feeds live kinematics, optimization, project snapshots, and engineering exports.
- Steering-curve sweep preview shows ideal and actual linkage response across the full articulation range.
- Swept-path preview and PDF engineering reports are available from the browser export links.
- Reviewed DXF import assignment and parametric reconstruction are available from the browser; activation requires explicit source units and CAD axis-frame confirmation, and unsupported entities block activation.
- Confirmed DXF source bytes can be retained against a saved revision when durable artifact storage is configured; the source hash is checked before retention.
- Articulated body combinations and a generalized planar mechanism graph are available for explicit multi-body steering studies, including per-joint physical articulation stops and Cartesian sweep validation.
- Explicit multi-wheel axle assemblies are supported when each lateral wheel position is supplied; the standard two-wheel case retains conventional left/right defaults.
- Multi-body revisions retain explicit steering coordination channels and export dimensioned diagnostic geometry as SVG, ASCII DXF, and PNG alongside JSON, CSV, and PDF evidence.
- Persistent project state is stored in `.easytowing-state/projects.json`.
- SaaS control-plane primitives cover authenticated sessions, tenant/role checks, assigned approvals, asynchronous jobs, audit events, and PostgreSQL schema/backup operations; local mode remains JSON/in-memory while database mode is enabled with `EASYTOWING_DATABASE_URL`.
- Authenticated administrators can provision tenant users with role assignment from the Workspace access panel; creation is tenant-scoped and audited, and new reviewers are available for revision routing immediately.
- Runtime health exposes separate liveness (`/api/health`) and dependency readiness (`/api/ready`) checks; PostgreSQL deployments can require a fresh worker heartbeat with `EASYTOWING_REQUIRE_WORKER=1`.
- Multi-body revision exports are diagnostic by default; a controlled release manifest is available only after engineering PASS, Monroc acceptance, and independent approval. SVG/DXF/PNG do not constitute a controlled CAD release package.
- Articulated acceptance includes the saved physical-feasibility gate, so an acceptance result cannot pass while combination kinematics, mechanism closure, collision, or clearance evidence is failing.
- Monroc acceptance evaluations remain diagnostic unless their exact criteria match a protected approved profile configured with `EASYTOWING_MONROC_ACCEPTANCE_PROFILES_JSON`.
- A separate `python -m easytowing.pilot` validator is ready for supplied Monroc case packages with hashed CAD, hand-calculation, and approved-reference evidence; pilot results never authorize release by themselves.
- Multi-body sweep samples retain per-pose ideal, actual, error, axle-center, and synchronization series so pilot comparisons are not limited to aggregate metrics.
- Controlled release delivery metadata is recorded with an artifact ID and SHA-256. Set `EASYTOWING_ARTIFACT_STORAGE_DIR` for atomic filesystem retention or `EASYTOWING_ARTIFACT_S3_BUCKET` for the bundled checksum-verified, server-side-encrypted S3-compatible adapter and authenticated artifact downloads.
- Analytical tests cover steering, linkage, collision, optimization, reporting, and project storage.

Remaining planned extensions:

- validation of chained multi-trailer coordination against Monroc references
- CAD-grade shared-network topology and geometry editing
- production deployment policy, identity integration, monitoring, and executed backup-restore drills
- Monroc-approved acceptance thresholds and pilot validation against real CAD

## Local run

```bash
python -m pip install -e .
python -m unittest discover -s tests -v
python -m easytowing --port 8000
```

Open `http://127.0.0.1:8000` in a browser after starting the demo server.
