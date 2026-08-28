# SaaS Operations

## Runtime modes

`GET /api/health` is a liveness check for the HTTP process and reports the
selected backend and the configured artifact-storage mode. It does not prove
PostgreSQL connectivity, worker health, or that a response-only artifact mode
retains files.

`GET /api/ready` is the dependency readiness check. In PostgreSQL mode it runs
`SELECT 1` against the configured database and returns HTTP 503 when the
database is unavailable. When `EASYTOWING_REQUIRE_ARTIFACT_STORAGE=1`, it also
returns HTTP 503 unless artifact retention is configured. Set
`EASYTOWING_REQUIRE_WORKER=1` to make it also require a fresh PostgreSQL worker
heartbeat. The freshness window defaults to 120 seconds and can be changed with
`EASYTOWING_WORKER_MAX_AGE_SECONDS`. Local mode reports its in-process runner as
healthy; the external worker gate applies only to PostgreSQL mode.

The local demo defaults to the JSON project store and in-memory control plane.
Set `EASYTOWING_DATABASE_URL` and install `psycopg[binary]` to run the same HTTP
server with PostgreSQL-backed projects, revisions, sessions, memberships,
approvals, jobs, and audit events. Database mode enables authentication
automatically; `EASYTOWING_AUTH_REQUIRED=1` can also require authentication in
JSON/in-memory mode.

The `Reference Demo Project` is seeded only by the dependency-free local demo.
PostgreSQL tenants are intentionally empty on first login, so simulation data
cannot be mistaken for customer engineering input.

The control-plane schema is [postgres_schema.sql](../easytowing/postgres_schema.sql).
It contains organizations, users, sessions, project memberships, revision
approvals, engineering jobs, artifact-delivery metadata, and append-only audit
events. Every control-plane record carries an organization ID for tenant
isolation.

## Reproducible deployment profile

The repository includes a production-shaped local deployment profile in
`ops/docker-compose.yml`. It runs PostgreSQL, the API, and the engineering
worker as separate services, with durable database and artifact volumes. Copy
`ops/.env.example` to `ops/.env`, replace the placeholders, then run from the
repository root:

```powershell
docker compose --env-file ops/.env -f ops/docker-compose.yml up --build
```

The API intentionally remains unready until the worker has published a fresh
heartbeat. This catches the common failure where the web process is alive but
queued engineering jobs cannot execute. The compose profile is a reproducible
pilot/deployment baseline, not a complete production platform: terminate TLS
at a controlled ingress, use a managed secret store, configure external
object storage, and supervise/monitor the services according to the Monroc
IT operating standard.

The PostgreSQL schema also enforces tenant ownership at the database boundary:
user sessions, project revisions, memberships, approvals, jobs, artifacts, and
audit actors use composite foreign keys containing `organization_id`. The
active-revision relationship is deferred so a project and its first revision
can be inserted atomically. Do not bypass the migration script with ad-hoc
single-column relationships.

## CAD import boundary

`POST /api/import.dxf` first returns a diagnostic parse. It detects the optional
DXF `$INSUNITS` header, reports supported and unsupported entities, and never
assumes that an undeclared CAD coordinate frame is the Monroc model frame.
Clients must send `source_units`, `coordinate_system`, and
`confirm_metadata: true` together with role assignments before geometry can be
activated. Units are converted to millimetres and the selected frame can mirror
X and/or Y. Any unsupported entity blocks activation so that a partial CAD file
cannot become an apparently complete vehicle. The applied vehicle configuration
retains the source name, SHA-256, units, scale, and coordinate frame for review.

After the confirmed layout is saved as a revision, an authenticated designer or
administrator can attach the exact source bytes with
`POST /api/projects/{project_id}/revisions/{revision_id}/cad-source`. The server
recomputes the SHA-256, requires it to match the revision metadata, and records
the file as a revision-scoped `cad-source-dxf` artifact. This endpoint requires
durable artifact storage; response-only mode intentionally refuses source-byte
retention. The existing artifact listing and download endpoints expose the
retained source to reviewers.

Closed CAD body outlines are validated as simple, non-zero-area polygons before
they can become an active vehicle model. Duplicate vertices, overlapping edges,
and self-intersections leave the import in preview-only state with a diagnostic
warning; do not work around this by entering a simplified outline without
recording the source CAD and review decision.

Controlled artifact bytes can be retained by setting
`EASYTOWING_ARTIFACT_STORAGE_DIR` to a protected directory. The server writes
release bytes atomically and verifies their SHA-256 and byte count when they are
downloaded. `GET /api/projects/{project_id}/revisions/{revision_id}/artifacts`
lists retained artifacts, and appending the artifact ID to that path downloads
one retained file. This is a dependency-free filesystem adapter for pilot and
single-node deployments; production should replace it with an encrypted,
access-controlled object-storage adapter.

## First administrator

Set a one-time `EASYTOWING_BOOTSTRAP_TOKEN` out of band and call
`POST /api/auth/bootstrap` with an organization ID, email, and password. The
endpoint always creates an administrator, and it rejects the organization after
the deployment has any user, including concurrent bootstrap attempts. This is
the single first-tenant provisioning operation; use authenticated admin user
management for subsequent accounts. Remove the bootstrap token after the first
administrator has been created.
Passwords are hashed with scrypt and bearer tokens are stored only as SHA-256
hashes in the control-plane session record. The PostgreSQL adapter runs the
schema migration on server startup; production deployments should run schema
migrations as a controlled release step instead.

## Approval boundary

Administrators can assign an active reviewer or administrator to a revision
through `POST /api/projects/{project_id}/revisions/{revision_id}/reviewer`.
The assignment is tenant-scoped and audited; once assigned, only that reviewer
can decide the revision. Designers submit a revision. A reviewer or
administrator must approve it, and a designer cannot approve their own
submission. The engineering verdict remains separate from workflow approval: a
failed or incomplete hard-check result is never an approved manufacturing
design. The revision `release.json` endpoint is
fail-closed and produces a controlled release manifest only when the saved
revision has engineering PASS and independent approval. Other revision JSON,
CSV, and PDF files remain diagnostic evidence.

The controlled release response is recorded in `artifact_records` with its
revision, artifact ID, filename, byte count, SHA-256, actor, and generation
time. The response exposes the ID and checksum as headers and the manifest
contains the ID and generation time. Without `EASYTOWING_ARTIFACT_STORAGE_DIR`,
the backend remains `response-only`, which records delivery metadata but does
not retain bytes. Set `EASYTOWING_REQUIRE_ARTIFACT_STORAGE=1` in a deployment
that must block release until retention is configured.

Monroc acceptance is fail-closed. A designer can evaluate trial limits for
diagnostics, but approval requires the exact criteria to match an approved
organization profile configured in the protected
`EASYTOWING_MONROC_ACCEPTANCE_PROFILES_JSON` environment variable. The value is
a JSON object keyed as `<organization_id>:<case_id>`, with the same criteria
fields accepted by the acceptance endpoint. If the variable is absent, invalid,
or the submitted limits differ, the result is `UNAPPROVED` and controlled
release remains blocked.

## Jobs and backups

Optimization jobs are submitted asynchronously and can be polled at
`GET /api/jobs/{job_id}`. In PostgreSQL mode the API only inserts a queued job;
it does not execute engineering work in the web process. Run a separate worker
process with:

```powershell
$env:EASYTOWING_DATABASE_URL = "<protected PostgreSQL DSN>"
python -m easytowing.worker --worker-id easytowing-worker-01
```

Workers claim jobs with PostgreSQL row locks and `SKIP LOCKED`, persist terminal
results, and return abandoned running jobs to the queue after the configured
lease period. Each claim has a unique worker lease token, so a stale worker
cannot overwrite a replacement worker's result. Workers publish a heartbeat
while idle, running, and after job completion. A deployment using
`EASYTOWING_REQUIRE_WORKER=1` should run at least one supervised worker and
alert on `/api/ready` returning HTTP 503. Local development keeps the
in-process runner so the demo remains easy to start.

Use [backup-postgres.ps1](../ops/backup-postgres.ps1) with a protected
`DATABASE_URL` and schedule it outside the application process. Store dumps on
a separate protected volume and periodically perform a restore drill into an
isolated database before relying on the backup as an operational control. The
drill restores without `--clean`, then verifies the control-plane tables and
row counts:

```powershell
.\ops\restore-drill-postgres.ps1 `
  -BackupPath .\backups\easytowing-20260827-120000.dump `
  -RestoreDatabaseUrl "<isolated PostgreSQL DSN>"
```
