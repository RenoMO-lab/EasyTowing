-- PostgreSQL persistence contract for the control plane and engineering data.
CREATE TABLE IF NOT EXISTS organizations (
    id text PRIMARY KEY,
    name text NOT NULL,
    created_at timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS users (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    email text NOT NULL,
    display_name text NOT NULL,
    role text NOT NULL CHECK (role IN ('viewer', 'designer', 'reviewer', 'admin')),
    password_hash text NOT NULL,
    active boolean NOT NULL DEFAULT true,
    created_at timestamptz NOT NULL DEFAULT now(),
    UNIQUE (organization_id, email),
    CONSTRAINT users_org_id_key UNIQUE (organization_id, id)
);

CREATE TABLE IF NOT EXISTS user_sessions (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    user_id text NOT NULL,
    token_hash text NOT NULL UNIQUE,
    created_at timestamptz NOT NULL,
    expires_at timestamptz NOT NULL,
    revoked_at timestamptz,
    CONSTRAINT user_sessions_org_user_fk
        FOREIGN KEY (organization_id, user_id) REFERENCES users(organization_id, id)
);

CREATE TABLE IF NOT EXISTS projects (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    name text NOT NULL,
    created_at timestamptz NOT NULL,
    updated_at timestamptz NOT NULL,
    active_revision_id text,
    CONSTRAINT projects_org_id_key UNIQUE (organization_id, id)
);

CREATE TABLE IF NOT EXISTS project_revisions (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    project_id text NOT NULL,
    created_at timestamptz NOT NULL,
    payload_json jsonb NOT NULL,
    UNIQUE (project_id, id),
    CONSTRAINT project_revisions_org_project_fk
        FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE,
    CONSTRAINT project_revisions_org_project_id_key UNIQUE (organization_id, project_id, id)
);

CREATE TABLE IF NOT EXISTS project_memberships (
    project_id text NOT NULL,
    organization_id text NOT NULL REFERENCES organizations(id),
    created_at timestamptz NOT NULL DEFAULT now(),
    PRIMARY KEY (project_id, organization_id),
    CONSTRAINT project_memberships_org_project_fk
        FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS revision_approvals (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    project_id text NOT NULL,
    revision_id text NOT NULL,
    assigned_reviewer_id text,
    status text NOT NULL CHECK (status IN ('draft', 'submitted', 'approved', 'rejected')),
    submitted_by text,
    submitted_at timestamptz,
    decided_by text,
    decided_at timestamptz,
    decision_note text NOT NULL DEFAULT '',
    CONSTRAINT revision_approvals_org_project_fk
        FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE,
    CONSTRAINT revision_approvals_org_revision_fk
        FOREIGN KEY (organization_id, project_id, revision_id) REFERENCES project_revisions(organization_id, project_id, id) ON DELETE CASCADE,
    CONSTRAINT revision_approvals_org_reviewer_fk
        FOREIGN KEY (organization_id, assigned_reviewer_id) REFERENCES users(organization_id, id),
    CONSTRAINT revision_approvals_org_submitter_fk
        FOREIGN KEY (organization_id, submitted_by) REFERENCES users(organization_id, id),
    CONSTRAINT revision_approvals_org_decider_fk
        FOREIGN KEY (organization_id, decided_by) REFERENCES users(organization_id, id),
    UNIQUE (project_id, revision_id)
);

ALTER TABLE revision_approvals
    ADD COLUMN IF NOT EXISTS assigned_reviewer_id text;

CREATE TABLE IF NOT EXISTS engineering_jobs (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    submitted_by text NOT NULL,
    project_id text,
    kind text NOT NULL,
    request_json jsonb NOT NULL,
    status text NOT NULL CHECK (status IN ('queued', 'running', 'succeeded', 'failed', 'cancelled')),
    progress integer NOT NULL DEFAULT 0 CHECK (progress BETWEEN 0 AND 100),
    result_json jsonb,
    error text,
    created_at timestamptz NOT NULL,
    started_at timestamptz,
    completed_at timestamptz,
    claimed_by text,
    lease_token text,
    CONSTRAINT engineering_jobs_org_submitter_fk
        FOREIGN KEY (organization_id, submitted_by) REFERENCES users(organization_id, id),
    CONSTRAINT engineering_jobs_org_project_fk
        FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id)
);

ALTER TABLE engineering_jobs
    ADD COLUMN IF NOT EXISTS claimed_by text;
ALTER TABLE engineering_jobs
    ADD COLUMN IF NOT EXISTS lease_token text;

CREATE TABLE IF NOT EXISTS worker_heartbeats (
    worker_id text PRIMARY KEY,
    last_seen timestamptz NOT NULL,
    status text NOT NULL CHECK (status IN ('idle', 'running')),
    job_id text,
    updated_at timestamptz NOT NULL DEFAULT now(),
    CONSTRAINT worker_heartbeats_job_fk
        FOREIGN KEY (job_id) REFERENCES engineering_jobs(id) ON DELETE SET NULL
);

CREATE INDEX IF NOT EXISTS idx_worker_heartbeats_last_seen
    ON worker_heartbeats (last_seen DESC);

CREATE TABLE IF NOT EXISTS artifact_records (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    project_id text NOT NULL,
    revision_id text NOT NULL,
    artifact_type text NOT NULL,
    filename text NOT NULL,
    content_sha256 text NOT NULL CHECK (content_sha256 ~ '^[0-9a-f]{64}$'),
    byte_size bigint NOT NULL CHECK (byte_size > 0),
    created_by text NOT NULL,
    created_at timestamptz NOT NULL,
    storage_backend text NOT NULL DEFAULT 'response-only',
    CONSTRAINT artifact_records_org_project_fk
        FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE,
    CONSTRAINT artifact_records_org_revision_fk
        FOREIGN KEY (organization_id, project_id, revision_id) REFERENCES project_revisions(organization_id, project_id, id) ON DELETE CASCADE,
    CONSTRAINT artifact_records_org_creator_fk
        FOREIGN KEY (organization_id, created_by) REFERENCES users(organization_id, id)
);

CREATE TABLE IF NOT EXISTS audit_events (
    id text PRIMARY KEY,
    organization_id text NOT NULL REFERENCES organizations(id),
    actor_user_id text,
    event_type text NOT NULL,
    target_type text NOT NULL,
    target_id text NOT NULL,
    metadata_json jsonb NOT NULL DEFAULT '{}'::jsonb,
    created_at timestamptz NOT NULL DEFAULT now(),
    CONSTRAINT audit_events_org_actor_fk
        FOREIGN KEY (organization_id, actor_user_id) REFERENCES users(organization_id, id)
);

-- Keep the audit trail and controlled artifact ledger append-only even when
-- database access is used outside the application process.
CREATE OR REPLACE FUNCTION reject_append_only_mutation()
RETURNS trigger
LANGUAGE plpgsql
AS $$
BEGIN
    RAISE EXCEPTION '% is append-only', TG_TABLE_NAME;
END;
$$;

DROP TRIGGER IF EXISTS audit_events_append_only ON audit_events;
CREATE TRIGGER audit_events_append_only
    BEFORE UPDATE OR DELETE ON audit_events
    FOR EACH ROW EXECUTE FUNCTION reject_append_only_mutation();

DROP TRIGGER IF EXISTS artifact_records_append_only ON artifact_records;
CREATE TRIGGER artifact_records_append_only
    BEFORE UPDATE OR DELETE ON artifact_records
    FOR EACH ROW EXECUTE FUNCTION reject_append_only_mutation();

-- Add the tenant-scoped keys and relationships to databases created by the
-- earlier single-column schema as well as to fresh installations.
DO $$
BEGIN
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'users_org_id_key'
    ) THEN
        ALTER TABLE users
            ADD CONSTRAINT users_org_id_key UNIQUE (organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'projects_org_id_key'
    ) THEN
        ALTER TABLE projects
            ADD CONSTRAINT projects_org_id_key UNIQUE (organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'project_revisions_org_project_id_key'
    ) THEN
        ALTER TABLE project_revisions
            ADD CONSTRAINT project_revisions_org_project_id_key
            UNIQUE (organization_id, project_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'user_sessions_org_user_fk'
    ) THEN
        ALTER TABLE user_sessions
            ADD CONSTRAINT user_sessions_org_user_fk
            FOREIGN KEY (organization_id, user_id) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'project_revisions_org_project_fk'
    ) THEN
        ALTER TABLE project_revisions
            ADD CONSTRAINT project_revisions_org_project_fk
            FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE;
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'project_memberships_org_project_fk'
    ) THEN
        ALTER TABLE project_memberships
            ADD CONSTRAINT project_memberships_org_project_fk
            FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE;
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'revision_approvals_org_project_fk'
    ) THEN
        ALTER TABLE revision_approvals
            ADD CONSTRAINT revision_approvals_org_project_fk
            FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE;
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'revision_approvals_org_revision_fk'
    ) THEN
        ALTER TABLE revision_approvals
            ADD CONSTRAINT revision_approvals_org_revision_fk
            FOREIGN KEY (organization_id, project_id, revision_id)
            REFERENCES project_revisions(organization_id, project_id, id) ON DELETE CASCADE;
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'revision_approvals_org_reviewer_fk'
    ) THEN
        ALTER TABLE revision_approvals
            ADD CONSTRAINT revision_approvals_org_reviewer_fk
            FOREIGN KEY (organization_id, assigned_reviewer_id) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'revision_approvals_org_submitter_fk'
    ) THEN
        ALTER TABLE revision_approvals
            ADD CONSTRAINT revision_approvals_org_submitter_fk
            FOREIGN KEY (organization_id, submitted_by) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'revision_approvals_org_decider_fk'
    ) THEN
        ALTER TABLE revision_approvals
            ADD CONSTRAINT revision_approvals_org_decider_fk
            FOREIGN KEY (organization_id, decided_by) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'engineering_jobs_org_submitter_fk'
    ) THEN
        ALTER TABLE engineering_jobs
            ADD CONSTRAINT engineering_jobs_org_submitter_fk
            FOREIGN KEY (organization_id, submitted_by) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'engineering_jobs_org_project_fk'
    ) THEN
        ALTER TABLE engineering_jobs
            ADD CONSTRAINT engineering_jobs_org_project_fk
            FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'artifact_records_org_project_fk'
    ) THEN
        ALTER TABLE artifact_records
            ADD CONSTRAINT artifact_records_org_project_fk
            FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id) ON DELETE CASCADE;
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'artifact_records_org_revision_fk'
    ) THEN
        ALTER TABLE artifact_records
            ADD CONSTRAINT artifact_records_org_revision_fk
            FOREIGN KEY (organization_id, project_id, revision_id)
            REFERENCES project_revisions(organization_id, project_id, id) ON DELETE CASCADE;
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'artifact_records_org_creator_fk'
    ) THEN
        ALTER TABLE artifact_records
            ADD CONSTRAINT artifact_records_org_creator_fk
            FOREIGN KEY (organization_id, created_by) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'audit_events_org_actor_fk'
    ) THEN
        ALTER TABLE audit_events
            ADD CONSTRAINT audit_events_org_actor_fk
            FOREIGN KEY (organization_id, actor_user_id) REFERENCES users(organization_id, id);
    END IF;
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'projects_active_revision_tenant_fk'
    ) THEN
        ALTER TABLE projects
            ADD CONSTRAINT projects_active_revision_tenant_fk
            FOREIGN KEY (organization_id, id, active_revision_id)
            REFERENCES project_revisions(organization_id, project_id, id)
            DEFERRABLE INITIALLY DEFERRED;
    END IF;
END
$$;

CREATE INDEX IF NOT EXISTS idx_sessions_active ON user_sessions (token_hash, expires_at) WHERE revoked_at IS NULL;
CREATE INDEX IF NOT EXISTS idx_projects_org_updated ON projects (organization_id, updated_at DESC);
CREATE INDEX IF NOT EXISTS idx_revisions_project_created ON project_revisions (project_id, created_at ASC);
CREATE INDEX IF NOT EXISTS idx_jobs_org_created ON engineering_jobs (organization_id, created_at DESC);
CREATE UNIQUE INDEX IF NOT EXISTS idx_jobs_lease_token
    ON engineering_jobs (lease_token) WHERE lease_token IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_artifacts_revision_created ON artifact_records (organization_id, project_id, revision_id, created_at ASC);
CREATE INDEX IF NOT EXISTS idx_audit_org_created ON audit_events (organization_id, created_at DESC);
CREATE INDEX IF NOT EXISTS idx_approvals_org_status ON revision_approvals (organization_id, status);
