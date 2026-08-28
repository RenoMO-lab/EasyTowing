"""Tenant, identity, approval, job, and audit primitives for the SaaS boundary.

The engineering package remains usable without this module.  The in-memory
store is deterministic and is intended for local development and tests; the
PostgreSQL schema in ``postgres_schema.sql`` is the production persistence
contract.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from enum import StrEnum
import base64
import hashlib
import hmac
import json
import math
from pathlib import Path
import secrets
from threading import Event, Lock
from typing import Any, Callable, Iterable
from uuid import uuid4


class SaaSAuthorizationError(PermissionError):
    """Raised when a principal lacks tenant or role access."""


class SaaSBootstrapError(RuntimeError):
    """Raised when deployment bootstrap has already been consumed."""


class UserRole(StrEnum):
    VIEWER = "viewer"
    DESIGNER = "designer"
    REVIEWER = "reviewer"
    ADMIN = "admin"


class ApprovalStatus(StrEnum):
    DRAFT = "draft"
    SUBMITTED = "submitted"
    APPROVED = "approved"
    REJECTED = "rejected"


class JobStatus(StrEnum):
    QUEUED = "queued"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


ROLE_PERMISSIONS: dict[UserRole, frozenset[str]] = {
    UserRole.VIEWER: frozenset({"project:read", "report:read"}),
    UserRole.DESIGNER: frozenset({"project:read", "project:write", "report:read", "job:submit", "revision:submit"}),
    UserRole.REVIEWER: frozenset({"project:read", "report:read", "revision:approve", "audit:read"}),
    UserRole.ADMIN: frozenset({
        "project:read",
        "project:write",
        "report:read",
        "job:submit",
        "revision:submit",
        "revision:approve",
        "audit:read",
        "user:manage",
    }),
}

APPROVAL_EVENT_TYPES = frozenset({
    "REVISION_SUBMITTED",
    "REVISION_APPROVED",
    "REVISION_REJECTED",
    "REVIEWER_ASSIGNED",
    "REVIEWER_UNASSIGNED",
})


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _new_id(prefix: str) -> str:
    return f"{prefix}_{uuid4().hex[:12]}"


def hash_password(password: str, *, salt: bytes | None = None) -> str:
    """Hash a password with a memory-hard KDF; never store the clear text."""

    if len(password) < 12:
        raise ValueError("Passwords must contain at least 12 characters.")
    salt = salt or secrets.token_bytes(16)
    digest = hashlib.scrypt(password.encode("utf-8"), salt=salt, n=2**14, r=8, p=1)
    return "scrypt$16384$8$1$" + base64.urlsafe_b64encode(salt + digest).decode("ascii")


def verify_password(password: str, encoded: str) -> bool:
    try:
        algorithm, n_value, r_value, p_value, encoded_payload = encoded.split("$", 4)
        if algorithm != "scrypt":
            return False
        payload = base64.urlsafe_b64decode(encoded_payload.encode("ascii"))
        salt, expected = payload[:16], payload[16:]
        actual = hashlib.scrypt(
            password.encode("utf-8"),
            salt=salt,
            n=int(n_value),
            r=int(r_value),
            p=int(p_value),
        )
        return hmac.compare_digest(actual, expected)
    except (ValueError, TypeError):
        return False


@dataclass(frozen=True, slots=True)
class Principal:
    user_id: str
    organization_id: str
    email: str
    role: UserRole
    display_name: str

    def can(self, permission: str) -> bool:
        return permission in ROLE_PERMISSIONS[self.role]


@dataclass(slots=True)
class UserAccount:
    id: str
    organization_id: str
    email: str
    display_name: str
    role: UserRole
    password_hash: str
    active: bool = True


@dataclass(slots=True)
class Session:
    id: str
    token_hash: str
    principal: Principal
    created_at: datetime
    expires_at: datetime
    revoked_at: datetime | None = None

    @property
    def active(self) -> bool:
        return self.revoked_at is None and self.expires_at > _utc_now()


@dataclass(frozen=True, slots=True)
class AuditEvent:
    id: str
    organization_id: str
    actor_user_id: str | None
    event_type: str
    target_type: str
    target_id: str
    metadata: dict[str, Any]
    created_at: datetime


@dataclass(slots=True)
class RevisionApproval:
    id: str
    organization_id: str
    project_id: str
    revision_id: str
    assigned_reviewer_id: str | None = None
    status: ApprovalStatus = ApprovalStatus.DRAFT
    submitted_by: str | None = None
    submitted_at: datetime | None = None
    decided_by: str | None = None
    decided_at: datetime | None = None
    decision_note: str = ""


@dataclass(slots=True)
class EngineeringJob:
    id: str
    organization_id: str
    submitted_by: str
    project_id: str | None
    kind: str
    request: dict[str, Any]
    status: JobStatus = JobStatus.QUEUED
    progress: int = 0
    result: dict[str, Any] | None = None
    error: str | None = None
    created_at: datetime = field(default_factory=_utc_now)
    started_at: datetime | None = None
    completed_at: datetime | None = None
    worker_id: str | None = None
    lease_id: str | None = None


@dataclass(frozen=True, slots=True)
class ArtifactRecord:
    """Immutable metadata for a file delivered from a controlled release."""

    id: str
    organization_id: str
    project_id: str
    revision_id: str
    artifact_type: str
    filename: str
    content_sha256: str
    byte_size: int
    created_by: str
    created_at: datetime
    storage_backend: str = "response-only"


class ArtifactStorageError(RuntimeError):
    """Raised when a retained artifact cannot be written or verified."""


class FileArtifactStore:
    """Durable local object store for controlled artifact bytes.

    This adapter is intentionally small and dependency-free. Production cloud
    storage can implement the same put/read/delete contract without changing
    the release route or its checksum ledger.
    """

    backend = "filesystem"

    def __init__(self, root: Path | str) -> None:
        configured_root = str(root).strip()
        if not configured_root:
            raise ValueError("An artifact storage directory is required.")
        self.root = Path(configured_root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def _path(self, artifact_id: str) -> Path:
        identifier = artifact_id.strip()
        if (
            not identifier
            or identifier in {".", ".."}
            or "/" in identifier
            or "\\" in identifier
        ):
            raise ValueError("Artifact IDs must be non-empty single path segments.")
        return self.root / f"{identifier}.blob"

    def put(
        self,
        artifact_id: str,
        content: bytes,
        *,
        expected_sha256: str | None = None,
        expected_size: int | None = None,
    ) -> Path:
        if not content:
            raise ArtifactStorageError("Artifact content cannot be empty.")
        actual_sha256 = hashlib.sha256(content).hexdigest()
        if expected_sha256 is not None and actual_sha256 != expected_sha256:
            raise ArtifactStorageError("Artifact checksum does not match the expected SHA-256.")
        if expected_size is not None and len(content) != expected_size:
            raise ArtifactStorageError("Artifact byte size does not match the expected size.")
        destination = self._path(artifact_id)
        temporary = self.root / f".{destination.name}.{secrets.token_hex(8)}.tmp"
        try:
            temporary.write_bytes(content)
            temporary.replace(destination)
        except OSError as error:
            raise ArtifactStorageError(f"Could not retain artifact {artifact_id!r}.") from error
        finally:
            temporary.unlink(missing_ok=True)
        return destination

    def read(
        self,
        artifact_id: str,
        *,
        expected_sha256: str | None = None,
        expected_size: int | None = None,
    ) -> bytes:
        path = self._path(artifact_id)
        try:
            content = path.read_bytes()
        except OSError as error:
            raise ArtifactStorageError(f"Retained artifact {artifact_id!r} is unavailable.") from error
        actual_sha256 = hashlib.sha256(content).hexdigest()
        if expected_sha256 is not None and actual_sha256 != expected_sha256:
            raise ArtifactStorageError("Retained artifact checksum verification failed.")
        if expected_size is not None and len(content) != expected_size:
            raise ArtifactStorageError("Retained artifact byte-size verification failed.")
        return content

    def delete(self, artifact_id: str) -> None:
        try:
            self._path(artifact_id).unlink(missing_ok=True)
        except OSError as error:
            raise ArtifactStorageError(f"Could not remove artifact {artifact_id!r}.") from error

    def health_check(self) -> None:
        """Verify that the configured directory can accept an atomic write."""

        probe = self.root / f".health-{secrets.token_hex(8)}.tmp"
        try:
            probe.write_bytes(b"easytowing-artifact-storage")
        except OSError as error:
            raise ArtifactStorageError("Artifact storage is not writable.") from error
        finally:
            probe.unlink(missing_ok=True)


class SaaSControlStore:
    """Thread-safe local control store with tenant and role enforcement."""

    def __init__(self, *, session_ttl_hours: int = 12) -> None:
        self._lock = Lock()
        self._session_ttl = timedelta(hours=session_ttl_hours)
        self._users_by_email: dict[tuple[str, str], UserAccount] = {}
        self._sessions_by_token: dict[str, Session] = {}
        self._project_organizations: dict[str, str] = {}
        self._approvals: dict[tuple[str, str], RevisionApproval] = {}
        self._jobs: dict[str, EngineeringJob] = {}
        self._artifacts: dict[str, ArtifactRecord] = {}
        self._audit: list[AuditEvent] = []

    def _audit_event(
        self,
        *,
        organization_id: str,
        actor_user_id: str | None,
        event_type: str,
        target_type: str,
        target_id: str,
        metadata: dict[str, Any] | None = None,
    ) -> AuditEvent:
        event = AuditEvent(
            id=_new_id("audit"),
            organization_id=organization_id,
            actor_user_id=actor_user_id,
            event_type=event_type,
            target_type=target_type,
            target_id=target_id,
            metadata=dict(metadata or {}),
            created_at=_utc_now(),
        )
        self._audit.append(event)
        return event

    def create_user(
        self,
        organization_id: str,
        email: str,
        password: str,
        *,
        role: UserRole = UserRole.DESIGNER,
        display_name: str = "",
        created_by: Principal | None = None,
    ) -> UserAccount:
        if created_by is not None:
            self.require(created_by, "user:manage")
            if created_by.organization_id != organization_id:
                raise SaaSAuthorizationError("Users can only be created in the administrator's organization.")
        normalized_email = email.strip().lower()
        if not normalized_email or "@" not in normalized_email:
            raise ValueError("A valid email address is required.")
        with self._lock:
            key = (organization_id, normalized_email)
            if key in self._users_by_email:
                raise ValueError("A user with this email already exists in the organization.")
            account = UserAccount(
                id=_new_id("usr"),
                organization_id=organization_id,
                email=normalized_email,
                display_name=display_name.strip() or normalized_email,
                role=UserRole(role),
                password_hash=hash_password(password),
            )
            self._users_by_email[key] = account
            self._audit_event(
                organization_id=organization_id,
                actor_user_id=created_by.user_id if created_by is not None else None,
                event_type="USER_CREATED",
                target_type="user",
                target_id=account.id,
                metadata={"email": normalized_email, "role": account.role.value},
            )
            return account

    def bootstrap_admin(
        self,
        organization_id: str,
        email: str,
        password: str,
        *,
        display_name: str = "",
        organization_name: str = "",
    ) -> UserAccount:
        """Create the only initial administrator for local bootstrap.

        This operation is intentionally separate from normal user creation so
        an out-of-band bootstrap token cannot be used to mint additional
        administrators after first setup.
        """

        normalized_organization = organization_id.strip()
        if not normalized_organization:
            raise ValueError("An organization ID is required.")
        normalized_email = email.strip().lower()
        if not normalized_email or "@" not in normalized_email:
            raise ValueError("A valid email address is required.")
        password_hash = hash_password(password)
        with self._lock:
            if self._users_by_email:
                raise SaaSBootstrapError("Bootstrap has already been consumed by this deployment.")
            account = UserAccount(
                id=_new_id("usr"),
                organization_id=normalized_organization,
                email=normalized_email,
                display_name=display_name.strip() or normalized_email,
                role=UserRole.ADMIN,
                password_hash=password_hash,
            )
            self._users_by_email[(normalized_organization, normalized_email)] = account
            self._audit_event(
                organization_id=normalized_organization,
                actor_user_id=None,
                event_type="BOOTSTRAP_ADMIN_CREATED",
                target_type="user",
                target_id=account.id,
                metadata={"email": normalized_email, "role": UserRole.ADMIN.value},
            )
            return account

    def login(self, organization_id: str, email: str, password: str) -> tuple[str, Principal]:
        with self._lock:
            account = self._users_by_email.get((organization_id, email.strip().lower()))
            if account is None or not account.active or not verify_password(password, account.password_hash):
                raise SaaSAuthorizationError("Invalid credentials.")
            principal = Principal(
                user_id=account.id,
                organization_id=account.organization_id,
                email=account.email,
                role=account.role,
                display_name=account.display_name,
            )
            raw_token = secrets.token_urlsafe(32)
            token_hash = hashlib.sha256(raw_token.encode("ascii")).hexdigest()
            self._sessions_by_token[token_hash] = Session(
                id=_new_id("ses"),
                token_hash=token_hash,
                principal=principal,
                created_at=_utc_now(),
                expires_at=_utc_now() + self._session_ttl,
            )
            self._audit_event(
                organization_id=organization_id,
                actor_user_id=account.id,
                event_type="SESSION_CREATED",
                target_type="session",
                target_id=self._sessions_by_token[token_hash].id,
            )
            return raw_token, principal

    def authenticate(self, raw_token: str) -> Principal:
        token_hash = hashlib.sha256(raw_token.encode("ascii")).hexdigest()
        with self._lock:
            session = self._sessions_by_token.get(token_hash)
            if session is None or not session.active:
                raise SaaSAuthorizationError("Authentication required.")
            return session.principal

    def logout(self, raw_token: str) -> None:
        token_hash = hashlib.sha256(raw_token.encode("ascii")).hexdigest()
        with self._lock:
            session = self._sessions_by_token.get(token_hash)
            if session is not None and session.revoked_at is None:
                session.revoked_at = _utc_now()
                self._audit_event(
                    organization_id=session.principal.organization_id,
                    actor_user_id=session.principal.user_id,
                    event_type="SESSION_REVOKED",
                    target_type="session",
                    target_id=session.id,
                )

    def require(self, principal: Principal, permission: str) -> None:
        if not principal.can(permission):
            raise SaaSAuthorizationError(f"Role {principal.role.value!r} cannot perform {permission!r}.")

    def list_users(self, principal: Principal) -> list[UserAccount]:
        """List active tenant users for administrative reviewer assignment."""

        self.require(principal, "user:manage")
        with self._lock:
            return [
                account
                for account in self._users_by_email.values()
                if account.organization_id == principal.organization_id and account.active
            ]

    def bind_project(self, principal: Principal, project_id: str) -> None:
        self.require(principal, "project:write")
        with self._lock:
            existing = self._project_organizations.get(project_id)
            if existing is not None and existing != principal.organization_id:
                raise SaaSAuthorizationError("Project belongs to another organization.")
            self._project_organizations[project_id] = principal.organization_id
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type="PROJECT_BOUND",
                target_type="project",
                target_id=project_id,
            )

    def _check_project(self, principal: Principal, project_id: str, permission: str) -> None:
        self.require(principal, permission)
        organization_id = self._project_organizations.get(project_id)
        if organization_id is not None and organization_id != principal.organization_id:
            raise SaaSAuthorizationError("Project belongs to another organization.")

    def submit_revision(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
        *,
        note: str = "",
    ) -> RevisionApproval:
        self._check_project(principal, project_id, "revision:submit")
        with self._lock:
            key = (project_id, revision_id)
            approval = self._approvals.get(key)
            if approval is None:
                approval = RevisionApproval(
                    id=_new_id("approval"),
                    organization_id=principal.organization_id,
                    project_id=project_id,
                    revision_id=revision_id,
                )
                self._approvals[key] = approval
            if approval.status not in {ApprovalStatus.DRAFT, ApprovalStatus.REJECTED}:
                raise ValueError(f"Revision is already {approval.status.value}.")
            approval.status = ApprovalStatus.SUBMITTED
            approval.submitted_by = principal.user_id
            approval.submitted_at = _utc_now()
            approval.decision_note = note.strip()
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type="REVISION_SUBMITTED",
                target_type="revision",
                target_id=revision_id,
                metadata={"project_id": project_id, "note": approval.decision_note},
            )
            return approval

    def assign_reviewer(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
        reviewer_user_id: str | None,
    ) -> RevisionApproval:
        self._check_project(principal, project_id, "user:manage")
        normalized_reviewer_id = reviewer_user_id.strip() if reviewer_user_id else None
        with self._lock:
            account = None
            if normalized_reviewer_id:
                account = next(
                    (
                        candidate
                        for candidate in self._users_by_email.values()
                        if candidate.id == normalized_reviewer_id
                        and candidate.organization_id == principal.organization_id
                        and candidate.active
                    ),
                    None,
                )
                if account is None or account.role not in {UserRole.REVIEWER, UserRole.ADMIN}:
                    raise ValueError("Assigned reviewer must be an active reviewer or administrator in this organization.")
            key = (project_id, revision_id)
            approval = self._approvals.get(key)
            if approval is None:
                approval = RevisionApproval(
                    id=_new_id("approval"),
                    organization_id=principal.organization_id,
                    project_id=project_id,
                    revision_id=revision_id,
                )
                self._approvals[key] = approval
            if approval.status == ApprovalStatus.APPROVED:
                raise ValueError("Approved revisions cannot be reassigned.")
            approval.assigned_reviewer_id = normalized_reviewer_id
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type="REVIEWER_ASSIGNED" if account else "REVIEWER_UNASSIGNED",
                target_type="revision",
                target_id=revision_id,
                metadata={
                    "project_id": project_id,
                    "reviewer_user_id": normalized_reviewer_id,
                },
            )
            return approval

    def decide_revision(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
        *,
        approved: bool,
        note: str = "",
    ) -> RevisionApproval:
        self._check_project(principal, project_id, "revision:approve")
        with self._lock:
            approval = self._approvals.get((project_id, revision_id))
            if approval is None or approval.status != ApprovalStatus.SUBMITTED:
                raise ValueError("Only submitted revisions can be approved or rejected.")
            if approval.submitted_by == principal.user_id:
                raise SaaSAuthorizationError("The submitting designer cannot approve their own revision.")
            if (
                approval.assigned_reviewer_id
                and approval.assigned_reviewer_id != principal.user_id
            ):
                raise SaaSAuthorizationError("This revision is assigned to another reviewer.")
            approval.status = ApprovalStatus.APPROVED if approved else ApprovalStatus.REJECTED
            approval.decided_by = principal.user_id
            approval.decided_at = _utc_now()
            approval.decision_note = note.strip()
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type="REVISION_APPROVED" if approved else "REVISION_REJECTED",
                target_type="revision",
                target_id=revision_id,
                metadata={"project_id": project_id, "note": approval.decision_note},
            )
            return approval

    def get_approval(self, principal: Principal, project_id: str, revision_id: str) -> RevisionApproval | None:
        self._check_project(principal, project_id, "project:read")
        with self._lock:
            approval = self._approvals.get((project_id, revision_id))
            if approval is None or approval.organization_id != principal.organization_id:
                return None
            return approval

    def approval_history(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
    ) -> list[AuditEvent]:
        self._check_project(principal, project_id, "project:read")
        with self._lock:
            return [
                event
                for event in self._audit
                if event.organization_id == principal.organization_id
                and event.target_type == "revision"
                and event.target_id == revision_id
                and event.event_type in APPROVAL_EVENT_TYPES
                and event.metadata.get("project_id") == project_id
            ]

    def audit_events(self, principal: Principal, *, target_id: str | None = None) -> list[AuditEvent]:
        self.require(principal, "audit:read")
        with self._lock:
            return [
                event
                for event in self._audit
                if event.organization_id == principal.organization_id
                and (
                    target_id is None
                    or event.target_id == target_id
                    or event.metadata.get("project_id") == target_id
                )
            ]

    def record_event(
        self,
        principal: Principal,
        *,
        project_id: str,
        event_type: str,
        target_type: str,
        target_id: str,
        metadata: dict[str, Any] | None = None,
    ) -> AuditEvent:
        self._check_project(principal, project_id, "project:write")
        with self._lock:
            return self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type=event_type,
                target_type=target_type,
                target_id=target_id,
                metadata={"project_id": project_id, **dict(metadata or {})},
            )

    def record_artifact(
        self,
        principal: Principal,
        *,
        project_id: str,
        revision_id: str,
        artifact_type: str,
        filename: str,
        content: bytes,
        artifact_id: str,
        created_at: datetime,
        storage_backend: str = "response-only",
    ) -> ArtifactRecord:
        """Record the exact bytes delivered without claiming blob retention."""

        self._check_project(principal, project_id, "report:read")
        if not artifact_id.strip() or not artifact_type.strip() or not filename.strip():
            raise ValueError("Artifact id, type, and filename are required.")
        if not content:
            raise ValueError("Artifact content cannot be empty.")
        artifact = ArtifactRecord(
            id=artifact_id.strip(),
            organization_id=principal.organization_id,
            project_id=project_id,
            revision_id=revision_id,
            artifact_type=artifact_type.strip(),
            filename=filename.strip(),
            content_sha256=hashlib.sha256(content).hexdigest(),
            byte_size=len(content),
            created_by=principal.user_id,
            created_at=created_at,
            storage_backend=storage_backend.strip() or "response-only",
        )
        with self._lock:
            if artifact.id in self._artifacts:
                raise ValueError(f"Artifact {artifact.id!r} already exists.")
            self._artifacts[artifact.id] = artifact
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type="ARTIFACT_RECORDED",
                target_type="artifact",
                target_id=artifact.id,
                metadata={
                    "project_id": project_id,
                    "revision_id": revision_id,
                    "artifact_type": artifact.artifact_type,
                    "filename": artifact.filename,
                    "content_sha256": artifact.content_sha256,
                    "byte_size": artifact.byte_size,
                    "storage_backend": artifact.storage_backend,
                },
            )
            return artifact

    def list_artifacts(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
    ) -> list[ArtifactRecord]:
        self._check_project(principal, project_id, "report:read")
        with self._lock:
            return [
                artifact
                for artifact in self._artifacts.values()
                if artifact.organization_id == principal.organization_id
                and artifact.project_id == project_id
                and artifact.revision_id == revision_id
            ]

    def create_job(
        self,
        principal: Principal,
        *,
        kind: str,
        request: dict[str, Any],
        project_id: str | None = None,
    ) -> EngineeringJob:
        self.require(principal, "job:submit")
        if project_id is not None:
            self._check_project(principal, project_id, "project:read")
        with self._lock:
            job = EngineeringJob(
                id=_new_id("job"),
                organization_id=principal.organization_id,
                submitted_by=principal.user_id,
                project_id=project_id,
                kind=kind,
                request=dict(request),
            )
            self._jobs[job.id] = job
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type="JOB_QUEUED",
                target_type="job",
                target_id=job.id,
                metadata={"kind": kind, "project_id": project_id},
            )
            return job

    def get_job(self, principal: Principal, job_id: str) -> EngineeringJob:
        self.require(principal, "project:read")
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None or job.organization_id != principal.organization_id:
                raise KeyError(job_id)
            return job

    def update_job(
        self,
        principal: Principal,
        job_id: str,
        *,
        status: JobStatus,
        progress: int | None = None,
        result: dict[str, Any] | None = None,
        error: str | None = None,
    ) -> EngineeringJob:
        with self._lock:
            self.require(principal, "project:read")
            job = self._jobs.get(job_id)
            if job is None or job.organization_id != principal.organization_id:
                raise KeyError(job_id)
            if status == JobStatus.RUNNING and job.status != JobStatus.QUEUED:
                raise ValueError("Only queued jobs can start.")
            if status in {JobStatus.SUCCEEDED, JobStatus.FAILED, JobStatus.CANCELLED} and job.status not in {JobStatus.RUNNING, JobStatus.QUEUED}:
                raise ValueError("Only active jobs can finish.")
            job.status = status
            if progress is not None:
                job.progress = max(0, min(100, int(progress)))
            if status == JobStatus.RUNNING:
                job.started_at = _utc_now()
            if status in {JobStatus.SUCCEEDED, JobStatus.FAILED, JobStatus.CANCELLED}:
                job.completed_at = _utc_now()
            job.result = result
            job.error = error
            self._audit_event(
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type=f"JOB_{status.value.upper()}",
                target_type="job",
                target_id=job.id,
                metadata={"progress": job.progress},
            )
            return job


class EngineeringJobRunner:
    """Small background runner; production deployments can replace its executor."""

    def __init__(self, store: SaaSControlStore, *, max_workers: int = 2) -> None:
        self._store = store
        self._executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="easytowing-job")
        self._futures: dict[str, Future[Any]] = {}

    def submit(
        self,
        principal: Principal,
        *,
        kind: str,
        request: dict[str, Any],
        operation: Callable[[dict[str, Any]], dict[str, Any]],
        project_id: str | None = None,
    ) -> EngineeringJob:
        job = self._store.create_job(
            principal,
            kind=kind,
            request=request,
            project_id=project_id,
        )

        def run() -> dict[str, Any]:
            self._store.update_job(principal, job.id, status=JobStatus.RUNNING, progress=5)
            try:
                result = operation(request)
            except Exception as error:  # noqa: BLE001 - persist job failure for polling clients
                self._store.update_job(
                    principal,
                    job.id,
                    status=JobStatus.FAILED,
                    progress=100,
                    error=str(error),
                )
                raise
            self._store.update_job(
                principal,
                job.id,
                status=JobStatus.SUCCEEDED,
                progress=100,
                result=result,
            )
            return result

        self._futures[job.id] = self._executor.submit(run)
        return job

    def shutdown(self, *, wait: bool = True) -> None:
        self._executor.shutdown(wait=wait)


def principal_payload(principal: Principal) -> dict[str, str]:
    return {
        "user_id": principal.user_id,
        "organization_id": principal.organization_id,
        "email": principal.email,
        "display_name": principal.display_name,
        "role": principal.role.value,
    }


def serialize_user(account: UserAccount) -> dict[str, str]:
    return {
        "user_id": account.id,
        "organization_id": account.organization_id,
        "email": account.email,
        "display_name": account.display_name,
        "role": account.role.value,
    }


def serialize_approval(approval: RevisionApproval) -> dict[str, Any]:
    return {
        "id": approval.id,
        "organization_id": approval.organization_id,
        "project_id": approval.project_id,
        "revision_id": approval.revision_id,
        "assigned_reviewer_id": approval.assigned_reviewer_id,
        "status": approval.status.value,
        "submitted_by": approval.submitted_by,
        "submitted_at": None if approval.submitted_at is None else approval.submitted_at.isoformat(),
        "decided_by": approval.decided_by,
        "decided_at": None if approval.decided_at is None else approval.decided_at.isoformat(),
        "decision_note": approval.decision_note,
    }


def serialize_job(job: EngineeringJob) -> dict[str, Any]:
    return {
        "id": job.id,
        "organization_id": job.organization_id,
        "submitted_by": job.submitted_by,
        "project_id": job.project_id,
        "kind": job.kind,
        "status": job.status.value,
        "progress": job.progress,
        "request": job.request,
        "result": job.result,
        "error": job.error,
        "created_at": job.created_at.isoformat(),
        "started_at": None if job.started_at is None else job.started_at.isoformat(),
        "completed_at": None if job.completed_at is None else job.completed_at.isoformat(),
    }


def serialize_artifact(artifact: ArtifactRecord) -> dict[str, Any]:
    return {
        "id": artifact.id,
        "organization_id": artifact.organization_id,
        "project_id": artifact.project_id,
        "revision_id": artifact.revision_id,
        "artifact_type": artifact.artifact_type,
        "filename": artifact.filename,
        "content_sha256": artifact.content_sha256,
        "byte_size": artifact.byte_size,
        "created_by": artifact.created_by,
        "created_at": artifact.created_at.isoformat(),
        "storage_backend": artifact.storage_backend,
    }


def serialize_audit_event(event: AuditEvent) -> dict[str, Any]:
    return {
        "id": event.id,
        "organization_id": event.organization_id,
        "actor_user_id": event.actor_user_id,
        "event_type": event.event_type,
        "target_type": event.target_type,
        "target_id": event.target_id,
        "metadata": event.metadata,
        "created_at": event.created_at.isoformat(),
    }


class PostgreSQLSaaSStore:
    """PostgreSQL control-plane adapter.

    The connector is injectable for application startup and integration tests;
    the default connector is ``psycopg.connect`` and is imported lazily so the
    calculation-only package does not require a database driver.
    """

    def __init__(self, dsn: str, *, connect: Callable[[str], Any] | None = None) -> None:
        if not dsn.strip():
            raise ValueError("A PostgreSQL DSN is required.")
        if connect is None:
            try:
                import psycopg  # type: ignore
            except ImportError as error:  # pragma: no cover - depends on deployment extras
                raise RuntimeError("Install psycopg[binary] to use PostgreSQLSaaSStore.") from error
            connect = psycopg.connect
        self._dsn = dsn
        self._connect = connect

    def _transaction(self, operation: Callable[[Any], Any]) -> Any:
        connection = self._connect(self._dsn)
        try:
            result = operation(connection)
            connection.commit()
            return result
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()

    def migrate(self, schema_path: Path | None = None) -> None:
        path = schema_path or Path(__file__).with_name("postgres_schema.sql")
        schema = path.read_text(encoding="utf-8")

        def apply_schema(connection: Any) -> None:
            # API and worker processes can start together; serialize DDL so
            # concurrent CREATE TABLE/TYPE operations cannot race.
            cursor = connection.cursor()
            cursor.execute("SELECT pg_advisory_xact_lock(hashtext('easytowing.schema.v1'))")
            cursor.execute(schema)

        self._transaction(apply_schema)

    def create_organization(self, organization_id: str, name: str) -> None:
        self._transaction(
            lambda connection: connection.cursor().execute(
                "INSERT INTO organizations (id, name) VALUES (%s, %s) ON CONFLICT (id) DO NOTHING",
                (organization_id, name.strip() or organization_id),
            )
        )

    def bootstrap_admin(
        self,
        organization_id: str,
        email: str,
        password: str,
        *,
        display_name: str = "",
        organization_name: str = "",
    ) -> UserAccount:
        """Atomically create an organization and its first administrator."""

        normalized_organization = organization_id.strip()
        if not normalized_organization:
            raise ValueError("An organization ID is required.")
        normalized_email = email.strip().lower()
        if not normalized_email or "@" not in normalized_email:
            raise ValueError("A valid email address is required.")
        password_hash = hash_password(password)
        account = UserAccount(
            id=_new_id("usr"),
            organization_id=normalized_organization,
            email=normalized_email,
            display_name=display_name.strip() or normalized_email,
            role=UserRole.ADMIN,
            password_hash=password_hash,
        )

        def bootstrap(connection: Any) -> UserAccount:
            cursor = connection.cursor()
            # Serialize bootstrap attempts across the deployment before
            # checking users, otherwise two first-admin requests can race.
            cursor.execute("SELECT pg_advisory_xact_lock(hashtext('easytowing.bootstrap.v1'))")
            cursor.execute(
                "SELECT 1 FROM users LIMIT 1",
            )
            if cursor.fetchone() is None:
                cursor.execute(
                    "INSERT INTO organizations (id, name) VALUES (%s, %s) ON CONFLICT (id) DO NOTHING",
                    (normalized_organization, organization_name.strip() or normalized_organization),
                )
            else:
                raise SaaSBootstrapError("Bootstrap has already been consumed by this deployment.")
            cursor.execute(
                """
                INSERT INTO users (id, organization_id, email, display_name, role, password_hash)
                VALUES (%s, %s, %s, %s, %s, %s)
                """,
                (
                    account.id,
                    account.organization_id,
                    account.email,
                    account.display_name,
                    account.role.value,
                    account.password_hash,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                normalized_organization,
                None,
                "BOOTSTRAP_ADMIN_CREATED",
                "user",
                account.id,
                {"email": account.email, "role": account.role.value},
            )
            return account

        return self._transaction(bootstrap)

    def create_user(
        self,
        organization_id: str,
        email: str,
        password: str,
        *,
        role: UserRole = UserRole.DESIGNER,
        display_name: str = "",
        created_by: Principal | None = None,
    ) -> UserAccount:
        if created_by is not None:
            self.require(created_by, "user:manage")
            if created_by.organization_id != organization_id:
                raise SaaSAuthorizationError("Users can only be created in the administrator's organization.")
        normalized_email = email.strip().lower()
        if not normalized_email or "@" not in normalized_email:
            raise ValueError("A valid email address is required.")
        account = UserAccount(
            id=_new_id("usr"),
            organization_id=organization_id,
            email=normalized_email,
            display_name=display_name.strip() or normalized_email,
            role=UserRole(role),
            password_hash=hash_password(password),
        )

        def insert(connection: Any) -> None:
            cursor = connection.cursor()
            cursor.execute(
                """
                INSERT INTO users (id, organization_id, email, display_name, role, password_hash)
                VALUES (%s, %s, %s, %s, %s, %s)
                """,
                (
                    account.id,
                    account.organization_id,
                    account.email,
                    account.display_name,
                    account.role.value,
                    account.password_hash,
                ),
            )
            if created_by is not None:
                self._insert_audit_cursor(
                    cursor,
                    organization_id,
                    created_by.user_id,
                    "USER_CREATED",
                    "user",
                    account.id,
                    {"email": account.email, "role": account.role.value},
                )

        self._transaction(insert)
        return account

    def login(self, organization_id: str, email: str, password: str) -> tuple[str, Principal]:
        def authenticate(connection: Any) -> tuple[str, Principal]:
            cursor = connection.cursor()
            cursor.execute(
                "SELECT id, email, display_name, role, password_hash FROM users WHERE organization_id = %s AND email = %s AND active",
                (organization_id, email.strip().lower()),
            )
            row = cursor.fetchone()
            if row is None or not verify_password(password, row[4]):
                raise SaaSAuthorizationError("Invalid credentials.")
            principal = Principal(
                user_id=row[0],
                organization_id=organization_id,
                email=row[1],
                display_name=row[2],
                role=UserRole(row[3]),
            )
            raw_token = secrets.token_urlsafe(32)
            token_hash = hashlib.sha256(raw_token.encode("ascii")).hexdigest()
            session_id = _new_id("ses")
            cursor.execute(
                "INSERT INTO user_sessions (id, organization_id, user_id, token_hash, created_at, expires_at) VALUES (%s, %s, %s, %s, %s, %s)",
                (session_id, organization_id, principal.user_id, token_hash, _utc_now(), _utc_now() + timedelta(hours=12)),
            )
            self._insert_audit_cursor(
                cursor,
                organization_id,
                principal.user_id,
                "SESSION_CREATED",
                "session",
                session_id,
                {},
            )
            return raw_token, principal

        return self._transaction(authenticate)

    @staticmethod
    def _insert_audit_cursor(
        cursor: Any,
        organization_id: str,
        actor_user_id: str | None,
        event_type: str,
        target_type: str,
        target_id: str,
        metadata: dict[str, Any],
        *,
        event_id: str | None = None,
        created_at: datetime | None = None,
    ) -> None:
        if created_at is None:
            cursor.execute(
                "INSERT INTO audit_events (id, organization_id, actor_user_id, event_type, target_type, target_id, metadata_json) VALUES (%s, %s, %s, %s, %s, %s, %s::jsonb)",
                (_new_id("audit"), organization_id, actor_user_id, event_type, target_type, target_id, json.dumps(metadata)),
            )
            return
        cursor.execute(
            "INSERT INTO audit_events (id, organization_id, actor_user_id, event_type, target_type, target_id, metadata_json, created_at) VALUES (%s, %s, %s, %s, %s, %s, %s::jsonb, %s)",
            (event_id or _new_id("audit"), organization_id, actor_user_id, event_type, target_type, target_id, json.dumps(metadata), created_at),
        )

    @staticmethod
    def _datetime(value: Any) -> datetime | None:
        if value is None:
            return None
        if isinstance(value, datetime):
            return value if value.tzinfo is not None else value.replace(tzinfo=timezone.utc)
        return datetime.fromisoformat(str(value).replace("Z", "+00:00"))

    @staticmethod
    def _json_value(value: Any) -> dict[str, Any]:
        if isinstance(value, str):
            value = json.loads(value)
        return dict(value) if isinstance(value, dict) else {}

    @staticmethod
    def _principal_from_row(row: tuple[Any, ...]) -> Principal:
        return Principal(
            user_id=str(row[0]),
            organization_id=str(row[1]),
            email=str(row[2]),
            display_name=str(row[3]),
            role=UserRole(row[4]),
        )

    @staticmethod
    def _approval_from_row(row: tuple[Any, ...]) -> RevisionApproval:
        return RevisionApproval(
            id=str(row[0]),
            organization_id=str(row[1]),
            project_id=str(row[2]),
            revision_id=str(row[3]),
            assigned_reviewer_id=None if row[4] is None else str(row[4]),
            status=ApprovalStatus(row[5]),
            submitted_by=None if row[6] is None else str(row[6]),
            submitted_at=PostgreSQLSaaSStore._datetime(row[7]),
            decided_by=None if row[8] is None else str(row[8]),
            decided_at=PostgreSQLSaaSStore._datetime(row[9]),
            decision_note=str(row[10] or ""),
        )

    @staticmethod
    def _job_from_row(row: tuple[Any, ...]) -> EngineeringJob:
        return EngineeringJob(
            id=str(row[0]),
            organization_id=str(row[1]),
            submitted_by=str(row[2]),
            project_id=None if row[3] is None else str(row[3]),
            kind=str(row[4]),
            request=PostgreSQLSaaSStore._json_value(row[5]),
            status=JobStatus(row[6]),
            progress=int(row[7]),
            result=None if row[8] is None else PostgreSQLSaaSStore._json_value(row[8]),
            error=None if row[9] is None else str(row[9]),
            created_at=PostgreSQLSaaSStore._datetime(row[10]) or _utc_now(),
            started_at=PostgreSQLSaaSStore._datetime(row[11]),
            completed_at=PostgreSQLSaaSStore._datetime(row[12]),
            worker_id=None if row[13] is None else str(row[13]),
            lease_id=None if row[14] is None else str(row[14]),
        )

    def authenticate(self, raw_token: str) -> Principal:
        token_hash = hashlib.sha256(raw_token.encode("ascii")).hexdigest()

        def read(connection: Any) -> Principal:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT u.id, u.organization_id, u.email, u.display_name, u.role
                FROM user_sessions AS s
                JOIN users AS u ON u.id = s.user_id
                WHERE s.token_hash = %s
                  AND s.revoked_at IS NULL
                  AND s.expires_at > now()
                  AND u.active
                """,
                (token_hash,),
            )
            row = cursor.fetchone()
            if row is None:
                raise SaaSAuthorizationError("Authentication required.")
            return self._principal_from_row(row)

        return self._transaction(read)

    def logout(self, raw_token: str) -> None:
        token_hash = hashlib.sha256(raw_token.encode("ascii")).hexdigest()

        def revoke(connection: Any) -> None:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, user_id
                FROM user_sessions
                WHERE token_hash = %s AND revoked_at IS NULL
                """,
                (token_hash,),
            )
            row = cursor.fetchone()
            if row is None:
                return
            cursor.execute(
                "UPDATE user_sessions SET revoked_at = %s WHERE id = %s",
                (_utc_now(), row[0]),
            )
            self._insert_audit_cursor(
                cursor,
                str(row[1]),
                str(row[2]),
                "SESSION_REVOKED",
                "session",
                str(row[0]),
                {},
            )

        self._transaction(revoke)

    def require(self, principal: Principal, permission: str) -> None:
        if not principal.can(permission):
            raise SaaSAuthorizationError(f"Role {principal.role.value!r} cannot perform {permission!r}.")

    def list_users(self, principal: Principal) -> list[UserAccount]:
        """List active tenant users for administrative reviewer assignment."""

        self.require(principal, "user:manage")

        def read(connection: Any) -> list[UserAccount]:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, email, display_name, role, password_hash, active
                FROM users
                WHERE organization_id = %s AND active
                ORDER BY display_name ASC, email ASC
                """,
                (principal.organization_id,),
            )
            return [
                UserAccount(
                    id=str(row[0]),
                    organization_id=str(row[1]),
                    email=str(row[2]),
                    display_name=str(row[3]),
                    role=UserRole(row[4]),
                    password_hash=str(row[5]),
                    active=bool(row[6]),
                )
                for row in cursor.fetchall()
            ]

        return self._transaction(read)

    def bind_project(self, principal: Principal, project_id: str) -> None:
        self.require(principal, "project:write")

        def bind(connection: Any) -> None:
            cursor = connection.cursor()
            cursor.execute("SELECT organization_id FROM projects WHERE id = %s", (project_id,))
            row = cursor.fetchone()
            if row is None:
                raise SaaSAuthorizationError("Project does not exist.")
            if str(row[0]) != principal.organization_id:
                raise SaaSAuthorizationError("Project belongs to another organization.")
            cursor.execute(
                """
                INSERT INTO project_memberships (project_id, organization_id)
                VALUES (%s, %s)
                ON CONFLICT (project_id, organization_id) DO NOTHING
                """,
                (project_id, principal.organization_id),
            )
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                "PROJECT_BOUND",
                "project",
                project_id,
                {},
            )

        self._transaction(bind)

    def _check_project_cursor(self, cursor: Any, principal: Principal, project_id: str, permission: str) -> None:
        self.require(principal, permission)
        cursor.execute("SELECT organization_id FROM projects WHERE id = %s", (project_id,))
        row = cursor.fetchone()
        if row is None or str(row[0]) != principal.organization_id:
            raise SaaSAuthorizationError("Project belongs to another organization or does not exist.")

    def submit_revision(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
        *,
        note: str = "",
    ) -> RevisionApproval:
        def submit(connection: Any) -> RevisionApproval:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "revision:submit")
            cursor.execute(
                "SELECT 1 FROM project_revisions WHERE project_id = %s AND id = %s",
                (project_id, revision_id),
            )
            if cursor.fetchone() is None:
                raise ValueError("Revision does not exist.")
            cursor.execute(
                """
                SELECT id, organization_id, project_id, revision_id, status,
                       assigned_reviewer_id, submitted_by, submitted_at,
                       decided_by, decided_at, decision_note
                FROM revision_approvals
                WHERE project_id = %s AND revision_id = %s
                FOR UPDATE
                """,
                (project_id, revision_id),
            )
            row = cursor.fetchone()
            note_value = note.strip()
            if row is None:
                approval = RevisionApproval(
                    id=_new_id("approval"),
                    organization_id=principal.organization_id,
                    project_id=project_id,
                    revision_id=revision_id,
                    status=ApprovalStatus.SUBMITTED,
                    submitted_by=principal.user_id,
                    submitted_at=_utc_now(),
                    decision_note=note_value,
                )
                cursor.execute(
                    """
                    INSERT INTO revision_approvals
                        (id, organization_id, project_id, revision_id, status,
                         submitted_by, submitted_at, decision_note)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                    """,
                    (
                        approval.id,
                        approval.organization_id,
                        approval.project_id,
                        approval.revision_id,
                        approval.status.value,
                        approval.submitted_by,
                        approval.submitted_at,
                        approval.decision_note,
                    ),
                )
            else:
                approval = self._approval_from_row(row)
                if approval.status not in {ApprovalStatus.DRAFT, ApprovalStatus.REJECTED}:
                    raise ValueError(f"Revision is already {approval.status.value}.")
                approval.status = ApprovalStatus.SUBMITTED
                approval.submitted_by = principal.user_id
                approval.submitted_at = _utc_now()
                approval.decided_by = None
                approval.decided_at = None
                approval.decision_note = note_value
                cursor.execute(
                    """
                    UPDATE revision_approvals
                    SET status = %s, submitted_by = %s, submitted_at = %s,
                        decided_by = NULL, decided_at = NULL, decision_note = %s
                    WHERE project_id = %s AND revision_id = %s
                    """,
                    (
                        approval.status.value,
                        approval.submitted_by,
                        approval.submitted_at,
                        approval.decision_note,
                        project_id,
                        revision_id,
                    ),
                )
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                "REVISION_SUBMITTED",
                "revision",
                revision_id,
                {"project_id": project_id, "note": note_value},
            )
            return approval

        return self._transaction(submit)

    def assign_reviewer(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
        reviewer_user_id: str | None,
    ) -> RevisionApproval:
        self.require(principal, "user:manage")
        normalized_reviewer_id = reviewer_user_id.strip() if reviewer_user_id else None

        def assign(connection: Any) -> RevisionApproval:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "user:manage")
            cursor.execute(
                "SELECT 1 FROM project_revisions WHERE project_id = %s AND id = %s",
                (project_id, revision_id),
            )
            if cursor.fetchone() is None:
                raise ValueError("Revision does not exist.")
            if normalized_reviewer_id:
                cursor.execute(
                    """
                    SELECT role, active
                    FROM users
                    WHERE id = %s AND organization_id = %s
                    """,
                    (normalized_reviewer_id, principal.organization_id),
                )
                reviewer_row = cursor.fetchone()
                if (
                    reviewer_row is None
                    or not bool(reviewer_row[1])
                    or UserRole(reviewer_row[0]) not in {UserRole.REVIEWER, UserRole.ADMIN}
                ):
                    raise ValueError("Assigned reviewer must be an active reviewer or administrator in this organization.")
            cursor.execute(
                """
                SELECT id, organization_id, project_id, revision_id, status,
                       assigned_reviewer_id, submitted_by, submitted_at,
                       decided_by, decided_at, decision_note
                FROM revision_approvals
                WHERE project_id = %s AND revision_id = %s
                FOR UPDATE
                """,
                (project_id, revision_id),
            )
            row = cursor.fetchone()
            if row is None:
                approval = RevisionApproval(
                    id=_new_id("approval"),
                    organization_id=principal.organization_id,
                    project_id=project_id,
                    revision_id=revision_id,
                    assigned_reviewer_id=normalized_reviewer_id,
                )
                cursor.execute(
                    """
                    INSERT INTO revision_approvals
                        (id, organization_id, project_id, revision_id,
                         assigned_reviewer_id, status)
                    VALUES (%s, %s, %s, %s, %s, %s)
                    """,
                    (
                        approval.id,
                        approval.organization_id,
                        approval.project_id,
                        approval.revision_id,
                        approval.assigned_reviewer_id,
                        approval.status.value,
                    ),
                )
            else:
                approval = self._approval_from_row(row)
                if approval.status == ApprovalStatus.APPROVED:
                    raise ValueError("Approved revisions cannot be reassigned.")
                approval.assigned_reviewer_id = normalized_reviewer_id
                cursor.execute(
                    """
                    UPDATE revision_approvals
                    SET assigned_reviewer_id = %s
                    WHERE project_id = %s AND revision_id = %s
                    """,
                    (approval.assigned_reviewer_id, project_id, revision_id),
                )
            event_type = "REVIEWER_ASSIGNED" if normalized_reviewer_id else "REVIEWER_UNASSIGNED"
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                event_type,
                "revision",
                revision_id,
                {"project_id": project_id, "reviewer_user_id": normalized_reviewer_id},
            )
            return approval

        return self._transaction(assign)

    def decide_revision(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
        *,
        approved: bool,
        note: str = "",
    ) -> RevisionApproval:
        def decide(connection: Any) -> RevisionApproval:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "revision:approve")
            cursor.execute(
                """
                SELECT id, organization_id, project_id, revision_id, status,
                       assigned_reviewer_id, submitted_by, submitted_at,
                       decided_by, decided_at, decision_note
                FROM revision_approvals
                WHERE project_id = %s AND revision_id = %s
                FOR UPDATE
                """,
                (project_id, revision_id),
            )
            row = cursor.fetchone()
            if row is None:
                raise ValueError("Only submitted revisions can be approved or rejected.")
            approval = self._approval_from_row(row)
            if approval.status != ApprovalStatus.SUBMITTED:
                raise ValueError("Only submitted revisions can be approved or rejected.")
            if approval.submitted_by == principal.user_id:
                raise SaaSAuthorizationError("The submitting designer cannot approve their own revision.")
            if (
                approval.assigned_reviewer_id
                and approval.assigned_reviewer_id != principal.user_id
            ):
                raise SaaSAuthorizationError("This revision is assigned to another reviewer.")
            approval.status = ApprovalStatus.APPROVED if approved else ApprovalStatus.REJECTED
            approval.decided_by = principal.user_id
            approval.decided_at = _utc_now()
            approval.decision_note = note.strip()
            cursor.execute(
                """
                UPDATE revision_approvals
                SET status = %s, decided_by = %s, decided_at = %s, decision_note = %s
                WHERE project_id = %s AND revision_id = %s
                """,
                (
                    approval.status.value,
                    approval.decided_by,
                    approval.decided_at,
                    approval.decision_note,
                    project_id,
                    revision_id,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                "REVISION_APPROVED" if approved else "REVISION_REJECTED",
                "revision",
                revision_id,
                {"project_id": project_id, "note": approval.decision_note},
            )
            return approval

        return self._transaction(decide)

    def get_approval(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
    ) -> RevisionApproval | None:
        def read(connection: Any) -> RevisionApproval | None:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "project:read")
            cursor.execute(
                """
                SELECT id, organization_id, project_id, revision_id, status,
                       assigned_reviewer_id, submitted_by, submitted_at,
                       decided_by, decided_at, decision_note
                FROM revision_approvals
                WHERE project_id = %s AND revision_id = %s
                """,
                (project_id, revision_id),
            )
            row = cursor.fetchone()
            return None if row is None else self._approval_from_row(row)

        return self._transaction(read)

    def approval_history(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
    ) -> list[AuditEvent]:
        def read(connection: Any) -> list[AuditEvent]:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "project:read")
            cursor.execute(
                """
                SELECT id, organization_id, actor_user_id, event_type,
                       target_type, target_id, metadata_json, created_at
                FROM audit_events
                WHERE organization_id = %s
                  AND target_type = 'revision'
                  AND target_id = %s
                  AND event_type IN (
                      'REVISION_SUBMITTED', 'REVISION_APPROVED', 'REVISION_REJECTED',
                      'REVIEWER_ASSIGNED', 'REVIEWER_UNASSIGNED'
                  )
                  AND metadata_json->>'project_id' = %s
                ORDER BY created_at ASC, id ASC
                """,
                (principal.organization_id, revision_id, project_id),
            )
            return [
                AuditEvent(
                    id=str(row[0]),
                    organization_id=str(row[1]),
                    actor_user_id=None if row[2] is None else str(row[2]),
                    event_type=str(row[3]),
                    target_type=str(row[4]),
                    target_id=str(row[5]),
                    metadata=self._json_value(row[6]),
                    created_at=self._datetime(row[7]) or _utc_now(),
                )
                for row in cursor.fetchall()
            ]

        return self._transaction(read)

    def audit_events(self, principal: Principal, *, target_id: str | None = None) -> list[AuditEvent]:
        self.require(principal, "audit:read")

        def read(connection: Any) -> list[AuditEvent]:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, actor_user_id, event_type,
                       target_type, target_id, metadata_json, created_at
                FROM audit_events
                WHERE organization_id = %s
                  AND (
                    %s::text IS NULL
                    OR target_id = %s
                    OR metadata_json->>'project_id' = %s
                  )
                ORDER BY created_at ASC, id ASC
                """,
                (principal.organization_id, target_id, target_id, target_id),
            )
            return [
                AuditEvent(
                    id=str(row[0]),
                    organization_id=str(row[1]),
                    actor_user_id=None if row[2] is None else str(row[2]),
                    event_type=str(row[3]),
                    target_type=str(row[4]),
                    target_id=str(row[5]),
                    metadata=self._json_value(row[6]),
                    created_at=self._datetime(row[7]) or _utc_now(),
                )
                for row in cursor.fetchall()
            ]

        return self._transaction(read)

    def record_event(
        self,
        principal: Principal,
        *,
        project_id: str,
        event_type: str,
        target_type: str,
        target_id: str,
        metadata: dict[str, Any] | None = None,
    ) -> AuditEvent:
        self.require(principal, "project:write")

        def record(connection: Any) -> AuditEvent:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "project:write")
            event_id = _new_id("audit")
            created_at = _utc_now()
            event_metadata = {"project_id": project_id, **dict(metadata or {})}
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                event_type,
                target_type,
                target_id,
                event_metadata,
                event_id=event_id,
                created_at=created_at,
            )
            return AuditEvent(
                id=event_id,
                organization_id=principal.organization_id,
                actor_user_id=principal.user_id,
                event_type=event_type,
                target_type=target_type,
                target_id=target_id,
                metadata=event_metadata,
                created_at=created_at,
            )

        return self._transaction(record)

    def record_artifact(
        self,
        principal: Principal,
        *,
        project_id: str,
        revision_id: str,
        artifact_type: str,
        filename: str,
        content: bytes,
        artifact_id: str,
        created_at: datetime,
        storage_backend: str = "response-only",
    ) -> ArtifactRecord:
        """Persist exact delivery metadata without claiming blob retention."""

        self.require(principal, "report:read")
        if not artifact_id.strip() or not artifact_type.strip() or not filename.strip():
            raise ValueError("Artifact id, type, and filename are required.")
        if not content:
            raise ValueError("Artifact content cannot be empty.")
        artifact = ArtifactRecord(
            id=artifact_id.strip(),
            organization_id=principal.organization_id,
            project_id=project_id,
            revision_id=revision_id,
            artifact_type=artifact_type.strip(),
            filename=filename.strip(),
            content_sha256=hashlib.sha256(content).hexdigest(),
            byte_size=len(content),
            created_by=principal.user_id,
            created_at=created_at,
            storage_backend=storage_backend.strip() or "response-only",
        )

        def persist(connection: Any) -> ArtifactRecord:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "report:read")
            cursor.execute(
                "SELECT 1 FROM project_revisions WHERE project_id = %s AND id = %s",
                (project_id, revision_id),
            )
            if cursor.fetchone() is None:
                raise ValueError("Revision does not exist for this project.")
            cursor.execute(
                """
                INSERT INTO artifact_records
                    (id, organization_id, project_id, revision_id, artifact_type,
                     filename, content_sha256, byte_size, created_by, created_at,
                     storage_backend)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                """,
                (
                    artifact.id,
                    artifact.organization_id,
                    artifact.project_id,
                    artifact.revision_id,
                    artifact.artifact_type,
                    artifact.filename,
                    artifact.content_sha256,
                    artifact.byte_size,
                    artifact.created_by,
                    artifact.created_at,
                    artifact.storage_backend,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                artifact.organization_id,
                principal.user_id,
                "ARTIFACT_RECORDED",
                "artifact",
                artifact.id,
                {
                    "project_id": artifact.project_id,
                    "revision_id": artifact.revision_id,
                    "artifact_type": artifact.artifact_type,
                    "filename": artifact.filename,
                    "content_sha256": artifact.content_sha256,
                    "byte_size": artifact.byte_size,
                    "storage_backend": artifact.storage_backend,
                },
            )
            return artifact

        return self._transaction(persist)

    def list_artifacts(
        self,
        principal: Principal,
        project_id: str,
        revision_id: str,
    ) -> list[ArtifactRecord]:
        self.require(principal, "report:read")

        def read(connection: Any) -> list[ArtifactRecord]:
            cursor = connection.cursor()
            self._check_project_cursor(cursor, principal, project_id, "report:read")
            cursor.execute(
                """
                SELECT id, organization_id, project_id, revision_id, artifact_type,
                       filename, content_sha256, byte_size, created_by, created_at,
                       storage_backend
                FROM artifact_records
                WHERE organization_id = %s AND project_id = %s AND revision_id = %s
                ORDER BY created_at ASC, id ASC
                """,
                (principal.organization_id, project_id, revision_id),
            )
            return [
                ArtifactRecord(
                    id=str(row[0]),
                    organization_id=str(row[1]),
                    project_id=str(row[2]),
                    revision_id=str(row[3]),
                    artifact_type=str(row[4]),
                    filename=str(row[5]),
                    content_sha256=str(row[6]),
                    byte_size=int(row[7]),
                    created_by=str(row[8]),
                    created_at=self._datetime(row[9]) or _utc_now(),
                    storage_backend=str(row[10]),
                )
                for row in cursor.fetchall()
            ]

        return self._transaction(read)

    def create_job(
        self,
        principal: Principal,
        *,
        kind: str,
        request: dict[str, Any],
        project_id: str | None = None,
    ) -> EngineeringJob:
        self.require(principal, "job:submit")

        def create(connection: Any) -> EngineeringJob:
            cursor = connection.cursor()
            if project_id is not None:
                self._check_project_cursor(cursor, principal, project_id, "project:read")
            job = EngineeringJob(
                id=_new_id("job"),
                organization_id=principal.organization_id,
                submitted_by=principal.user_id,
                project_id=project_id,
                kind=kind,
                request=dict(request),
            )
            cursor.execute(
                """
                INSERT INTO engineering_jobs
                    (id, organization_id, submitted_by, project_id, kind,
                     request_json, status, progress, created_at)
                VALUES (%s, %s, %s, %s, %s, %s::jsonb, %s, %s, %s)
                """,
                (
                    job.id,
                    job.organization_id,
                    job.submitted_by,
                    job.project_id,
                    job.kind,
                    json.dumps(job.request),
                    job.status.value,
                    job.progress,
                    job.created_at,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                "JOB_QUEUED",
                "job",
                job.id,
                {"kind": kind, "project_id": project_id},
            )
            return job

        return self._transaction(create)

    def get_job(self, principal: Principal, job_id: str) -> EngineeringJob:
        self.require(principal, "project:read")

        def read(connection: Any) -> EngineeringJob:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, submitted_by, project_id, kind,
                       request_json, status, progress, result_json, error,
                       created_at, started_at, completed_at, claimed_by, lease_token
                FROM engineering_jobs
                WHERE id = %s AND organization_id = %s
                """,
                (job_id, principal.organization_id),
            )
            row = cursor.fetchone()
            if row is None:
                raise KeyError(job_id)
            return self._job_from_row(row)

        return self._transaction(read)

    def update_job(
        self,
        principal: Principal,
        job_id: str,
        *,
        status: JobStatus,
        progress: int | None = None,
        result: dict[str, Any] | None = None,
        error: str | None = None,
    ) -> EngineeringJob:
        self.require(principal, "project:read")

        def update(connection: Any) -> EngineeringJob:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, submitted_by, project_id, kind,
                       request_json, status, progress, result_json, error,
                       created_at, started_at, completed_at, claimed_by, lease_token
                FROM engineering_jobs
                WHERE id = %s AND organization_id = %s
                """,
                (job_id, principal.organization_id),
            )
            row = cursor.fetchone()
            if row is None:
                raise KeyError(job_id)
            job = self._job_from_row(row)
            if status == JobStatus.RUNNING and job.status != JobStatus.QUEUED:
                raise ValueError("Only queued jobs can start.")
            if status in {JobStatus.SUCCEEDED, JobStatus.FAILED, JobStatus.CANCELLED} and job.status not in {
                JobStatus.RUNNING,
                JobStatus.QUEUED,
            }:
                raise ValueError("Only active jobs can finish.")
            now = _utc_now()
            if status == JobStatus.RUNNING:
                job.started_at = now
            if status in {JobStatus.SUCCEEDED, JobStatus.FAILED, JobStatus.CANCELLED}:
                job.completed_at = now
            if progress is not None:
                job.progress = max(0, min(100, int(progress)))
            job.status = JobStatus(status)
            job.result = result
            job.error = error
            cursor.execute(
                """
                UPDATE engineering_jobs
                SET status = %s, progress = %s, result_json = %s::jsonb,
                    error = %s, started_at = %s, completed_at = %s
                WHERE id = %s AND organization_id = %s
                """,
                (
                    job.status.value,
                    job.progress,
                    json.dumps(job.result) if job.result is not None else "null",
                    job.error,
                    job.started_at,
                    job.completed_at,
                    job.id,
                    principal.organization_id,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                principal.organization_id,
                principal.user_id,
                f"JOB_{job.status.value.upper()}",
                "job",
                job.id,
                {"progress": job.progress},
            )
            return job

        return self._transaction(update)

    def record_worker_heartbeat(
        self,
        *,
        worker_id: str,
        status: str = "idle",
        job_id: str | None = None,
    ) -> None:
        """Publish liveness for a PostgreSQL worker without creating audit noise."""

        worker = worker_id.strip()
        normalized_status = status.strip().lower()
        if not worker:
            raise ValueError("worker_id is required.")
        if normalized_status not in {"idle", "running"}:
            raise ValueError("worker status must be idle or running.")
        normalized_job_id = job_id.strip() if job_id is not None else None
        if normalized_status == "idle":
            normalized_job_id = None

        def heartbeat(connection: Any) -> None:
            connection.cursor().execute(
                """
                INSERT INTO worker_heartbeats (worker_id, last_seen, status, job_id, updated_at)
                VALUES (%s, %s, %s, %s, %s)
                ON CONFLICT (worker_id) DO UPDATE SET
                    last_seen = EXCLUDED.last_seen,
                    status = EXCLUDED.status,
                    job_id = EXCLUDED.job_id,
                    updated_at = EXCLUDED.updated_at
                """,
                (worker, _utc_now(), normalized_status, normalized_job_id, _utc_now()),
            )

        self._transaction(heartbeat)

    def worker_health(self, *, max_age_seconds: float = 120.0) -> dict[str, Any]:
        """Return whether at least one worker heartbeat is fresh."""

        if not math.isfinite(max_age_seconds) or max_age_seconds <= 0.0:
            raise ValueError("max_age_seconds must be positive and finite.")

        def read(connection: Any) -> dict[str, Any]:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT count(*)
                FROM worker_heartbeats
                WHERE last_seen > now() - (%s * interval '1 second')
                """,
                (max_age_seconds,),
            )
            worker_count = int((cursor.fetchone() or (0,))[0])
            return {
                "healthy": worker_count > 0,
                "worker_count": worker_count,
                "max_age_seconds": max_age_seconds,
            }

        return self._transaction(read)

    def claim_next_job(
        self,
        *,
        worker_id: str,
        kind: str | None = None,
    ) -> EngineeringJob | None:
        """Atomically claim one queued job for an external worker process."""

        worker = worker_id.strip()
        if not worker:
            raise ValueError("worker_id is required.")

        def claim(connection: Any) -> EngineeringJob | None:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, submitted_by, project_id, kind,
                       request_json, status, progress, result_json, error,
                       created_at, started_at, completed_at, claimed_by, lease_token
                FROM engineering_jobs
                WHERE status = %s
                  AND (%s::text IS NULL OR kind = %s)
                ORDER BY created_at ASC, id ASC
                FOR UPDATE SKIP LOCKED
                LIMIT 1
                """,
                (JobStatus.QUEUED.value, kind, kind),
            )
            row = cursor.fetchone()
            if row is None:
                return None
            job = self._job_from_row(row)
            now = _utc_now()
            lease_id = secrets.token_urlsafe(24)
            cursor.execute(
                """
                UPDATE engineering_jobs
                SET status = %s, progress = %s, started_at = %s,
                    completed_at = NULL, result_json = NULL, error = NULL,
                    claimed_by = %s, lease_token = %s
                WHERE id = %s AND status = %s
                """,
                (
                    JobStatus.RUNNING.value,
                    5,
                    now,
                    worker,
                    lease_id,
                    job.id,
                    JobStatus.QUEUED.value,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                job.organization_id,
                None,
                "JOB_RUNNING",
                "job",
                job.id,
                {"progress": 5, "worker_id": worker},
            )
            job.status = JobStatus.RUNNING
            job.progress = 5
            job.started_at = now
            job.completed_at = None
            job.result = None
            job.error = None
            job.worker_id = worker
            job.lease_id = lease_id
            return job

        return self._transaction(claim)

    def finish_claimed_job(
        self,
        job: EngineeringJob,
        *,
        status: JobStatus,
        worker_id: str,
        result: dict[str, Any] | None = None,
        error: str | None = None,
    ) -> EngineeringJob:
        """Persist the terminal result of a job claimed by a worker."""

        if status not in {JobStatus.SUCCEEDED, JobStatus.FAILED, JobStatus.CANCELLED}:
            raise ValueError("A claimed job can only be finished with a terminal status.")
        worker = worker_id.strip()
        if not worker:
            raise ValueError("worker_id is required.")

        def finish(connection: Any) -> EngineeringJob:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, submitted_by, project_id, kind,
                       request_json, status, progress, result_json, error,
                       created_at, started_at, completed_at, claimed_by, lease_token
                FROM engineering_jobs
                WHERE id = %s AND organization_id = %s
                FOR UPDATE
                """,
                (job.id, job.organization_id),
            )
            row = cursor.fetchone()
            if row is None:
                raise KeyError(job.id)
            current = self._job_from_row(row)
            if current.status != JobStatus.RUNNING:
                raise ValueError("Only running jobs can be finished by a worker.")
            if current.worker_id != worker or not job.lease_id or current.lease_id != job.lease_id:
                raise ValueError("The worker lease is no longer active for this job.")
            current.status = status
            current.progress = 100
            current.result = result
            current.error = error
            current.completed_at = _utc_now()
            current.worker_id = None
            current.lease_id = None
            cursor.execute(
                """
                UPDATE engineering_jobs
                SET status = %s, progress = %s, result_json = %s::jsonb,
                    error = %s, completed_at = %s,
                    claimed_by = NULL, lease_token = NULL
                WHERE id = %s AND organization_id = %s AND status = %s
                  AND claimed_by = %s AND lease_token = %s
                """,
                (
                    status.value,
                    current.progress,
                    json.dumps(result) if result is not None else "null",
                    error,
                    current.completed_at,
                    current.id,
                    current.organization_id,
                    JobStatus.RUNNING.value,
                    worker,
                    job.lease_id,
                ),
            )
            self._insert_audit_cursor(
                cursor,
                current.organization_id,
                None,
                f"JOB_{status.value.upper()}",
                "job",
                current.id,
                {"progress": current.progress, "worker_id": worker},
            )
            return current

        return self._transaction(finish)

    def requeue_stale_jobs(
        self,
        *,
        older_than_seconds: float = 900.0,
        worker_id: str = "",
    ) -> int:
        """Return abandoned running jobs to the queue after a worker crash."""

        if not math.isfinite(older_than_seconds) or older_than_seconds <= 0.0:
            raise ValueError("older_than_seconds must be positive and finite.")
        worker = worker_id.strip() or "recovery"

        def requeue(connection: Any) -> int:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id
                FROM engineering_jobs
                WHERE status = %s
                  AND started_at IS NOT NULL
                  AND started_at < now() - (%s * interval '1 second')
                ORDER BY started_at ASC, id ASC
                FOR UPDATE SKIP LOCKED
                """,
                (JobStatus.RUNNING.value, older_than_seconds),
            )
            rows = cursor.fetchall()
            for job_id, organization_id in rows:
                cursor.execute(
                    """
                    UPDATE engineering_jobs
                    SET status = %s, progress = 0, started_at = NULL,
                        completed_at = NULL, result_json = NULL,
                        error = %s, claimed_by = NULL, lease_token = NULL
                    WHERE id = %s AND status = %s
                    """,
                    (
                        JobStatus.QUEUED.value,
                        "Requeued after worker lease expired.",
                        job_id,
                        JobStatus.RUNNING.value,
                    ),
                )
                self._insert_audit_cursor(
                    cursor,
                    str(organization_id),
                    None,
                    "JOB_REQUEUED",
                    "job",
                    str(job_id),
                    {"worker_id": worker, "reason": "stale worker lease"},
                )
            return len(rows)

        return self._transaction(requeue)


class PostgreSQLJobWorker:
    """Durable PostgreSQL-backed worker for jobs submitted by the API."""

    def __init__(
        self,
        store: PostgreSQLSaaSStore,
        *,
        operations: dict[str, Callable[[dict[str, Any]], dict[str, Any]]],
        worker_id: str,
        stale_after_seconds: float = 900.0,
    ) -> None:
        if not operations:
            raise ValueError("At least one job operation is required.")
        if not worker_id.strip():
            raise ValueError("worker_id is required.")
        if not math.isfinite(stale_after_seconds) or stale_after_seconds <= 0.0:
            raise ValueError("stale_after_seconds must be positive and finite.")
        self._store = store
        self._operations = dict(operations)
        self._worker_id = worker_id.strip()
        self._stale_after_seconds = stale_after_seconds

    def run_once(self, *, kind: str | None = None) -> EngineeringJob | None:
        self._store.record_worker_heartbeat(
            worker_id=self._worker_id,
            status="idle",
        )
        self._store.requeue_stale_jobs(
            older_than_seconds=self._stale_after_seconds,
            worker_id=self._worker_id,
        )
        job = self._store.claim_next_job(worker_id=self._worker_id, kind=kind)
        if job is None:
            return None
        self._store.record_worker_heartbeat(
            worker_id=self._worker_id,
            status="running",
            job_id=job.id,
        )
        try:
            operation = self._operations.get(job.kind)
            if operation is None:
                return self._store.finish_claimed_job(
                    job,
                    status=JobStatus.FAILED,
                    worker_id=self._worker_id,
                    error=f"No worker operation is registered for job kind {job.kind!r}.",
                )
            try:
                result = operation(job.request)
            except Exception as error:  # noqa: BLE001 - persist worker failures for polling clients
                return self._store.finish_claimed_job(
                    job,
                    status=JobStatus.FAILED,
                    worker_id=self._worker_id,
                    error=str(error),
                )
            return self._store.finish_claimed_job(
                job,
                status=JobStatus.SUCCEEDED,
                worker_id=self._worker_id,
                result=result,
            )
        finally:
            # A lost lease must not leave the readiness heartbeat stuck in
            # running state while another worker owns the replacement claim.
            self._store.record_worker_heartbeat(
                worker_id=self._worker_id,
                status="idle",
            )

    def run_forever(
        self,
        *,
        poll_seconds: float = 1.0,
        kind: str | None = None,
        stop_event: Event | None = None,
    ) -> None:
        if not math.isfinite(poll_seconds) or poll_seconds <= 0.0:
            raise ValueError("poll_seconds must be positive and finite.")
        event = stop_event or Event()
        while not event.is_set():
            try:
                job = self.run_once(kind=kind)
            except Exception as error:  # noqa: BLE001 - keep the supervisor alive after a lease race
                print(f"EasyTowing worker cycle failed: {error}")
                event.wait(poll_seconds)
                continue
            if job is None:
                event.wait(poll_seconds)
