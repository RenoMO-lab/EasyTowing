from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile
import time
import unittest
from datetime import datetime, timezone

from easytowing.saas import (
    ArtifactStorageError,
    ApprovalStatus,
    EngineeringJob,
    EngineeringJobRunner,
    FileArtifactStore,
    JobStatus,
    PostgreSQLJobWorker,
    Principal,
    SaaSAuthorizationError,
    SaaSBootstrapError,
    SaaSControlStore,
    UserRole,
)


class FakePostgreSQLWorkerStore:
    def __init__(self, job: EngineeringJob | None = None) -> None:
        self.job = job
        self.heartbeats: list[tuple[str, str, str | None]] = []

    def record_worker_heartbeat(
        self,
        *,
        worker_id: str,
        status: str = "idle",
        job_id: str | None = None,
    ) -> None:
        self.heartbeats.append((worker_id, status, job_id))

    def requeue_stale_jobs(self, *, older_than_seconds: float, worker_id: str) -> int:
        return 0

    def claim_next_job(self, *, worker_id: str, kind: str | None = None) -> EngineeringJob | None:
        if self.job is None or self.job.status != JobStatus.QUEUED:
            return None
        if kind is not None and self.job.kind != kind:
            return None
        self.job.status = JobStatus.RUNNING
        return self.job

    def finish_claimed_job(
        self,
        job: EngineeringJob,
        *,
        status: JobStatus,
        worker_id: str,
        result: dict[str, object] | None = None,
        error: str | None = None,
    ) -> EngineeringJob:
        job.status = status
        job.result = result
        job.error = error
        return job


class SaaSControlTests(unittest.TestCase):
    def setUp(self) -> None:
        self.store = SaaSControlStore()
        self.store.create_user("monroc", "designer@monroc.example", "correct horse battery staple", role=UserRole.DESIGNER)
        self.store.create_user("monroc", "reviewer@monroc.example", "reviewer password 123", role=UserRole.REVIEWER)
        self.store.create_user("monroc", "admin@monroc.example", "admin password 123", role=UserRole.ADMIN)
        self.store.create_user("other", "designer@other.example", "other password 123", role=UserRole.DESIGNER)
        self.designer_token, self.designer = self.store.login("monroc", "designer@monroc.example", "correct horse battery staple")
        self.reviewer_token, self.reviewer = self.store.login("monroc", "reviewer@monroc.example", "reviewer password 123")
        self.admin_token, self.admin = self.store.login("monroc", "admin@monroc.example", "admin password 123")
        self.other_token, self.other = self.store.login("other", "designer@other.example", "other password 123")
        self.store.bind_project(self.designer, "project_1")

    def test_bootstrap_creates_only_the_first_administrator(self) -> None:
        store = SaaSControlStore()
        account = store.bootstrap_admin(
            "new-tenant",
            "owner@example.com",
            "owner password 123",
            display_name="Owner",
            organization_name="Ignored by local store",
        )

        self.assertEqual(account.role, UserRole.ADMIN)
        self.assertEqual(account.display_name, "Owner")
        with self.assertRaises(SaaSBootstrapError):
            store.bootstrap_admin(
                "new-tenant",
                "second-owner@example.com",
                "second owner password 123",
            )
        with self.assertRaises(SaaSBootstrapError):
            store.bootstrap_admin(
                "another-tenant",
                "another-owner@example.com",
                "another owner password 123",
            )
        self.assertEqual(
            [event.event_type for event in store.audit_events(
                Principal(
                    user_id=account.id,
                    organization_id=account.organization_id,
                    email=account.email,
                    role=account.role,
                    display_name=account.display_name,
                ),
            )],
            ["BOOTSTRAP_ADMIN_CREATED"],
        )

    def test_reviewer_assignment_is_tenant_scoped_and_audited(self) -> None:
        users = self.store.list_users(self.admin)
        self.assertEqual({user.email for user in users}, {"designer@monroc.example", "reviewer@monroc.example", "admin@monroc.example"})
        with self.assertRaises(SaaSAuthorizationError):
            self.store.list_users(self.reviewer)

        assigned = self.store.assign_reviewer(
            self.admin,
            "project_1",
            "revision_1",
            self.reviewer.user_id,
        )
        self.assertEqual(assigned.assigned_reviewer_id, self.reviewer.user_id)
        self.store.submit_revision(self.designer, "project_1", "revision_1")
        with self.assertRaises(SaaSAuthorizationError):
            self.store.decide_revision(
                self.admin,
                "project_1",
                "revision_1",
                approved=True,
            )
        approved = self.store.decide_revision(
            self.reviewer,
            "project_1",
            "revision_1",
            approved=True,
        )
        self.assertEqual(approved.assigned_reviewer_id, self.reviewer.user_id)
        self.assertEqual(
            [event.event_type for event in self.store.approval_history(self.reviewer, "project_1", "revision_1")],
            ["REVIEWER_ASSIGNED", "REVISION_SUBMITTED", "REVISION_APPROVED"],
        )

    def test_tenant_and_role_boundaries(self) -> None:
        with self.assertRaises(SaaSAuthorizationError):
            self.store.bind_project(self.other, "project_1")
        with self.assertRaises(SaaSAuthorizationError):
            self.store.submit_revision(self.reviewer, "project_1", "revision_1")
        with self.assertRaises(SaaSAuthorizationError):
            self.store.get_approval(self.other, "project_1", "revision_1")

    def test_submission_requires_independent_reviewer_approval(self) -> None:
        submitted = self.store.submit_revision(self.designer, "project_1", "revision_1", note="Ready for review")
        self.assertEqual(submitted.status, ApprovalStatus.SUBMITTED)
        with self.assertRaises(SaaSAuthorizationError):
            self.store.decide_revision(self.designer, "project_1", "revision_1", approved=True)
        approved = self.store.decide_revision(
            self.reviewer,
            "project_1",
            "revision_1",
            approved=True,
            note="Reviewed against pilot checklist",
        )
        self.assertEqual(approved.status, ApprovalStatus.APPROVED)
        events = self.store.audit_events(self.reviewer, target_id="revision_1")
        self.assertEqual([event.event_type for event in events], ["REVISION_SUBMITTED", "REVISION_APPROVED"])
        history = self.store.approval_history(self.reviewer, "project_1", "revision_1")
        self.assertEqual([event.event_type for event in history], ["REVISION_SUBMITTED", "REVISION_APPROVED"])

    def test_background_job_has_pollable_lifecycle(self) -> None:
        runner = EngineeringJobRunner(self.store, max_workers=1)
        try:
            job = runner.submit(
                self.designer,
                kind="optimization",
                request={"mode": "quick"},
                project_id="project_1",
                operation=lambda request: {"mode": request["mode"], "feasible": True},
            )
            for _ in range(30):
                current = self.store.get_job(self.designer, job.id)
                if current.status == JobStatus.SUCCEEDED:
                    break
                time.sleep(0.01)
            current = self.store.get_job(self.designer, job.id)
            self.assertEqual(current.status, JobStatus.SUCCEEDED)
            self.assertEqual(current.result, {"mode": "quick", "feasible": True})
            self.assertEqual(current.progress, 100)
        finally:
            runner.shutdown()

    def test_postgres_worker_publishes_idle_running_and_idle_heartbeats(self) -> None:
        job = EngineeringJob(
            id="job_worker_heartbeat",
            organization_id="monroc",
            submitted_by="designer",
            project_id="project_1",
            kind="optimization",
            request={"mode": "quick"},
        )
        store = FakePostgreSQLWorkerStore(job)
        worker = PostgreSQLJobWorker(
            store,  # type: ignore[arg-type]
            operations={"optimization": lambda request: {"feasible": True}},
            worker_id="worker-01",
        )

        completed = worker.run_once(kind="optimization")

        self.assertIs(completed, job)
        self.assertEqual(completed.status, JobStatus.SUCCEEDED)
        self.assertEqual(
            store.heartbeats,
            [
                ("worker-01", "idle", None),
                ("worker-01", "running", "job_worker_heartbeat"),
                ("worker-01", "idle", None),
            ],
        )

    def test_controlled_artifact_records_exact_delivery_metadata(self) -> None:
        content = b'{"release_status":"APPROVED"}\n'
        created_at = datetime(2026, 8, 27, tzinfo=timezone.utc)
        artifact = self.store.record_artifact(
            self.reviewer,
            project_id="project_1",
            revision_id="revision_1",
            artifact_type="controlled-engineering-release-manifest",
            filename="release.json",
            content=content,
            artifact_id="artifact_test_1",
            created_at=created_at,
        )

        self.assertEqual(artifact.content_sha256, hashlib.sha256(content).hexdigest())
        self.assertEqual(artifact.byte_size, len(content))
        self.assertEqual(self.store.list_artifacts(self.reviewer, "project_1", "revision_1"), [artifact])
        self.assertEqual(
            [event.event_type for event in self.store.audit_events(self.reviewer, target_id=artifact.id)],
            ["ARTIFACT_RECORDED"],
        )
        with self.assertRaises(SaaSAuthorizationError):
            self.store.list_artifacts(self.other, "project_1", "revision_1")

    def test_file_artifact_store_retains_and_verifies_bytes(self) -> None:
        content = b"controlled release bytes\n"
        digest = hashlib.sha256(content).hexdigest()
        with tempfile.TemporaryDirectory() as root:
            store = FileArtifactStore(Path(root))
            path = store.put(
                "artifact_filesystem_1",
                content,
                expected_sha256=digest,
                expected_size=len(content),
            )

            self.assertTrue(path.is_file())
            store.health_check()
            self.assertEqual(
                store.read(
                    "artifact_filesystem_1",
                    expected_sha256=digest,
                    expected_size=len(content),
                ),
                content,
            )
            with self.assertRaises(ArtifactStorageError):
                store.read("artifact_filesystem_1", expected_sha256="0" * 64)
            with self.assertRaises(ValueError):
                store.put("../outside", content)

            store.delete("artifact_filesystem_1")
            with self.assertRaises(ArtifactStorageError):
                store.read("artifact_filesystem_1")


if __name__ == "__main__":
    unittest.main()
