from __future__ import annotations

import importlib.util
import hashlib
import os
import time
import unittest
from datetime import datetime, timezone
from uuid import uuid4

from easytowing.projects import PostgreSQLProjectStore
from easytowing.saas import (
    ApprovalStatus,
    EngineeringJobRunner,
    JobStatus,
    PostgreSQLSaaSStore,
    PostgreSQLJobWorker,
    UserRole,
)


TEST_DATABASE_URL = os.environ.get("EASYTOWING_TEST_DATABASE_URL", "").strip()
POSTGRES_AVAILABLE = importlib.util.find_spec("psycopg") is not None


@unittest.skipUnless(
    TEST_DATABASE_URL and POSTGRES_AVAILABLE,
    "Set EASYTOWING_TEST_DATABASE_URL and install psycopg[binary] to run PostgreSQL integration tests.",
)
class PostgreSQLAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.saas = PostgreSQLSaaSStore(TEST_DATABASE_URL)
        self.projects = PostgreSQLProjectStore(TEST_DATABASE_URL)
        self.saas.migrate()
        suffix = uuid4().hex[:10]
        self.organization_id = f"test-org-{suffix}"
        self.saas.create_organization(self.organization_id, "Adapter Test Organization")
        self.designer_account = self.saas.create_user(
            self.organization_id,
            f"designer-{suffix}@test.example",
            "designer password 123",
            role=UserRole.DESIGNER,
        )
        self.reviewer_account = self.saas.create_user(
            self.organization_id,
            f"reviewer-{suffix}@test.example",
            "reviewer password 123",
            role=UserRole.REVIEWER,
        )
        _designer_token, self.designer = self.saas.login(
            self.organization_id,
            self.designer_account.email,
            "designer password 123",
        )
        _reviewer_token, self.reviewer = self.saas.login(
            self.organization_id,
            self.reviewer_account.email,
            "reviewer password 123",
        )
        self.project = self.projects.create_project(
            "PostgreSQL adapter project",
            organization_id=self.organization_id,
        )
        self.saas.bind_project(self.designer, self.project.id)

    def test_project_revision_approval_job_and_audit_round_trip(self) -> None:
        revision = self.projects.append_revision(
            self.project.id,
            organization_id=self.organization_id,
            beta_deg=5.0,
            optimization_mode="quick",
            note="Database revision",
        )
        reloaded = PostgreSQLProjectStore(TEST_DATABASE_URL).get_project(
            self.project.id,
            self.organization_id,
        )
        self.assertIsNotNone(reloaded)
        assert reloaded is not None
        self.assertEqual(reloaded.organization_id, self.organization_id)
        self.assertEqual(reloaded.active_revision_id, revision.id)
        self.assertEqual(len(reloaded.revisions), 2)

        submitted = self.saas.submit_revision(
            self.designer,
            self.project.id,
            revision.id,
            note="Ready for review",
        )
        self.assertEqual(submitted.status, ApprovalStatus.SUBMITTED)
        rejected = self.saas.decide_revision(
            self.reviewer,
            self.project.id,
            revision.id,
            approved=False,
            note="Needs pilot review",
        )
        self.assertEqual(rejected.status, ApprovalStatus.REJECTED)

        runner = EngineeringJobRunner(self.saas, max_workers=1)
        try:
            job = runner.submit(
                self.designer,
                kind="optimization",
                request={"mode": "quick"},
                project_id=self.project.id,
                operation=lambda request: {"persisted": True, "mode": request["mode"]},
            )
            for _ in range(50):
                if self.saas.get_job(self.designer, job.id).status == JobStatus.SUCCEEDED:
                    break
                time.sleep(0.02)
            completed = self.saas.get_job(self.designer, job.id)
            self.assertEqual(completed.status, JobStatus.SUCCEEDED)
            self.assertEqual(completed.result, {"persisted": True, "mode": "quick"})
        finally:
            runner.shutdown()

        events = self.saas.audit_events(self.reviewer, target_id=revision.id)
        self.assertEqual(
            [event.event_type for event in events],
            ["REVISION_SUBMITTED", "REVISION_REJECTED"],
        )
        history = self.saas.approval_history(self.reviewer, self.project.id, revision.id)
        self.assertEqual(
            [event.event_type for event in history],
            ["REVISION_SUBMITTED", "REVISION_REJECTED"],
        )

    def test_postgres_worker_claims_and_finishes_a_job_durably(self) -> None:
        job = self.saas.create_job(
            self.designer,
            kind="durable-test",
            request={"mode": "quick", "case": "durable-worker"},
            project_id=self.project.id,
        )
        worker = PostgreSQLJobWorker(
            self.saas,
            operations={
                "optimization": lambda request: {
                    "persisted": True,
                    "case": request["case"],
                },
                "durable-test": lambda request: {
                    "persisted": True,
                    "case": request["case"],
                },
            },
            worker_id="integration-worker",
        )

        completed = worker.run_once(kind="durable-test")

        self.assertIsNotNone(completed)
        assert completed is not None
        self.assertEqual(completed.id, job.id)
        self.assertEqual(completed.status, JobStatus.SUCCEEDED)
        self.assertEqual(completed.result, {"persisted": True, "case": "durable-worker"})
        reloaded = self.saas.get_job(self.designer, job.id)
        self.assertEqual(reloaded.status, JobStatus.SUCCEEDED)
        self.assertEqual(reloaded.progress, 100)
        self.assertEqual(
            [event.event_type for event in self.saas.audit_events(self.reviewer, target_id=job.id)],
            ["JOB_QUEUED", "JOB_RUNNING", "JOB_SUCCEEDED"],
        )

    def test_postgres_artifact_ledger_round_trip(self) -> None:
        content = b'{"release_status":"APPROVED"}\n'
        created_at = datetime(2026, 8, 27, tzinfo=timezone.utc)
        artifact = self.saas.record_artifact(
            self.reviewer,
            project_id=self.project.id,
            revision_id=self.project.active_revision_id or "revision_1",
            artifact_type="controlled-engineering-release-manifest",
            filename="release.json",
            content=content,
            artifact_id=f"artifact-{uuid4().hex[:10]}",
            created_at=created_at,
        )

        self.assertEqual(artifact.content_sha256, hashlib.sha256(content).hexdigest())
        self.assertEqual(artifact.byte_size, len(content))
        reloaded = self.saas.list_artifacts(
            self.reviewer,
            self.project.id,
            artifact.revision_id,
        )
        self.assertEqual([item.id for item in reloaded], [artifact.id])
        self.assertEqual(
            [event.event_type for event in self.saas.audit_events(self.reviewer, target_id=artifact.id)],
            ["ARTIFACT_RECORDED"],
        )


if __name__ == "__main__":
    unittest.main()
