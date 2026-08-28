from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class OperationsContractTests(unittest.TestCase):
    def test_postgres_backup_script_fails_closed_before_retention_cleanup(self) -> None:
        script = (ROOT / "ops" / "backup-postgres.ps1").read_text(encoding="utf-8")
        self.assertIn('if ($RetentionDays -lt 1)', script)
        self.assertIn('Get-Command pg_dump', script)
        self.assertIn('Test-Path -LiteralPath $backupPath -PathType Leaf', script)
        self.assertIn('$backupSize -le 0', script)

    def test_compose_profile_separates_api_worker_and_durable_dependencies(self) -> None:
        compose = (ROOT / "ops" / "docker-compose.yml").read_text(encoding="utf-8")
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        env_example = (ROOT / "ops" / ".env.example").read_text(encoding="utf-8")
        saas = (ROOT / "easytowing" / "saas.py").read_text(encoding="utf-8")
        self.assertIn("services:", compose)
        self.assertIn("db:", compose)
        self.assertIn("api:", compose)
        self.assertIn("worker:", compose)
        self.assertIn("postgres-data:", compose)
        self.assertIn("artifacts:", compose)
        self.assertIn("EASYTOWING_REQUIRE_WORKER: \"1\"", compose)
        self.assertIn("condition: service_healthy", compose)
        self.assertIn("pip install --no-cache-dir \".[postgres]\"", dockerfile)
        self.assertIn("EASYTOWING_DATABASE_URL=", env_example)
        self.assertIn("EASYTOWING_BOOTSTRAP_TOKEN=", env_example)
        self.assertIn("pg_advisory_xact_lock(hashtext('easytowing.schema.v1'))", saas)


if __name__ == "__main__":
    unittest.main()
