"""Run durable PostgreSQL engineering jobs outside the API process."""

from __future__ import annotations

import argparse
import os
from uuid import uuid4

from .saas import PostgreSQLJobWorker, PostgreSQLSaaSStore


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the EasyTowing PostgreSQL job worker.")
    parser.add_argument(
        "--database-url",
        default=os.environ.get("EASYTOWING_DATABASE_URL", ""),
        help="PostgreSQL DSN, or EASYTOWING_DATABASE_URL",
    )
    parser.add_argument(
        "--worker-id",
        default=os.environ.get("EASYTOWING_WORKER_ID", f"worker-{uuid4().hex[:10]}"),
    )
    parser.add_argument("--poll-seconds", type=float, default=1.0)
    parser.add_argument("--stale-after-seconds", type=float, default=900.0)
    parser.add_argument("--kind", default="", help="Process only one job kind when provided.")
    args = parser.parse_args()
    database_url = str(args.database_url).strip()
    if not database_url:
        raise SystemExit("EASYTOWING_DATABASE_URL or --database-url is required.")

    # Load the engineering operation only after the worker target is explicit.
    os.environ["EASYTOWING_DATABASE_URL"] = database_url
    from .demo_server import _optimization_job_payload

    store = PostgreSQLSaaSStore(database_url)
    store.migrate()
    worker = PostgreSQLJobWorker(
        store,
        operations={"optimization": _optimization_job_payload},
        worker_id=str(args.worker_id),
        stale_after_seconds=float(args.stale_after_seconds),
    )
    print(f"EasyTowing worker {args.worker_id} polling PostgreSQL")
    try:
        worker.run_forever(
            poll_seconds=float(args.poll_seconds),
            kind=str(args.kind).strip() or None,
        )
    except KeyboardInterrupt:
        print("EasyTowing worker stopped")


if __name__ == "__main__":
    main()
