from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from threading import Lock
from uuid import uuid4
from typing import Any, Callable, Iterable

from .design_cases import DesignCase
from .linkage import LinkageDemoRig
from .model import (
    VehicleLayout,
    build_reference_demo_combination,
    serialize_vehicle_combination,
)
from .errors import OptimizationNoFeasibleSolutionError
from .reporting import build_export_bundle


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _new_id(prefix: str) -> str:
    return f"{prefix}_{uuid4().hex[:10]}"


def _reference_combination_seed() -> dict[str, Any]:
    """Return a multi-body starting model for a new engineering workspace."""

    combination = serialize_vehicle_combination(build_reference_demo_combination())
    combination["joint_ranges"] = {
        "front_joint": {
            "min_deg": -45.0,
            "max_deg": 45.0,
            "step_deg": 5.0,
        }
    }
    return combination


@dataclass(slots=True)
class ProjectRevision:
    id: str
    created_at: str
    note: str
    beta_deg: float
    optimization_mode: str
    snapshot: dict[str, Any]
    optimization_enabled_ids: tuple[str, ...] | None = None
    accepted_optimization: bool = False
    beta_min_deg: float = -45.0
    beta_max_deg: float = 45.0
    design_cases: tuple[DesignCase, ...] = ()
    linkage_config: dict[str, Any] | None = None
    wheelbase_mm: float = 4360.0
    track_mm: float = 2500.0
    vehicle_config: dict[str, Any] | None = None
    combination_config: dict[str, Any] | None = None
    root_turn_radius_mm: float | None = None
    mechanism_graph_config: dict[str, Any] | None = None
    mechanism_drivers: tuple[dict[str, Any], ...] = ()
    steering_assignments: tuple[dict[str, Any], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "created_at": self.created_at,
            "note": self.note,
            "beta_deg": self.beta_deg,
            "optimization_mode": self.optimization_mode,
            "snapshot": self.snapshot,
            "optimization_enabled_ids": self.optimization_enabled_ids,
            "accepted_optimization": self.accepted_optimization,
            "beta_min_deg": self.beta_min_deg,
            "beta_max_deg": self.beta_max_deg,
            "design_cases": [case.to_dict() for case in self.design_cases],
            "linkage_config": self.linkage_config,
            "wheelbase_mm": self.wheelbase_mm,
            "track_mm": self.track_mm,
            "vehicle_config": self.vehicle_config,
            "combination_config": self.combination_config,
            "root_turn_radius_mm": self.root_turn_radius_mm,
            "mechanism_graph_config": self.mechanism_graph_config,
            "mechanism_drivers": list(self.mechanism_drivers),
            "steering_assignments": list(self.steering_assignments),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ProjectRevision":
        return cls(
            id=str(data["id"]),
            created_at=str(data["created_at"]),
            note=str(data.get("note", "")),
            beta_deg=float(data["beta_deg"]),
            optimization_mode=str(data.get("optimization_mode", "quick")),
            snapshot=dict(data.get("snapshot", {})),
            optimization_enabled_ids=(
                tuple(str(value) for value in data["optimization_enabled_ids"])
                if isinstance(data.get("optimization_enabled_ids"), list)
                else None
            ),
            accepted_optimization=bool(data.get("accepted_optimization", False)),
            beta_min_deg=float(data.get("beta_min_deg", -45.0)),
            beta_max_deg=float(data.get("beta_max_deg", 45.0)),
            design_cases=tuple(
                DesignCase.from_dict(case)
                for case in data.get("design_cases", [])
                if isinstance(case, dict)
            ),
            linkage_config=(
                dict(data["linkage_config"])
                if isinstance(data.get("linkage_config"), dict)
                else None
            ),
            wheelbase_mm=float(data.get("wheelbase_mm", 4360.0)),
            track_mm=float(data.get("track_mm", 2500.0)),
            vehicle_config=(
                dict(data["vehicle_config"])
                if isinstance(data.get("vehicle_config"), dict)
                else None
            ),
            combination_config=(
                dict(data["combination_config"])
                if isinstance(data.get("combination_config"), dict)
                else None
            ),
            root_turn_radius_mm=(
                float(data["root_turn_radius_mm"])
                if data.get("root_turn_radius_mm") is not None
                else None
            ),
            mechanism_graph_config=(
                dict(data["mechanism_graph_config"])
                if isinstance(data.get("mechanism_graph_config"), dict)
                else None
            ),
            mechanism_drivers=tuple(
                dict(driver)
                for driver in data.get("mechanism_drivers", [])
                if isinstance(driver, dict)
            ),
            steering_assignments=tuple(
                dict(assignment)
                for assignment in data.get("steering_assignments", [])
                if isinstance(assignment, dict)
            ),
        )

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "created_at": self.created_at,
            "note": self.note,
            "beta_deg": self.beta_deg,
            "optimization_mode": self.optimization_mode,
            "optimization_enabled_ids": self.optimization_enabled_ids,
            "accepted_optimization": self.accepted_optimization,
            "beta_min_deg": self.beta_min_deg,
            "beta_max_deg": self.beta_max_deg,
            "design_cases": [case.to_dict() for case in self.design_cases],
            "linkage_config": self.linkage_config,
            "wheelbase_mm": self.wheelbase_mm,
            "track_mm": self.track_mm,
            "vehicle_config": self.vehicle_config,
            "combination_config": self.combination_config,
            "root_turn_radius_mm": self.root_turn_radius_mm,
            "mechanism_graph_config": self.mechanism_graph_config,
            "mechanism_drivers": list(self.mechanism_drivers),
            "steering_assignments": list(self.steering_assignments),
        }


@dataclass(slots=True)
class ProjectRecord:
    id: str
    name: str
    created_at: str
    updated_at: str
    organization_id: str | None = None
    active_revision_id: str | None = None
    revisions: list[ProjectRevision] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "organization_id": self.organization_id,
            "active_revision_id": self.active_revision_id,
            "revisions": [revision.to_dict() for revision in self.revisions],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ProjectRecord":
        return cls(
            id=str(data["id"]),
            name=str(data["name"]),
            created_at=str(data["created_at"]),
            updated_at=str(data["updated_at"]),
            organization_id=(
                str(data["organization_id"])
                if data.get("organization_id") is not None
                else None
            ),
            active_revision_id=data.get("active_revision_id"),
            revisions=[ProjectRevision.from_dict(revision) for revision in data.get("revisions", [])],
        )

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "active_revision_id": self.active_revision_id,
            "revision_count": len(self.revisions),
        }

    def detail(self) -> dict[str, Any]:
        return {
            **self.summary(),
            "revisions": [revision.summary() for revision in self.revisions],
        }

    def get_revision(self, revision_id: str) -> ProjectRevision | None:
        return next((revision for revision in self.revisions if revision.id == revision_id), None)


class ProjectStore:
    def __init__(self, path: Path) -> None:
        self._path = Path(path).expanduser().resolve()
        self._lock = Lock()
        self._projects: dict[str, ProjectRecord] = {}
        self._load()

    @classmethod
    def default(cls) -> "ProjectStore":
        root = Path(__file__).resolve().parents[1]
        return cls(root / ".easytowing-state" / "projects.json")

    def _load(self) -> None:
        if not self._path.exists():
            self._projects = {}
            return
        payload = json.loads(self._path.read_text(encoding="utf-8"))
        self._projects = {
            project_data["id"]: ProjectRecord.from_dict(project_data)
            for project_data in payload.get("projects", [])
        }

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"projects": [project.to_dict() for project in self._projects.values()]}
        self._path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def ensure_seed_project(self, organization_id: str | None = None) -> ProjectRecord:
        with self._lock:
            projects = self._scoped_projects_locked(organization_id)
            if projects:
                return projects[0]
            project = self._create_project_locked(
                "Reference Demo Project",
                organization_id=organization_id,
                combination_config=_reference_combination_seed(),
                root_turn_radius_mm=9000.0,
                beta_min_deg=-45.0,
                beta_max_deg=45.0,
            )
            return project

    def list_projects(self, organization_id: str | None = None) -> list[ProjectRecord]:
        with self._lock:
            return self._scoped_projects_locked(organization_id)

    def get_project(self, project_id: str, organization_id: str | None = None) -> ProjectRecord | None:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None or (
                organization_id is not None and project.organization_id != organization_id
            ):
                return None
            return project

    def _scoped_projects_locked(self, organization_id: str | None) -> list[ProjectRecord]:
        projects = list(self._projects.values())
        if organization_id is None:
            return projects
        return [project for project in projects if project.organization_id == organization_id]

    def create_project(
        self,
        name: str,
        *,
        organization_id: str | None = None,
        beta_deg: float = 0.0,
        optimization_mode: str = "quick",
        note: str = "Initial revision",
        enabled_ids: Iterable[str] | None = None,
        accepted_optimization: bool = False,
        beta_min_deg: float = -45.0,
        beta_max_deg: float = 45.0,
        design_cases: Iterable[DesignCase] | None = None,
        linkage_config: dict[str, Any] | None = None,
        wheelbase_mm: float = 4360.0,
        track_mm: float = 2500.0,
        linkage_rig: LinkageDemoRig | None = None,
        vehicle: VehicleLayout | None = None,
        vehicle_config: dict[str, Any] | None = None,
        combination_config: dict[str, Any] | None = None,
        root_turn_radius_mm: float | None = None,
        mechanism_graph_config: dict[str, Any] | None = None,
        mechanism_drivers: Iterable[dict[str, Any]] | None = None,
        steering_assignments: Iterable[dict[str, Any]] | None = None,
        engineering_snapshot: dict[str, Any] | None = None,
    ) -> ProjectRecord:
        with self._lock:
            return self._create_project_locked(
                name,
                organization_id=organization_id,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
                enabled_ids=enabled_ids,
                accepted_optimization=accepted_optimization,
                beta_min_deg=beta_min_deg,
                beta_max_deg=beta_max_deg,
                design_cases=design_cases,
                linkage_config=linkage_config,
                wheelbase_mm=wheelbase_mm,
                track_mm=track_mm,
                linkage_rig=linkage_rig,
                vehicle=vehicle,
                vehicle_config=vehicle_config,
                combination_config=combination_config,
                root_turn_radius_mm=root_turn_radius_mm,
                mechanism_graph_config=mechanism_graph_config,
                mechanism_drivers=mechanism_drivers,
                steering_assignments=steering_assignments,
                engineering_snapshot=engineering_snapshot,
            )

    def _create_project_locked(
        self,
        name: str,
        *,
        organization_id: str | None = None,
        beta_deg: float = 0.0,
        optimization_mode: str = "quick",
        note: str = "Initial revision",
        enabled_ids: Iterable[str] | None = None,
        accepted_optimization: bool = False,
        beta_min_deg: float = -45.0,
        beta_max_deg: float = 45.0,
        design_cases: Iterable[DesignCase] | None = None,
        linkage_config: dict[str, Any] | None = None,
        wheelbase_mm: float = 4360.0,
        track_mm: float = 2500.0,
        linkage_rig: LinkageDemoRig | None = None,
        vehicle: VehicleLayout | None = None,
        vehicle_config: dict[str, Any] | None = None,
        combination_config: dict[str, Any] | None = None,
        root_turn_radius_mm: float | None = None,
        mechanism_graph_config: dict[str, Any] | None = None,
        mechanism_drivers: Iterable[dict[str, Any]] | None = None,
        steering_assignments: Iterable[dict[str, Any]] | None = None,
        engineering_snapshot: dict[str, Any] | None = None,
    ) -> ProjectRecord:
        created_at = _utc_now_iso()
        project = ProjectRecord(
            id=_new_id("proj"),
            name=name.strip() or "Untitled Project",
            created_at=created_at,
            updated_at=created_at,
            organization_id=organization_id,
        )
        revision = self._create_revision(
            beta_deg=beta_deg,
            optimization_mode=optimization_mode,
            note=note,
            enabled_ids=enabled_ids,
            accepted_optimization=accepted_optimization,
            beta_min_deg=beta_min_deg,
            beta_max_deg=beta_max_deg,
            design_cases=design_cases,
            linkage_config=linkage_config,
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            linkage_rig=linkage_rig,
            vehicle=vehicle,
            vehicle_config=vehicle_config,
            combination_config=combination_config,
            root_turn_radius_mm=root_turn_radius_mm,
            mechanism_graph_config=mechanism_graph_config,
            mechanism_drivers=mechanism_drivers,
            steering_assignments=steering_assignments,
            engineering_snapshot=engineering_snapshot,
        )
        project.revisions.append(revision)
        project.active_revision_id = revision.id
        self._projects[project.id] = project
        self._save()
        return project

    def _create_revision(
        self,
        *,
        beta_deg: float,
        optimization_mode: str,
        note: str,
        enabled_ids: Iterable[str] | None = None,
        accepted_optimization: bool = False,
        beta_min_deg: float = -45.0,
        beta_max_deg: float = 45.0,
        design_cases: Iterable[DesignCase] | None = None,
        linkage_config: dict[str, Any] | None = None,
        wheelbase_mm: float = 4360.0,
        track_mm: float = 2500.0,
        linkage_rig: LinkageDemoRig | None = None,
        vehicle: VehicleLayout | None = None,
        vehicle_config: dict[str, Any] | None = None,
        combination_config: dict[str, Any] | None = None,
        root_turn_radius_mm: float | None = None,
        mechanism_graph_config: dict[str, Any] | None = None,
        mechanism_drivers: Iterable[dict[str, Any]] | None = None,
        steering_assignments: Iterable[dict[str, Any]] | None = None,
        engineering_snapshot: dict[str, Any] | None = None,
    ) -> ProjectRevision:
        if beta_min_deg >= beta_max_deg or beta_min_deg > 0.0 or beta_max_deg < 0.0:
            raise ValueError("Articulation bounds must straddle zero with min below max.")
        if wheelbase_mm <= 0.0 or track_mm <= 0.0:
            raise ValueError("Wheelbase and track must be positive.")
        normalized_enabled_ids = None if enabled_ids is None else tuple(sorted({str(value) for value in enabled_ids}))
        normalized_design_cases = tuple(design_cases or ())
        if accepted_optimization and engineering_snapshot is not None:
            raise ValueError("Graph-backed revisions cannot be accepted by the legacy optimizer.")
        snapshot = (
            dict(engineering_snapshot)
            if engineering_snapshot is not None
            else build_export_bundle(
                beta_deg,
                optimization_mode,
                normalized_enabled_ids,
                normalized_design_cases,
                linkage_rig=linkage_rig,
                vehicle=vehicle,
                require_feasible=accepted_optimization,
            )
        )
        if accepted_optimization:
            baseline_metrics = snapshot["optimization"]["baseline"]
            if not baseline_metrics["feasible"]:
                raise OptimizationNoFeasibleSolutionError(
                    tuple(str(item) for item in baseline_metrics["violations"]),
                    minimum_clearance_mm=baseline_metrics["minimum_clearance_mm"],
                    clearance_target_mm=snapshot["optimization"]["objective"]["clearance_target_mm"],
                )

        return ProjectRevision(
            id=_new_id("rev"),
            created_at=_utc_now_iso(),
            note=note.strip() or "Revision",
            beta_deg=beta_deg,
            optimization_mode=optimization_mode,
            snapshot=snapshot,
            optimization_enabled_ids=normalized_enabled_ids,
            accepted_optimization=accepted_optimization,
            beta_min_deg=beta_min_deg,
            beta_max_deg=beta_max_deg,
            design_cases=normalized_design_cases,
            linkage_config=None if linkage_config is None else dict(linkage_config),
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            vehicle_config=None if vehicle_config is None else dict(vehicle_config),
            combination_config=None if combination_config is None else dict(combination_config),
            root_turn_radius_mm=root_turn_radius_mm,
            mechanism_graph_config=(
                None if mechanism_graph_config is None else dict(mechanism_graph_config)
            ),
            mechanism_drivers=tuple(dict(driver) for driver in (mechanism_drivers or ())),
            steering_assignments=tuple(
                dict(assignment) for assignment in (steering_assignments or ())
            ),
        )

    def append_revision(
        self,
        project_id: str,
        *,
        organization_id: str | None = None,
        beta_deg: float,
        optimization_mode: str,
        note: str,
        enabled_ids: Iterable[str] | None = None,
        accepted_optimization: bool = False,
        beta_min_deg: float = -45.0,
        beta_max_deg: float = 45.0,
        design_cases: Iterable[DesignCase] | None = None,
        linkage_config: dict[str, Any] | None = None,
        wheelbase_mm: float = 4360.0,
        track_mm: float = 2500.0,
        linkage_rig: LinkageDemoRig | None = None,
        vehicle: VehicleLayout | None = None,
        vehicle_config: dict[str, Any] | None = None,
        combination_config: dict[str, Any] | None = None,
        root_turn_radius_mm: float | None = None,
        mechanism_graph_config: dict[str, Any] | None = None,
        mechanism_drivers: Iterable[dict[str, Any]] | None = None,
        steering_assignments: Iterable[dict[str, Any]] | None = None,
        engineering_snapshot: dict[str, Any] | None = None,
    ) -> ProjectRevision:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None or (
                organization_id is not None and project.organization_id != organization_id
            ):
                raise KeyError(project_id)
            revision = self._create_revision(
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
                enabled_ids=enabled_ids,
                accepted_optimization=accepted_optimization,
                beta_min_deg=beta_min_deg,
                beta_max_deg=beta_max_deg,
                design_cases=design_cases,
                linkage_config=linkage_config,
                wheelbase_mm=wheelbase_mm,
                track_mm=track_mm,
                linkage_rig=linkage_rig,
                vehicle=vehicle,
                vehicle_config=vehicle_config,
                combination_config=combination_config,
                root_turn_radius_mm=root_turn_radius_mm,
                mechanism_graph_config=mechanism_graph_config,
                mechanism_drivers=mechanism_drivers,
                steering_assignments=steering_assignments,
                engineering_snapshot=engineering_snapshot,
            )
            project.revisions.append(revision)
            project.active_revision_id = revision.id
            project.updated_at = revision.created_at
            self._save()
            return revision

    def restore_revision(
        self,
        project_id: str,
        revision_id: str,
        organization_id: str | None = None,
    ) -> ProjectRevision:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None or (
                organization_id is not None and project.organization_id != organization_id
            ):
                raise KeyError(project_id)
            revision = project.get_revision(revision_id)
            if revision is None:
                raise KeyError(revision_id)
            project.active_revision_id = revision.id
            project.updated_at = _utc_now_iso()
            self._save()
            return revision

    def get_active_revision(self, project_id: str, organization_id: str | None = None) -> ProjectRevision | None:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None or (
                organization_id is not None and project.organization_id != organization_id
            ) or project.active_revision_id is None:
                return None
            return project.get_revision(project.active_revision_id)

    def get_revision(
        self,
        project_id: str,
        revision_id: str,
        organization_id: str | None = None,
    ) -> ProjectRevision | None:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None or (
                organization_id is not None and project.organization_id != organization_id
            ):
                return None
            return project.get_revision(revision_id)

    def record_acceptance(
        self,
        project_id: str,
        revision_id: str,
        acceptance: dict[str, Any],
        organization_id: str | None = None,
    ) -> ProjectRevision:
        """Append acceptance evidence without changing solver inputs."""

        with self._lock:
            project = self._projects.get(project_id)
            if project is None or (
                organization_id is not None and project.organization_id != organization_id
            ):
                raise KeyError(project_id)
            revision = project.get_revision(revision_id)
            if revision is None:
                raise KeyError(revision_id)
            revision.snapshot = {
                **revision.snapshot,
                "monroc_acceptance": json.loads(json.dumps(acceptance)),
            }
            project.updated_at = _utc_now_iso()
            self._save()
            return revision


class PostgreSQLProjectStore(ProjectStore):
    """PostgreSQL-backed project and immutable revision repository.

    The calculation and serialization model remains shared with `ProjectStore`;
    only persistence changes. Organization ID is mandatory for mutating or
    tenant-scoped reads so a database-backed server cannot accidentally expose
    another organization's project history.
    """

    def __init__(self, dsn: str, *, connect: Callable[[str], Any] | None = None) -> None:
        if not dsn.strip():
            raise ValueError("A PostgreSQL DSN is required.")
        if connect is None:
            try:
                import psycopg  # type: ignore
            except ImportError as error:  # pragma: no cover - deployment extra
                raise RuntimeError("Install psycopg[binary] to use PostgreSQLProjectStore.") from error
            connect = psycopg.connect
        self._dsn = dsn
        self._connect = connect
        self._lock = Lock()

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
        self._transaction(lambda connection: connection.cursor().execute(schema))

    def health_check(self) -> None:
        """Verify that the configured PostgreSQL connection accepts a query."""

        self._transaction(lambda connection: connection.cursor().execute("SELECT 1"))

    @staticmethod
    def _iso_timestamp(value: Any) -> str:
        return value.isoformat() if isinstance(value, datetime) else str(value)

    @staticmethod
    def _json_object(value: Any) -> dict[str, Any]:
        if isinstance(value, str):
            value = json.loads(value)
        if not isinstance(value, dict):
            raise ValueError("Stored project revision payload must be a JSON object.")
        return value

    @staticmethod
    def _organization_id(organization_id: str | None) -> str:
        value = (organization_id or "").strip()
        if not value:
            raise ValueError("organization_id is required for PostgreSQL project access.")
        return value

    def _load_project_cursor(
        self,
        cursor: Any,
        project_row: tuple[Any, ...],
    ) -> ProjectRecord:
        project = ProjectRecord(
            id=str(project_row[0]),
            organization_id=str(project_row[1]),
            name=str(project_row[2]),
            created_at=self._iso_timestamp(project_row[3]),
            updated_at=self._iso_timestamp(project_row[4]),
            active_revision_id=None if project_row[5] is None else str(project_row[5]),
        )
        cursor.execute(
            """
            SELECT payload_json
            FROM project_revisions
            WHERE project_id = %s
            ORDER BY created_at ASC, id ASC
            """,
            (project.id,),
        )
        project.revisions = [
            ProjectRevision.from_dict(self._json_object(row[0]))
            for row in cursor.fetchall()
        ]
        return project

    def _fetch_project_cursor(
        self,
        cursor: Any,
        project_id: str,
        organization_id: str,
    ) -> ProjectRecord | None:
        cursor.execute(
            """
            SELECT id, organization_id, name, created_at, updated_at, active_revision_id
            FROM projects
            WHERE id = %s AND organization_id = %s
            """,
            (project_id, organization_id),
        )
        row = cursor.fetchone()
        return None if row is None else self._load_project_cursor(cursor, row)

    def ensure_seed_project(self, organization_id: str | None = None) -> ProjectRecord:
        projects = self.list_projects(organization_id)
        if projects:
            return projects[0]
        return self.create_project(
            "Reference Demo Project",
            organization_id=self._organization_id(organization_id),
            combination_config=_reference_combination_seed(),
            root_turn_radius_mm=9000.0,
            beta_min_deg=-45.0,
            beta_max_deg=45.0,
        )

    def list_projects(self, organization_id: str | None = None) -> list[ProjectRecord]:
        organization = self._organization_id(organization_id)

        def read(connection: Any) -> list[ProjectRecord]:
            cursor = connection.cursor()
            cursor.execute(
                """
                SELECT id, organization_id, name, created_at, updated_at, active_revision_id
                FROM projects
                WHERE organization_id = %s
                ORDER BY updated_at DESC, id ASC
                """,
                (organization,),
            )
            return [self._load_project_cursor(cursor, row) for row in cursor.fetchall()]

        return self._transaction(read)

    def get_project(
        self,
        project_id: str,
        organization_id: str | None = None,
    ) -> ProjectRecord | None:
        organization = self._organization_id(organization_id)

        def read(connection: Any) -> ProjectRecord | None:
            return self._fetch_project_cursor(connection.cursor(), project_id, organization)

        return self._transaction(read)

    def create_project(
        self,
        name: str,
        *,
        organization_id: str | None = None,
        beta_deg: float = 0.0,
        optimization_mode: str = "quick",
        note: str = "Initial revision",
        enabled_ids: Iterable[str] | None = None,
        accepted_optimization: bool = False,
        beta_min_deg: float = -45.0,
        beta_max_deg: float = 45.0,
        design_cases: Iterable[DesignCase] | None = None,
        linkage_config: dict[str, Any] | None = None,
        wheelbase_mm: float = 4360.0,
        track_mm: float = 2500.0,
        linkage_rig: LinkageDemoRig | None = None,
        vehicle: VehicleLayout | None = None,
        vehicle_config: dict[str, Any] | None = None,
        combination_config: dict[str, Any] | None = None,
        root_turn_radius_mm: float | None = None,
        mechanism_graph_config: dict[str, Any] | None = None,
        mechanism_drivers: Iterable[dict[str, Any]] | None = None,
        steering_assignments: Iterable[dict[str, Any]] | None = None,
        engineering_snapshot: dict[str, Any] | None = None,
    ) -> ProjectRecord:
        organization = self._organization_id(organization_id)
        created_at = _utc_now_iso()
        project = ProjectRecord(
            id=_new_id("proj"),
            name=name.strip() or "Untitled Project",
            created_at=created_at,
            updated_at=created_at,
        )
        revision = self._create_revision(
            beta_deg=beta_deg,
            optimization_mode=optimization_mode,
            note=note,
            enabled_ids=enabled_ids,
            accepted_optimization=accepted_optimization,
            beta_min_deg=beta_min_deg,
            beta_max_deg=beta_max_deg,
            design_cases=design_cases,
            linkage_config=linkage_config,
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            linkage_rig=linkage_rig,
            vehicle=vehicle,
            vehicle_config=vehicle_config,
            combination_config=combination_config,
            root_turn_radius_mm=root_turn_radius_mm,
            mechanism_graph_config=mechanism_graph_config,
            mechanism_drivers=mechanism_drivers,
            steering_assignments=steering_assignments,
            engineering_snapshot=engineering_snapshot,
        )
        project.revisions.append(revision)
        project.active_revision_id = revision.id

        def insert(connection: Any) -> None:
            cursor = connection.cursor()
            cursor.execute(
                """
                INSERT INTO projects (id, organization_id, name, created_at, updated_at, active_revision_id)
                VALUES (%s, %s, %s, %s, %s, %s)
                """,
                (
                    project.id,
                    organization,
                    project.name,
                    project.created_at,
                    project.updated_at,
                    project.active_revision_id,
                ),
            )
            cursor.execute(
                """
                INSERT INTO project_revisions
                    (id, organization_id, project_id, created_at, payload_json)
                VALUES (%s, %s, %s, %s, %s::jsonb)
                """,
                (
                    revision.id,
                    organization,
                    project.id,
                    revision.created_at,
                    json.dumps(revision.to_dict()),
                ),
            )

        self._transaction(insert)
        return project

    def append_revision(
        self,
        project_id: str,
        *,
        organization_id: str | None = None,
        beta_deg: float,
        optimization_mode: str,
        note: str,
        enabled_ids: Iterable[str] | None = None,
        accepted_optimization: bool = False,
        beta_min_deg: float = -45.0,
        beta_max_deg: float = 45.0,
        design_cases: Iterable[DesignCase] | None = None,
        linkage_config: dict[str, Any] | None = None,
        wheelbase_mm: float = 4360.0,
        track_mm: float = 2500.0,
        linkage_rig: LinkageDemoRig | None = None,
        vehicle: VehicleLayout | None = None,
        vehicle_config: dict[str, Any] | None = None,
        combination_config: dict[str, Any] | None = None,
        root_turn_radius_mm: float | None = None,
        mechanism_graph_config: dict[str, Any] | None = None,
        mechanism_drivers: Iterable[dict[str, Any]] | None = None,
        steering_assignments: Iterable[dict[str, Any]] | None = None,
        engineering_snapshot: dict[str, Any] | None = None,
    ) -> ProjectRevision:
        organization = self._organization_id(organization_id)
        revision = self._create_revision(
            beta_deg=beta_deg,
            optimization_mode=optimization_mode,
            note=note,
            enabled_ids=enabled_ids,
            accepted_optimization=accepted_optimization,
            beta_min_deg=beta_min_deg,
            beta_max_deg=beta_max_deg,
            design_cases=design_cases,
            linkage_config=linkage_config,
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            linkage_rig=linkage_rig,
            vehicle=vehicle,
            vehicle_config=vehicle_config,
            combination_config=combination_config,
            root_turn_radius_mm=root_turn_radius_mm,
            mechanism_graph_config=mechanism_graph_config,
            mechanism_drivers=mechanism_drivers,
            steering_assignments=steering_assignments,
            engineering_snapshot=engineering_snapshot,
        )

        def insert(connection: Any) -> None:
            cursor = connection.cursor()
            project = self._fetch_project_cursor(cursor, project_id, organization)
            if project is None:
                raise KeyError(project_id)
            cursor.execute(
                """
                INSERT INTO project_revisions
                    (id, organization_id, project_id, created_at, payload_json)
                VALUES (%s, %s, %s, %s, %s::jsonb)
                """,
                (
                    revision.id,
                    organization,
                    project_id,
                    revision.created_at,
                    json.dumps(revision.to_dict()),
                ),
            )
            cursor.execute(
                """
                UPDATE projects
                SET active_revision_id = %s, updated_at = %s
                WHERE id = %s AND organization_id = %s
                """,
                (revision.id, revision.created_at, project_id, organization),
            )

        self._transaction(insert)
        return revision

    def restore_revision(
        self,
        project_id: str,
        revision_id: str,
        organization_id: str | None = None,
    ) -> ProjectRevision:
        organization = self._organization_id(organization_id)

        def restore(connection: Any) -> ProjectRevision:
            cursor = connection.cursor()
            project = self._fetch_project_cursor(cursor, project_id, organization)
            if project is None:
                raise KeyError(project_id)
            revision = project.get_revision(revision_id)
            if revision is None:
                raise KeyError(revision_id)
            cursor.execute(
                """
                UPDATE projects
                SET active_revision_id = %s, updated_at = %s
                WHERE id = %s AND organization_id = %s
                """,
                (revision_id, _utc_now_iso(), project_id, organization),
            )
            return revision

        return self._transaction(restore)

    def get_active_revision(
        self,
        project_id: str,
        organization_id: str | None = None,
    ) -> ProjectRevision | None:
        project = self.get_project(project_id, organization_id)
        if project is None or project.active_revision_id is None:
            return None
        return project.get_revision(project.active_revision_id)

    def get_revision(
        self,
        project_id: str,
        revision_id: str,
        organization_id: str | None = None,
    ) -> ProjectRevision | None:
        project = self.get_project(project_id, organization_id)
        return None if project is None else project.get_revision(revision_id)

    def record_acceptance(
        self,
        project_id: str,
        revision_id: str,
        acceptance: dict[str, Any],
        organization_id: str | None = None,
    ) -> ProjectRevision:
        organization = self._organization_id(organization_id)

        def record(connection: Any) -> ProjectRevision:
            cursor = connection.cursor()
            project = self._fetch_project_cursor(cursor, project_id, organization)
            if project is None:
                raise KeyError(project_id)
            revision = project.get_revision(revision_id)
            if revision is None:
                raise KeyError(revision_id)
            revision.snapshot = {
                **revision.snapshot,
                "monroc_acceptance": json.loads(json.dumps(acceptance)),
            }
            cursor.execute(
                """
                UPDATE project_revisions
                SET payload_json = %s::jsonb
                WHERE project_id = %s AND id = %s AND organization_id = %s
                """,
                (json.dumps(revision.to_dict()), project_id, revision_id, organization),
            )
            cursor.execute(
                """
                UPDATE projects
                SET updated_at = %s
                WHERE id = %s AND organization_id = %s
                """,
                (_utc_now_iso(), project_id, organization),
            )
            return revision

        return self._transaction(record)


def serialize_project(project: ProjectRecord, *, include_snapshots: bool = False) -> dict[str, Any]:
    data = project.summary()
    if include_snapshots:
        data["revisions"] = [revision.to_dict() for revision in project.revisions]
    else:
        data["revisions"] = [revision.summary() for revision in project.revisions]
    return data


def serialize_revision(revision: ProjectRevision, *, include_snapshot: bool = False) -> dict[str, Any]:
    if include_snapshot:
        return revision.to_dict()
    return revision.summary()
