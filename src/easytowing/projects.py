from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from threading import Lock
from uuid import uuid4
from typing import Any

from .reporting import build_export_bundle


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _new_id(prefix: str) -> str:
    return f"{prefix}_{uuid4().hex[:10]}"


@dataclass(slots=True)
class ProjectRevision:
    id: str
    created_at: str
    note: str
    beta_deg: float
    optimization_mode: str
    snapshot: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "created_at": self.created_at,
            "note": self.note,
            "beta_deg": self.beta_deg,
            "optimization_mode": self.optimization_mode,
            "snapshot": self.snapshot,
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
        )

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "created_at": self.created_at,
            "note": self.note,
            "beta_deg": self.beta_deg,
            "optimization_mode": self.optimization_mode,
        }


@dataclass(slots=True)
class ProjectRecord:
    id: str
    name: str
    created_at: str
    updated_at: str
    active_revision_id: str | None = None
    revisions: list[ProjectRevision] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
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
        self._path = path
        self._lock = Lock()
        self._projects: dict[str, ProjectRecord] = {}
        self._load()

    @classmethod
    def default(cls) -> "ProjectStore":
        root = Path(__file__).resolve().parents[2]
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

    def ensure_seed_project(self) -> ProjectRecord:
        with self._lock:
            if self._projects:
                return next(iter(self._projects.values()))
            project = self._create_project_locked("Reference Demo Project")
            return project

    def list_projects(self) -> list[ProjectRecord]:
        with self._lock:
            return list(self._projects.values())

    def get_project(self, project_id: str) -> ProjectRecord | None:
        with self._lock:
            return self._projects.get(project_id)

    def create_project(
        self,
        name: str,
        *,
        beta_deg: float = 0.0,
        optimization_mode: str = "quick",
        note: str = "Initial revision",
    ) -> ProjectRecord:
        with self._lock:
            return self._create_project_locked(
                name,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
            )

    def _create_project_locked(
        self,
        name: str,
        *,
        beta_deg: float = 0.0,
        optimization_mode: str = "quick",
        note: str = "Initial revision",
    ) -> ProjectRecord:
        created_at = _utc_now_iso()
        project = ProjectRecord(
            id=_new_id("proj"),
            name=name.strip() or "Untitled Project",
            created_at=created_at,
            updated_at=created_at,
        )
        revision = self._create_revision(beta_deg=beta_deg, optimization_mode=optimization_mode, note=note)
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
    ) -> ProjectRevision:
        return ProjectRevision(
            id=_new_id("rev"),
            created_at=_utc_now_iso(),
            note=note.strip() or "Revision",
            beta_deg=beta_deg,
            optimization_mode=optimization_mode,
            snapshot=build_export_bundle(beta_deg, optimization_mode),
        )

    def append_revision(
        self,
        project_id: str,
        *,
        beta_deg: float,
        optimization_mode: str,
        note: str,
    ) -> ProjectRevision:
        with self._lock:
            project = self._projects[project_id]
            revision = self._create_revision(beta_deg=beta_deg, optimization_mode=optimization_mode, note=note)
            project.revisions.append(revision)
            project.active_revision_id = revision.id
            project.updated_at = revision.created_at
            self._save()
            return revision

    def restore_revision(self, project_id: str, revision_id: str) -> ProjectRevision:
        with self._lock:
            project = self._projects[project_id]
            revision = project.get_revision(revision_id)
            if revision is None:
                raise KeyError(revision_id)
            project.active_revision_id = revision.id
            project.updated_at = _utc_now_iso()
            self._save()
            return revision

    def get_active_revision(self, project_id: str) -> ProjectRevision | None:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None or project.active_revision_id is None:
                return None
            return project.get_revision(project.active_revision_id)

    def get_revision(self, project_id: str, revision_id: str) -> ProjectRevision | None:
        with self._lock:
            project = self._projects.get(project_id)
            if project is None:
                return None
            return project.get_revision(revision_id)


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
