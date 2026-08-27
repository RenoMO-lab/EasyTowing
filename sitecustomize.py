from __future__ import annotations

from pathlib import Path
from uuid import uuid4
import shutil
import tempfile
import sys


class _PathList(list):
    def insert(self, index, object):  # noqa: A003 - match list API
        try:
            candidate = Path(object)
        except TypeError:
            return super().insert(index, object)

        repo_root = Path(__file__).resolve().parent
        if candidate == repo_root / "src":
            if str(repo_root) not in self:
                super().insert(0, str(repo_root))
            return

        return super().insert(index, object)


if not isinstance(sys.path, _PathList):
    sys.path = _PathList(sys.path)
    repo_root = str(Path(__file__).resolve().parent)
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)


class _WorkspaceTemporaryDirectory:
    def __init__(self, suffix: str | None = None, prefix: str | None = None, dir: str | None = None, ignore_cleanup_errors: bool = False) -> None:
        self._ignore_cleanup_errors = ignore_cleanup_errors
        base_dir = Path(dir) if dir is not None else Path(__file__).resolve().parent / ".codex-temp"
        base_dir.mkdir(parents=True, exist_ok=True)
        name = f"{prefix or 'tmp'}{uuid4().hex[:8]}{suffix or ''}"
        path = base_dir / name
        path.mkdir(parents=False, exist_ok=False)
        self.name = str(path)

    def __enter__(self):
        return self.name

    def __exit__(self, exc_type, exc, tb):
        self.cleanup()
        return False

    def cleanup(self) -> None:
        path = Path(self.name)
        if not path.exists():
            return
        try:
            shutil.rmtree(path)
        except Exception:
            if not self._ignore_cleanup_errors:
                raise

    @classmethod
    def _cleanup(cls, name, warn_message=None, ignore_errors=False, delete=True):  # noqa: D401
        path = Path(name)
        if not delete or not path.exists():
            return
        try:
            shutil.rmtree(path)
        except Exception:
            if not ignore_errors:
                raise


tempfile.TemporaryDirectory = _WorkspaceTemporaryDirectory
