from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class EngineeringTolerances:
    """Explicit numerical tolerances used at engineering API boundaries."""

    geometric_mm: float = 0.01
    angle_rad: float = 1e-5
    solver_residual_mm: float = 0.01
    branch_continuity_mm: float = 250.0


DEFAULT_TOLERANCES = EngineeringTolerances()


__all__ = ["EngineeringTolerances", "DEFAULT_TOLERANCES"]
