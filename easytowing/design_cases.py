from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal


DesignCaseDirection = Literal["left", "right"]


@dataclass(frozen=True, slots=True)
class DesignCase:
    """A weighted engineering condition used by simulation and optimization."""

    id: str
    name: str
    beta_deg: float | None = None
    turn_radius_mm: float | None = None
    outer_diameter_mm: float | None = None
    direction: DesignCaseDirection = "left"
    weight: float = 1.0
    enabled: bool = True

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.name.strip():
            raise ValueError("Design cases require non-empty IDs and names")
        targets = sum(
            value is not None
            for value in (self.beta_deg, self.turn_radius_mm, self.outer_diameter_mm)
        )
        if targets != 1:
            raise ValueError("A design case must define exactly one target")
        if self.beta_deg is not None and not math.isfinite(self.beta_deg):
            raise ValueError("beta_deg must be finite")
        if self.turn_radius_mm is not None and (
            not math.isfinite(self.turn_radius_mm) or abs(self.turn_radius_mm) <= 1e-9
        ):
            raise ValueError("turn_radius_mm must be a non-zero finite value")
        if self.outer_diameter_mm is not None and (
            not math.isfinite(self.outer_diameter_mm) or self.outer_diameter_mm <= 0.0
        ):
            raise ValueError("outer_diameter_mm must be positive and finite")
        if self.direction not in {"left", "right"}:
            raise ValueError("direction must be left or right")
        if not math.isfinite(self.weight) or self.weight < 0.0:
            raise ValueError("weight must be a non-negative finite value")

    def resolved_beta_deg(self, reference_length_mm: float) -> float:
        if not math.isfinite(reference_length_mm) or reference_length_mm <= 0.0:
            raise ValueError("reference_length_mm must be positive and finite")
        if self.beta_deg is not None:
            return self.beta_deg

        if self.turn_radius_mm is not None:
            radius = self.turn_radius_mm
            sign = 1.0 if radius > 0.0 else -1.0
            return math.degrees(math.atan(reference_length_mm / abs(radius))) * sign

        assert self.outer_diameter_mm is not None
        sign = 1.0 if self.direction == "left" else -1.0
        return math.degrees(math.atan(reference_length_mm / (self.outer_diameter_mm / 2.0))) * sign

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "name": self.name,
            "beta_deg": self.beta_deg,
            "turn_radius_mm": self.turn_radius_mm,
            "outer_diameter_mm": self.outer_diameter_mm,
            "direction": self.direction,
            "weight": self.weight,
            "enabled": self.enabled,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "DesignCase":
        def optional_float(key: str) -> float | None:
            value = data.get(key)
            return None if value is None else float(value)

        return cls(
            id=str(data.get("id", "")),
            name=str(data.get("name", "")),
            beta_deg=optional_float("beta_deg"),
            turn_radius_mm=optional_float("turn_radius_mm"),
            outer_diameter_mm=optional_float("outer_diameter_mm"),
            direction=str(data.get("direction", "left")),  # type: ignore[arg-type]
            weight=float(data.get("weight", 1.0)),
            enabled=bool(data.get("enabled", True)),
        )


__all__ = ["DesignCase", "DesignCaseDirection"]
