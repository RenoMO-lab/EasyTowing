from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import itertools
import math

from .errors import SweepSampleLimitError


@dataclass(frozen=True, slots=True)
class JointSweepRange:
    """Signed articulation range used by a deterministic multi-joint sweep."""

    joint_id: str
    minimum_deg: float
    maximum_deg: float
    step_deg: float

    def __post_init__(self) -> None:
        if not self.joint_id.strip():
            raise ValueError("Joint sweep IDs must not be empty.")
        values = (self.minimum_deg, self.maximum_deg, self.step_deg)
        if any(not math.isfinite(value) for value in values):
            raise ValueError(f"Joint sweep {self.joint_id!r} must contain finite bounds and step.")
        if self.minimum_deg >= self.maximum_deg:
            raise ValueError(f"Joint sweep {self.joint_id!r} minimum must be below maximum.")
        if self.minimum_deg > 0.0 or self.maximum_deg < 0.0:
            raise ValueError(f"Joint sweep {self.joint_id!r} bounds must straddle zero.")
        if self.step_deg <= 0.0:
            raise ValueError(f"Joint sweep {self.joint_id!r} step must be positive.")

    def values(self) -> tuple[float, ...]:
        values: list[float] = []
        current = self.minimum_deg
        while current <= self.maximum_deg + 1e-9:
            values.append(min(current, self.maximum_deg))
            current += self.step_deg
        if not values or values[-1] < self.maximum_deg - 1e-9:
            values.append(self.maximum_deg)
        if not any(abs(value) <= 1e-9 for value in values):
            values.append(0.0)
        return tuple(sorted(set(round(value, 12) for value in values)))

    def to_dict(self) -> dict[str, float | str]:
        return {
            "joint_id": self.joint_id,
            "min_deg": self.minimum_deg,
            "max_deg": self.maximum_deg,
            "step_deg": self.step_deg,
        }


def normalize_joint_sweep_ranges(
    joint_ids: Iterable[str],
    raw_ranges: object | None,
    *,
    default_min_deg: float,
    default_max_deg: float,
    default_step_deg: float,
    primary_joint_id: str | None = None,
) -> tuple[JointSweepRange, ...]:
    """Normalize API/UI range data into one range for every articulation joint.

    A multi-body approval sweep must not silently hold an omitted joint at its
    nominal pose. Explicit ranges override the defaults; omitted joints use the
    supplied defaults so the resulting grid is always auditable.
    """

    ordered_ids = tuple(str(joint_id) for joint_id in joint_ids)
    if not ordered_ids:
        raise ValueError("A combination sweep requires at least one articulation joint.")
    known_ids = set(ordered_ids)
    primary_id = primary_joint_id or ordered_ids[0]
    if primary_id not in known_ids:
        raise ValueError(f"Unknown primary articulation joint {primary_id!r}.")

    entries: list[tuple[str, object]] = []
    if raw_ranges is None:
        entries = []
    elif isinstance(raw_ranges, Mapping):
        entries = [(str(joint_id), raw_range) for joint_id, raw_range in raw_ranges.items()]
    elif isinstance(raw_ranges, list):
        for index, raw_range in enumerate(raw_ranges):
            if not isinstance(raw_range, Mapping):
                raise ValueError(f"joint_ranges[{index}] must be an object.")
            joint_id = raw_range.get("joint_id", raw_range.get("id"))
            if joint_id in (None, ""):
                raise ValueError(f"joint_ranges[{index}] requires joint_id.")
            entries.append((str(joint_id), raw_range))
    else:
        raise ValueError("joint_ranges must be an object keyed by joint ID or an array.")

    configured: dict[str, JointSweepRange] = {}
    seen: set[str] = set()
    for joint_id, raw_range in entries:
        if joint_id not in known_ids:
            raise ValueError(f"joint_ranges references unknown joint {joint_id!r}.")
        if joint_id in seen:
            raise ValueError(f"joint_ranges contains duplicate joint {joint_id!r}.")
        if not isinstance(raw_range, Mapping):
            raise ValueError(f"joint_ranges[{joint_id!r}] must be an object.")
        try:
            minimum = float(raw_range.get("min_deg", raw_range.get("minimum_deg", default_min_deg)))
            maximum = float(raw_range.get("max_deg", raw_range.get("maximum_deg", default_max_deg)))
            step = float(raw_range.get("step_deg", default_step_deg))
        except (TypeError, ValueError) as error:
            raise ValueError(f"joint_ranges[{joint_id!r}] contains non-numeric values.") from error
        configured[joint_id] = JointSweepRange(joint_id, minimum, maximum, step)
        seen.add(joint_id)

    return tuple(
        configured.get(
            joint_id,
            JointSweepRange(
                joint_id,
                default_min_deg,
                default_max_deg,
                default_step_deg,
            ),
        )
        for joint_id in ordered_ids
    )


def build_joint_sweep_grid(
    ranges: Iterable[JointSweepRange],
    *,
    maximum_samples: int = 10_000,
) -> tuple[dict[str, float], ...]:
    """Build a bounded Cartesian product and never silently truncate it."""

    normalized = tuple(ranges)
    if not normalized:
        raise ValueError("At least one joint sweep range is required.")
    if maximum_samples <= 0:
        raise ValueError("maximum_samples must be positive.")
    axes = tuple(item.values() for item in normalized)
    requested_count = math.prod(len(axis) for axis in axes)
    if requested_count > maximum_samples:
        raise SweepSampleLimitError(requested_count, maximum_samples)
    # Visit the full Cartesian grid as a serpentine path. This keeps adjacent
    # samples physically adjacent, allowing mechanism branch continuity to be
    # checked without a lexicographic carry jumping several joints at once.
    axis_indices = tuple(range(len(axis)) for axis in axes)
    samples: list[dict[str, float]] = []
    for index_point in itertools.product(*axis_indices):
        values = tuple(
            axes[axis_index][
                item_index
                if sum(index_point[:axis_index]) % 2 == 0
                else len(axes[axis_index]) - 1 - item_index
            ]
            for axis_index, item_index in enumerate(index_point)
        )
        samples.append(
            {
                item.joint_id: float(value)
                for item, value in zip(normalized, values, strict=True)
            }
        )
    return tuple(samples)


__all__ = [
    "JointSweepRange",
    "build_joint_sweep_grid",
    "normalize_joint_sweep_ranges",
]
