class EngineeringError(Exception):
    """Base class for engineering-specific errors."""

    code = "ENGINEERING_ERROR"


class InvalidGeometryError(EngineeringError):
    """Raised when a geometric configuration cannot be solved."""

    code = "INVALID_GEOMETRY"


class SolverBranchError(EngineeringError):
    """Raised when a solver branch becomes inconsistent or invalid."""

    code = "LINKAGE_BRANCH_CHANGE"


class LinkageNoSolutionError(InvalidGeometryError):
    """Raised when a rigid-link stage has no physical circle intersection."""

    code = "LINKAGE_NO_SOLUTION"

    def __init__(self, stage: str, message: str) -> None:
        super().__init__(f"{stage}: {message}")
        self.stage = stage


class LinkageBranchChangeError(SolverBranchError):
    """Raised when branch continuity is lost for a linkage stage."""

    code = "LINKAGE_BRANCH_CHANGE"

    def __init__(self, stage: str, message: str) -> None:
        super().__init__(f"{stage}: {message}")
        self.stage = stage


class SteeringLimitExceededError(EngineeringError):
    """Raised when a solved mechanism exceeds its explicit steering stop."""

    code = "STEERING_LIMIT_EXCEEDED"

    def __init__(self, angle_deg: float, limit_deg: float) -> None:
        super().__init__(
            f"STEERING_LIMIT_EXCEEDED: {angle_deg:.6f} deg exceeds "
            f"the +/-{limit_deg:.6f} deg steering stop."
        )
        self.angle_deg = angle_deg
        self.limit_deg = limit_deg


class ArticulationLimitExceededError(EngineeringError):
    """Raised when the requested drawbar articulation exceeds the vehicle limit."""

    code = "DRAWBAR_LIMIT_EXCEEDED"

    def __init__(self, angle_deg: float, limit_deg: float, *, joint_id: str | None = None) -> None:
        subject = "Articulation" if joint_id is None else f"Joint {joint_id!r} articulation"
        super().__init__(
            f"DRAWBAR_LIMIT_EXCEEDED: {subject} {angle_deg:.6f} deg exceeds "
            f"the +/-{limit_deg:.6f} deg articulation limit."
        )
        self.angle_deg = angle_deg
        self.limit_deg = limit_deg
        self.joint_id = joint_id


class MultiBodyKinematicConstraintError(EngineeringError):
    """Raised when one articulated body cannot share the maneuver ICR."""

    code = "MULTIBODY_KINEMATIC_INCONSISTENT"

    def __init__(self, component_id: str, residual_mm: float, tolerance_mm: float) -> None:
        super().__init__(
            f"MULTIBODY_KINEMATIC_INCONSISTENT: {component_id!r} has a "
            f"{residual_mm:.6f} mm rolling-constraint residual; tolerance is "
            f"{tolerance_mm:.6f} mm."
        )
        self.component_id = component_id
        self.residual_mm = residual_mm
        self.tolerance_mm = tolerance_mm


class ClearanceViolationError(EngineeringError):
    """Raised when a design does not meet a required minimum clearance."""

    code = "MIN_CLEARANCE_VIOLATED"


class OptimizationNoFeasibleSolutionError(EngineeringError):
    """Raised when no candidate remains solvable across the requested sweep."""

    code = "OPTIMIZATION_NO_FEASIBLE_SOLUTION"

    def __init__(
        self,
        violations: tuple[str, ...] = (),
        *,
        minimum_clearance_mm: float | None = None,
        clearance_target_mm: float | None = None,
    ) -> None:
        detail = ", ".join(violations) or "unknown constraint violation"
        clearance_detail = ""
        if minimum_clearance_mm is not None and clearance_target_mm is not None:
            clearance_detail = (
                f" Best minimum clearance was {minimum_clearance_mm:.3f} mm; "
                f"required clearance is {clearance_target_mm:.3f} mm."
            )
        super().__init__(
            "OPTIMIZATION_NO_FEASIBLE_SOLUTION: no candidate satisfied all hard "
            f"constraints ({detail}).{clearance_detail}"
        )
        self.violations = violations
        self.minimum_clearance_mm = minimum_clearance_mm
        self.clearance_target_mm = clearance_target_mm


class SweepSampleLimitError(EngineeringError):
    """Raised when a requested multi-joint sweep exceeds its configured budget."""

    code = "SWEEP_SAMPLE_LIMIT"

    def __init__(self, requested_count: int, maximum_count: int) -> None:
        super().__init__(
            "SWEEP_SAMPLE_LIMIT: the requested multi-joint sweep contains "
            f"{requested_count} poses, exceeding the configured maximum of {maximum_count}."
        )
        self.requested_count = requested_count
        self.maximum_count = maximum_count
