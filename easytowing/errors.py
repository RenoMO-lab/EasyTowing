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

    def __init__(self, angle_deg: float, limit_deg: float) -> None:
        super().__init__(
            f"DRAWBAR_LIMIT_EXCEEDED: {angle_deg:.6f} deg exceeds "
            f"the +/-{limit_deg:.6f} deg articulation limit."
        )
        self.angle_deg = angle_deg
        self.limit_deg = limit_deg


class ClearanceViolationError(EngineeringError):
    """Raised when a design does not meet a required minimum clearance."""

    code = "MIN_CLEARANCE_VIOLATED"


class OptimizationNoFeasibleSolutionError(EngineeringError):
    """Raised when no candidate remains solvable across the requested sweep."""

    code = "OPTIMIZATION_NO_FEASIBLE_SOLUTION"
