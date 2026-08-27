class EngineeringError(Exception):
    """Base class for engineering-specific errors."""


class InvalidGeometryError(EngineeringError):
    """Raised when a geometric configuration cannot be solved."""


class SolverBranchError(EngineeringError):
    """Raised when a solver branch becomes inconsistent or invalid."""


class LinkageNoSolutionError(InvalidGeometryError):
    """Raised when a rigid-link stage has no physical circle intersection."""

    def __init__(self, stage: str, message: str) -> None:
        super().__init__(f"{stage}: {message}")
        self.stage = stage


class LinkageBranchChangeError(SolverBranchError):
    """Raised when branch continuity is lost for a linkage stage."""

    def __init__(self, stage: str, message: str) -> None:
        super().__init__(f"{stage}: {message}")
        self.stage = stage
