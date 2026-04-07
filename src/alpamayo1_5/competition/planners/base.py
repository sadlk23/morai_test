"""Planner backend interfaces."""

from __future__ import annotations

from abc import ABC, abstractmethod

from alpamayo1_5.competition.contracts import PlanResult, PlannerInput


class PlannerBackend(ABC):
    """Common backend contract for competition planners."""

    name: str

    @abstractmethod
    def plan(self, planner_input: PlannerInput) -> PlanResult:
        """Produce an interpretable plan from fused runtime inputs."""
