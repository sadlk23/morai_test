"""Planner runtime orchestration and backend fallback policy."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import PlanResult, PlannerInput
from alpamayo1_5.competition.planners.legacy_backend import LegacyAlpamayoPlannerBackend
from alpamayo1_5.competition.planners.lightweight_backend import LightweightWaypointPlannerBackend
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


class CompetitionPlanner:
    """Owns planner backend selection and fallback behavior."""

    def __init__(self, config: CompetitionConfig):
        self.config = config
        self.lightweight = LightweightWaypointPlannerBackend(config.planner)
        self.legacy = LegacyAlpamayoPlannerBackend(config.planner, config.cameras)

    def _primary_backend(self):
        if self.config.planner.backend == "legacy_alpamayo":
            return self.legacy
        return self.lightweight

    def plan(self, planner_input: PlannerInput) -> PlanResult:
        primary = self._primary_backend()
        plan = primary.plan(planner_input)
        if plan.valid or not self.config.planner.allow_backend_fallback:
            return plan

        if primary is self.lightweight:
            return plan

        fallback = self.lightweight.plan(planner_input)
        fallback.used_fallback = True
        fallback.diagnostics["fallback_reason"] = plan.diagnostics.get("error", "invalid_primary_plan")
        return fallback
