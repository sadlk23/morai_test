"""Competition runtime orchestration."""

__all__ = ["CompetitionRuntimePipeline"]


def __getattr__(name: str):
    """Lazy re-export to avoid import cycles with live integration adapters."""

    if name == "CompetitionRuntimePipeline":
        from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline

        return CompetitionRuntimePipeline
    raise AttributeError(name)
