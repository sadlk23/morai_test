"""Per-stage latency monitoring."""

from __future__ import annotations

from contextlib import contextmanager
from time import perf_counter


class LatencyMonitor:
    """Collects stage latency measurements in milliseconds."""

    def __init__(self) -> None:
        self._latencies_ms: dict[str, float] = {}

    @contextmanager
    def measure(self, stage_name: str):
        start = perf_counter()
        try:
            yield
        finally:
            self._latencies_ms[stage_name] = (perf_counter() - start) * 1_000.0

    def snapshot(self) -> dict[str, float]:
        """Return a copy of the current stage latency map."""

        return dict(self._latencies_ms)

    def reset(self) -> None:
        """Clear collected latency measurements."""

        self._latencies_ms.clear()
