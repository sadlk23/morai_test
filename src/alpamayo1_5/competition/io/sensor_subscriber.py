"""Generic sensor-source utilities for live and replay modes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, Iterator

from alpamayo1_5.competition.contracts import SensorPacket


@dataclass(slots=True)
class SensorBuffer:
    """Small in-memory packet buffer for integration tests and replay."""

    maxlen: int = 128

    def __post_init__(self) -> None:
        self._packets: deque[SensorPacket] = deque(maxlen=self.maxlen)

    def push(self, packet: SensorPacket) -> None:
        self._packets.append(packet)

    def latest(self) -> SensorPacket | None:
        return self._packets[-1] if self._packets else None

    def pop_left(self) -> SensorPacket | None:
        return self._packets.popleft() if self._packets else None

    def __len__(self) -> int:
        return len(self._packets)


class ReplaySensorSource(Iterator[SensorPacket]):
    """Iterator-backed replay source for dry-runs and tests."""

    def __init__(self, packets: Iterable[SensorPacket]):
        self._packets = iter(packets)

    def __iter__(self) -> "ReplaySensorSource":
        return self

    def __next__(self) -> SensorPacket:
        return next(self._packets)
