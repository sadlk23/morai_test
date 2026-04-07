"""UDP command publisher for competition runtime."""

from __future__ import annotations

import json
import socket

from alpamayo1_5.competition.contracts import SafetyDecision
from alpamayo1_5.competition.runtime.config_competition import UdpOutputConfig


class UdpCommandPublisher:
    """Publish safety-filtered commands over UDP."""

    def __init__(self, config: UdpOutputConfig):
        self.config = config
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, decision: SafetyDecision) -> None:
        payload = {
            "frame_id": decision.frame_id,
            "timestamp_s": decision.timestamp_s,
            "steering": decision.command.steering,
            "throttle": decision.command.throttle,
            "brake": decision.command.brake,
            "target_speed_mps": decision.command.target_speed_mps,
            "intervention": decision.intervention,
            "risk_level": decision.risk_level,
            "flags": decision.safety_flags,
        }
        self._socket.sendto(
            json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            (self.config.host, self.config.port),
        )

    def close(self) -> None:
        self._socket.close()
