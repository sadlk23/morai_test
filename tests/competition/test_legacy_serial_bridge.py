"""Tests for moo-compatible legacy serial bridge payload conversion."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.contracts import ControlCommand, SafetyDecision
from alpamayo1_5.competition.integrations.morai.legacy_serial_bridge import (
    build_legacy_serial_payload,
    legacy_serial_bridge_diagnostics,
    resolve_legacy_brake_scaling,
)
from alpamayo1_5.competition.runtime.config_competition import LegacySerialBridgeConfig


class LegacySerialBridgeTest(unittest.TestCase):
    def _decision(self, brake: float = 0.2, intervention: str = "none", fallback: bool = False) -> SafetyDecision:
        return SafetyDecision(
            frame_id=7,
            timestamp_s=1.5,
            command=ControlCommand(
                frame_id=7,
                timestamp_s=1.5,
                steering=0.3,
                throttle=0.4,
                brake=brake,
                target_speed_mps=6.2,
            ),
            intervention=intervention,
            fallback_used=fallback,
        )

    def test_payload_shape_and_units(self) -> None:
        payload = build_legacy_serial_payload(
            self._decision(),
            LegacySerialBridgeConfig(),
            alive_counter=9,
        )
        self.assertEqual(len(payload), 7)
        self.assertEqual(payload[0], 1.0)
        self.assertEqual(payload[2], 0.0)
        self.assertAlmostEqual(payload[3], 6.2)
        self.assertAlmostEqual(payload[4], 0.3)
        self.assertAlmostEqual(payload[5], 0.2)
        self.assertEqual(payload[6], 9.0)

    def test_brake_can_be_scaled_to_erp_style_range(self) -> None:
        config = LegacySerialBridgeConfig(brake_mode="erp_200", brake_output_max=200.0)
        payload = build_legacy_serial_payload(self._decision(brake=0.75), config, alive_counter=1)
        self.assertAlmostEqual(payload[5], 150.0)

    def test_e_stop_sets_when_fallback_or_stop_intervention(self) -> None:
        payload_fallback = build_legacy_serial_payload(
            self._decision(intervention="none", fallback=True),
            LegacySerialBridgeConfig(),
            alive_counter=0,
        )
        payload_stop = build_legacy_serial_payload(
            self._decision(intervention="live_input_wait_stop", fallback=False),
            LegacySerialBridgeConfig(),
            alive_counter=0,
        )
        self.assertEqual(payload_fallback[1], 1.0)
        self.assertEqual(payload_stop[1], 1.0)

    def test_alive_counter_can_be_disabled(self) -> None:
        config = LegacySerialBridgeConfig(include_alive_counter=False)
        payload = build_legacy_serial_payload(self._decision(), config, alive_counter=123)
        self.assertEqual(payload[6], 0.0)

    def test_auto_mode_keeps_backward_compatible_erp_inference(self) -> None:
        mode, brake_output_max, warnings = resolve_legacy_brake_scaling(
            LegacySerialBridgeConfig(brake_mode="auto", brake_output_max=200.0)
        )
        self.assertEqual(mode, "erp_200")
        self.assertEqual(brake_output_max, 200.0)
        self.assertEqual(warnings, [])

    def test_explicit_brake_mode_wins_over_conflicting_output_max(self) -> None:
        mode, brake_output_max, warnings = resolve_legacy_brake_scaling(
            LegacySerialBridgeConfig(brake_mode="erp_200", brake_output_max=1.0)
        )
        self.assertEqual(mode, "erp_200")
        self.assertEqual(brake_output_max, 200.0)
        self.assertTrue(warnings)
        diagnostics = legacy_serial_bridge_diagnostics(
            LegacySerialBridgeConfig(brake_mode="normalized", brake_output_max=200.0)
        )
        self.assertEqual(diagnostics["brake_mode"], "normalized")
        self.assertEqual(diagnostics["brake_output_max"], 1.0)
        self.assertTrue(diagnostics["warnings"])


if __name__ == "__main__":
    unittest.main()
