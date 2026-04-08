"""Tests for MORAI direct actuation publisher self-check behavior."""

from __future__ import annotations

import unittest
from unittest.mock import patch

from alpamayo1_5.competition.integrations.morai.publishers import (
    MoraiActuationContractError,
    MoraiCtrlCmdPublisher,
)
from alpamayo1_5.competition.runtime.config_competition import RosOutputConfig


class _FakePublisher:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs


class _FakeRospy:
    class core:
        @staticmethod
        def is_initialized() -> bool:
            return True

    @staticmethod
    def Publisher(*args, **kwargs):
        return _FakePublisher(*args, **kwargs)


class _CtrlCmdValid:
    def __init__(self) -> None:
        self.longlCmdType = 0
        self.front_steer = 0.0
        self.accel = 0.0
        self.brake = 0.0


class _CtrlCmdInvalid:
    def __init__(self) -> None:
        self.front_steer = 0.0
        self.accel = 0.0


class MoraiPublisherTest(unittest.TestCase):
    def test_startup_self_check_accepts_valid_contract(self) -> None:
        config = RosOutputConfig(publish_actuation=True, actuation_armed=True)
        with patch(
            "alpamayo1_5.competition.integrations.morai.publishers.import_rospy",
            return_value=_FakeRospy(),
        ), patch(
            "alpamayo1_5.competition.integrations.morai.publishers.import_message_class",
            return_value=_CtrlCmdValid,
        ):
            publisher = MoraiCtrlCmdPublisher(config)
        self.assertEqual(publisher._contract_summary["steering_field"], "front_steer")

    def test_startup_self_check_fails_fast_on_contract_mismatch(self) -> None:
        config = RosOutputConfig(publish_actuation=True, actuation_armed=True)
        with patch(
            "alpamayo1_5.competition.integrations.morai.publishers.import_rospy",
            return_value=_FakeRospy(),
        ), patch(
            "alpamayo1_5.competition.integrations.morai.publishers.import_message_class",
            return_value=_CtrlCmdInvalid,
        ):
            with self.assertRaises(MoraiActuationContractError) as ctx:
                MoraiCtrlCmdPublisher(config)
        self.assertIn("longi type 1 pedal mode", str(ctx.exception))
        self.assertIn("accel/brake + steering", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
