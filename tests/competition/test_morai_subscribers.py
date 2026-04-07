"""Tests for optional MORAI subscriber parsing helpers."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.integrations.morai.subscribers import (
    _extract_optional_heading_rad,
    _optional_ego_diagnostics,
    _extract_optional_utm,
    _extract_vehicle_status,
)


class _Point:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class _PointStamped:
    def __init__(self) -> None:
        self.point = _Point(12.0, 34.0)


class _FloatArray:
    def __init__(self) -> None:
        self.data = [56.0, 78.0]


class _Float32Heading:
    def __init__(self) -> None:
        self.data = 1.25


class _Float64Heading:
    def __init__(self) -> None:
        self.data = 2.5


class _UnsupportedHeading:
    pass


class _UnsupportedUtm:
    pass


class _VehicleStatusArray:
    def __init__(self) -> None:
        self.data = [1.0, 0.0, 0.0, 4.5, 0.1, 0.2, 7.0]


class _VehicleStatusMsg:
    def __init__(self) -> None:
        self.speed = 3.2
        self.gear = 1.0
        self.brake = 0.4
        self.steering = 0.12


class MoraiSubscribersTest(unittest.TestCase):
    def test_extract_optional_heading_supports_float32(self) -> None:
        heading = _extract_optional_heading_rad(_Float32Heading())
        self.assertEqual(heading, 1.25)

    def test_extract_optional_heading_supports_float64(self) -> None:
        heading = _extract_optional_heading_rad(_Float64Heading())
        self.assertEqual(heading, 2.5)

    def test_extract_optional_heading_gracefully_rejects_unsupported_message(self) -> None:
        with self.assertRaises(ValueError):
            _extract_optional_heading_rad(_UnsupportedHeading())
        diagnostics = _optional_ego_diagnostics(
            local_heading_rad=None,
            local_heading_timestamp_s=None,
            local_heading_source_type=None,
            last_heading_error="ValueError: could not parse heading from message",
            local_utm_xy=None,
            local_utm_timestamp_s=None,
            local_utm_source_type=None,
            last_utm_error=None,
        )
        self.assertFalse(diagnostics["heading_available"])
        self.assertEqual(diagnostics["last_heading_error"], "ValueError: could not parse heading from message")

    def test_extract_optional_utm_supports_point_stamped(self) -> None:
        utm = _extract_optional_utm(_PointStamped())
        self.assertEqual(utm["x_m"], 12.0)
        self.assertEqual(utm["y_m"], 34.0)

    def test_extract_optional_utm_supports_float_array(self) -> None:
        utm = _extract_optional_utm(_FloatArray())
        self.assertEqual(utm["x_m"], 56.0)
        self.assertEqual(utm["y_m"], 78.0)

    def test_extract_optional_utm_gracefully_rejects_unsupported_message(self) -> None:
        with self.assertRaises(ValueError):
            _extract_optional_utm(_UnsupportedUtm())

    def test_extract_vehicle_status_supports_serial_array(self) -> None:
        status = _extract_vehicle_status(_VehicleStatusArray())
        self.assertEqual(status["speed_mps"], 4.5)
        self.assertEqual(status["steer_rad"], 0.1)

    def test_extract_vehicle_status_supports_named_fields(self) -> None:
        status = _extract_vehicle_status(_VehicleStatusMsg())
        self.assertEqual(status["speed_mps"], 3.2)
        self.assertEqual(status["gear"], 1.0)


if __name__ == "__main__":
    unittest.main()
