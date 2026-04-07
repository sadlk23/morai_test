"""Unit tests for MORAI/ROS message mapping helpers."""

from __future__ import annotations

import math
import unittest

from alpamayo1_5.competition.contracts import ControlCommand
from alpamayo1_5.competition.integrations.morai.message_mapping import (
    map_camera_message,
    map_gps_message,
    map_imu_message,
    map_route_message,
    populate_control_message,
)


class _Stamp:
    def __init__(self, value: float):
        self._value = value

    def to_sec(self) -> float:
        return self._value


class _Header:
    def __init__(self, stamp_s: float, frame_id: str = "map"):
        self.stamp = _Stamp(stamp_s)
        self.frame_id = frame_id


class _ImageMessage:
    def __init__(self) -> None:
        self.header = _Header(12.5, "camera_front")
        self.width = 640
        self.height = 480
        self.encoding = "rgb8"
        self.data = b"1234"


class _GpsMessage:
    def __init__(self) -> None:
        self.header = _Header(4.0, "gps")
        self.latitude = 37.4
        self.longitude = 127.1
        self.altitude = 30.0
        self.position_covariance = [0.1] * 9


class _Orientation:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = math.sin(math.pi / 4.0)
        self.w = math.cos(math.pi / 4.0)


class _AngularVelocity:
    def __init__(self) -> None:
        self.z = 0.25


class _LinearAcceleration:
    def __init__(self) -> None:
        self.x = 1.5


class _ImuMessage:
    def __init__(self) -> None:
        self.header = _Header(5.0, "imu")
        self.orientation = _Orientation()
        self.angular_velocity = _AngularVelocity()
        self.linear_acceleration = _LinearAcceleration()


class _RouteMessage:
    data = "turn left in 20m"


class _CtrlCmd:
    def __init__(self) -> None:
        self.longlCmdType = 0
        self.front_steer = 0.0
        self.rear_steer = 1.0
        self.accel = 0.0
        self.brake = 0.0
        self.velocity = 0.0


class MoraiMappingTest(unittest.TestCase):
    def test_map_camera_message(self) -> None:
        frame = map_camera_message(_ImageMessage(), "front", frame_id=7)
        self.assertEqual(frame.camera_id, "front")
        self.assertEqual(frame.frame_id, 7)
        self.assertEqual(frame.shape, (480, 640, 3))
        self.assertEqual(frame.encoding, "rgb8")

    def test_map_gps_message(self) -> None:
        gps_fix = map_gps_message(_GpsMessage())
        self.assertAlmostEqual(gps_fix.latitude_deg, 37.4)
        self.assertAlmostEqual(gps_fix.longitude_deg, 127.1)
        self.assertEqual(len(gps_fix.covariance or ()), 9)

    def test_map_imu_message(self) -> None:
        imu_sample = map_imu_message(_ImuMessage())
        self.assertAlmostEqual(imu_sample.yaw_rad or 0.0, math.pi / 2.0, places=4)
        self.assertAlmostEqual(imu_sample.yaw_rate_rps or 0.0, 0.25)
        self.assertAlmostEqual(imu_sample.accel_mps2 or 0.0, 1.5)

    def test_route_and_control_mapping(self) -> None:
        route = map_route_message(_RouteMessage())
        self.assertEqual(route, "turn left in 20m")
        message = populate_control_message(
            _CtrlCmd(),
            ControlCommand(
                frame_id=1,
                timestamp_s=1.0,
                steering=0.2,
                throttle=0.3,
                brake=0.0,
                target_speed_mps=4.0,
            ),
            command_mode="pedal",
        )
        self.assertEqual(message.longlCmdType, 1)
        self.assertAlmostEqual(message.front_steer, 0.2)
        self.assertAlmostEqual(message.rear_steer, 0.0)
        self.assertAlmostEqual(message.accel, 0.3)
        self.assertAlmostEqual(message.brake, 0.0)

    def test_velocity_mode_uses_kph(self) -> None:
        message = populate_control_message(
            _CtrlCmd(),
            ControlCommand(
                frame_id=1,
                timestamp_s=1.0,
                steering=0.1,
                throttle=0.2,
                brake=0.0,
                target_speed_mps=5.0,
            ),
            command_mode="velocity",
        )
        self.assertEqual(message.longlCmdType, 2)
        self.assertAlmostEqual(message.velocity, 18.0)


if __name__ == "__main__":
    unittest.main()
