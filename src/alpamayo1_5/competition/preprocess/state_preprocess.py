"""State preprocessing and simple ego-state estimation."""

from __future__ import annotations

import math

from alpamayo1_5.competition.contracts import EgoState, GpsFix, ImuSample


def _meters_per_degree_lat() -> float:
    return 111_320.0


def _meters_per_degree_lon(latitude_deg: float) -> float:
    return 111_320.0 * math.cos(math.radians(latitude_deg))


class StatePreprocessor:
    """Estimate a lightweight local ego state from GPS and IMU."""

    def __init__(self) -> None:
        self._origin: GpsFix | None = None
        self._prev_gps: GpsFix | None = None
        self._prev_timestamp_s: float | None = None

    def _local_xy(self, gps_fix: GpsFix) -> tuple[float, float]:
        if self._origin is None:
            self._origin = gps_fix
        dx = (gps_fix.longitude_deg - self._origin.longitude_deg) * _meters_per_degree_lon(
            self._origin.latitude_deg
        )
        dy = (gps_fix.latitude_deg - self._origin.latitude_deg) * _meters_per_degree_lat()
        return dx, dy

    def estimate(
        self,
        timestamp_s: float,
        gps_fix: GpsFix | None,
        imu_sample: ImuSample | None,
    ) -> EgoState:
        diagnostics: dict[str, float | str | None] = {}
        x_m = y_m = speed_mps = 0.0
        heading_rad = yaw_rate_rps = accel_mps2 = 0.0
        valid = True

        if gps_fix is None and imu_sample is None:
            return EgoState(
                timestamp_s=timestamp_s,
                valid=False,
                diagnostics={"reason": "missing_gps_and_imu"},
            )

        if gps_fix is not None:
            x_m, y_m = self._local_xy(gps_fix)
            if gps_fix.speed_mps is not None:
                speed_mps = gps_fix.speed_mps
            elif self._prev_gps is not None and self._prev_timestamp_s is not None:
                dt = max(1e-3, gps_fix.timestamp_s - self._prev_timestamp_s)
                prev_x, prev_y = self._local_xy(self._prev_gps)
                dx = x_m - prev_x
                dy = y_m - prev_y
                speed_mps = math.hypot(dx, dy) / dt
                if gps_fix.track_rad is None and abs(dx) + abs(dy) > 1e-6:
                    heading_rad = math.atan2(dy, dx)
            if gps_fix.track_rad is not None:
                heading_rad = gps_fix.track_rad
            diagnostics["gps_timestamp_s"] = gps_fix.timestamp_s
            self._prev_gps = gps_fix
            self._prev_timestamp_s = gps_fix.timestamp_s
        else:
            valid = False
            diagnostics["gps_missing"] = "true"

        if imu_sample is not None:
            if imu_sample.yaw_rad is not None:
                heading_rad = imu_sample.yaw_rad
            if imu_sample.yaw_rate_rps is not None:
                yaw_rate_rps = imu_sample.yaw_rate_rps
            if imu_sample.accel_mps2 is not None:
                accel_mps2 = imu_sample.accel_mps2
            diagnostics["imu_timestamp_s"] = imu_sample.timestamp_s
        else:
            diagnostics["imu_missing"] = "true"

        return EgoState(
            timestamp_s=timestamp_s,
            x_m=x_m,
            y_m=y_m,
            heading_rad=heading_rad,
            speed_mps=speed_mps,
            yaw_rate_rps=yaw_rate_rps,
            accel_mps2=accel_mps2,
            valid=valid,
            diagnostics=diagnostics,
        )
