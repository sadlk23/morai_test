"""Waypoint validation and speed postprocessing."""

from __future__ import annotations

import math


def _is_finite_pair(point: tuple[float, float]) -> bool:
    return math.isfinite(point[0]) and math.isfinite(point[1])


def compute_segment_lengths(waypoints_xy: list[tuple[float, float]]) -> list[float]:
    """Compute Euclidean segment lengths."""

    lengths: list[float] = []
    for idx in range(1, len(waypoints_xy)):
        x0, y0 = waypoints_xy[idx - 1]
        x1, y1 = waypoints_xy[idx]
        lengths.append(math.hypot(x1 - x0, y1 - y0))
    return lengths


def compute_path_curvatures(waypoints_xy: list[tuple[float, float]]) -> list[float]:
    """Estimate discrete curvature from successive headings."""

    if len(waypoints_xy) < 3:
        return [0.0]

    curvatures: list[float] = []
    for idx in range(2, len(waypoints_xy)):
        x0, y0 = waypoints_xy[idx - 2]
        x1, y1 = waypoints_xy[idx - 1]
        x2, y2 = waypoints_xy[idx]
        heading_a = math.atan2(y1 - y0, x1 - x0)
        heading_b = math.atan2(y2 - y1, x2 - x1)
        ds = max(1e-3, math.hypot(x2 - x1, y2 - y1))
        d_heading = math.atan2(math.sin(heading_b - heading_a), math.cos(heading_b - heading_a))
        curvatures.append(abs(d_heading) / ds)
    return curvatures or [0.0]


def is_valid_waypoint_set(waypoints_xy: list[tuple[float, float]]) -> bool:
    """Check for finite, non-empty, forward-progressing waypoints."""

    if len(waypoints_xy) < 2:
        return False
    if not all(_is_finite_pair(point) for point in waypoints_xy):
        return False
    forward_progress = sum(max(0.0, point[0]) for point in waypoints_xy[1:])
    return forward_progress > 0.1


def derive_target_speed(
    waypoints_xy: list[tuple[float, float]],
    min_speed_mps: float,
    max_speed_mps: float,
    max_curvature_for_full_speed: float,
) -> float:
    """Convert path curvature into a conservative target speed."""

    if not is_valid_waypoint_set(waypoints_xy):
        return min_speed_mps
    max_curvature = max(compute_path_curvatures(waypoints_xy))
    if max_curvature <= 1e-6:
        return max_speed_mps
    ratio = min(1.0, max_curvature / max(1e-6, max_curvature_for_full_speed))
    target = max_speed_mps * (1.0 - 0.65 * ratio)
    return max(min_speed_mps, min(max_speed_mps, target))
