"""Topic registry helpers for live MORAI runtime wiring."""

from __future__ import annotations

from dataclasses import dataclass

from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


@dataclass(slots=True)
class SubscriptionSpec:
    """One ROS subscription definition tied to a runtime contract mapping."""

    name: str
    topic: str
    message_type: str
    sensor_kind: str
    required: bool
    max_staleness_s: float


def build_subscription_specs(config: CompetitionConfig) -> list[SubscriptionSpec]:
    """Build live ROS subscriptions from the competition config."""

    specs: list[SubscriptionSpec] = []
    for camera in config.cameras:
        specs.append(
            SubscriptionSpec(
                name=camera.name,
                topic=camera.topic,
                message_type=camera.message_type,
                sensor_kind="camera",
                required=camera.required,
                max_staleness_s=camera.max_staleness_s,
            )
        )

    specs.append(
        SubscriptionSpec(
            name="gps",
            topic=config.gps.topic,
            message_type=config.gps.message_type,
            sensor_kind="gps",
            required=config.gps.required,
            max_staleness_s=config.gps.max_staleness_s,
        )
    )
    specs.append(
        SubscriptionSpec(
            name="imu",
            topic=config.imu.topic,
            message_type=config.imu.message_type,
            sensor_kind="imu",
            required=config.imu.required,
            max_staleness_s=config.imu.max_staleness_s,
        )
    )

    if config.route_command.topic:
        specs.append(
            SubscriptionSpec(
                name="route_command",
                topic=config.route_command.topic,
                message_type=config.route_command.message_type,
                sensor_kind="route_command",
                required=config.route_command.required,
                max_staleness_s=config.route_command.max_staleness_s,
            )
        )

    if config.use_lidar and config.lidar.topic:
        specs.append(
            SubscriptionSpec(
                name="lidar",
                topic=config.lidar.topic,
                message_type=config.lidar.message_type,
                sensor_kind="lidar",
                required=config.lidar.required,
                max_staleness_s=config.lidar.max_staleness_s,
            )
        )
    return specs
