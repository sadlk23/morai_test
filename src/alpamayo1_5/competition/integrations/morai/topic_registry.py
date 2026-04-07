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
    seen_topics: set[str] = set()
    for camera in config.cameras:
        for topic in [camera.topic] + list(camera.fallback_topics):
            if not topic or topic in seen_topics:
                continue
            seen_topics.add(topic)
            specs.append(
                SubscriptionSpec(
                    name=camera.name,
                    topic=topic,
                    message_type=camera.message_type,
                    sensor_kind="camera",
                    required=camera.required,
                    max_staleness_s=camera.max_staleness_s,
                )
            )

    for topic in [config.gps.topic] + list(config.gps.fallback_topics):
        if not topic or topic in seen_topics:
            continue
        seen_topics.add(topic)
        specs.append(
            SubscriptionSpec(
                name="gps",
                topic=topic,
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
    if config.optional_ego_topics.heading_topic:
        specs.append(
            SubscriptionSpec(
                name="local_heading",
                topic=config.optional_ego_topics.heading_topic,
                message_type=config.optional_ego_topics.heading_message_type,
                sensor_kind="optional_heading",
                required=False,
                max_staleness_s=1e9,
            )
        )
    if config.optional_ego_topics.utm_topic:
        for message_type in [config.optional_ego_topics.utm_message_type] + list(
            config.optional_ego_topics.utm_fallback_message_types
        ):
            if not message_type:
                continue
            specs.append(
                SubscriptionSpec(
                    name="local_utm",
                    topic=config.optional_ego_topics.utm_topic,
                    message_type=message_type,
                    sensor_kind="optional_utm",
                    required=False,
                    max_staleness_s=1e9,
                )
            )
    if config.vehicle_status.enabled and config.vehicle_status.topic:
        specs.append(
            SubscriptionSpec(
                name="vehicle_status",
                topic=config.vehicle_status.topic,
                message_type=config.vehicle_status.message_type,
                sensor_kind="vehicle_status",
                required=False,
                max_staleness_s=config.vehicle_status.max_staleness_s,
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
