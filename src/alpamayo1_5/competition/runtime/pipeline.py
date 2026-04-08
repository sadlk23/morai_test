"""End-to-end competition runtime pipeline."""

from __future__ import annotations

import logging
from contextlib import nullcontext
from time import perf_counter
from typing import Iterable

from alpamayo1_5.competition.contracts import (
    ControlCommand,
    DebugSnapshot,
    PlanResult,
    PlannerInput,
    SafetyDecision,
    SensorPacket,
    SynchronizedFrame,
)
from alpamayo1_5.competition.controllers.controller_runtime import ControllerRuntime
from alpamayo1_5.competition.io.ros_interface import RosCommandPublisher, RosInterfaceUnavailable
from alpamayo1_5.competition.io.sync import SensorSynchronizer
from alpamayo1_5.competition.io.udp_interface import UdpCommandPublisher
from alpamayo1_5.competition.planners.planner_runtime import CompetitionPlanner
from alpamayo1_5.competition.preprocess.image_preprocess import ImagePreprocessor
from alpamayo1_5.competition.preprocess.model_input import ModelInputPackager
from alpamayo1_5.competition.preprocess.sensor_fusion import SensorFusion
from alpamayo1_5.competition.preprocess.state_preprocess import StatePreprocessor
from alpamayo1_5.competition.integrations.morai.legacy_serial_bridge import legacy_serial_bridge_diagnostics
from alpamayo1_5.competition.runtime.config_competition import (
    CompetitionConfig,
    competition_profile_diagnostics,
    morai_udp_reference_diagnostics,
    runtime_policy_diagnostics,
)
from alpamayo1_5.competition.runtime.debug_dump import DebugDumper
from alpamayo1_5.competition.runtime.latency_monitor import LatencyMonitor
from alpamayo1_5.competition.runtime.metrics import JsonlWriter
from alpamayo1_5.competition.safety.safety_filter import SafetyFilter

logger = logging.getLogger(__name__)


class CompetitionRuntimePipeline:
    """Single-cycle competition runtime with explicit stage boundaries."""

    def __init__(self, config: CompetitionConfig, publishers: list[object] | None = None):
        self.config = config
        self.synchronizer = SensorSynchronizer(config)
        self.image_preprocessor = ImagePreprocessor(config)
        self.state_preprocessor = StatePreprocessor()
        self.sensor_fusion = SensorFusion()
        self.model_input_packager = ModelInputPackager(config)
        self.planner = CompetitionPlanner(config)
        self.controller = ControllerRuntime(config)
        self.safety_filter = SafetyFilter(config)
        self.debug_dumper = DebugDumper(config.logging.log_dir)
        self.metrics_writer = JsonlWriter(f"{config.logging.log_dir}/metrics.jsonl")
        self.publishers: list[object] = list(publishers or [])
        self.publisher_warnings: list[str] = []

        if publishers is None:
            if config.output_mode in {"ros", "dual"} and config.ros_output.enabled:
                try:
                    self.publishers.append(RosCommandPublisher(config.ros_output))
                except RosInterfaceUnavailable as exc:
                    warning = f"ros_publisher_unavailable: {exc}"
                    self.publisher_warnings.append(warning)
                    logger.warning(warning)
            if config.output_mode in {"udp", "dual"} and config.udp_output.enabled:
                self.publishers.append(UdpCommandPublisher(config.udp_output))
        if not self.publishers:
            warning = "no_publishers_configured_or_available"
            self.publisher_warnings.append(warning)
            logger.warning(warning)

    def _publish(self, decision: SafetyDecision, snapshot: DebugSnapshot) -> list[str]:
        publish_errors: list[str] = []
        for publisher in self.publishers:
            try:
                if hasattr(publisher, "publish"):
                    publisher.publish(decision)
                if hasattr(publisher, "publish_debug"):
                    publisher.publish_debug(snapshot)
            except Exception as exc:
                error = f"{publisher.__class__.__name__}:{type(exc).__name__}:{exc}"
                publish_errors.append(error)
                logger.exception("Publisher failure: %s", error)
        return publish_errors

    def _measure(self, monitor: LatencyMonitor, stage_name: str):
        """Return a timing context only when latency profiling is enabled."""

        if self.config.logging.enable_latency_profiling:
            return monitor.measure(stage_name)
        return nullcontext()

    def _build_fail_closed_command(
        self,
        packet: SensorPacket,
        planner_input: PlannerInput | None,
        reason: str,
    ) -> ControlCommand:
        """Build an emergency stop command even when runtime recovery is partial."""

        if planner_input is not None:
            return self.safety_filter._stop_command(planner_input, reason)
        return ControlCommand(
            frame_id=packet.frame_id,
            timestamp_s=packet.timestamp_s,
            steering=0.0,
            throttle=0.0,
            brake=self.config.safety.emergency_brake_value,
            target_speed_mps=0.0,
            valid=False,
            saturated=True,
            reason=reason,
        )

    def _recover_runtime_context(
        self,
        packet: SensorPacket,
        synchronized: SynchronizedFrame | None,
        planner_input: PlannerInput | None,
    ) -> tuple[SynchronizedFrame, PlannerInput | None]:
        """Recover enough runtime context to build a conservative stop command."""

        recovered_sync = synchronized
        recovered_planner_input = planner_input
        if recovered_sync is None:
            recovered_sync = self.synchronizer.synchronize(packet)
        if recovered_planner_input is None:
            image_features = self.image_preprocessor.preprocess(recovered_sync)
            ego_state = self.state_preprocessor.estimate(
                packet.timestamp_s,
                recovered_sync.gps_fix,
                recovered_sync.imu_sample,
            )
            recovered_planner_input = self.sensor_fusion.fuse(
                recovered_sync,
                ego_state,
                image_features,
            )
        return recovered_sync, recovered_planner_input

    def run_cycle(self, packet: SensorPacket) -> tuple[SafetyDecision, DebugSnapshot]:
        """Execute one full control cycle."""

        dt_s = 1.0 / max(1.0, self.config.control_hz)
        monitor = LatencyMonitor()
        cycle_start = perf_counter()
        synchronized = None
        planner_input = None
        plan = PlanResult(
            frame_id=packet.frame_id,
            timestamp_s=packet.timestamp_s,
            planner_name="uninitialized",
            waypoints_xy=[],
            target_speed_mps=0.0,
            confidence=0.0,
            stop_probability=1.0,
            risk_score=1.0,
            valid=False,
        )
        try:
            with self._measure(monitor, "sync"):
                synchronized = self.synchronizer.synchronize(packet)
            with self._measure(monitor, "image_preprocess"):
                image_features = self.image_preprocessor.preprocess(synchronized)
            with self._measure(monitor, "state_preprocess"):
                ego_state = self.state_preprocessor.estimate(
                    packet.timestamp_s,
                    synchronized.gps_fix,
                    synchronized.imu_sample,
                )
            with self._measure(monitor, "fusion"):
                planner_input = self.sensor_fusion.fuse(synchronized, ego_state, image_features)
            with self._measure(monitor, "model_input"):
                planner_input.model_input_package = self.model_input_packager.build(planner_input)
            with self._measure(monitor, "planner"):
                plan = self.planner.plan(planner_input)
            with self._measure(monitor, "controller"):
                command = self.controller.compute(planner_input, plan, dt_s)
            with self._measure(monitor, "safety"):
                decision = self.safety_filter.apply(planner_input, plan, command)

            snapshot = DebugSnapshot(
                frame_id=packet.frame_id,
                timestamp_s=packet.timestamp_s,
                stage_latency_ms=monitor.snapshot(),
                fused_feature_summary={
                    **planner_input.fused_features,
                    "model_input": planner_input.model_input_package.diagnostics,
                },
                waypoints_xy=plan.waypoints_xy,
                target_speed_mps=plan.target_speed_mps,
                controller_output={
                    "steering": decision.command.steering,
                    "throttle": decision.command.throttle,
                    "brake": decision.command.brake,
                },
                safety_flags=decision.safety_flags,
                diagnostics={
                    "planner_name": plan.planner_name,
                    "plan_confidence": plan.confidence,
                    "plan_valid": plan.valid,
                    "decision_intervention": decision.intervention,
                    "competition_profile": competition_profile_diagnostics(self.config),
                    "runtime_policy": runtime_policy_diagnostics(self.config),
                    "morai_udp_reference": morai_udp_reference_diagnostics(self.config),
                },
            )
            decision.diagnostics["competition_profile"] = competition_profile_diagnostics(self.config)
            decision.diagnostics["runtime_policy"] = runtime_policy_diagnostics(self.config)
            decision.diagnostics["morai_udp_reference"] = morai_udp_reference_diagnostics(self.config)
            if self.config.legacy_serial_bridge.enabled:
                snapshot.diagnostics["legacy_serial_bridge"] = legacy_serial_bridge_diagnostics(
                    self.config.legacy_serial_bridge
                )
                decision.diagnostics["legacy_serial_bridge"] = legacy_serial_bridge_diagnostics(
                    self.config.legacy_serial_bridge
                )

            publish_errors: list[str] = []
            with self._measure(monitor, "publish"):
                publish_errors = self._publish(decision, snapshot)
            if publish_errors:
                snapshot.safety_flags.extend(["publish_failure"])
                snapshot.diagnostics["publish_errors"] = publish_errors
        except Exception as exc:
            runtime_error = f"{type(exc).__name__}: {exc}"
            logger.exception("Competition runtime cycle failed frame=%s", packet.frame_id)
            recovery_error: str | None = None
            recovery_status = "context_reused"
            try:
                synchronized, planner_input = self._recover_runtime_context(
                    packet,
                    synchronized,
                    planner_input,
                )
            except Exception as recovery_exc:
                recovery_status = "recovery_failed"
                recovery_error = f"{type(recovery_exc).__name__}: {recovery_exc}"
                logger.exception(
                    "Competition runtime fail-closed recovery failed frame=%s",
                    packet.frame_id,
                )
            else:
                if synchronized is None and planner_input is None:
                    recovery_status = "no_context_available"
                elif planner_input is None:
                    recovery_status = "sync_only_recovered"
                else:
                    recovery_status = "planner_input_recovered"
            safety_flags = ["runtime_exception"]
            if recovery_error is not None:
                safety_flags.append("runtime_exception_recovery_failed")
            decision = SafetyDecision(
                frame_id=packet.frame_id,
                timestamp_s=packet.timestamp_s,
                command=self._build_fail_closed_command(packet, planner_input, "runtime_exception"),
                intervention="runtime_exception_stop",
                risk_level="critical",
                safety_flags=safety_flags,
                fallback_used=True,
                diagnostics={
                    "error": runtime_error,
                    "recovery_status": recovery_status,
                },
            )
            if recovery_error is not None:
                decision.diagnostics["recovery_error"] = recovery_error
            if synchronized is not None:
                decision.diagnostics["sync_diagnostics"] = dict(synchronized.diagnostics)
            if planner_input is not None:
                decision.diagnostics["planner_input_valid"] = planner_input.valid
                decision.diagnostics["planner_input_diagnostics"] = dict(planner_input.diagnostics)
            decision.diagnostics["competition_profile"] = competition_profile_diagnostics(self.config)
            decision.diagnostics["runtime_policy"] = runtime_policy_diagnostics(self.config)
            decision.diagnostics["morai_udp_reference"] = morai_udp_reference_diagnostics(self.config)
            snapshot = DebugSnapshot(
                frame_id=packet.frame_id,
                timestamp_s=packet.timestamp_s,
                safety_flags=list(safety_flags),
                stage_latency_ms=monitor.snapshot(),
                diagnostics={
                    "runtime_error": runtime_error,
                    "recovery_status": recovery_status,
                    "publisher_warnings": list(self.publisher_warnings),
                    "competition_profile": competition_profile_diagnostics(self.config),
                    "runtime_policy": runtime_policy_diagnostics(self.config),
                    "morai_udp_reference": morai_udp_reference_diagnostics(self.config),
                },
            )
            if recovery_error is not None:
                snapshot.diagnostics["recovery_error"] = recovery_error
            if synchronized is not None:
                snapshot.diagnostics["sync_diagnostics"] = dict(synchronized.diagnostics)
            if planner_input is not None:
                snapshot.diagnostics["planner_input_valid"] = planner_input.valid
                snapshot.diagnostics["planner_input_diagnostics"] = dict(planner_input.diagnostics)
            publish_errors = []
            with self._measure(monitor, "publish"):
                publish_errors = self._publish(decision, snapshot)
            if publish_errors:
                snapshot.safety_flags.extend(["publish_failure"])
                snapshot.diagnostics["publish_errors"] = publish_errors

        snapshot.stage_latency_ms = monitor.snapshot()
        snapshot.stage_latency_ms["total_cycle"] = (perf_counter() - cycle_start) * 1_000.0
        snapshot.diagnostics.setdefault("publisher_warnings", list(self.publisher_warnings))
        if self.config.logging.write_metrics_jsonl:
            self.metrics_writer.write(
                {
                    "frame_id": packet.frame_id,
                    "timestamp_s": packet.timestamp_s,
                    "stage_latency_ms": snapshot.stage_latency_ms,
                    "intervention": decision.intervention,
                    "risk_level": decision.risk_level,
                }
            )
        if self.config.logging.write_debug_jsonl:
            self.debug_dumper.write_snapshot(snapshot)
        if self.config.logging.write_command_history_jsonl:
            self.debug_dumper.write_command(
                {
                    "frame_id": decision.frame_id,
                    "timestamp_s": decision.timestamp_s,
                    "steering": decision.command.steering,
                    "throttle": decision.command.throttle,
                    "brake": decision.command.brake,
                    "target_speed_mps": decision.command.target_speed_mps,
                    "intervention": decision.intervention,
                    "flags": decision.safety_flags,
                }
            )
        if self.config.logging.save_last_valid_plan and plan.valid:
            self.debug_dumper.write_last_valid_plan(plan)

        return decision, snapshot

    def run_replay(self, packets: Iterable[SensorPacket]) -> list[tuple[SafetyDecision, DebugSnapshot]]:
        """Execute repeated cycles over a replay iterator."""

        outputs: list[tuple[SafetyDecision, DebugSnapshot]] = []
        for packet in packets:
            outputs.append(self.run_cycle(packet))
        return outputs
