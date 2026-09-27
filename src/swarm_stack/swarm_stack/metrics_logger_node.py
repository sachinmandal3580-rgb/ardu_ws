#!/usr/bin/env python3
"""Run-level evidence for the challenge metrics, recorded once per second."""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from .scenario import Scenario
from swarm_interfaces.msg import (CommunicationStats, ConnectivityGraph, FaultEvent,
                                  PoI, UAVStatus)
from .planning import distance, within_fence


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger_node")
        self.declare_parameter("scenario_config_path", "")
        self.scenario = Scenario.load(self.get_parameter("scenario_config_path").value) if self.get_parameter("scenario_config_path").value else None
        self.sim_time = 0.0
        self.epoch = None
        self.sample_violations = {k: set() for k in ("separation", "speed", "altitude", "endurance", "late_report", "landing_deadline")}
        self.create_subscription(Clock, "/drone1/clock", self._on_sim_clock, 10)
        self.declare_parameter("uav_names", [f"drone{i}" for i in range(1, 11)])
        self.declare_parameter("log_path", os.path.expanduser(time.strftime("~/ardu_ws/logs/swarm_metrics_%Y%m%d_%H%M%S.csv")))
        self.declare_parameter("log_rate_hz", 1.0)
        self.declare_parameter("fence_center", [0.0, 0.0, 0.0])
        self.declare_parameter("fence_radius_m", 400.0)
        self.declare_parameter("collision_distance_m", 1.5)
        self.names = list(self.get_parameter("uav_names").value)
        self.path = self.get_parameter("log_path").value
        self.fence = tuple(self.get_parameter("fence_center").value)
        self.fence_radius = float(self.get_parameter("fence_radius_m").value)
        self.collision_distance = float(self.get_parameter("collision_distance_m").value)
        self.start = time.monotonic()
        self.status = {}
        self.pois = {}
        self.graph = None
        self.comms = None
        self.samples = 0
        self.availability_sum = 0.0
        self.downtime_uav_s = 0.0
        self.minimum_separation = float("inf")
        self.collision_pairs = set()
        self.collision_count = 0
        self.geofence_violations = set()
        self.battery_exhaustions = set()
        self.disconnected_since = {}
        self.recovery_times = []
        self.connectivity_recovery_times = []
        self.network_recoveries_seen = 0
        self.fault_count = 0
        self.completion_time = None
        self.relay_reallocations = 0
        self.create_subscription(UAVStatus, "/swarm/uav_status", self._status, 10)
        self.create_subscription(PoI, "/swarm/poi_updates", self._poi, 10)
        self.create_subscription(ConnectivityGraph, "/swarm/connectivity_graph", self._graph, 10)
        self.create_subscription(CommunicationStats, "/swarm/communication_stats", self._comms, 10)
        self.create_subscription(FaultEvent, "/swarm/fault_events", self._fault, 10)
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "t_s", "mission_completion_pct", "priority_weighted_score_pct", "mission_completion_time_s",
            "packets_sent", "packets_delivered", "end_to_end_pdr_pct", "mean_delivered_latency_ms",
            "connectivity_availability_pct", "communication_downtime_uav_s", "relay_reallocations",
            "mean_recovery_s", "mean_connectivity_recovery_s", "fault_count", "collision_count", "min_inter_uav_separation_m",
            "geofence_violations", "battery_exhaustions", "separation_violations",
            "speed_violations", "altitude_violations", "endurance_violations", "late_reports", "landing_deadline_failures",
        ])
        self.create_timer(1.0 / float(self.get_parameter("log_rate_hz").value), self._tick)
        self.get_logger().info(f"Writing run metrics to {self.path}")

    def _on_sim_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec/1e9
        if self.epoch is None: self.epoch = now
        self.sim_time = now-self.epoch

    def _status(self, msg):
        previous = self.status.get(msg.name)
        if previous is not None and previous.connected and not msg.connected:
            self.disconnected_since[msg.name] = time.monotonic()
        if previous is not None and not previous.connected and msg.connected:
            started = self.disconnected_since.pop(msg.name, None)
            if started is not None:
                self.connectivity_recovery_times.append(time.monotonic() - started)
        if msg.network_recoveries > self.network_recoveries_seen:
            self.network_recoveries_seen = msg.network_recoveries
            self.recovery_times.append(msg.last_network_recovery_s)
        self.status[msg.name] = msg
        self.relay_reallocations = max(self.relay_reallocations, msg.relay_reallocations)

    def _poi(self, msg):
        self.pois[msg.id] = msg

    def _graph(self, msg):
        self.graph = msg

    def _comms(self, msg):
        self.comms = msg

    def _fault(self, msg):
        if msg.event_type in ("uav_failure", "link_degraded"):
            self.fault_count += 1

    def _tick(self):
        if not self.graph or (not self.scenario and not self.pois):
            return
        self.samples += 1
        count = self.scenario.poi_count if self.scenario else len(self.pois)
        surveyed = sum(p.surveyed for p in self.pois.values())
        complete = 100.0 * surveyed / count
        total_priority = (sum(p["priority"] for p in self.scenario.random_pois()) if self.scenario else
                          sum(max(1, p.priority) for p in self.pois.values()))
        weighted = 100.0 * sum(max(1, p.priority) for p in self.pois.values() if p.surveyed) / total_priority
        elapsed = self.sim_time if self.scenario else time.monotonic() - self.start
        if surveyed == count and self.completion_time is None:
            self.completion_time = elapsed
        operating = [n for n in self.names if n in self.status and self.status[n].flight_state == "ready" and self.status[n].role not in ("failed", "rtl")]
        hops = dict(zip(self.graph.uav_names, self.graph.hop_counts))
        connected = sum(hops.get(n, -1) >= 0 for n in operating)
        availability = 100.0 * connected / len(operating) if operating else 100.0
        self.availability_sum += availability
        self.downtime_uav_s += (len(operating) - connected) / float(self.get_parameter("log_rate_hz").value)
        current_pairs = set()
        current_min = float("inf")
        positioned = [(n, self.status[n].position) for n in self.names if n in self.status
                      and self.status[n].position_valid]
        for i, (a, pos_a) in enumerate(positioned):
            xyz_a = (pos_a.x, pos_a.y, pos_a.z)
            for b, pos_b in positioned[i + 1:]:
                separation = distance(xyz_a, (pos_b.x, pos_b.y, pos_b.z))
                current_min = min(current_min, separation)
                if self.scenario and separation < self.scenario.min_separation_m:
                    self.sample_violations["separation"].add((a,b))
                if separation < self.collision_distance:
                    current_pairs.add((a, b))
            if not (self.scenario.allowed(xyz_a) if self.scenario else within_fence(xyz_a, self.fence, self.fence_radius)):
                self.geofence_violations.add(a)
            if self.status[a].battery_pct >= 0 and self.status[a].battery_pct <= 0.0:
                self.battery_exhaustions.add(a)
        if self.scenario:
            for name, status in self.status.items():
                if status.speed_mps > self.scenario.max_speed_mps + 0.01: self.sample_violations["speed"].add(name)
                if status.position.z > self.scenario.max_altitude_m: self.sample_violations["altitude"].add(name)
                if status.flight_elapsed_s > self.scenario.uav_endurance_s: self.sample_violations["endurance"].add(name)
                if elapsed >= self.scenario.mission_duration_s and (status.armed or status.position.z > 1.5):
                    self.sample_violations["landing_deadline"].add(name)
            for poi in self.pois.values():
                if poi.detected_s >= 0 and ((poi.reported_s if poi.surveyed else elapsed)-poi.detected_s > self.scenario.report_deadline_s):
                    self.sample_violations["late_report"].add(poi.id)
        self.collision_count += len(current_pairs - self.collision_pairs)
        self.collision_pairs = current_pairs
        self.minimum_separation = min(self.minimum_separation, current_min)
        sent = self.comms.packets_sent if self.comms else 0
        delivered = self.comms.packets_delivered if self.comms else 0
        pdr = 100.0 * delivered / sent if sent else 0.0
        latency = self.comms.mean_delivered_latency_ms if self.comms else 0.0
        recovery = sum(self.recovery_times) / len(self.recovery_times) if self.recovery_times else -1.0
        connectivity_recovery = (sum(self.connectivity_recovery_times) / len(self.connectivity_recovery_times)
                                 if self.connectivity_recovery_times else -1.0)
        self.writer.writerow([
            f"{elapsed:.1f}", f"{complete:.1f}", f"{weighted:.1f}",
            f"{self.completion_time:.1f}" if self.completion_time is not None else "",
            sent, delivered, f"{pdr:.1f}", f"{latency:.1f}",
            f"{self.availability_sum / self.samples:.1f}", f"{self.downtime_uav_s:.1f}",
            self.relay_reallocations, f"{recovery:.1f}", f"{connectivity_recovery:.1f}", self.fault_count,
            self.collision_count,
            f"{self.minimum_separation:.2f}" if math.isfinite(self.minimum_separation) else "",
            len(self.geofence_violations), len(self.battery_exhaustions),
            *[len(self.sample_violations[k]) for k in ("separation", "speed", "altitude", "endurance", "late_report", "landing_deadline")],
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
