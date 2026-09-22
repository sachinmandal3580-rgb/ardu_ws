#!/usr/bin/env python3
"""
metrics_logger_node
--------------------
Subscribes to the swarm's live state and writes a CSV row every second with
exactly the figures the Stage-1 report template asks for (see the "Evaluation
& Performance Metrics" table in the technical proposal template). Also prints
a final summary line at shutdown (Ctrl+C) for a quick copy into the report.
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node

from swarm_interfaces.msg import ConnectivityGraph, UAVStatus, PoI

SAFETY_SEPARATION_M = 1.0  # minimum acceptable inter-UAV separation


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger_node")

        self.declare_parameter("uav_names", ["drone1", "drone2", "drone3", "drone4", "drone5"])
        self.declare_parameter("log_path", os.path.expanduser("~/ardu_ws/swarm_metrics.csv"))
        self.declare_parameter("log_rate_hz", 1.0)

        self.uav_names = self.get_parameter("uav_names").value
        self.log_path = self.get_parameter("log_path").value
        rate = self.get_parameter("log_rate_hz").value

        self.latest_graph = None
        self.uav_status = {}
        self.pois = {}
        self.relay_count_seen = 0
        self._last_roles = {}
        self.unsafe_ticks = 0
        self.total_ticks = 0
        self.disconnect_start = {}  # name -> timestamp when it went disconnected
        self.recovery_times = []
        self.start_time = time.time()

        self.create_subscription(ConnectivityGraph, "/swarm/connectivity_graph", self._graph_cb, 10)
        self.create_subscription(UAVStatus, "/swarm/uav_status", self._status_cb, 10)
        self.create_subscription(PoI, "/swarm/poi_updates", self._poi_cb, 10)

        self._init_csv()
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"Logging metrics to {self.log_path}")

    def _init_csv(self):
        new_file = not os.path.exists(self.log_path)
        self._csv_file = open(self.log_path, "a", newline="")
        self._writer = csv.writer(self._csv_file)
        if new_file:
            self._writer.writerow([
                "t_s", "mission_completion_pct", "avg_pdr", "avg_latency_ms",
                "connectivity_availability_pct", "relay_reallocations",
                "min_separation_m", "unsafe_tick_pct",
            ])

    def _graph_cb(self, msg: ConnectivityGraph):
        self.latest_graph = msg

    def _status_cb(self, msg: UAVStatus):
        prev_role = self._last_roles.get(msg.name)
        if prev_role != "relay" and msg.role == "relay":
            self.relay_count_seen += 1
        self._last_roles[msg.name] = msg.role

        was_connected = self.uav_status.get(msg.name, UAVStatus()).connected if msg.name in self.uav_status else True
        if was_connected and not msg.connected:
            self.disconnect_start[msg.name] = time.time()
        if (not was_connected) and msg.connected and msg.name in self.disconnect_start:
            self.recovery_times.append(time.time() - self.disconnect_start.pop(msg.name))

        self.uav_status[msg.name] = msg

    def _poi_cb(self, msg: PoI):
        self.pois[msg.id] = msg

    def _tick(self):
        self.total_ticks += 1

        # Mission completion
        if self.pois:
            surveyed = sum(1 for p in self.pois.values() if p.surveyed)
            mission_completion = 100.0 * surveyed / len(self.pois)
        else:
            mission_completion = 0.0

        # Comms metrics from the latest graph snapshot
        avg_pdr, avg_latency, connectivity_avail, min_sep = 0.0, 0.0, 0.0, float("inf")
        if self.latest_graph and self.latest_graph.edges:
            up_edges = [e for e in self.latest_graph.edges if e.link_up]
            if up_edges:
                avg_pdr = sum(e.pdr for e in up_edges) / len(up_edges)
                avg_latency = sum(e.latency_ms for e in up_edges) / len(up_edges)
            min_sep = min((e.distance_m for e in self.latest_graph.edges), default=float("inf"))
            if self.latest_graph.hop_counts:
                connected = sum(1 for h in self.latest_graph.hop_counts if h != -1)
                connectivity_avail = 100.0 * connected / len(self.latest_graph.hop_counts)

        if min_sep < SAFETY_SEPARATION_M:
            self.unsafe_ticks += 1

        t = time.time() - self.start_time
        self._writer.writerow([
            f"{t:.1f}", f"{mission_completion:.1f}", f"{avg_pdr:.3f}", f"{avg_latency:.1f}",
            f"{connectivity_avail:.1f}", self.relay_count_seen,
            f"{min_sep if min_sep != float('inf') else -1:.2f}",
            f"{100.0 * self.unsafe_ticks / self.total_ticks:.2f}",
        ])
        self._csv_file.flush()

    def destroy_node(self):
        avg_recovery = sum(self.recovery_times) / len(self.recovery_times) if self.recovery_times else 0.0
        self.get_logger().info(
            f"--- Final summary ---\n"
            f"PoIs surveyed: {sum(1 for p in self.pois.values() if p.surveyed)}/{len(self.pois)}\n"
            f"Relay reallocations: {self.relay_count_seen}\n"
            f"Average recovery time: {avg_recovery:.1f}s over {len(self.recovery_times)} disconnect(s)\n"
            f"Unsafe ticks (<{SAFETY_SEPARATION_M}m separation): "
            f"{100.0*self.unsafe_ticks/max(self.total_ticks,1):.2f}%"
        )
        self._csv_file.close()
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
