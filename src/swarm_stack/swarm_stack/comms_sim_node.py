#!/usr/bin/env python3
"""Logical lossy multi-hop network for the Gazebo fleet.

ROS/DDS remains the simulator's control bus. Survey images are only accepted at
/swarm/gcs/survey_image after simulated traversal of this network; this is not
an RF or physical mesh emulator.
"""

import random
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from .scenario import Scenario

from swarm_interfaces.msg import (CommunicationStats, ConnectivityEdge,
                                  ConnectivityGraph, FaultEvent, SurveyPayload,
                                  SurveyReceipt)
from .planning import best_path, distance


class CommsSimNode(Node):
    def __init__(self):
        super().__init__("comms_sim_node")
        self.declare_parameter("scenario_config_path", "")
        self.scenario = Scenario.load(self.get_parameter("scenario_config_path").value) if self.get_parameter("scenario_config_path").value else None
        self.declare_parameter("uav_names", [f"drone{i}" for i in range(1, 11)])
        self.declare_parameter("gcs_position", [-5.0, 0.0, 0.0])
        self.declare_parameter("range_full_m", 100.0)
        self.declare_parameter("range_max_m", 140.0)
        self.declare_parameter("base_latency_ms", 20.0)
        self.declare_parameter("minimum_link_pdr", 0.5)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("odom_timeout_s", 4.0)
        self.declare_parameter("random_seed", 7)
        self.names = list(self.get_parameter("uav_names").value)
        self.gcs = tuple(float(v) for v in self.get_parameter("gcs_position").value)
        self.full_range = float(self.get_parameter("range_full_m").value)
        self.max_range = float(self.get_parameter("range_max_m").value)
        self.base_latency = float(self.get_parameter("base_latency_ms").value)
        if self.scenario:
            self.gcs = tuple(self.scenario.gcs_position)
            self.full_range = self.scenario.comm_range_m - 1e-6
            self.max_range = self.scenario.comm_range_m
        self.minimum_link_pdr = float(self.get_parameter("minimum_link_pdr").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if not 0 < self.full_range < self.max_range or rate <= 0:
            raise ValueError("Invalid communication range or publish rate")
        self.rng = random.Random(int(self.get_parameter("random_seed").value))
        self.positions = {}
        self.failed = set()
        self.degraded_nodes = set()
        self.degraded_edges = set()
        self.last_graph = None
        self.pending = []
        self.sent = 0
        self.delivered = 0
        self.latency_total = 0.0
        for name in self.names:
            self.create_subscription(Odometry, f"/{name}/odometry",
                                     lambda msg, n=name: self._odom(n, msg), 10)
        self.create_subscription(FaultEvent, "/swarm/fault_events", self._fault, 10)
        self.create_subscription(SurveyPayload, "/swarm/outbound_survey", self._payload, 10)
        self.graph_pub = self.create_publisher(ConnectivityGraph, "/swarm/connectivity_graph", 10)
        self.stats_pub = self.create_publisher(CommunicationStats, "/swarm/communication_stats", 10)
        self.gcs_pub = self.create_publisher(SurveyPayload, "/swarm/gcs/survey_image", 10)
        self.receipt_pub = self.create_publisher(SurveyReceipt, "/swarm/survey_receipt", 10)
        self.create_timer(1.0 / rate, self._tick)

    def _odom(self, name, msg):
        p = msg.pose.pose.position
        self.positions[name] = ((p.x, p.y, p.z), time.monotonic())

    def _fault(self, msg):
        if msg.event_type == "uav_failure" and msg.target in self.names:
            self.failed.add(msg.target)
        elif msg.event_type == "link_degraded":
            if ":" in msg.target:
                self.degraded_edges.add(frozenset(msg.target.split(":")))
            elif msg.target in self.names:
                self.degraded_nodes.add(msg.target)
        elif msg.event_type == "link_restored":
            self.degraded_edges.discard(frozenset(msg.target.split(":")))
            self.degraded_nodes.discard(msg.target)

    def _link(self, a, b, length):
        if self.scenario:
            pdr, latency = (1.0, self.base_latency) if length <= self.max_range else (0.0, -1.0)
        elif length <= self.full_range:
            pdr, latency = 1.0, self.base_latency
        elif length >= self.max_range:
            pdr, latency = 0.0, -1.0
        else:
            fraction = (length - self.full_range) / (self.max_range - self.full_range)
            pdr = 1.0 - fraction
            latency = self.base_latency + 180.0 * fraction
        if a in self.degraded_nodes or b in self.degraded_nodes or frozenset((a, b)) in self.degraded_edges:
            pdr *= 0.1
            if pdr < self.minimum_link_pdr:
                latency = -1.0
        return pdr, latency

    def _tick(self):
        now = time.monotonic()
        nodes = {"GCS": self.gcs}
        nodes.update({n: p for n, (p, seen) in self.positions.items()
                      if n not in self.failed and now - seen < self.odom_timeout})
        edges = []
        for i, a in enumerate(nodes):
            for b in list(nodes)[i + 1:]:
                length = distance(nodes[a], nodes[b])
                pdr, latency = self._link(a, b, length)
                edges.append(ConnectivityEdge(node_a=a, node_b=b,
                                              distance_m=float(length), pdr=float(pdr),
                                              latency_ms=float(latency), link_up=pdr >= self.minimum_link_pdr))
        graph = ConnectivityGraph()
        graph.uav_names = self.names
        graph.edges = edges
        graph.hop_counts = [len(best_path(edges, n)[0]) - 1 if best_path(edges, n)[0] else -1
                            for n in self.names]
        self.last_graph = graph
        self.graph_pub.publish(graph)
        # Each live aircraft originates one small situational-data heartbeat.
        for n in self.names:
            if n in self.failed or n not in nodes:
                continue
            self.sent += 1
            _, pdr, latency = best_path(edges, n)
            if self.rng.random() < pdr:
                self.delivered += 1
                self.latency_total += latency
        ready, self.pending = [p for p in self.pending if p[0] <= now], [p for p in self.pending if p[0] > now]
        for _, payload, latency in ready:
            self.gcs_pub.publish(payload)
            self.receipt_pub.publish(SurveyReceipt(poi_id=payload.poi_id, uav_name=payload.uav_name,
                                                   delivered=True, latency_ms=float(latency)))
        self.stats_pub.publish(CommunicationStats(
            packets_sent=self.sent, packets_delivered=self.delivered,
            mean_delivered_latency_ms=float(self.latency_total / self.delivered if self.delivered else 0.0)))

    def _payload(self, msg):
        self.sent += 1
        _, pdr, latency = best_path(self.last_graph.edges if self.last_graph else [], msg.uav_name)
        if self.rng.random() < pdr:
            self.delivered += 1
            self.latency_total += latency
            self.pending.append((time.monotonic() + latency / 1000.0, msg, latency))
        else:
            self.receipt_pub.publish(SurveyReceipt(poi_id=msg.poi_id, uav_name=msg.uav_name,
                                                   delivered=False, latency_ms=-1.0))


def main(args=None):
    rclpy.init(args=args)
    node = CommsSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
