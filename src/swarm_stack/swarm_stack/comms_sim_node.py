#!/usr/bin/env python3
"""
comms_sim_node
--------------
Simulates a range-limited, lossy communication link between every UAV in the
swarm and a virtual Ground Control Station (GCS), since neither ArduPilot nor
Gazebo model RF propagation. Publishes a live connectivity graph on
/swarm/connectivity_graph at a fixed rate.

IMPORTANT — frame choice:
Each drone's /ap/pose/filtered and /ap/geopose/filtered are reported relative
to that drone's OWN EKF origin/home, not a shared world frame, so they are NOT
directly comparable across drones for distance calculations. There is also a
known upstream bug where /ap/geopose/filtered does not reflect the actual
global position set in the Gazebo world (ardupilot_gz#74).

This node instead subscribes to each drone's /<name>/odometry topic, which is
bridged from Gazebo (ros_gz_bridge) and reports pose in the *shared Gazebo
world frame* — the correct choice for computing real pairwise distances
between vehicles spawned at different offsets in the same world.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from swarm_interfaces.msg import ConnectivityGraph, ConnectivityEdge


class CommsSimNode(Node):
    def __init__(self):
        super().__init__("comms_sim_node")

        self.declare_parameter("uav_names", ["drone1", "drone2", "drone3", "drone4", "drone5"])
        self.declare_parameter("gcs_position", [-5.0, 0.0, 0.0])  # world-frame x, y, z
        self.declare_parameter("range_full_m", 100.0)   # below this: perfect link
        self.declare_parameter("range_max_m", 250.0)    # beyond this: link is down
        self.declare_parameter("base_latency_ms", 20.0)
        self.declare_parameter("publish_rate_hz", 5.0)

        self.uav_names = self.get_parameter("uav_names").value
        gcs_pos = self.get_parameter("gcs_position").value
        self.gcs_position = (gcs_pos[0], gcs_pos[1], gcs_pos[2])
        self.range_full_m = self.get_parameter("range_full_m").value
        self.range_max_m = self.get_parameter("range_max_m").value
        self.base_latency_ms = self.get_parameter("base_latency_ms").value
        rate = self.get_parameter("publish_rate_hz").value

        # latest known world-frame position of each UAV; None until first odom arrives
        self.positions = {name: None for name in self.uav_names}

        for name in self.uav_names:
            topic = f"/{name}/odometry"
            self.create_subscription(
                Odometry, topic,
                lambda msg, n=name: self._odom_cb(n, msg),
                10,
            )
            self.get_logger().info(f"Subscribed to {topic}")

        self.graph_pub = self.create_publisher(ConnectivityGraph, "/swarm/connectivity_graph", 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _odom_cb(self, name, msg: Odometry):
        p = msg.pose.pose.position
        self.positions[name] = (p.x, p.y, p.z)

    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _link_model(self, distance):
        """Return (pdr, latency_ms, link_up) for a given distance in metres."""
        if distance <= self.range_full_m:
            return 1.0, self.base_latency_ms, True
        if distance >= self.range_max_m:
            return 0.0, float("inf"), False
        span = self.range_max_m - self.range_full_m
        frac = (distance - self.range_full_m) / span
        pdr = max(0.0, 1.0 - frac)
        latency = self.base_latency_ms + frac * (200.0 - self.base_latency_ms)
        return pdr, latency, True

    def _tick(self):
        nodes = {"GCS": self.gcs_position}
        for name in self.uav_names:
            if self.positions[name] is not None:
                nodes[name] = self.positions[name]

        edges = []
        adjacency = {n: [] for n in nodes}
        for a in nodes:
            for b in nodes:
                if a >= b:
                    continue
                d = self._dist(nodes[a], nodes[b])
                pdr, latency, up = self._link_model(d)
                edges.append(ConnectivityEdge(
                    node_a=a, node_b=b, distance_m=float(d),
                    pdr=float(pdr), latency_ms=float(latency if up else -1.0),
                    link_up=up,
                ))
                if up:
                    adjacency[a].append(b)
                    adjacency[b].append(a)

        hop_counts = {name: self._bfs_hops(adjacency, "GCS", name) for name in self.uav_names}

        msg = ConnectivityGraph()
        msg.uav_names = list(self.uav_names)
        msg.hop_counts = [hop_counts[n] for n in self.uav_names]
        msg.edges = edges
        self.graph_pub.publish(msg)

    @staticmethod
    def _bfs_hops(adjacency, start, target):
        if start not in adjacency or target not in adjacency:
            return -1
        if start == target:
            return 0
        visited = {start}
        frontier = [start]
        hops = 0
        while frontier:
            hops += 1
            next_frontier = []
            for node in frontier:
                for neighbor in adjacency.get(node, []):
                    if neighbor == target:
                        return hops
                    if neighbor not in visited:
                        visited.add(neighbor)
                        next_frontier.append(neighbor)
            frontier = next_frontier
        return -1


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
