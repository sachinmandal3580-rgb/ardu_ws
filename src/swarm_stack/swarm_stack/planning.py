"""Pure geometry and routing helpers shared by the ROS nodes and tests."""

import heapq
import math


def distance(a, b):
    return math.dist(a, b)


def horizontal_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def relay_positions(gcs, destination, reliable_range_m, altitude_m):
    """Positions for a GCS-to-destination chain with bounded segment length."""
    length = horizontal_distance(gcs, destination)
    segments = max(1, math.ceil(length / reliable_range_m))
    return [
        (gcs[0] + (destination[0] - gcs[0]) * i / segments,
         gcs[1] + (destination[1] - gcs[1]) * i / segments,
         altitude_m)
        for i in range(1, segments)
    ]


def best_path(edges, source, target="GCS", minimum_pdr=0.05):
    """Return the highest-delivery-probability path and its PDR and latency."""
    adjacency = {}
    for edge in edges:
        if not edge.link_up or edge.pdr < minimum_pdr:
            continue
        for a, b in ((edge.node_a, edge.node_b), (edge.node_b, edge.node_a)):
            adjacency.setdefault(a, []).append((b, edge.pdr, edge.latency_ms))
    queue = [(0.0, 0.0, source, [source])]
    best = {}
    while queue:
        cost, latency, node, path = heapq.heappop(queue)
        if node in best and best[node] <= cost:
            continue
        best[node] = cost
        if node == target:
            return path, math.exp(-cost), latency
        for neighbor, pdr, delay in adjacency.get(node, []):
            if neighbor not in path:
                heapq.heappush(queue, (cost - math.log(pdr), latency + delay,
                                       neighbor, path + [neighbor]))
    return [], 0.0, float("inf")


def within_fence(point, center, radius_m, margin_m=0.0):
    return horizontal_distance(point, center) <= radius_m - margin_m


def safe_to_continue(battery_pct, point, home, reserve_pct, pct_per_m):
    return battery_pct > reserve_pct + horizontal_distance(point, home) * pct_per_m
