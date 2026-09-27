#!/usr/bin/env python3
"""Publish an RViz world-frame map of the live swarm mission."""

from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rosgraph_msgs.msg import Clock
from swarm_interfaces.msg import ConnectivityGraph, PoI, UAVStatus
from .scenario import Scenario


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class SwarmVisualizationNode(Node):
    def __init__(self):
        super().__init__("swarm_visualization_node")
        self.declare_parameter("gcs_position", [-5.0, 0.0, 0.0])
        self.declare_parameter("fence_radius_m", 400.0)
        self.declare_parameter("scenario_config_path", "")
        self.declare_parameter("uav_names", [f"drone{i}" for i in range(1, 11)])
        self.names = list(self.get_parameter("uav_names").value)
        scenario_path = self.get_parameter("scenario_config_path").value
        self.scenario = Scenario.load(scenario_path) if scenario_path else None
        self.gcs = (tuple(self.scenario.gcs_position) if self.scenario else
                    tuple(float(v) for v in self.get_parameter("gcs_position").value))
        self.fence_radius = float(self.get_parameter("fence_radius_m").value)
        self.uavs = {}
        self.pois = {}
        self.scheduled_pois = self.scenario.random_pois() if self.scenario else []
        self.sim_time = 0.0
        self.clock_epoch = None
        self.graph = None
        self.trails = {}
        self.trail_last_at = {}
        self.pad_by_name = (dict(zip(self.names, self.scenario.pads(len(self.names))))
                            if self.scenario else {})
        self.first_publish = True
        self.create_subscription(UAVStatus, "/swarm/uav_status", self._status, 10)
        self.create_subscription(PoI, "/swarm/poi_updates", self._poi, 10)
        self.create_subscription(Clock, "/drone1/clock", self._on_clock, 10)
        self.create_subscription(ConnectivityGraph, "/swarm/connectivity_graph", self._graph, 10)
        self.publisher = self.create_publisher(MarkerArray, "/swarm/map_markers", 10)
        self.create_timer(0.5, self._publish)

    def _status(self, msg):
        self.uavs[msg.name] = msg
        if not msg.position_valid:
            return
        point = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        trail = self.trails.setdefault(msg.name, deque(maxlen=250))
        now = time.monotonic()
        if not trail or (now - self.trail_last_at.get(msg.name, 0.0) >= 1.0 and
                         math.dist((point.x, point.y, point.z),
                                   (trail[-1].x, trail[-1].y, trail[-1].z)) >= 0.5):
            trail.append(point)
            self.trail_last_at[msg.name] = now

    def _poi(self, msg):
        self.pois[msg.id] = msg

    def _on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec / 1e9
        if self.clock_epoch is None:
            self.clock_epoch = now
        self.sim_time = max(0.0, now - self.clock_epoch)

    def _graph(self, msg):
        self.graph = msg

    def _marker(self, namespace, ident, kind, rgb, scale=(1.0, 1.0, 1.0)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = ident
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color = rgb
        return marker

    def _publish(self):
        markers = MarkerArray()
        if self.first_publish:
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.first_publish = False

        if self.scenario:
            arena = self.scenario.operational_area
            boundary = self._marker("arena", 0, Marker.LINE_STRIP,
                                    color(0.35, 0.75, 0.95), (2.0, 0.0, 0.0))
            x0, y0, x1, y1 = arena
            boundary.points = [Point(x=x, y=y, z=0.1) for x, y in
                               ((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0))]
            markers.markers.append(boundary)
            base = self.scenario.base_area
            launch = self._marker("launch_area", 0, Marker.LINE_STRIP,
                                  color(0.2, 0.85, 0.75), (1.5, 0.0, 0.0))
            bx0, by0, bx1, by1 = base
            launch.points = [Point(x=x, y=y, z=0.1) for x, y in
                             ((bx0,by0),(bx1,by0),(bx1,by1),(bx0,by1),(bx0,by0))]
            markers.markers.append(launch)
            for i, pad in enumerate(self.scenario.pads(len(self.names)), start=1):
                marker = self._marker("launch_pads", i, Marker.SPHERE,
                                      color(0.25, 0.75, 0.75, 0.8), (5.0, 5.0, 0.4))
                marker.pose.position = Point(x=pad[0], y=pad[1], z=0.2)
                markers.markers.append(marker)
            caption = self._marker("arena_label", 0, Marker.TEXT_VIEW_FACING,
                                   color(0.6, 0.85, 1.0), (0.0, 0.0, 20.0))
            caption.pose.position = Point(x=(x0+x1)/2, y=y1+25.0, z=1.0)
            caption.text = f"OPERATIONAL AREA  {arena[2]-arena[0]:g} x {arena[3]-arena[1]:g} m"
            markers.markers.append(caption)
        else:
            fence = self._marker("safety", 0, Marker.LINE_STRIP,
                                 color(0.9, 0.55, 0.1, 0.5), (1.5, 0.0, 0.0))
            for i in range(73):
                angle = 2.0 * math.pi * i / 72
                fence.points.append(Point(x=self.fence_radius * math.cos(angle),
                                          y=self.fence_radius * math.sin(angle), z=0.1))
            markers.markers.append(fence)

        gcs = self._marker("gcs", 0, Marker.CUBE, color(0.15, 0.4, 1.0), (12.0, 12.0, 3.0))
        gcs.pose.position = Point(x=self.gcs[0], y=self.gcs[1], z=1.5)
        markers.markers.append(gcs)
        gcs_label = self._marker("labels", 0, Marker.TEXT_VIEW_FACING,
                                 color(0.3, 0.55, 1.0), (0.0, 0.0, 18.0))
        gcs_label.pose.position = Point(x=self.gcs[0], y=self.gcs[1], z=6.0)
        gcs_label.text = "GCS"
        markers.markers.append(gcs_label)

        # The seeded t=0 PoI appears before the mission manager finishes booting.
        # Later scheduled PoIs follow Gazebo time; live updates override their state.
        visible_pois = {
            poi["id"]: (poi["x"], poi["y"], poi["priority"], False)
            for poi in self.scheduled_pois if poi["spawn_s"] <= self.sim_time
        }
        for poi in self.pois.values():
            visible_pois[poi.id] = (poi.position.x, poi.position.y,
                                    poi.priority, poi.surveyed)
        for i, (poi_id, (x, y, priority, surveyed)) in enumerate(sorted(visible_pois.items()), start=1):
            shade = color(0.15, 0.85, 0.25) if surveyed else color(1.0, 0.2, 0.15)
            site = self._marker("pois", i, Marker.CYLINDER, shade, (14.0, 14.0, 1.0))
            site.pose.position = Point(x=x, y=y, z=0.5)
            markers.markers.append(site)
            label = self._marker("poi_labels", i, Marker.TEXT_VIEW_FACING,
                                 shade, (0.0, 0.0, 17.0))
            label.pose.position = Point(x=x, y=y, z=5.0)
            label.text = f"{poi_id}  P{priority}" + ("  DONE" if surveyed else "")
            markers.markers.append(label)

        palette = {"surveyor": color(0.1, 0.85, 0.95), "relay": color(1.0, 0.75, 0.15),
                   "idle": color(0.7, 0.7, 0.75), "rtl": color(0.9, 0.35, 0.8),
                   "failed": color(0.9, 0.1, 0.1)}
        positions = {"GCS": Point(x=self.gcs[0], y=self.gcs[1], z=self.gcs[2])}
        # Show every planned aircraft at its pad before telemetry arrives.
        # The scaled mesh is a map icon; Gazebo retains the physical-size model.
        for i, name in enumerate(self.names, start=1):
            status = self.uavs.get(name)
            valid = status is not None and status.position_valid
            if valid:
                point = status.position
                shade = palette.get(status.role, palette["idle"])
                positions[name] = point
            else:
                pad = self.pad_by_name.get(name, (self.gcs[0], self.gcs[1], 0.0))
                point = Point(x=pad[0], y=pad[1], z=0.5)
                shade = color(0.48, 0.56, 0.67)
            halo = self._marker("aircraft_halos", i, Marker.CYLINDER,
                                color(shade.r, shade.g, shade.b, 0.3), (15.0, 15.0, 0.3))
            halo.pose.position = Point(x=point.x, y=point.y, z=max(0.2, point.z - 1.0))
            markers.markers.append(halo)
            drone = self._marker("aircraft_meshes", i, Marker.MESH_RESOURCE, shade,
                                 (20.0, 20.0, 20.0))
            drone.mesh_resource = "package://ardupilot_gazebo/models/iris_with_standoffs/meshes/iris.dae"
            drone.mesh_use_embedded_materials = False
            drone.pose.position = point
            markers.markers.append(drone)
            label = self._marker("aircraft_labels", i, Marker.TEXT_VIEW_FACING,
                                 shade, (0.0, 0.0, 15.0))
            label.pose.position = Point(x=point.x, y=point.y, z=point.z + 12.0)
            if valid:
                task = f" → {status.current_task_id}" if status.current_task_id else ""
                state = status.role if status.flight_state == "ready" else status.flight_state
                label.text = f"{name}  {state}{task}  {status.battery_pct:.0f}%"
            else:
                label.text = f"{name}  WAITING FOR TELEMETRY"
            markers.markers.append(label)
            if valid and len(self.trails.get(name, ())) > 1:
                trail = self._marker("trails", i, Marker.LINE_STRIP,
                                     color(shade.r, shade.g, shade.b, 0.75), (1.5, 0.0, 0.0))
                trail.points = list(self.trails[name])
                markers.markers.append(trail)

        if self.graph is not None:
            links = self._marker("live_links", 0, Marker.LINE_LIST,
                                 color(0.2, 0.95, 0.25, 0.45), (1.5, 0.0, 0.0))
            for edge in self.graph.edges:
                if edge.link_up and edge.node_a in positions and edge.node_b in positions:
                    links.points.extend((positions[edge.node_a], positions[edge.node_b]))
            if links.points:
                markers.markers.append(links)
        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmVisualizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
