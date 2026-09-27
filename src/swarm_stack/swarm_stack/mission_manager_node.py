#!/usr/bin/env python3
"""Centralized, communication-aware mission manager for the configured Copter SITL fleet."""

import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image
from rosgraph_msgs.msg import Clock
from .scenario import Scenario, next_waypoint, separation_safe_step
import yaml

from pymavlink import mavutil
from swarm_interfaces.msg import (ConnectivityGraph, FaultEvent, PoI,
                                  SurveyPayload, SurveyReceipt, UAVStatus)
from .planning import distance, horizontal_distance, relay_positions, safe_to_continue, within_fence

GUIDED_MODE = 4
RTL_MODE = 6
EARTH_RADIUS_M = 6378137.0


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")
        defaults = {
            "uav_names": [f"drone{i}" for i in range(1, 11)],
            "poi_config_path": "", "scenario_config_path": "", "control_rate_hz": 2.0,
            "gcs_position": [-5.0, 0.0, 0.0], "origin_lat": -35.363262,
            "origin_lon": 149.165237, "survey_altitude_m": 15.0,
            "altitude_lane_m": 5.0, "reliable_range_m": 80.0,
            "survey_radius_m": 5.0, "relay_arrival_m": 7.0,
            "fence_center": [0.0, 0.0, 0.0], "fence_radius_m": 400.0,
            "fence_margin_m": 10.0, "battery_reserve_pct": 25.0,
            "battery_pct_per_m": 0.04, "image_timeout_s": 4.0,
            "mavlink_udp_base_port": 14550,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        self.scenario = Scenario.load(param("scenario_config_path")) if param("scenario_config_path") else None
        self.sim_time = 0.0
        self.clock_epoch = None
        self.finished = False
        self.desired_goals = {}
        self.names = list(param("uav_names"))
        self.flight_started = {n: None for n in self.names}
        self.landed_at = {n: None for n in self.names}
        self.recharging = {n: False for n in self.names}
        self.recharge_jobs = {}
        self.last_recharge_try = {n: -10.0 for n in self.names}
        self.velocity = {n: (0.0, 0.0, 0.0) for n in self.names}
        self.gcs = tuple(float(v) for v in param("gcs_position"))
        self.origin_lat, self.origin_lon = float(param("origin_lat")), float(param("origin_lon"))
        self.survey_alt = float(param("survey_altitude_m"))
        self.lane = float(param("altitude_lane_m"))
        self.reliable_range = float(param("reliable_range_m"))
        self.survey_radius = float(param("survey_radius_m"))
        self.relay_arrival = float(param("relay_arrival_m"))
        self.fence_center = tuple(float(v) for v in param("fence_center"))
        self.fence_radius = float(param("fence_radius_m"))
        self.fence_margin = float(param("fence_margin_m"))
        self.battery_reserve = float(param("battery_reserve_pct"))
        self.pct_per_m = float(param("battery_pct_per_m"))
        self.image_timeout = float(param("image_timeout_s"))
        self.pois = self._load_pois(param("poi_config_path"))
        if self.scenario:
            self.gcs = tuple(self.scenario.gcs_position)
            self.survey_alt = self.scenario.cruise_altitude_m
            self.reliable_range = self.scenario.relay_spacing_m
            self.relay_arrival = 3.0
            self.pois = [dict(p, surveyed=False, surveyed_by="") for p in self.scenario.random_pois()]
            required = self.scenario.worst_case_aircraft()
            if len(self.names) < required:
                self.get_logger().warn(f"Sample area needs up to {required} UAVs for a far-corner relay chain; "
                                       f"this SITL fleet has {len(self.names)}. Some PoIs are unreachable.")
            else:
                self.get_logger().info(f"Sample area relay chain needs up to {required} UAVs; "
                                       f"this SITL fleet has {len(self.names)}.")
        for poi in self.pois:
            poi.setdefault("spawn_s", 0.0)
            poi.update(detected_s=-1.0, reported_s=-1.0)
        self.position = {n: None for n in self.names}
        self.last_position_at = {n: 0.0 for n in self.names}
        self.home = {n: None for n in self.names}
        self.battery = {n: None for n in self.names}
        self.images = {}
        self.hops = {n: -1 for n in self.names}
        self.role = {n: "idle" for n in self.names}
        self.stage = {n: "mode" for n in self.names}
        self.ascent_started = {}
        self.last_command = {}
        self.mav_state = {n: {"heartbeat": 0.0, "mode": None, "armed": False} for n in self.names}
        self.gps_origin = {n: None for n in self.names}
        self.assigned = {n: None for n in self.names}
        self.last_goal = {}
        self.last_image_sent = {}
        self.active_poi = None
        self.active_surveyor = None
        self.active_relays = []
        self.active_tasks = {}
        self.relay_reallocations = 0
        self.network_recoveries = 0
        self.last_network_recovery_s = -1.0
        self.recovery_started = None
        self.last_graph_at = 0.0
        self.blocked_nodes = set()
        self.degraded_edge_blocks = {}
        self.last_warning_at = 0.0
        self.min_separation = 2.5
        base_port = int(param("mavlink_udp_base_port"))
        self.links = {name: mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{base_port + 10 * index}", source_system=220 + index)
            for index, name in enumerate(self.names)}
        self.image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
        self.camera_subscriptions = {}
        for name in self.names:
            self.create_subscription(Odometry, f"/{name}/odometry",
                                     lambda msg, n=name: self._odom(n, msg), 10)
            self.create_subscription(BatteryState, f"/{name}/battery",
                                     lambda msg, n=name: self._battery(n, msg), 10)
        self.create_subscription(Clock, "/drone1/clock", self._on_sim_clock, 10)
        self.create_subscription(ConnectivityGraph, "/swarm/connectivity_graph", self._graph, 10)
        self.create_subscription(FaultEvent, "/swarm/fault_events", self._fault, 10)
        self.create_subscription(SurveyReceipt, "/swarm/survey_receipt", self._receipt, 10)
        self.status_pub = self.create_publisher(UAVStatus, "/swarm/uav_status", 10)
        self.poi_pub = self.create_publisher(PoI, "/swarm/poi_updates", 10)
        self.survey_pub = self.create_publisher(SurveyPayload, "/swarm/outbound_survey", 10)
        self.create_timer(1.0 / float(param("control_rate_hz")), self._tick)
        self.get_logger().info(f"Managing {len(self.names)} UAVs and {len(self.pois)} PoIs")

    def _on_sim_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec / 1e9
        if self.clock_epoch is None:
            self.clock_epoch = now
        self.sim_time = max(0.0, now - self.clock_epoch)

    def _allowed(self, point):
        return self.scenario.allowed(point) if self.scenario else within_fence(
            point, self.fence_center, self.fence_radius, self.fence_margin)

    def _return_budget(self, name, point=None):
        pos = point or self.position[name]
        home = self.home[name]
        if not pos or not home:
            return 0.0
        gate = (75.0, home[1], 70.0)
        length = (horizontal_distance(pos, gate) + horizontal_distance(gate, home)
                  if pos[0] > 75.0 else horizontal_distance(pos, home))
        return length / self.scenario.max_speed_mps + 60.0 + self.scenario.return_reserve_s

    def _load_pois(self, path):
        if not path:
            raise ValueError("poi_config_path is required")
        with open(path, encoding="utf-8") as stream:
            entries = yaml.safe_load(stream).get("pois", [])
        return [{"id": str(p["id"]), "x": float(p["x"]), "y": float(p["y"]),
                 "z": float(p.get("z", 0.0)), "priority": int(p.get("priority", 1)),
                 "surveyed": False, "surveyed_by": ""} for p in entries]

    def _odom(self, name, msg):
        p = msg.pose.pose.position
        self.position[name] = (p.x, p.y, p.z)
        self.last_position_at[name] = time.monotonic()
        v = msg.twist.twist.linear
        self.velocity[name] = (v.x, v.y, v.z)
        if self.home[name] is None:
            self.home[name] = self.position[name]

    def _battery(self, name, msg):
        if math.isfinite(msg.percentage) and 0.0 <= msg.percentage <= 1.0:
            self.battery[name] = msg.percentage * 100.0
        elif math.isfinite(msg.percentage) and 1.0 < msg.percentage <= 100.0:
            self.battery[name] = msg.percentage
        elif msg.design_capacity > 0 and msg.charge >= 0:
            self.battery[name] = min(100.0, 100.0 * msg.charge / msg.design_capacity)

    def _sync_camera_subscriptions(self, surveyors):
        # Gazebo's lazy bridge only sends images for actively subscribed UAVs.
        needed = set(surveyors)
        for name, subscription in list(self.camera_subscriptions.items()):
            if name not in needed:
                self.destroy_subscription(subscription)
                self.camera_subscriptions.pop(name)
                self.images.pop(name, None)
        for name in needed - self.camera_subscriptions.keys():
            self.camera_subscriptions[name] = self.create_subscription(
                Image, f"/{name}/camera/image",
                lambda msg, n=name: self._image(n, msg), self.image_qos)

    def _image(self, name, msg):
        if msg.data:
            self.images[name] = (msg, time.monotonic())

    def _graph(self, msg):
        self.last_graph_at = time.monotonic()
        for name, hops in zip(msg.uav_names, msg.hop_counts):
            if name in self.hops:
                self.hops[name] = hops

    def _fault(self, msg):
        if msg.event_type == "uav_failure" and msg.target in self.role:
            self.recovery_started = time.monotonic()
            name = msg.target
            if self.scenario:
                self._rtl(name, "simulated radio failure")
                self.role[name] = "failed"
            else:
                self.role[name] = "failed"
                self.stage[name] = "failed"
                self.assigned[name] = None
                self._send_mode(name, "RTL")
            self.get_logger().warn(f"Simulated failure: {name} removed from service; RTL requested")
        elif msg.event_type == "link_degraded":
            self.recovery_started = time.monotonic()
            if ":" in msg.target:
                endpoints = [n for n in msg.target.split(":") if n in self.names]
                active = [n for n in endpoints if n in self.active_relays]
                if active:
                    # Replace only one aircraft on the degraded edge, preferably
                    # the inner relay which an idle UAV can reach sooner.
                    blocked = min(active, key=self.active_relays.index)
                elif any(task["surveyor"] in endpoints for task in self.active_tasks.values()):
                    blocked = next(task["surveyor"] for task in self.active_tasks.values()
                                   if task["surveyor"] in endpoints)
                else:
                    blocked = endpoints[0] if endpoints else None
                if blocked:
                    self.degraded_edge_blocks[msg.target] = blocked
                    self.get_logger().warn(f"Replacing {blocked} for degraded link {msg.target}")
            elif msg.target in self.names:
                self.blocked_nodes.add(msg.target)
                self.get_logger().warn(f"Avoiding degraded UAV link: {msg.target}")
        elif msg.event_type == "link_restored":
            self.degraded_edge_blocks.pop(msg.target, None)
            self.blocked_nodes.discard(msg.target)
        elif msg.event_type == "new_poi":
            if any(p["id"] == msg.target for p in self.pois):
                return
            point = (float(msg.x), float(msg.y), float(msg.z))
            if not (self.scenario.in_arena(point) if self.scenario else self._allowed(point)):
                self.get_logger().error("New PoI outside geofence; rejected")
                return
            self.pois.append({"id": msg.target, "x": msg.x, "y": msg.y, "z": msg.z,
                              "priority": max(1, msg.priority), "surveyed": False, "surveyed_by": "",
                              "spawn_s": self.sim_time, "detected_s": -1.0, "reported_s": -1.0})
            self.get_logger().info(f"Added emergency PoI {msg.target} (priority {msg.priority})")

    def _receipt(self, msg):
        if not msg.delivered:
            return
        for poi in self.pois:
            if poi["id"] == msg.poi_id and not poi["surveyed"]:
                poi["surveyed"] = True
                poi["surveyed_by"] = msg.uav_name
                poi["reported_s"] = self.sim_time
                if self.scenario and poi["detected_s"] >= 0 and self.sim_time-poi["detected_s"] > self.scenario.report_deadline_s:
                    self.get_logger().error(f"Report deadline missed for {poi['id']}")
                self.get_logger().info(f"GCS received image for {poi['id']} from {msg.uav_name}")
                break

    def _lane_alt(self, name):
        return self.survey_alt if self.scenario else self.survey_alt + self.names.index(name) * self.lane

    def _poll_mavlink(self, name):
        link = self.links[name]
        for _ in range(100):
            msg = link.recv_match(blocking=False)
            if msg is None:
                break
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.get_srcSystem() == self.names.index(name) + 1:
                state = self.mav_state[name]
                state["heartbeat"] = time.monotonic()
                state["mode"] = msg.custom_mode
                state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if state["armed"] and self.flight_started[name] is None:
                    self.flight_started[name] = self.sim_time
            elif kind == "SYS_STATUS" and 0 <= msg.battery_remaining <= 100 and self.battery[name] is None:
                self.battery[name] = float(msg.battery_remaining)
            elif kind == "GLOBAL_POSITION_INT" and self.position[name] is not None and self.gps_origin[name] is None:
                world = self.position[name]
                lat = msg.lat / 1e7 - math.degrees(world[1] / EARTH_RADIUS_M)
                lon = msg.lon / 1e7 - math.degrees(world[0] / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
                self.gps_origin[name] = (lat, lon)
                self.get_logger().info(f"{name}: calibrated world-to-GPS origin {lat:.7f}, {lon:.7f}")
            elif kind == "COMMAND_ACK" and msg.result not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED, mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                self.get_logger().warn(f"{name}: MAVLink command {msg.command} rejected ({msg.result})")
            elif kind == "STATUSTEXT" and msg.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                self.get_logger().warn(f"{name}: {msg.text}")

    def _send_mode(self, name, mode):
        if self._command_due(name, "mode_" + mode, 3.0):
            self.links[name].set_mode_apm(mode)

    def _command_due(self, name, kind, interval):
        now = time.monotonic()
        key = (name, kind)
        if now - self.last_command.get(key, 0.0) < interval:
            return False
        self.last_command[key] = now
        return True

    def _position_fresh(self, name):
        return self.position[name] is not None and time.monotonic() - self.last_position_at[name] < 4.0

    def _launch_blocked(self, name):
        if self.scenario:
            # Separated vertical launch columns may climb concurrently. A
            # vehicle still failing pre-arm on another pad must not block them.
            home = self.home[name]
            if home is None:
                return True
            clearance = self.scenario.min_separation_m + 2.0
            planned = dict(zip(self.names, self.scenario.pads(len(self.names))))
            for other in self.names:
                if other == name:
                    continue
                point = self.position[other] or self.home[other] or planned[other]
                if (horizontal_distance(home, point) < clearance and
                        point[2] < home[2] + self._lane_alt(name) + clearance):
                    return True
            return False
        earlier = self.names[:self.names.index(name)]
        return any(self.stage[other] in ("mode", "arm", "takeoff") or
                   (self.stage[other] == "ascending" and self.position[other] is not None and
                    self.position[other][2] < self.home[other][2] + 8.0)
                   for other in earlier)

    def _advance_flight(self, name):
        state = self.mav_state[name]
        if self.stage[name] in ("failed", "rtl", "ready", "returning", "landing", "landed") or not self._position_fresh(name) or self.battery[name] is None:
            return
        if self.gps_origin[name] is None or time.monotonic() - state["heartbeat"] > 4.0:
            return
        if self.stage[name] == "mode":
            if state["mode"] == GUIDED_MODE:
                self.stage[name] = "arm"
                self.get_logger().info(f"{name}: Guided confirmed")
            else:
                self._send_mode(name, "GUIDED")
        elif self.stage[name] == "arm":
            # Arm as soon as this vehicle's own launch column is clear.
            if self._launch_blocked(name):
                return
            if state["armed"]:
                self.stage[name] = "takeoff"
                self.get_logger().info(f"{name}: armed confirmed")
            elif self._command_due(name, "arm", 3.0):
                self.links[name].arducopter_arm()
        elif self.stage[name] in ("takeoff", "ascending"):
            altitude = self.position[name][2] - self.home[name][2]
            if not state["armed"] and altitude < 2.0:
                self.stage[name] = "arm"
                self.get_logger().warn(f"{name}: disarmed before takeoff; rearming")
            elif altitude >= self._lane_alt(name) - 2.0:
                self.stage[name] = "ready"
                self.get_logger().info(f"{name}: airborne and ready")
            elif self.stage[name] == "takeoff" or time.monotonic() - self.ascent_started.get(name, 0.0) > 45.0:
                if self.stage[name] == "takeoff" and self._launch_blocked(name):
                    return
                if self._command_due(name, "takeoff", 5.0):
                    link = self.links[name]
                    link.mav.command_long_send(link.target_system, link.target_component,
                                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                               0, 0, 0, 0, 0, 0, float(self._lane_alt(name)))
                    self.ascent_started[name] = time.monotonic()
                    self.stage[name] = "ascending"
                    self.get_logger().info(f"{name}: takeoff requested to {self._lane_alt(name):.1f} m")

    def _goal(self, name, target, dispatch=False):
        if self.stage[name] not in ("ready", "returning") or not self._position_fresh(name) or not self._allowed(target):
            return
        if self.scenario and not dispatch:
            self.desired_goals[name] = target
            return
        now = time.monotonic()
        previous = self.last_goal.get(name)
        if previous and distance(previous[0], target) < 1.0 and now - previous[1] < (0.4 if self.scenario else 5.0):
            return
        origin = self.gps_origin[name]
        if origin is None:
            return
        lat = origin[0] + math.degrees(target[1] / EARTH_RADIUS_M)
        lon = origin[1] + math.degrees(target[0] / (EARTH_RADIUS_M * math.cos(math.radians(origin[0]))))
        link = self.links[name]
        mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        link.mav.set_position_target_global_int_send(
            0, link.target_system, link.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
            int(lat * 1e7), int(lon * 1e7), float(target[2] - self.home[name][2]),
            0, 0, 0, 0, 0, 0, 0, 0)
        self.last_goal[name] = (target, now)

    def _rtl(self, name, reason):
        if self.role[name] in ("rtl", "failed"):
            return
        self.get_logger().warn(f"{name}: RTL ({reason})")
        self.role[name] = "rtl"
        self.stage[name] = "returning" if self.scenario else "rtl"
        self.assigned[name] = None
        if not self.scenario:
            self._send_mode(name, "RTL")

    def _set_recharge(self, name, enable):
        """Toggle Gazebo's physical battery charger without blocking ROS callbacks."""
        job = self.recharge_jobs.get(name)
        if job is not None:
            process, requested = job
            result = process.poll()
            if result is None:
                return False
            del self.recharge_jobs[name]
            if result == 0:
                self.recharging[name] = requested
                self.get_logger().info(f"{name}: pad charging {'started' if requested else 'stopped'}")
            else:
                self._warn(f"{name}: Gazebo battery charger request failed; retrying")
        if self.recharging[name] == enable:
            return True
        if time.monotonic() - self.last_recharge_try[name] < 5.0:
            return False
        self.last_recharge_try[name] = time.monotonic()
        service = f"/model/{name}/battery/lipo_3500mAh/recharge/"
        service += "start" if enable else "stop"
        try:
            process = subprocess.Popen(
                ["gz", "service", "-s", service, "--reqtype", "gz.msgs.Boolean",
                 "--reptype", "gz.msgs.Empty", "--req", "data:true", "--timeout", "1000"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self._warn(f"{name}: charger command unavailable: {exc}")
            return False
        self.recharge_jobs[name] = (process, enable)
        return False

    def _unavailable(self, name):
        return name in self.blocked_nodes or name in self.degraded_edge_blocks.values()

    def _eligible(self, name, target):
        if self.stage[name] != "ready" or self.role[name] in ("failed", "rtl") or self._unavailable(name):
            return False
        if not self._position_fresh(name) or self.home[name] is None or self.battery[name] is None or self.gps_origin[name] is None:
            return False
        if self.scenario:
            used = self.sim_time - (self.flight_started[name] or 0.0)
            needed = horizontal_distance(self.position[name], target) / self.scenario.max_speed_mps + self._return_budget(name, target)
            if used + needed >= self.scenario.uav_endurance_s or self.sim_time + needed >= self.scenario.mission_duration_s:
                return False
            return self.battery[name] * self.scenario.uav_endurance_s / 100.0 > needed + 60.0
        travel = horizontal_distance(self.position[name], target) + horizontal_distance(target, self.home[name])
        return self.battery[name] > self.battery_reserve + self.pct_per_m * travel

    def _plan(self):
        remaining = [p for p in self.pois if not p["surveyed"] and p["spawn_s"] <= self.sim_time]
        if not remaining:
            self.active_tasks = {}
            self.active_poi, self.active_surveyor, self.active_relays = None, None, []
            self._sync_camera_subscriptions(())
            waiting_for_spawn = any(not p["surveyed"] for p in self.pois)
            for name in self.names:
                if self.stage[name] == "ready":
                    self.assigned[name] = None
                    if waiting_for_spawn:
                        self.role[name] = "idle"
                    else:
                        self._rtl(name, "mission complete")
            return
        if self.scenario:
            remaining.sort(key=lambda p: (p["priority"] < 10,
                                          p["id"] not in self.active_tasks,
                                          p["detected_s"] < 0,
                                          -len(self.scenario.chain((p["x"], p["y"], p["z"]))),
                                          -p["priority"], p["spawn_s"]))
        else:
            remaining.sort(key=lambda p: (p["id"] not in self.active_tasks,
                                          p["detected_s"] < 0, -p["priority"], p["id"]))
        free = {name for name in self.names if self.stage[name] == "ready"
                and self._position_fresh(name) and not self._unavailable(name)
                and self.role[name] not in ("failed", "rtl")}
        relay_targets = {}
        tasks = {}
        for poi in remaining:
            ground = (poi["x"], poi["y"], poi["z"])
            if not self._allowed(ground):
                self._warn(f"PoI {poi['id']} outside geofence")
                continue
            chain = self.scenario.chain(ground) if self.scenario else None
            stations = (chain[:-1] if chain else
                        relay_positions(self.gcs, ground, self.reliable_range, self.survey_alt))
            survey_target = chain[-1] if chain else (ground[0], ground[1], self.survey_alt)
            trial_free = free.copy()
            trial_targets = relay_targets.copy()
            trial_relays = []
            incumbent = self.active_tasks.get(poi["id"], {})
            for index, station in enumerate(stations):
                shared = next((name for name, target in trial_targets.items()
                               if distance(target, station) <= 19.0), None)
                if shared:
                    trial_relays.append(shared)
                    continue
                if self.scenario and (any(distance(target, station) < self.scenario.min_separation_m + 0.1
                                          for target in trial_targets.values()) or
                                      any(distance(task['survey_target'], station) < self.scenario.min_separation_m + 0.1
                                          for task in tasks.values())):
                    break
                candidates = [name for name in trial_free if self._eligible(name, station)]
                if not candidates:
                    break
                previous = (incumbent.get("relays", [])[index]
                            if index < len(incumbent.get("relays", [])) else None)
                relay = previous if previous in candidates else min(
                    candidates, key=lambda name: distance(self.position[name], station))
                trial_free.remove(relay)
                trial_targets[relay] = station
                trial_relays.append(relay)
            if len(trial_relays) != len(stations):
                continue
            if self.scenario and (any(distance(target, survey_target) < self.scenario.min_separation_m + 0.1
                                      for target in trial_targets.values()) or
                                  any(distance(task['survey_target'], survey_target) < self.scenario.min_separation_m + 0.1
                                      for task in tasks.values())):
                continue
            surveyors = [name for name in trial_free if self._eligible(name, ground)]
            if not surveyors:
                continue
            previous = incumbent.get("surveyor")
            surveyor = previous if previous in surveyors else min(
                surveyors, key=lambda name: horizontal_distance(self.position[name], ground))
            trial_free.remove(surveyor)
            free, relay_targets = trial_free, trial_targets
            tasks[poi["id"]] = {"poi": poi, "surveyor": surveyor,
                                "relays": trial_relays, "stations": stations,
                                "ground": ground, "survey_target": survey_target}
        if not tasks:
            ready = sum(self.stage[name] == "ready" for name in self.names)
            if not ready:
                armed = sum(self.mav_state[name]["armed"] for name in self.names)
                self._warn(f"Waiting for takeoff: sim {self.sim_time:.1f}s, "
                           f"armed {armed}/{len(self.names)}, ready {ready}/{len(self.names)}; "
                           "see per-UAV pre-arm messages")
            else:
                self._warn(f"PoIs queued: {ready}/{len(self.names)} UAVs ready; "
                           "no team meets relay, energy and separation requirements yet")
            return
        old_relays = {key: value["relays"] for key, value in self.active_tasks.items()}
        new_relays = {key: value["relays"] for key, value in tasks.items()}
        if old_relays and old_relays != new_relays:
            self.relay_reallocations += 1
            self.get_logger().info(f"Relay reallocation #{self.relay_reallocations}")
        for poi_id, task in tasks.items():
            if poi_id not in self.active_tasks:
                self.get_logger().info(f"Assigned {task['surveyor']} to {poi_id}")
        self.active_tasks = tasks
        self.active_poi = next(iter(tasks))
        self.active_surveyor = tasks[self.active_poi]["surveyor"]
        self.active_relays = list(relay_targets)
        self._sync_camera_subscriptions(task["surveyor"] for task in tasks.values())
        assignments = {task["surveyor"]: poi_id for poi_id, task in tasks.items()}
        for name in self.names:
            if self.stage[name] == "ready":
                self.role[name] = ("relay" if name in relay_targets else
                                   "surveyor" if name in assignments else "idle")
                self.assigned[name] = assignments.get(name)
        for task in tasks.values():
            poi, surveyor = task["poi"], task["surveyor"]
            chain_ready = True
            for relay in task["relays"]:
                target = relay_targets[relay]
                target = (target[0], target[1], self._lane_alt(relay))
                if chain_ready:
                    self._goal(relay, target)
                chain_ready = (chain_ready and
                               distance(self.position[relay], target) <= self.relay_arrival and
                               self.hops[relay] != -1)
            if not chain_ready:
                pos = self.position[surveyor]
                if pos is not None:
                    self._goal(surveyor, (pos[0], pos[1], self._lane_alt(surveyor)))
                continue
            if self.recovery_started is not None and self.hops[surveyor] != -1:
                self.last_network_recovery_s = time.monotonic() - self.recovery_started
                self.network_recoveries += 1
                self.recovery_started = None
                self.get_logger().info(f"Relay network recovered in {self.last_network_recovery_s:.1f} s")
            ground = (poi["x"], poi["y"], poi["z"])
            self._goal(surveyor, (poi["x"], poi["y"], self._lane_alt(surveyor)))
            if horizontal_distance(self.position[surveyor], ground) <= self.survey_radius:
                if poi["detected_s"] < 0:
                    poi["detected_s"] = self.sim_time
                image = self.images.get(surveyor)
                now = time.monotonic()
                if image and now - image[1] < 3.0 and now - self.last_image_sent.get(poi["id"], 0.0) >= self.image_timeout:
                    self.survey_pub.publish(SurveyPayload(poi_id=poi["id"], uav_name=surveyor, image=image[0]))
                    self.last_image_sent[poi["id"]] = now
                    self.get_logger().info(f"Sending image for {poi['id']} via simulated mesh")

    def _warn(self, message):
        if time.monotonic() - self.last_warning_at > 5.0:
            self.get_logger().warn(message)
            self.last_warning_at = time.monotonic()

    def _dispatch_sample_goals(self):
        # Bounded waypoints and a projected separation guard. Flight dynamics
        # still require SITL validation; measured violations are never suppressed.
        proposed = {}
        for name in self.names:
            if self.stage[name] not in ("ready", "returning") or not self._position_fresh(name):
                continue
            pos = self.position[name]
            destination = self.desired_goals.get(name, pos)
            target = next_waypoint(pos, destination, self.scenario)
            # Pass above deployed relays before descending onto a station.
            # Routing every aircraft through the gate at 40 m blocks the gate
            # once its first relay is in place.
            if (self.stage[name] == "ready" and self.role[name] in ("relay", "surveyor")
                    and horizontal_distance(pos, destination) > 25.0):
                transit_alt = min(self.scenario.max_altitude_m, self.survey_alt + 30.0)
                if pos[2] < transit_alt - 2.0:
                    target = (pos[0], pos[1], transit_alt)
                else:
                    target = (target[0], target[1], transit_alt)
            delta = tuple(t-p for t,p in zip(target,pos))
            length = math.sqrt(sum(v*v for v in delta))
            ratio = min(1.0, 8.0 / max(length, 1e-6))
            chosen = pos
            for angle in (0, 35, -35, 70, -70):
                a = math.radians(angle)
                step = (delta[0]*math.cos(a)-delta[1]*math.sin(a), delta[0]*math.sin(a)+delta[1]*math.cos(a),delta[2])
                q = tuple(p+ratio*v for p,v in zip(pos,step))
                if not all(self._allowed(tuple(p+f*(v-p) for p,v in zip(pos,q))) for f in (0.25,0.5,0.75,1.0)):
                    continue
                safe = True
                for other in self.names:
                    if other == name or not self._position_fresh(other): continue
                    op = self.position[other]
                    future = proposed.get(other, tuple(p+2.0*v for p,v in zip(op,self.velocity[other])))
                    if not separation_safe_step(pos, q, op, future, self.scenario.min_separation_m + 2.0):
                        safe = False; break
                if safe:
                    chosen = q; break
            proposed[name] = chosen
            self._goal(name, chosen, dispatch=True)

    def _tick(self):
        self.desired_goals.clear()
        if self.scenario and self.clock_epoch is None:
            return
        if self.scenario:
            reserve = max((self._return_budget(n) for n in self.names), default=0.0)
            if self.sim_time + reserve >= self.scenario.mission_duration_s:
                self.finished = True
                for n in self.names:
                    if self.mav_state[n]["armed"]: self._rtl(n, "45 minute landing deadline")
                    elif self.stage[n] in ("mode", "arm", "takeoff"): self.stage[n] = "landed"
        for name in self.names:
            self._poll_mavlink(name)
            if not self.finished:
                self._advance_flight(name)
            pos = self.position[name]
            if self.scenario and self.stage[name] in ("ready", "ascending") and self.flight_started[name] is not None:
                if self.sim_time-self.flight_started[name]+self._return_budget(name) >= self.scenario.uav_endurance_s:
                    self._rtl(name, "20 minute sortie reserve")
            if self.stage[name] in ("ready", "ascending") and pos is not None:
                if not self._allowed(pos):
                    self._rtl(name, "geofence breached")
                elif self.battery[name] is not None:
                    if self.scenario:
                        battery_time = self.battery[name] * self.scenario.uav_endurance_s / 100.0
                        if battery_time <= self._return_budget(name) + 30.0:
                            self._rtl(name, "energy reserve")
                    elif not safe_to_continue(self.battery[name], pos, self.home[name],
                                              self.battery_reserve, self.pct_per_m):
                        self._rtl(name, "energy reserve")
        for name in self.names:
            if self.scenario and self.stage[name] == "returning" and self._position_fresh(name):
                pos, home = self.position[name], self.home[name]
                if horizontal_distance(pos, home) < 2.0:
                    self.links[name].set_mode_apm("LAND")
                    self.stage[name] = "landing"
                elif pos[2] < 68.0:
                    self._goal(name, (pos[0],pos[1],70.0))
                elif pos[0] > 76.0:
                    self._goal(name, (75.0, home[1],70.0))
                else:
                    self._goal(name, (home[0],home[1],70.0))
            if self.scenario and self.stage[name] == "landing" and not self.mav_state[name]["armed"]:
                self.stage[name], self.role[name] = "landed", "landed"
                self.landed_at[name] = self.sim_time
                self.get_logger().info(f"{name}: landed at pad; requesting recharge")
            if self.scenario and self.stage[name] == "landed" and self.landed_at[name] is not None:
                pending = any(not p["surveyed"] and p["spawn_s"] <= self.sim_time for p in self.pois)
                if not self.finished and pending and self.battery[name] is not None and self.battery[name] >= 88.0 and \
                        self.sim_time - self.landed_at[name] >= self.scenario.recharge_duration_s:
                    stop_pending = (name in self.recharge_jobs and
                                    self.recharge_jobs[name][1] is False)
                    if self.recharging[name] or stop_pending:
                        if self._set_recharge(name, False):
                            self.flight_started[name] = None
                            self.landed_at[name] = None
                            self.stage[name], self.role[name] = "mode", "idle"
                            self.get_logger().info(f"{name}: charged and rejoining pending PoI mission")
                    else:
                        self._set_recharge(name, True)
                elif not self.finished and any(not p["surveyed"] for p in self.pois):
                    self._set_recharge(name, True)
            if self.stage[name] == "rtl" and self.mav_state[name]["mode"] != RTL_MODE:
                self._send_mode(name, "RTL")
            if self.stage[name] == "rtl" and self.position[name] is not None and self.home[name] is not None:
                if horizontal_distance(self.position[name], self.home[name]) < 4.0 and self.position[name][2] < self.home[name][2] + 1.5:
                    if self.battery[name] is not None and self.battery[name] > 95.0:
                        self.stage[name], self.role[name] = "mode", "idle"
                        self.get_logger().info(f"{name}: battery restored at home; rejoining mission")
        if time.monotonic() - self.last_graph_at < 3.0 and not self.finished:
            self._plan()
        else:
            self._warn("Waiting for fresh communication graph")
        if self.scenario:
            self._dispatch_sample_goals()
        for name in self.names:
            status = UAVStatus()
            status.name = name
            if self.position[name] is not None:
                status.position = Point(x=self.position[name][0], y=self.position[name][1], z=self.position[name][2])
            status.battery_pct = float(self.battery[name] if self.battery[name] is not None else -1.0)
            status.role = self.role[name]
            status.current_task_id = self.assigned[name] or ""
            status.connected = self.hops[name] != -1
            status.hop_count = int(self.hops[name])
            status.flight_state = self.stage[name]
            status.position_valid = self._position_fresh(name)
            status.relay_reallocations = self.relay_reallocations
            status.flight_elapsed_s = float(self.sim_time-self.flight_started[name] if self.flight_started[name] is not None and self.mav_state[name]["armed"] else 0.0)
            status.speed_mps = float(math.sqrt(sum(v*v for v in self.velocity[name])))
            status.armed = self.mav_state[name]["armed"]
            status.network_recoveries = self.network_recoveries
            status.last_network_recovery_s = self.last_network_recovery_s
            self.status_pub.publish(status)
        for poi in self.pois:
            if poi["spawn_s"] > self.sim_time:
                continue
            update = PoI()
            update.id = poi["id"]
            update.position = Point(x=poi["x"], y=poi["y"], z=poi["z"])
            update.priority = poi["priority"]
            update.spawn_s = float(poi["spawn_s"])
            update.detected_s = float(poi["detected_s"])
            update.reported_s = float(poi["reported_s"])
            update.required_aircraft = len(self.scenario.chain((poi["x"],poi["y"],poi["z"]))) if self.scenario else 0
            update.surveyed, update.surveyed_by = poi["surveyed"], poi["surveyed_by"]
            self.poi_pub.publish(update)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for link in node.links.values():
            link.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
