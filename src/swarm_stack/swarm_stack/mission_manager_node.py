#!/usr/bin/env python3
"""
mission_manager_node
---------------------
Central decision-maker for the swarm:
  - allocates PoIs to idle surveyor UAVs (greedy nearest-PoI baseline)
  - detects UAVs that have lost their multi-hop path to the GCS and
    reassigns the nearest idle UAV to a relay role
  - sends UAVs to RTL when battery is low, freeing their assigned PoI
  - reacts to /swarm/fault_events (uav_failure) by freeing tasks and
    marking that UAV as failed
  - publishes /swarm/uav_status (one message per UAV) for the metrics logger

COORDINATE FRAMES — read this before changing anything:
Task allocation and relay-midpoint math are done entirely in the shared
Gazebo *world frame* (same frame as comms_sim_node, via /<name>/odometry).
Flight commands, however, must go out through ArduPilot's DDS interface as
*global* GPS coordinates (/<name>/ap/cmd_gps_pose). This node converts
world-frame (x, y) offsets to lat/lon using a flat-earth approximation
around a single shared SITL origin (see ORIGIN_LAT/ORIGIN_LON below).
This is valid ONLY because all ArduPilot SITL instances in this workspace
share the same default global origin and Gazebo's world x/y map to ENU
metres from that origin — verify this against your own launch output
(the "Home: <lat> <lon>" line each drone prints on boot) before trusting
it, and adjust ORIGIN_LAT/ORIGIN_LON if your world file changes it.
"""

import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from ardupilot_msgs.msg import GlobalPosition

from swarm_interfaces.msg import ConnectivityGraph, UAVStatus, PoI, FaultEvent

# --- Tunables -----------------------------------------------------------
ORIGIN_LAT = -35.363262
ORIGIN_LON = 149.165237
EARTH_RADIUS_M = 6378137.0

GUIDED_MODE = 4       # ArduCopter mode number for GUIDED
RTL_MODE = 6          # ArduCopter mode number for RTL
LOW_BATTERY_PCT = 25.0
MAX_HOP_COUNT_OK = 2  # if hop_count exceeds this (or is -1), trigger relay
SURVEY_RADIUS_M = 3.0  # PoI counted "surveyed" once a UAV is this close


def local_to_latlon(x, y):
    """Flat-earth conversion of an ENU (x=East, y=North) offset in metres
    from ORIGIN_LAT/ORIGIN_LON to (lat, lon) in degrees."""
    lat = ORIGIN_LAT + (y / EARTH_RADIUS_M) * (180.0 / math.pi)
    lon = ORIGIN_LON + (x / (EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))) * (180.0 / math.pi)
    return lat, lon


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("uav_names", ["drone1", "drone2", "drone3", "drone4", "drone5"])
        self.declare_parameter("poi_config_path", "")
        self.declare_parameter("control_rate_hz", 1.0)

        self.uav_names = self.get_parameter("uav_names").value
        poi_path = self.get_parameter("poi_config_path").value
        rate = self.get_parameter("control_rate_hz").value

        self.pois = self._load_pois(poi_path)

        # per-UAV live state
        self.position = {n: None for n in self.uav_names}
        self.battery_pct = {n: 100.0 for n in self.uav_names}
        self.hop_count = {n: -1 for n in self.uav_names}
        self.role = {n: "idle" for n in self.uav_names}
        self.assigned_poi = {n: None for n in self.uav_names}
        self.armed_guided = {n: False for n in self.uav_names}
        self.relay_reallocation_count = 0

        # per-UAV service clients / publishers, created lazily
        self._arm_clients = {}
        self._mode_clients = {}
        self._goal_pubs = {}
        for n in self.uav_names:
            self._arm_clients[n] = self.create_client(ArmMotors, f"/{n}/ap/arm_motors")
            self._mode_clients[n] = self.create_client(ModeSwitch, f"/{n}/ap/mode_switch")
            self._goal_pubs[n] = self.create_publisher(GlobalPosition, f"/{n}/ap/cmd_gps_pose", 10)
            self.create_subscription(
                Odometry, f"/{n}/odometry",
                lambda msg, name=n: self._odom_cb(name, msg), 10)
            self.create_subscription(
                BatteryState, f"/{n}/ap/battery/battery0",
                lambda msg, name=n: self._battery_cb(name, msg), 10)

        self.create_subscription(
            ConnectivityGraph, "/swarm/connectivity_graph", self._graph_cb, 10)
        self.create_subscription(
            FaultEvent, "/swarm/fault_events", self._fault_cb, 10)

        self.status_pub = self.create_publisher(UAVStatus, "/swarm/uav_status", 10)
        self.poi_pub = self.create_publisher(PoI, "/swarm/poi_updates", 10)

        self.timer = self.create_timer(1.0 / rate, self._control_loop)
        self.get_logger().info(
            f"mission_manager_node up. {len(self.pois)} PoIs loaded, "
            f"managing {len(self.uav_names)} UAVs.")

    # ---------------------------------------------------------------- IO
    def _load_pois(self, path):
        if not path:
            self.get_logger().warn("No poi_config_path set — starting with zero PoIs.")
            return []
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        pois = []
        for entry in data.get("pois", []):
            pois.append({
                "id": entry["id"],
                "x": float(entry["x"]), "y": float(entry["y"]), "z": float(entry.get("z", 0.0)),
                "priority": int(entry.get("priority", 1)),
                "surveyed": False,
                "surveyed_by": "",
            })
        return pois

    def _odom_cb(self, name, msg: Odometry):
        p = msg.pose.pose.position
        self.position[name] = (p.x, p.y, p.z)

    def _battery_cb(self, name, msg: BatteryState):
        if msg.percentage is not None and msg.percentage >= 0:
            self.battery_pct[name] = msg.percentage * (100.0 if msg.percentage <= 1.0 else 1.0)

    def _graph_cb(self, msg: ConnectivityGraph):
        for name, hops in zip(msg.uav_names, msg.hop_counts):
            self.hop_count[name] = hops

    def _fault_cb(self, msg: FaultEvent):
        if msg.event_type == "uav_failure" and msg.target in self.role:
            self.get_logger().warn(f"FAULT: {msg.target} marked as failed. Freeing its task.")
            if self.assigned_poi[msg.target]:
                self._free_poi(self.assigned_poi[msg.target])
            self.role[msg.target] = "failed"
        elif msg.event_type == "new_poi":
            self.get_logger().info(f"New high-priority PoI event: {msg.description}")
            # Operator should also add the PoI via the config or a future
            # dedicated 'add_poi' service; this just logs for now.

    # ------------------------------------------------------------- utils
    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _free_poi(self, poi_id):
        for poi in self.pois:
            if poi["id"] == poi_id and not poi["surveyed"]:
                for n, assigned in self.assigned_poi.items():
                    if assigned == poi_id:
                        self.assigned_poi[n] = None

    def _idle_uavs(self):
        return [n for n in self.uav_names
                if self.role[n] in ("idle", "surveyor") and self.assigned_poi[n] is None
                and self.position[n] is not None]

    # -------------------------------------------------------- commanding
    def _ensure_armed_guided(self, name):
        if self.armed_guided[name]:
            return
        arm_cli = self._arm_clients[name]
        mode_cli = self._mode_clients[name]
        if arm_cli.service_is_ready():
            req = ArmMotors.Request()
            req.arm = True
            arm_cli.call_async(req)
        if mode_cli.service_is_ready():
            req = ModeSwitch.Request()
            req.mode = GUIDED_MODE
            mode_cli.call_async(req)
        self.armed_guided[name] = True

    def _send_goal(self, name, x, y, z, alt_is_relative=True):
        self._ensure_armed_guided(name)
        lat, lon = local_to_latlon(x, y)
        msg = GlobalPosition()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = float(z if z > 0.5 else 15.0)  # default survey altitude
        msg.coordinate_frame = 5  # matches ArduPilot's documented waypoint-goal example
        self._goal_pubs[name].publish(msg)

    def _send_rtl(self, name):
        mode_cli = self._mode_clients[name]
        if mode_cli.service_is_ready():
            req = ModeSwitch.Request()
            req.mode = RTL_MODE
            mode_cli.call_async(req)

    # --------------------------------------------------------- main loop
    def _control_loop(self):
        # 1. Battery / RTL check
        for n in self.uav_names:
            if self.role[n] == "failed":
                continue
            if self.battery_pct[n] < LOW_BATTERY_PCT and self.role[n] != "rtl":
                self.get_logger().info(f"{n} battery {self.battery_pct[n]:.1f}% -> RTL")
                if self.assigned_poi[n]:
                    self._free_poi(self.assigned_poi[n])
                self.role[n] = "rtl"
                self._send_rtl(n)

        # 2. Task allocation — greedy nearest unsurveyed PoI to an idle UAV
        unsurveyed = [p for p in self.pois if not p["surveyed"]
                      and p["id"] not in self.assigned_poi.values()]
        unsurveyed.sort(key=lambda p: -p["priority"])
        for poi in unsurveyed:
            candidates = self._idle_uavs()
            if not candidates:
                break
            poi_pos = (poi["x"], poi["y"], poi["z"])
            best = min(candidates, key=lambda n: self._dist(self.position[n], poi_pos))
            self.assigned_poi[best] = poi["id"]
            self.role[best] = "surveyor"
            self._send_goal(best, poi["x"], poi["y"], poi["z"])
            self.get_logger().info(f"Assigned {best} -> {poi['id']}")

        # 3. Check for arrival at assigned PoI
        for n in self.uav_names:
            if self.assigned_poi[n] and self.position[n]:
                for poi in self.pois:
                    if poi["id"] == self.assigned_poi[n] and not poi["surveyed"]:
                        d = self._dist(self.position[n], (poi["x"], poi["y"], poi["z"]))
                        if d <= SURVEY_RADIUS_M:
                            poi["surveyed"] = True
                            poi["surveyed_by"] = n
                            self.assigned_poi[n] = None
                            self.role[n] = "idle"
                            self.get_logger().info(f"{n} surveyed {poi['id']}")

        # 4. Relay reassignment — any surveyor with a broken/too-long path?
        for n in self.uav_names:
            if self.role[n] != "surveyor":
                continue
            hops = self.hop_count[n]
            if hops == -1 or hops > MAX_HOP_COUNT_OK:
                candidates = [c for c in self._idle_uavs() if c != n]
                if not candidates:
                    continue
                relay = min(candidates, key=lambda c: self._dist(self.position[c], self.position[n]))
                midpoint = tuple((a + b) / 2.0 for a, b in zip(self.position[n], self.position[relay]))
                self.role[relay] = "relay"
                self.assigned_poi[relay] = None
                self._send_goal(relay, midpoint[0], midpoint[1], midpoint[2])
                self.relay_reallocation_count += 1
                self.get_logger().warn(
                    f"{n} disconnected (hops={hops}) -> {relay} reassigned to relay "
                    f"(reallocation #{self.relay_reallocation_count})")

        # 5. Publish status + PoI state for the metrics logger
        for n in self.uav_names:
            msg = UAVStatus()
            msg.name = n
            if self.position[n]:
                msg.position = Point(x=self.position[n][0], y=self.position[n][1], z=self.position[n][2])
            msg.battery_pct = float(self.battery_pct[n])
            msg.role = self.role[n]
            msg.current_task_id = self.assigned_poi[n] or ""
            msg.connected = self.hop_count[n] != -1
            msg.hop_count = int(self.hop_count[n])
            self.status_pub.publish(msg)

        for poi in self.pois:
            msg = PoI()
            msg.id = poi["id"]
            msg.position = Point(x=poi["x"], y=poi["y"], z=poi["z"])
            msg.priority = poi["priority"]
            msg.surveyed = poi["surveyed"]
            msg.surveyed_by = poi["surveyed_by"]
            self.poi_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
