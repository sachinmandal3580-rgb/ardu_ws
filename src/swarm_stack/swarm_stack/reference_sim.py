"""Deterministic kinematic reference model, independent of Gazebo/ArduPilot.

Motion and radio geometry are simulated; payloads are synthetic reports, not
camera images. Limits and every measured violation are exposed in the snapshot.
"""
import argparse
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from .scenario import Scenario, next_waypoint, swept_separation


@dataclass
class Aircraft:
    name: str
    home: tuple
    position: tuple
    role: str = 'landed'
    task_id: str = ''
    target: tuple | None = None
    flight_s: float = 0.0
    max_sortie_s: float = 0.0
    charge_until: float = 0.0
    trail: list = field(default_factory=list)


class ReferenceSwarm:
    def __init__(self, scenario=None, fleet_size=None):
        self.scenario = scenario or Scenario.load()
        s = self.scenario
        self.drones = [Aircraft(f'drone{i+1}', p, p) for i, p in enumerate(s.pads(fleet_size or s.reference_fleet_size))]
        self.pois = [dict(p, detected_s=None, reported_s=None, reporter=None, delivery_due=None)
                     for p in s.random_pois()]
        self.time = 0.0
        self.events = []
        self.active_id = None
        self.assignments = {}
        self.task_assignments = {}
        self.slot_by_name = {}
        self.plan_at = -1.0
        self.reallocations = 0
        self.degraded_edges = set()
        self.degraded_drones = set()
        self.failed = set()
        self.returning_all = False
        self.minimum_separation = s.pad_spacing_m
        self.max_speed = 0.0
        self.max_altitude = 0.0
        self.violations = {k: 0 for k in ('separation', 'geofence', 'speed', 'altitude', 'endurance', 'late_report')}
        self.packet_sent = 0
        self.packet_delivered = 0
        self.disconnected_uav_s = 0.0
        self.recovery_started = None
        self.recovery_times = []
        self.report_times = []
        self._last_logged_spawn = set()
        self.log(f'{len(self.drones)} UAVs; full-area chain needs {s.worst_case_aircraft()} (plus spares)')

    def log(self, text):
        self.events.append((self.time, text))

    def graph(self, positions=None, names=None):
        positions = positions or [d.position for d in self.drones]
        points = [tuple(self.scenario.gcs_position)] + positions
        adjacency = {i: [] for i in range(len(points))}
        edges = []
        for i, a in enumerate(points):
            if i and (self.drones[i-1].name in self.failed or (names is not None and self.drones[i-1].name not in names)):
                continue
            for j in range(i+1, len(points)):
                if self.drones[j-1].name in self.failed or (names is not None and self.drones[j-1].name not in names) or frozenset((i, j)) in self.degraded_edges:
                    continue
                if math.dist(a, points[j]) <= self.scenario.comm_range_m:
                    adjacency[i].append(j); adjacency[j].append(i)
                    edges.append((i, j))
        hops = {0: 0}
        parents = {}
        queue = [0]
        for i in queue:
            for j in adjacency[i]:
                if j not in hops:
                    hops[j] = hops[i] + 1
                    parents[j] = i
                    queue.append(j)
        return hops, parents, edges

    def return_distance(self, drone, position=None):
        p = position or drone.position
        gate = (self.scenario.operational_area[0], 0.0, self.scenario.cruise_altitude_m)
        return (math.dist(p[:2], gate[:2]) + math.dist(gate[:2], drone.home[:2])
                if p[0] > gate[0] else math.dist(p[:2], drone.home[:2]))

    def return_budget(self, drone, position=None):
        s = self.scenario
        return self.return_distance(drone, position) / s.max_speed_mps + (2*70-s.cruise_altitude_m) / s.landing_speed_mps + s.return_reserve_s

    def fail_relay(self):
        candidates = [d for d in self.drones if d.role == 'relay']
        if not candidates:
            self.log('No active relay available to fail')
            return
        d = min(candidates, key=lambda v: math.dist(v.position, self.scenario.gcs_position))
        self.failed.add(d.name)
        # Logical radio/mission failure, with a controlled return in this model.
        d.role = 'returning'
        self.recovery_started = self.time
        self.plan_at = -1.0
        self.log(f'{d.name}: radio failure; returning to pad')

    def degrade_relay(self):
        candidates = [i+1 for i, d in enumerate(self.drones) if d.role == 'relay']
        hops, parents, _ = self.graph()
        candidates = [i for i in candidates if i in parents]
        if candidates:
            i = min(candidates, key=lambda k: hops[k])
            self.degraded_edges.add(frozenset((i, parents[i])))
            affected = self.drones[i-1]
            self.degraded_drones.add(affected.name)
            affected.role = 'returning'
            self.recovery_started = self.time
            self.plan_at = -1.0
            self.log(f'Degraded link to {affected.name}; replacing relay and returning affected UAV')

    def add_emergency_poi(self):
        """Spawn a repeatable high-priority task while the mission is running."""
        if self.returning_all:
            self.log('Landing phase: emergency task arrived after dispatch cutoff')
            return
        import random
        rng = random.Random(self.scenario.random_seed + int(self.time))
        x0, y0, x1, y1 = self.scenario.operational_area
        poi_id = f'emergency_{1 + sum(p["id"].startswith("emergency_") for p in self.pois)}'
        self.pois.append(dict(id=poi_id, x=rng.uniform(x0 + 5, x1 - 5),
                              y=rng.uniform(y0 + 5, y1 - 5), z=0.0,
                              priority=10, spawn_s=self.time, detected_s=None,
                              reported_s=None, reporter=None, delivery_due=None))
        self.plan_at = -1.0
        self._last_logged_spawn.add(poi_id)
        self.log(f'{poi_id} spawned with priority 10; replanning')

    def _plan(self):
        s = self.scenario
        available = [d for d in self.drones if d.role not in ('returning', 'charging', 'failed')
                     and d.name not in self.failed and d.name not in self.degraded_drones]
        pending = [p for p in self.pois if p['spawn_s'] <= self.time and p['reported_s'] is None]
        # Priority tasks take the first feasible chain. Keep an existing task
        # ahead of a newly spawned task of the same priority.
        pending.sort(key=lambda p: (p['priority'] < 10, p['id'] not in self.task_assignments,
                                    p['detected_s'] is None,
                                    -len(s.chain((p['x'], p['y'], 0.0))),
                                    -p['priority'], p['spawn_s']))
        pool = list(available)
        relay_targets = {}
        tasks = {}
        slots = {}

        def energy_safe(drone, target):
            return (drone.flight_s + math.dist(drone.position, target) / s.max_speed_mps
                    + self.return_budget(drone, target) + 30 < s.uav_endurance_s)

        for poi in pending:
            chain = s.chain((poi['x'], poi['y'], 0.0))
            trial_pool = pool[:]
            trial_targets = relay_targets.copy()
            trial_relays = {}
            feasible = True
            old = self.task_assignments.get(poi['id'], {})
            for slot, station in enumerate(chain[:-1]):
                shared = next((name for name, target in trial_targets.items()
                               if math.dist(target, station) <= 19.0), None)
                if shared:
                    trial_relays[slot] = shared
                    continue
                if (any(math.dist(target, station) < s.min_separation_m + 0.1
                        for target in trial_targets.values()) or
                    any(math.dist(task['endpoint'], station) < s.min_separation_m + 0.1
                        for task in tasks.values())):
                    feasible = False
                    break
                good = [d for d in trial_pool if energy_safe(d, station)]
                if not good:
                    feasible = False
                    break
                incumbent = old.get('relays', {}).get(slot)
                chosen = next((d for d in good if d.name == incumbent), None)
                if chosen is None:
                    chosen = min(good, key=lambda d: math.dist(d.position, station))
                trial_relays[slot] = chosen.name
                trial_targets[chosen.name] = station
                trial_pool.remove(chosen)
            endpoint = chain[-1]
            surveyors = [d for d in trial_pool if energy_safe(d, endpoint)]
            if any(math.dist(target, endpoint) < s.min_separation_m + 0.1
                   for target in trial_targets.values()):
                feasible = False
            if any(math.dist(tasks[task_id]['endpoint'], endpoint) < s.min_separation_m + 0.1
                   for task_id in tasks):
                feasible = False
            if not feasible or not surveyors:
                continue
            incumbent = old.get('surveyor')
            surveyor = next((d for d in surveyors if d.name == incumbent), None)
            if surveyor is None:
                surveyor = min(surveyors, key=lambda d: math.dist(d.position, endpoint))
            trial_pool.remove(surveyor)
            pool, relay_targets = trial_pool, trial_targets
            tasks[poi['id']] = {'surveyor': surveyor.name, 'relays': trial_relays, 'endpoint': endpoint}
            for slot, name in trial_relays.items():
                slots[name] = max(slots.get(name, 0), slot)
            surveyor.target = endpoint
            slots[surveyor.name] = len(chain) - 1
        if tasks != self.task_assignments and self.task_assignments:
            self.reallocations += 1
        for poi_id in tasks:
            if poi_id not in self.task_assignments:
                self.log(f'Assigned {poi_id}: {tasks[poi_id]["surveyor"]} surveying')
        self.task_assignments = tasks
        self.slot_by_name = slots
        self.active_id = next(iter(tasks), None)
        self.assignments = tasks[self.active_id]['relays'].copy() if self.active_id else {}
        for drone in available:
            if drone.name in relay_targets:
                drone.role, drone.target = 'relay', relay_targets[drone.name]
            elif any(task['surveyor'] == drone.name for task in tasks.values()):
                drone.role = 'surveyor'
            elif drone.position[2] > 0.1:
                drone.role = 'returning'
            else:
                drone.role = 'landed'
        for drone in self.drones:
            drone.task_id = next((poi_id for poi_id, task in tasks.items()
                                  if task['surveyor'] == drone.name and drone.role == 'surveyor'), '')

    def _target(self, d):
        s = self.scenario
        if d.role == 'returning':
            if math.dist(d.position[:2], d.home[:2]) < 0.4:
                return d.home
            return_alt = min(70.0, s.max_altitude_m)
            if d.position[2] < return_alt - 0.1:
                return (d.position[0], d.position[1], return_alt)
            if d.position[0] > s.operational_area[0] + 0.2:
                return (s.operational_area[0], d.home[1], return_alt)
            return (d.home[0], d.home[1], return_alt)
        if d.role in ('relay', 'surveyor'):
            if d.role == 'surveyor' and math.dist(d.position[:2],d.target[:2]) < 30:
                ready = all(math.dist(v.position,v.target) < 3 for v in self.drones if v.role == 'relay')
                if not ready:
                    return d.position
            # Use a vertical transit lane above deployed relays. A 40 m
            # station otherwise blocks an outbound vehicle at the entry gate.
            slot = self.slot_by_name.get(d.name, 0)
            remaining_xy = math.dist(d.position[:2], d.target[:2])
            transit_alt = min(70.0, s.max_altitude_m)
            if slot > 0 and remaining_xy > 25.0:
                if d.position[2] < transit_alt - 0.1:
                    return (d.position[0], d.position[1], transit_alt)
                route = next_waypoint(d.position, d.target, s)
                return (route[0], route[1], transit_alt)
            if d.position[2] > s.cruise_altitude_m + 0.1:
                return (d.position[0], d.position[1], s.cruise_altitude_m)
            if d.position[2] < s.cruise_altitude_m - 0.1:
                return (d.position[0], d.position[1], s.cruise_altitude_m)
            return next_waypoint(d.position, d.target, s)
        return d.position

    def _move(self, dt):
        s = self.scenario
        old = [d.position for d in self.drones]
        new = list(old)
        connected = set(self.graph(old)[0])
        # Rotating priority prevents a particular UAV from always yielding.
        order = list(range(len(self.drones)))
        shift = int(self.time // 5) % len(order)
        order = order[shift:] + order[:shift]
        for i in order:
            d = self.drones[i]
            target = self._target(d)
            delta = tuple(t-p for t, p in zip(target, old[i]))
            length = math.sqrt(sum(v*v for v in delta))
            if length < 1e-7:
                continue
            speed = s.landing_speed_mps if abs(delta[2]) > 0.1 else s.max_speed_mps
            ratio = min(1.0, speed*dt/length)
            step = tuple(v*ratio for v in delta)
            candidates = []
            angles = (0, 25, -25, 50, -50, 80, -80, 110, -110) if abs(step[2]) < 0.1 else (0,)
            for degrees in angles:
                a = math.radians(degrees)
                v = (step[0]*math.cos(a)-step[1]*math.sin(a), step[0]*math.sin(a)+step[1]*math.cos(a), step[2])
                q = tuple(p+x for p,x in zip(old[i], v))
                if all(s.allowed(tuple(p+f*(x-p) for p,x in zip(old[i],q))) for f in (0.25,0.5,0.75,1.0)):
                    candidates.append(q)
            if abs(step[2]) > 0.1 and math.hypot(step[0], step[1]) < 0.1:
                # A vehicle directly above can block a pure vertical climb.
                # Shift sideways while climbing, within the 5 m/s cap.
                for dx,dy in ((2,0),(0,2),(-2,0),(0,-2),(2,2),(-2,2),(2,-2),(-2,-2)):
                    q=(old[i][0]+dx,old[i][1]+dy,old[i][2]+step[2])
                    if math.dist(old[i],q) <= s.max_speed_mps*dt and all(
                            s.allowed(tuple(p+f*(x-p) for p,x in zip(old[i],q)))
                            for f in (0.25,0.5,0.75,1.0)):
                        candidates.append(q)
            for q in candidates:
                if any(j != i and swept_separation(old[i],q,old[j],new[j]) < s.min_separation_m + 0.1
                       for j in range(len(new))):
                    continue
                trial = new[:]; trial[i] = q
                # Returning vehicles may have to leave the network to land before
                # endurance expiry; this is measured as downtime, never hidden.
                after = set(self.graph(trial)[0])
                active_connected = {j for j in connected if j and self.drones[j-1].role not in ('returning','charging')}
                if d.role != 'returning' and not active_connected.issubset(after):
                    continue
                new[i] = q
                connected = after
                break
        for i,d in enumerate(self.drones):
            speed = math.dist(old[i], new[i])/dt
            self.max_speed = max(self.max_speed, speed)
            self.max_altitude = max(self.max_altitude, new[i][2])
            if speed > s.max_speed_mps + 1e-6: self.violations['speed'] += 1
            if new[i][2] > s.max_altitude_m + 1e-6: self.violations['altitude'] += 1
            if not s.allowed(new[i]): self.violations['geofence'] += 1
            for j in range(i):
                sep = swept_separation(old[i], new[i], old[j], new[j])
                self.minimum_separation = min(self.minimum_separation, sep)
                if sep < s.min_separation_m - 1e-6: self.violations['separation'] += 1
            d.position = new[i]
            if old[i][2] > 0.01 or new[i][2] > 0.01:
                d.flight_s += dt
                d.max_sortie_s = max(d.max_sortie_s, d.flight_s)
                if d.flight_s > s.uav_endurance_s: self.violations['endurance'] += 1
            if int(self.time) % 5 == 0:
                d.trail.append(new[i]); d.trail = d.trail[-250:]
            if d.role == 'returning' and math.dist(d.position,d.home) < 0.1:
                d.role = 'failed' if d.name in self.failed else 'charging'
                d.charge_until = self.time + s.recharge_duration_s
                self.log(f'{d.name} landed after {d.flight_s:.0f} s; recharge timer started')

    def step(self, dt=1.0):
        s = self.scenario
        if self.time >= s.mission_duration_s: return
        dt = min(dt, s.mission_duration_s-self.time)
        self.time += dt
        for p in self.pois:
            if p['spawn_s'] <= self.time and p['id'] not in self._last_logged_spawn:
                self._last_logged_spawn.add(p['id']); self.plan_at = -1.0
                self.log(f"{p['id']} spawned, priority {p['priority']}")
        for d in self.drones:
            if d.role == 'charging' and self.time >= d.charge_until:
                d.role = 'landed'; d.flight_s = 0.0; self.plan_at = -1.0
            if d.role in ('relay','surveyor') and d.flight_s + self.return_budget(d) >= s.uav_endurance_s:
                d.role = 'returning'; self.plan_at = -1.0
                self.log(f'{d.name}: return for endurance reserve')
        all_done = all(p['reported_s'] is not None for p in self.pois)
        return_time = max(self.return_budget(d) for d in self.drones)
        if not self.returning_all and (all_done or self.time + return_time >= s.mission_duration_s):
            self.returning_all = True
            self.log('All aircraft returning: completion or landing deadline reserve')
        if self.returning_all:
            for d in self.drones:
                if d.position[2] > 0.01: d.role = 'returning'
        elif self.time >= self.plan_at:
            self._plan(); self.plan_at = self.time + 5.0
        self._move(dt)
        hops, _, _ = self.graph()
        active = [i+1 for i,d in enumerate(self.drones) if d.position[2] > 0.1 and d.name not in self.failed]
        self.packet_sent += len(active)
        self.packet_delivered += sum(i in hops for i in active)
        self.disconnected_uav_s += dt*sum(i not in hops for i in active)
        if self.recovery_started is not None and self.assignments:
            settled = all(math.dist(d.position,d.target) < 5 for d in self.drones if d.role == 'relay')
            if settled and all(i in hops for i in active):
                elapsed = self.time-self.recovery_started
                self.recovery_times.append(elapsed); self.recovery_started=None
                self.log(f'Network recovered in {elapsed:.1f} s')
        for p in self.pois:
            if p['spawn_s'] > self.time or p['reported_s'] is not None: continue
            for i,d in enumerate(self.drones,start=1):
                if d.role != 'surveyor' or math.dist(d.position[:2],(p['x'],p['y'])) > 5: continue
                if p['detected_s'] is None:
                    p['detected_s'] = self.time; p['reporter'] = i
                    self.log(f"Detected {p['id']}; reporting deadline starts")
            if p['detected_s'] is not None:
                i = p['reporter']
                if i in hops:
                    if p['delivery_due'] is None: p['delivery_due'] = self.time + hops[i]*s.hop_latency_s
                    if self.time >= p['delivery_due']:
                        p['reported_s']=self.time
                        delay=self.time-p['detected_s']; self.report_times.append(delay)
                        if delay > s.report_deadline_s: self.violations['late_report'] += 1
                        self.log(f"GCS received {p['id']} after {delay:.1f} s")
                        self.plan_at=-1.0
                else:
                    p['delivery_due']=None

    def summary(self):
        s=self.scenario
        return dict(time_s=self.time, fleet_size=len(self.drones), reported=sum(p['reported_s'] is not None for p in self.pois),
                    poi_count=len(self.pois), landed=sum(d.position[2] <= 0.1 for d in self.drones),
                    min_separation_m=self.minimum_separation, max_speed_mps=self.max_speed,
                    max_altitude_m=self.max_altitude, max_sortie_s=max(d.max_sortie_s for d in self.drones),
                    max_report_delay_s=max(self.report_times,default=None), violations=self.violations,
                    pdr_pct=100*self.packet_delivered/max(1,self.packet_sent), downtime_uav_s=self.disconnected_uav_s,
                    relay_reallocations=self.reallocations, recovery_times_s=self.recovery_times,
                    all_landed_by_deadline=self.time <= s.mission_duration_s and all(d.position[2] <= 0.1 for d in self.drones))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario'); parser.add_argument('--fleet',type=int)
    parser.add_argument('--output',default='logs/reference_sample.json')
    args=parser.parse_args()
    sim=ReferenceSwarm(Scenario.load(args.scenario),args.fleet)
    while sim.time < sim.scenario.mission_duration_s:
        sim.step()
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    payload=dict(model='Ideal kinematics and synthetic reports; not Gazebo flight validation',
                 summary=sim.summary(),pois=sim.pois,events=sim.events)
    path.write_text(json.dumps(payload,indent=2))
    print(json.dumps(sim.summary(),indent=2))


if __name__=='__main__': main()
