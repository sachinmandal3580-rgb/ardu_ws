"""Shared sample constraints and geometry; no ROS or GUI dependency."""
from dataclasses import dataclass, fields
import math
import random
from pathlib import Path
import yaml


@dataclass(frozen=True)
class Scenario:
    mission_duration_s: float = 2700.0
    uav_endurance_s: float = 1200.0
    comm_range_m: float = 100.0
    operational_area: tuple = (75.0, -250.0, 575.0, 250.0)
    gcs_position: tuple = (0.0, 0.0, 0.0)
    max_altitude_m: float = 100.0
    max_speed_mps: float = 5.0
    min_separation_m: float = 20.0
    report_deadline_s: float = 10.0
    poi_count: int = 10
    random_seed: int = 7
    spawn_window_s: tuple = (0.0, 1800.0)
    reference_fleet_size: int = 10
    relay_spacing_m: float = 80.0
    cruise_altitude_m: float = 40.0
    landing_speed_mps: float = 2.0
    return_reserve_s: float = 90.0
    recharge_duration_s: float = 120.0
    base_area: tuple = (-125.0, -75.0, 25.0, 75.0)
    corridor_area: tuple = (0.0, -75.0, 75.0, 75.0)
    pad_spacing_m: float = 25.0
    hop_latency_s: float = 0.02

    @classmethod
    def load(cls, path=None):
        if path is None:
            path = Path(__file__).resolve().parents[1] / 'config' / 'sample_scenario.yaml'
            if not path.exists():
                from ament_index_python.packages import get_package_share_directory
                path = Path(get_package_share_directory('swarm_stack')) / 'config' / 'sample_scenario.yaml'
        with open(path, encoding='utf-8') as f:
            raw = yaml.safe_load(f)
        allowed = {f.name for f in fields(cls)}
        if set(raw) - allowed:
            raise ValueError(f'Unknown scenario settings: {set(raw) - allowed}')
        obj = cls(**raw)
        obj.validate()
        return obj

    def validate(self):
        for key in ('mission_duration_s', 'uav_endurance_s', 'comm_range_m',
                    'max_speed_mps', 'min_separation_m', 'report_deadline_s', 'poi_count'):
            if getattr(self, key) <= 0:
                raise ValueError(f'{key} must be positive')
        if not self.min_separation_m < self.relay_spacing_m < self.comm_range_m:
            raise ValueError('Relay spacing must be between separation and communication limits')
        if math.hypot(self.relay_spacing_m, self.cruise_altitude_m) >= self.comm_range_m:
            raise ValueError('First relay must be within 3D radio range of ground GCS')
        if self.pad_spacing_m < self.min_separation_m:
            raise ValueError('Launch pads violate minimum separation')
        if not 0 < self.cruise_altitude_m <= self.max_altitude_m:
            raise ValueError('Invalid cruise altitude')
        if not 0 <= self.spawn_window_s[0] <= self.spawn_window_s[1] < self.mission_duration_s:
            raise ValueError('Invalid PoI spawn interval')
        for rect in (self.operational_area, self.base_area, self.corridor_area):
            if len(rect) != 4 or rect[0] >= rect[2] or rect[1] >= rect[3]:
                raise ValueError('Invalid area rectangle')

    def in_arena(self, point):
        x0, y0, x1, y1 = self.operational_area
        return x0 <= point[0] <= x1 and y0 <= point[1] <= y1

    def allowed(self, point):
        if not -1e-6 <= point[2] <= self.max_altitude_m + 1e-6:
            return False
        return any(r[0] <= point[0] <= r[2] and r[1] <= point[1] <= r[3]
                   for r in (self.operational_area, self.base_area, self.corridor_area))

    def pads(self, count):
        spacing = self.pad_spacing_m
        pads = [(x * spacing, y * spacing, 0.0) for x in range(-4, 1) for y in range(-2, 3)]
        pads.sort(key=lambda p: (math.dist(p, self.gcs_position), p))
        if count > len(pads):
            raise ValueError('This launch area supports at most 25 separated pads')
        return pads[:count]

    def chain(self, destination):
        """Relay stations and survey endpoint along the authorized entry corridor."""
        entry = (self.operational_area[0], 0.0)
        path = [tuple(self.gcs_position[:2]), entry, tuple(destination[:2])]
        lengths = [math.dist(path[i], path[i + 1]) for i in range(2)]
        total = sum(lengths)
        count = max(1, math.ceil(total / self.relay_spacing_m))
        result = []
        for i in range(1, count + 1):
            along = total * i / count
            segment = 0 if along <= lengths[0] else 1
            remaining = along if segment == 0 else along - lengths[0]
            ratio = remaining / max(lengths[segment], 1e-9)
            a, b = path[segment], path[segment + 1]
            result.append((a[0] + ratio * (b[0] - a[0]),
                           a[1] + ratio * (b[1] - a[1]), self.cruise_altitude_m))
        return result

    def random_pois(self):
        rng = random.Random(self.random_seed)
        x0, y0, x1, y1 = self.operational_area
        entries = []
        for i in range(self.poi_count):
            entries.append(dict(id=f'poi_{i + 1}', x=rng.uniform(x0 + 5, x1 - 5),
                                y=rng.uniform(y0 + 5, y1 - 5), z=0.0,
                                priority=rng.randint(1, 5),
                                spawn_s=rng.uniform(*self.spawn_window_s)))
        # Give the live map a visible, actionable PoI as soon as the mission
        # clock starts; keep the other nine spawn times randomized.
        if entries:
            entries[0]['spawn_s'] = self.spawn_window_s[0]
        return sorted(entries, key=lambda p: (p['spawn_s'], p['id']))

    def worst_case_aircraft(self):
        x0, y0, x1, y1 = self.operational_area
        return max(len(self.chain((x, y, 0.0))) for x in (x0, x1) for y in (y0, y1))


def swept_separation(a0, a1, b0, b1):
    """Minimum distance throughout two simultaneous linear movements."""
    r = [a - b for a, b in zip(a0, b0)]
    v = [(a - old_a) - (b - old_b) for a, old_a, b, old_b in zip(a1, a0, b1, b0)]
    vv = sum(t * t for t in v)
    u = max(0.0, min(1.0, -sum(x * y for x, y in zip(r, v)) / vv)) if vv else 0.0
    return math.sqrt(sum((x + u * y) ** 2 for x, y in zip(r, v)))


def separation_safe_step(start, end, other_start, other_end, clearance):
    """Maintain clearance, or strictly separate an already-too-close pair.

    A swept-distance threshold alone rejects every recovery move when the
    initial distance is below the threshold. Permit only monotonically
    increasing separation in that case; measured violations remain visible.
    """
    current = math.dist(start, other_start)
    if current >= clearance:
        return swept_separation(start, end, other_start, other_end) >= clearance
    relative = tuple(a - b for a, b in zip(start, other_start))
    velocity = tuple((a1-a0) - (b1-b0)
                     for a0, a1, b0, b1 in zip(start, end, other_start, other_end))
    return (sum(r*v for r, v in zip(relative, velocity)) >= 0.0 and
            math.dist(end, other_end) > current + 0.1)


def next_waypoint(position, target, scenario):
    entry_x = scenario.operational_area[0]
    if position[0] < entry_x - 0.2 and target[0] >= entry_x:
        return (entry_x, 0.0, target[2])
    if position[0] > entry_x + 0.2 and target[0] < entry_x:
        return (entry_x, 0.0, target[2])
    return target
