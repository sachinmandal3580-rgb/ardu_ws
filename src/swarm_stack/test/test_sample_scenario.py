"""End-to-end checks for the published sample geometry and ideal reference run."""
import math
from dataclasses import replace

from swarm_stack.reference_sim import ReferenceSwarm
from swarm_stack.scenario import Scenario, swept_separation


def test_sample_relay_geometry_and_seed():
    s = Scenario.load()
    assert s.worst_case_aircraft() == 8
    assert len(s.random_pois()) == 10
    assert s.random_pois() == s.random_pois()
    assert all(s.in_arena((p['x'], p['y'], 0)) for p in s.random_pois())
    pads = s.pads(10)
    assert all(math.dist(a, b) >= 20 for i, a in enumerate(pads) for b in pads[i + 1:])
    for target in ((75, -250, 0), (575, -250, 0), (575, 250, 0)):
        chain = [s.gcs_position, *s.chain(target)]
        assert all(math.dist(a, b) <= s.comm_range_m for a, b in zip(chain, chain[1:]))
        assert all(s.allowed(p) for p in chain)
    assert swept_separation((0, 0, 40), (20, 0, 40), (20, 0, 40), (0, 0, 40)) == 0


def test_randomized_reference_missions_meet_measured_limits():
    base = Scenario.load()
    # Seed 5 previously trapped an aircraft during return; seed 6 previously
    # froze a disconnected surveyor just outside the final relay hop.
    for seed in (5, 6, 7):
        sim = ReferenceSwarm(replace(base, random_seed=seed))
        while sim.time < sim.scenario.mission_duration_s:
            sim.step()
        result = sim.summary()
        assert result['reported'] == result['poi_count'] == 10, seed
        assert result['landed'] == result['fleet_size'] == 10, seed
        assert result['all_landed_by_deadline'], seed
        assert result['min_separation_m'] >= sim.scenario.min_separation_m, seed
        assert result['max_speed_mps'] <= sim.scenario.max_speed_mps + 1e-6, seed
        assert result['max_altitude_m'] <= sim.scenario.max_altitude_m, seed
        assert result['max_sortie_s'] <= sim.scenario.uav_endurance_s, seed
        assert result['max_report_delay_s'] <= sim.scenario.report_deadline_s, seed
        assert not any(result['violations'].values()), seed


def test_relay_faults_recover_and_finish_reference_mission():
    for fault in ('fail_relay', 'degrade_relay'):
        sim = ReferenceSwarm()
        while sim.time < 250:
            sim.step()
        getattr(sim, fault)()
        while sim.time < sim.scenario.mission_duration_s:
            sim.step()
        result = sim.summary()
        assert result['reported'] == 10, fault
        assert result['landed'] == 10, fault
        assert result['recovery_times_s'], fault
        assert result['relay_reallocations'] > 0, fault
        assert not any(result['violations'].values()), fault


def test_new_urgent_poi_preempts_and_reports():
    sim = ReferenceSwarm()
    while sim.time < 250:
        sim.step()
    sim.add_emergency_poi()
    sim.step()
    assert sim.active_id == 'emergency_1'
    while sim.time < sim.scenario.mission_duration_s:
        sim.step()
    result = sim.summary()
    assert result['poi_count'] == result['reported'] == 11
    assert result['all_landed_by_deadline']
    assert not any(result['violations'].values())


def test_simultaneous_pois_share_relays_and_use_separate_surveyors():
    sim = ReferenceSwarm()
    sim.pois = [dict(id=poi_id, x=x, y=y, z=0.0, priority=priority,
                     spawn_s=0.0, detected_s=None, reported_s=None,
                     reporter=None, delivery_due=None)
                for poi_id, x, y, priority in
                [('poi_1', 200.0, 0.0, 1), ('emergency_1', 225.0, 10.0, 10)]]
    sim.step()
    tasks = sim.task_assignments
    assert set(tasks) == {'poi_1', 'emergency_1'}
    assert tasks['poi_1']['surveyor'] != tasks['emergency_1']['surveyor']
    assert set(tasks['poi_1']['relays'].values()) & set(tasks['emergency_1']['relays'].values())
    assert sum(d.role == 'surveyor' for d in sim.drones) == 2
    while sim.time < 600 and sim.summary()['reported'] < 2:
        sim.step()
    assert sim.summary()['reported'] == 2
    assert all(p['reported_s'] - p['detected_s'] <= sim.scenario.report_deadline_s for p in sim.pois)
    assert not any(sim.violations.values())
