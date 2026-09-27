"""Behavioral checks for routing and the safety constraints used by the planner."""

from types import SimpleNamespace

from swarm_stack.planning import best_path, relay_positions, safe_to_continue, within_fence


def edge(a, b, pdr, latency=20.0):
    return SimpleNamespace(node_a=a, node_b=b, pdr=pdr, latency_ms=latency, link_up=pdr > 0)


def test_relay_chain_reaches_distant_poi_with_bounded_hops():
    stations = relay_positions((-5, 0, 0), (285, 20, 0), 80, 15)
    points = [(-5, 0, 0), *stations, (285, 20, 15)]
    assert len(stations) == 3
    assert all(((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5 <= 80 for a, b in zip(points, points[1:]))


def test_best_path_uses_relay_and_excludes_failed_link():
    links = [edge("drone1", "GCS", 0), edge("drone1", "drone2", 0.9), edge("drone2", "GCS", 0.8)]
    path, pdr, latency = best_path(links, "drone1")
    assert path == ["drone1", "drone2", "GCS"]
    assert abs(pdr - 0.72) < 1e-9
    assert latency == 40.0
    assert best_path(links[:-1], "drone1")[0] == []


def test_fence_and_return_energy_reject_unsafe_target():
    assert within_fence((285, 20, 0), (0, 0, 0), 400, 10)
    assert not within_fence((405, 0, 0), (0, 0, 0), 400, 10)
    assert safe_to_continue(55, (120, 0, 15), (0, 0, 0), 25, 0.08)
    assert not safe_to_continue(30, (120, 0, 15), (0, 0, 0), 25, 0.08)


def test_close_aircraft_can_separate_but_not_move_closer():
    from swarm_stack.scenario import separation_safe_step
    assert separation_safe_step((0, 0, 40), (-5, 0, 40), (19, 0, 40), (19, 0, 40), 22)
    assert not separation_safe_step((0, 0, 40), (5, 0, 40), (19, 0, 40), (19, 0, 40), 22)
    assert not separation_safe_step((0, 0, 40), (40, 0, 40), (25, 0, 40), (25, 0, 40), 22)
    assert separation_safe_step((0, 0, 40), (0, 5, 40), (25, 0, 40), (25, 5, 40), 22)
