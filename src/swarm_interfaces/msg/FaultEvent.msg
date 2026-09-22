# Published to /swarm/fault_events to trigger a simulated disturbance.
string event_type   # "uav_failure" | "link_degraded" | "new_poi"
string target        # UAV name for uav_failure/link_degraded; PoI id for new_poi
string description
