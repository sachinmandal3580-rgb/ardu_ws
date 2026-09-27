# Current state of a single UAV in the swarm, published on /swarm/uav_status.
string name
geometry_msgs/Point position
float32 battery_pct
string role              # "idle" | "surveyor" | "relay" | "rtl" | "failed"
string current_task_id   # PoI id if surveying, empty otherwise
bool connected           # has a live (possibly multi-hop) path to the GCS
int32 hop_count          # -1 if unreachable
uint32 relay_reallocations
string flight_state
bool position_valid
uint32 network_recoveries
float32 last_network_recovery_s
float32 flight_elapsed_s
float32 speed_mps
bool armed
