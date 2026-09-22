# Current state of a single UAV in the swarm, published on /swarm/uav_status.
string name
geometry_msgs/Point position
float32 battery_pct
string role              # "idle" | "surveyor" | "relay" | "rtl" | "failed"
string current_task_id   # PoI id if surveying, empty otherwise
bool connected           # has a live (possibly multi-hop) path to the GCS
int32 hop_count          # -1 if unreachable
