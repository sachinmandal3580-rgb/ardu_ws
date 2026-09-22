# The full live connectivity graph, published every tick by comms_sim_node.
string[] uav_names
int32[] hop_counts        # parallel array to uav_names; hop count to GCS, -1 if unreachable
ConnectivityEdge[] edges
