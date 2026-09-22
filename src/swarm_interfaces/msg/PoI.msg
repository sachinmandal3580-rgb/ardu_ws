# A single Point of Interest to be surveyed by the swarm.
string id
geometry_msgs/Point position
int32 priority          # higher = more urgent
bool surveyed
string surveyed_by       # name of the UAV that surveyed it, empty if not yet surveyed
