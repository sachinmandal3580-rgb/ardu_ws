# swarm_interfaces + swarm_stack — starter skeleton for UAV-X Stage 1

This is a working baseline, not a finished solution. It compiles and runs
against your existing `ardupilot_gz_bringup multiagent.launch.py` fleet, and
implements the minimum logic needed to demonstrate every scored criterion:
task allocation, comms-aware relay reassignment, fault recovery, and metrics
logging. Expect to tune the link model, allocation algorithm, and thresholds
against your own PoI layout and world file before your demo.

## 1. Install into your existing workspace

Copy both folders into your existing `~/ardu_ws/src/` (do NOT create a new
workspace — this must build alongside your already-working ardupilot_gz,
ardupilot_sitl, and micro_ros_agent packages):

```bash
cp -r swarm_interfaces swarm_stack ~/ardu_ws/src/
```

## 2. Build

```bash
cd ~/ardu_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select swarm_interfaces swarm_stack
source install/setup.bash
```

If `rosdep` can't resolve `ardupilot_msgs`, that's expected if it's not
registered as a system rosdep key in your setup — it will still resolve at
build time via colcon's own package discovery as long as `ardupilot_msgs` is
built in the same workspace (which it already is, from your ardupilot_gz work).

## 3. Run

**Terminal 1** — your existing fleet (edit `robots` list for your drone count first):
```bash
cd ~/ardu_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ardupilot_gz_bringup multiagent.launch.py rviz:=false gui:=false
```

Wait until all drones report `Waiting for connection` / connect successfully.

**Terminal 2** — the swarm stack (comms sim + mission manager + metrics logger):
```bash
cd ~/ardu_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch swarm_stack swarm_stack.launch.py \
  uav_names:="['drone1','drone2','drone3','drone4','drone5']"
```

Make sure `uav_names` here matches exactly the `robots` list in your
`multiagent.launch.py` — this is the single most common source of "nothing
is happening" if you've changed the fleet size or names.

Watch Terminal 2's output: you should see `Assigned droneN -> poi_M` lines
as PoIs get allocated, and (once you inject a fault or a drone naturally
loses range) `disconnected ... reassigned to relay` lines.

## 4. Watch the metrics accumulate

```bash
tail -f ~/ardu_ws/swarm_metrics.csv
```

## 5. Trigger a fault for your demonstration video

```bash
ros2 run swarm_stack inject_fault --event_type uav_failure --target drone3
```

Watch Terminal 2 for the reassignment/recovery log lines, and check
`swarm_metrics.csv`'s `relay_reallocations` column ticking up.

## 6. Things you should tune before submitting

- **`config/pois.yaml`** — replace with PoI positions matching your actual
  world/runway layout; the example values are placeholders.
- **`comms_sim_node.py` link model** (`range_full_m`, `range_max_m`,
  `base_latency_ms`) — set these to match the organizers' standardised
  communication assumptions once published, and document the values you
  chose in report §5.1.
- **`mission_manager_node.py` `MAX_HOP_COUNT_OK`, `LOW_BATTERY_PCT`,
  `SURVEY_RADIUS_M`** — tune against your actual world scale and drone speed.
- **`ORIGIN_LAT` / `ORIGIN_LON`** in `mission_manager_node.py` — verify these
  match the "Home: <lat> <lon>" line each drone actually prints on boot in
  your setup; if your world file changes ArduPilot's default SITL origin,
  update these two constants accordingly.
- **Task allocation algorithm** — currently a simple greedy-nearest baseline
  (documented as such is fine for Stage 1). A real coverage/TSP-style planner
  is a good Stage-2 upgrade, not a Stage-1 requirement.

## Known limitations to disclose in your report

- The comms link model is a synthetic distance-threshold model, not a
  physical RF propagation model — state this explicitly, since the
  organizers may supply their own model in Stage 2.
- `mission_manager_node`'s reaction to `new_poi` fault events currently only
  logs the event; wiring it to actually insert a new PoI into the live list
  is a small extension (append to `self.pois` inside `_fault_cb`) left for
  you to add once you're ready to demonstrate dynamic re-tasking specifically.
- Collision avoidance is currently observation-only (the metrics logger flags
  unsafe-separation ticks) — there is no active avoidance controller. If you
  want it, the cleanest addition is a repulsive-velocity term layered on top
  of `_send_goal` in `mission_manager_node.py`.
