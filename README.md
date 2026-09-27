# UAV-X resilient BVLOS swarm simulation

This workspace has two views of the sample challenge. The **10-aircraft Gazebo/ArduCopter SITL demo** autonomously requests Guided mode, arms, takes off from separated pads, assigns survey and relay roles, sends camera reports through a modeled radio network, and returns aircraft for landing. The **10-aircraft Python reference simulation** covers the configured 500 × 500 m area with idealized motion and synthetic reports. Its dashboard shows aircraft, PoIs, trails, radio links, and relay recovery in motion.

## Set up a fresh clone on Ubuntu 24.04

Install [ROS 2 Jazzy Desktop](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) on Ubuntu 24.04 first. Use the linked official instructions to enable their package repositories. The reference simulation alone needs Python 3, PyYAML, and Tkinter; it does not need ROS or Gazebo.

```bash
git clone https://github.com/sachinmandal3580-rgb/ardu_ws.git
cd ardu_ws
src/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-venv python3-tk python3-yaml \
  ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-gps-msgs \
  ros-jazzy-robot-state-publisher ros-jazzy-topic-tools ros-jazzy-sdformat-urdf
python3 -m venv --system-site-packages ~/.ros2_venv
source ~/.ros2_venv/bin/activate
python3 -m pip install pymavlink MAVProxy
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
colcon build
```

The ArduPilot prerequisite script installs its SITL build tools; see the [ArduPilot build setup](https://ardupilot.org/dev/docs/building-setup-linux.html). This guide uses `~/.ros2_venv`, which is the launcher’s default virtual-environment path for checking `pymavlink`. Activate it in each terminal used to launch SITL so `mavproxy.py` is on `PATH`. The first `colcon` build also compiles ArduCopter and the Gazebo plugin, so it can take considerably longer than later builds. After a successful build, use the run commands below from the `ardu_ws` directory; `scripts/run_swarm_demo.sh` sources `install/setup.bash` itself.

Check the setup before launching the fleet:

```bash
source ~/.ros2_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix swarm_stack
ros2 pkg prefix ardupilot_gz_bringup
python3 -c 'import pymavlink; print("pymavlink OK")'
command -v mavproxy.py
```

## Sample scenario

| Scenario setting | Value used |
| --- | ---: |
| Mission and landing deadline | 45 min |
| Flight time per UAV | 20 min |
| Maximum radio hop | 100 m in 3D |
| Operational area | 500 × 500 m |
| Operational center | 75 m outside the left edge |
| Maximum altitude | 100 m |
| Maximum speed | 5 m/s |
| Minimum vehicle separation | 20 m |
| Detection to GCS report | 10 s |
| PoIs | 10 random positions; one at start, nine random spawn times |

The slide leaves the PoI spawn interval, fleet size, relay spacing, recharge time, and aircraft dynamics unspecified. These assumptions are explicit in [sample_scenario.yaml](src/swarm_stack/config/sample_scenario.yaml). The 10-aircraft reference model uses a fixed random seed for repeatable runs, an 80 m relay chain, and a 120 s simulated recharge. One PoI appears at mission start so the live map is immediately populated; the other nine have random spawn times.

## See the swarm move now

From the cloned `ardu_ws` directory, run the Python dashboard without Gazebo or ROS:

```bash
PYTHONPATH=src/swarm_stack /usr/bin/python3 -m swarm_stack.scenario_viewer
```

The title says **KINEMATIC MODEL**. Click aircraft or PoIs on the map or in the side lists to inspect them. Use Follow to track a selected aircraft, Center selection to locate it, and the radio range, trail, and link buttons to control map layers. Scroll to zoom around the pointer, drag to pan, or press `F` to fit the area. The PoI list filters pending and reported tasks. Use the speed buttons, pause, and `+1 min` / `+5 min` to move through the reference run; the scenario buttons add a priority-10 PoI, fail a relay, or degrade a link. Aircraft colors show surveyors, relays, returns, and charging. This run uses synthetic reports and idealized motion; it is a planning demonstration, not flight validation.

To export a full 45-minute reference run with per-PoI times and events:

```bash
PYTHONPATH=src/swarm_stack /usr/bin/python3 -m swarm_stack.reference_sim --output logs/reference_sample.json
```

For the seeded 10-aircraft run, all 10 PoIs were reported and all 10 aircraft landed. The reference planner can assign multiple surveyors at once when their relay paths fit within the fleet. For reference seed 7, the measured maximum detection-to-report time was 1 s, longest sortie 417 s, minimum separation 20.10 m, maximum altitude 70 m, and maximum speed 5 m/s. No modeled limit violation was recorded. In a sweep of random seeds 0–9, the 10-aircraft model reported all 10 PoIs and landed all aircraft with no recorded limit violation in every run. These numbers apply to the idealized Python model only.

## Run Gazebo and the live map

After the fresh-clone setup above, open a terminal in `ardu_ws` and run the 10-aircraft Gazebo simulation with its live dashboard:

```bash
source ~/.ros2_venv/bin/activate
bash scripts/run_swarm_demo.sh --gui --dashboard
```

To open **Gazebo and RViz only** (no Python dashboard), run:

```bash
bash scripts/run_swarm_demo.sh --gui --map
```

Gazebo shows the physical drone models. RViz shows the 500 × 500 m mission map, launch pads, drone icons, the first PoI from the start, later PoIs as Gazebo time advances, trails, and radio links. The RViz drone meshes are enlarged map icons so they remain visible at arena scale. In RViz, select **Move Camera** on the toolbar. Use the mouse wheel or right-button drag to zoom; middle-button drag or Shift + left-button drag pans. The **Views** panel exposes the map scale and center. Wait for telemetry before the pad icons change from `WAITING FOR TELEMETRY` to live roles. Physical PoI objects are not spawned in Gazebo.

Takeoff now proceeds concurrently from the separated pads once each aircraft passes its own pre-arm checks. A failing aircraft on another clear pad does not hold up the fleet. The RViz labels show `arm`, `ascending`, and the active role. Startup time depends on Gazebo's real time factor: 30 simulated seconds take about five wall-clock minutes at RTF 0.10. Pre-arm failures must clear before takeoff; a fixed wall-clock wait cannot guarantee readiness. The launcher prints the flight progress log path and rejects unknown flags before launching.

Aircraft serve either survey or relay roles. Several PoIs can have surveyors simultaneously when enough connected, energy-safe teams fit in the fleet. Outbound aircraft transit above deployed relays before descending to survey altitude.

The default is **10 drones** (eight needed for the farthest relay chain plus two spares). For a headless Gazebo run with the live dashboard:

```bash
bash scripts/run_swarm_demo.sh --dashboard
```

Add `--gui --map` to a `--dashboard` run if you want all three windows. The swarm launcher uses a smaller ROS bridge configuration with odometry, battery, and lazy camera topics for every fleet size. The lean fleet also publishes odometry at 10 Hz instead of Gazebo's 50 Hz default, uses a 320 × 240 camera at 5 Hz, and omits the unused GStreamer and zoom plugins. The dashboard redraws at 10 Hz in live mode. The script gives all vehicles unique ports and launch pads at least 25 m apart. The live mission manager allows concurrent vertical takeoff from separated, clear launch columns and subscribes to camera frames for each active surveyor. The Gazebo battery load is 90 W for the 3.5 Ah modeled pack. The manager still enforces the sample 20 minute maximum flight time per sortie and returns a drone with an energy reserve. Landed drones recharge at their pads and can rejoin when pending PoIs need them. When two or more PoIs are available, it assigns separate surveyors if it can provide connected relay paths and maintain the 20 m separation and energy reserves; overlapping routes share relays. Otherwise, the extra task stays pending. Emergency PoIs get first priority. The live dashboard shows Gazebo time, wall time, and real time factor; a slow Gazebo clock means flight and PoI spawns also progress slowly.

The `--dashboard` flag is optional and works with or without Gazebo and RViz windows. The launch script sets `ROS_DOMAIN_ID=30`, starts a fresh SITL state under `/tmp`, and writes `logs/fleet_*.log`, `logs/swarm_*.log`, and `logs/swarm_metrics_*.csv`. Press Ctrl+C to stop. Gazebo uses the runway world; the arena, PoIs, launch area, trails, and radio chain are shown on the dashboard and RViz mission map. PoIs are mission markers, not physical Gazebo objects.

For the lowest CPU load, run `bash scripts/run_swarm_demo.sh` without `--gui`, `--map`, or `--dashboard`. The default lean world uses a simple ground plane without the textured airfield or shadow rendering; `--full-model` restores the original world and full ROS bridge. Add only `--dashboard` when you need a live view. The 10 ArduCopter SITL instances and Gazebo physics still require substantial CPU; use `--fleet-size 3` for a quick integration check, then use 10 for a fleet run. Smaller fleets cannot demonstrate the complete 500 × 500 m relay coverage. Keep the 1 ms Gazebo physics step: slower physics caused ArduPilot gyro/main-loop pre-arm failures in testing.

For ROS commands in another terminal, start in the cloned `ardu_ws` directory:

```bash
source ~/.ros2_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 topic echo /swarm/uav_status
```

The live dashboard labels itself **LIVE GAZEBO** and subscribes to ROS telemetry. Its map selection, Follow, filters, and layer controls remain available; pause, speed, time jump, and scenario actions are available only in the reference model. Its warning says when the selected fleet cannot span the arena. The sample scenario is the default for this launch; the legacy hand-placed [pois.yaml](src/swarm_stack/config/pois.yaml) is used only when the scenario path is explicitly set to empty.

## Relay and task demonstrations

After aircraft become airborne, inject a simulated failure or a new PoI. The fault changes the logical radio and mission model; it does not stop an ArduPilot process or physically damage a radio.

```bash
ros2 run swarm_stack inject_fault --event_type uav_failure --target drone2
ros2 run swarm_stack inject_fault --event_type link_degraded --target drone1:drone3
ros2 run swarm_stack inject_fault --event_type link_restored --target drone1:drone3
ros2 run swarm_stack inject_fault --event_type new_poi --target emergency_1 --x 125 --y 25 --priority 10
```

The live stack publishes `/swarm/connectivity_graph`, `/swarm/poi_updates`, `/swarm/gcs/survey_image`, and `/swarm/communication_stats`. The source in `src/swarm_stack/swarm_stack` contains the mission manager, communications model, and metrics logger.

## Model limits

The link graph enforces the 100 m radio cutoff, but ROS 2/DDS and MAVLink still carry simulator control and telemetry outside that modeled mesh. The mission manager is centralized. The Python reference has ideal motion, instant model state, synthetic reports, and simulated charging. Gazebo can launch the ten-aircraft fleet with real SITL dynamics and camera frames. A full 45-minute Gazebo mission has not been validated; read its measured safety and completion results from its own metrics rather than inferring them from the Python run. The Gazebo battery model supports pad recharge after landing; its 120 s minimum recharge wait and charging rate are simulation assumptions. Gazebo has no general obstacle avoidance.
