#!/usr/bin/env bash
# Start a clean configurable SITL fleet without touching workspace EEPROM files.
set -eo pipefail
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
set -u
show_gui=false
show_map=false
show_dashboard=false
lean_model=true
fleet_size=10
stack_args=()
while (($#)); do
  case "$1" in
    --gui) show_gui=true; shift ;;
    --map) show_map=true; shift ;;
    --dashboard) show_dashboard=true; shift ;;
    --full-model) lean_model=false; shift ;;
    --fleet-size)
      if (($# < 2)); then echo "--fleet-size needs a number" >&2; exit 2; fi
      fleet_size="$2"; shift 2 ;;
    --fleet-size=*) fleet_size="${1#*=}"; shift ;;
    --*)
      echo "Unknown option: $1. Use --gui, --map, --dashboard, --full-model, or --fleet-size N (two hyphens)." >&2
      exit 2 ;;
    *:=*) stack_args+=("$1"); shift ;;
    *) echo "Unexpected argument: $1. ROS launch arguments must use name:=value." >&2; exit 2 ;;
  esac
done
if [[ ! "$fleet_size" =~ ^[0-9]+$ ]]; then
  echo "Fleet size must be an integer from 1 to 25" >&2
  exit 2
fi
fleet_size=$((10#$fleet_size))
if ((fleet_size < 1 || fleet_size > 25)); then
  echo "Fleet size must be between 1 and 25" >&2
  exit 2
fi
uav_names='['
for ((i=1; i<=fleet_size; i++)); do
  if ((i > 1)); then uav_names+=','; fi
  uav_names+="drone$i"
done
uav_names+=']'
export ROS_DOMAIN_ID=30
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  export VIRTUAL_ENV="$HOME/.ros2_venv"
fi
if [[ ! -f "$VIRTUAL_ENV/lib/python3.12/site-packages/pymavlink/__init__.py" ]]; then
  echo "pymavlink is required in $VIRTUAL_ENV" >&2
  exit 1
fi
run_dir=$(mktemp -d /tmp/uavx-sitl-XXXXXX)
mkdir -p "$workspace/logs"
cd "$run_dir"
setsid ros2 launch ardupilot_gz_bringup multiagent.launch.py rviz:=false gui:="$show_gui" fleet_size:="$fleet_size" lean:="$lean_model" \
  > "$workspace/logs/fleet_$(date +%Y%m%d_%H%M%S).log" 2>&1 &
fleet_pid=$!
stack_pid=
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$stack_pid" ]]; then kill -TERM -- "-$stack_pid" 2>/dev/null || true; fi
  kill -TERM -- "-$fleet_pid" 2>/dev/null || true
  # ArduCopter may ignore the first signal after ros2 launch exits.
  for _ in {1..15}; do
    if ! kill -0 -- "-$fleet_pid" 2>/dev/null &&
       { [[ -z "$stack_pid" ]] || ! kill -0 -- "-$stack_pid" 2>/dev/null; }; then
      break
    fi
    sleep 0.2
  done
  if [[ -n "$stack_pid" ]]; then kill -KILL -- "-$stack_pid" 2>/dev/null || true; fi
  kill -KILL -- "-$fleet_pid" 2>/dev/null || true
  if [[ -n "$stack_pid" ]]; then wait "$stack_pid" 2>/dev/null || true; fi
  wait "$fleet_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
printf 'Fresh SITL state: %s (%s drones)\n' "$run_dir" "$fleet_size"
printf 'Waiting for Gazebo and MAVProxy startup...\n'
sleep 12
stack_log="$workspace/logs/swarm_$(date +%Y%m%d_%H%M%S).log"
setsid ros2 launch swarm_stack swarm_stack.launch.py map_view:="$show_map" dashboard:="$show_dashboard" uav_names:="$uav_names" "${stack_args[@]}" \
  > "$stack_log" 2>&1 &
stack_pid=$!
printf 'Fleet and swarm running. Metrics: %s/logs/swarm_metrics_*.csv\n' "$workspace"
printf 'Use a second terminal for fault injection; Ctrl+C stops this run.\n'
printf 'Flight progress log: %s\n' "$stack_log"
if wait "$stack_pid"; then
  exit 0
else
  stack_exit=$?
  echo "Swarm stack exited with status $stack_exit. Recent log:" >&2
  tail -n 25 "$stack_log" >&2
  exit "$stack_exit"
fi
