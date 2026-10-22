#!/bin/bash
# Runs the demo robot: the simulation with Nav2, the Osiris bridge, a patrol
# that keeps the robot moving, and the clean-up of recordings. When the
# simulation, the bridge or the patrol stops, this exits, and Fly starts the
# machine again from a fresh disk.
source /opt/ros/jazzy/setup.bash
set -u
# Job control: without it, what a script starts in the background ignores
# SIGINT, the signal that stops a recording cleanly.
set -m
# Without OSIRIS_WS_URL the bridge would connect to Osiris Cloud's public
# address instead of the gateway's private one.
: "${OSIRIS_WS_URL:?}" "${OSIRIS_AUTH_TOKEN:?}"
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:99
log() { echo "[demo] $*"; }

# Gazebo's processes find each other by multicast, here on loopback only.
ip link set lo multicast on
Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp &

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False &
SIM=$!
nodes=0
for _ in $(seq 120); do
  nodes=$(ros2 node list 2>/dev/null | wc -l)
  [ "$nodes" -ge 20 ] && break
  sleep 2
done
log "simulation up with $nodes nodes"
python3 /demo/initial_pose.py || { log "AMCL never took the robot's pose"; exit 1; }

osiris --ros-args --params-file /demo/osiris_params.yaml &
BRIDGE=$!
python3 /demo/patrol.py &
PATROL=$!
/demo/clean_bags.sh &

wait -n "$SIM" "$BRIDGE" "$PATROL"
log "the simulation, the bridge or the patrol stopped (exit $?); starting over"
exit 1
