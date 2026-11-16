#!/usr/bin/env bash
# Flies the simulated drone in Cosys-AirSim (Unreal) and runs positioning_sim,
# the Kalman prediction, against it over ROS 2.
#
#   sim/run.sh [--scenario field|antwerp] [--no-gui] [weather] [positioning_sim options]
#
#   field     (default) open ground at the site of the recorded data
#   antwerp   over Groenplaats in Antwerp's 3D city model
#   --no-gui  Unreal without a window
#   weather   --rain I --snow I --fog I --dust I --leaves I (0..1), --wind SPEED[@FROM_DEG]
#   positioning_sim options: its ROS parameters written --name value
#             (--duration S, --horizon S, --gps-noise M, --manual to fly with
#             localization_core/airsim/keyboard.sh, ...; the list with
#             descriptions: ros2 param describe /positioning_sim <name>, or
#             the top of sim/src/positioning_sim.cpp)
#
# The simulator side is localization_core/airsim (run airsim/setup.sh once);
# ROS 2 runs in the "ros2" distrobox ($LOCSIM_ROS_BOX); the ROS programs are
# started by sim/launch/positioning.launch.py. Watch the ghost in
# rviz2 (fixed frame "enu", topic /positioning/markers).
#
# Each run is recorded into build-ros/runs/positioning-<scenario>-<time>
# (rosbag2: the truth as TF and the estimates) and scored at the end by
# localization_core airsim/evaluate.py (evo); evo's own tools plot it.
# LOCALIZATION_CORE_DIR=../localization_core uses a side by side checkout.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
core=${LOCALIZATION_CORE_DIR:-$root/external/localization_core}
core=$(cd "$core" 2>/dev/null && pwd || true)
if [[ ! -x $core/airsim/run.sh ]]; then
  echo "localization_core with airsim/ not found at ${LOCALIZATION_CORE_DIR:-external/localization_core}," \
    "run: git submodule update --init (or set LOCALIZATION_CORE_DIR)" >&2
  exit 1
fi
box=${LOCSIM_ROS_BOX:-ros2}
# shellcheck source=/dev/null
. "$core/airsim/params.sh"

scenario=field
engine_args=()
args=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --scenario) scenario=$2; shift 2 ;;
    --no-gui|--headless) engine_args+=(--offscreen); shift ;;
    --rain|--snow|--fog|--dust|--leaves|--wind) engine_args+=("$1" "$2"); shift 2 ;;
    *) args+=("$1"); shift ;;
  esac
done
case $scenario in
  field) map=Field ;;
  antwerp) map=Antwerp ;;
  *) echo "unknown scenario $scenario (field, antwerp)" >&2; exit 2 ;;
esac
settings=$core/airsim/settings/positioning_$scenario.json

# Build positioning_sim where ROS 2 is.
distrobox enter "$box" -- bash -c "
  set -e; . /opt/ros/jazzy/setup.bash
  cmake -S '$root' -B '$root/build-ros' -DCMAKE_BUILD_TYPE=Release -DLOCALIZATION_CORE_DIR='$core' >/dev/null
  cmake --build '$root/build-ros' -j --target positioning_sim"

to_ros_params ${args[@]+"${args[@]}"}
bag=$root/build-ros/runs/positioning-$scenario-$(date +%Y%m%d-%H%M%S)
mkdir -p "$(dirname "$bag")"

# Unreal, then sim/launch/positioning.launch.py (AirSim's ROS node,
# positioning_sim and the recorder); the run ends when positioning_sim does.
status=0
"$core/airsim/run.sh" --map "$map" --settings "$settings" ${engine_args[@]+"${engine_args[@]}"} \
  --launch "$root/sim/launch/positioning.launch.py" "settings:=$settings" "record:=$bag" \
  "params:=${ros_params[*]+${ros_params[*]}}" || status=$?

if [[ -d $bag ]]; then
  # evo reads the TF from the recording with ROS's tf2_py.
  distrobox enter "$box" -- bash -c ". /opt/ros/jazzy/setup.bash && exec '$core/airsim/.venv/bin/python' \
    '$core/airsim/evaluate.py' '$bag' 'ghost (prediction)=/positioning/prediction' \
    'baseline (last fix + velocity)=/positioning/baseline' 'Kalman filter, now=/positioning/estimate' \
    'raw GPS=/positioning/gps'" || true
fi
exit "$status"
