#!/usr/bin/env bash
set -e
cd /home/ubuntu/Dragonfly
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
source install-flight/setup.bash
prefix="${1:-log/acceptance_$(date +%Y%m%d_%H%M%S)}"
for round in 1 2 3; do
  ros2 run dragonfly_flight flight --scenario acceptance --record --output "${prefix}_$round"
  ros2 run dragonfly_flight verify "${prefix}_$round"
  python3 src/dragonfly_flight/test/plot_run.py "${prefix}_$round"
done
