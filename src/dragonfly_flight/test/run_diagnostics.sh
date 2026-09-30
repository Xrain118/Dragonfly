#!/usr/bin/env bash
set -e
cd /home/ubuntu/Dragonfly
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=75
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
source install-flight/setup.bash
prefix="${1:-log/diagnostics_$(date +%Y%m%d_%H%M%S)}"
for scenario in freefall force_x force_y force_z; do
  ros2 run dragonfly_flight flight --mode ideal_wrench --scenario "$scenario" --output "${prefix}_$scenario"
done
for step in .002 .001 .0005; do
  ros2 run dragonfly_flight flight --mode physics --scenario physics --step "$step" --output "${prefix}_momentum_$step"
done
