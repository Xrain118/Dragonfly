#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../../.."
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
source install-flight/setup.bash
if [[ "${1:-}" == keyboard ]]; then
  exec ros2 run dragonfly_flight keyboard
fi
exec ros2 run dragonfly_flight flight "$@"
