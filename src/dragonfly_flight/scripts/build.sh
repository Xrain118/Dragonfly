#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
export PYTHONNOUSERSITE=1
# ROS setup scripts may read variables that have not been set yet.
set +u
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
set -u
test "$(git -C /home/ubuntu/px4_ros2_ws/src/px4_msgs rev-parse HEAD)" = 392e831c1f659429ca83902e66820d7094591410
test "$(git -C /home/ubuntu/Micro-XRCE-DDS-Agent describe --tags --always)" = v2.4.3
patch_file="$PWD/src/dragonfly_flight/config/px4_mag_enu.patch"
if ! git -C /home/ubuntu/PX4-Autopilot apply --reverse --check "$patch_file" 2>/dev/null; then
  git -C /home/ubuntu/PX4-Autopilot apply --check "$patch_file"
  git -C /home/ubuntu/PX4-Autopilot apply "$patch_file"
fi
bash "$PWD/src/dragonfly_flight/scripts/apply_px4_vector.sh" /home/ubuntu/PX4-Autopilot
make -C /home/ubuntu/PX4-Autopilot px4_sitl_default
colcon build --build-base build-flight --install-base install-flight --symlink-install \
  --packages-select dragonfly_arm dragonfly_flight
