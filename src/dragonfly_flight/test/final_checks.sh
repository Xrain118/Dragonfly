#!/usr/bin/env bash
set -e
cd /home/ubuntu/Dragonfly
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
source install-flight/setup.bash
ros2 run dragonfly_flight flight --scenario acceptance --record --output log/effort_acceptance_4
ros2 run dragonfly_flight verify log/effort_acceptance_4
python3 src/dragonfly_flight/test/plot_run.py log/effort_acceptance_4
python3 src/dragonfly_flight/test/probe_dds_loss.py log/dds_loss_verified
