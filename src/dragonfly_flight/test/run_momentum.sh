#!/usr/bin/env bash
set -e
cd /home/ubuntu/Dragonfly
export ROS_DOMAIN_ID=75
prefix="${1:-log/momentum_$(date +%Y%m%d_%H%M%S)}"
for step in .002 .001 .0005; do
  bash src/dragonfly_flight/scripts/sim.sh --mode physics --scenario physics --step "$step" --output "${prefix}_$step"
done
