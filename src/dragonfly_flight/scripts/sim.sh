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
if [[ "${1:-}" == monitor ]]; then
  layout="$(ros2 pkg prefix dragonfly_flight --share)/config/plotjuggler.xml"
  # PlotJuggler 3.17.2 restores these saved values AFTER its command-line options.
  # Preserve other preferences, but enforce the monitor's buffer and subscriber.
  python3 - <<'PY'
from PyQt5.QtCore import QSettings
settings = QSettings(QSettings.IniFormat, QSettings.UserScope,
                     'PlotJuggler', 'io.plotjuggler.PlotJuggler')
settings.setValue('MainWindow.streamingBufferValue', 300)
settings.setValue('MainWindow.previousStreamingPlugin', 'ROS2 Topic Subscriber')
settings.sync()
if settings.status() != QSettings.NoError:
    raise SystemExit('Unable to save PlotJuggler monitor preferences')
PY
  echo 'Monitor: choose Yes in Start Streaming, then OK for the selected Dragonfly topics.'
  # WSLg/Mesa crashes in the OpenGL paint path; use the native raster canvas.
  # The layout starts the ROS2 subscriber. --start_streamer would toggle it OFF.
  exec ros2 run plotjuggler plotjuggler --nosplash --disable_opengl \
    --layout "$layout" --buffer_size 300
fi
exec ros2 run dragonfly_flight flight "$@"
