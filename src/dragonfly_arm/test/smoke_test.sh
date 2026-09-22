#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not available; source ROS 2 and this workspace first." >&2
  exit 2
fi

smoke_log="${TMPDIR:-/tmp}/dragonfly_arm_smoke_$$.log"
launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 launch dragonfly_arm demo.launch.py gui:=false >"${smoke_log}" 2>&1 &
launch_pid=$!

controllers_ready=false
for _ in {1..60}; do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Launch process exited before the controllers became ready." >&2
    cat "${smoke_log}" >&2
    exit 3
  fi

  controller_output="$(ros2 control list_controllers 2>/dev/null || true)"
  if grep -q "joint_state_broadcaster.*active" <<<"${controller_output}" && \
     grep -q "arm_controller.*active" <<<"${controller_output}"; then
    controllers_ready=true
    break
  fi
  sleep 1
done

if [[ "${controllers_ready}" != true ]]; then
  echo "Controllers did not become active within 60 seconds." >&2
  cat "${smoke_log}" >&2
  exit 4
fi

if ! timeout 10 ros2 topic echo /joint_states --once >/dev/null; then
  echo "No /joint_states message was received." >&2
  cat "${smoke_log}" >&2
  exit 5
fi

trajectory_succeeded=false
for _ in {1..30}; do
  if grep -q "Trajectory completed successfully" "${smoke_log}"; then
    trajectory_succeeded=true
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    break
  fi
  sleep 1
done

if [[ "${trajectory_succeeded}" != true ]]; then
  echo "The demonstration trajectory did not complete successfully." >&2
  cat "${smoke_log}" >&2
  exit 6
fi

echo "Dragonfly arm smoke test passed."
echo "Log: ${smoke_log}"
