#!/usr/bin/env bash
set -euo pipefail
px4_root="${1:-/home/ubuntu/PX4-Autopilot}"
patch_file="$(cd "$(dirname "$0")/../config/px4_vector" && pwd)/px4_v1.16.2_vector.patch"
expected_commit="$(git -C "$px4_root" rev-parse 'v1.16.2^{commit}')"
if [[ "$(git -C "$px4_root" rev-parse HEAD)" != "$expected_commit" ]]; then
  echo 'Vector patch requires PX4 v1.16.2; refusing to apply to another revision.' >&2
  exit 1
fi
if git -C "$px4_root" apply --reverse --check "$patch_file" 2>/dev/null; then
  echo 'PX4 vector patch already applied.'
else
  git -C "$px4_root" apply --check "$patch_file"
  git -C "$px4_root" apply "$patch_file"
  echo 'PX4 vector patch applied.'
fi
