#!/usr/bin/env bash
# Build the ROS 2 workspace on top of ROS Humble + the px4_msgs underlay.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
cd "$ROOT/ros2_ws"
colcon build --symlink-install "$@"
echo "done. run: source $ROOT/ros2_ws/install/setup.bash"
