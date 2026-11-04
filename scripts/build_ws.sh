#!/bin/bash
# Buduje pakiety ROS 2 na Jetsonie (rov_msgs, rov_tools, rov_bringup) w ~/nowy_rov/ros2_ws
# (tam gdzie micro_ros_setup zbudowal agenta). Pakiety sa dowiazane symbolicznie z ROV_ROS2/ros2_ws/src.
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
WS=$HOME/nowy_rov/ros2_ws
source /opt/ros/humble/setup.bash
mkdir -p $WS/src
for p in rov_msgs rov_tools rov_control rov_camera rov_bringup; do ln -sfn "$HERE/ros2_ws/src/$p" $WS/src/$p; done
cd $WS
colcon build --packages-select rov_msgs rov_tools rov_control rov_camera rov_bringup --symlink-install
echo "WS_OK: source $WS/install/setup.bash"
