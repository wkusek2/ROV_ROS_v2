#!/bin/bash
# Kopiuje ROV_ROS2 z Windows na Jetsona do ~/nowy_rov/ROV_ROS2 (bez katalogow build i zbudowanej libmicroros).
# Uzycie (Git Bash): ./scripts/sync_to_jetson.sh
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
HOST=${JETSON:-orin@192.168.1.108}
KEY=${JETSON_KEY:-$HOME/.ssh/jetson}
cd "$HERE"
tar czf - --exclude='./pico_fw/build' --exclude='./pico_fw/build_ram' --exclude='libmicroros' --exclude='__pycache__' --exclude='.vscode' . \
  | ssh -i "$KEY" "$HOST" 'mkdir -p ~/nowy_rov/ROV_ROS2 && tar xzmf - -C ~/nowy_rov/ROV_ROS2 && chmod +x ~/nowy_rov/ROV_ROS2/scripts/*.sh ~/nowy_rov/ROV_ROS2/microros_lib/*.sh && echo SYNC_OK'
