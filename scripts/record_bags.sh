#!/bin/bash
# Wrapper - wlasciwy skrypt (instalowany z pakietem i odpalany z launcha) jest w ros2_ws/src/rov_bringup/scripts/.
exec "$(dirname "$(readlink -f "$0")")/../ros2_ws/src/rov_bringup/scripts/record_bags.sh" "$@"
