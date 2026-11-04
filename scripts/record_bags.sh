#!/bin/bash
# Ciagle nagrywanie wszystkich topikow ROS 2 do bagow (do analizy problemow). Uruchamiane z launcha.
#   katalog: ~/nowy_rov/logs/bags/<data_czas>/, podzial pliku co 10 min, bez /rov/camera/compressed (obraz nagrywa sie osobno)
#   sprzatanie: najstarsze bagi kasowane, gdy katalog przekroczy MAX_GB
# Odtwarzanie: ros2 bag info <katalog>;  ros2 bag play <katalog>;  albo rov bags
set +u
BAGS=${BAGS:-$HOME/nowy_rov/logs/bags}
MAX_GB=${MAX_GB:-6}
EXCLUDE='/rov/camera/compressed|/client_count|/connected_clients|/parameter_events'
mkdir -p "$BAGS"
source /opt/ros/humble/setup.bash
source "$HOME/nowy_rov/ros2_ws/install/setup.bash"

cleanup() {
    local used
    used=$(du -sm "$BAGS" 2>/dev/null | cut -f1)
    while [ "${used:-0}" -gt $((MAX_GB * 1024)) ]; do
        old=$(ls -1d "$BAGS"/*/ 2>/dev/null | head -1)
        [ -z "$old" ] && break
        # nie kasuj katalogu, do ktorego aktualnie piszemy
        [ "$old" = "$CUR/" ] && break
        echo "bags: usuwam $old (zajete ${used} MB > ${MAX_GB} GB)"
        rm -rf "$old"
        used=$(du -sm "$BAGS" 2>/dev/null | cut -f1)
    done
}

( while true; do sleep 300; cleanup; done ) &
CLEANER=$!
trap 'kill $CLEANER 2>/dev/null; kill $REC 2>/dev/null; wait; exit 0' INT TERM

while true; do
    CUR="$BAGS/$(date +%Y%m%d_%H%M%S)"
    cleanup
    ros2 bag record -a -x "$EXCLUDE" -o "$CUR" --max-bag-duration 600 &
    REC=$!
    wait $REC
    echo "bags: ros2 bag record zakonczyl sie (kod $?), restart za 5 s"
    sleep 5
done
