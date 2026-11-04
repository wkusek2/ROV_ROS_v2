#!/bin/bash
# Ciagle nagrywanie WSZYSTKICH topikow ROS 2 do bagow (pelny zapis pojazdu do analizy). Uruchamiane z launcha.
#   katalog: ~/nowy_rov/logs/bags/<data_czas>/, podzial pliku co 10 min, format MCAP (odporny na odciecie zasilania,
#   czytelny w Foxglove/PlotJuggler; fallback sqlite3 gdy brak ros-humble-rosbag2-storage-mcap)
#   bez /rov/camera/compressed (obraz: scripts/pc_record.py na PC albo /rov/camera/record na Jetsonie)
#   <bag>/meta/: konfiguracja, parametry wezlow, wersje (git, md5 firmware), stan zegara - stan pojazdu w chwili startu
#   sprzatanie: najstarsze bagi kasowane, gdy katalog przekroczy MAX_GB (launch: bag_max_gb:=...)
# Analiza: ros2 run rov_tools rov_bag report <katalog>;  rov_bag export <katalog>  (CSV na kazdy topik)
set +u
BAGS=${BAGS:-$HOME/nowy_rov/logs/bags}   # na laptopie: ROV_BAGS=/sciezka
BAGS=${ROV_BAGS:-$BAGS}
MAX_GB=${MAX_GB:-6}
EXCLUDE='/rov/camera/compressed'
mkdir -p "$BAGS"
# srodowisko dziedziczone z launcha; samodzielne uruchomienie na Jetsonie -> zrodla ROS
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
    [ -f "$HOME/nowy_rov/ros2_ws/install/setup.bash" ] && source "$HOME/nowy_rov/ros2_ws/install/setup.bash"
fi
QOS="$(dirname "$(readlink -f "$0")")/../config/bag_qos.yaml"
[ -f "$QOS" ] || QOS=$(ros2 pkg prefix rov_bringup 2>/dev/null)/share/rov_bringup/config/bag_qos.yaml
QOS_ARGS=(); [ -f "$QOS" ] && QOS_ARGS=(--qos-profile-overrides-path "$QOS")
STORAGE=sqlite3
ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1 && STORAGE=mcap

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

# snapshot stanu pojazdu do <bag>/meta (po starcie recordera, bo ros2 bag record tworzy katalog sam)
snapshot() {
    local m="$1/meta"
    for _ in $(seq 1 20); do [ -d "$1" ] && break; sleep 0.5; done
    mkdir -p "$m"
    sleep 8   # wezly musza wstac, zeby dalo sie zrzucic parametry
    {
        echo "host: $(hostname)"; echo "start: $(date -Is)"; echo "storage: $STORAGE"
        echo "uptime: $(cut -d' ' -f1 /proc/uptime) s"
    } > "$m/info.txt"
    cp -r "$HOME/nowy_rov/config" "$m/user_config" 2>/dev/null
    for pkg in rov_control rov_bringup; do
        d=$(ros2 pkg prefix "$pkg" 2>/dev/null)/share/$pkg/config
        [ -d "$d" ] && cp -rL "$d" "$m/${pkg}_config"
    done
    for src in "$HOME/nowy_rov/ROV_ROS2" "$(dirname "$(readlink -f "$0")")/.."; do
        if git -C "$src" rev-parse HEAD >/dev/null 2>&1; then
            { git -C "$src" log -1 --format='%H %ci %s'; git -C "$src" status --short; } > "$m/git.txt" 2>&1
            git -C "$src" diff > "$m/git_diff.patch" 2>/dev/null
            break
        fi
    done
    md5sum "$HOME"/nowy_rov/ROV_ROS2/pico_fw/build/*.uf2 "$HOME"/nowy_rov/ROV_ROS2/pico_bms/build/*.uf2 > "$m/firmware_md5.txt" 2>/dev/null
    ros2 node list > "$m/nodes.txt" 2>&1
    ros2 topic list -t > "$m/topics.txt" 2>&1
    while read -r n; do
        [ -z "$n" ] && continue
        timeout 10 ros2 param dump "$n" > "$m/params_$(echo "${n#/}" | tr '/' '_').yaml" 2>/dev/null
    done < <(grep -vE '^/(rosbag2|_ros2cli|launch_ros)' "$m/nodes.txt")
    { chronyc tracking 2>/dev/null || timedatectl 2>/dev/null; } > "$m/clock.txt"
}

( while true; do sleep 300; cleanup; done ) &
CLEANER=$!
# zamykanie: SIGINT do recordera (tylko na SIGINT domyka plik MCAP i pisze metadata.yaml), czekamy az skonczy
trap 'kill $CLEANER $SNAP 2>/dev/null; kill -INT $REC 2>/dev/null; wait $REC; exit 0' INT TERM

while true; do
    CUR="$BAGS/$(date +%Y%m%d_%H%M%S)"
    cleanup
    echo "bags: $CUR ($STORAGE, limit ${MAX_GB} GB)"
    ros2 bag record -a -x "$EXCLUDE" -o "$CUR" -s "$STORAGE" --max-bag-duration 600 --include-hidden-topics "${QOS_ARGS[@]}" &
    REC=$!
    snapshot "$CUR" &
    SNAP=$!
    wait $REC
    echo "bags: ros2 bag record zakonczyl sie (kod $?), restart za 5 s"
    sleep 5
done
