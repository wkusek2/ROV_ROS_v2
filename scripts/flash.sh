#!/bin/bash
# Wgrywa firmware do Pico 2 podpietego do Jetsona po USB, bez dotykania BOOTSEL.
# Uzycie: ./flash.sh [fw|bms|plik.uf2] [SERIAL]
#   fw  (domyslnie) -> pico_fw/build/rov_pico.uf2, Pico o numerze seryjnym z PICO_FW_SER
#   bms             -> pico_bms/build/rov_bms.uf2, Pico o numerze seryjnym z PICO_BMS_SER
#   SERIAL          -> numer seryjny USB (z /dev/serial/by-id/usb-Raspberry_Pi_Pico_<SERIAL>-if00); gdy podpiete jest
#                      jedno Pico, mozna pominac.
# Kolejnosc prob restartu do BOOTSEL: picotool (vendor reset) -> 1200 baud -> cykl zasilania portu (uhubctl).
# Zatrzymuje usluge rov (albo launch) i uruchamia ja ponownie na koncu.
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
PICO_FW_SER=${PICO_FW_SER:-A320C46EADFA28CF}
PICO_BMS_SER=${PICO_BMS_SER:-1DBD3353E3ED6781}
WHAT=${1:-fw}
case "$WHAT" in
    fw)  UF2=$HERE/pico_fw/build/rov_pico.uf2; SER=${2:-$PICO_FW_SER} ;;
    bms) UF2=$HERE/pico_bms/build/rov_bms.uf2; SER=${2:-$PICO_BMS_SER} ;;
    *)   UF2=$WHAT; SER=${2:-} ;;
esac
PT=$HOME/nowy_rov/tools/picotool/build/picotool
SUDO="sudo -n"; sudo -n true 2>/dev/null || SUDO="sudo"
[ -x "$PT" ] || { echo "brak picotool: $PT"; exit 1; }
[ -f "$UF2" ] || { echo "brak $UF2"; exit 1; }

NPICO=$(lsusb | grep -c "2e8a:")
if [ "$NPICO" -le 1 ]; then SEL=""; else SEL="--ser $SER"; fi
[ -n "$SER" ] && TTY=$(readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Pico_${SER}-if00 2>/dev/null) || TTY=$(ls /dev/ttyACM* 2>/dev/null | head -1)
echo "== cel: $UF2  serial=${SER:-?}  tty=${TTY:-brak}  (Pico na USB: $NPICO)"

in_bootsel() { $PT info $SEL >/dev/null 2>&1 && lsusb | grep -q "2e8a:000f"; }
wait_bootsel() { for i in $(seq 1 "$1"); do in_bootsel && return 0; sleep 1; done; return 1; }

USE_SYSTEMD=0
if systemctl is-enabled rov.service >/dev/null 2>&1; then USE_SYSTEMD=1; echo "== stop rov.service"; $SUDO systemctl stop rov.service; fi
pkill -f "ros2 launch rov_bringup" 2>/dev/null; sleep 1; pkill -f micro_ros_agent 2>/dev/null; sleep 2; pkill -9 -f micro_ros_agent 2>/dev/null

if ! in_bootsel; then
    echo "== 1. picotool reboot -f -u $SEL"; $PT reboot -f -u $SEL >/dev/null 2>&1
    if ! wait_bootsel 5 && [ -n "${TTY:-}" ] && [ -e "$TTY" ]; then
        echo "== 2. 1200 baud na $TTY"; stty -F "$TTY" 1200 2>/dev/null; wait_bootsel 5
    fi
    if ! in_bootsel && command -v uhubctl >/dev/null; then
        PORT=$($SUDO uhubctl 2>/dev/null | awk '/^Current status for hub/{hub=$5} /2e8a/{print hub, $2}' | head -1)
        if [ -n "$PORT" ]; then set -- $PORT; echo "== 3. cykl zasilania hub $1 port ${2%:}"; $SUDO uhubctl -l "$1" -p "${2%:}" -a cycle -d 3 >/dev/null 2>&1
        else echo "== 3. cykl zasilania portu 1-2 p4 (Pico niewidoczne)"; $SUDO uhubctl -l 1-2 -p 4 -a cycle -d 3 >/dev/null 2>&1; fi
        wait_bootsel 8
    fi
fi
if ! in_bootsel; then
    echo "Pico nie weszlo w BOOTSEL. Odepnij i wepnij USB (lub przytrzymaj BOOTSEL przy wpinaniu) i uruchom ponownie."
    [ "$USE_SYSTEMD" = 1 ] && $SUDO systemctl start rov.service
    exit 1
fi
echo "== load"; $PT load -x $SEL "$UF2" 2>&1 | tail -1
for i in 1 2 3 4 5 6 7 8; do sleep 1; ls /dev/ttyACM* >/dev/null 2>&1 && break; done
ls -l /dev/serial/by-id/ 2>/dev/null | awk '{print $9, $11}'
if [ "$USE_SYSTEMD" = 1 ]; then
    $SUDO systemctl start rov.service; echo "FLASH_OK, rov.service wystartowany (journalctl -u rov -f)"
else
    set +u; source "$HERE/scripts/env.sh"
    nohup setsid ros2 launch rov_bringup rov.launch.py > "$HOME/nowy_rov/logs/stack.log" 2>&1 < /dev/null &
    echo "FLASH_OK, stos uruchomiony (log: ~/nowy_rov/logs/stack.log)"
fi
