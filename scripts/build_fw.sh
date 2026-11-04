#!/bin/bash
# Buduje firmware Pico 2 na Jetsonie. Uzycie: ./build_fw.sh [fw|bms] [ram]
#   fw  (domyslnie): pico_fw/build/rov_pico.uf2   - Pico sterujace ESC/IMU/cisnienie
#   bms            : pico_bms/build/rov_bms.uf2   - Pico z CAN do BMS Daly
#   ram            : obraz do SRAM (build_ram)
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS=$HOME/nowy_rov/tools
export PICO_SDK_PATH=${PICO_SDK_PATH:-$TOOLS/pico-sdk}
export PICO_TOOLCHAIN_PATH=${PICO_TOOLCHAIN_PATH:-$TOOLS/arm-gnu-toolchain-13.2.Rel1-aarch64-arm-none-eabi}
export PATH=$PICO_TOOLCHAIN_PATH/bin:$PATH
PICOTOOL_CFG=$TOOLS/picotool/install/lib/cmake/picotool

DIR=pico_fw; NAME=rov_pico; B=build; EXTRA=""
for a in "$@"; do
    case "$a" in
        bms) DIR=pico_bms; NAME=rov_bms ;;
        fw)  DIR=pico_fw;  NAME=rov_pico ;;
        ram) B=build_ram; EXTRA="-DRAM_BUILD=ON" ;;
        *) echo "nieznany argument: $a"; exit 1 ;;
    esac
done
cd "$HERE/$DIR"
cmake -S . -B $B -G Ninja -DPICO_BOARD=pico2 -Dpicotool_DIR=$PICOTOOL_CFG $EXTRA
ninja -C $B
arm-none-eabi-size $B/$NAME.elf
echo "FW_OK: $HERE/$DIR/$B/$NAME.uf2"
