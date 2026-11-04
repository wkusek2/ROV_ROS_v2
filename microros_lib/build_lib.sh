#!/bin/bash
# Buduje libmicroros.a (Humble) dla RP2350 na Jetsonie przy pomocy micro_ros_setup.
# Wynik: ~/nowy_rov/ROV_ROS2/microros_lib/libmicroros/{libmicroros.a,include/}
# Wymaga: ~/nowy_rov/ros2_ws z micro_ros_setup (robi to tools/setup_tools.sh) i toolchainu Arm 13.2.
# Uzycie: ./build_lib.sh            (pelna budowa, ~15-30 min na Orinie)
#         ./build_lib.sh rebuild    (tylko ponowne build_firmware.sh, np. po zmianie rov_msgs)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
export PICO_TOOLCHAIN_PATH=${PICO_TOOLCHAIN_PATH:-$HOME/nowy_rov/tools/arm-gnu-toolchain-13.2.Rel1-aarch64-arm-none-eabi}
export PATH=$PICO_TOOLCHAIN_PATH/bin:$PATH
WS=$HOME/nowy_rov/ros2_ws

source /opt/ros/humble/setup.bash
source $WS/install/local_setup.bash
cd $WS

if [ "$1" != "rebuild" ] || [ ! -d firmware/mcu_ws ]; then
    rm -rf firmware
    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
fi

# nasze wiadomosci do biblioteki (typesupport generowany dla MCU)
rm -rf firmware/mcu_ws/extra_packages/rov_msgs
mkdir -p firmware/mcu_ws/extra_packages
cp -R "$ROOT/ros2_ws/src/rov_msgs" firmware/mcu_ws/extra_packages/

rm -rf firmware/build
ros2 run micro_ros_setup build_firmware.sh "$HERE/toolchain.cmake" "$HERE/colcon.meta"

OUT="$HERE/libmicroros"
rm -rf "$OUT"; mkdir -p "$OUT/include"
find firmware/build/include/ -name "*.c" -delete
cp -R firmware/build/include/* "$OUT/include"
cp firmware/build/libmicroros.a "$OUT/libmicroros.a"

# Humble: naglowki leza w include/<pkg>/<pkg>/... - splaszcz do include/<pkg>/...
pushd firmware/mcu_ws > /dev/null
PKGS=$(colcon list | awk '{print $1}')
popd > /dev/null
for p in $PKGS; do
    if [ -d "$OUT/include/$p/$p" ]; then
        rsync -a "$OUT/include/$p/$p/" "$OUT/include/$p/"
        rm -rf "$OUT/include/$p/$p"
    fi
done
arm-none-eabi-readelf -A "$OUT/libmicroros.a" 2>/dev/null | grep -m3 -E "Tag_CPU_arch|Tag_ABI_VFP_args|Tag_FP_arch" || true
echo "LIB_OK: $OUT/libmicroros.a ($(du -h "$OUT/libmicroros.a" | cut -f1))"
