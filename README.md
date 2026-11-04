# ROV_ROS2 - Pico 2 (micro-ROS) + Jetson Orin (ROS 2 Humble)

Nowy software ROV od zera. Pico 2 (RP2350) jest pelnoprawnym wezlem ROS 2 (micro-ROS, Humble)
po USB CDC, Jetson uruchamia agenta micro-ROS i reszte stosu ROS 2.

```
 Jetson Orin (Ubuntu 22.04, JetPack 6 / R36.5, ROS 2 Humble)
   micro_ros_agent serial /dev/ttyACM0  <--USB CDC-->  Pico 2 (wezel /rov_pico)
   ros2 run rov_tools rov ...                             core0: micro-ROS, IMU, cisnienie, KISS, ADC
                                                          core1: 8x bidir DShot 1 kHz (PIO)
```

## Topiki i serwisy (wezel `/rov_pico`)

| Nazwa | Typ | Kierunek | Hz | Uwagi |
|---|---|---|---|---|
| `/rov/imu` | sensor_msgs/Imu | pub | 50 | LSM6DSO32, kwaternion z filtru Mahony, uklad x=przod y=lewo z=gora, rad/s, m/s2 |
| `/rov/imu/temperature` | sensor_msgs/Temperature | pub | 5 | |
| `/rov/pressure/ms5837`, `/rov/pressure/ms5803` | sensor_msgs/FluidPressure | pub | 10 | Pa |
| `/rov/temperature/ms5837`, `/rov/temperature/ms5803` | sensor_msgs/Temperature | pub | 10 | |
| `/rov/depth` | std_msgs/Float32 | pub | 10 | m, z MS5837 (zewnetrzny); p0 ustawiane automatycznie po starcie, potem serwisem |
| `/rov/esc/telemetry` | rov_msgs/EscTelemetry | pub | 20 | 8 silnikow: DShot, eRPM/RPM bidir, KISS (T, V, I, mAh), prad CUR, failsafe |
| `/rov/status` | std_msgs/String | pub | 1 | diagnostyka: uptime, IMU/MS5837/MS5803 obecne?, gdy brak: poziomy SDA/SCL z pull-down i skan I2C1 |
| `/rov/thrusters` | rov_msgs/ThrusterCommand | sub | >=5 | float32[8] -1..1; **failsafe po 500 ms ciszy** |
| `/rov/esc/command` | rov_msgs/EscCommand | sub | | komendy DShot (0..47) i konfiguracja (100+), patrz msg |
| `/rov/depth/zero` | std_srvs/Trigger | srv | | zeruj glebokosc (powierzchnia) |
| `/rov/imu/calibrate` | std_srvs/Trigger | srv | | bias zyro, 2 s bez ruchu |

Failsafe: silniki na 0 gdy brak agenta, brak `/rov/thrusters` > 500 ms, albo USB odpiete.
Komendy DShot przyjmowane tylko gdy wszystkie silniki stoja.

## Pinout Pico 2 (jak w zweryfikowanym stanowisku testowym)

| GPIO | Sygnal |
|---|---|
| 9,10,11,12 | ESC #1 M1..M4 (DShot bidir, PIO0) |
| 16,17,18,19 | ESC #2 M5..M8 (DShot bidir, PIO1) |
| 13 | ESC #1 TLM -> UART0 RX (AUX) |
| 21 | ESC #2 TLM -> UART1 RX (AUX) |
| 27 / 28 | ESC #1 / #2 CUR -> ADC1 / ADC2 |
| 14 / 15 | I2C1 SDA / SCL: LSM6DSO32 (0x6A), MS5837-02BA (0x76), MS5803-05BA (0x77) |
| 6 | LSM6DSO32 INT1 (DRDY) |

## Katalogi

- `pico_fw/` - firmware Pico 2: `main.c` (micro-ROS), `microros_transport.c` (USB CDC), sterowniki
  `dshot.*`, `kiss.*`, `imu.*`, `press.*` (bez zmian z firmware testowego).
- `microros_lib/` - budowa `libmicroros.a` dla Cortex-M33 (gotowa biblioteka micro-ROS jest tylko dla
  RP2040), z naszymi `rov_msgs` w srodku. `build_lib.sh` uruchamiac na Jetsonie.
- `ros2_ws/src/rov_msgs` - EscTelemetry, ThrusterCommand, EscCommand.
- `ros2_ws/src/rov_bringup` - `rov.launch.py` (agent micro-ROS).
- `ros2_ws/src/rov_tools` - CLI `rov` (status, thrust, cmd, 3d, save, p0, imucal...).
- `scripts/` - `sync_to_jetson.sh` (z Windows), na Jetsonie: `build_ws.sh`, `build_fw.sh`, `flash.sh`, `env.sh`.

Na Jetsonie wszystko lezy w `~/nowy_rov/`:
`tools/` (Arm GCC 13.2, pico-sdk 2.1.1, picotool), `ros2_ws/` (micro_ros_setup + agent + pakiety ROV),
`ROV_ROS2/` (kopia tego katalogu), `logs/`.

## Workflow

Na Windows (Git Bash) po zmianach:
```sh
./scripts/sync_to_jetson.sh
```
Na Jetsonie (`ssh orin@192.168.1.108`):
```sh
cd ~/nowy_rov/ROV_ROS2
./microros_lib/build_lib.sh rebuild   # tylko gdy zmienily sie rov_msgs (dlugo)
./scripts/build_ws.sh                 # pakiety ROS 2 na Jetsonie
./scripts/build_fw.sh                 # firmware -> pico_fw/build/rov_pico.uf2
./scripts/flash.sh                    # wgranie (agent musi byc zatrzymany)
source scripts/env.sh
ros2 launch rov_bringup rov.launch.py # agent (terminal 1)
ros2 run rov_tools rov status         # terminal 2
ros2 run rov_tools rov thrust 1 0.1   # M1 na 10 %, Ctrl+C = stop
```

## Sterowanie z laptopa przez siec (swiatlowod) - GUI w przegladarce

Na laptopie nie trzeba ROS 2. Jetson serwuje konsole operatora:

- **GUI:** `http://192.168.1.108:8088` (rosbridge WebSocket na porcie 9090, strona z `rov_bringup/web/`).
- Pad (Gamepad API, uklad Xbox): lewa galka = przod/tyl + bok, prawa = obrot + gora/dol,
  `A` = ARM, `B` = DISARM, `LB`/`RB` = gain -/+. Klawiatura: `WASD` + `QE` obrot + `RF` gora/dol, `Enter` ARM, `Spacja` STOP.
- Przyciski: zeruj glebokosc, kalibruj zyro, STOP, beep, 3D ON/OFF + save, zeruj prad, bieguny.
- Lancuch: GUI -> `/rov/cmd_vel` (Twist) + `/rov/arm` + `/rov/gain` -> **thruster_mixer** (rov_control) -> `/rov/thrusters` -> Pico.
- Zabezpieczenia: mikser daje zera gdy DISARM albo brak `/rov/cmd_vel` > 0.5 s (zamkniecie karty = stop),
  Pico daje zera gdy brak `/rov/thrusters` > 0.5 s albo brak agenta.
- **Macierz miksera** (rozmieszczenie pednikow) jest w `rov_control/config/mixer.yaml` - domyslnie uklad
  "BlueROV2 Heavy" (4 poziome wektorowane + 4 pionowe). Trzeba ja dopasowac do rzeczywistego ROV.
- Wstecz/dol wymaga trybu 3D w ESC: przycisk "3D ON + save" przy stojacych silnikach (raz, zapisuje sie w ESC).

Caly stos startuje jedna komenda `ros2 launch rov_bringup rov.launch.py` (agent + mikser + rosbridge + http),
a po instalacji `scripts/install_service.sh` automatycznie przy starcie Jetsona (usluga systemd `rov`,
logi: `journalctl -u rov -f`). `flash.sh` sam zatrzymuje i wznawia usluge.

## Drugie Pico 2: BMS Daly po CAN (`pico_bms/`)

RP2350 nie ma sprzetowego CAN - uzywamy can2040 (CAN 2.0B na PIO0) + transceiver SN65HVD230 (3.3 V):
GP4 = CTX (TX), GP5 = CRX (RX), 250 kbit/s. Protokol Daly: host 0x40 pyta BMS 0x01 ramkami `0x18<DID>0140`,
BMS odpowiada `0x18<DID>4001`; DID 0x90..0x98 odpytywane w kolko co 120 ms.
Wezel `/rov_bms` publikuje `/rov/bms/status` (rov_msgs/BmsStatus: V, I, SOC, Ah, ogniwa, temperatury, MOS,
balans, bledy, cykle, liczniki CAN), `/rov/battery` (sensor_msgs/BatteryState) i `/rov/bms/info` (tekst: statystyki CAN
i widziane ID - do diagnostyki). GUI ma panel baterii z ogniwami, balansem i lista bledow.
Pico BMS ma numer seryjny USB `1DBD3353E3ED6781` (launch `bms_dev`, flash `./scripts/flash.sh bms`).
Build: `./scripts/build_fw.sh bms`.

## Kamera IMX708 (`rov_camera`)

Argus (`nvarguscamerasrc`, sterownik Arducam imx708 + DT z ~/imx708, patrz PODSUMOWANIE.md tam) -> sprzetowy JPEG
(`nvjpegenc`) -> wezel `camera`: strumien **MJPEG** `http://192.168.1.108:8089/cam.mjpg` (snapshot `/cam.jpg`),
`/rov/camera/compressed` (CompressedImage, 5 Hz do ROS), `/rov/camera/info` (JSON: fps, REC, focus).
Orin Nano nie ma enkodera H.264, dlatego MJPEG (720p30 q60 ~ 40-50 Mbit/s). Uslugi: `/rov/camera/record` (SetBool,
plik `~/nowy_rov/recordings/rov_*.mjpeg` + CSV czasow klatek; odtwarzanie `ffplay -f mjpeg plik`), `/rov/camera/snapshot`,
`/rov/camera/autofocus` (CDAF), topic `/rov/camera/focus` (UInt16 0..1023, VCM DW9807 po /dev/i2c-10).
Parametry w launchu: `sensor_id` (0 = CAM1 przy jednej kamerze), `sensor_mode` (1 = 2304x1296@55), `width/height/fps`,
`quality`, `flip` (2 = obrot 180), `ros_fps`. GUI ma podglad, zdjecie, nagrywanie, suwak ostrosci i AF.
CAM0 ma uszkodzona tasme (stan z 2026-10-21) - po wymianie druga kamera = drugi wezel z `sensor_id:=1`, `http_port:=8090`, `vcm_bus:=9`.

FPS/rozdzielczosc z launchu: `cam_mode` (0=4608x2592@14, 1=2304x1296@55, 2=1536x864@90), `cam_fps` (domyslnie 50),
`cam_width`/`cam_height` (1280x720), `cam_quality` (60). Np. `ros2 launch rov_bringup rov.launch.py cam_mode:=2 cam_fps:=90`.
720p50 q60 to ok. 70-80 Mbit/s na kazdego klienta strumienia (GUI + nagrywanie na PC = 2x).

**Nagrywanie na PC (na biezaco):** `python3 scripts/pc_record.py --host 192.168.1.108` - bez ROS, zapis 1:1 bez rekompresji do
`~/rov_recordings/rov_*.mjpeg` + `.csv` (nr klatki, czas Jetsona `X-Timestamp`, czas PC), pliki co 10 min, auto-reconnect,
limit `--max-gb 50`. Odtwarzanie `ffplay -f mjpeg plik.mjpeg`.

## Czujnik zalania, SSR, ring LED - patrz `docs/czujnik_zalania_4093.md` i naglowek `pico_bms/main.c`

## Stabilizacja (rov_control/attitude_controller.py)
GUI -> `/rov/cmd_vel_pilot` -> regulator -> `/rov/cmd_vel` -> mikser. Orientacja z surowego IMU (filtr komplementarny,
mapowanie osi `imu_axes` w `config/attitude.yaml`), zerowanie `/rov/imu/zero` (GUI: "ZERUJ IMU", `rov zero`).
Tryby: kurs (yaw), poziom (roll/pitch), glebokosc; wektor zadany `/rov/ctrl/setpoint`; PID z anty-windupem
(`i_limit`, `out_limit`); ruch galka zawiesza os. CLI: `rov att`, `rov hold`, `rov sp`, `rov gains`.

## Logowanie i diagnostyka
- `~/nowy_rov/logs/events.csv` - zdarzenia (zalanie, SSR, ladowanie, ARM, Pico/kamera online-offline, skok zegara, GUI).
- `~/nowy_rov/logs/bags/<data>/` - **ciagle nagrywanie wszystkich topikow** (ros2 bag, podzial co 10 min, bez obrazu kamery,
  limit 6 GB, najstarsze kasowane). `rov bags`, `ros2 bag info <kat>`, `ros2 bag play <kat>`. Wylaczenie: `record:=false`.
- `journalctl -u rov` - logi wszystkich wezlow (journal trwaly, przezywa restart). `rov logs` = tylko bledy/ostrzezenia.
- `rov diag` - paczka tar.gz (journal 2 h, dmesg, system, snapshot topikow, zdarzenia, konfiguracja) do wyslania.
- Kamera nagrywa osobno do `~/nowy_rov/recordings/` (przycisk w GUI).

## Uruchomienie na laptopie z Ubuntu 22.04 (bez pojazdu, symulator)

```sh
sudo apt install build-essential ros-humble-desktop ros-humble-rosbridge-suite python3-colcon-common-extensions python3-numpy python3-opencv
git clone https://github.com/wkusek2/ROV_ROS_v2.git ~/ROV_ROS2 && mkdir -p ~/rov_ws/src && ln -s ~/ROV_ROS2/ros2_ws/src/* ~/rov_ws/src/
cd ~/rov_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch rov_bringup rov.launch.py sim:=true agents:=false camera:=false record:=false
```
Potem GUI: `http://localhost:8088`. `sim:=true` uruchamia `sim_pico` - udaje oba Pico (telemetria ESC z eRPM reagujacym
na gaz, IMU z prosta dynamika, cisnienie/glebokosc, BMS, zalanie: `ros2 topic pub --once /rov/sim/leak std_msgs/msg/String "{data: '1'}"`).
Mikser, alokacja, regulator, logger, GUI dzialaja identycznie jak na Jetsonie. `rov ...` = `ros2 run rov_tools rov ...`.
Nie dziala na laptopie: agenty micro-ROS (brak Pico), kamera (Argus), flash/build firmware (toolchain na Jetsonie).

## Konfiguracja ESC dla ROV (raz, silniki stoja)
```sh
rov 3d on      # tryb 3D w ESC + mapowanie gazu -1..1
rov dir 3 r    # odwrocenie kierunku wybranego silnika
rov save       # zapis w ESC
rov poles 14   # bieguny silnika (do RPM)
```
BLHeliSuite32 (passthrough 4-way) nie ma w tym firmware - do ustawien przez Suite wgrac stare `esc_test`.
