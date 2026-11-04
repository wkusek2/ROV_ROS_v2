# Telemetria 8 silnikow (2x 4-in-1 ESC BLHeli_32). Indeks 0..7 = silnik M1..M8 (0..3 = ESC #1, 4..7 = ESC #2).
std_msgs/Header header

uint16[8]  dshot_value        # aktualnie wysylana wartosc DShot: 0 = stop, 48..2047 = gaz (3D: 48..1047 wstecz, 1048..2047 naprzod)

# bidirectional DShot (odpowiedz w linii sygnalowej)
uint32[8]  erpm               # eRPM, 0 = stoi / brak odpowiedzi
uint32[8]  rpm                # erpm / (poles/2)
bool[8]    bidir_ok           # poprawna odpowiedz < 200 ms temu
uint32[8]  bidir_ok_count
uint32[8]  bidir_crc_errors
uint32[8]  bidir_no_reply

# telemetria KISS po przewodzie TLM (UART), odpytywana rotacyjnie - kazdy silnik co ~40 ms
int8[8]    kiss_temp_c
float32[8] kiss_voltage_v
float32[8] kiss_current_a
uint16[8]  kiss_mah
uint32[8]  kiss_erpm
uint32[8]  kiss_age_ms        # ile ms temu ostatnia ramka KISS dla silnika (0xFFFFFFFF = nigdy)

float32[2] esc_current_a      # prad z wyjscia CUR obu ESC (ADC), po skali i offsecie

uint8      poles              # liczba biegunow uzyta do RPM
uint16     dshot_rate_kbit    # 150/300/600/1200
bool       mode_3d            # mapowanie gazu w firmware (ujemne wartosci dozwolone)
bool       failsafe           # true = silniki zatrzymane przez failsafe (brak komend / brak agenta)
bool       agent_connected
uint32     loop_count         # licznik petli DShot (1 kHz)
uint32     cmd_age_ms         # ile ms od ostatniego ThrusterCommand
