# Pelny odczyt z BMS Daly (CAN, rekordy 0x90..0x98). Publikowane przez Pico "bms" (/rov_bms).
std_msgs/Header header
bool online                 # odpowiedz z BMS < 5 s temu

# 0x90
float32 voltage_v           # napiecie pakietu
float32 current_a           # + ladowanie, - rozladowanie
float32 soc_pct
# 0x93
float32 remaining_ah        # pozostala pojemnosc
uint8 state                 # 0 = spoczynek, 1 = ladowanie, 2 = rozladowanie
bool charge_mos
bool discharge_mos
uint8 bms_life
# 0x94
uint8 cell_count
uint8 temp_count
bool charger_connected
bool load_connected
uint8 dio_bits
uint16 cycles
# 0x91 / 0x92
uint16 max_cell_mv
uint8  max_cell_no
uint16 min_cell_mv
uint8  min_cell_no
int8   max_temp_c
uint8  max_temp_no
int8   min_temp_c
uint8  min_temp_no
# 0x95 / 0x96
float32[48] cell_voltage_v  # ogniwa 1..cell_count (reszta 0)
int8[16] temperature_c      # czujniki 1..temp_count
# 0x97 / 0x98
uint8[8] balance_bits       # bit i bajtu b = ogniwo 8*b+i+1 w aktywnym balansowaniu
uint8[8] fault_bits         # bajty 0..7 z rekordu 0x98 (tablica bledow Daly)

# diagnostyka CAN
uint32 can_rx
uint32 can_tx
uint32 can_errors
uint32 age_ms               # od ostatniej ramki z BMS (0xFFFFFFFF = nigdy)
