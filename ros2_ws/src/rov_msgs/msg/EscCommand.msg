# Komenda DShot do ESC (tylko gdy silniki stoja) albo konfiguracja firmware Pico.
uint8 motor_mask      # bit i = silnik M(i+1); 0xFF = wszystkie, 0x0F = ESC #1, 0xF0 = ESC #2
uint8 command         # 0..47 = komenda DShot, >=100 = konfiguracja firmware (nie idzie do ESC)
uint8 repeat          # ile razy wyslac ramke (0 = domyslnie 6); dla komend >=100 = wartosc parametru

# najczesciej uzywane komendy DShot (BLHeli_32):
uint8 DSHOT_BEEP1=1
uint8 DSHOT_ESC_INFO=6
uint8 DSHOT_3D_OFF=9
uint8 DSHOT_3D_ON=10
uint8 DSHOT_SAVE=12            # zapis ustawien w ESC - po 3D/kierunku
uint8 DSHOT_EDT_ON=13          # Extended DShot Telemetry (BLHeli_32 >= 32.10)
uint8 DSHOT_DIR_NORMAL=20
uint8 DSHOT_DIR_REVERSE=21

# konfiguracja firmware (repeat = wartosc):
uint8 CMD_SET_POLES=100        # repeat = liczba biegunow silnika (domyslnie 14)
uint8 CMD_SET_RATE=101         # repeat: 1 = DShot150, 2 = 300, 3 = 600, 4 = 1200
uint8 CMD_SET_3D=102           # repeat: 1 = mapowanie 3D gazu w firmware, 0 = jednokierunkowe
uint8 CMD_CURRENT_ZERO=103     # zeruj offset czujnikow pradu (silniki stoja)
uint8 CMD_CURRENT_SCALE=104    # repeat = mV/A czujnika pradu (domyslnie 25)
uint8 CMD_SET_EDT=105          # repeat: 1 = dekoduj ramki EDT w odpowiedzi bidir
uint8 CMD_STOP=110             # natychmiastowy stop wszystkich silnikow
