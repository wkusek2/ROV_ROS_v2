# Gaz 8 pednikow. Trzeba publikowac ciagle (>= 5 Hz): po 500 ms ciszy firmware zatrzymuje silniki (failsafe).
# Indeks 0..7 = M1..M8. Zakres -1.0..1.0. Ujemne = wstecz - TYLKO gdy ESC ma wlaczony tryb 3D
# (EscCommand 10 + 12 i flaga CMD_SET_3D). Bez 3D ujemne sa obcinane do 0.
float32[8] throttle
