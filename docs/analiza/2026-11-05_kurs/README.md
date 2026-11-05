# Test trzymania kursu, 2026-11-05 (10:03–10:10)

Dane: bag `20261105_094555` (Jetson `~/nowy_rov/logs/bags/`, kopia `~/rov_logs/bags/` na laptopie).
Eksport: `rov_bag export … --topics '^/rov/(debug/ctrl|debug/mixer|esc/telemetry|…)$'`.
Analiza i wykres: `analiza.py` (tabela: `analiza.py <csv> <out> tabela HH:MM:SS HH:MM:SS`).

Warunki: ROV przy powierzchni (głębokość ~0.09 m), gałka kursu nieruszana. Wszystko robił regulator (`yaw_hold`, ki = 0).

## Wynik: dodatnie sprzężenie w osi kursu (`kurs.png`)
- 10:07:39.6, gain 0.15: ROV zaczyna wolno skręcać w lewo. Regulator poprawnie każe skręcać w prawo (`out` −0.6, mikser N = +7.2 Nm, czyli dziób w prawo). **ROV rozkręca się w lewo do 91°/s w 1.2 s**, kurs rośnie z 0° do 81°. Pilot robi DISARM.
- 10:03:41: to samo w drugą stronę. Regulator „w lewo” (+0.6), ROV obraca się w prawo (−56°/s).
- 10:08–10:10, gain 0.05: kurs ustala się **przy ±180°**, czyli naprzeciw zadanego, a komenda szarpie między ±0.6 przy każdym przejściu błędu przez ±180°. To typowy obraz odwróconej pętli: stabilnym punktem staje się kurs przeciwny.
- Cała sesja, 2956 ticków ARM: korelacja (moment N z miksera, przyspieszenie obrotu w lewo) wynosi **+0.57**. Fizycznie powinna być ujemna.

## Co wykluczone
- **Pomiar kursu:** oś pionowa wynika z grawitacji (przyspieszenie na osi z kadłuba +10 m/s²), znak obrotu jest fizyczny. Odwrócenie osi przód/bok nie zmienia znaku yaw.
- **Matematyka miksera:** `sign_check` przechodzi.
- **Pędniki na powierzchni / w powietrzu:** przy czystym yaw górne i dolne pędniki **osobno** dają moment w tę samą stronę (po N = +5 z 10). Utrata połowy pędników nie odwraca kierunku.

## Najbardziej prawdopodobna przyczyna
Fizyczny tor wykonawczy: przypisanie silników do pozycji lub flagi `inv` w `~/nowy_rov/config/thrusters.json` (5/8 z `inv: true`), kierunek obrotów w ESC albo śmigła.

Możliwe, że flagi były dobierane, kiedy mikser miał jeszcze odwrócony znak pitch (poprawiony 2026-11-04). Wtedy „naprawiały” pitch kosztem innych osi.

**Potrzebny test na sucho dla każdego silnika** (`rov spin M 0.15 2`): gdzie fizycznie siedzi i w którą stronę pcha. Do tego `rov move yaw/surge/sway/heave/roll/pitch`.

## Inne obserwacje
- Szybkie przełączanie ARM/DISARM 10:07:45–52 (10 razy w 7 s): reakcja pilota na rozkręcanie.
- Telemetria ESC: szpilki RPM do 140 000, czyli błędne ramki bidir DShot. Do sprawdzenia liczniki CRC.
- Trzymanie głębokości (10:04:32): po zmianie zadanej z 0.09 na 0.5 m ROV zszedł o 3 cm i wrócił mimo pełnej komendy „w dół” (gain 0.15, max |gaz| 0.23). Przy powierzchni i tak małym gainie nie da się tego ocenić.
