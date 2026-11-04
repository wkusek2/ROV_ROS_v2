# Czujnik zalania (plytka "leakage sensor", CD4093)

Dwukanalowy detektor wody z lokalnym alarmem (brzeczyk + LED) i dwoma wyjsciami logicznymi do sterownika
(u nas: glowne Pico 2, GP2/GP3).

## Zasada dzialania

Uklad to poczworna bramka NAND z wejsciami Schmitta **CD4093** (IC4), zasilana z Uz (3..15 V, u nas 3,3 V lub 5 V).

**Kanal 1 (IC4A, piny 1-2-3)** i **kanal 2 (IC4D, piny 12-13-11)** sa identyczne:
- wejscie bramki jest podciagniete do Uz rezystorem **1 MOhm** (R1 / R3) i filtrowane kondensatorem **1 uF** do masy
  (C5 / C2), stala czasowa ok. **1 s**;
- do tego wezla podlaczona jest sonda (zlacze X1 = IN1, X2 = IN2); druga elektroda sondy to **masa** ukladu;
- na sucho wezel ma poziom wysoki -> wyjscie bramki NAND = **niskie (0 V)**;
- woda miedzy elektroda a masa (rezystancja ponizej kilkuset kOhm) sciaga wezel ponizej progu Schmitta
  (ok. 0,45*Uz) -> wyjscie bramki = **wysokie (~Uz)**.
  Histereza wejscia Schmitta daje czyste przelaczenie bez drgan; kondensator 1 uF odfiltrowuje krotkie zaklocenia,
  ale tez opoznia reakcje o ok. 1 s w obie strony.

**Suma alarmow:** wyjscia obu kanalow przez diody D1/D2 (1N4148) tworza bramke OR i trafiaja na pin 5 IC4B.
Diody odsprzegaja kanaly, zeby wyjscie jednego nie bylo zwierane przez drugi.

**Generator alarmu:** IC4B (pin 5 = zezwolenie, pin 6 = sprzezenie) i IC4C (inwerter, piny 8-9 zwarte) z R2 **470 kOhm**
i C3 **4,7 uF** tworza bramkowany multiwibrator. Gdy ktorys kanal zglasza wode, generator rusza i daje przebieg
o okresie rzedu **2-3 s** (R2*C3 = 2,2 s); na sucho pin 5 jest niski, pin 4 stale wysoki i generator stoi.

**Sygnalizator:** wyjscie IC4C (pin 10) steruje baza tranzystora NPN **BC547** (T1). Kolektor zasila rownolegle
**LED2 z R4 200 Ohm** i aktywny brzeczyk **F/CM12P** (SG1) z Uz. Efekt: przy zalaniu LED i brzeczyk **pulsuja**
(ok. 1 s wlaczone / 1 s wylaczone), a nie swieca ciagle. Brak rezystora w bazie jest dopuszczalny, bo wyjscie CMOS 4093
samo ogranicza prad do kilku mA.

C1 100 nF to odsprzeganie zasilania IC4.

## Zlacza i piny

| Zlacze | Pin | Nazwa | Kierunek | Opis |
|---|---|---|---|---|
| **X3** | X3-2 | **Uz** | wejscie zasilania | +3,3 V lub +5 V. Pobor na sucho < 1 mA (tylko 4093), przy alarmie ok. 20-40 mA (LED + brzeczyk w impulsach). |
| X3 | X3-1 | **GND** | masa | Wspolna masa z Pico. |
| **X1** | X1-1, X1-2 | **IN1** | wejscie sondy 1 | Wezel kanalu 1 (1 MOhm do Uz, 1 uF do GND). Obie piny zlacza siedza na tym samym wezle; sonda = ta elektroda + **masa**. |
| **X2** | X2-1, X2-2 | **IN2** | wejscie sondy 2 | Jak IN1, kanal 2. |
| **X4** | **X4-2** | **OUT1** | wyjscie logiczne | Bezposrednio pin 3 IC4A. **Sucho = 0 V, woda = ~Uz** (poziom logiczny = napiecie zasilania plytki). Wydajnosc pradowa tylko ok. 1 mA - sygnal do wejscia mikrokontrolera, nie do LED/przekaznika. |
| X4 | X4-1 | GND | masa | Masa odniesienia dla OUT1 (wygodna tez jako druga elektroda sondy 1). |
| **X5** | **X5-2** | **OUT2** | wyjscie logiczne | Pin 11 IC4D, kanal 2, jak OUT1. |
| X5 | X5-1 | GND | masa | Masa odniesienia dla OUT2. |

## Parametry czasowe

- wykrycie wody: ok. 1 s (filtr 1 MOhm / 1 uF),
- powrot do "sucho" po osuszeniu: ok. 1 s + ewentualne dokonczenie impulsu brzeczyka,
- czestotliwosc pulsowania alarmu: ok. 0,3-0,5 Hz.

## Podlaczenie do glownego Pico 2 (ESC/IMU)

| Plytka | Pico 2 |
|---|---|
| X3-2 Uz | 3V3 (pin 36) - wtedy OUT1/OUT2 maja poziom 3,3 V |
| X3-1 GND | GND |
| X4-2 OUT1 | GP2 (pin 4) |
| X5-2 OUT2 | GP3 (pin 5) |

Przy zasilaniu z 5 V (glosniejszy brzeczyk) na OUT1/OUT2 dac dzielnik 10 k / 15 k, zeby na Pico bylo ok. 3 V.
Firmware trzyma GP2/GP3 z wewnetrznym pull-downem (odpieta plytka = "sucho") i filtruje stan 300 ms.
ROS 2: topic `/rov/leak` (std_msgs/UInt8, bit0 = czujnik 1, bit1 = czujnik 2), wpis `ZALANIE=...` w `/rov/status`,
alarm w GUI (czerwona ramka + log).

## Uwagi praktyczne

- Elektrody sondy: dwa nierdzewne druty/pady 5-10 mm od siebie, jeden na IN, drugi na GND. Slodka woda ma rezystancje
  rzedu dziesiatek-setek kOhm na takim odstepie, slona kilka kOhm - obie wykryje. Woda destylowana moze nie zadzialac.
- Elektroda na IN jest stale pod napieciem Uz przez 1 MOhm - w wodzie slonej bedzie powoli korodowac (prad < 5 uA,
  praktycznie pomijalne).
- Wyjscia OUT nie sa zabezpieczone - nie podlaczac do niczego poza wejsciem logicznym.
- Sondy umiescic w najnizszym punkcie kadluba (woda zbiera sie na dnie), po jednej na przod i tyl.
