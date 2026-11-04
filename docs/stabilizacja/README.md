# Stabilizacja - błąd znaków roll/pitch (poprawka 2026-11-04)

## Objaw w wodzie
Po włączeniu stabilizacji pojazd się rozbujał i przewracał, tym szybciej, im większy gain. Przy zanurzaniu się wywracał i zaczynał kręcić.

## Przyczyna
Konwencja alokacji (`allocation.py`, Fossen FRD, `tau = r x F`): **K+ = prawa burta w dół, M+ = dziób w górę, N+ = dziób w prawo**.

| Miejsce | Było | Skutek | Jest |
|---|---|---|---|
| `thruster_mixer.py` `DOF_OF_AXIS` | `pitch -> M * -1` | pilot „dziób w górę” opuszczał dziób, regulator pitch miał dodatnie sprzężenie | `pitch -> M * +1` |
| `attitude_controller.py` `relative_rpy` | `return [-roll, ...]` | roll odczytywany z odwrotnym znakiem, regulator roll miał dodatnie sprzężenie | `return [roll, ...]` |
| `attitude_controller.py` `tick` | `rate=-gx` (człon D roll) | spójne ze złym znakiem kąta | `rate=gx` |
| `sim_pico.py` | `pitch = -M`, `wx = -rates[0]` | symulator miał te same błędy, więc ich nie wykrywał | poprawione |

Yaw był poprawny.

## Dowód: `przed/` vs `po/`
- `przed/1_pedniki.png`: stary kod. Pilot pitch+ opuszcza dziób. Przy przechyle regulator pcha **dalej** w tę samą stronę.
- `przed/2_petla.png`: stary kod, zamknięta pętla. Pitch wywraca się przy każdym gainie, roll przy gainie 1.0.
- `po/1_pedniki.png`: obecny kod. Wszystkie testy znaków OK.
- `po/2_porownanie.png`: stary i obecny kod na tej samej fizyce. Uderzenia roll, pitch i yaw, a do tego zanurzanie z nierównymi pędnikami (podróbki T200, 80–100% ciągu). Stary kod przy zanurzaniu wywraca się w pitch po 1.6–4 s. Obecny trzyma kąty w granicach ±1.5°.
- `po/wyniki.txt`: liczby.

Wykresy powstają z **prawdziwego kodu** (alokacja, `ThrusterMixer.allocate_cmd`, filtr i PID `AttitudeController`) podłączonego do niezależnej fizyki bryły sztywnej. Kierunki są sprawdzane geometrycznie, po położeniu punktów kadłuba.

Jedyne założenie o sprzęcie: IMU leży zgodnie z `imu_axes: "x,z,-y"` (`config/attitude.yaml`).

Fizyka jest przybliżona: bezwładność, tłumienie, wypór 2 cm nad środkiem masy. Liczby są więc orientacyjne, a znaki pewne.

## Odtworzenie / test po zmianach
```sh
source ros2_ws/install/setup.bash
python3 ros2_ws/src/rov_control/test/sign_check.py docs/stabilizacja/po   # exit 1 = test znaków nie przeszedł
```

## Sprawdzenie na pojeździe (na sucho, stabilizacja wyłączona)
1. `rov move pitch 0.15 2`: dziób w **górę**.
2. `rov move roll 0.15 2`: prawa burta w **dół**.
3. Przechylenie ręką prawą burtą w dół: GUI pokazuje roll **+**. Dziób w górę: pitch **+**. Obrót w lewo: yaw **+**.

Jeśli punkt 3 się nie zgadza, `imu_axes` nie pasuje do montażu IMU. Trzeba je poprawić, zanim włączysz stabilizację w wodzie.
