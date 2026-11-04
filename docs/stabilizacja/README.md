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

# Model ciągu: krzywa T200 zamiast liniowego 40/30 N (2026-11-04)

Mikser (`thrust_model: "t200"` w `config/mixer.yaml`) i `sim_pico` używają krzywej Blue Robotics T200 przy 16 V z `allocation.py`. Maksimum to +51.5 / −39.9 N.

Oś sygnału jest przeliczona z PWM na nasz DShot 3D (−1..1), bez martwej strefy ±25 µs z ESC Blue Robotics. Punkty krzywej są przybliżone z wykresu producenta.

`thrust_scale` mnoży całą krzywą. Służy do podróbek i innego napięcia: po pomiarze ciągu wystarczy zmienić tę jedną liczbę. Stary model jest dostępny jako `thrust_model: "power"`.

Wyniki (`po_t200/3_model_ciagu.png`, liczby w `po_t200/wyniki.txt`). Pędniki w fizyce mają krzywą T200, w niektórych wariantach przemnożoną przez rozrzut podróbek 80–100%:
- **Liniowy model, czyste zanurzanie heave −0.6:** wychodzi pasożytnicze **X = −12 N**, czyli pojazd cofa się przy schodzeniu. Z daje 46 N zamiast planowanych 55 N.
  Przyczyna: przód i tył są niesymetryczne, a krzywa nieliniowa, więc pędniki kręcące wstecz i naprzód dają inne siły, niż zakłada mikser.
- **Liniowy model, czysty yaw +0.4:** moment 16 Nm zamiast 24 Nm (−35%). Do tego X = −6 N.
- **Model T200, nominalne pędniki:** faktyczne siły równe planowanym, zero sprzężeń.
- **Rozrzut podróbek 80–100%:** przy zanurzaniu zostaje moment yaw około 1.3 Nm, pitch około 0.7 Nm i siła Y około 3 N. **To jest źródło samoczynnego obracania przy zanurzaniu.** Tego nie usunie żaden model, który zakłada identyczne pędniki. Usuwa to regulator (po poprawce znaków trzyma ±1.5°, `po_t200/2_porownanie.png`) albo pomiar i ustawienie wag lub skali dla każdego pędnika osobno.

Uwaga: model T200 ma około 29% wyższe limity niż stary model 40/30 N. Ten sam gain daje teraz większe siły i momenty, także po stronie regulatora.

W `po_t200/2_porownanie.png` „STARY kod” oznacza stare znaki i liniowy model ciągu, a pędniki w fizyce to T200. Liczby różnią się więc od `po/`, gdzie fizyka była liniowa.
