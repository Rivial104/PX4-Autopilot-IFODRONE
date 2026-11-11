# Plan: pionowy booster 4× EDF (jeden sygnał PWM)

**Cel:** 4 nowe EDF-y skierowane w górę (ciąg wzdłuż body −Z), sterowane jednym wyjściem PWM (S7, Y-kabel → 4 ESC).
Booster przejmuje **większość Fz**. Para coax zostaje głównie do **yaw** i tylko w niewielkim stopniu dokłada Fz.
Booster nie daje momentów (roll/pitch/yaw).

Status: **DO AKCEPTACJI (v2, po odpowiedziach)**. Zmiana względem v1: ratio boostera nie jest już ≤ 1, tylko jest
gałką podziału Fz (§1). Doszły też dwa twarde ograniczenia: §3 kompromis yaw i §4 budżet jednostek Fz.

---

## 1. Architektura: wariant B, „booster podąża za coax z przełożeniem g”

```
u_boost = g · (u_coax0 + u_coax1) / 2          g = IFO_BST_RATIO (0..10), clip [0, 1]
B[Fz, coax_i] = CT0 + g · CT_boost / 2          (kolumna boostera w macierzy = 0)
```

- Pseudo-inwersja widzi tylko coax. Fz = (2·CT0 + g·CT_b)·ū, czyli dokładnie ciąg fizyczny przy `u_boost = g·ū`,
  więc macierz pozostaje spójna z rzeczywistością.
- **Yaw bez zmian:** różnica coax (KM ±0.4) jest liczona wokół średniej, a booster bierze tylko średnią. Skok yaw nie rusza boostera.
- **Roll/pitch bez zmian:** tilty działają jak dotąd. Booster nie ma kolumny momentów, bo jest modelowany
  w centroidzie (PX = PY = 0).
- **g to gałka podziału Fz** (model liniowy, `CT_b ≈ 1.4`, `CT0 = 0.6`):

  | g | udział boostera w Fz | ū coax w zawisie vs dziś (ta sama masa) |
  |---|---|---|
  | 1 | 54% | 0.46× |
  | 2 | 70% | 0.30× |
  | 3 | 78% | 0.22× |
  | 4 | 82% | 0.18× |

- `IFO_BST_ROTOR = −1` (default) daje zachowanie identyczne jak dziś.
- Dlaczego nie wariant A (zwykły 7. rotor w pseudo-inwersji): podział Fz jest tam sztywny (∝ CT²) i nie ma gałki.
  Przy obecnym CT_b daje ~73%, czyli tyle samo co B z g ≈ 2.3, ale bez możliwości strojenia.
- Booster = **CA_ROTOR6**. Indeksy 0–5 i funkcje 101–106 bez zmian (okablowanie 4072 regresowało 2× przy
  „porządkowaniu”). Serwa przesuwają się w wektorze aktuatorów 6..9 → 7..10, a ich funkcje 201–204 zostają.

## 2. Dane wejściowe (z odpowiedzi)

| # | Odpowiedź | Wynika z tego |
|---|-----------|---------------|
| Q1 | wariant B | jak §1, ale z g > 1 |
| Q2 | QX-Motor 64 mm: 2370 g @ 100%, 750 g @ 50% | 1 EDF: 23.2 N max, 7.4 N @ 0.5 ⇒ **F ≈ 23.2·u^1.66 N**. Booster razem: 93 N max (~9.5 kg). Sieczna przy u ≈ 0.4–0.5: 51–59 N/u na 4 EDF-y ⇒ **CT_b ≈ 1.4** w jednostkach coax (CT0 0.6). Do kalibracji w SITL. |
| Q3 | 2× CW + 2× CCW | `CA_ROTOR6_KM 0`. **CW na jednej przekątnej, CCW na drugiej**, wtedy momenty reakcyjne i żyroskopowe się znoszą. |
| Q4 | konfiguracja X (+45° względem bocznych EDF-ów), promień o 7 cm większy niż EDF-y tilt | r = 0.535 + 0.07 = **0.605 m**, pozycje (±0.428, ±0.428). W CA jeden rotor w centroidzie (0, 0, PZ). |
| Q5 | S7 | `PWM_MAIN_FUNC7 107` |
| Q6 | booster znacznie silniejszy, ma nieść większość Fz | g > 1, start **g = 2** (§3) |

**Wciąż otwarte** (nie blokują Etapu 1–2; do realu potrzebne przed Etapem 4):

| # | Pytanie | Założenie robocze |
|---|---------|-------------------|
| Q7 | Masa zestawu boostera (4 EDF + 4 ESC + mocowania + kable) | +0.9 kg |
| Q8 | Aktualna masa startowa na realu (bez boostera) | z `hover_thrust_estimate` (0.5) |
| Q9 | Wysokość EDF-ów boostera względem środka (Z, FRD dół +) | PZ = 0.045 (jak EDF-y tilt) |
| Q10 | Czy „+7 cm” to promień (0.605 m), czy odległość w poziomie od EDF-a tilt? | promień 0.605 m |

## 3. Kompromis yaw ↔ udział boostera (fizyka, nie do obejścia parametrem)

Yaw pochodzi wyłącznie z różnicy momentów reakcyjnych coax: `u0 = ū + d`, `u1 = ū − d`, więc **zapas yaw ≤ ū**
(dolne ograniczenie 0). Coax „tylko do yaw” musi więc się kręcić, a wtedy dokłada Fz ≈ 2·CT0·ū. Im większe g,
tym mniejsze ū w zawisie i mniejszy zapas yaw. Przy g = 2 i masie +30%: ū ≈ 0.16 vs dziś 0.42, czyli ~2.5× mniejszy
zapas. Pamięć „manual yaw saturates coax” pokazuje, że yaw już raz nasycał coax.

Procedura doboru g:
1. Z istniejących logów zawisu/lotu wyciągnąć rzeczywiste |`control[0]` − `control[1]`|/2 (p95 i max).
2. Wybrać największe g, dla którego ū w zawisie ≥ 1.5 × p95 zapotrzebowania yaw.
3. Zweryfikować w SITL skokiem yaw 90° (Pos) i pełnym drążkiem yaw (Stabilize).

Jeśli wyjdzie, że potrzebny jest duży udział boostera **i** duży zapas yaw, rozwiązaniem jest podłoga coax
(`ū ≥ IFO_COAX_YMIN`, booster domyka resztę). To osobne zadanie po pierwszych lotach, nie wchodzi w ten plan.

## 4. Budżet jednostek Fz (do sprawdzenia w Etapie 2)

Fz jest w surowych jednostkach CT (skala normalizacji = 1). Ograniczenia w łańcuchu:

- `vehicle_thrust_setpoint` ma konwencję ±1 (clamp przy `MC_BAT_SCALE_EN`);
- `IFO_THR_MAX` i `IFO_THR_HOVER` mają metadane ≤ 1 / ≤ 0.9;
- `mc_hover_thrust_estimator` tnie hover do **[0.1, 0.9]**.

Więcej masy i niższy punkt pracy coax podnoszą hover wyrażony w jednostkach CT. Reguła:

- **hover ≤ 0.7 i `IFO_THR_MAX` ≤ 1.0** ⇒ jednostki zostają;
- inaczej ⇒ **przeskalowanie jednostek o s**:
  - wszystkie `CA_ROTOR*_CT` × s, `CA_ROTOR0/1_KM` ÷ s (zachowuje wiersz yaw);
  - `IFO_THR_HOVER/MIN/MAX/XY_MAX`, `MPC_THR_HOVER` × s;
  - wiersze roll/pitch są niezmiennicze (dzielone przez torque_ref);
  - **twarde ograniczenie:** `CT_side · s · cos(35°) > 0.05`, bo inaczej ControlAllocator wyzeruje wiersze Fx/Fy
    (pamięć „weak-row zeroing”). Przy CT_side 0.1 daje to **s ≥ 0.65**.

T/W = `IFO_THR_MAX` / hover; cel ≥ 1.4 (dziś 0.8/0.5 = 1.6 real, 0.8/0.6 = 1.33 SITL). Booster ma zapas
(93 N), więc ograniczeniem będzie budżet jednostek, nie silniki.

---

## 5. Zakres prac

### Etap 1: alokator (C++), bez zmiany zachowania przy defaultach

1. `control_allocator/module.yaml`:
   - `IFO_BST_ROTOR` (int32, default −1, −1..11): indeks rotora boostera; −1 = brak;
   - `IFO_BST_RATIO` (float, default 1.0, 0..10): g, przełożenie komendy boostera do średniej coax.
2. `ActuatorEffectivenessIfodrone.{hpp,cpp}`:
   - `boosterIndex()`: poprawny tylko gdy `2 ≤ idx < num_rotors` i `tilt_index < 0`; inaczej −1 + `PX4_WARN` (raz);
   - `getEffectivenessMatrix()`: po `addActuators()` zeruje kolumnę boostera i dodaje `g·CT_b/2·(oś boostera)`
     do części siłowej kolumn coax 0 i 1;
   - `updateSetpoint()`: `u_boost = g·(u0 + u1)/2`, clip do [min, max] boostera, **przed** aktualizacją
     `_lin_point`;
   - komentarze nagłówkowe (Motors 0–1 coax, 2–5 side, 6 booster).
3. gtest (obok `IfodroneControlMathTest`), sprawdza że:
   - (a) `IFO_BST_ROTOR −1` daje macierz identyczną jak przed zmianą;
   - (b) kolumna boostera = 0 i Fz coax = CT0 + g·CT_b/2;
   - (c) `u_boost = g·ū`;
   - (d) czysty yaw nie zmienia `u_boost`;
   - (e) nieprawidłowy indeks (coax, rotor z tiltem) daje booster wyłączony.
4. Build: `make px4_sitl_default` + `make matek_h743-slim_default` (rozmiar flash).

### Etap 2: SITL (submoduł `Rivial104/PX4-gz-IFODRONE @ifo-model`)

1. `models/ifodrone/model.sdf`:
   - 4 linki `booster_0..3` na (±0.428, ±0.428), Z z Q9, revolute spin-jointy do `base_link`;
   - **oś w górę w ENU = +Z** (w FRD −Z; pułapka z 15.07). Y w ENU jest odwrócone, ale układ X jest
     symetryczny, więc pozycje się nie zmieniają;
   - 4 pluginy `multicopter-motor-model` z **tym samym `motorNumber` 6**, co odpowiada Y-kablowi;
   - dopasowanie do krzywej QX-64: `motorConstant 2.325e-05`, ω = 125 + 875·u (`SIM_GZ_EC_MIN7 125`,
     `MAX7 1000`) daje 23.2 N @ 1.0, 7.4 N @ 0.5 i 0.36 N na biegu jałowym;
   - `maxRotVelocity ≥ 1000`;
   - `turningDirection` CW/CCW po przekątnych, `momentConstant 0` (spójnie z KM 0 w CA);
   - masa z Q7: rozłożona na linki boostera + korekta inercji `base_link`.
2. `49000_gz_ifodrone`:
   - `CA_ROTOR_COUNT 7`, `CA_ROTOR6_PX/PY 0`, `PZ`, `AX/AY 0`, `AZ −1` (jawnie), `KM 0`, `CT 1.4`, `TILT 0`;
   - `IFO_BST_ROTOR 6`, `IFO_BST_RATIO 2.0`;
   - `SIM_GZ_EC_FUNC7 107`, `MIN7 125`, `MAX7 1000`;
   - `IFO_THR_HOVER = MPC_THR_HOVER`, `IFO_THR_MIN/MAX`: z kalibracji; jeśli trzeba, ze skalowaniem z §4.
3. Kalibracja (iteracyjnie):
   - zawis ⇒ odczyt `hover_thrust_estimate` i komend `control[0..1]`, `control[6]`;
   - korekta CT_b tak, żeby estymata hover zgadzała się z modelem;
   - decyzja o skali s (§4).
4. Commit w submodule + bump wskaźnika. **Push do forka tylko po Twojej zgodzie.**

### Etap 3: weryfikacja SITL

- Regresja: `IFO_BST_ROTOR −1` + model bez boostera ⇒ hover/takeoff jak dziś.
- Z boosterem:
  - `control[6] == g·mean(control[0], control[1])`;
  - udział Fz zgodny z tabelą §1;
  - `hover_thrust_estimate` ≈ `IFO_THR_HOVER`;
  - takeoff bez „stania” (pułapka 10.11);
  - skok yaw 90° i pełny drążek yaw bez nasycenia coax (§3);
  - Stabilize/Alt/Pos;
  - roll/pitch bez nowego dryfu;
  - brak `unallocated_thrust` w zawisie.
- Test odporności: wstrzyknięcie asymetrii, np. `motorConstant` jednego EDF-a boostera −5%. Mierzymy kąt/moment
  potrzebny tiltom, żeby ocenić R2.

### Etap 4: real (4072, Matek H743-Slim)

1. `4072_ifodrone`:
   - `CA_ROTOR6_*`, `IFO_BST_*` jak SITL; CT_b przeliczone wg Q7/Q8, docelowo z kalibracji lotnej;
   - `PWM_MAIN_FUNC7 107`, `PWM_MAIN_MIN7 1020`, ewentualnie `MAX7`; `FUNC8` = 0;
   - tabela wyjść w nagłówku.
2. Sprzęt:
   - **S7 = TIM4 razem z S8, S9, S10** (S9/S10 = serwa tilt). Grupa zostaje na PWM 400 Hz (`PWM_MAIN_TIM2`),
     **bez DShot**. Serwa już dziś chodzą tam na 400 Hz;
   - Y-kabel: sygnał + GND do 4 ESC, **+5 V (BEC) tylko z jednego**;
   - kalibracja ESC wszystkich 4 naraz albo BLHeli ze stałymi endpointami, bo rozjazd endpointów daje
     asymetrię (R2). Opcjonalnie pomiar ciągu każdego EDF-a na wadze przy tej samej komendzie;
   - **CG wyważony na centroid boostera** (R2);
   - zasilanie: 4 EDF-y 64 mm to duży prąd. Sprawdzić C baterii, przekroje i `BAT1_A_PER_V 40` (zakres czujnika).
3. Na płycie: `param set` + `param save` (nie `set-default`), potem weryfikacja z logu.
4. Stół, **zablokowany dron**:
   - `actuator_test` motor 7: 4 EDF-y ruszają, CW/CCW po przekątnych, ciąg w górę;
   - po uzbrojeniu booster podąża za gazem, a yaw z drążka nie rusza boostera.
5. Pierwszy lot:
   - na uwięzi, Stabilize, **g = 1** (54%), potem 1.5 → 2;
   - przy każdym kroku kontrola zapasu yaw i dryfu roll/pitch;
   - kalibracja `IFO_THR_HOVER`/`MPC_THR_HOVER` z `hover_thrust_estimate`.

### Etap 5: dokumentacja

- `IFODRONE-params.md`:
  - §2 (`CA_ROTOR6`), §4 (`IFO_BST_*`), §5 (S7 = 107, TIM4), §11 (THR_*), §13 (`SIM_GZ_EC_*7`);
  - pułapki: „booster jako zwykły rotor = brak gałki podziału” oraz „g ↑ ⇒ zapas yaw ↓”;
  - ewentualnie skala s.
- `IFODRONE.md` (CHANGELOG), nagłówki airframe'ów.
- Pamięć projektu po wdrożeniu.

---

## 6. Pliki do zmiany

| Plik | Zmiana |
|------|--------|
| `src/modules/control_allocator/module.yaml` | `IFO_BST_ROTOR`, `IFO_BST_RATIO` |
| `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessIfodrone.{hpp,cpp}` | logika boostera |
| `src/modules/control_allocator/VehicleActuatorEffectiveness/` (nowy test) + `CMakeLists.txt` | gtest |
| `ROMFS/px4fmu_common/init.d-posix/airframes/49000_gz_ifodrone` | SITL params |
| `ROMFS/px4fmu_common/init.d/airframes/4072_ifodrone` | real params + PWM |
| `Tools/simulation/gz/models/ifodrone/model.sdf` (submoduł) | 4 linki + pluginy |
| `IFODRONE-params.md`, `IFODRONE.md` | docs |

Bez zmian: `ifo_pos_control`, `ifo_att_control`, `mc_rate_control`, `ControlAllocator.cpp`, board config.

## 7. Kolejność commitów (branch `feature/edf-booster` od `devel`)

1. `control_allocator: booster IFODRONE (IFO_BST_*), default wyłączony` + test
2. `gz: model ifodrone – 4× EDF booster QX-64` (submoduł) + bump
3. `49000_gz_ifodrone: booster na EC7, kalibracja CT/THR`
4. `4072_ifodrone: booster na S7` (po Q7/Q8)
5. `docs: booster`

---

## 8. Ryzyka

| # | Ryzyko | Mitygacja |
|---|--------|-----------|
| **R2** | **Krytyczne.** Booster niesie ~70% ciężaru na r = 0.605 m, a autorytet tiltów to ~1 Nm (~0.1–0.5 rad/s²/jedn.). **Awaria jednego EDF-a/ESC ⇒ moment kilka Nm ⇒ niesterowalny.** Rozjazd 5% ciągu (~0.3 N/EDF) daje ~0.25 Nm. Offset CG 1 cm daje ~0.25 Nm stale. | Jeden sygnał = brak redundancji (świadoma decyzja). Wspólna kalibracja ESC, pomiar ciągu na wadze, CG na centroid, test asymetrii w SITL, pierwsze loty na uwięzi z g = 1. |
| R1 | Zapas yaw maleje z g (§3). | Dobór g z logów, start g = 1–2, test pełnego drążka yaw. |
| R3 | Nieliniowość EDF (u^1.66) ≠ coax (kwadrat z offsetem), więc proporcja ciągów zmienia się z gazem. | Integrator Z + kalibracja CT_b w punkcie zawisu. |
| R4 | Hover w jednostkach CT > 0.7 (masa, niższy punkt pracy coax): estymator tnie do 0.9, za mały T/W. | §4: reguła i skala s ≥ 0.65. |
| R5 | Stare parametry na płycie. | `param set` + weryfikacja z logu. |
| R6 | Odwrócona oś w SDF (booster dmucha w dół). | SITL: g ↑ ⇒ komenda coax ↓ w zawisie; na realu test kierunku na stole. |
| R7 | Moment reakcyjny boostera przy rozjeździe CW/CCW (różne ESC/EDF) daje stały moment yaw na coax. | Integrator yaw; para CW/CCW z tej samej partii. |

## 9. Odpowiedzi
Q7. Możesz dodać 0.9kg
Q8. Może być na ten moment
Q9. Ta sama co tilty
Q10. odległość w płaszczyźnie XY od autopilota. 7cm to dodatek do promienia względem edfów tilt (tylko pamietaj że w konfiguracji X a nie + jak silniki)
