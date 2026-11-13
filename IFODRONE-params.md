# IFODRONE – parametry platformy

Źródło prawdy dla konfiguracji parametrów IFODRONE. Opisuje **co** ustawiają skrypty
startowe, **dlaczego** każda wartość musi taka być i czym real różni się od SITL.

| Plik | Rola |
|---|---|
| `ROMFS/px4fmu_common/init.d/airframes/4072_ifodrone` | real, Matek H743 Slim (V3/V4) |
| `ROMFS/px4fmu_common/init.d-posix/airframes/49000_gz_ifodrone` | SITL gz |
| `ROMFS/px4fmu_common/init.d/rc.ifodrone_defaults` | wspólne, ładowane przez oba airframe'y (zamiast `rc.mc_defaults`) |
| `ROMFS/px4fmu_common/init.d/rc.ifodrone_apps` | start modułów (zamiast `rc.mc_apps`) |
| `boards/matek/h743-slim/init/rc.board_defaults` | `SYS_AUTOSTART 4072`, dzielnik baterii 11 / 40 |
| `boards/matek/h743-slim/init/rc.board_sensors` | start IMU, baro i kompasu QMC5883L |

**Zasada:** w airframe zostaje parametr, który (a) różni się od defaultu firmware'u albo
(b) jest równy defaultowi, ale jego zła wartość już kiedyś spowodowała incydent i jest
przypięty celowo (oznaczone **[pin]**). Wszystko inne usuwamy. **Najpierw przeczytaj
[Pułapki](#pułapki)**: edycja pliku airframe sama z siebie **nie** naprawia płyty.

**Styl plików airframe (od 10.11):** w skryptach zostają tylko parametry, nagłówki sekcji i
najwyżej jednolinijkowe komentarze. Każde uzasadnienie, historia incydentu i wyliczenie trafia
**tutaj**, nie do airframe'u. Kolejność sekcji jest wspólna dla 4072 i 49000 (alokacja →
geometria rotorów → serwa → trymy IFO → wyjścia PWM/gz → RC → porty → sensory → bateria →
attitude/rate → pozycja → failsafe/SITL), więc bloki sterowania da się porównać 1:1 diffem.

---

## 1. Alokacja sterowania (control_allocator)

| Parametr | Real | SITL | Dlaczego |
|---|---|---|---|
| `CA_AIRFRAME` | 90 | 90 | `ActuatorEffectivenessIfodrone`: macierz zależna od stanu, przeliczana co cykl. |
| `CA_ROTOR_COUNT` | 7 | 7 | 2 coax + 4 boczne EDF-y + rotor 6 = booster pionowy (4× EDF, jeden sygnał; [§15](#15-booster-pionowy-4-edf)). Indeksy 0–5 bez zmian (okablowanie). |
| `CA_METHOD` | 0 | 0 | **Krytyczne.** Default 2 (AUTO) → `getDesiredAllocationMethod` = SEQUENTIAL_DESATURATION → `mixYaw()` „reduce thrust only” ściąga ciąg pary coax na rzecz yaw. Objaw: gaz nie działa na realu, działa w SITL. |
| `MC_AIRMODE` | 0 **[pin]** | 0 | Przy `CA_METHOD 0` nieczytany (czyta go tylko sequential desaturation; `mixer_module` go tylko deklaruje). Przypięty jako bezpiecznik: gdy `CA_METHOD` na płycie wróci do 2 (stało się 01.09), airmode dodatkowo podbijałby ciąg pod roll/pitch, a ten realizują serwa. SITL miał 1 z komentarzem „Airmode off” – ujednolicone na 0. |

## 2. Geometria rotorów (FRD; geometria identyczna real/SITL, CT/KM różnią się skalą Fz – §11)

| Parametr | Wartość | Dlaczego |
|---|---|---|
| `CA_ROTOR0_PZ` / `CA_ROTOR1_PZ` | −0.1 / +0.1 | Górny coax nad CoM (−Z w FRD), dolny pod. Współliniowe z osią Z ⇒ **zerowy** wpływ na macierz (r × F = 0). Poprawione 10.11 tylko dla porządku (real miał odwrotnie, SITL miał oba +0.1). |
| `CA_ROTOR0/1_AZ` | −1 **[pin]** | Ciąg w górę. Pełne wektory P/A zostają jawne mimo zer: to blok geometrii porównywany 1:1 z SITL. |
| `CA_ROTOR0_KM` / `CA_ROTOR1_KM` | real +0.4 / −0.4; SITL +0.556 / −0.556 | SITL = real ÷ s (s = 0.72, §11) – zachowuje wiersz yaw KM·CT. Rotor 0 CCW, rotor 1 CW. **Nie odwracać** – odwrócony znak rozkręca yaw > 40 rad/s (gz). Na płycie znaleziono kiedyś −0.4 dla rotora 0. |
| `CA_ROTOR0/1_CT` | real 0.6; SITL 0.432 | SITL = real · s (s = 0.72, §11). Stosunek ct coax:EDF ≈ nachylenie siły w punkcie pracy (37.6 vs 3.7 N/u). Coax musi dominować w kolumnie Fz, inaczej alokator realizuje Fz kolektywnym ciągiem EDF-ów i dron nie wznosi się. **Wartość absolutna też ma znaczenie:** dla IFODRONE skala normalizacji ciągu w alokatorze zostaje 1 (udokumentowane w `ControlAllocator.cpp`), więc komenda coax ≈ \|Fz\| / (2·CT0) – zmiana CT przesuwa punkt zawisu i wymaga ponownej kalibracji `IFO_THR_HOVER`/`MPC_THR_HOVER` (§11). |
| `CA_ROTOR2..5_P*` | ±0.535 radialnie, PZ 0.045 | Ramiona: 2=przód, 3=prawo, 4=tył, 5=lewo. Patrz [problem PZ](#problemy-otwarte). |
| `CA_ROTOR2..5_A*` | radialne, **AZ 0** | Osie bazowe POZIOME; kąt spoczynkowy daje `IFO_TILT_HOVER`, alokator obraca osie do bieżącego tiltu. AZ musi być jawne – default to −1. |
| `CA_ROTOR2..5_KM` | 0 | Moment reakcyjny EDF niemodelowany (w SDF `momentConstant` EDF też musi być 0 – regresowało). |
| `CA_ROTOR2..5_CT` | real 0.1; SITL 0.072 | SITL = real · s. Uczciwe CT; podniesienie psuje wiersze sił (sprzężenie Fz ↔ kolektywny tilt, crash 15.07). |
| `CA_ROTOR2..5_TILT` | 1..4 | Rotor 2+N ↔ serwo `CA_SV_TL<N>`. |
| `CA_ROTOR6_PX/PY/PZ` | 0 / 0 / 0.045 | Booster (4× EDF w X, r = 0.605 m) modelowany jako **jeden** rotor w centroidzie: brak momentów roll/pitch. PZ jak EDF-y tilt. |
| `CA_ROTOR6_AX/AY/AZ` | 0 / 0 / −1 **[pin]** | Ciąg w górę (FRD −Z); AZ jawne – patrz pułapka kierunku osi w SDF. |
| `CA_ROTOR6_KM` | 0 | Momenty reakcyjne znoszą się po przekątnych (2× CW + 2× CCW); `momentConstant 0` w SDF. |
| `CA_ROTOR6_CT` | real 1.4 (placeholder); SITL 1.008 | 1.4 z krzywej QX-64 (sieczna 51–59 N/u na 4 EDF-y, CT0 0.6); SITL = 1.4 · s. Real: nieużywane przy `IFO_DEVEL 1`, do kalibracji w Etapie 4 (§15). |
| `CA_ROTOR6_TILT` | 0 | Bez serwa (indeks z tiltem ⇒ booster wyłączony, §4). |

## 3. Serwa tiltu

| Parametr | Real | SITL | Dlaczego |
|---|---|---|---|
| `CA_SV_TL_COUNT` | 4 | 4 | |
| `CA_SV_TL*_MINA/MAXA` | ±60 | ±35 | Muszą opisywać **rzeczywisty** kąt przy PWM MIN/MAX: dδ/ds wchodzi do jakobianu alokatora. SITL: ±35 = uczciwa redukcja autorytetu + ograniczenie najgorszego tiltu (razem z `SIM_GZ_SV_MINA/MAXA`). Real: pełny mechaniczny zakres serwa. Patrz [rozbieżności](#rozbieżności-real-vs-sitl). |
| `CA_SV_TL*_TD` | 0/90/180/270 | jw. | **Nie** czytane przez efektywność IFODRONE (zawias liczony z osi bazowej: `hingeAxisForBase`). Zostaje jako opis ramienia dla QGC i konwencji „side N”. |
| `CA_SV_TL*_CT` | 0 | 0 | Też nieczytane przez IFODRONE; default 1 = „yaw” – 0 chroni przed interpretacją przez kod stockowy. |
| `CA_SV0..3_SLEW` | 0 **[pin]** | 0 | Serwa to JEDYNY aktuator roll/pitch; każdy limit narastania = czyste opóźnienie fazowe w pętli attitude. Przeniesione na real 01.09. |

## 4. Trymy i wzmocnienia alokacji IFO (`control_allocator/module.yaml`)

| Parametr | Real | SITL | Dlaczego |
|---|---|---|---|
| `IFO_TILT_HOVER` | 0.0 **[pin]** | 0.0 | Kąt serw przy zerowym żądaniu momentu; 0 = osie EDF poziomo = PWM 1500. Historia: param zgubiony przy przetasowaniu modułów (07.14 → hover 0°), wcześniej default 45, na płycie znaleziono zapisane −45 (01.09). **Ustawiany dokładnie raz w pliku** – przy `set-default` wygrywa OSTATNIA linia. |
| `IFO_CA_LIN_TAU` | 0.05 **[pin]** | 0.05 | Stała czasowa filtru punktu linearyzacji macierzy efektywności (kąty tiltu / ciąg EDF). Wcześniej macierz liczona z surowej poprzedniej alokacji ⇒ pętla algebraiczna między cyklami z ujemną wartością własną ⇒ serwa przerzucane co cykl (szybkie zmiany PWM, przy `IFO_TILT_HOVER −45` pełnozakresowy cykl graniczny ~400 Hz, logi 03.09). **0 = stare, niestabilne zachowanie – nie ustawiać.** |
| `IFO_EDF_TRIM` | 0.2 **[pin]** | 0.7 | Bazowy ciąg EDF; pary przeciwległe alokowane różnicowo wokół niego. Ustala też minimalny ciąg EDF przyjmowany przy skalowaniu autorytetu tiltu (moment tiltu ∝ ciąg EDF). **Największa rozbieżność real/SITL** – patrz niżej. |
| `IFO_EDF_GAIN` | 1.0 **[pin]** | 1.0 | Wzmocnienia po alokacji = tylko awaryjne gałki; < 1 ukrywa wzmocnienie obiektu przed alokatorem i psuje księgowanie saturacji. Kod przycina do [0, 1]. SITL miał 1.5 – po przycięciu i tak działało jak 1.0, ujednolicone. |
| `IFO_TILT_GAIN` | 1.0 **[pin]** | 1.0 | jw. SITL miał 2.5 (przycinane do 1.0). |
| `IFO_BST_ROTOR` | 6 | 6 | Indeks rotora boostera. Booster **nie** jest w pseudo-inwersji: jego kolumna (wszystkie 6 wierszy) jest wsuwana po g/2 w kolumny coax i zerowana, więc B·u zostaje fizycznym wrenchem. −1 = stare zachowanie (macierz identyczna). Zły indeks (0/1, rotor z tiltem, poza zakresem) ⇒ booster wyłączony + jedno ostrzeżenie. |
| `IFO_BST_RATIO` | 2.0 | 2.0 | g: `u_boost = g · mean(coax0, coax1)`, zakres 0..10. Gałka podziału Fz (model CT_b ≈ 1.4 vs CT0 0.6): g=1 → 54 %, 2 → 70 %, 3 → 78 %, 4 → 82 %. Większe g ⇒ niższy punkt pracy coax ⇒ mniejszy zapas yaw (§15). Nie 7. rotor w pseudo-inwersji: pinv dzieli komendy ∝ CT (ciąg ∝ CT²), brak gałki. Przy `IFO_DEVEL 1` traktowane jak 0. |
| `IFO_DEVEL` | 1 | 0 | Tryb deweloperski: booster skonfigurowany (suwak 7 w teście silników QGC, `actuator_test set -m 7 -v 0.1 -t 2`), ale w locie g = 0 (kolumny coax jak bez boostera). Real zostaje na 1 do kalibracji jednostek Fz (Etap 4); SITL na 0. |

## 5. Wyjścia PWM (tylko real)

Trzy równoległe numeracje: `CA_ROTOR2` = funkcja 103 = „Motor 3” w QGC. „side N” = ramię serwa `CA_SV_TL<N>`.

| Wyjście | Funkcja | Element |
|---|---|---|
| S1 / S2 | 101 / 102 | coax góra / dół |
| S3 / S4 / S5 / S6 | 106 / 104 / 105 / 103 | EDF lewy / prawy / tył / przód |
| S7 | 107 | booster: 4× EDF w górę, Y-kabel (sygnał + GND do 4 ESC, +5 V BEC tylko z jednego) |
| S8 | 0 **[pin]** | wolne |
| S9 / S10 / S11 / S12 | 202 / 203 / 204 / 201 | tilt prawy / tył / lewy / przód |

| Parametr | Wartość | Dlaczego |
|---|---|---|
| `PWM_MAIN_FUNC1..12` | jw. | **Fizyczne okablowanie** po przelutowaniu 02.09. Regresowało 2× przez „porządkowanie” na S3–S6, gdy EDF-y były jeszcze na S5–S8. Teraz S3–S6 jest poprawne – nie cofać. 01.09: FUNC6 był duplikatem 103, a 105 nie było nigdzie (podwójny ciąg z przodu, zero z tyłu). S7 = 107 (booster). FUNC8 = 0 jawnie, bo kiedyś niosło EDF-a. **S7 dzieli TIM4 z S8–S10** (S9/S10 = serwa tilt) ⇒ grupa tylko PWM 400 Hz, **bez DShot** na boosterze. |
| `PWM_MAIN_MIN1..7` | 1020 | Uzbrojony idle: ESC uzbrojony, śmigła stoją (próg startu ESC ~1050). EDF-y i tak dostają `IFO_EDF_TRIM` po uzbrojeniu. MIN7 1020 = booster, idle po uzbrojeniu. |
| `PWM_MAIN_CENT9..12`, `PWM_MAIN_DIS9..12` | 1500 | Serwo: środek = 0° = rozbrojone, rampa uzbrajania bez skoku. Odwrócone cięgło poprawiać reverse'em, nie przesuwaniem środka. |
| *(usunięte)* `PWM_MAIN_MAX1..6/9..12`, `DIS1..6`, `MIN9..12` | 2000 / 1000 / 1000 | Równe defaultom PX4 – zakres 1000..2000 µs. |

## 6. RC (CRSF), uzbrajanie, tryby

| Parametr | Real | SITL | Dlaczego |
|---|---|---|---|
| `crsf_rc start -d /dev/ttyS4` (w `rc.ifodrone_apps`) | ✔ | – | Odbiornik siedzi na **dedykowanych padach RC** (`RC_SERIAL_PORT "/dev/ttyS4"`). Płyta buduje `crsf_rc`, nie `rc_input`. |
| `RC_CRSF_PRT_CFG` | **nieustawiony** (0) | – | Enum zna tylko Disabled/GPS1/TEL1..4, ttyS4 nie ma etykiety. Ustawienie na TEL3 (103) dało jednocześnie „no manual control input” i zabity backpack MAVLink. Linia zniknęła też raz przy edycji (02.09). **Nigdy TELx.** |
| `COM_RC_IN_MODE` | 2 | 0 | Real: RC lub MAVLink z fallbackiem. SITL: obejście (patrz §13). |
| `RC_CHAN_CNT`, `RC_MAP_ROLL/PITCH/THROTTLE/YAW` | 16, 1/2/3/4 | – | Defaulty = 0 = nieprzypisane ⇒ po resecie radio całkowicie martwe. |
| `RC_MAP_FLTMODE`, `COM_FLTMODE1/3/4/6` | 7; 8/1/1/11 | – | Stabilized / Altitude / Altitude / Land. |
| `RC_MAP_ARM_SW` | 5 | – | Uzbrajanie przełącznikiem. |
| `COM_ARM_SWISBTN` | 0 **[pin]** | – | Przełącznik, nie przycisk chwilowy. Nie jest na keep-liście resetu. |
| `MAN_ARM_GESTURE` | 0 | – | Po resecie wraca do 1 = uzbrajanie gestem drążków. |
| *(usunięte 10.11)* `RC1..8_MIN/TRIM/MAX` | – | – | Kalibracja radia należy do płyty (QGC → `param set`), nie do airframe'u. `RC*` jest na keep-liście resetu z `rcS`, więc przeżywa zmianę `SYS_PARAM_VER`. **Ginie** przy ręcznym `param reset_all` bez argumentów i przy wymianie płyty – wtedy radio ma defaulty 1000/1500/2000 i trzeba **przekalibrować w QGC przed lotem** (szczególnie `RC3_TRIM`: gaz ma trim na dole, 994, a default 1500 dałby środek drążka = zero gazu). Wartości z działającej płyty (log 01.09) dla porównania: RC1/2 989/1500/2012, RC3 994/994/2012, RC4 989/1500/2010, RC5..8 999/1499/2000. |

## 7. Porty szeregowe / MAVLink (tylko real)

Wszystkie wpisy zostają (także równe defaultom) – to pełny kontrakt portów, a `SER_*`/`MAV_*` giną przy `reset_all`.

| Parametr | Wartość | Dlaczego |
|---|---|---|
| `MAV_1_CONFIG`, `MAV_1_MODE`, `MAV_1_RADIO_CTL`, `MAV_1_FLOW_CTRL`, `SER_TEL1_BAUD` | 101, 0, 1, 2, 57600 | Radio SiK na TEL1, profil GCS (Normal). |
| `SER_TEL3_BAUD`, `MAV_0_CONFIG`, `MAV_0_MODE`, `MAV_0_FLOW_CTRL`, `MAV_0_RADIO_CTL`, `MAV_0_FORWARD`, `MAV_0_RATE` | 460800, 103, 0, 2, 1, 1, 9600 | **TEL3 = backpack telemetrii MAVLink**, nie odbiornik RC. Reset przywracał 57600 ⇒ ELRS milkł. Wartości z logu działającej płyty. |

## 8. Sensory / estymator (tylko real)

Wszystkie wartości poniżej są równe defaultom firmware'u i są przypięte **[pin]**: dron latał wcześniej
z GPS/kompasem na DroneCAN, a jeszcze wcześniej bez magnetometru, i płyty z tymi zapisanymi wartościami gubiły kurs/GPS.

| Parametr | Wartość | Dlaczego |
|---|---|---|
| `UAVCAN_ENABLE` | 0 | GPS i kompas nie są na CAN (płyta V4, od ~24.07). |
| `GPS_1_CONFIG`, `SER_GPS1_BAUD`, `GPS_1_PROTOCOL` | 201, 0 (auto), 1 (u-blox) | GPS na UART TX2/RX2 = GPS1 = ttyS1. |
| `SYS_HAS_MAG`, `EKF2_MAG_TYPE`, `EKF2_MAG_CHECK`, `SENS_MAG_MODE` | 1, 0, 1, 1 | Kompas QMC5883L @0x0d na I2C1, start w `rc.board_sensors`. |
| `CAL_MAG0_ROT` | **nie w airframe** | Wartość z kalibracji w QGC. Kompas to instancja 0 (`CAL_MAG0_ID 527625`); stare `CAL_MAG2_ROT 6` było no-opem. Błąd kursu kosztuje ciąg (yaw robi para coax). |
| *(usunięte)* `UAVCAN_SUB_GPS 0` | – | Martwy przy `UAVCAN_ENABLE 0` (moduł uavcan nie startuje). |
| *(usunięte)* `IMU_GYRO_DNF_EN 2` | – | Patrz [ocena zmian z 08.09](#ocena-zmian-z-0809-clipping). |

## 9. Bateria (tylko real)

| Parametr | Wartość | Dlaczego |
|---|---|---|
| `BAT1_N_CELLS`, `BAT1_V_EMPTY`, `BAT1_V_CHARGED` | 6, 3.3, 4.2 | 6S Li-Po. |
| `BAT1_V_DIV`, `BAT1_A_PER_V` | nie w airframe | Default płyty 11 / 40 (`rc.board_defaults`); zmierzona kalibracja (11.0046) żyje w zapisanych parametrach. |

## 10. Attitude + rate (identyczne real/SITL)

Do 01.09 real nie ustawiał **żadnego** z nich i latał na stockowych wzmocnieniach quada
(MC_YAW_P 2.8, MC_YAWRATE_I 0.1, MC_ROLL/PITCH_P 4.0, D 0.003). Przy parze coax niosącej
yaw I ciąg przewzmocniony yaw rozjeżdża parę i zjada ciąg.

| Parametr | Wartość | Uwagi |
|---|---|---|
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | 1.0, 1.0, 0.48 | Czyta `ifo_att_control` (P attitude → rate sp). |
| `MC_ROLLRATE_P/I/D`, `MC_RR_INT_LIM` | 0.20 / 0.20 / 0.0003, 0.50 | `mc_rate_control` (MUSI być startowany w `rc.ifodrone_apps` – regresja 05.08). |
| `MC_PITCHRATE_P/I/D`, `MC_PR_INT_LIM` | 0.30 / 0.20 / 0.0003, 0.50 | |
| `MC_YAWRATE_P/I/D`, `MC_YR_INT_LIM` | 0.22 / 0.01 / 0.0, 0.30 | I/D/limity równe defaultom też są przypięte – to tabela strojenia, a defaulty PX4 dla MC zmieniały się między wydaniami. |
| *(usunięte)* `MC_*RATE_K` | 1.0 | Default; postać idealna PID = no-op. |
| *(usunięte)* `MC_YAW_WEIGHT` | 0.4 | Czyta go tylko `mc_att_control`, który na IFODRONE nie jest startowany; i tak równy defaultowi. |

**Strojenie SITL 10.11 (wynik: wartości bez zmian).** Pętla pitch rate w gz ma zapas wzmocnienia
tylko ~1.3: `MC_PITCHRATE_P` 0.4 / 0.45 / 0.6 (oba osie) ⇒ cykl graniczny 2.6–2.9 Hz (pitch 17–21°
w reposition), 1.0 ⇒ przewrotka. `CA_SV*_SLEW 0.15` tego nie ratuje (0.6 + slew: dalej 2.9 Hz).
Podniesienie pętli zewnętrznej (`MC_*_P` 2.0 + `MC_*RATE_FF` 0.15–0.2 przy rate P 0.18–0.2) zwiększa
resztkową oscylację 0.8 Hz (RMS 1.0° → 2.2–2.6°) przy podobnym maksimum (5.6–5.8° vs 6.3°).
`CA_ROTOR2..5_PZ −0.045` (zgodne z SDF) też bez zysku (6.5°). Przyczyna to **model serwa w SDF, nie
wzmocnienia**: linki `motor_2..5` mają bezwładność 0.0167 kg·m² (geometria cylindra 50 g ⇒ ~1e-5;
wartość wygląda na literówkę ×100 od iyy rotora) ⇒ moment reakcji obracanego linka jest przeciwny do
momentu z tiltu (zero w prawej półpłaszczyźnie ~11 rad/s przy trymie 0.7), a tarcie złącza 1.0 Nm przy
`p_gain` 40 = martwa strefa 1.4° (0.04 komendy) ⇒ korekty < ~0.04 nie ruszają serwa ⇒ wolna oscylacja
~0.5 Hz i 6° opadania nosa przy rozpędzaniu. Diagnostyka (tymczasowo podmieniony SDF, przywrócony):
bezwładność linków 0.001 ⇒ rate P 1.0 stabilne, pitch w reposition 3.7° (RMS 1.05°, reszta 0.47°).
Do decyzji: poprawić `<inertia>` linków `motor_2..5` (i ew. `<friction>`) w `models/ifodrone/model.sdf`
i dopiero wtedy stroić rate P w SITL. **Wzmocnień z obecnego SITL nie traktować jako górnej granicy dla
realu** – real ma lekkie serwa, ograniczeniem będzie szum żyro i prędkość serwa.

Alokacja działa bez normalizacji RPY, a wiersze roll/pitch są dzielone przez stały
`torque_ref` (moment serwa przy `IFO_EDF_TRIM`), więc żądanie ±1 ≈ pełny autorytet serw
**przy trymie**. Fizyczny moment na jednostkę żądania skaluje się z `IFO_EDF_TRIM` – stąd
wzmocnienia rate nie przenoszą się 1:1 między realem (0.2) a SITL (0.7).

## 11. Pozycja / tryby manualne (`ifo_pos_control`) i referencje ciągu

| Parametr | Real | SITL | Dlaczego |
|---|---|---|---|
| `IFO_THR_HOVER` | 0.5 **[pin]** | 0.64 | Środek drążka gazu w Stabilized + człon hover. **Kalibracja per pojazd**, w jednostkach `vehicle_thrust_setpoint` (surowe CT): hover = 2·`CA_ROTOR0_CT`·komenda coax w zawisie = 1.2·0.496 ≈ 0.6 w gz (`hover_thrust_estimate` 0.597). SITL 0.5 → auto takeoff stał ~11 s rozkręcony na ziemi, bo brakujące 0.1 musiał nadrobić integrator Z (`IFO_VEL_Z_I` 0.2) – poprawione 10.11. Real: odczytać `hover_thrust_estimate` z logu zawisu (0.5 oznacza komendę coax ~0.42). |
| `MPC_THR_HOVER` | 0.5 **[pin]** | 0.64 | Czyta go `land_detector` (dopóki estymator hover thrust nie zbiegnie). Nieaktualna wartość = dron „wylądowany”, w Alt/Pos włączony docisk do ziemi. Trzymać równe `IFO_THR_HOVER`. |
| `IFO_THR_XY_MAX` | 0.2 **[pin]** | 0.072 | Twardy limit siły bocznej body-XY. Real 0.2 „cięższe EDF-y” (decyzja użytkownika). SITL miał podwójną linię 0.2 → 0.1 (wygrywała ostatnia) – zostawiona tylko 0.1. |
| `IFO_THR_MAX` / `IFO_THR_MIN` | real default (bez zmian) | 1.0 / 0.09 | **Skala jednostek Fz s = 0.72 (11.11, tylko SITL).** Z boosterem i +0.9 kg hover w starych jednostkach CT wynosił ~0.89 > `IFO_THR_MAX` 0.8, a estymator tnie hover do [0.1, 0.9] ⇒ dron nie startował. Fix: wszystkie `CA_ROTOR*_CT` ×s, coax KM ÷s, `IFO_THR_*` ×s (HOVER/MPC_THR_HOVER 0.64, MAX 1.0, MIN 0.09, XY_MAX 0.072). Dolna granica s ≥ 0.65: `CT_side·s·cos35°` musi zostać > 0.05, inaczej alokator zeruje wiersze Fx/Fy. Wynik: `hover_thrust_estimate` 0.643 vs przewidziane 0.644, coax 0.223, booster 0.446. Real zostaje na starych jednostkach do Etapu 4 (§15). |
| `IFO_STB_VD_TAU` | 2.0 **[pin]** | 2.0 | Horyzont washoutu tłumienia w Stabilized. Przy 10 s bias akcelerometru 0.26 m/s² żąda całego autorytetu bocznego (incydent 08.09). Default modułu był 10 s; w working tree (równoległa zmiana w `ifo_pos_control_params.c`) jest już 2.0 – przypięty, bo default się zmieniał. Dodany też do SITL – to prawo sterowania, nie obejście sprzętowe. |
| `IFO_VEL_MAX_XY`, `IFO_VEL_XY_P`, `IFO_POS_XY_P` | 2.0, 1.2, 0.6 | jw. | Profil „gentle translation”: każda siła boczna zaburza attitude. `IFO_VEL_MAX_XY` ogranicza też estymatę tłumienia (`_vel_damp_body`). |
| `MPC_XY_CRUISE`, `MPC_ACC_HOR`, `MPC_ACC_HOR_MAX`, `MPC_JERK_AUTO` | 2.0, 1.0, 2.0, 2.5 | jw. | Kształtowanie trajektorii auto w `flight_mode_manager`. `MPC_JERK_AUTO` 1.0 → 2.5 (11.11): przy celu `PositionSmoothing` ogranicza prędkość do v ≈ d·J/(2A), czyli setpoint dochodzi wykładniczo z τ = 2·ACC_HOR/JERK = 2 s; „dron staje przed punktem” to był pełznący setpoint, nie regulator (dron trzymał setpoint ±5 cm). SITL 10 m: setpoint w celu po ~8 s zamiast ~15 s, peak pitch bez zmian (~6°), przestrzał ~0.15 m. Uwaga: 2.0 i 1.0 są poniżej `@min` z metadanych (3.0 / 2.0) – PX4 tego nie wymusza, QGC pokaże na czerwono. |
| `IFO_VEL_Z_P` / `IFO_VEL_Z_I` | default 1.0 / 0.2 | jw. | Wolne wznoszenie przy takeoff (vz 0.3 m/s zamiast ~1.1) nie było kwestią wzmocnień: rampa startu zaczyna limit wznoszenia od −g/`IFO_VEL_Z_P` przy dronie stojącym na ziemi, integrator Z nawijał się w dół (~1.8 m/s²) i odwijał ~P/I = 5 s. **Naprawione w kodzie** (10.11, `ifo_pos_control`: `resetIntegralZ()` przez cały stan `rampup`, auto i Altitude): oderwanie 2.2 s po starcie rampy (było 4.0 s), 90 % wysokości po 6.8 s (było 17.3 s), vz_max 1.07 m/s (było 0.31). Wzmocnienia bez zmian. |
| `MPC_YAW_MODE` | 4 | 4 | 4 = „towards waypoint (yaw first)” (**nie** „trzymaj kurs”, jak twierdził stary komentarz): w misji/goto dron najpierw obraca się w miejscu nosem do celu, dopiero potem leci – yaw (różnica coax) i siła boczna EDF nie są żądane jednocześnie. Zweryfikowane w SITL 10.11. Stały kurs = 5 (yaw fixed, powyżej `@max` 4 w metadanych). |

## 12. Failsafe

| Parametr | Real | SITL | Dlaczego |
|---|---|---|---|
| `FD_FAIL_R/P` | default 60° | 0 | Real **celowo** na defaulcie: przy domyślnym `CBRK_FLIGHTTERM` detektor rozbraja tylko w oknie `COM_LKDOWN_TKO` (3 s) po starcie, w locie tylko ostrzega – czyli wyłącza silniki przy przewrotce na starcie. Linia `0` była na realu 01–02.09 i zniknęła w `1ce9b8d9cc` (razem z innymi komentarzami); świadomie nie przywrócona – do potwierdzenia przez użytkownika. |

## 13. Tylko SITL

| Parametr | Wartość | Dlaczego |
|---|---|---|
| `SIM_GZ_EN` | 1 | |
| `SYS_AUTOSTART` | 49000 | Efekt uboczny: wartość = runtime default ⇒ `param save` jej nie eksportuje ⇒ przy następnym starcie posix `rcS` widzi inny SYS_AUTOSTART i robi `param reset_all`. **Każdy start SITL zaczyna od tego pliku**, bez starych zapisanych paramów. Nie usuwać, jeśli harness polega na czystym starcie. |
| `SIM_GZ_EC_FUNC/MIN/MAX1..6` | 101..106; 100/1000 coax, 350/1000 EDF | Kalibracja gz: ω = MIN + cmd·(MAX−MIN); coax wisi przy ~0.5, EDF max 6 N. Regresowało raz (crash 15.07) – sprawdzać po każdej zmianie SDF/CT. EDF MIN 350 – niski, żeby EDF-y nie rozkręcały się mocno po uzbrojeniu. Brak `SIM_GZ_EC_REV`: dolny rotor w SDF jest już `cw`, zgodnie z `CA_ROTOR1_KM −0.4`. |
| `SIM_GZ_EC_FUNC7/MIN7/MAX7` | 107; 125/1000 | Booster. ω = 125 + 875·u ⇒ 23.2 N @ 1.0, 7.4 N @ 0.5 na EDF (krzywa QX-64, `motorConstant 2.325e-05`). Model (`models/ifodrone/model.sdf`, fork submodułu): 4 linki `booster_0..3`, wszystkie pluginy `motorNumber 6` (= Y-kabel), `momentConstant 0`, śmigła niebieskie = CCW (0/2), czarne = CW (1/3). `base_link` 2.0 → 2.8 kg, ixx/iyy 0.03 → 0.177, izz 0.05 → 0.343 (inercja roll/pitch ~6× większa – tak samo na realu). Oś jointów w ENU = +Z (w górę) – po zmianie SDF sprawdzić kierunek (R6: g ↑ ⇒ komenda coax ↓ w zawisie). |
| `SIM_GZ_SV_FUNC/MINA/MAXA/MIN/DIS/MAX1..4` | 201..204; ±35; 1000/1500/2000 | Zmieniać RAZEM z `CA_SV_TL*_MINA/MAXA`. |
| `COM_RC_IN_MODE 0`, `COM_RCL_EXCEPT 4`, `NAV_RCL_ACT 0`, `FD_ACT_EN 0`, `FD_ESCS_EN 0`, `CBRK_SUPPLY_CHK 894281`, `FD_FAIL_R/P 0` | | **Obejścia bezpieczeństwa SITL – nigdy na real.** |
| *(usunięte)* `SYS_AUTOCONFIG 1` | | No-op: posix `rcS` sprawdza go PRZED airframe'em, a `set-default` nie jest eksportowany. |
| *(usunięte)* `MPC_THR_MAX 30.0` | | Czytają go tylko `mc_att_control`/`mc_pos_control` – nie startowane. Wartość i tak poza zakresem [0, 1]. |
| *(usunięte)* `MPC_MANTHR_MIN 0.08` | | Default (komentarz „ifo_att_control manual mode” był nieprawdziwy). |
| *(usunięte)* `COM_ARM_WO_GPS 1`, `NAV_DLL_ACT 0`, `SIM_GZ_EC_DIS1..6 0`, `SIM_GZ_EC_REV 0` | | Defaulty. |
| *(usunięte)* `FD_ACT_MOT_THR 0.0` | | Martwy przy `FD_ACT_EN 0`. |

## 14. `rc.ifodrone_defaults` i `rc.ifodrone_apps`

* `rc.ifodrone_defaults` = odpowiednik `rc.mc_defaults` (który dla `VEHICLE_TYPE ifodrone` się nie ładuje):
  `MAV_TYPE 2`, `IMU_GYRO_RATEMAX 800`, `NAV_ACC_RAD 2`, `RTL_RETURN_ALT 30`, `RTL_DESCEND_ALT 10`.
  **Celowo bez `SYS_AUTOSTART`/`SYS_AUTOCONFIG`** – 49000 tutaj + `SYS_AUTOCONFIG 1` w 4072 zrobiły z
  realnej płyty konfigurację samoniszczącą się (01.09).
* `rc.ifodrone_apps`: `crsf_rc` (ttyS4) → `control_allocator` → `ifo_pos_control` → `ifo_att_control` →
  **`mc_rate_control`** (wymagany: `ifo_att_control` emituje tylko rate sp) → `mc_hover_thrust_estimator`
  → `flight_mode_manager` → `land_detector multicopter`.

---

## Rozbieżności real vs SITL

| Parametr | Real | SITL | Decyzja |
|---|---|---|---|
| `IFO_EDF_TRIM` | 0.2 | 0.7 | **Zostawione, do rozstrzygnięcia testem.** Real: wyższy idle EDF = więcej drgań (clipping akcelerometru 08.09 pojawił się przy rozpędzaniu EDF-ów) i prądu. Ale: moment roll/pitch na jednostkę żądania ∝ trim, więc przy tych samych `MC_*RATE_*` pętla na realu ma ~3.5× mniejsze wzmocnienie fizyczne niż w SITL (przy liniowym ciągu EDF). Rekomendacja: przelecieć SITL z `IFO_EDF_TRIM 0.2` (to stan realu) zanim wzmocnienia rate pójdą na real. **Sprawdzone 10.11:** pełny test z obecnym SDF i trymem 0.2 = FAIL w loiter/reposition/mission/rtl (cykl graniczny ~0.8 Hz, roll do 7.7°, pitch do 10.8°, nieprzydzielony moment do 0.15, serwa std 0.07); ten sam zestaw MC_* z poprawionym modelem serwa (bezwładność linków 0.001, tarcie 0.25) = PASS (reposition pitch 5.6°, roll 1.5°). Przy trymie 0.2 martwa strefa serwa w gz jest ~2.8× większa w jednostkach momentu – znowu artefakt modelu, ale też sygnał, że przy trymie 0.2 autorytet roll/pitch na realu jest mały. |
| `CA_SV_TL*_MINA/MAXA` | ±60 | ±35 | Fizycznie uzasadnione tylko jeśli serwo realne naprawdę ma ±60° przy 1000/2000 µs (do zmierzenia na stole). SITL ±35 to decyzja projektowa (autorytet + bezpieczeństwo przewrotki). Jeśli ±35 ma obowiązywać na realu: zawęzić PWM serw do kątów ±35 i ustawić MINA/MAXA ±35 **razem**. |
| `IFO_THR_XY_MAX` | 0.2 | 0.072 | Zostawione (decyzja użytkownika: cięższe EDF-y). W SITL ogranicznik estymaty tłumienia z 08.09 jest przez to nieskuteczny ([Problemy otwarte](#problemy-otwarte), pkt 2). |
| `CA_ROTOR0..6_CT`, `CA_ROTOR0/1_KM`, `IFO_THR_*`, `MPC_THR_HOVER` | 0.6/0.1/1.4, KM 0.4, hover 0.5 | CT ×0.72 (0.432/0.072/1.008), KM ÷0.72 (0.556); hover 0.64 skalibrowany na nowo z boosterem (nie ×s) | **Skala Fz s = 0.72 tylko w SITL**, do kalibracji realu w Etapie 4 (§11, §15). Nie przenosić 1:1 w żadną stronę. |
| `IFO_DEVEL` | 1 | 0 | Real: booster skonfigurowany, ale w locie wyłączony do kalibracji. SITL: pełny booster (g = 2). |
| `COM_RC_IN_MODE`, `COM_RCL_EXCEPT`, `NAV_RCL_ACT`, `FD_*`, `CBRK_SUPPLY_CHK` | default / 2 | obejścia | Celowe. Nie przenosić. |
| `RC_*`, `COM_FLTMODE*`, `COM_ARM_SWISBTN`, `MAN_ARM_GESTURE`, `MAV_*`, `SER_*`, `GPS_*`, sensory, `BAT1_*`, `PWM_*` | ✔ | – | Sprzęt. |
| `SIM_GZ_*`, `SIM_GZ_EN`, `SYS_AUTOSTART` | – | ✔ | Symulator. |
| Ujednolicone 10.11 | | | `MC_AIRMODE` (SITL 1→0), `IFO_EDF_GAIN`/`IFO_TILT_GAIN` (SITL 1.5/2.5→1.0, i tak przycinane), `CA_ROTOR0/1_PZ` (no-op), `IFO_STB_VD_TAU 2.0` dodany do SITL; `IFO_THR_HOVER`/`MPC_THR_HOVER` SITL 0.5→0.6 (kalibracja z `hover_thrust_estimate`). |

## Problemy otwarte

1. **Znak `CA_ROTOR2..5_PZ`.** SDF (ENU, z w górę) ma EDF-y na z = +0.045, czyli **nad** CoM; w FRD
   to PZ = **−0.045**, a obie konfiguracje mają +0.045. Dla poziomej siły EDF PZ daje moment
   (τ_pitch = PZ·Fx), więc alokator zakłada przeciwny znak sprzężenia „siła boczna → pitch/roll”
   niż w symulacji. Wymaga testu w SITL przed zmianą; na realu – zmierzyć, czy oś ciągu EDF jest nad czy pod CoM.
2. **Ogranicznik `_vel_damp_body` (commit e6e3642) = `IFO_VEL_MAX_XY`.** Maksymalne żądanie tłumienia
   to `IFO_STB_VD_P · IFO_VEL_MAX_XY` = 1.5 · 2 = 3.0 m/s², a cały autorytet boczny to
   `IFO_THR_XY_MAX · g / IFO_THR_HOVER` = 3.92 m/s² (real) / 1.63 m/s² (SITL, hover 0.6). Na realu zaklamana
   estymata może nadal żądać ~77 % autorytetu bocznego, w SITL – ~180 % (ogranicznik nic nie robi).
   Zmiana w `src/` (np. limit liczony z autorytetu) – poza zakresem tego porządku.
3. `MPC_XY_CRUISE 2.0` / `MPC_ACC_HOR 1.0` poniżej `@min` z metadanych.
4. `rc.board_sensors`: `qmc5883l -X start -a 13` – opcje PO verbie `start` są ignorowane; ratuje to, że
   default adres i tak 0x0d. Poprawnie: `qmc5883l -X -a 13 start`.
5. **Booster – kalibracja realu (Etap 4) czeka na dane/loty.** Do tego czasu real: `IFO_DEVEL 1`,
   `CA_ROTOR6_CT 1.4` to placeholder, jednostki Fz niezeskalowane. Procedura: [§15](#15-booster-pionowy-4-edf).
6. **Stabilize + pełny drążek yaw ⇒ wznoszenie do ~1.7 m/s** (nie alokator, SITL): ciąg coax ∝ ω², więc
   różnica przy niskiej średniej dokłada ~44 % ciągu coax (~10 % ciężaru). Istniało przed boosterem
   (gorzej: +67 % przy średniej 0.5). Pilot koryguje gazem; możliwe rozwiązanie: linearyzacja ciągu coax.
   Pełny drążek yaw nasyca coax ~30 % czasu przy g = 2.
7. **Asymetria boostera a `MC_RR/PR_INT_LIM`.** Test SITL: jeden EDF −5 % ⇒ stabilne, serwa ~±0.08,
   integratory rate ~0.15 = 30 % limitu 0.5, roll/pitch ±1.3° (symetrycznie ±0.12°). Asymetria ~15 %
   wyczerpie limit integratora.
8. **Awaria jednego EDF-a boostera jest nieodzyskiwalna.** Moment kilka Nm na 0.6 m vs ~1 Nm autorytetu
   tiltów; jeden sygnał = brak redundancji (świadoma decyzja). CG musi leżeć na centroidzie boostera
   (1 cm ≈ 0.25 Nm stałego momentu).

---

## Pułapki

1. **`param set-default` NIE nadpisuje zapisanych wartości.** Ustawia tylko *default*. Każdy param
   kiedykolwiek zapisany na płycie (QGC, `param set`, import pliku) wygrywa z airframe'em. Poprawka w
   pliku + flash ≠ poprawka na płycie. 01.09 po naprawie `SYS_AUTOSTART` zostało 19 starych
   parametrów (m.in. `CA_METHOD 2`, `CA_ROTOR0_KM −0.4`, `IFO_TILT_HOVER −45`, `PWM_MAIN_FUNC6 103`
   jako duplikat, FUNC7/8 = 0). Wniosek: usunięcie linii równej defaultowi niczego na płycie nie zmienia
   – ani nie psuje, ani nie naprawia.
2. **`param reset_all <wzorce>` – argumenty to lista TRZYMANYCH.** `rcS` woła
   `param reset_all SYS_AUTOSTART SYS_PARAM_VER RC* CAL_* COM_FLTMODE* LND_FLIGHT* TC_* COM_FLIGHT*`.
   Przeżywa: `RC*` (z `RC_MAP_*`, `RC_CRSF_PRT_CFG`, kalibracją), `CAL_*`, `COM_FLTMODE*`.
   Ginie: `SER_*`, `MAV_*`, `COM_RC_IN_MODE`, `COM_ARM_SWISBTN`, `MAN_ARM_GESTURE`, `EKF2_*`,
   `SYS_HAS_MAG`, `SENS_*`, całe `CA_*` i `PWM_MAIN_*`. Jedynym „keep” jest airframe. **Nie używać
   ręcznego `param reset_all` do naprawy** – bez argumentów kasuje WSZYSTKO, łącznie z kalibracjami
   IMU/kompasu/RC (`CAL_*`/`RC*` przeżywają tylko reset wołany z `rcS` z keep-listą). Naprawiać
   punktowo: `param set` albo `param reset <NAZWA>`.
3. **Duplikaty: wygrywa ostatnia linia.** `set-default` nadpisuje bieżący default, więc druga linia
   tego samego parametru po cichu unieważnia pierwszą (SITL: `IFO_THR_XY_MAX 0.2` → `0.1`; real:
   `IFO_TILT_HOVER −45` pojawił się raz jako druga linia).
4. **`param save` nie eksportuje wartości równych runtime defaultowi.** Stąd trik z `SYS_AUTOSTART`
   w SITL (reset przy każdym starcie). Z kolei `param show-for-airframe` wypisuje każdy parametr
   z wartością ustawioną przez użytkownika (zapisaną na płycie), bez `RC*`/`CAL_*` – czyli
   kandydatów do nadpisania airframe'u.
5. **`CA_METHOD` musi być 0** (§1). Alokator nie zgłasza błędu – ciąg po prostu nie słucha drążka.
6. **`IFO_EDF_GAIN`/`IFO_TILT_GAIN` są przycinane do [0, 1]** – wartości > 1 nic nie dają (SITL miał 1.5/2.5).
7. **`RC_CRSF_PRT_CFG` nigdy na TELx** (§6). Brak/zła wartość jest całkowicie cicha: płyta się uzbraja,
   telemetria działa, drążki nic nie robią. Po każdej edycji sekcji RC/SERIAL: `grep crsf_rc rc.ifodrone_apps`.
8. **`PWM_MAIN_FUNC*` = fizyczne kable.** Nie „porządkować”. Sprawdzić duplikaty funkcji i funkcje
   nieprzypisane (`103..107`, `201..204` – każda dokładnie raz).
9. **Parametr może być martwy, bo moduł nie startuje.** `mc_att_control`/`mc_pos_control` nie są
   uruchamiane ⇒ `MC_YAW_WEIGHT`, `MPC_THR_MAX`, `MPC_MANTHR_MIN` (poza land detectorem) nic nie robią.
   `IMU_GYRO_DNF_EN 2` bez `IMU_GYRO_FFT_EN 1` nic nie robi (`gyro_fft` nie startuje).
10. **Real nie dziedziczy po SITL.** Każde „działa w sim, nie na realu” zaczynać od diffu paramów, nie od kodu.
11. **Booster jako zwykły 7. rotor = brak gałki podziału Fz.** Pseudo-inwersja dzieli komendy ∝ CT (ciąg ∝ CT²),
    udział stały (~73 % przy CT_b 1.4). Dlatego `IFO_BST_RATIO` (g) i kolumna boostera wsunięta w coax.
12. **g ↑ ⇒ zapas yaw ↓.** Yaw tylko z różnicy coax, |d| ≤ średnia coax. Przy g = 2 coax wisi na ~0.22 (było
    ~0.5 SITL / ~0.42 real). SITL: skok yaw 90° zużył 0.11 z 0.22.
13. **Yaw → Fz przez przycięte średnie coax ×g (naprawione 11.11, nie cofać).** Booster podążał za średnią
    *indywidualnie przyciętych* komend coax: pełny drążek yaw wpychał dolny coax poniżej 0, przycięcie
    podnosiło średnią, ×g ⇒ ~2 g wznoszenia, ucieczka w Stabilize > 30 m. Teraz para coax jest odsaturowana
    **z zachowaniem średniej** (m ∈ [0,1], |d| ≤ min(m, 1−m)), booster liczony z m. Nigdy „clip, potem średnia”.
    Pos/Alt przy pełnym yaw: ±0.3–0.4 m.
14. **Jednostki Fz i hover > 0.7 ⇒ przeskalować.** Fz w surowych CT (skala 1); hover > 0.7 przy `IFO_THR_MAX` 1.0
    i estymatorze tnącym do 0.9 = za mały T/W (dron nie startuje). Skala s (§11) z ograniczeniem **s ≥ 0.65**
    (próg słabych wierszy 0.05 dla `CT_side·s·cos35°`).
15. **`set-default` nie dociera na płytę także dla boostera** – `CA_ROTOR_COUNT 7`, `CA_ROTOR6_*`, `IFO_BST_*`,
    `IFO_DEVEL`, `PWM_MAIN_FUNC7/MIN7` trzeba `param set` + `param save` (pułapka 1, §15).

## 15. Booster pionowy (4× EDF)

**Sprzęt:** 4× QX-Motor 64 mm (2370 g @ 100 %, 750 g @ 50 % ⇒ F ≈ 23.2·u^1.66 N na EDF, 93 N razem), ciąg w górę,
konfiguracja X, r = 0.605 m (= 0.535 + 0.07), pozycje (±0.428, ±0.428), CW/CCW po przekątnych. Jeden sygnał PWM
(S7) przez Y-kabel do 4 ESC (sygnał + GND do wszystkich, +5 V BEC tylko z jednego). W CA: **jeden** rotor
`CA_ROTOR6` w centroidzie (§2).

**Architektura** (`ActuatorEffectivenessIfodrone`): booster nie jest w pseudo-inwersji. `u_boost = g · mean(coax0, coax1)`,
g = `IFO_BST_RATIO`. Kolumna boostera (6 wierszy) jest wsunięta po g/2 w kolumny coax i zerowana, więc B·u = fizyczny
wrench; Fz = (2·CT0 + g·CT_b)·średnia_coax. Para coax odsaturowana z zachowaniem średniej (pułapka 13). Yaw nie rusza
boostera. `IFO_BST_ROTOR −1` = zachowanie sprzed boostera. `IFO_DEVEL 1` = booster skonfigurowany, w locie g = 0.
Szczegóły i uzasadnienia: §4, pułapki 11–15, `plan.md`.

**Stan:** SITL 49000 z boosterem (`IFO_DEVEL 0`, g = 2, s = 0.72) zweryfikowany. Real 4072: skonfigurowany w trybie
deweloperskim (`IFO_DEVEL 1`), kalibracja jednostek czeka na Etap 4.

**Procedura realu (Etap 4):**

1. Flash, potem na płycie `param set` (`set-default` nie nadpisuje zapisanych): `CA_ROTOR_COUNT 7`;
   `CA_ROTOR6_PX 0`, `PY 0`, `PZ 0.045`, `AX 0`, `AY 0`, `AZ −1`, `KM 0`, `CT 1.4`, `TILT 0`; `IFO_BST_ROTOR 6`;
   `IFO_BST_RATIO 2.0`; `IFO_DEVEL 1`; `PWM_MAIN_FUNC7 107`; `PWM_MAIN_MIN7 1020`; `param save`, reboot.
   Zweryfikować z logu (procedura niżej).
2. Stół (dron przywiązany): `actuator_test` motor 7 – ruszają **wszystkie 4** EDF-y, CW/CCW po przekątnych, powietrze
   dmucha w dół (ciąg w górę). Wspólna kalibracja ESC albo stałe endpointy BLHeli; opcjonalnie pomiar ciągu
   każdego EDF-a na wadze przy tej samej komendzie.
3. Zamontować booster, wyważyć CG na centroid boostera. Lot w `IFO_DEVEL 1` (booster wyłączony) jak dotąd: odczyt
   `hover_thrust_estimate` h_dev (nowa masa, stare jednostki) i komendy coax w zawisie.
4. Na uwięzi: `IFO_DEVEL 0`, `IFO_BST_RATIO 1.0`, Stabilize, ostrożny gaz. Odczyt `hover_thrust_estimate` h1 i komend
   coax/booster. Dron nie unosi się przy ciągu `IFO_THR_MAX` ⇒ jednostki za małe (tak było w SITL) – bezpieczna
   porażka. h1 > 0.7 ⇒ s = 0.64 / h1 (musi być ≥ 0.65, inaczej przemyśleć też `CT_side`) i lista skalowania: wszystkie
   CT ×s, coax KM ÷s, `IFO_THR_HOVER`/`MPC_THR_HOVER`/`IFO_THR_MIN`/`IFO_THR_XY_MAX` ×s, `IFO_THR_MAX` ≤ 1.0.
   `CA_ROTOR6_CT` dobrać tak, by model zawisu zgadzał się z h1: Fz = (2·CT0 + g·CT_b)·średnia_coax.
5. Podnosić `IFO_BST_RATIO` 1 → 1.5 → 2 krokami; w każdym: pełny drążek yaw w Alt (zapas yaw), dryf roll/pitch,
   estymata hover.

---

## Procedura weryfikacji parametrów na płycie

**A. Konsola (MAVLink shell / NSH), po boocie z nowym firmware'em:**

```sh
param show SYS_AUTOSTART          # musi być 4072
param show-for-airframe           # wszystkie parametry z wartością zapisaną na płycie (bez RC*/CAL_*).
                                  # Każdy, który jest też w 4072_ifodrone z INNĄ wartością, to stara
                                  # wartość nadpisująca airframe -> param set <NAME> <wartość z airframe>
param show CA_METHOD              # 0
param show IFO_TILT_HOVER         # 0.0
param show PWM_MAIN_FUNC*         # tabela z §5, każda funkcja raz
crsf_rc status                    # running na /dev/ttyS4
mc_rate_control status            # running
```

Poprawki: `param set <NAME> <wartość>` dla każdego rozjechanego, potem `param save` i reboot.
Alternatywnie `param reset <NAME1> <NAME2> ...` (reset wybranych do defaultu = wartości z airframe po rebootcie).

**B. Z logu (`.ulg`) – porównanie z plikiem airframe:**

```python
import re, sys
from pyulog import ULog

airframe, ulg = sys.argv[1], sys.argv[2]   # 4072_ifodrone, log.ulg
want = {}
for ln in open(airframe):
    m = re.match(r'\s*param set-default\s+([A-Z0-9_]+)\s+(\S+)', ln)
    if m:
        want[m.group(1)] = float(m.group(2))

have = ULog(ulg, [], False).initial_parameters
for name, val in sorted(want.items()):
    if name not in have:
        print(f'{name:20} airframe={val:<10} BRAK W LOGU (param nie istnieje w tym firmware?)')
    elif abs(have[name] - val) > 1e-4:
        print(f'{name:20} airframe={val:<10} plyta={have[name]}')
```

Puste wyjście = płyta zgodna z airframe'em. Każda linia = `param set` na płycie. Dodatkowo sprawdzić
w logu: `CA_METHOD`, `MC_*`, `IFO_*` oraz brak niechcianych wartości dla parametrów **nieobecnych** w
airframe (np. `RC_CRSF_PRT_CFG` ≠ 0, `CA_SV*_SLEW` ≠ 0, `FD_FAIL_R/P`).

## Ocena zmian z 08.09 (clipping)

* `IFO_STB_VD_TAU 2.0` – **sensowne.** Matematyka w komentarzu się zgadza: przy τ = 10 s bias
  0.26 m/s² nasyca kanał boczny, przy 2 s potrzeba 1.31 m/s². Dodane też do SITL.
* Ogranicznik `_vel_damp_body` do `IFO_VEL_MAX_XY` w `ifo_pos_control` – **kierunek dobry, wartość za
  duża** ([Problemy otwarte](#problemy-otwarte), pkt 2).
* `IMU_GYRO_DNF_EN 2` – **nie działało i nie celowało w problem.** Bit 1 = dynamiczny notch z FFT,
  który wymaga `IMU_GYRO_FFT_EN 1` (inaczej `gyro_fft` nie startuje); nigdzie nie był ustawiony. Poza
  tym to filtr **żyroskopu**, a łańcuch z 08.09 idzie przez **akcelerometr** (`vehicle_acceleration` →
  `_vel_damp_body`). Linia usunięta. Jeśli chcesz notch FFT dla pętli rate, dodaj oba:
  `IMU_GYRO_FFT_EN 1` + `IMU_GYRO_DNF_EN 2` i sprawdź `cpuload` (już ~54 %).
* Clipping akcelerometru (5139 zdarzeń przy rozpędzaniu EDF-ów) to problem mechaniczny – izolacja
  drgań FC / wyważenie EDF-ów; parametrami się tego nie naprawi.
* Niezacommitowane `param set-default IFO_TILT_HOVER 0.0` pod `IFO_STB_VD_TAU` – **duplikat** linii z
  sekcji serw (ta sama wartość, więc bez skutku), łamiący regułę zapisaną kilka linii niżej („nie
  ustawiać drugi raz”). Usunięty. Jeśli celem było naprawienie −45 na płycie: `set-default` tego nie
  zrobi – `param set IFO_TILT_HOVER 0` + `param save`.
