# IFODRONE Changelog and Documentation file

Plik który zbiera informacje o zmianach `CHANGELOG` oraz służy jako dokumentacja oprogramowania dla platformy IFODRONE.

## CHANGELOG

Added:
- Initial commit, dodano IFODRONE.md [08.12.2025]
- Dodano gałąź ifo-model oraz odpowiedni fork submodułu gz [08.12.2025]
- Dodano gałąź ifo-airframe [08.12.2025]
- Pionowy booster 4× EDF (jeden sygnał PWM S7): alokator `IFO_BST_ROTOR`/`IFO_BST_RATIO`, tryb deweloperski `IFO_DEVEL`, model gz (4 linki `booster_0..3`, `SIM_GZ_EC_*7`), parametry 49000/4072 (`CA_ROTOR6_*`, `PWM_MAIN_FUNC7 107`); real 4072 w trybie `IFO_DEVEL 1` do kalibracji, opis w `IFODRONE-params.md` §15 [11.11.2026]

Changed:
- SITL: skala jednostek Fz s = 0.72 (`CA_ROTOR*_CT` ×s, coax KM ÷s, `IFO_THR_*` ×s) – z boosterem hover > `IFO_THR_MAX`, dron nie startował [11.11.2026]

Fixed:
- Booster: wyciek yaw → Fz (średnia przyciętych komend coax ×g dawała ~2 g wznoszenia przy pełnym yaw); para coax odsaturowana z zachowaniem średniej [11.11.2026]


## Dokumentacja

### GIT Workflow

Struktura projektu przedstawia się w następujący sposób:
1. Główną gałęzią do rozwijania aktualnej, działającej wersji kodu jest `ifodrone`.
2. Chcąc dodać nową funkcjonalność wychodzimy z nowym branchem z linii `ifodrone`.
3. Gdy uznamy, że funkcja jest gotowa to pytamy szanownego kolegi czy jest okej i czy akceptuje.
4. Po Approvalu mergujemy zmiany do gałęzi `ifodrone` (po drodze rozwiazując konfliky jeśli takie nastąpią).
5. Jest porządek i sigma.

### Zmiany w submodułach (to do Ciebie Mikololus szczególnie)

W PX4 pracujemy często z submodułami, to znaczy zagnieżdżonymi repozytoriami. Przykładem jest repozytorium `gazebo`, którym dodaje się swoje modele do symulacji. Stąd należało dodać i również forka dla tego repozytorium, żeby móc dodawać zmiany. Poniżej znajduje się instrukcja, jak poprawnie pracować z submodułem `Tools/simulation/gz`, wprowadzać zmiany, commitować je i wysyłać do własnego forka.

#### Aktualizacja submodułów na początku pracy
Po pobraniu lub przełączeniu się na gałąź w głównym repozytorium PX4 zawsze wykonuj:
```
git submodule update
```
To powinno zaktualizować zmiany (jeśli robiłbym coś w innych submodułach).

### Przejście do submodułu
Wejdź do katalogu `Tools/simulation/gz` i sprawdź na jakim jesteś commicie i czy nie jesteś w detached HEAD:
```
git status
```
Koniecznie przełącz się na swój branch - stworzyłem `ifo-model`.
```
git checkout ifo-model
```
Jeżeli gałąź nie istnieje lokalnie, ale jest w Twoim forku, pobierz ją:
```
git fetch ifo-fork
```
^ TO POWINNO WYSTARCZYĆ, KOMENDĄ PONIŻEJ ROBISZ NOWY BRANCH CO NIE POWINNO MIEĆ RACZEJ MIEJSCA PRZY POPRAWNIE ZPULLOWANYM SUBMODULE I ZROBIEBNIU UPDATE'A
```
git checkout -b ifo-model ifo-fork/ifo-model
```

#### Wprowadzanie zmian w submodule
Zmieniasz pliki jak zwykle — np. dodajesz modele. Następnie dodajesz i commitujesz.

#### Wypchnięcie zmian do forka submodułu
Wysyłasz swoją gałąź do forka:
```
git push ifo-fork ifo-model
```

### Powrót do PX4 repo i sprawdzenie zmian
Wróć do głównego repo i sprawdź status. Powinieneś widzieć:
```
modified: Tools/simulation/gz (new commit)
```
Dodaj swoją aktualizacje:
```
git add Tools/simulation/gz
```
i zatwierdź ją:
```
git commit -m "Dodano kolejną cześć modelu IFO"
```
Wypchnij zmiany w głównym repo PX4:
```
git push origin ifo-model
```

Można przejsć do testowania modelu w SITL.
