# Symulacja quadcoptera — instrukcja uruchomienia

Projekt symuluje drona o masie 1,5 kg w ROS 2 Humble i Gazebo Classic 11. Dostępne są trzy regulatory: PID, LQR i nieliniowy MPC (NMPC). Dron automatycznie wykonuje trasę z pliku YAML, a RViz pokazuje trasę zadaną i rzeczywistą.

## 1. Wymagania

Instrukcja dotyczy Ubuntu 22.04 i powłoki Bash. Potrzebujesz zainstalowanego ROS 2 Humble, Gazebo Classic 11, integracji `gazebo_ros`, RViz oraz narzędzia `colcon`. Projekt korzysta z Gazebo Classic, więc wymaga pakietów `gazebo_ros` i `gazebo_dev`.

Jeżeli ROS 2 Humble jest już zainstalowany i repozytorium pakietów ROS jest skonfigurowane, doinstaluj narzędzia i zależności:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-rviz2 ros-humble-rclpy
```

W dalszych poleceniach przyjęto lokalizację projektu `/home/marton/Repos/drone_ws`. Jeśli projekt znajduje się gdzie indziej, zmień ścieżkę w poleceniu `cd`.

## 2. Pierwsze zbudowanie projektu

Otwórz terminal w katalogu projektu:

```bash
cd /home/marton/Repos/drone_ws
source /opt/ros/humble/setup.bash
```

Jeśli `rosdep` nie był wcześniej inicjalizowany na tym komputerze, wykonaj jednorazowo:

```bash
sudo rosdep init
```

Następnie zainstaluj zależności zadeklarowane przez pakiet i zbuduj projekt:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select quadcopter_sim
source install/setup.bash
```

Po poprawnym zbudowaniu pakietu można uruchomić symulację.

## 3. Uruchomienie z regulatorem MPC

```bash
ros2 launch quadcopter_sim simulation.launch.py controller:=mpc
```

Polecenie uruchamia Gazebo, dodaje model drona, uruchamia generator trasy, regulator MPC, ocenę sterowania oraz RViz. Nie trzeba osobno uruchamiać silników ani wysyłać komendy startu.

Domyślna misja trwa 50 sekund czasu symulacji, z opóźnieniem startowym 2 sekundy. Obejmuje start, przelot po zadanej trasie i zejście do wysokości 0,15 m. Po zakończeniu misji nadal publikowany jest ostatni punkt zadany; symulacja nie zamyka się automatycznie.

Aby zakończyć pracę, naciśnij **Ctrl+C** w terminalu, w którym uruchomiono symulację. Przed uruchomieniem kolejnego regulatora zakończ poprzednią symulację.

### Kolejne uruchomienia

W każdym nowym terminalu wczytaj środowisko ROS i projektu:

```bash
cd /home/marton/Repos/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quadcopter_sim simulation.launch.py controller:=mpc
```

Ponowne budowanie jest potrzebne po zmianach kodu C++.

## 4. Wybór regulatora i okien

Uruchom tylko jeden z poniższych wariantów:

```bash
# PID — regulator domyślny
ros2 launch quadcopter_sim simulation.launch.py controller:=pid

# LQR
ros2 launch quadcopter_sim simulation.launch.py controller:=lqr

# Nieliniowy MPC
ros2 launch quadcopter_sim simulation.launch.py controller:=mpc
```

Uruchomienie bez RViz:

```bash
ros2 launch quadcopter_sim simulation.launch.py controller:=mpc rviz:=false
```

Uruchomienie bez okien Gazebo i RViz, z działającą symulacją w tle terminala:

```bash
ros2 launch quadcopter_sim simulation.launch.py controller:=mpc gui:=false rviz:=false
```

## 5. Scenariusz z przeszkodami

Domyślny scenariusz ładuje świat `empty.world` i trasę `mission.yaml`. Aby załadować świat z przeszkodami oraz dopasowaną do niego misję:

```bash
ros2 launch quadcopter_sim simulation.launch.py \
  controller:=mpc scenario:=obstacle_course
```

Scenariusz korzysta z `worlds/obstacle_course.world` i `config/mission_obstacle_course.yaml`. Przeszkody są częścią świata symulacji; model predykcji MPC nie zawiera ograniczeń unikania kolizji. Przebieg lotu określa przygotowana trasa.

## 6. Zmiana trasy

Domyślna trasa znajduje się w [mission.yaml](src/quadcopter_sim/config/mission.yaml). Punkty mają następujący format:

```yaml
waypoints:
  - {t: 0.0,  x: 0.0, y: 0.0, z: 0.15, yaw: 0.0}
  - {t: 5.0,  x: 0.0, y: 0.0, z: 2.00, yaw: 0.0}
  - {t: 10.0, x: 2.0, y: 0.0, z: 2.00, yaw: 0.0}
  - {t: 15.0, x: 2.0, y: 0.0, z: 0.15, yaw: 0.0}
```

- `t` — czas od początku misji w sekundach; stosuj rosnące wartości, zaczynając od zera.
- `x`, `y`, `z` — pozycja w metrach, w układzie świata.
- `yaw` — obrót wokół osi pionowej w radianach.

Misja musi zawierać co najmniej dwa punkty. Generator płynnie interpoluje pozycję między punktami. Przy budowaniu z `--symlink-install` zmiana istniejącego pliku YAML jest odczytywana przy następnym uruchomieniu.

Możesz też wskazać własną misję, podając pełną ścieżkę do pliku:

```bash
ros2 launch quadcopter_sim simulation.launch.py \
  controller:=mpc \
  mission:=/home/marton/Repos/drone_ws/src/quadcopter_sim/config/mission.yaml
```

Argument `world` pozwala analogicznie wskazać własny plik `.world`. Jawne argumenty `mission` i `world` zastępują odpowiednie pliki wybrane przez `scenario`.

## 7. Raport i podgląd działania

Ocena sterowania jest domyślnie włączona. Po upływie misji raport pojawia się w logach i w pliku:

```bash
cat /tmp/quadcopter_control_report.txt
```

Poczekaj na komunikat `Raport zapisany w:` — przed jego pojawieniem się plik może zawierać wynik poprzedniego uruchomienia. Czas symulacji może płynąć wolniej niż czas rzeczywisty.

Własną ścieżkę raportu można ustawić przy starcie:

```bash
ros2 launch quadcopter_sim simulation.launch.py \
  controller:=mpc report_file:=/tmp/raport_mpc.txt
```

Raport ocenia m.in. odległość od geometrycznej trasy i czas w dopuszczalnym korytarzu. Odległość od trasy nie jest tym samym co błąd względem punktu zadanego w tej samej chwili. Domyślna tolerancja wynosi 0,25 m (`position_tolerance:=0.25`). Ocenę można wyłączyć argumentem `evaluator:=false`.

W drugim terminalu, po wczytaniu obu plików `setup.bash`, można sprawdzić komunikację:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /quadcopter/odom --once
ros2 topic hz /quadcopter/odom
```

Najważniejsze tematy:

| Temat | Zawartość |
| --- | --- |
| `/quadcopter/command/pose` | Zadana pozycja i orientacja |
| `/quadcopter/odom` | Aktualna pozycja, orientacja i prędkości |
| `/quadcopter/command/rotor_speed` | Zadane prędkości czterech wirników |
| `/quadcopter/rotor_speed` | Rzeczywiste prędkości wirników |
| `/quadcopter/path/reference` | Trasa zadana |
| `/quadcopter/path/actual` | Trasa rzeczywista |

## 8. Konfiguracja regulatorów

| Regulator | Plik parametrów |
| --- | --- |
| PID | [controller.yaml](src/quadcopter_sim/config/controller.yaml) |
| LQR | [lqr_controller.yaml](src/quadcopter_sim/config/lqr_controller.yaml) |
| MPC | [mpc_controller.yaml](src/quadcopter_sim/config/mpc_controller.yaml) |

MPC domyślnie przewiduje 35 kroków po 0,04 s, czyli 1,4 s ruchu. Publikuje bezpośrednio komendy prędkości wirników. Launch automatycznie wyłącza dla niego `motor_allocator`.

Opis modelu, funkcji kosztu i testów znajduje się w [dokumentacji nieliniowego MPC](src/quadcopter_sim/docs/nonlinear_mpc.md).

## 9. Typowe problemy

- **`ros2: command not found`** — wczytaj `source /opt/ros/humble/setup.bash` i upewnij się, że ROS 2 Humble jest zainstalowany.
- **`Package 'quadcopter_sim' not found`** — zbuduj pakiet i w tym samym terminalu wykonaj `source install/setup.bash` z katalogu projektu.
- **Brak `gazebo_ros` lub `gazebo_dev` podczas budowania** — wykonaj instalację zależności z punktów 1–2.
- **Problemy z oknami graficznymi** — uruchom wariant `gui:=false rviz:=false`, aby sprawdzić działanie samej symulacji.
- **Dron nie porusza się** — sprawdź, czy Gazebo nie jest wstrzymane, i czy pojawiają się wiadomości `/quadcopter/odom` oraz `/quadcopter/command/pose`.
- **Ostrzeżenie MPC o przekroczeniu 40 ms** — obliczenia regulatora przekroczyły jego okres. Zamknij zbędne aplikacje lub uruchom symulację bez okien; solver nie gwarantuje zakończenia w 40 ms.

Listę argumentów uruchomienia można wyświetlić bez uruchamiania symulacji:

```bash
ros2 launch quadcopter_sim simulation.launch.py --show-args
```
