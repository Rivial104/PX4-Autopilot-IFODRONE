## 🔗 Wizualizacja wiadomości uORB w czasie rzeczywistym za pomocą PlotJuggler'a

## Wstęp
Do wizualizacji wiadomości uORB oprócz PlotJugglera będzie potrzebne środowisko ROS 2 oraz middleware uXRCE-DDS

## Instalacja niezbędnych narzędzi

#### ROS 2 - "Humble" na Ubuntu 22.04:
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
Jest to forma streszczona ze źródła: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Uwaga! Jeżeli nie chcemy
echo "source /opt/ros/humble/setup.bash" >> .bashrc``` to przed późniejszymi wystąpieniami ```colcon build``` oraz uruchamianiem plotjugglera z ROS koniecznej jest ustawienie tych zmiennych za każdym razem

#### Instalacja zależności Python'a
pip install --user -U empy==3.3.4 pyros-genmsg setuptools

#### Instalacja Micro XRCE-DDS
Instalacja ze źródeł w dowolnym katalogu, najlepiej tam gdzie trzymane są repozytoria
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

#### Instalacja pluginu PlotJugglera do ROS
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros

## Przygotowanie środowiska roboczego

1. Uruchomić terminal i przejść do katalogu roboczego
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/

2. Pobrać repozytoria związane z wiadomościami uORB w PX4 w wybranym wyżej katalogu
git clone https://github.com/PX4/px4_msgs.git

3. Wykonać kompilację środowiska roboczego
cd ..
colcon build

## Uruchomienie

1. Po kompilacji środowiska roboczego uruchomić PlotJuggler w środowisku ROS
source install/setup.bash
ros2 run plotjuggler plotjuggler

2. Przed zbudowaniem autopilota włączyć klienta uXRCE w konfiguracji budowania np. boards/px4/sitl/gladius.px4board
CONFIG_MODULES_UXRCE_DDS_CLIENT=y

3. Wykonać kompilacje i uruchomić autopilota
make px4_sitl_gladius mavlink
Teraz w autopilocie uruchomiony jest klient uXRCE

4. Uruchomienie Agenta uXRCE - w nowym terminalu wykonać polecenie
MicroXRCEAgent udp4 -p 8888; exec bash

5. Weryfikacja poprawności działania

W konsoli Agenta uXRCE powinniśmy widzieć
...
[1675929445.268957] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001, publisher_id: 0x0DA(3), participant_id: 0x001(1)
[1675929445.269521] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001, datawriter_id: 0x0DA(5), publisher_id: 0x0DA(3)
[1675929445.270412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x0DF(2), participant_id: 0x001(1)
...
Natomiast w konsoli PX4:
...
INFO  [uxrce_dds_client] synchronized with time offset 1675929429203524us
INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 83
INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 188
...

6. Ostatnim etapem jest otworzenie uruchomionego PlotJugglera i wybór _ROS2 Topic Subscriber_ w panelu _Streaming_

## Dodanie wiadomości spoza oficjalnego repozytorium PX4

W celu wizualizacji wiadomości nie będących w oficjalnym repo PX4-Autopilot należy je skopiować do katalogu ```~/ws_sensor_combined/src/px4_msgs/msg/

Następnie usunąć folder build i ponownie zbudować środowisko
cd ~/ws_sensor_combined/
rm -r build
colcon build

W repozytorium PX4_autopilot należy dodać wiadomości uORB, które mają być uwzględnione przez środowisko ROS. W tym dodać do ```dds_topics.yaml``` interesujące nas wiadomości. Oznaczenie ```fmu/in``` oznacza wiadomości które mogą zostać wysłane z zewnątrz do PX4 natomiast ```fmu/out``` oznaczają wiadomości publikowane przez PX4

## Referencje

https://docs.px4.io/main/en/debug/plotting_realtime_uorb_data
https://docs.px4.io/main/en/ros2/user_guide#build-ros-2-workspace
https://github.com/PlotJuggler/plotjuggler-ros-plugins
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
