# Masterarbeit: Entwicklung eines automatisierten Systems mit einem mobilen Manipulator zur Sortierung von Kunststoffabfall

## Beschreibung
Diese Masterarbeit beschäftigt sich mit der Entwicklung eines automatisierten Systems, das einen statischen Roboterarm (Universal Robot UR5e) in Kombination mit einer Robotiq EPick Vakuumpumpe, einem Softgreifer von Piab und einer Intel RealSense 415D Kamera verwendet. Das System läuft unter ROS Noetic und Ubuntu 20.04 und ist in der Lage, die Farben Grün, Gelb und Rot zu erkennen, um Legosteine oder andere Objekte nach Farben zu sortieren und an einer bestimmten Stelle abzulegen.

## Voraussetzungen
- ROS Noetic
- Ubuntu 20.04
- Universal Robot UR5e
- Robotiq EPick Vakuumpumpe
- Piab Softgreifer
- Intel RealSense 415D Kamera

## Installation und Setup
Stellen Sie sicher, dass alle Hardwarekomponenten korrekt installiert und konfiguriert sind, und dass ROS Noetic auf Ubuntu 20.04 läuft.

Folgen Sie diesen Schritten für die Einrichtung:
- Feste IP-Adresse am Laptop und am UR5e einstellen. Siehe Übersicht.
- Laptop per LAN mit dem UR5e verbinden.
- Kamera per USB-Kabel mit dem Laptop verbinden.
- Rs485 URCap installieren. Funktioniert nicht gleichzeitig mit dem ROBOTIQ URCap.
  - [Setup Tool Communication](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md)
- URCap external control installiert?
  - [Install URCap E-Series](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md)

## Verwendung
Bevor Sie die Befehle ausführen, beachten Sie bitte folgende Schritte:

- Externalcontrol URCap am Bedienpanel des UR5e starten.
- 5 Terminals öffnen und die Befehle wie aufgeführt in dieser Reihenfolge ablaufen lassen.
- Das Programm startet und sortiert alle Legosteine nach den Farben.
- Bei einem neuen Versuch müssen nur die letzten zwei Terminals neu gestartet werden.

Führen Sie dann die folgenden Befehle in der angegebenen Reihenfolge aus, um das Programm zu starten:

1. Hauptprogramm mit allen Treibern starten:
roslaunch sort_programm master.launch
2. Node für Vakuumpumpe starten:
rosrun robotiq_epick_vaccum_gripper_control robotiq_epick_vaccum_gripper_control_node _port_name:=/tmp/ttyUR _baudrate:=115200
3. Actionserver für Vakuumpumpe starten:
rosrun robotiq_epick_vaccum_gripper_control gripper_action_server.py
4. Programm zur Erkennung von Farbe und Koordinaten der Legosteine:
rosrun color_detection color_detector_legobrick.py
5. Bewegungsablauf für Roboterarm und Greifer:
rosrun sort_programm ur5e_moveit_script_erweitert.py


Objekte können bis zu einem Meter Entfernung und 30 cm Abstand von der Kamera erkannt und sortiert werden. Die Sortierreihenfolge beginnt mit dem Objekt, das der Kamera am nächsten ist.

## Anpassungen
- **Ablagepositionen ändern:**
Im Ordner `sort_programm`, im Programm `ur5e_moveit_script_erweitert.py`, können Sie die Ablagepositionen wie folgt ändern:
```python
if color == "yellow":
   pose_drop_off.position.x = 0.3
   pose_drop_off.position.y = 0.678
   pose_drop_off.position.z = 0.1
elif color == "red":
   pose_drop_off.position.x = 0.3
   pose_drop_off.position.y = 0.678 - 0.12
   pose_drop_off.position.z = 0.1
elif color == "green":
   pose_drop_off.position.x = 0.3
   pose_drop_off.position.y = 0.678 - 0.24
   pose_drop_off.position.z = 0.1
```

- **Kameraposition anpassen:**
Um die Position der Kamera anzupassen, bearbeiten Sie die Datei `camera_transform.launch` im Ordner `my_camera_config`. Dort können Sie die x, y, z Koordinaten direkt im XML-Tag für die Kameraposition ändern. Beispiel:
```xml
<arg name="x" default="1.0"/>
<arg name="y" default="0.0"/>
<arg name="z" default="1.5"/>
```
- **Farberkennung anpassen/erweitern:**
Die Farberkennung kann im Ordner color_detection durch Ausführung des Programms color_detector_Trackbar.py angepasst oder erweitert werden. Dies ermöglicht Ihnen, den HSV-Farbbereich über Schieberegler in einer Live-Ansicht zu justieren. Die festgelegten Werte sollten anschließend in color_functions.py übernommen werden, um die Erkennung der Farben entsprechend zu modifizieren.

### Kontakt

Für Fragen oder Anmerkungen zu diesem Projekt kontaktieren Sie mich bitte unter tobias.kurka@studmail.htw-aalen.de.
