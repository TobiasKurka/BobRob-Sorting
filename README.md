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

## Verwendung
Führen Sie die folgenden Befehle in der angegebenen Reihenfolge aus, um das Programm zu starten:

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

Kameraposition ändern:
Im Ordner my_camera_config, im Programm camera_transform.launch, können Sie die Position der Kamera in x, y, z Koordinaten anpassen.
Farberkennung anpassen:
Im Ordner color_detection können Sie das Programm color_detector_Trackbar.py ausführen, um mit Schiebereglern in einer Live-Ansicht den HSV-Farbbereich festzulegen und diesen dann im Programm color_functions.py einzutragen.

Kontakt

Für Fragen oder Anmerkungen zu diesem Projekt kontaktieren Sie mich bitte unter tobias.kurka@studmail.htw-aalen.de.
