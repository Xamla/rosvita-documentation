Building a complex scene
*************************

TODO: Translate this text to english!

**Hinweis:** Zum besseren Verständnis sollte man zunächst das Kapitel ["Erzeugen einer einfachen Roboterkonfiguration"](#tag3) gelesen haben. <br />

Wenn man mehrere Roboterteile ("RobotParts") samt zugehöriger Aktuatoren in eine Szene einfügt (z.B. einen "Yaskawa Motoman SDA10D" und die verschiedenen Teile des "Weiss Robotics WSG-50 Greifers"), so müssen diese korrekt miteinander verknüpft werden. Dies geschieht, indem man im "Configuration"-View rechts in der Liste der verwendeten "RobotParts" das entsprechende Teil anklickt (z.B. den "wsg50 mount") und in der sich öffnenden "Properties"-Leiste den "Parent link" für die Verknüpfung auswählt (hier z.B. den "arm_right_link_tool0 (Yaskawa motoman SDA10D)"). Insgesamt benötigt man folgende Roboterteile und Verknüpfungen, um den Weiss Robotics WSG-50 Greifer mit dem rechten Arm des Motoman SDA10D zu verbinden:
* Yaskawa motoman SDA10D; ``Parent link: world``
* wsg50 mount; ``Parent link: arm_right_link_tool0 (Yaskawa motoman SDA10D)``
* Weiss Robotics WSG-50; ``Parent link: mount_tool0 (wsg50 mount)``
* Weiss Robotics WSG-50 w1 right finger; ``Parent link: gripper_right_tool0 (Weiss Robotics WSG-50)``
* Weiss Robotics WSG-50 w1 left finger; ``Parent link: gripper_left_tool0 (Weiss Robotics WSG-50)``

Zusätzlich benötigt man die folgenden Aktuatoren:
* Motoman SDA10D (Yaskawa)
* Gripper WSG-50 (Weiss Robotics)

Nach drücken der "Compile"-Taste sollte der Roboter mit Gripper in der 3D-Ansicht des "Configuration"-View erscheinen.

**Wichtig:** Will man mehrere **gleiche Roboterteile** in der Szene verwenden (z.B. zwei WSG-50 Greifer), so müssen diese per "**Prefix**" unterschieden werden. Dazu einfach das "Prefix"-Feld in der "Properties"-Liste zu dem entsprechenden Roboterteil verwenden.

**Wichtig:** Wenn man Änderungen in den .xacro-Dateien eines eigenen Projektordners (unter ``/home/xamla/Rosvita.Control/projects``) durchführt (z.B. weil man die Koordinatenachse für den "joint_tool0" eines Roboterarms drehen will), so werden diese Änderungen lokal im eigenen Projektordner gespeichert. Änderungen außerhalb des eigenen Projektordners, also z.B. an einem RobotPart in der Library (``/home/xamla/Rosvita.Control/library``) werden jedoch **nicht gespeichert** und gehen nach Beendigung von ROSVITA wieder verloren.

Zusätzlich zum WSG-50 Gripper am rechten Arm des SDA10D, lässt sich jetzt am linken Arm des Roboters ein weiterer WSG-50 Gripper anbinden. Außerdem kann noch die SDA10D-Zelle hinzugefügt werden, sowie im "World"-View einige Kollisionsobjekte platziert werden.


... wird fortgesetzt

