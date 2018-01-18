Creation and upload of own robot models
****************************************

TODO: Translate this text to english!

Im File-Browser findet man unter dem Pfad ``/home/xamla/Rosvita.Control/library/robot_parts/`` die verschiedenen Robotermodelle (z.B. ``universal_robots/part_UR5``). Diese beinhalten folgende Dateien und Ordner:

* Ordner "collision" (enthält die .stl-Dateien für das Kollisionsmodell)
* Ordner "visual" (entält die .dae-Dateien für die Visualisierung)
* .xacro-Datei mit der Beschreibung des Robotermodells
* Datei "part.json" mit der MoveIt!-Konfiguration für das Robotermodell
* Dateien "CMakeLists.txt" und "package.xml", die für das Kompilieren in einem "Catkin-Workspace" unter ROS benötigt werden.

Zum Hochladen eines eigenen Robotermodells muss man zunächst all diese Dateien, die zu einem Robotermodell gehören, bereitstellen. Evtl. werden entsprechende Dateien bereits vom Roboter-Hersteller zur Verfügung gestellt (wie z.B. von [Universal Robots unter GitHub](https://github.com/ros-industrial/universal_robot)). In diesem Fall müssen die entsprechenden Ordner und Dateien nur noch angepasst werden, so dass sie zur ROSVITA Ordner-Struktur passen. Falls man selbst ein Robotermodell erstellen will, helfen die folgenden [URDF/Xacro-Tutorials](http://wiki.ros.org/urdf/Tutorials) zur Konstruktion eines eigenen Robotermodells.

*Das "Unified Robot Description Format" (URDF) ist eine XML-Spezifikation zur Beschreibung eines Roboters. Die Verwendung dieser Spezifikation setzt voraus, dass der Roboter sich in Form einer Baumstruktur beschreiben lässt und dass der Roboter aus starren Verbindungen besteht, die durch Gelenke verbunden sind. Die Spezifikation umfasst die kinematische und dynamische Beschreibung des Roboters, die visuelle Darstellung des Roboters und das Kollisionsmodell des Roboters.
"Xacro" wiederum ist eine XML-Macro-Sprache. Mit "Xacro" lassen sich kürzere und besser lesbare XML-Dateien bauen, indem Macros verwendet werden, die größere XML-Ausdrücke zu Kompakteren bündeln.*

Um einen Ordner mit eigenem Robotermodell auf den ROSVITA Server hochzuladen, verpackt und komprimiert man diesen am besten zunächst: ``tar cvfz my_robot.tgz my_robot``. 
Danach geht man mit dem ROSVITA Datei-Browser in das Verzeichnis ``/home/xamla/Rosvita.Control/library/robot_parts/`` und drückt den "Upload"-Knopf, woraufhin man den soeben verpackten Ordner "``my_robot.tgz``" 
hochladen kann. Jetzt muss der Ordner nur noch wieder entpackt werden. Dazu geht man mit dem ROSVITA Terminal in den übergeordneten Ordner ``cd /home/xamla/Rosvita.Control/library/robot_parts/`` 
und gibt anschließend den folgenden Befehl zum Entpacken ein: ``tar -xzvf my_robot.tgz``.

**Wichtig:** Bei der Erstellung eigener Robotermodelle ist darauf zu achten, dass diese Modelle nicht zu groß werden (nicht mehrere GigaByte große CAD-Dateien verwenden), da das Arbeiten (insbesondere die 3D-Visualisierung) mit solch riesigen Modellen sonst naturgemäß sehr träge wird.

Unter ``/home/xamla/git/`` findet man alle möglichen GitHub-Repositories zu Robotermodellen und Treibern. Dort kann man auch eigene GitHub-Repositories zu eigenen Robotermodellen hochladen. Unter ``/home/xamla/catkin_ws/src/`` sind diese Repositories verlinkt. Mit dem ROSVITA Terminal lassen sich dort Links zu weiteren Repositories erzeugen (z.B. ``ln -s /home/xamla/git/my_robot /home/xamla/catkin_ws/src/``). Um den Catkin Workspace ``/home/xamla/catin_ws/`` zu kompilieren, 
einfach mit dem Terminal in den Catkin Workspace gehen ``cd /home/xamla/catkin_ws/`` und den Befehl ``catkin_make`` eingeben.

**Wichtig:** Alle eigens in die Library hochgeladenen Robotermodelle gehen nach Beendigung von ROSVITA verloren (-> keine dauerhafte Speicherung). Deshalb ist es ratsam, die selbst erstellten Robotermodelle auch in den eigenen Projektordner (unter ``/home/xamla/Rosvita.Control/projects/<project name>/robotModel/``) hochzuladen. Änderungen im eigenen Projektordner werden auch lokal in dem entsprechenden Projektordner (unter ``/home/<username>/rosvita/projects/<project name>/robotModel/``) gespeichert und bleiben somit auch nach Beendigung von ROSVITA erhalten.

TODO:
* Kommentar am Ende jeder .xacro-Datei zum Einzel-Editieren der Dateien


### Versetzen des "Tool Center Points (TCP)":

Zum Versetzen des "Tool Center Points" und damit automatisch auch des interaktiven Markers im 3D-View, muss ein "tcp_link" in die Datei "robotModel/main.xacro" des entsprechenden eigenen Projektordners hinzufüg werden. <br />
Beispiel: <br />
```
<link name="tcp_link" />
<joint name="tcp_joint_F" type="fixed">
  <child link="tcp_link" />
  <parent link="wrist_3_link" />
  <origin xyz="-0.0037 -0.0072 0.2092" rpy="0 0 0" />      
</joint>
```
("origin" und "rpy" sind die Position und Rotation des TCP relativ zum "parent link" (hier: "wrist_3_link")).
Danach muss die Datei "main.xacro" durch Drücken des entsprechenden Feldes neu kompiliert werden.
Daraufhin erscheint der neue "tcp_link" im "3D View" rechts neben der geöffneten .xacro-Datei.
Bei Wechsel in den "Configuration View" muss die neue Konfiguration erst kompiliert werden, damit der neue Link angezeigt und auswählbar wird (z.B. als "Tip Link" einer "Move Group"), denn erst dann wird die entsprechende URDF-Datei neu generiert.
Wenn man jetzt im "Configuration View" (unter "MotionPlanning"->"MoveIt! Motion Planning Framework (ROS)"->"groups"->"*<group_name>*" den neuen "tcp_link" als "Tip link" auswählt, kompiliert, ROS startet und in den "World View" wechselt, so erscheint auch der interaktive Marker am neuen "tcp_link".

