The graph concept
******************

TODO: Translate this text to english!

Durch Klick auf das Feld "+ New" in der oberen Leiste wird ein neuer Graph angelegt. Es öffnet sich die Graphen-Ansicht und der neue Graph erscheint oben links in der Liste der "Runtime Slots" zusammen mit seiner Prozess-ID ("PID"). Rechts in der Leiste lassen sich nun unter "Module Catalog" einzelne Module auswählen und per Drag&Drop in den Graphen-Editor ziehen, der zu Begin nur ein "Start/Input"-Modul und ein "End/Output"-Modul besitzt. Wenn man mit der linken Maustaste in den Graphen-Editor klickt, erscheint zudem ein Textfeld, mit dem man gezielt nach Modulen suchen kann. Durch Klick mit der rechten Maustaste in den Graphen-Editor erhält man eine Auswahl, in der man zwischen dem Einfügen eines Kommentars, dem Hinzufügen eines weiteren Moduls, dem Löschen und dem Kompilieren des aktuellen Graphen auswählen kann. Ist ein bestimmtes Modul angeklickt, so erscheint unten in der rechten Leiste unter "Help" eine kurze Beschreibung zu dem angeklickten Modul. Bei Klick auf einen der Ein/Ausgänge (bzw. Parameter) des Moduls, erhält man dort eine Beschreibung zu dem angeklickten Pin. Per Rechtsklick auf ein Modul lässt sich dieses duplizieren oder löschen. Durch Anklicken eines Modul-Pins lassen sich zudem Verbindungen zwischen Module, bzw. Modulparametern ziehen. Dazu nach Klick auf den entsprechenden Pin die Maustaste gedrückt halten, zu einem anderen Modul-Pin ziehen und dort loslassen. Durch Rechtsklick auf eine Verbindung und Auswahl von "Disconnect" lassen sich Verbindungen zwischen Modulen bzw. deren Parametern wieder lösen.

**Wichtig:** Die dreieckigen, grünen Pins an den Modulen, bezeichnen sogenannte "Flow-Connections", d.h. zwischen diesen Pins kann immer nur genau eine Verbindung gezogen werden. An einem dreieckigen Pin können also nicht mehrere Verbindungen anliegen. Die grauen, runden Pins an den Modulen dagegen sind "Parallel-Connections", d.h. ein solcher Pin kann mit mehreren anderen Pins Verbunden sein.

Viele Modulparameterwerte lassen sich entweder von Hand in das entsprechende Textfeld eintragen, oder durch eine Verbindung zu einem anderen Modul, das diese Werte liefert, setzen. Hat man den Graphen von "Start" bis "Ende" fertiggestellt, so lässt er sich über die Felder "Save as..." bzw. "Save" in der oberen Leiste abspeichern und durch Klick auf das Feld "Start" bzw. "Stop" ausführen bzw. anhalten. Beinhaltet die Ausführung des Graphen eine Roboterbewegung (wie im nachfolgenden Beispiel), so lässt sich diese im "World View" (z.B. in einem weiteren Browser-Fenster) beobachten. 

### <a name="tag9.1"></a> Beispiel-Graph:

Im Folgenden wollen wir einen einfachen Beispielgraphen bauen, der das Graphenkonzept noch einmal anschaulich verdeutlicht. Dieser Graph soll bei Ausführung bewirken, dass der Roboter (hier ein UR5 Arm) in eine bestimmte Gelenkwinkelstellung (die nicht die aktuelle Gelenkwinkelstellung ist) fährt. Wir setzen voraus, dass im "World View" bereits einige Gelenkwinkelstellungen abgespeichert wurden, von denen nun eine angefahren werden soll.

#### Modul-Auswahl:

Für die Erstellung des Beispielgraphen benötigen wir folgenden Module: 
* "MoveJ" (im "Module Catalog" unter "Xamla"->"Robotics"->"JointPath", oder direkt über die Modul-Suche)
* "CreatePlanParameters" (im "Module Catalog" unter "Xamla"->"Robotics"->"JointPath")
* "GetJointValuesById" (im "Module Catalog" unter "Rosvita->WorldView")
* optional: "PropertyAccessor" (2x) (im "Module Catalog" unter "Xamla")
* optional: "ViewString" (3x) (im "Module Catalog" unter "Xamla"->"Graph"->"Controls")

Mit dem Modul "MoveJ" lässt sich der Roboter in eine bestimmte Gelenkwinkelstellung fahren. Dazu werden die Planungsparameter ("moveGroupName", "joints", "maxVelocity", ...) und die Zielwerte für die Gelenkwinkel benötigt. Die aktuellen Planungsparameter erhält man mit dem Modul "CreatePlanParameters". In diesem Modul muss nur der Namen der zu bewegenden MoveGroup (z.B. "urcontroller") eingetragen werden. Für alle weiteren Planungsparameter werden automatisch die Default-Werte geladen. Zielwerte für die Gelenkwinkel lassen sich mit dem Modul "GetJointValuesById" laden. Dazu muss man nur den Namen einer der zuvor im "World View" abgespeicherten Gelenkwinkelkonfigurationen (z.B. "JointValues_1") in das Feld "id" dieses Moduls eintragen. 

#### Modul-Verbindungen:

Verbunden werden die so konfigurierten Module nun folgendermaßen: 

Flow-Connections: <br />
Start-Modul -> "CreatePlanParameters"-Modul -> "GetJointValueById"-Modul -> "MoveJ"-Modul -> "End"-Modul

Parallel-Connections: <br />
* "CreatePlanParameters":"Parameters" -> "MoveJ":"Parameters"
* "GetJointValuesById":"Result0" -> "MoveJ":"target"

Will man überprüfen, ob die Planungsparameter und die Zielgelenkwinkel richtig gesetzt wurden, so baut man noch die Module "PropertyAccessor" und "ViewString" an verschiedenen Stellen ein:

* "CreatePlanParameters":"Parameters" -> "PropertyAccessor":"Item" und "PropertyAccessor":"MoveGroupName" -> "ViewString":"string" sollte jetzt den Namen der MoveGroup also "urcontroller" Anzeigen.
* "GetJointValuesById":"Result0" -> "PropertyAccessor":"Item" und "PropertyAccessor":"JointSet" -> "ViewString":"string" bzw. "PropertyAccessor":"Values" -> "ViewString":"string" sollte jetzt die Namen bzw. Werte der Gelenke anzeigen.

#### Abbildung des Beispielgraphen:

.. figure:: images/ROSVITA_Example_Graph.png

   Figure 9.1  ROSVITA example graph.

TODO:
* Subgraphen


