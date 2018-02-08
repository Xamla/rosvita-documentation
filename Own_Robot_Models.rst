*******************************
Creation of Own Robot Models
*******************************

In the ROSVITA file browser under the path ``/home/xamla/Rosvita.Control/library/robot_parts/`` you will find the different robot models (for example ``universal_robots/part_ur5``). These include the following files and folders:

* Folder "collision" (contains the .stl files for the collision model)
* Folder "visual" (contains the .dae files for the visualization)
* .xacro file containing the description of the robot model
* File "part.json" containing the MoveIt! configuration of the robot model
* Files "CMakeLists.txt" and "package.xml" for compiling in a "Catkin" workspace under ROS.

To upload your own robot model, you first have to provide all of the above listed files that belong to a robot configuration.
In some cases, corresponding files are already provided by the robot manufacturer (such as from `Universal Robots at GitHub <https://github.com/ros-industrial/universal_robot/>`_). In this case, the corresponding files and folders only need to be adjusted so that they match the ROSVITA folder structure.
If you want to build a robot model completely by yourself, the following `URDF/Xacro-Tutorials <http://wiki.ros.org/urdf/Tutorials>`_, as well as this `explanation of the URDF <http://wiki.ros.org/urdf/XML/model>`_ will help to construct your own robot model.

*The "Unified Robot Description Format" (URDF) is an XML specification to describe a robot. The use of this specification assumes that the robot can be described as a tree structure and that the robot consists of rigid links connected by joints. The specification covers the kinematic and dynamic description of the robot, the visual representation and the collision model of the robot. Moreover, "Xacro" is an XML macro language. With "Xacro" you can construct shorter and more readable XML files by using macros that bundle larger XML expressions into more compact ones.*

To upload a folder containing your own robot model to the ROSVITA server, it is best to pack and compress it first: ``tar cvfz my_robot.tgz my_robot``. 
Then with the ROSVITA file browser go into the directory ``/home/xamla/Rosvita.Control/library/robot_parts/`` and press the "Upload" button, after which you can upload the folder "``my_robot.tgz``". Now the uploaded folder only needs to be unpacked. To do this, use the ROSVITA terminal to go to the higher-level folder ``cd /home/xamla/Rosvita.Control/library/robot_parts/`` and then enter the following command: ``tar -xzvf my_robot.tgz``.

.. note:: In the current ROSVITA start script **rosvita-start.sh** (see chapter :ref:`getting-started-label`) there is a Docker mapping of your own local "robot_parts" folder (``~/Rosvita/robot_parts``) to a subfolder "custom" of the folder "robot_parts" in the ROSVITA library (``/home/xamla/Rosvita.Control/library/robot_parts/custom``). So to make your own robot model selectable in ROSVITA, it is not necessary to upload it to the ROSVITA server, but it is sufficient to place it in your own "robot_parts" folder (``/home/<username>/Rosvita/robot_parts``) and restart ROSVITA once (``docker stop rosvita; ./rosvita-start.sh``).

.. note:: When creating your own robot models, make sure that these models do not become too large (do not use several GigaByte large CAD files), because working with (and especially visualizing) such huge models is naturally very slow.

In the ROSVITA file browser under the path ``/home/xamla/git/`` you will find some additional GitHub repositories with robot models and drivers. Under the path ``/home/xamla/catkin_ws/src/`` there are links to some of these repositories, links to some other components of ROSVITA, and most important a link to the folder ``/home/xamla/Rosvita.Control/library/robot_parts``. The ROSVITA terminal can be used to create links to other repositories (e.g. ``ln -s /home/xamla/git/my_robot_driver /home/xamla/catkin_ws/src/``). To build the catkin workspace with the ROSVITA terminal simply go into the catkin workspace (``cd /home/xamla/catkin_ws/``) and enter the command ``catkin_make``.

.. note:: All robot models uploaded to the library will get lost when ROSVITA is stopped (-> no permanent storage). It is therefore advisable to upload the self-created robot models to your own project folder (under ``/home/xamla/Rosvita.Control/projects/<project name>/robotModel/``). Changes in your own project folder are also stored locally in the corresponding project folder (under ``/home/<username>/Rosvita/projects/<project name>/robotModel/``) and are thus retained even after ROSVITA has been stopped and detached from the Docker container.


Relocation of the "Tool Center Point (TCP)":
--------------------------------------------

To relocate the tool center point and the corresponding interactive marker in the 3D View, a "tcp_link" must be added to the file "robotModel/main.xacro" in the corresponding project folder.

Example::

   <link name="tcp_link" />
   <joint name="tcp_joint_F" type="fixed">
     <child link="tcp_link" />
     <parent link="wrist_3_link" />
     <origin xyz="-0.0037 -0.0072 0.2092" rpy="0 0 0" />      
   </joint>

"origin xyz" and "rpy" are the position and rotation (Euler angles) of the TCP relative to the parent link (here: "wrist_3_link").
After this addition of the "tcp_link" in the file "robotModel/main.xacro", this file has to be recompiled by pressing the "Compile" button of the ROSVITA xacro editor.
Hereupon, the new tcp link appears in the 3D view to the right of the opened .xacro file.
When changing to the "Configuration View", the new configuration must be compiled first (i.e. click the "Compile" button in the "Configuration View") so that the new link is displayed and selectable (e.g. as "Tip Link" of a "Move Group").
Thereto, in the "Configuration View" under "MotionPlanning"->"MoveIt! Motion Planning Framework (ROS)"->"groups"->"*<group_name>*" select the new "tcp_link" as "Tip link". After building the new robot configuration, starting ROS and changing into the "World View", the interactive marker appears at the new tcp link.


