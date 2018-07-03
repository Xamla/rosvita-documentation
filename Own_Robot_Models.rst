*******************************
Creation of Own Robot Models
*******************************

In the ROSVITA file browser under the path ``/home/xamla/Rosvita.Control/library/robot_parts/`` you will find the different robot models (for example ``universal_robots/part_ur5``). These include the following files and folders:

* Folder "collision" (contains the .stl files for the collision model)
* Folder "visual" (contains the .dae files for the visualization)
* .xacro file containing the description of the robot model
* File "part.json" containing the MoveIt! configuration of the robot model
* Files "CMakeLists.txt" and "package.xml" for compiling in a "Catkin" workspace under ROS.

To make your own robot model selectable in ROSVITA, you first have to **provide all of the above listed files that belong to a ROSVITA robot configuration**.
In some cases, corresponding files are already provided by the robot manufacturer (such as from `Universal Robots at GitHub <https://github.com/ros-industrial/universal_robot/>`_). In this case, the corresponding files and folders only need to be adjusted so that they **match the ROSVITA folder structure**.
If you want to build a robot model completely by yourself, the following `URDF/Xacro-Tutorials <http://wiki.ros.org/urdf/Tutorials>`_, as well as this `explanation of the URDF <http://wiki.ros.org/urdf/XML/model>`_ will help to construct your own robot model.

*The "Unified Robot Description Format" (URDF) is an XML specification to describe a robot. The use of this specification assumes that the robot can be described as a tree structure and that the robot consists of rigid links connected by joints. The specification covers the kinematic and dynamic description of the robot, the visual representation and the collision model of the robot. Moreover, "Xacro" is an XML macro language. With "Xacro" you can construct shorter and more readable XML files by using macros that bundle larger XML expressions into more compact ones.*

Once you have created all files belonging to a ROSVITA robot configuration and ensured that they match the ROSVITA folder structure, you simply have to **place the parent folder with these files into your local robot_parts folder** (``/home/<username>/Rosvita/robot_parts``) and **restart ROSVITA** once (e.g. ``docker stop rosvita; /opt/Rosvita/rosvita_start.sh``). Now, your new robot model should be selectable in the ROSVITA **Configuration View**.

.. note:: In the current ROSVITA start script **rosvita_start.sh** (see chapter :ref:`getting-started-label`) there is a Docker mapping of your own local **robot_parts** folder (``/home/<username>/Rosvita/robot_parts``) to the folder **robot_parts/custom** in the ROSVITA library (``/home/xamla/Rosvita.Control/library/robot_parts/custom``). So to make your own robot model selectable in ROSVITA, it is not necessary to upload it to the ROSVITA server, but it is sufficient to place it in your own local **robot_parts** folder and restart ROSVITA once.

.. note:: When creating your own robot models, make sure that these models do not become too large (do not use several GigaByte large CAD files), because working with (and especially visualizing) such huge models is naturally very slow.


Relocation of the "Tool Center Point (TCP)":
--------------------------------------------

To relocate the tool center point and the corresponding interactive marker in the 3D View, a **tcp_link** must be added to the file **robotModel/main.xacro** in the corresponding project folder. 
More precisely, you have to add the following lines inside the **<robot> </robot>** environment

Example::

   <link name="tcp_link" />
   <joint name="tcp_joint_F" type="fixed">
     <child link="tcp_link" />
     <parent link="wrist_3_link" />
     <origin xyz="-0.0037 -0.0072 0.2092" rpy="0 0 0" />      
   </joint>

**Origin xyz** and **rpy** are the position and rotation (Euler angles) of the TCP relative to the parent link (here: **wrist_3_link**).
After this addition of the **tcp_link** in the file **robotModel/main.xacro**, this file has to be recompiled by pressing the **Compile** button of the ROSVITA xacro editor.
Hereupon, the new tcp link appears in the 3D view to the right of the opened .xacro file.
When changing to the **Configuration View**, the new configuration must be compiled first (i.e. click the **Compile** button in the **Configuration View**) so that the new link is displayed and selectable (e.g. as **Tip link** of a **move group** and **Parent link** of an **end effector**).
Thereto, in the configuration view under **MotionPlanning->MoveIt! Motion Planning Framework (ROS)->groups-><group_name>** select the new **tcp_link** as **Tip link** and under **...->endEffectors-><end_effector_name>** select the new **tcp_link** as **Parent link**. After building the new robot configuration, starting ROS and changing into the **World View**, the interactive marker appears at the new tcp link.


