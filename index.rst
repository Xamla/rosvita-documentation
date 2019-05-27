Welcome to ROSVITA
===================

ROSVITA is an easy-to-use in-browser **robot programming environment** based on the "Robot Operating System" `ROS <http://www.ros.org/>`_.
ROS is a collection of open-source software libraries and programs for the development of robotic applications. The programming platform ROSVITA has as many degrees of freedom as ROS, but is characterized by its ease of use and the reliability of all integrated ROS components.


Why Rosvita?
------------

ROSVITA's intuitive user interface and **high-quality 3D visualizations** allow the user to interact with the robot and its environment in **real-time**. This enables the user to quickly test a variety of strategies to plan complex operations, such as sensor-feedback-driven pick-and-place operations. The ease of use and especially the graphical user interface of ROSVITA enable even inexperienced users to train complex robot behavior fast and easily. For more experienced users, an **integrated source code editor** is available. Of course, you can also use your own ROS nodes with the system and start your own experiments. The configuration of ROS including the robot motion planner `"MoveIt!" <http://moveit.ros.org/>`_ can be done in just a few minutes. All integrated ROS components run reliably and **can be used industrially** immediately.
In addition, ROSVITA has a **visual workflow editor with expandable module system**, a **file browser**, a **text editor with shell integration** for expert commands, and **permanent monitoring** of all system components using so-called "heartbeats" (periodically generated signals send from the individual components to the central system).

The main application area for ROSVITA is the programming of adaptive behavior of robot arms, in particular dual-arm robots and kinematics with more than 6 degrees of freedom. The associated end effectors and sensor heads developed by our company (http://xamla.com) are equipped with optical and tactile sensors and allow the robot to "see" and "feel" its environment. Together with our **gripper jaw changing system**, ROSVITA enables the use of a single robot cell assembly for a wide range of tasks.



.. toctree::
   :maxdepth: 2
   :caption: User Documentation

   source/Getting_Started
   source/Main_View
   source/New_Project
   source/Robot_Configuration
   source/Path_Planning
   source/Robot_Jogging
   source/File_Browser
   source/Code_Editor
   source/Own_Robot_Models
   source/Graph_Concept
   source/Module_Debugging
   source/Lua_Scripts
   source/Combination_with_External_ROS
   source/Complex_Scene
   source/Python_In_Rosvita_Graphs
   source/How_To_Stereo_Laser_Line_Client
   source/Calibration_Part1
   source/Calibration_Part2
   source/Force_Torque_Data

