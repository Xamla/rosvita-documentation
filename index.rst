.. Rosvita documentation master file, created by
   sphinx-quickstart on Mon Jan 15 15:08:46 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Rosvita's documentation!
===================================

ROSVITA is an easy-to-use in-browser robot programming environment based on the "Robot Operating System" ROS.


Why Rosvita?
------------

ROSVITA's intuitive user interface and high-quality 3D visualizations allow the user to interact with the robot and the scene in real-time. This enables the user to quickly test a variety of strategies and approaches to plan complex operations, such as sensor-feedback-driven pick-and-place operations. The ease of use and graphical user interface of ROSVITA enables even inexperienced users to train complex robot behavior fast and easily. For more experienced users, an integrated source code editor is available. You can also use your own ROS nodes with the system and start your own experiments.

ROS is a collection of open-source software libraries and programs for the development of robotic applications.
The programming platform ROSVITA has as many degrees of freedom as ROS, but is characterized by its ease of use and the reliability of all integrated ROS components. The configuration of ROS including the robot motion planner "MoveIt!" can be done in just a few minutes. All integrated ROS components run reliably and can be used industrially immediately.

In addition, ROSVITA has a visual workflow editor with expandable module system, a file system browser, a text editor with shell integration for expert commands, and permanent monitoring of all system components using so-called "heartbeats" (periodically generated signals send from the  individual components to the central system).

The main application area for ROSVITA is the programming of adaptive behavior of robot arms, in particular dual-arm robots and kinematics with more than 6 degrees of freedom. The associated end effectors and sensor heads developed by us are equipped with optical and tactile sensors and allow the robot to "see" and "feel" its environment. Together with the gripper jaw changing system developed by us, ROSVITA enables the use of a single robot cell assembly for a wide range of tasks.


The Docker Image Concept
------------------------

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is an up-to-date Ubuntu operating system, as well as Docker `Docker <https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#install-docker-ce>`_ . After downloading the ROSVITA Docker image, you can import it with the following command::

   cat rosvita_production_container.tar.gz | docker import - rosvita-server-prod-flat:v0.1

After that, ROSVITA can be started by executing the following shell script (which in the following will be called rosvita-start.sh)::

   #!/bin/bash

   printf "Check if ROSvita is already running... "
   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
           echo "YES"
           echo "ROSvita is already running"
           echo "Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container."
           exit 0
   else
           echo "NO"
           echo "Starting ROSvita"
           docker run -dti --net=host --rm --name=rosvita --privileged -v /dev/bus/usb:/dev/bus/usb -v /home/rosvita/Rosvita/data:/home/xamla/Rosvita.Control/data -v /home/rosvita/Rosvita/projects:/home/xamla/Rosvita.Control/projects rosvita-server-prod-flat:v0.1 rosvita
   fi

   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
           echo "Ok"
           echo "Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container." 
   else 
           echo "Failed"
           exit 1
   fi

**Important**: All changes to files on the ROSVITA server are temporary and get lost after an update of the ROSVITA Docker image. Therefore, **you should not modify the ROSVITA Docker image**, but **always make changes locally in your own project folder**. This is also the reason why "RobotParts" are copied into your own project folder when you select them from the library and add them to your project. In this way, your individual changes to the robot model are stored in your local project folder.


Getting Startet
---------------

After the computer has been set up successfully (including import of the ROSVITA Docker image), ROSVITA can be easily started by calling the start script. To do this, open a terminal and enter the following commands::

   cd /home/rosvita/
   ./rosvita-start.sh

Then the following text should appear in the terminal::

   Check if ROSvita is already running... NO
   Starting ROSvita
   <longer chain of characters>
   Ok
   Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container.

In an internet browser (for example in "Chrome" or "Firefox"), the user interface of the "Rosvita Robot Programming System" can now be opened by entering ``localhost: 5000`` in the address bar.
The login screen appears. After successful login with username and password, the ROSVITA main development environment opens.


Contents:
=========

.. toctree::
   :maxdepth: 2

   Main_View
   New_Project
   Robot_Configuration
   Path_Planning
   Robot_Jogging
   File_Browser
   Code_Editor
   Own_Robot_Models
   Graph_Concept
   Lua_Scripts
   Complex_Scene


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
