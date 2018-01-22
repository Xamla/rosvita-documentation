Welcome to ROSVITA
===================

ROSVITA is an easy-to-use in-browser **robot programming environment** based on the "Robot Operating System" `ROS <http://www.ros.org/>`_.
ROS is a collection of open-source software libraries and programs for the development of robotic applications. The programming platform ROSVITA has as many degrees of freedom as ROS, but is characterized by its ease of use and the reliability of all integrated ROS components.


Why Rosvita?
------------

ROSVITA's intuitive user interface and **high-quality 3D visualizations** allow the user to interact with the robot and its environment in **real-time**. This enables the user to quickly test a variety of strategies to plan complex operations, such as sensor-feedback-driven pick-and-place operations. The ease of use and especially the graphical user interface of ROSVITA enable even inexperienced users to train complex robot behavior fast and easily. For more experienced users, an **integrated source code editor** is available. Of course, you can also use your own ROS nodes with the system and start your own experiments. The configuration of ROS including the robot motion planner `"MoveIt!" <http://moveit.ros.org/>`_ can be done in just a few minutes. All integrated ROS components run reliably and **can be used industrially** immediately.
In addition, ROSVITA has a **visual workflow editor with expandable module system**, a **file browser**, a **text editor with shell integration** for expert commands, and **permanent monitoring** of all system components using so-called "heartbeats" (periodically generated signals send from the individual components to the central system).

The main application area for ROSVITA is the programming of adaptive behavior of robot arms, in particular dual-arm robots and kinematics with more than 6 degrees of freedom. The associated end effectors and sensor heads developed by our company (http://xamla.com) are equipped with optical and tactile sensors and allow the robot to "see" and "feel" its environment. Together with our **gripper jaw changing system**, ROSVITA enables the use of a single robot cell assembly for a wide range of tasks.


Getting Startet
----------------

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is:

* An up-to-date `Ubuntu <https://help.ubuntu.com/community/Installation/>`_ operating system (16.04 or higher),
* The software container platform `Docker <https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#install-docker-ce>`_,
* And finally the `ROSVITA Docker image <http://xamla.com/en/#early-access>`_.

After downloading the ROSVITA Docker image ("rosvita_production_container.tar.gz"), you can import it to Docker with the following command::

   cat rosvita_production_container.tar.gz | docker import - rosvita-server-prod-flat:v0.1

After that, ROSVITA can be started by creating and executing a shell script "rosvita-start.sh" with the following content::

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

Before executing the script, go into your home folder and create a folder "Rosvita" with two subfolders "data" and "projects"::

   cd /home/<username>/; mkdir Rosvita; cd Rosvita; mkdir data projects

To be able to execute the start script, you probably first have to change permissions with the following command::

   usermod u+x rosvita-start.sh

Then execute the start script by simply typing::

   ./rosvita-start.sh

The following text should appear in the terminal::

   Check if ROSvita is already running... NO
   Starting ROSvita
   <longer chain of characters>
   Ok
   Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container.

In an internet browser (for example in "Chrome"), the user interface of the "Rosvita Robot Programming System" can now be opened by entering ``localhost: 5000`` in the address bar.
The login screen appears. After successful login with username and password, the ROSVITA main development environment opens.

.. note:: All changes to files of the ROSVITA Docker image are temporary and get lost after an update of the image. Therefore, **do not modify the ROSVITA Docker image, but always make changes locally in your own project folder** (/home/<username>/Rosvita/projects/).


.. toctree::
   :maxdepth: 2
   :caption: User Documentation

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

