.. _getting-started-label:

*****************
Getting Startet
*****************

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is:

* An up-to-date `Ubuntu <https://help.ubuntu.com/community/Installation/>`_ operating system (16.04 or higher),
* The software container platform `Docker <https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#install-docker-ce>`_ (:ref:`short installation instructions <docker-installation-label>`),
* And finally the **ROSVITA Docker image**.

.. note:: For safety reasons, it is recommended to use **two network cards**: one for your intranet and one for your robot. For setting up the two network connections, click on the symbol with the two arrows in the top bar of your Ubuntu installation and select the menu item "Edit Connections...". For your intranet choose a dynamic IP address (IPv4 Settings -> Method: Automatic (DHCP)) and for your robot choose a static IP address (IPv4 Settings -> Method: Manual).

To be able to pull the ROSVITA Docker image from `Docker Hub <https://hub.docker.com/>`_, you need a Docker account. If you do not have a Docker ID already, simply `sign up to Docker <https://cloud.docker.com/>`_ with username, Email address and password. If you are interested in getting early access to a BETA version of ROSVITA, please `contact us <http://xamla.com/en/#early-access>`_ so we can give you permission to pull the ROSVITA Docker image from Docker Hub. Moreover, we intend to make publicly available a first official version of ROSVITA in mid-2018. From then on, everyone with valid Docker account should be able to ``docker pull`` the ROSVITA image from Docker Hub without restrictions.

Use the following commands to pull the latest ROSVITA Docker image from Docker Hub::

   docker login
   docker pull xamla/early-access-rosvita:rosvita-images-v0.1

Thereafter, ROSVITA can be started by creating and executing a shell script **rosvita-start.sh** with the following content::

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
           docker run -dti --net=host --rm --name=rosvita --user xamla --privileged -v /dev/bus/usb:/dev/bus/usb -v ~/Rosvita/data:/home/xamla/Rosvita.Control/data -v ~/Rosvita/projects:/home/xamla/Rosvita.Control/projects -v ~/Rosvita/robot_parts:/home/xamla/Rosvita.Control/library/robot_parts/custom xamla/early-access-rosvita:rosvita-images-v0.1 rosvita
   fi

   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
           echo "Ok"
           echo "Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container." 
   else 
           echo "Failed"
           exit 1
   fi

.. note:: Change the name of the ROSVITA Docker image used in this script to the actual name of the current image ("xamla/early-access-rosvita:rosvita-images-v0.1" -> "<name of current rosvita image>").

Before executing the script, go into your home folder and create a folder "Rosvita" with three subfolders "data", "projects" and "robot_parts"::

   cd /home/<username>/; mkdir Rosvita; cd Rosvita; mkdir data projects robot_parts

To be able to execute the start script, you probably first have to change permissions with the following command::

   chmod u+x rosvita-start.sh

Then execute the start script by simply typing::

   ./rosvita-start.sh

The following text should appear in the terminal::

   Check if ROSvita is already running... NO
   Starting ROSvita
   <longer chain of characters>
   Ok
   Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container.

In an internet browser (for example in "Chrome"), the user interface of the "Rosvita Robot Programming System" can now be opened by entering ``localhost:5000`` in the address bar.
The login screen appears. After successful login with username and password, the ROSVITA main development environment opens.

To stop ROSVITA simply type::

   docker stop rosvita

.. note:: All changes to files of the ROSVITA Docker image are temporary and get lost after an update of the image. Therefore, **do not modify the ROSVITA Docker image, but always make changes locally in your own project folder** (/home/<username>/Rosvita/projects/).

