*****************
Getting Startet
*****************

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is:

* An up-to-date `Ubuntu <https://help.ubuntu.com/community/Installation/>`_ operating system (16.04 or higher),
* The software container platform `Docker <https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#install-docker-ce>`_,
* And finally the `ROSVITA Docker image <http://xamla.com/en/#early-access>`_.

.. note:: For safety reasons, it is recommended to use **two network cards**: one for your intranet and one for your robot. For setting up the two network connections, click on the symbol with the two arrows in the top bar of your Ubuntu installation and select the menu item "Edit Connections...". For your intranet choose a dynamic IP address (IPv4 Settings -> Method: Automatic (DHCP)) and for your robot choose a static IP address (IPv4 Settings -> Method: Manual).

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
           docker run -dti --net=host --rm --name=rosvita --user xamla --privileged -v /dev/bus/usb:/dev/bus/usb -v /home/rosvita/Rosvita/data:/home/xamla/Rosvita.Control/data -v /home/rosvita/Rosvita/projects:/home/xamla/Rosvita.Control/projects rosvita-server-prod-flat:v0.1 rosvita
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

In an internet browser (for example in "Chrome"), the user interface of the "Rosvita Robot Programming System" can now be opened by entering ``localhost:5000`` in the address bar.
The login screen appears. After successful login with username and password, the ROSVITA main development environment opens.

To stop ROSVITA and to remove the ROSVITA container simply type::

   docker stop rosvita
   docker rm rosvita

.. note:: All changes to files of the ROSVITA Docker image are temporary and get lost after an update of the image. Therefore, **do not modify the ROSVITA Docker image, but always make changes locally in your own project folder** (/home/<username>/Rosvita/projects/).

