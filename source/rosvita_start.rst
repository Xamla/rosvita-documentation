:orphan:

.. _rosvita_start-label:

**************
rosvita_start
**************

.. code-block:: bash

   #!/bin/bash

   echo "Check if folder 'Rosvita' with subfolders 'data', 'projects' and 'robot_parts' exist and create them otherwise..."
   cd ~
   if [[ $(ls | grep -x Rosvita | wc -l) > 0 ]]; then
      cd Rosvita
      if [[ $(ls | grep -x data | wc -l) == 0 ]]; then
         echo "Creating subfolder 'data'."
         mkdir data
      fi
      if [[ $(ls | grep -x projects | wc -l) == 0 ]]; then
         echo "Creating subfolder 'projects'."
         mkdir projects
      fi
      if [[ $(ls | grep -x robot_parts | wc -l) == 0 ]]; then
         echo "Creating subfolder 'robot_parts'."
         mkdir robot_parts
      fi
   else
      echo "Creating folder 'Rosvita' with subfolders 'data', 'projects' and 'robot_parts'."
      mkdir Rosvita
      cd Rosvita
      mkdir data projects robot_parts
   fi

   echo "Check if ROSVITA is already running..."
   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
      echo "YES"
      echo "ROSVITA is already running"
   else
      echo "NO"
      if [[ "$1" == "--debug" ]]; then
         echo "Starting debug mode of ROSVITA"
         echo "To start ROSVITA afterwards, simply type 'rosvita' after pressing 'Enter'."
         echo "Then open an internet browser and type 'http://localhost:5000/' in the adress bar."
         read -p "Press ENTER to continue"
         docker run -ti --net=host --rm --name=rosvita --user xamla --privileged -e DISPLAY=$DISPLAY -e host_uid=$(id -u) -e host_gid=$(id -g) -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix:/tmp/.X11-unix -v ${HOME}/Rosvita/data:/home/xamla/Rosvita.Control/data -v ${HOME}/Rosvita/projects:/home/xamla/Rosvita.Control/projects -v ${HOME}/Rosvita/robot_parts:/home/xamla/Rosvita.Control/library/robot_parts/custom xamla-buildserver:5000/devel/rosvita-basf-cuda:latest bash
      else
         echo "Starting ROSVITA"
         docker run -dti --net=host --rm --name=rosvita --user xamla --privileged -e DISPLAY=$DISPLAY -e host_uid=$(id -u) -e host_gid=$(id -g) -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix:/tmp/.X11-unix -v ${HOME}/Rosvita/data:/home/xamla/Rosvita.Control/data -v ${HOME}/Rosvita/projects:/home/xamla/Rosvita.Control/projects -v ${HOME}/Rosvita/robot_parts:/home/xamla/Rosvita.Control/library/robot_parts/custom xamla-buildserver:5000/devel/rosvita-basf-cuda:latest rosvita
      fi
   fi

   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
      echo "Ok"
      echo "Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container."
   else
      echo "Failed"
      exit 1
   fi

   echo "Opening ROSVITA in your default web browser..."
   attempt_counter=0
   max_attempts=40
   until $(curl --output /dev/null --silent --fail http://localhost:5000/api/version); do
      if [[ ${attempt_counter} -eq ${max_attempts} ]]; then
         echo "Maximum number of attempts reached. Please try again."
         exit 1
      fi
      printf '.'
      attempt_counter=$(($attempt_counter+1))
      sleep 0.25
   done
   printf '\n'
   xdg-open http://localhost:5000
