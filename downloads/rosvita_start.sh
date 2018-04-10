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

echo "Check if ROSvita is already running..."
if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
        echo "YES"
        echo "ROSvita is already running"
else
        echo "NO"
        echo "Starting ROSvita"
        docker run -dti --net=host --rm --name=rosvita --user xamla --privileged -e DISPLAY=$DISPLAY -e host_uid=$(id -u) -e host_gid=$(id -g) -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix:/tmp/.X11-unix -v ${HOME}/Rosvita/data:/home/xamla/Rosvita.Control/data -v ${HOME}/Rosvita/projects:/home/xamla/Rosvita.Control/projects -v ${HOME}/Rosvita/robot_parts:/home/xamla/Rosvita.Control/library/robot_parts/custom xamla/early-access-rosvita:v0.2 rosvita
fi

if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
        echo "Ok"
        echo "Use 'docker attach rosvita' to attach to the container. Use CTRL+P, CTRL+Q to detach from the container."
else
        echo "Failed"
        exit 1
fi


echo "Open browser interface with Google Chrome"
if [[ $(google-chrome --version | grep -c 'Google Chrome') > 0 ]]; then
   google-chrome http://localhost:5000
else
   echo "Google Chrome will be installed, press CTRL+C to cancel"
   read -p "Press ENTER to continue"
   sudo apt-get install libxss1 libappindicator1 libindicator7
   cd ${HOME}/Downloads
   wget -q https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
   sudo dpkg -i google-chrome*.deb
   sudo apt-get install -f
   sudo dpkg -i google-chrome*.deb
   google-chrome http://localhost:5000
fi
