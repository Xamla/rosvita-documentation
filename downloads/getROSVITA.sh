#!/bin/bash

echo "Docker will be installed if not already there, press CTRL+C to cancel"
read -p "Press ENTER to continue"
if [[ $(docker -v | grep -c 'Docker version') > 0 ]]; then
   echo "Docker is already installed."
else
   echo "Starting installation of Docker.."
   sudo apt-get remove docker docker-engine docker.io
   sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
   if [[ $(lsb_release -rs) == "18.04" ]]; then
      sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu artful stable"
   else
      sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
   fi
   sudo apt-get update
   sudo apt-get install docker-ce
   if [[ $(sudo docker -v | grep -c 'Docker version') > 0 ]]; then
      echo "Docker CE has been installed successfully."
   else
      echo "Docker installation failed. Please try again."
      exit 1
   fi
   echo "Adding user to group docker.."
   sudo usermod -aG docker $USER
   echo "Log off your user and then log in again to activate group changes." 
   echo "After that open a terminal and run 'cd ~/Downloads; ./getROSVITA.sh' again."
   exit 0
fi
echo ""

echo "The ROSVITA image will be downloaded from Docker Hub, press CTRL+C to cancel"
read -p "Press ENTER to continue"
echo "Getting ROSVITA from Docker Hub.."
docker pull xamla/early-access-rosvita:v0.2
if [[ $(docker images | grep -c 'early-access-rosvita') == 0 ]]; then
   echo "Please log in to Docker first (via 'docker login'). Then run './getROSVITA.sh' again."
   exit 1
else
   echo "ROSVITA image has been downloaded successfully."
   echo "Use 'docker rmi <image-id>' to remove old images and save disk space."
fi
echo ""

echo "Creating script '/opt/Rosvita/rosvita_start.sh' for starting ROSVITA.."
cd /opt
if [[ $(ls | grep -x Rosvita | wc -l) == 0 ]]; then
   sudo mkdir Rosvita
   sudo chown -R $USER /opt/Rosvita/
   cd Rosvita
   wget -q https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_start.sh
   sudo chmod a+x rosvita_start.sh
else
   cd Rosvita
   if [[ $(ls | grep -x rosvita_start.sh | wc -l) == 0 ]]; then
      wget -q https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_start.sh
      sudo chmod a+x rosvita_start.sh
   else
      echo "ROSVITA start script already exists."
   fi
fi

echo "Creating desktop icon for starting ROSVITA.."
cd ${HOME}/Desktop
if [[ $(ls | grep -x ROSVITA.desktop | wc -l) > 0 ]]; then
   echo "ROSVITA desktop icon already exists."
else
   cd /opt/Rosvita
   wget -q https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_icon.png
   cd ${HOME}/Desktop
   wget -q https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/ROSVITA.desktop
   sudo chmod a+x ROSVITA.desktop
fi
echo "Finished."
echo ""

echo "ROSVITA will be started, press CTRL+C to cancel"
read -p "Press ENTER to continue"
/opt/Rosvita/rosvita_start.sh
