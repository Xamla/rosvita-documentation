:orphan:

.. _get_docker-label:

**************
get_docker
**************

.. code-block:: bash

   #!/bin/bash
   
   echo "Installing ROSVITA dependencies.."
   echo ""
   
   echo "Step 1:"
   echo "======="
   echo "Docker will be installed if not already there, press CTRL+C to cancel"
   read -p "Press ENTER to continue and follow the instructions on screen"
   
   if [[ $(docker -v | grep -c 'Docker version') > 0 ]]; then
      echo "Docker is already installed."
   else
      echo "Starting installation of Docker.."
      sudo apt-get remove docker docker-engine docker.io containerd runc
      sudo apt-get install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
      curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
      sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
      sudo apt-get update
      sudo apt-get install -y docker-ce docker-ce-cli containerd.io
      if [[ $(sudo docker -v | grep -c 'Docker version') > 0 ]]; then
         echo "Docker CE has been installed successfully."
      else
         echo "Docker installation failed. Please try again."
         exit 1
      fi
      echo "Adding user to group docker.."
      sudo usermod -aG docker $USER
   fi
   echo ""
   
   echo "Step 2:"
   echo "======="
   echo "NVIDIA-Docker 2.0 will be installed if not already there."
   echo "Make sure you have installed the NVIDIA driver and Docker before."
   echo "Press CTRL+C to cancel"
   read -p "Press ENTER to continue"
   
   if [[ $(nvidia-docker version | grep -c 'NVIDIA Docker: 2.') > 0 ]]; then
      echo "NVIDIA-Docker 2.0 is already installed."
   else
      echo "Starting installation of NVIDIA-Docker 2.0.."
      if [[ $(nvidia-docker version | grep -c 'NVIDIA Docker: 1.') > 0 ]]; then
         echo "Remove NVIDIA-Docker 1.0 and all existing GPU containers."
         sudo docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
         sudo apt-get purge -y nvidia-docker
      fi
      echo "Add the package repositories"
      curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
      distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
      curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
      sudo apt-get update
      echo "Install nvidia-docker2 and reload the Docker daemon configuration."
      sudo apt-get install -y nvidia-docker2
      sudo pkill -SIGHUP dockerd
      echo "Test nvidia-smi with the latest official CUDA image"
      sudo docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi
   fi
   echo ""
   
   echo "Step 3:"
   echo "======="
   echo "If you are not already in group docker, the computer will be"
   echo "restarted now to make the addition to the docker group effective."
   echo "Press CTRL+C to cancel"
   read -p "Press ENTER to continue"
   if [[ $(groups | grep docker | wc -l) > 0 ]]; then
      echo "You are already in group docker."
   else
      echo "IMPORTANT:"
      echo "The computer will be restarted now to make the addition to the docker group effective."
      sudo reboot
   fi

