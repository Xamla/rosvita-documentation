:orphan:

.. _docker-installation-label:

*********************************************
Short description of Docker CE installation:
*********************************************

The required steps for the Docker CE installation are:

* Uninstall any older Docker versions if existing.
* Install packages to allow "apt" to use a repository over HTTPS.
* Add Docker’s official GPG key.
* Add the Docker repository to the "apt" sources.
* Update the package database.
* Install the latest version of Docker CE.

To perform these steps, enter the following commands into the terminal::

   sudo apt-get remove docker docker-engine docker.io
   sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
   sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
   sudo apt-get update
   sudo apt-get install docker-ce

Now, optionally check if Docker is running and add your username (here: "rosvita") to the Docker group::

   sudo systemctl status docker
   sudo usermod -aG docker rosvita
