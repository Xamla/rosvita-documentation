.. _getting-started-label:

*****************
Getting Started
*****************

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is:

* An up-to-date |Ubuntu_link| operating system (16.04 or higher),
* The software container platform |Docker_link| (:ref:`short installation instructions <docker-installation-label>`),
* And finally the **ROSVITA Docker image**.

.. note:: For safety reasons, it is recommended to use **two network cards**: one for your intranet and one for your robot. For setting up the two network connections, click on the symbol with the two arrows in the top bar of your Ubuntu installation and select the menu item "Edit Connections...". For your **intranet** choose a **dynamic IP address** (IPv4 Settings -> Method: Automatic (DHCP)) and for your **robot** choose a **static IP address** (IPv4 Settings -> Method: Manual).

To be able to pull the ROSVITA Docker image from |Docker_Hub_link|, you need a Docker account. If you do not have a Docker ID already, simply |Docker_SignUp_link| with username, Email address and password. If you are interested in getting early access to a BETA version of ROSVITA, please |Xamla_EarlyAccess_link| so we can give you permission to pull the ROSVITA Docker image from Docker Hub. Moreover, we intend to make publicly available a first official version of ROSVITA in mid-2018. From then on, everyone with valid Docker account should be able to ``docker pull`` the ROSVITA image from Docker Hub without restrictions.

Use the following commands to pull the latest ROSVITA Docker image from Docker Hub. 
**Change the version tag v0.2 to the current version number!**

.. code-block:: bash

   sudo docker login
   sudo docker pull xamla/early-access-rosvita:v0.2

Thereafter, ROSVITA can be started by following the steps:

1. Go into your home folder and download the shell script |Rosvita_StartScript_link|.
2. Change permissions to be able to execute the start script.
3. And finally execute the script.

.. code-block:: bash

   cd ~
   wget https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/Downloads/rosvita_start.sh
   chmod u+x rosvita_start.sh
   ./rosvita_start.sh

.. note:: **Change the version tag v0.2** of the ROSVITA Docker image used in this script **to the actual version number:** ``xamla/early-access-rosvita:v0.2 -> xamla/early-access-rosvita:vX.X``.

The user interface of the "Rosvita Robot Programming System" can now be opened by entering ``localhost:5000`` in the address bar of your internet browser. We strongly recommend to use |Chrome_link|, because the user experience will be limited in Firefox due to the significantly slower browser performance. The login screen of ROSVITA appears. After successful login with username and password (default: **admin** and **r2d2c3po**), the ROSVITA main development environment opens (see :ref:`next chapter <main-view-label>`).

To stop ROSVITA simply type:

.. code-block:: bash

   docker stop rosvita

.. note:: All changes to files of the ROSVITA Docker image are temporary and get lost after an update of the image. Therefore, **do not modify the ROSVITA Docker image, but always make changes locally in your own project folder** (~/Rosvita/projects/).




.. |Ubuntu_link| raw:: html

   <a href="https://help.ubuntu.com/community/Installation/" target="_blank">Ubuntu</a> 

.. |Docker_link| raw:: html

   <a href="https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce" target="_blank">Docker</a> 

.. |Docker_Hub_link| raw:: html

   <a href="https://hub.docker.com/" target="_blank">Docker Hub</a> 

.. |Docker_SignUp_link| raw:: html

   <a href="https://cloud.docker.com/" target="_blank">sign up to Docker</a> 

.. |Xamla_EarlyAccess_link| raw:: html

   <a href="http://xamla.com/en/#early-access" target="_blank">contact us</a> 

.. |Rosvita_StartScript_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/Downloads/rosvita_start.sh" target="_blank">rosvita_start.sh</a> 

.. |Chrome_link| raw:: html

   <a href="https://www.google.com/intl/en-CA/chrome/" target="_blank">Chrome</a> 

