.. _getting-started-label:

*****************
Getting Started
*****************

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is an up-to-date |Ubuntu_link| operating system (preferentially **Ubuntu 16.04 or higher**) and an up-to-date internet browser (preferentially |Chrome_link|).

To **get ROSVITA**, simply download the shell script |get_script_link|, then change permissions to be able to execute the script und finally run the script, i.e. in a terminal type:

.. code-block:: bash

   wget -q https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/get_rosvita
   chmod u+x get_rosvita
   ./get_rosvita

The script will:

1. Install |Docker_link| if not already installed, and add user to group docker.
2. Download the **ROSVITA Docker image** from |Docker_Hub_link|.
3. Download the **ROSVITA start script**: |start_script_link|.
4. Download the **ROSVITA stop script**: |stop_script_link|.
5. Create a **desktop icon** for starting ROSVITA.
6. If not already in group docker, **restart the computer** to activate group changes.

Now you can **start ROSVITA by double clicking the desktop icon**,
or by opening a terminal and running the start script by simply typing:

.. code-block:: bash

   rosvita_start

(To run ROSVITA in **debug mode** type ``rosvita_start --debug``. 
Afterwards enter **rosvita** into the terminal, open a web browser and enter ``localhost:5000`` into the address bar.)

As a result, the ROSVITA login screen appears in your default web browser. 
After successful login with username and password (default: **admin** and **r2d2c3po**), 
the ROSVITA main development environment opens (see :ref:`next chapter <main-view-label>`).

To **stop ROSVITA** simply run the ROSVITA stop script:

.. code-block:: bash

   rosvita_stop

If you have any trouble or would like to give us feedback, feel free to visit our |Rosvita_forum_link|.
Moreover, check out our |Rosvita_video_link|.

.. note:: We strongly recommend to run ROSVITA in **Google Chrome**, because in other internet browsers like Firefox the user experience will be limited due to the significantly slower browser performance.




.. |Ubuntu_link| raw:: html

   <a href="https://www.ubuntu.com/download/desktop" target="_blank">Ubuntu</a> 

.. |Docker_link| raw:: html

   <a href="https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce" target="_blank">Docker</a> 

.. |Docker_Hub_link| raw:: html

   <a href="https://hub.docker.com/explore/" target="_blank">Docker Hub</a> 

.. |get_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/get_rosvita" target="_blank">get_rosvita</a> 

.. |start_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_start" target="_blank">rosvita_start</a> 

.. |stop_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_stop" target="_blank">rosvita_stop</a> 

.. |Chrome_link| raw:: html

   <a href="https://www.google.com/intl/en-CA/chrome/" target="_blank">Google Chrome</a>

.. |Rosvita_forum_link| raw:: html

   <a href="http://discuss.xamla.com" target="_blank">ROSVITA forum</a>

.. |Rosvita_video_link| raw:: html

   <a href="https://youtu.be/VAfwk-MnBuA" target="_blank">ROSVITA tutorial video</a>

