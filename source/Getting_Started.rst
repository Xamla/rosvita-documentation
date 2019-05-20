.. _getting-started-label:

*****************
Getting Started
*****************

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is an up-to-date |Ubuntu_link| operating system (preferentially **Ubuntu 16.04 or higher**) and an up-to-date internet browser (preferentially |Chrome_link|). Moreover, please read the ROSVITA :ref:`license-label`.

To **get** the **newest version of ROSVITA**, simply pull the latest ROSVITA docker image:

.. code-block:: bash

   docker pull xamla-buildserver:5000/devel/rosvita-basf-cuda

Now you can **start ROSVITA** by using the :ref:`rosvita_start-label` script.
More precisely, copy the content of this script into a file ``/home/<your-username>/rosvita_start``, 
then open a terminal, change permissions to be able to execute the script and run the script:

.. code-block:: bash

   cd /home/<your-username/
   sudo chmod a+x rosvita_start
   rosvita_start

(To run ROSVITA in **debug mode** type ``rosvita_start --debug``. 
Afterwards enter **rosvita** into the terminal, open a web browser and enter ``localhost:5000`` into the address bar.)

As a result, the ROSVITA login screen appears in your default web browser. 
After successful login with username and password (default: **admin** and **r2d2c3po**), 
the ROSVITA main development environment opens (see :ref:`next chapter <main-view-label>`).

To **stop ROSVITA** simply type:

.. code-block:: bash

   docker stop rosvita

or create and run a :ref:`rosvita_stop-label` script, i.e. copy the content of this script into a file ``/home/<your-username>/rosvita_stop``,
change permissions and run the script:

.. code-block:: bash

   cd /home/<your-username/
   sudo chmod a+x rosvita_stop
   rosvita_stop

With the terminal command ``docker images`` you will see all downloaded ROSVITA versions with corresponding image ids.
Use ``docker rmi <image-id>`` to remove old versions and save disk space.

In addition to reading this documentation, also check out our |Rosvita_video_link|, especially the |QuickStart_video_link|.

.. note:: We strongly recommend to run ROSVITA in **Google Chrome**, because in other internet browsers like Firefox the user experience will be limited due to the significantly slower browser performance.




.. |Ubuntu_link| raw:: html

   <a href="https://www.ubuntu.com/download/desktop" target="_blank">Ubuntu</a> 

.. |Docker_link| raw:: html

   <a href="https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce" target="_blank">Docker</a> 

.. |Docker_Hub_link| raw:: html

   <a href="https://hub.docker.com/explore/" target="_blank">Docker Hub</a> 

.. |get_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/master/rosvita/downloads/get_rosvita?token=ADKZO3UFCOADAVPFTNUQIYC45O6A2" target="_blank">get_rosvita</a> 

.. |start_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_start" target="_blank">rosvita_start</a> 

.. |stop_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_stop" target="_blank">rosvita_stop</a> 

.. |Chrome_link| raw:: html

   <a href="https://www.google.com/intl/en-CA/chrome/" target="_blank">Google Chrome</a>

.. |Rosvita_forum_link| raw:: html

   <a href="http://discuss.xamla.com" target="_blank">ROSVITA forum</a>

.. |Rosvita_video_link| raw:: html

   <a href="https://www.youtube.com/channel/UC37X4g0bLY7ID00RO_k8O4Q" target="_blank">ROSVITA tutorial videos</a>

.. |QuickStart_video_link| raw:: html

   <a href="https://youtu.be/VAfwk-MnBuA" target="_blank">ROSVITA quick start video</a>

.. |License_link| raw:: html

   <a href="https://github.com/Xamla/docs.xamla.com/blob/master/rosvita/downloads/LICENSE" target="_blank">license agreement</a>

