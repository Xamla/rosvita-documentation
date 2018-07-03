.. _getting-started-label:

*****************
Getting Started
*****************

To use ROSVITA, no complex installation process is necessary, because the ROSVITA Docker image already contains all the necessary packages and dependencies. All you need is an up-to-date |Ubuntu_link| operating system (preferentially **Ubuntu 16.04 or higher**) and an up-to-date internet browser (preferentially |Chrome_link|).

To **get ROSVITA**, simply download the shell script :download:`getROSVITA.sh <downloads/getROSVITA.sh>`.
Then change permissions to be able to execute the script und finally run the script, i.e. in a terminal type:

.. code-block:: bash

   cd ~/Downloads; sudo chmod u+x getROSVITA.sh; ./getROSVITA.sh

The script will:

1. Install |Docker_link| if not already installed.
2. Download the **ROSVITA Docker image** from |Docker_Hub_link|.
3. Download the **ROSVITA start script**: |start_script_link|.
4. Create a **desktop icon** for starting ROSVITA.
5. Finally **start ROSVITA** and open the user interface in your default web browser.

As a result, the ROSVITA login screen appears. After successful login with username and password (default: **admin** and **r2d2c3po**), the ROSVITA main development environment opens (see :ref:`next chapter <main-view-label>`).

To **stop ROSVITA** simply type:

.. code-block:: bash

   docker stop rosvita

To **restart ROSVITA** simply **doubleclick the desktop icon** or open a terminal and run the start script by typing:

.. code-block:: bash

   /opt/Rosvita/rosvita_start.sh

.. note:: Our shell scripts are written for **Bash**. Hence, **do not run them with \"sh\"**, but simply set them to executable (e.g. \"sudo chmod u+x rosvita_start.sh\") and call them directly as descriped above.

.. note:: We strongly recommend to run ROSVITA in **Google Chrome**, because in other internet browsers like Firefox the user experience will be limited due to the significantly slower browser performance.




.. |Ubuntu_link| raw:: html

   <a href="https://www.ubuntu.com/download/desktop" target="_blank">Ubuntu</a> 

.. |Docker_link| raw:: html

   <a href="https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce" target="_blank">Docker</a> 

.. |Docker_Hub_link| raw:: html

   <a href="https://hub.docker.com/explore/" target="_blank">Docker Hub</a> 

.. |getROSVITA_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/getROSVITA.sh" target="_blank">getROSVITA.sh</a> 

.. |start_script_link| raw:: html

   <a href="https://raw.githubusercontent.com/Xamla/docs.xamla.com/gh-pages/rosvita/downloads/rosvita_start.sh" target="_blank">rosvita_start.sh</a> 

.. |Chrome_link| raw:: html

   <a href="https://www.google.com/intl/en-CA/chrome/" target="_blank">Google Chrome</a> 

.. |Xamla_EarlyAccess_link| raw:: html

   <a href="http://xamla.com/en/#early-access" target="_blank">contact us</a> 

