
*****************
Module Debugging
*****************

To be able to debug Rosvita modules, two main steps need to be done: 
First, allowing the debug slot in Rosvita and second running the GraphHost on the host machine.

Start Rosvita from the host machine with --debug option, this will login to an interactive docker session without Rosvita being automatically started:

.. code-block:: bash

   rosvita_start --debug

Now, inside the docker session edit the file appsettings.json:

.. code-block:: bash

   sudo vi /home/xamla/Rosvita.Control/Rosvita.Server/bin/appsettings.json

Add this line: 

.. code-block:: html

   "debugSlotId": "2999D35B-1347-45E7-9AB2-7C5AC62D91E1"

such that the file looks like:

.. code-block:: html

   {
     "RosvitaOptions": {
       "paths": {
         "projects": "../../projects",
         "presets": "../../library/presets/",
         "robotParts": "../../library/robot_parts/",
         "meshCacheDirectory": "$(TEMP)/xamla/rosvita/cache/meshes/",
         "credentialsFile": "../../data/credentials.json",
         "rosGardenerAssembly": "../../Rosvita.RosGardener/lib/RosGardener.dll",
         "graphHostAssembly": "../../Rosvita.GraphHost/lib/Rosvita.GraphHost.dll",
         "rosServicesAssembly": "../../Rosvita.RosServices/lib/Rosvita.RosServices.dll",
         "mruProjects": "../../data/projectsMru.json",
         "mruFiles": "../../data/filesMru.json",
         "configurationElements": "../../configurationElements"
       },
       "debugSlotId": "2999D35B-1347-45E7-9AB2-7C5AC62D91E1"
     },
     "Logging": {
       "IncludeScopes": false,
       "LogLevel": {
         "Default": "Debug",
         "System": "Information",
         "Microsoft": "Information"
       }
     }
   }

Start Rosvita, i.e. in the docker terminal enter:

.. code-block:: bash

   rosvita

Then, go to Rosvita (``localhost:5000``) with your web browser. You should find a **Debug Slot** available in your Xgraph slots.

The second step is to run the GraphHost in the host machine.

Thereto, open a terminal in Rosvita and archive the GraphHost binaries:

.. code-block:: bash

   cd /home/xamla/Rosvita.Control/Rosvita.GraphHost/lib
   tar -czvf graphhost.tar.gz .

Download the archived GraphHost binaries to the host machine with the Rosvita file browser on the left. More precisely, with the Rosvita file browser go into the folder ``/home/xamla/Rosvita.Control/Rosvita.GraphHost/lib``, choose the archived file and click the download button at the top bar of the file browser.

On your host machine extract the downloaded file and with the terminal go to the corresponding directory. 

Test if the GraphHost will run correctly by entering:

.. code-block:: bash

   ./Rosvita.GraphHost run --token S2999d35b134745e79ab27c5ac62d91e1@rosvita:d3bu9 --server ws://localhost:5000/xblk

In Rosvita click on the debug slot and add a new graph in the debug slot (the new button below the Debug Slot tab, not the one next to it). Try adding any new module you should be able to see the debug messages being printed.

This indicates that the debugging process is working correctly. Close **Rosvita.GraphHost** running in the host terminal to start the debugger from another IDE. Every time the debug process is restarted the debug slot has to be closed and reopened.

Try inserting a break point in your code with any module being created and start the debugging process. 

For example, use **vscode** for debugging:

Start the debugger from **vscode**:
Add a debug configuration to the module workspace and change **launch.json** fields as:

.. code-block:: html

   // "preLaunchTask": "build",
   "program": "/PATH_TO_GRAPH_HOST/Rosvita.GraphHost",
   "args": ["run", "--token", "S2999d35b134745e79ab27c5ac62d91e1@Arosvita:d3bu9", "--server", "ws://localhost:5000/xblk"],
   "cwd": "${workspaceFolder}/bin",

Change **PATH_TO_GRAPH_HOST** above with the path to the Rosvita.GraphHost

Go to the debug windows, run and choose the .net core debugger. 
Close and start a new "Debug slot". The debugged module should now be available to load from the graph. 
Now loading modules and inserting break points should be working correctly.
