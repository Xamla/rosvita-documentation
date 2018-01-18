The ROSVITA Main View
**********************

.. figure:: images/Main_View.png

   Figure 1.1  The ROSVITA Main View.

The main view of the ROSVITA robot programming environment (see Fig. 1.1) consists of several parts:

1. A central start window ("multi document area") from which a new project can be created and a stored project can be opened. Later, this window will display either the configuration of the robot ("Configuration View"), the "Robot Jogging", the path planning ("World View"), the system monitoring ("Monitoring View"), an executable graph (.xgraph), or an open file (e.g. a .xacro). The details of these different views will be discussed later.
2. A header that displays the current project name and the status of the underlying ROS robot operating system.
3. Another header for each view. Depending on the selected view, there are buttons for compiling, saving, starting, stopping, selecting a kinematic chain ("Move Group") for path planning, etc.
4. A display of the loaded graphs including the corresponding process IDs.
5. A file browser with file search, path display, upload/download between own computer and ROSVITA server, etc.
6. And finally at the bottom several switchable panes with:

   * a display of possible error messages
   * a display of all outputs
   * a terminal (e.g. to execute scripts)
   * a display of the log outputs
   * a display of the heartbeat system monitoring
   * a ROS display

All parts of the main view, apart from the central multi document area, can be collapsed.
