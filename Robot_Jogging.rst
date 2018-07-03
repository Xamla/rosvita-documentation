****************
Robot Jogging
****************

In addition to automatic path planning, ROSVITA also offers the possibility of manual, **step-by-step control** of the robot movement (comparable to the robot control via **teach pendant**). Click on the blue-highlighted **Xamla** symbol and select the menu item **Robot Jogging** to switch to the view for manual, step-by-step control of the robot movement. The current robot configuration appears in a 3D view. Underneath the 3D view you find a display of the current **Tool Position**, which is updated in real-time as the robot moves. To the right of the 3D view is a bar where you can set the **Speed**, the **Move Group** to be controlled, the **End Effector** being moved, and the **Reference Frame** for the driving commands. To the right of this bar is a control panel with the buttons for the driving commands. By selecting the **Cartesian** robot control in the top bar of the **Robot Jogging** view and pressing the **START** button (also located at the top bar), 12 driving command buttons appear in the control panel, namely translation in +/- x, y, and z direction, as well as rotation around +/- x, y, and z axis of the reference coordinate system (see Fig. 5.1). The robot moves as soon as you click on one of the driving commands in the control panel. When the Cartesian robot jogging is selected, the interactive marker for moving the robot is also available.

.. figure:: images/Robot_jogging_cartesian.png

   Figure 5.1  Cartesian robot jogging.

Alternatively, instead of choosing the Cartesian robot control, one can also select the control of the individual joint angles (**Joints**) at the top bar and press **START**. The control panel will then show the joints of the selected **Move Group** along with the current joint angle for each joint and with left/right arrows next to each joint. Pressing one of these arrows will rotate the corresponding robot axis gradually around the choosen joint.

.. figure:: images/Robot_jogging_joints.png

   Figure 5.2  Robot jogging via joints.

.. hint:: **Robot Jogging** can also be performed from the **World View** by choosing the **Jogging** pane on the right side bar of this view.
