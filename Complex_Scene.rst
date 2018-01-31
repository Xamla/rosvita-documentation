**************************
Building a Complex Scene
**************************

.. hint:: For a better understanding, it is recommended to first read the chapter :ref:`robot-config-label`.

If several robot parts and associated actuators shall be added to a scene (e.g. a "Yaskawa Motoman SDA10D" and the different parts of the "Weiss Robotics WSG-50 Gripper"), they must be linked together correctly.
This is done by clicking on the corresponding robot part in the list of added "RobotParts" on the left side of the "Configuration" view and choosing a "Parent link" in the "Properties" bar belonging to this robot part. (E.g. robot part: "wsg50 mount", parent link: "arm_right_link_tool0" of the Yaskawa Motoman SDA10D.)

Overall, the following robot parts and parent links are needed to e.g. connect the Weiss Robotics WSG-50 gripper to the right arm of the Yaskawa Motoman SDA10D:

* Yaskawa Motoman SDA10D; ``Parent link: world``
* wsg50 mount; ``Parent link: arm_right_link_tool0 (Yaskawa Motoman SDA10D)``
* Weiss Robotics WSG-50; ``Parent link: mount_tool0 (wsg50 mount)``
* Weiss Robotics WSG-50 w1 right finger; ``Parent link: gripper_right_tool0 (Weiss Robotics WSG-50)``
* Weiss Robotics WSG-50 w1 left finger; ``Parent link: gripper_left_tool0 (Weiss Robotics WSG-50)``

In addition, the following actuators are needed:

* Motoman SDA10D (Yaskawa)
* Gripper WSG-50 (Weiss Robotics)

After pressing the "Compile" button, the robot with gripper should appear in the 3D view of the "Configuration" window.

.. note:: If you want to use several **same robot parts** in the scene (for example, two WSG-50 grippers), they must be distinguished by "**Prefix**". Simply use the "Prefix" field in the "Properties" list for the corresponding robot part.

.. note:: All changes in files of your own project folder (under ``/home/xamla/Rosvita.Control/projects``) are saved locally to your computer (at ``/home/<username>/Rosvita/projects``). **Changes outside of your own project folder**, e.g. changes to a robot part in the library (``/home/xamla/Rosvita.Control/library``) **will not be saved permanently** and get lost when ROSVITA is stopped.


... to be continued

