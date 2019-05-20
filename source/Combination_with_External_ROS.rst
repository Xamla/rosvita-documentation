************************************
Access to ROSVITA from External ROS
************************************

In the following we will describe the access to ROSVITA from an external ROS development environment outside the Docker container. 

Assume a robot (e.g. a UR5 arm) has been started with ROSVITA on a master machine ("computer1") via web interface.
So on the master machine, the steps from the first four tutorials were carried out, i.e.:

* Download of the ROSVITA docker image
* Running the ROSVITA start script
* Start of a web browser and entering ``localhost:5000`` in the address bar
* Signing in to ROSVITA at the start screen
* Creation of a simple robot configuration consisting of e.g. only the UR5 arm (RobotPart: UR5, Actuator: Robot Arm UR) (see chapter 4)
* Compiling the robot configuration
* Starting ROS

Now, we want to access the roscore process running in the Docker container on the master machine from another computer ("computer2") with own ROS development environment (see also the ROS tutorial: `"Running ROS across multiple machines" <http://wiki.ros.org/ROS/Tutorials/MultipleMachines>`_).
This requires **ssh access** from computer2 to computer1 (and vice versa).

Suppose on computer2 we have our own ROS environment, i.e both a running ROS installation and a catkin workspace with various packages, in particular:

* a checked out `"universal_robot" <https://github.com/ros-industrial/universal_robot>`_ package,
* and in there a "lesson_move_group" package with a CMakeLists.txt::

   cmake_minimum_required(VERSION 2.8.3)
   project(lesson_move_group)

   add_compile_options(-std=c++11)

   find_package(catkin REQUIRED COMPONENTS
     moveit_ros_planning_interface
     roscpp
   )

   include_directories(
     ${catkin_INCLUDE_DIRS}
   )

   add_executable(lesson_move_group_1 src/lesson_move_group_1.cpp)
   target_link_libraries(lesson_move_group_1 ${catkin_LIBRARIES})

 and a package.xml::

   <?xml version="1.0"?>
   <package>
     <name>lesson_move_group</name>
     <version>0.0.0</version>
     <description>The lesson_move_group package</description>
     <maintainer email="altrogge@todo.todo">altrogge</maintainer>
     <license>TODO</license>
     <buildtool_depend>catkin</buildtool_depend>
     <build_depend>moveit_ros_planning_interface</build_depend>
     <build_depend>roscpp</build_depend>
     <run_depend>moveit_ros_planning_interface</run_depend>
     <run_depend>roscpp</run_depend>
   </package>

 as well as a source folder "src" with a small C++ program "lesson_move_group_1.cpp" 
 (see this `"ROS industrial exercise 3.6" <http://aeswiki.datasys.swri.edu/rositraining/Exercises/3.6>`_ 
 and this `"ROS industrial exercise 4.0" <http://ros-industrial.github.io/industrial_training/_source/session4/Motion-Planning-CPP.html>`_)::

   #include <ros/ros.h>
   #include <moveit/move_group_interface/move_group_interface.h>
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "lesson_move_group");
     ros::AsyncSpinner spinner(1);
     spinner.start();

     moveit::planning_interface::MoveGroupInterface group("urcontroller"); -- #in ROSVITA the UR5 move group is called "urcontroller" instead of "manipulater"

     group.setRandomTarget();
     group.move();
   }


To start this C++ script from computer2 so that it accesses the roscore process running in Docker on computer1, the following steps must be performed on computer2:

1. Set the ROS environment variable **"ROS_MASTER_URI"** to the IP address of the master machine (computer1)::

      export ROS_MASTER_URI=http://<ip-address-of-master>:11311

   (You can find out the IP address of the master machine by login to computer1 via ssh and typing ``ifconfig``.)
   Check the new ROS_MASTER_URI setting on computer2 by typing commands ``echo $ROS_MASTER_URI`` and ``rostopic list``, which should list all processes/nodes, that have been started in ROSVITA on computer1.

2. Source your catkin workspace setup file, if not already done automatically (via .bashrc)::

      source ~/catkin_ws/devel/setup.bash

   Now, the **"ROS_PACKAGE_PATH"** variable should not only point to your ROS installation, but also to your catkin workspace. Check this by looking at this variable: ``echo $ROS_PACKAGE_PATH``.

3. **Download folder "part_ur5"** from your current robot configuration in ROSVITA and **add it to the ROS_PACKAGE_PATH**:

   * Open a web browser (e.g. Chrome) and type ``http://<ip-address-of-master>:5000`` in the address bar. The ROSVITA development environment should appear.
   * Log in to ROSVITA and with the ROSVITA terminal go into your project folder containing the current robot configuration: 
     ``cd /home/xamla/Rosvita.Control/projects/my_project/robotModel/``.
     Then pack and compress the folder "part_ur5": 
     ``tar cvfz part_ur5.tgz part_ur5``
     and finally download it to your computer, i.e. click on "part_ur5.tgz" 
     and press the download button of the ROSVITA file browser.
   * Now, on your local computer (computer2) go into your Downloads folder and unpack the part_ur5 folder: ``tar -xzvf part_ur5.tgz``
   * Add the path of the "part_ur5" folder to your ROS_PACKAGE_PATH::

      export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/<username>/Downloads/part_ur5/

     Check this setting by typing ``echo $ROS_PACKAGE_PATH`` and ``rospack find part_ur5`` in your terminal.

4. Finally, start your C++ script by typing the following command in a terminal on computer2:: 

      rosrun lesson_move_group lesson_move_group_1
   
   Now, you should see a movement of your UR5 robot arm in the ROSVITA environment, which previously has been started in Docker on computer1 (see beginning of this chapter).

