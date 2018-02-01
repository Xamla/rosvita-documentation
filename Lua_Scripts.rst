*******************
Using Lua Scripts
*******************

In the following, two simple example LUA scripts are shown.
The first of these scripts simply prints the current joint positions and endeffector pose of a robot "MoveGroup" (kinematic chain)::

   local ros = require 'ros'
   local motionLibrary = require 'xamlamoveit.motionLibrary'

   local function main()
     ros.init("readPose")

     local nh = ros.NodeHandle()
     local motion_service = motionLibrary.MotionService(nh)
     local move_group = motion_service:getMoveGroup()  -- #If no name is specified first move group is used.

     local q = move_group:getCurrentJointValues()
     -- #Show joint names and corresponding joint values:
     print('Current joint positions:')
     local v = q.values
     local n = q:getNames()
     for i=1,v:size(1) do
       print(string.format('%s: %f', n[i], v[i]))
     end

     local p = move_group:getCurrentPose()
     print('Current pose (relative to world):')
     print(p:toTensor()) -- #Show pose as 4x4 tensor.

     motion_service:shutdown()
     ros.shutdown()
   end

   main()

.. note:: Elements of type "JointValues" (here: "q") consist of a vector of values ("q.values") for each joint and of the "xamlamoveit" datatype "JointSet", which contains the names of the joints. Hence, "JointValues" always needs a "JointSet", so it is quite clear which joints the vector of values refers to. 

Execution of the second script causes the robot (e.g. a UR5 arm) to move to a random joint angle position within joint limits::

   local ros = require "ros"
   local motionLibrary = require 'xamlamoveit.motionLibrary'
   local datatypes = require "xamlamoveit.datatypes"

   math.randomseed(os.time())

   local function sign(x)
     if x > 0 then return 1 elseif x < 0 then return -1 else return 0 end
   end

   function main()
     ros.init("simpleMovement")

     local nh = ros.NodeHandle()
     local motion_service = motionLibrary.MotionService(nh)
     local move_group = motion_service:getMoveGroup()
 
     local plan_parameters = move_group:getDefaultPlanParameters()
     local q = move_group:getCurrentJointValues()
     print("start:")
     print(q)

     -- #Get joint limits:
     local max_min_pos, max_vel, max_acc = motion_service:queryJointLimits(q:getNames())

     -- #Construct random target joint values within joint limits:
     local target_joint_values = torch.DoubleTensor(q.values:size(1))
     for i=1,q.values:size(1) do
       target_joint_values[i] = (max_min_pos[i][1]-max_min_pos[i][2]) * math.random() + max_min_pos[i][2]
     end
     local target = datatypes.JointValues(q.joint_set, target_joint_values)
     print("target:")
     print(target)

     move_group:moveJ(target)
     print("Movement finished.")

     motion_service:shutdown()
     ros.shutdown()
   end
   
   main()

Copy the contents shown above into files "``readPose.lua``" and "``simpleMovement.lua``" and place them into the subfolder "src" of your project folder.
Then execute the LUA scripts by typing the following commands in the ROSVITA terminal::

   cd /home/xamla/Rosvita.Control/projects/<project name>/src
   th readPose.lua
   th simpleMovement.lua

Now, the current joint angles and endeffector pose are printed in the terminal. Moreover, in the "World View" (e.g. opened in a second browser), the random robot movement can be observed.

.. note:: Before running the scripts, make sure that your current robot configuration has been compiled and ROS has  been started successfully (indicated by a green "GO" with check mark at the top bar of the ROSVITA environment).

Here we used some subpackages of the package "xamlamoveit". The package "xamlamoveit" can be found at ``/home/xamla/Rosvita.Control/lua/xamlamoveit``. In particular, we used some functions of the "MotionService" and "MoveGroup" classes, which are implemented in files "MotionService.lua" and "MoveGroup.lua", respectively, and can be found here: ``/home/xamla/Rosvita.Control/lua/xamlamoveit/motionLibrary/``.

Further helpful scripts can be found within the subfolders of the higher-level path ``/home/xamla/Rosvita.Control/lua/``, in particular ``cli/teachWaypoints.lua`` and ``cli/moveRobot.lua``, as well as ``auto_calibration/runCalibration.lua`` (the latter requires the addition of a camera to the robot configuration).


