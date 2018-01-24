*******************
Using Lua Scripts
*******************

In the following, a simple example script is shown, the execution of which causes the robot (e.g. a UR5 arm) to move to a specific joint angle position::

   local ros = require "ros"
   local moveit = require "moveit"
   local xutils = require "xamlamoveit.xutils"
   ros.init("simple_movement")
   local nh = ros.NodeHandle()
   local sp = ros.AsyncSpinner() -- background job
   sp:start()
   local components = require "xamlamoveit.components"
   local mc = require "xamlamoveit.motionLibrary".MotionService(nh)  -- motion client
   
   math.randomseed(os.time())

   local function sign(x)
     if x > 0 then return 1 elseif x < 0 then return -1 else return 0 end
   end

   function main()
     local move_group_names, move_group_details = mc:queryAvailableMovegroups()
     local move_group = move_group_names[1]
     local dim = #move_group_details[move_group].joint_names
     local current_joint_values = mc:queryJointState(move_group_details[move_group].joint_names)
     local plan_parameters = mc:getDefaultPlanParameters(move_group, move_group_details[move_group].joint_names)
   
     -- Get joint limits:
     local max_min_pos, max_vel, max_acc = mc:queryJointLimits(plan_parameters.joint_names)

     -- Construct random target joint values within joint limits:
     local target_joint_values = torch.DoubleTensor(dim)
     for i=1,dim do
       target_joint_values[i] = (max_min_pos[i][1]-max_min_pos[i][2]) * math.random() + max_min_pos[i][2]
     end
  
     local ok, joint_path = mc:planJointPath(current_joint_values, target_joint_values, plan_parameters)
     print("Ok?")
     print(ok)
     print("Joint path:")
     print(joint_path)
   
     local success, joint_trajectory = mc:planMoveJoint(joint_path, plan_parameters)
   
     if success == 1 then
       print("Moving ...")
       mc:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
       print("Movement successfully finished.")
     else
       ros.ERROR("Planning failed.")
     end
     
     sp:stop()
     ros.shutdown()
   end
   
   main()

Copy the content shown above into a file "``simple_movement.lua``" and place it into the subfolder "src" of your project folder.
Then execute the LUA script by typing the command ``th simple_movement.lua`` in the ROSVITA terminal.
Now, in the "World View" (e.g. opened in a second browser) the robot movement can be observed.

.. note:: Before running the script, make sure that your current robot configuration has been compiled and ROS has  been started successfully (indicated by a green "GO" with check mark at the top bar of the ROSVITA environment).

Here we used some subpackages of the package "xamlamoveit". The package "xamlamoveit" can be found at ``/home/xamla/Rosvita.Control/lua/xamlamoveit``. In particular, we used some functions of the "MotionService" class, which is implemented in the file "motionService.lua" and can be found here: ``/home/xamla/Rosvita.Control/lua/xamlamoveit/motionLibrary/motionService.lua``.

