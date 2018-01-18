Using Lua scripts
******************

TODO: Translate this text to english!

Im Folgenden wird ein einfaches Beispielskript gezeigt, dessen Ausführung bewirkt, dass der Roboter (hier ein UR5 Arm) in eine bestimmte Gelenkwinkelstellung (die nicht die aktuelle Gelenkwinkelstellung ist) fährt. Dazu wird im Unterordner "``src``" des aktuellen Projektordners z.B. folgendes LUA-Skript mit Namen "``simple_movement.lua``" angelegt::

   local ros = require "ros"
   local moveit = require "moveit"
   local xutils = require "xamlamoveit.xutils"
   ros.init("simple_movement")
   local nh = ros.NodeHandle()
   local sp = ros.AsyncSpinner() -- background job
   sp:start()
   local components = require "xamlamoveit.components"
   local mc = require "xamlamoveit.motionLibrary".MotionService(nh)  -- motion client
   
   function main()
     local move_group_names, move_group_details = mc:queryAvailableMovegroups()
     local move_group = move_group_names[1]
     local current_joint_values = mc:queryJointState(move_group_details[move_group].joint_names)
     local plan_parameters = mc:getDefaultPlanParameters(move_group, move_group_details[move_group].joint_names)
   
     local target_joint_values_1 = torch.load("target_joint_values_1.t7")
     local target_joint_values_2 = torch.load("target_joint_values_2.t7")
     local target_values = target_joint_values_2
   
     local ok, joint_path = mc:planJointPath(current_joint_values, target_values, plan_parameters)
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
       ros.ERROR("Planning FAILD")
     end
     
     -- shutdown
     sp:stop()
     ros.shutdown()
   end
   
   main()

Im gleichen Ordner müssen zudem Zielwerte für die Gelenkwinkel ("target_joint_values_1.t7" und "target_joint_values_2.t7") hinterlegt sein. Der Aufruf dieses LUA-Skripts erfolgt dann im ROSVITA Terminal über den Befehl ``th simple_movement.lua``.
Im "World View" lässt sich die Roboterbewegung bei Ausführung des Skripts beobachten. 

Hier wurden einige Unterpakete des Pakets "xamlamoveit" verwendet. Dieses findet man unter ``/home/xamla/Rosvita.Control/lua/xamlamoveit``. Insbesondere die Funktionen der MotionService-Klasse (in ``/home/xamla/Rosvita.Control/lua/xamlamoveit/motionLibrary/motionService.lua``) wurden hier verwendet.

