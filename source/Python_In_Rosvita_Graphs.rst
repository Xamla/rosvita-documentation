.. _python_in_graph_label:

*************************
Python in ROSVITA Graphs
*************************

.. hint:: To make this tutorial an interactive one please download the examples: `Link </examples/python_in_rosvita_graph_tutorial.tar>`_. After that, untar it in your ROSVITA projects folder and open the project in ROSVITA. All graphs and python files which are named in this tutorial can be found in this project.

In this tutorial we try to explain some inner workings of python and the ROSVITA Graph
itself to motivate some patterns how to use python and rospy in ROSVITA Graphs.

First we start with one of the most important aspects of python in ROSVITA Graphs and this is how to use python scripts in them. 
The answer is pretty simple, add a python script to your ROSVITA project. 
Then open a ROSVITA Graph instance and drag and drop the script into it. 
Then just click on Python3.PythonScriptFile and you ready to go.

If the Python Module within the Graph instance is highlighted in red a problem while loading
the python module occurs. This could have several reasons, imports of specific modules are not possible, definition of the method which should be called in the graph is not correct or
just syntactic errors. To get more information just click on th module and read the parserError in the properties window.

If an import was not possible make sure the necessary library is installed or all necessary file are available in the project. 
We recommend to install all available python libraries with help of `pip <https://docs.python.org/3/installing/index.html>`_. 
If you have created some python files which should be imported, we recommend to place them under lib/python/ in a ROSVITA project. 
This is possible due to the properties of the python runtime instance which runs in a ROSVITA Graph. 
The python instance in a ROSVITA Graph includes the complete project file system to its PYTHONPATH, therefore all files in it can be used and imported in the ROSVITA Graph Python Modules. 
To do so use the path from Project root to file to import it e.g.

.. code-block:: Python

  import lib.python.example_lib

Properties of python import and how to use them
------------------------------------------------

Now we know how to use python in ROSVITA Graphs in generell. Next we will explain the inner workings of `imports <https://docs.python.org/3/reference/import.html>`_ in python and how we can use same in a ROSVITA Graph.

In python every file is a module and a module can be interpreted as a singleton. Therefore, if you for example create an instance of an object in a file and assign it to a variable which lives in the module namespace this will only be done once if you import this file.

Short example:

Library File (e.g. lib/python/my_lib.py):

.. code-block:: Python

  class MyClass(object):
      def __int__(self, name):
          self.name = name

  my_class_instance = MyClass('an instance of MyClass')

Run File:

.. code-block:: Python

  from lib.python.my_lib import my_class_instance

  def main() -> str:

      return 'my_class_instance object memory adress {}'.format(id(my_class_instance))

With help of this property you can create objects which lives longer then a single run of the ROSVITA Graph and also can be shared between multiple python modules in a ROSVITA Graph.
Python objects which are made available in this manner lives as long as the python runtime instance lives. In context of a ROSVITA Graph this means as long as the Graph is open and not distroyed with help of the trash bin button this kind of objects are available.

But wait, many software developer will argue that this kind of using python modules to solve our problem of sharing objects is not a very clean approach and that is correct. Therefore, to do it in a more sophisticated and cleaner why please use the `borg pattern <https://www.oreilly.com/library/view/python-cookbook/0596001673/ch05s23.html>`_.

As an example we provide the files showcase_graph_instance_and_python_object.py the library
example_lib.py (lib/python/) and the Graph showcase_graph_instance_and_python_object.xgraph
If you run the Graph you can see the explained inner python and ROSVITA Graph workings in action.

ROSVITA Graph Python Modules (call, input, output)
--------------------------------------------------

So far we only talk about python itself and how to import python modules in a ROSVITA Graph context. This chapter now explains how to transfer data into and out of a ROSVITA Graph Python Module and what pitfalls perhaps exists.

What a ROSVITA Graph Python Module does is calling a specific function of a python file. If you click on the PythonScriptFile Module in a ROSVITA Graph and take a look at the
properties the properties of interest in a first glance are path (path to the python file which should be used) and functionName (name of the function which should be called).

If you add a PythonScriptFile to the Graph via drag and drop the path is already known. But perhaps the function name is not correct. As default main is used. Therefore, if this is not correct change it to the correct function name.

Perhaps the Module is still highlighted in red. As mentioned a reason for that could be that the function which should be called is not correctly defined to use it in a ROSVITA Graph Python Module even if it is syntactically correct python code.

The solution is to add `type hints <https://docs.python.org/3/library/typing.html>`_ to the python function which should be called by the ROSVITA Graph Module. Why, is perhaps now the
your question. The answer is, because the ROSVITA Graph Runtime needs to know how to convert
the python types into the inner ROSVITA Graph types. The problem is that python is a dynamic language and every variable could be any type. Therefore, we must help the ROSVITA Graph runtime with the type hints. Thus, please provided full type hints for all inputs and outputs
of the python function which should be called. As an example please take a look into the already known file showcase_graph_instance_and_python_object.py.

One of the major pitfalls in the interaction of Python Modules and the Graph is the mentioned type conversion. For example it is not possible to return a custom python type,
because it is not know by the ROSVITA Graph Runtime. Currently the following data types are supported:

* all standard python data types (int, float, dict, list, tuple, str)
* numpy ndarray
* all data types from xamla_motion.data_types (e.g. Pose, JointSet, JointValues, ...)

ROS (Python) in ROSVITA Graphs
------------------------------

Directly interacting with ROS in ROSVITA Graph Python Modules and in ROSVITA Graphs in general is only necessary if you try to call services, actions or subscribe to topics which are not already supported in a way by ROSVITA itself. For example to interact with the robot or the worldview you can use xamla_motion. To interact with the cameras xamla_vision is provided and so on.

If you want to provide a service, action or publish a topic python in the graph context is
not the right place for it. The correct solution is for this kind of purpose to create a
ROS Node which can be launched by adding it to the custom launch file of your ROSVITA Project.

But back to how to use ROS / rospy in ROSVITA Graph Python Modules. So, if you need to call ROS service, action or subscribe to topics and ROSVITA not provides a way to do so please take in mind following things:

As already mentioned in the previous chapters the python runtime lives as long as the Graph
instance lives. Therefore, if you once initialize rospy in a newly opened ROSVITA Graph it is not necessary to do it again. But you may ask now how to do so when e.g. your python code runs in a Graph for loop or you run the same graph a second time. The answer is check if python is already initialized and only initialize it when this is not the case.
For this purpose use the following python snipped in your implementations:

.. code-block:: Python

  import rospy
  import re

  if (re.sub('[^A-Za-z0-9]+', '', rospy.get_name()) == 'unnamed'):
      rospy.init_node('my_node', anonymous=True)

For a running example please take a look into the showcase_rospy_in_rosvita_graph.py
and run the graph showcase_rospy_in_rosvita_graph.xgraph

ROS packages and usage in ROSVITA Graphs via Python
----------------------------------------------------------

Every ROSVITA project has it own already initialized `catkin <http://wiki.ros.org/catkin/workspaces>`_.
This workspace is the place where all project specific ROS packages should be places which are not
already available by the ROS installation which is the backbone of ROSVITA.

To perform actions in the project catkin workspace please use `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_
which are already installed and can be used from terminal when you browse to the project catkin workspace.

After building the catkin workspace with help of the command **catkin build** the package content should be ready to use in ROSVITA Graphs.
If you want access to those packages from the terminal please first source the catkin workspace (source devel/setup.bash --extend in catkin workspace).

.. hint:: If you get erros which claim that a ROS package is not available e.g. a ROS message of a package, please consider first to clean the catkin workspace with the command **catkin clean** and then rebuild the catkin workspace. Reopen Graphs or resource the catkin workspace in terminals where this kind of error were stated. 

Also please take in mind that a ROSVITA Graph is not a good place for long running or always on task. In practise this means that e.g. subscribing to a ROS topic and handle these messages for 
a long time is not a good graph application. The idea is that heavy and long running computations are implemented in ROS packages and you can access and collect the results via ROS services or
ROS actions. 

To start your custom ROS packages every ROSVITA project provides a custom launch file. An introduction on how to use custom launch files can be found :doc:`here <ROSVITA_and_roslaunch>`. 