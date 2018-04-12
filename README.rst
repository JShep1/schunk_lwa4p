BRIEF: Biological Robotic Imaging Experimentation Framework
##########################################################################

The Biological Robotic Imaging Experimentation Framework (BRIEF) is a `ROS <http://www.ros.org/>`_ based robot,
tailored to conduct biological imaging and manipulation experiments over extended periods of time.
BRIEF combines a `Schunk Lightweight Arm LWA 4P  <http://www.schunk-modular-robotics.com/en/home/products/powerball-lightweight-arm-lwa-4p.html>`_
with an imaging table and arm constructed in house at The George Washington University `SEAS <https://www.seas.gwu.edu/>`_
for a `Department of Computer Science <https://www.cs.seas.gwu.edu/>`_ Senior Design Project.

.. class:: no-web
..
    .. 	image:: https://github.com/gw-cs-sd/sd-2017-BRIEF/blob/master/brief.png
        :alt: BRIEF
        :width: 100%
        :align: center
    ..

.. class:: no-web no-pdf


.. contents::

.. section-numbering::



Main Features
=============

* Robotic Manipulation of Biological Samples
* Point Cloud Imaging
* Point Cloud to Mesh Conversion (future)

A Word of Caution
=================
ROS based projects are heavily dependent on specific computer hardware,
operating system, and ROS version. Although it is tempting to use a virtual machine,
we have found this creates difficulties. Therefore, in order to guarantee that
the installation process works properly we highly recommend that you use the
identical configuration of components we suggest in Installation. 

Installation
============

Hardware
--------
We suggest you use either a native Ubuntu 14.04 machine or run Ubuntu 14.04 on a VM of your choosing.  Both options are described below and either is acceptable for simulation development; however, most physical arm testing will need to be on a native machine.

Setting up a Native Environment
-------------------------------
Create a `Bootable USB <https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-ubuntu/>`_
with `Ubuntu 14.04.5 Desktop (64-bit)  <https://www.ubuntu.com/download/alternative-downloads>`_
and set up your new operating system on a native machine. Configure the
Ubuntu 14.04 Desktop to your specifications. We suggest Tools That Will Make Your Life Easier in this README.

Setting up a VM
---------------
VMWare is suggested for a VM if you choose to go this route.  First, download the `VMWare Workstation Player <https://www.vmware.com/products/workstation-player.html>`_.  Follow the guide to install your Ubuntu 14.04 VM on the Workstation Player `here <https://askubuntu.com/questions/142549/how-to-install-ubuntu-on-virtualbox>`_.


Setup Guide
===========
The following steps are documentation for how to setup the project ROS packages.

Setup ROS
---------

.. code-block:: bash

    $ sudo apt-get install git
    $ git clone https://github.com/gw-cs-sd/sd-18-roblox-let-s-play-part-1.git
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update
    $ sudo apt-get install ros-indigo-desktop-full
    $ sudo rosdep init
    $ rosdep update
    $ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    $ sudo apt-get install python-rosinstall

Create your Catkin Workspace
----------------------------

.. code-block:: bash

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws
    $ catkin_make
    $ source devel/setup.bash
    $ echo $ROS_PACKAGE_PATH should output something like: 
   /home/youruser/catkin_ws/src:/opt/ros/indigo/share

Download Dependencies for Driver and Workspace
----------------------------------------------

.. code-block:: bash

    $ wget ftp://anduin.linuxfromscratch.org/BLFS/popt/popt-1.16.tar.gz
    $ tar -xzf ftp://anduin.linuxfromscratch.org/BLFS/popt/popt-1.16.tar.gz
    $ cd popt-1.16/
    $ sudo ./configure --prefix=/usr --disable-static
    $ sudo make
    $ sudo make install
    $ sudo apt-get install ros-indigo-libpcan ros-indigo-ros-control ros-indigo-ros-controllers
    $ sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control gazebo2 
    $ sudo apt-get install libsdformat1 ros-indigo-gazebo-plugins ros-indigo-gazebo-ros
    $ sudo apt-get install ros-indigo-joystick-drivers 
    $ sudo apt-get install ros-indigo-trac-ik
    $ sudo apt-get install python-dev
    $ sudo apt-get install python3-dev
    $ rosdep install robot_state_publisher urdf xacro controller_manager geometry_msgs
    $ rosdep install ros_control ros_controllers sensor_msgs
    $ rosdep install sensor_msgs gazebo_msgs gazebo_plugins
    $ rosdep install gazebo_ros gazebo_ros_control

Download PCAN-USB Driver
------------------------

1. Go `here <https://www.peak-system.com/fileadmin/media/linux/index.htm>`_
2. Go to Driver Download and download the most recent tar.gz
    
.. code-block:: bash

    $ cd ~/Downloads
    $ tar -xzf peak-linux-driver-X.Y.Z.tar.gz
    $ cd peak-linux-driver-X.Y.Z
    $ make clean
    $ sudo make
    $ sudo make install

Install MoveIt Motion Planner
-----------------------------

.. code-block:: bash

    $ sudo apt-get install ros-indigo-moveit
    $ source /opt/ros/indigo/setup.bash

Test your setup
---------------
1. Run

.. code-block:: bash

    $ roscore

2. Open a new terminal and run

.. code-block:: bash

    $ roslaunch schunk_gripper_communication schunk_test.launch

3. An RViz simulator should appear with a visualization of the Schunk LWA4P

The Importance of Simulation
============================
As a physical robot arm has super human strength and is expensive, there is a huge importance to hundreds if not thousands of simulations before a physical trial is conducted.  The arm has the capability of breaking itself, breaking other objects, or breaking people so the utmost care and attention should be fed into a live, physical demo.

Simulation will reduce the risks of any collision that the robot arm could have when operating in the real world.  A proper and valid simulation will make sure that all is well within your program and motion plan.  The simulator we are currently using is RViz - ROS's own simulator.  Our choosing of this simulator is not to say that another cannot be used; in fact, another simulation could just as easily be used given they will take in the same .urdf and meshes that RViz uses.  We have just chosen to use RViz given it is native to ROS and provided quicker progress in the initial phases of the project. Future development may very well opt for another simulator such as Gazebo.



The Organization of ROS Packages and BRIEF
==========================================
The organization of ROS packages is well defined and has common structure. Please read and be knowledgeable of the information `here <http://wiki.ros.org/Packages>`_ as this information is crucial to developing within the current working ROS Package.  A rundown of the main project ROS package, schunk_gripper_communication, as well as a few support packages are as follows:

The schunk_gripper_communication package
----------------------------------------

urdf directory
~~~~~~~~~~~~~~
The urdf/ directory holds the necessary 'universal robotic descriptor files' needed to simulate the arm within RViz or any other desired simulator like gazebo.  If any modifications to the arm (addition of joints, environmental elements, etc.) in simulation need to be made, the arm.urdf.xacro is likely the file that needs to be modified.

If one wants to generate the actual arm.urdf file from the arm.urdf.xacro file, then a

.. code-block:: bash

    $ rosrun xacro xacro --inorder -o arm.urdf arm.urdf.xacro

call needs to be made.  

meshes directory
~~~~~~~~~~~~~~~~
The meshes/ directory is the one that holds the visualization data for the given arm or object that is going to be placed in simulation.

srv directory
~~~~~~~~~~~~~
The srv folder is ROS's simplified service description language.  The values above the dashed line in the file specify values that the client or user will provide to the server.  The values below the line specify values that the server will send back to the client.  The values within schunk_gripper.srv may need to be updated or changed based on the desired functionality and the necessary values that need to be passed between the established server and client services.  This communication will be described in the src/ directory description.

For more information on ROS srv, read `here <http://wiki.ros.org/srv>`_.

launch directory
~~~~~~~~~~~~~~~~
The launch directory holds the ROS launch files for the given package.  A launch file will launch multiple ROS `nodes <http://wiki.ros.org/Nodes>`_.  A launch file is the main way of starting up a functionality within ROS.  The command `roslaunch <http://wiki.ros.org/roslaunch>`_ is used.  Note the above testing command 

.. code-block:: bash

    $ roslaunch schunk_gripper_communication schunk_test.launch

uses roslaunch and then specifies the ROS package and then specifies the desired launch file.

include directory
~~~~~~~~~~~~~~~~~
As with any project, the include directory holds the library files that other files will include.  Nothing new here.

config directory
~~~~~~~~~~~~~~~~
The config directory should not necessarily be touched by a human unless she or he knows positively what she or he is doing.  These files were generated using MoveIt's Setup Assistant which pulls in the urdf of the robot to generate these files.  If new config's are needed, see `here <http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html>`_ for automatically generating new ones.

src directory
~~~~~~~~~~~~~
The src directory contains all of the cpp files from which the CMakeLists creates nodes. The main project code is within the three files schunk_gripper_server.cpp, schunk_gripper_client.cpp, and set_schunk.cpp.  

* schunk_gripper_server.cpp is the server portion of the code as it is so named.  This piece of code is ran when one runs the aforementioned schunk_test.launch file.  It constantly polls the client, asking for a function to run.

* schunk_gripper_client.cpp is the client portion of the code and simply parses a function argument passed in by the calling user and asks the server to perform this function if it is a valid one.  Currently, the client can ask the server to create_box, plan_motion, and execute_motion. A client call can be run using

.. code-block:: bash

    $ rosrun schunk_gripper_communication schunk_gripper_client *function-name-here*

Note the use of rosrun instead of roslaunch, given there is no launch file for the client node since it is only one node that needs to be run.

* set_schunk.cpp holds the functionality for the created Schunk Object.  The Schunk Object contains various basic functionalities:

- insert
- descriptions
- here

CMakeLists.txt and package.xml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
These files are what specify dependency packages for the whole system as well as link libraries to the necessary nodes.  It creates nodes from the specified code in src which a launch file can then initiate upon roslaunch'ing. The package.xml file specifies dependencies for the whole system as well and must include the dependency packages that the CMakeLists.txt file calls.

The fzi_icl_can and fzi_icl_core Packages
-----------------------------------------
These packages were obtained from FZI Forschungszentrum Informatik.  fzi_icl_can can be found `here <https://github.com/fzi-forschungszentrum-informatik/fzi_icl_can>`_, and fzi_icl_core can be found `here <https://github.com/fzi-forschungszentrum-informatik/fzi_icl_core>`_.  These packages are dependencies of the main project and do not really need to be studied for the context of this project.



Tools That Will Make Your Life Easier
=====================================
Vim

.. code-block:: bash

    $ sudo apt-get install vim



Useful Links
============
Ubuntu 14.04.5 Desktop (64-bit)

  https://www.ubuntu.com/download/alternative-downloads

ROS Indigo

  http://wiki.ros.org/indigo/Installation/Ubuntu

Catkin Tutorials

  http://wiki.ros.org/catkin/Tutorials

  http://wiki.ros.org/catkin/Tutorials/create_a_workspace

MoveIt!

  http://docs.ros.org/indigo/api/moveit_tutorials/html/index.html

  http://moveit.ros.org

Install Gazebo

  http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

Build a Ros Package

  http://wiki.ros.org/ROS/Tutorials/BuildingPackages
  http://wiki.ros.org/Packages


For a Gazebo Simulation

  http://gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros

  http://gazebosim.org/tutorials?tut=install&cat=install

  http://gazebosim.org/tutorials?tut=ros_wrapper_versions

Install gazebo via ROS

  http://gazebosim.org/tutorials?tut=ros_installing


License
============

BSD-3-Clause: `LICENSE <https://github.com/gw-cs-sd/sd-2017-BRIEF-Crandall/blob/master/LICENSE>`_.

Authors
============

Joseph Crandall and Karl Preisner created BRIEF for their
George Washington University Senior Design Project

John Shepherd and Liam Douglass have continued the project
for their Senior Design Project
