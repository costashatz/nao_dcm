#nao_dcm
 
##ROS Stack for Aldebaran's Nao Humanoid
 
  - Goal is to **connect to the machine of Nao** and not the API provided by Aldebaran.
  - Ιt makes use only of the **DCM and Memory Proxies**.
  - Supports **V4.0 robots (H21 and H25 Body Types)**.
  - **Written purely in C++** and uses the *C++ SDK (1.14.5)*.


Version
----

0.0.1 (Beta)

Requirements
-----------

nao_dcm requires several packages to be installed in order to work properly:

* [ROS] - ROS **Hydro**
* [NaoQi C++ SDK] - **Version 1.14.5**
* [ROS MoveIt!] - Used for motion planning
* [ROS Control] - **Version >=0.6.0**
* [Webots for Nao] - The best? simulator so far [optional]
* [Gazebo] - Work in progress but with satisfactory results (**Version >= 2.2.2 alongside gazebo-ros-pkgs >= 2.3.4 and [roboticsgroup_gazebo_plugins]**) [optional]
* [Nao Robot] - A real working Nao is the best "simulator" you'll ever get!! **Version >= V4.0 and flashed OpenNao OS >= 1.14.5**

Basic Usage
--------------

###Bringup nao_dcm driver (remotely)
```sh
roslaunch nao_dcm_bringup nao_dcm_{BodyType}_remote.launch
```

{BodyType} is either **H21** or **H25**.

This will connect to Nao Robot, start the cameras and open up a simple dashboard for monitoring and basic control over Nao.

###MoveIt Demo
```sh
roslaunch nao_dcm_moveit_config moveit_planner_{BodyType}.launch
```

{BodyType} is either **H21** or **H25**.

*This requires that you have connected to the Robot/Simulation via nao_dcm driver* and will open up **rviz with the Navigation plugin configured** to give Nao trajectories to execute.

###Webots Simulation

Launch your **Webots for Nao Simulator** and then **follow the instructions above to bringup nao_dcm driver** (remotely). *On some versions of Webots, it is required that you move the head before you get camera feedback. Also, changing the camera resolution via dynamic reconfigure will not have any effect as Webots uses fixed resolution.*

###Gazebo Simulation
```sh
roslaunch nao_dcm_gazebo nao_dcm_gazebo_{BodyType}.launch
```

{BodyType} is either **H21** or **H25**.

**NOTE: **This is work in progress and may not work very well.

This will **launch Gazebo Simulator** and **trajectory controllers** to simulate the real robot. *This is somewhat equivalent as bringing up nao_dcm_driver (remotely)*. Thus, then for example you can use MoveIt Demo to give trajectories to Nao to execute.

Basic Options/Guidelines
--------------

###Gazebo arguments/options

All properties are located in the [gazebo launch] files (in each body type).

* **use_pid** - true/false => Choose whether to use pid control in motors or not. Currently, usage without PIDs is highly unstable. Also, PID values REALLY need tuning. So, please contribute to that direction.

###URDF/XACRO properties

All properties are located in [robot.xacro] file.

* **nao_meshes** - true/false => Choose whether to use nao_meshes 3D Objects or not
* **use_helmet** - true/false => Choose whether to mount on Nao the Helmet (along with 2 cameras)
* **helmet_cameras_viz** - true/false => Choose whether to visualize the cameras on Helmet
* **use_odroid** - true/false => Choose whether to mount Odroid Bag on Nao
* **use_hector_imu** - true/false => Choose whether to use hector_imu_plugin or the default by Gazebo for IMU simulation

###nao_meshes Integration

Download [nao_meshes] repo, generate the meshes (it creates .obj for visual and .ply for collisions) and convert them to collada (.dae) in the same folders with Blender or Meshlab (do not change any orientation [visual components are rotated] - export only the parts and not the whole scene). No \*\_dcm\_\* is needed for the nao\_meshes. For quick results you can always fall-back to boxes and cylinders (simple change to false the xacro property {nao_meshes} at this file). All packages work the same with or without nao_meshes.

Notes/Limitations
-----------------
* **nao_dcm can be run locally**, *but you need to have built and installed ROS Hydro on Nao as nao_dcm requires ros_controls packages* (available only on >= Hydro). Yet, running **nao_dcm** driver remotely is identical in performance as running it locally (if the connection is strong and persistent - Wi-Fi usage is not recommended).
* Tutorials will become available as soon as possible.
* Integration with [nao_meshes] for better visual feedback. You can disable it if you want a rough 3D model made of boxes and cylinders. **NOTE**: *You need to convert the .obj and the .ply 3D objects to the collada format (in the same folder).*
* Although my intension is to provide ROS integration to the machine, basic gait and motion planning schemes are on the way for those that want quick results/feedback.
* *Integration for LED, IR and Audio hardware is not available and is not on my agenda*. So, **feel free to contribute in that direction**.
* **This is only ONE part of my Diploma Thesis** (*"Navigation of Humanoid Robots in Unknown Space With Dynamic Obstacles"*), so many parts of the code are not perfect and well-thought. So, **PRs for code improvement are welcomed.**

Origin of the Name
------------------

Since my intention is to use the lowest level of Aldebaran's API possible and its name is DCM, I decided to name the ROS Stack nao_dcm.

License
----

BSD


Copyright (c) 2014, **Konstantinos Chatzilygeroudis**

[ros]: http://www.ros.org
[naoqi c++ sdk]: https://community.aldebaran-robotics.com/doc/1-14/index.html
[webots for nao]: https://community.aldebaran-robotics.com/doc/1-14/software/webots/webots_index.html
[gazebo]: http://gazebosim.org/
[ros moveit!]: http://moveit.ros.org/
[nao robot]: http://www.aldebaran.com/en/humanoid-robot/nao-robot
[nao_meshes]: https://github.com/vrabaud/nao_meshes
[ros control]: http://wiki.ros.org/ros_control
[roboticsgroup_gazebo_plugins]: http://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
[robot.xacro]: https://github.com/costashatz/nao_dcm/blob/master/nao_dcm_common/nao_dcm_description/urdf/modules/robot.xacro
[gazebo launch]: https://github.com/costashatz/nao_dcm/tree/master/nao_dcm_apps/nao_dcm_gazebo/launch
