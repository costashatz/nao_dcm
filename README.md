#nao_dcm
 
##ROS Stack for Aldebaran's Nao Humanoid
 
  - Goal is to **connect to the machine of Nao** and not the API provided by Aldebaran.
  - Ιt makes use only of the **DCM and Memory Proxies**.
  - Supports **V4.0/V5.0 robots (H21 and H25 Body Types)**.
  - **Written purely in C++** and uses the lastest *C++ SDK (2.1.2.17)*.
  - **Coupled with official packages**


Version
----

0.0.1 (Beta)

Requirements
-----------

nao_dcm requires several packages to be installed in order to work properly:

* [ROS] - ROS **Hydro**
* [NaoQi C++ SDK] - **Version 2.1.2.17** - Should work on *1.14.5* too.
* [ROS MoveIt!] - Used for motion planning
* [ROS Control] - **Version >=0.6.0**
* [Webots for Nao] - The best? simulator so far [optional]
* [Gazebo] - Work in progress but with satisfactory results (**Version >= 2.2.2 alongside gazebo-ros-pkgs >= 2.3.4 and [roboticsgroup_gazebo_plugins]**) [optional]
* [Nao Robot] - A real working Nao is the best "simulator" you'll ever get!! **Version >= V4.0 and flashed OpenNao OS >= 2.1.2.17**

Basic Usage
--------------

###Bringup nao_dcm driver
```sh
roslaunch nao_dcm_bringup nao_dcm_bringup.launch
```

This will connect to Nao Robot and provide basic control over Nao.

###Webots Simulation

Launch your **Webots for Nao Simulator** and then **follow the instructions above to bringup nao_dcm driver** (remotely). *On some versions of Webots, it is required that you move the head before you get camera feedback.*

Notes/Limitations
-----------------
* **nao_dcm can be run locally**, *but you need to have built and installed ROS Hydro/Indigo on Nao as nao_dcm requires ros_controls packages* (available only on >= Hydro). Yet, running **nao_dcm** driver remotely is identical in performance as running it locally (if the connection is strong and persistent - Wi-Fi usage is not recommended).
* Tutorials will become available as soon as possible.
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
[naoqi c++ sdk]: https://community.aldebaran-robotics.com/doc/2-1/index.html
[webots for nao]: https://community.aldebaran-robotics.com/doc/2-1/software/webots/webots_index.html
[gazebo]: http://gazebosim.org/
[ros moveit!]: http://moveit.ros.org/
[nao robot]: http://www.aldebaran.com/en/humanoid-robot/nao-robot
[ros control]: http://wiki.ros.org/ros_control
[roboticsgroup_gazebo_plugins]: http://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
