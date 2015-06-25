
pioneer3at_ETSIDI: Pioneer 3 AT setup at ETSIDI-UPM
===================================================

This repo hosts ROS packages working on Indigo version. This packages are needed to setup a CATKIN workspace  and include all files needed for Pioneer 3 AT robot at ETSIDI-UPM university.

Submodules
----------------
Pioneer 3 AT uses the following additional ROS packages showed up as git submodules:

- [rosaria](https://github.com/amor-ros-pkg/rosaria): Interface with Aria library to control motors, battery and encoders. (See [rosaria docs](http://wiki.ros.org/ROSARIA))

- [LMS1xx](https://github.com/clearpathrobotics/LMS1xx): Sick ROS drivers from ClearPath Robotics to use Sick LMS100 ethernet laser scanner. (See [LMS1xx docs](http://wiki.ros.org/LMS1xx)).

- [freenect_stack](https://github.com/ros-drivers/freenect_stack): For Kinect 1 XBOX 360, (See [freenect_stack docs](http://wiki.ros.org/freenect_stack) and [freenect_launch docs](http://wiki.ros.org/freenect_launch)).

- [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan): Creates LaserScan data from depthimage devices such as kinect. (See [depthimage_to_laserscan docs](http://wiki.ros.org/depthimage_to_laserscan)).

- [turtlebot_apps](https://github.com/turtlebot/turtlebot_apps): Interactive implementations reused for P3AT robot such as "follower" demo. (See [turtlebot docs](http://wiki.ros.org/Robots/TurtleBot)).


Catkin workspace
----------------

All theese files and directories should be placed at src/ directory in a [catkin workspace](http://wiki.ros.org/catkin).
Follow steps in a terminal:

  ``$ catkin_init_workspace catkin_ws``
  
  ``$ cd catkin_ws/src``
  
  ``$ git clone --recursive https://github.com/danimtb/pioneer3at_ETSIDI.git .``	(NOTE THE . AT THE END OF THE LINE)

Content: pioneer_utils
---------------
This repo mainly adds some config specific parameters to keep all things working. They are stored at pioneer_utils.

- Odometry params calibrations used in rosaria.

- Laser IP address.

- Pioneer URDF model from p2os package.

- Navigation tweaks in costmaps, base and planners.

- depthimage to scan config.

And implements easy to use tiny nodes:

- Teleoperation node.

	  ``rosrun pioneer_utils teleop_p3at``

- Dead Reckoning node: Let robot move alone and making turns.

	  ``rosrun pioneer_utils moving_alone``

- nav-waypoints node: Send point goals to navigation stack.

	  ``rosrun pioneer_utils nav-waypoints``
    
Usage
-----

This repo is intended to be a simple method to setup and run everything needed to use Pioneer 3 AT.

After clonning this github repo in your catkin_ws src/ directory do the following:

``$ cd catkin_ws``

``$ catkin_make``

Now run "roscore" and the nodes needed with the .launch file you'll find in src/ directory.

In your terminal run "roscore":

  ``$ roscore``

In other terminal, we'll bring up all drivers for hardware using kinect, laser Sick, and Rosaria with calibration config setup:

  ``$ roslaunch pioneer_utils pioneer3at.launch``

Now, you can start navigation stack with amcl like this:

  ``$ roslaunch pioneer_utils navigation-pioneer3at.launch``
