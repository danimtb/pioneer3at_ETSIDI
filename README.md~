
pioneer3at_ETSIDI: Pioneer 3 AT setup at ETSIDI-UPM
===================================================

This repo hosts ROS packages needed to setup a CATKIN workspace to setup all files needed for Pioneer 3 AT robot at ETSIDI-UPM university.

Catkin workspace
----------------

All theese files and directories should be placed at src/ directory in a catkin workspace.

  ``$ catkin_init_workspace catkin_ws``
  
  ``$ cd cd catkin_ws/src``
  
  ``$ git clone https://github.com/danimtb/pioneer3at_ETSIDI.git``

Content
-------

Following packages are used:

- rosaria: To connect to pionner 3at robot with motors (odometry)

    ``$ rosrun rosaria RosAria``

- lms1xx: To connect to Laser SICK LMS100

    ``$ rosrun lms1xx LMS1xx_node``
    
- teleop_pioneer: To teleoperate pioneer 3at with key arrows

    ``$ rosrun teleop_pioneer teleop_p3at``
    
Usage
-----

This repo is intended to be a simple method to setup and run everything needed to use Pioneer 3 AT.

After clonning this github repo in your catkin_ws src/ directory do the following:

``$ cd catkin_ws``

``$ catkin_make``

Now run "roscore" and the nodes needed with the .launch file you'll find in src/ directory.

In your terminal run "roscore":

  ``$ roscore``

In other terminal:

  ``$ cd src``
  
  ``$ roslaunch pioneer3at.launch``
