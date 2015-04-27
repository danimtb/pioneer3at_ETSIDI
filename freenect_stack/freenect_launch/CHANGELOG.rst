^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package freenect_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2015-02-27)
------------------
* fix tf_prefix leading slash issue `#13 <https://github.com/ros-drivers/freenect_stack/issues/13>`_
* Contributors: Jihoon Lee, Piyush Khandelwal

0.4.0 (2015-01-30)
------------------

0.3.2 (2014-01-17)
------------------

0.3.1 (2014-01-16)
------------------

0.3.0 (2013-08-19)
------------------
* fixed cmake and package.xml for rgbd_launch migration
* migration of common rgbd_launch is now complete
* added all old launch files with upgrade notices pointing to rgbd_launch
* marked all internal files as package.xml
* fixed cloud generation with device enabled depth registration
* Added data_skip argument to launch files.

0.2.1 (2013-05-21 14:53:47 -0500)
---------------------------------
* bumping version to 0.2.1
* convert freenect_launch to catkin (`#2 <https://github.com/ros-drivers/freenect_stack/issues/2>`_)
* improved the ns/tf_prefix distinction, allowing you to set both in freenect.launch. also fixed some launch file typos. closes `#8 <https://github.com/ros-drivers/freenect_stack/issues/8>`_
* fixed small typo in manifest
* added more examples - also good for testing
* fixed namespacing in freenect_launch
* added debugging/diagnostics capabilities to main launch file
* added ability to have a top level namespace in the launch file. NOTE: kinect_frames.launch is not fixed
* greatly simplified launch examples by allowing freenect.launch to take the processing constellation launch file name as an argument
* reorganized stack for release
