^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package freenect_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2015-02-27)
------------------

0.4.0 (2015-01-30)
------------------
* Update header file to match libfreenect v0.5.1
* Contributors: Piyush Khandelwal

0.3.2 (2014-01-17)
------------------
* Add include for log4cxx/logger.h (`#10`_).

0.3.1 (2014-01-16)
------------------
* Add dependency on log4cxx (`#10`_).

0.3.0 (2013-08-19)
------------------
* Added dependency on pluginlib.

0.2.1 (2013-05-21 14:53:47 -0500)
---------------------------------
* bumping version to 0.2.1
* removed a bit of redundant code
* if the device times out (aka no streams are produced for 5 seconds after request), then the device streams are flushed again. Repeat flushing typically fixes the problem. This is an experimental fix. closes `#3 <https://github.com/ros-drivers/freenect_stack/issues/3>`_
* enabled flushing the device streams at the start, which typically fixes the issue of device streams not starting with a cold start. `#3 <https://github.com/ros-drivers/freenect_stack/issues/3>`_
* the driver no longer acquires the device for the camera motor as that is not used
* now install the nodelets file to the system package. closes `#5 <https://github.com/ros-drivers/freenect_stack/issues/5>`_
* fixing header references inside catkin packages for libfreenect (`ros-drivers-gbp/libfreenect-release#3 <https://github.com/ros-drivers-gbp/libfreenect-release/issues/3>`_)
* install node and nodelet
* add maintainer, url links
* clean up library references
* use catkin version of dynamic parameter generator
* partial catkin conversion of freenect_camera, still problems
* fixed a few bugs instroduced while creating diagnostics
* added diagnostics support to freenect_camera
* added a debug flag to set the libfreenect log level. might be useful for remote debugging
* changed header file location to refrain from patching libfreenect internally
* removed redundant device shutdown call (done by FreenectDriver already)
* reorganized stack for release

.. _`#10`: https://github.com/ros-drivers/freenect_stack/issues/10
