^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ssc_joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2004.0.0 (2021-10-27)
---------------------
* Use joy_linux instead of joy due to diagnostics support (`#48 <https://github.com/astuff/ssc_joystick/issues/48>`_)
* Port to ROS2 (`#46 <https://github.com/astuff/ssc_joystick/issues/46>`_)
* Contributors: icolwell-as

4.0.0 (2021-10-15)
------------------
* Change DEBUG to INFO to restore previous behaviour (`#43 <https://github.com/astuff/ssc_joystick/issues/43>`_)
* Update to Noetic CI (`#41 <https://github.com/astuff/ssc_joystick/issues/41>`_)
* Break up joystick callback into separate functions (`#39 <https://github.com/astuff/ssc_joystick/issues/39>`_)
* Update README and config params (`#37 <https://github.com/astuff/ssc_joystick/issues/37>`_)
* Reorder the inclusion of header files in source files (`#36 <https://github.com/astuff/ssc_joystick/issues/36>`_)
* Remove hexagon tractor vehicle specific code from the master branch (`#32 <https://github.com/astuff/ssc_joystick/issues/32>`_)
* Add missing module_states callback  (`#31 <https://github.com/astuff/ssc_joystick/issues/31>`_)
* Convert the code using ros nodelet (`#28 <https://github.com/astuff/ssc_joystick/issues/28>`_)
* Update phrasing on disengage and override. `#26 <https://github.com/astuff/ssc_joystick/issues/26>`_ from astuff/fix/readme_update
* Add controller image, update button defs. `#25 <https://github.com/astuff/ssc_joystick/issues/25>`_ from astuff/fix/readme_update
* Feat/quick brake test `#24 <https://github.com/astuff/ssc_joystick/issues/24>`_ from astuff/feat/quick_brake_test
* Contributors: Daniel-Stanek, Jilin Zhou, Sneha Ganesh, icolwell-as, jilinzhouas

3.0.2 (2020-10-28)
------------------
* Added Hexagon Tractor - based on https://github.com/astuff/astuff_sensor_msgs #edb3bf1 and https://github.com/astuff/pacmod3 #1425e64.
* README updated - individual module enable support unavailble on all platforms, vehicle_platform field and tractor params.
* Contributors: Sneha Ganesh

3.0.1 (2020-03-02)
------------------
* Added feature - option to enable speed/steering modules individually.
* Changed logic for the gear selection for faults and enabling
* Start using velocity_accel_cov instead of velocity_accel, since it's no longer supported
* Contributors: Daniel Stanek, Joshua Whitley, Sneha Ganesh

3.0.0 (2019-05-07)
------------------
* Merge pull request `#11 <https://github.com/astuff/ssc_joystick/issues/11>`_ from astuff/maint/adding-config-params
* added config params and button descriptions
* Fixing _sim launch file.
* Merge pull request `#10 <https://github.com/astuff/ssc_joystick/issues/10>`_ from astuff/maint/rename_to_ssc_joystick
* Make json file an arg
* Rename to ssc_joystick and update readme
* Merge pull request `#9 <https://github.com/astuff/ssc_joystick/issues/9>`_ from astuff/maint/roslint
* Adding roslint and implementing suggestions.
* Contributors: Daniel Stanek, Joshua Whitley, Sam Rustan, Zach Oakes

2.0.0 (2018-12-03)
------------------
* Merge pull request `#7 <https://github.com/astuff/joystick_vehicle_test/issues/7>`_ from astuff/feature/gazebo-sim
* added dbw simulated CAN, removed the dataspeed joystick node
  Tested by launching the single launch file.
  Noticed some odd behavior initially on the terminal output, but
  SSC/joystick_vehicle_test works as expected.
* Merge pull request `#6 <https://github.com/astuff/joystick_vehicle_test/issues/6>`_ from astuff/bug/reduce_joystick_timeouts
* Other diagnostic messsages were swamping out the ones from the joy node, so it was timing out
* Merge pull request `#3 <https://github.com/astuff/joystick_vehicle_test/issues/3>`_ from astuff/maint/ros_release_changes
* Merge branch 'fix/cmake_error' into maint/ros_release_changes
* Changes to messages in automotive_autonomy_msgs.
* Since radar_msgs moved, need to add another depend.
* Fixing CMAKE error with catkin_make.
* Re-releasing under MIT license.
* Merge branch 'master' of https://bitbucket.org/autonomoustuff/joystick_vehicle_test
* Updating messages to reflect changes to platform_automation_msgs.
* Remove dataspeed message dependency since platform_comm_msgs now supports cruise control buttons on the steering wheel
* Reflecting changes to platform_automation_msgs.
* Changes to match new naming in core_veh_controller.
* Set gear to neutral on faults
* Add reverse functionality
* Update readme
* Bug fixes and tuning
* Add buttons to increment/decrement curvature
* Was engaging/disengaging repeatly while both joystick buttons were pressed. Fixed.
* two button enable, brake pedal, engage only in park or neutral, must be stopped to go to park
* Engage/disengage with steering wheel buttons, add vehicle control monitoring
* Adjust gains
* Change desired speed to mph, add exponent factor to steering to provide some modulation, add readme file
* Contributors: AStuff software team, Daniel Stanek, Joshua Whitley, Lucas Buckland, Sam Rustan, Zach Oakes

1.0.0 (2017-05-02)
------------------
* Adding install rule for json file and removing Fusion-specific includes in launch.
* Adding install rules.
* Initial version
* Contributors: Daniel Stanek, Joshua Whitley
