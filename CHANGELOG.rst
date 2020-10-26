^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joystick_vehicle_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

devel-dbc-extra-3.0.2 (2020-10-26)
----------------------------------
* Added Hexagon Tractor.
* README update - individual module enable support unavailble on all platforms
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
