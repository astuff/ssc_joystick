^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joystick_vehicle_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Contributors: AStuff software team, Daniel Stanek, Daniel-Stanek, Joshua Whitley, Lucas Buckland, Sam Rustan, Zach Oakes

1.0.0 (2017-05-02)
------------------
* Adding install rule for json file and removing Fusion-specific includes in launch.
* Adding install rules.
* Initial version
* Contributors: Daniel Stanek, Joshua Whitley
