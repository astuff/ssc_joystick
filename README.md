# ROS SSC Joystick Application #

[![CircleCI](https://circleci.com/gh/astuff/ssc_joystick/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/ssc_joystick/tree/master)

# Overview
The joystick_vehicle_test module is used to verify that the SSC is operational.  It is similar to the ROS joystick node with some notable exceptions. 

This application is intended to convert the user's joystick commands into gear, steering, speed, and turn signal commands
to pass to the AutonomouStuff Speed and Steering Control software.  Use the launch file provided to start this module
and the joy node.  The SSC and drive-by-wire modules need to be started outside of this launch file.

Once the software is running, push either both engage buttons on the joystick or both the cruise control set/dec and
 decrease gap buttons on the steering wheel (not supported on all platforms) to take control with the joystick,
 ENGAGED will be output to the screen. The desired speed defaults to 0 mph when you engage, so the software will
 automatically engage the brakes.
 You can also choose to enable the speed and steering modules individually (not supported on all platforms).

You can then press the drive button to place the gear in drive, since the desired speed is still zero the brakes
will still be applied.  Pressing the speed up and down buttons will increment and decrement by the configured step
amount, limiting the speed between 0 and the maximum speed set.  Any time a speed button is pressed the new desired
speed will be output to the screen.  To bring the vehicle to a stop, step the speed back down to zero, and the control
software will gently apply the brakes to bring the vehicle to a stop.  Pressing the brake trigger will cause the
desired velocity to drop to zero, effectively applying the brakes. (The brake pedal can always be applied by the driver
 to override the joystick control mode.)

The steering gain and exponent convert the steering joystick to a desired curvature which is passed down to the
steering model.  The gain defines the maximum curvature, so the default of 0.12 1/meters allows for a minimum turning
radius of about 8 meters.  The exponent controls the shape of the response: a number closer to 2 or above will mean
small joystick movements will translate to very small desired curvatures and therefore steering wheel angles,
a number closer to 1 will mean the curvature varies more linearly across the full joystick range.

The curvature command can also be changed by pressing the left and right steering buttons.  The updated curvature
will be output to the screen.  The steering joystick will override the value set with the buttons.

The left and right turn signals can also be controlled with the buttons.  The turn signals will stay on as long
as the button is pressed.

Pressing the disengage button on the joystick will give control back to the driver.
On supported vehicles, pressing both the cruise control set/inc and increase gap buttons on the steering wheel will
result in a disengage.
On vehicles with the default override behavior, any drive override on the brakes, throttle, or steering wheel will also
result in returning control to the driver.
DISENGAGE or a message with information for an override will be sent to the screen.

It is also intended that this application be used as an example of how to interface to the speed and steering control
 software modules and can be used as a starting point for the development of higher level autonomy features.

# Joystick Button Layout
![Left: Front Layout of logitech Controller; Right: Side-button layout of logitech controller
](/controller_img.png "controller_img.png")

# Configuration Parameters (JSON)

| Parameter  | Default Value  | Description |
|:---|:---|:---|
|publish_interval           |     0.05              |The publish interval of the steering and speed commands, in seconds.
|joy_fault_timeout           |     3.0              |The timeout before a lack of joystick messages will trigger a fault, in seconds.
|vel_controller_name           |"/as/veh_controller"              |The namespace and name of the vehicle controller ROS node (from the launch file) used to detect overrides and faults in the dbw system
|vehicle_platform           |"Lexus"              |The vehicle platform name relevant to Speed and Steering Control configuration (check /ssc/vehicle_platform message, if not available, use default value)
|engage_speed_module           |true              |Engage speed control module, steering will be manual unless enabled using engage_only_steering
|engage_steering_module           |true              |Engage steering control module, speed will be manual unless enabled using engage_only_speed
|engage1_button           |6 (Upper center button left)              |The joystick button used to engage the joystick controller
|engage2_button           |7 (Upper center button right)              |The joystick button used to disengage the joystick controller
|park_button           |3 (Y button)              |The joystick button used to change the gear to park
|neutral_button           |2 (X button)              |The joystick button used to change the gear to neutral
|reverse_button           |1 (B button)              |
|drive_button           |0 (A button)              |The joystick button used to change the gear to drive
|right_turn_button           |5 (Right bumper)              |The joystick button used to command the right turn signal on
|left_turn_button           |4( (Left bumper)              |The joystick button used to command the left turn signal on
|speed_axes           |7 (Directional pad UP/DOWN)              |The joystick axes use to provide speed up and slow down commands
|speed_up_sign           |1              |The sign of the speed axes to control which is speed up and which is slow down
|speed_step           |1.0              |How much the speed should increase or decrease with each button press, in mph.
|speed_max           |15.0              |The maximum speed that can be commanded, in mph.
|acceleration_limit           |2.0              |The acceleration limit passed to the speed module, in m/s2
|deceleration_limit           |2.5              |The deceleration limit passed to the speed module, in m/s2
|brake_axes           |2              |
|brake_sign           |1              |
|max_decceleration_limit           |4.0              |The maximum deceleration limit in m/s2
|steer_btn_axes           |6 (Directional pad LEFT/RIGHT)             |The joystick axes used to step the curvature to the left or right
|steer_btn_sign           |1              |Determines which button steps positive (to the left) and negative (to the right)
|steer_btn_step           |0.01              |How much to increment or decrement the curvature with each button press
|steering_axes           |3              |The joystick axes use to control the desired curvature
|steering_sign           |1.0              |The sign of the joystick to control left and right
|steering_gain           |0.12              |The gain of the steering, since the joystick is generally -1.0 to 1.0, this is essentially the maximum curvature, in 1/meter
|steering_exponent           |2.5              |The exponent to control the shape/modulation of the steering command, needs to be >= 1
|max_curvature_rate           |0.1              |The maximum curvature rate passed to the steering module, in 1/meter/msec
|test_quick_brake           |false              |Added for testing quick brake condition, quick_brake_speed needs to be > 0 and related mode parameters in SSC's configuration need to be non-zero
|quick_brake_speed           |0.0              |The speed limit beyond which zero speed is requested with a zero decel

# Vehicle Specific Configuration

Hexagon Tractor (refer to TractorControlMode definition in msg/ folder)
| Parameter  | Default Value  | Description |
|:---|:---|:---|
|joy_engage           |false              |Engage joystick level from user.
|rpm_dial_engage           |false              |Engage rpm dial from user.
|hydraulics_engage           |false              |Engage hydraulics.
|joy_sens           |0              |Joystick sensitivity level, usage requires "joy engage" to be set to true.
|rpm_dial_val           |0.0              |RPM command, in percent (0-1), usage requires "rpm dial engage" to be set to true.
|hyd_in           |0.0              |Hydraulics system command, usage requires "hydraulics engage" to be set to true.
|hyd_in_id           |0              |Hydraulics system id, usage requires "hydraulics engage" to be set to true.
|beacon_in           |false              |Turn beacon lights on/off.
|horn_in           |false              |Turn horn on/off.
