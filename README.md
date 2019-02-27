# ROS SSC Joystick Application #

[![CircleCI](https://circleci.com/gh/astuff/joystick_vehicle_test/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/joystick_vehicle_test/tree/master)

An application for converting user's joystick commands into gear, steering, speed, and turn signal commands
to pass to the AutonomouStuff Speed and Steering Control software.  Use the launch file provided to start this module
and the joy node.  The SSC and drive-by-wire modules need to be started outside of this launch file.

Once the software is running, push either both engage buttons on the joystick or both the cruise control set/dec and
 decrease gap buttons on the steering wheel (not supported on all platforms) to take control with the joystick,
 ENGAGED will be output to the screen. The desired speed defaults to 0 mph when you engage, so the software will
 automatically engage the brakes.

You can then press the drive button to place the gear in drive, since the desired speed is still zero the brakes
will still be applied.  Pressing the speed up and down buttons will increment and decrement by the configured step
amount, limiting the speed between 0 and the maximum speed set.  Any time a speed button is pressed the new desired
speed will be output to the screen.  To bring the vehicle to a stop, step the speed back down to zero, and the control
software will gently apply the brakes to bring the vehicle to a stop.  Pressing the brake trigger will cause the
desired velocity to drop to zero, effectively applying the brakes. (The brake pedal can always be applied by the driver
 to override the joystick control mode.)

The steering gain and exponent convert the steering joystick to a desired curvature which is passed down to the
steering model.  The gain defines the maximum curvature, so the default of 0.12 1/meters allows for a minimum turning
radius of about 8 meters.  The exponent controls the shape of the reponse: a number closer to 2 or above will mean
small joystick movements will translate to very small desired curvatures and therefore steering wheel angles,
a number closer to 1 will mean the curvature varies more linearly across the full joystick range. 

The curvature command can also be changed by pressing the left and right steering buttons.  The updated curvature
will be output to the screen.  The steering joystick will override the value set with the buttons.

The left and right turn signals can also be controlled with the buttons.  The turn signals will stay on as long
as the button is pressed.

Pressing either the disengage button on they joystick or both the cruise control set/inc and increase gap buttons on
the steering wheel will give control back to the driver (as will any drive override on the brakes, throttle, or
 steering wheel).  DISENGAGE will be sent to the screen, or a message with information if there was an override.

It is also intended that this application be used as an example of how to interface to the speed and steering control
 software modules and can be used as a starting point for the development of higher level autonomy features.

# JSON Parameters #
    - publish_interval: The publish interval of the steering and apeed commands, in seconds.
    - joy_fault_timeout: The timeout before a lack of joystick messages will trigger a fault, in seconds.
    - vel_controller_name: The namespace and name of the vehicle controller ROS node (from the launch file)
          used to detect overrides and faults in the dbw system

    - engage_button: The joystick button used to engage the joystick controller
    - disengage_button: The joystick button used to disengage the joystick controller

    - park_button: The joystick button used to change the gear to park
    - neutral_button: The joystick button used to change the gear to neutral
    - drive_button: The joystick button used to change the gear to drive

    - right_turn_button: The joystick button used to command the right turn signal on
    - left_turn_button: The joystick button used to command the left turn signal on

    - speed_axes: The joystick axes use to provide speed up and slow down commands
    - speed_up_sign: The sign of the speed axes to control which is speed up and which is slow down
    - speed_step: How much the speed should increase or decrease with each button press, in mph.
    - speed_max: The maximum speed that can be commanded, in mph.
    - acceleration_limit: The acceleration limit passed to the speed module, in m/sec
    - deceleration_limit: The deceleration limit passed to the speed module, in m/sec

    - steer_btn_axes: The joystick axes used to step the curvature to the left or right
    - steer_btn_sign: Determines which button steps positive (to the left) and negative (to the right)
    - steer_btn_step: How much to increment or decrement the curvature with each button press
    - steering_axes: The joystick axes use to control the desired curvature
    - steering_sign: The sign of the joystick to control left and right
    - steering_gain: The gain of the steering, since the joystick is generally -1.0 to 1.0, this is essentially the
          maximum curvature, in 1/meter
    - steering_exponent: The exponent to control the shape/modulation of the steering command, needs to be >= 1
    - max_curvature_rate: The maximum curvature rate passed to the steering module, in 1/meter/msec
