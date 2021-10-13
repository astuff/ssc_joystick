# ROS SSC Joystick Application #

[![CircleCI](https://circleci.com/gh/astuff/ssc_joystick/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/ssc_joystick/tree/master)

## Overview

The ssc_joystick ROS package is used to verify that the SSC is operational. 
It is similar to the ROS joystick node with some notable exceptions. 

This application is intended to convert the user's joystick commands into gear, steering, speed, and turn signal commands
to pass to the AutonomouStuff Speed and Steering Control software.  
Use the launch file provided to start this module and the joy node.  
The SSC and drive-by-wire modules need to be started outside of this launch file.

## Installation

The `ssc_joystick` package can be installed using our AutonomouStuff apt repo:

```sh
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-ssc-joystick
```

## Control Scheme

The `ssc_joystick` package is intended to be used with the Logitech F310 gamepad controller as described below.
However, if you wish to use a different controller, the ROS parameters specified in the `config/params.yaml` file can be modified to enable any other controller or control scheme.

![Left: Front Layout of logitech Controller; Right: Side-button layout of logitech controller
](/controller_img.png "controller_img.png")

| Action | Button | Notes |
| - | - | - |
| **Enable/Disable** | **Center region** | |
| Enable | BACK and START | Buttons must be pressed simultaneously to engage |
| Disable | BACK | button must be pressed to disengage |
| **Gear Selection** | **Button Pad (right-hand side)** | |
| Drive | A | |
| Reverse | B | |
| Neutral | X | |
| Park | Y | |
| **Speed and Steering** | | |
| Speed setpoint adjust | Directional Pad Up/Down | Adjust speed in steps |
| Steering setpoint adjust | Directional Pad Left/Right | Adjust steering in steps |
| Brake Override | Left Trigger | Applies brakes immediately |
| Steering Override | Right Stick | Steers immediately |
| **Other** | | |
| Left turn signal | Left Bumper | Activate the left turn signal |
| Right turn signal | Right Bumper | Activate the right turn signal |

## Functional Overview

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
