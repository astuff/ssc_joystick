# Copyright (c) 2020 AutonomouStuff, LLC
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import launch.actions
import launch.substitutions
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    this_launch_file_dir = launch.substitutions.ThisLaunchFileDir()

    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            node_executable='joy_node',
            output='screen',
            node_namespace='ssc',
        ),
        launch_ros.actions.Node(
            package='ssc_joystick',
            node_executable='ssc_joystick',
            output='screen',
            node_namespace='ssc',
            arguments=['-f', [this_launch_file_dir, '/ssc_joystick.json']],
        )
    ])